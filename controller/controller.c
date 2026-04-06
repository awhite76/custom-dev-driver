// SPDX-License-Identifier: MIT
// controller.c – Thumbsticks + button matrix exposed as a virtual joystick

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <unistd.h>
#include <fcntl.h>
#include <linux/uinput.h>
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include <gpiod.h>
#include <string.h>
#include <math.h>

#define ADS1015_ADDR 0x48
#define REG_CONVERT 0x00
#define REG_CONFIG 0x01

// Configuration bits for ADS1015
#define CONFIG_OS_SINGLE (0x8000)
#define CONFIG_MUX_OFFSET (12)
#define CONFIG_PGA_4_096V (0x0200)
#define CONFIG_MODE_SINGLE (0x0100)
#define CONFIG_DR_1600SPS (0x0080)
#define CONFIG_COMP_QUE_DISABLE (0x0003)

// GPIO pin numbers
#define CHIPNAME "/dev/gpiochip0"
#define GPIO22 22
#define GPIO27 27
#define Uin 24
#define Rin 25
#define Din 5
#define Lin 6
#define Lout 26
#define Rout 23

// -----------------------------------------------------------------------------
// uinput helper
// -----------------------------------------------------------------------------
static int ufd;
static struct uinput_user_dev udev;
static struct input_event ev;

static void emit(int type, int code, int val)
{
    memset(&ev, 0, sizeof(ev));
    ev.type = type;
    ev.code = code;
    ev.value = val;
    write(ufd, &ev, sizeof(ev));
}

static void flush_events(void) { emit(EV_SYN, SYN_REPORT, 0); }

static int init_uinput(void)
{
    ufd = open("/dev/uinput", O_WRONLY | O_NONBLOCK);
    if (ufd < 0)
    {
        perror("uinput");
        return -1;
    }

    ioctl(ufd, UI_SET_EVBIT, EV_KEY);
    ioctl(ufd, UI_SET_KEYBIT, BTN_A);
    ioctl(ufd, UI_SET_KEYBIT, BTN_B);
    ioctl(ufd, UI_SET_KEYBIT, BTN_X);
    ioctl(ufd, UI_SET_KEYBIT, BTN_Y);
    ioctl(ufd, UI_SET_KEYBIT, BTN_TL);
    ioctl(ufd, UI_SET_KEYBIT, BTN_TR);

    ioctl(ufd, UI_SET_EVBIT, EV_ABS);
    ioctl(ufd, UI_SET_ABSBIT, ABS_X);
    ioctl(ufd, UI_SET_ABSBIT, ABS_Y);
    ioctl(ufd, UI_SET_ABSBIT, ABS_RX);
    ioctl(ufd, UI_SET_ABSBIT, ABS_RY);
    ioctl(ufd, UI_SET_KEYBIT, BTN_START);
    ioctl(ufd, UI_SET_KEYBIT, BTN_SELECT);

    memset(&udev, 0, sizeof(udev));
    snprintf(udev.name, UINPUT_MAX_NAME_SIZE, "Custom Thumbstick Controller");
    udev.id.bustype = BUS_USB;
    udev.id.vendor = 0x1234;
    udev.id.product = 0x5678;

    udev.absmin[ABS_X] = -32768;
    udev.absmax[ABS_X] = 32767;
    udev.absmin[ABS_Y] = -32768;
    udev.absmax[ABS_Y] = 32767;
    udev.absmin[ABS_RX] = -32768;
    udev.absmax[ABS_RX] = 32767;
    udev.absmin[ABS_RY] = -32768;
    udev.absmax[ABS_RY] = 32767;

    write(ufd, &udev, sizeof(udev));
    ioctl(ufd, UI_DEV_CREATE);
    return 0;
}

static void close_uinput(void)
{
    ioctl(ufd, UI_DEV_DESTROY);
    close(ufd);
}

// -----------------------------------------------------------------------------
// ADS1015 read
// -----------------------------------------------------------------------------
static int16_t read_channel(int fd, int channel)
{
    uint16_t config = CONFIG_OS_SINGLE |
                      ((0x04 + channel) << CONFIG_MUX_OFFSET) |
                      CONFIG_PGA_4_096V |
                      CONFIG_MODE_SINGLE |
                      CONFIG_DR_1600SPS |
                      CONFIG_COMP_QUE_DISABLE;

    uint8_t w[3] = {REG_CONFIG, config >> 8, config & 0xFF};
    if (write(fd, w, 3) != 3)
        return -1;
    usleep(2000);
    uint8_t reg = REG_CONVERT;
    write(fd, &reg, 1);
    uint8_t r[2];
    if (read(fd, r, 2) != 2)
        return -1;
    int16_t v = ((r[0] << 8) | r[1]) >> 4;
    if (v & 0x800)
        v -= 1 << 12;
    return v;
}

// -----------------------------------------------------------------------------
// Button matrix
// -----------------------------------------------------------------------------
static const char *scan_button_matrix(struct gpiod_chip *chip)
{
    static const char *buttons[2][4] = {
        {"Up", "Right", "Down", "Left"},
        {"Y", "B", "A", "X"}};

    int outputs[2] = {Lout, Rout};
    int inputs[4] = {Uin, Rin, Din, Lin};

    struct gpiod_line *r[2];
    struct gpiod_line *c[4];

    for (int i = 0; i < 2; i++)
    {
        r[i] = gpiod_chip_get_line(chip, outputs[i]);
        gpiod_line_request_output(r[i], "row", 1); // idle high
    }
    for (int i = 0; i < 4; i++)
    {
        c[i] = gpiod_chip_get_line(chip, inputs[i]);
        gpiod_line_request_input_flags(c[i], "col",
                                       GPIOD_LINE_REQUEST_FLAG_BIAS_PULL_UP);
    }

    const char *pressed = "None";

    for (int row = 0; row < 2; row++)
    {
        for (int j = 0; j < 2; j++)
            gpiod_line_set_value(r[j], 1);
        gpiod_line_set_value(r[row], 0);
        usleep(1000);

        for (int col = 0; col < 4; col++)
        {
            int v = gpiod_line_get_value(c[col]);
            if (v == 0)
            {
                pressed = buttons[row][col];
                goto done;
            }
        }
    }
done:
    for (int i = 0; i < 2; i++)
        gpiod_line_set_value(r[i], 1);
    for (int i = 0; i < 2; i++)
        gpiod_line_release(r[i]);
    for (int i = 0; i < 4; i++)
        gpiod_line_release(c[i]);
    return pressed;
}

// -----------------------------------------------------------------------------
// Main
// -----------------------------------------------------------------------------
int main(void)
{
    const char *i2cdev = "/dev/i2c-1";
    int fd = open(i2cdev, O_RDWR);
    if (fd < 0)
    {
        perror("I2C open");
        return 1;
    }
    if (ioctl(fd, I2C_SLAVE, ADS1015_ADDR) < 0)
    {
        perror("I2C ioctl");
        close(fd);
        return 1;
    }

    struct gpiod_chip *chip = gpiod_chip_open_by_name("gpiochip0");
    if (!chip)
    {
        perror("gpiochip");
        close(fd);
        return 1;
    }

    struct gpiod_line *line22 = gpiod_chip_get_line(chip, GPIO22);
    struct gpiod_line *line27 = gpiod_chip_get_line(chip, GPIO27);
    gpiod_line_request_input(line22, "thumb_L");
    gpiod_line_request_input(line27, "thumb_R");

    if (init_uinput() < 0)
        return 1;
    printf("Virtual joystick active\n");

    while (1)
    {
        float n[4];
        for (int ch = 0; ch < 4; ch++)
        {
            int16_t v = read_channel(fd, ch);
            float volts = v * 4.096 / 2048.0;
            n[ch] = (volts - 1.6f) / 1.1f;
        }

        int16_t ax = (int)(-n[3] * 32767);
        int16_t ay = (int)(n[2] * 32767);
        int16_t rx = (int)(-n[1] * 32767);
        int16_t ry = (int)(n[0] * 32767);

        emit(EV_ABS, ABS_X, ax);
        emit(EV_ABS, ABS_Y, ay);
        emit(EV_ABS, ABS_RX, rx);
        emit(EV_ABS, ABS_RY, ry);

        const char *btn = scan_button_matrix(chip);
        emit(EV_KEY, BTN_A, strcmp(btn, "A") == 0);
        emit(EV_KEY, BTN_B, strcmp(btn, "B") == 0);
        emit(EV_KEY, BTN_X, strcmp(btn, "X") == 0);
        emit(EV_KEY, BTN_Y, strcmp(btn, "Y") == 0);
        emit(EV_KEY, BTN_TL, strcmp(btn, "Left") == 0);
        emit(EV_KEY, BTN_TR, strcmp(btn, "Right") == 0);
        emit(EV_KEY, BTN_START, strcmp(btn, "Up") == 0);
        emit(EV_KEY, BTN_SELECT, strcmp(btn, "Down") == 0);

        flush_events();

        usleep(50000);
    }

    close_uinput();
    gpiod_line_release(line22);
    gpiod_line_release(line27);
    gpiod_chip_close(chip);
    close(fd);
    return 0;
}
