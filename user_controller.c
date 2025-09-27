#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/gpio.h>
#include <linux/interrupt.h>
#include <linux/input.h>

static int __init user_controller_init(void) {
        pr_info("Custom user controller driver loaded!");

        return 0;
}

static void __exit user_controller_exit(void) {
        pr_info("Custom user controller driver unloaded!");

}

module_init(user_controller_init);
module_exit(user_controller_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("PLAT");
MODULE_DESCRIPTION("Custom driver for thumbstick and button matrix with GPIO + input events");
