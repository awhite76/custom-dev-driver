# custom-dev-driver
Custom device driver for Raspberry Pi powered handheld device. This software drives two analog thumb sticks and eight buttons in a matrix configuration. 

# build and load
make
sudo insmod user_controller.ko
// To check
dmesg -T

# unload
sudo rmmod user_controller
