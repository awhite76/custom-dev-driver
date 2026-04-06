# custom-dev-driver
Custom device driver for Raspberry Pi powered handheld device. This software drives two analog thumb sticks and eight buttons in a matrix configuration. 

# build and load
cd controller

make

sudo ./controller

// To test

sudo jstest /dev/input/js0

