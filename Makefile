TARGET       = quadcopteremb
BOARD_TAG    = mega_pic32
ARDUINO_LIBS = Wire Wire/utility
USER_LIBS_SEL = I2C MPU6050 UAVLink/include
SKIP_SUFFIX_CHECK = 1
ARDUINO_PORT = /dev/ttyUSB1
AVRDUDE_TOOLS_PATH = /usr/bin
MPIDE_DIR = /home/alienx/Documents/Drone/MPIDE/mpide-0023-linux-20120903
include chipKIT.mk
