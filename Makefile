TARGET       = quadcopteremb
BOARD_TAG    = mega_pic32

ARDUINO_LIBS = Wire Wire/utility
USER_LIBS_SEL = I2C MPU6050 MS5611 ID_Sonar UAVLink/include ID_Math ID_Matrix ID_Kinematics

SKIP_SUFFIX_CHECK = 1

ARDUINO_PORT = /dev/ttyUSB1
AVRDUDE_TOOLS_PATH = /usr/bin
MPIDE_DIR = /home/alienx/Dropbox/Drone/MPIDE/mpide-0023-linux-20120903


include chipKIT.mk
