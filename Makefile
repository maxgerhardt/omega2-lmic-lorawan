# C compiler
CC:= mipsel-openwrt-linux-g++
# path where the toolchain is
TOOLCHAIN_ROOT_DIR:= /home/max/source/staging_dir/target-mipsel_24kc_musl-1.1.16
#path where the omega_includes and omega_libs folder are
OMEGA_DIR:= /home/max/omega

# additional includes from toolchain
INCLUDE_DIRS:=$(TOOLCHAIN_ROOT_DIR)/usr/include
LIB_DIRS:=$(TOOLCHAIN_ROOT_DIR)/usr/lib

#links to link against
LDFLAGS_LIB:= -loniondebug -lonionspi -lonioni2c -lugpio
LDFLAGS_PROGRAM = 
CFLAGS:= -O3 -ggdb -g -Wall -Wextra -std=c++14
IFLAGS:= -I $(INCLUDE_DIRS) -I $(OMEGA_DIR)/omega_includes -I arduino-lmic/src/ -I.

EXAMPLE_SOURCE = lorawan_send
PROGRAM_SOURCES = $(EXAMPLE_SOURCE).cpp  SC18IS602B.cpp
EXECUTABLE:= $(EXAMPLE_SOURCE)

LMIC_SOURCES = \
	arduino-lmic/src/aes/other.c \
	arduino-lmic/src/aes/ideetron/AES-128_V10.cpp \
	arduino-lmic/src/aes/lmic.c \
	arduino-lmic/src/hal/hal.cpp \
	arduino-lmic/src/lmic/lmic.c \
	arduino-lmic/src/lmic/oslmic.c \
	arduino-lmic/src/lmic/radio.c \

export STAGING_DIR="$TOOLCHAIN_ROOT_DIR/staging_dir/"

.PHONY : program all clean all

program:
	$(CC) -o $(EXECUTABLE) $(CFLAGS) $(IFLAGS) -L $(LIB_DIRS) -L $(OMEGA_DIR)/omega_libs -L. $(PROGRAM_SOURCES) $(LMIC_SOURCES) $(LDFLAGS_PROGRAM) $(LDFLAGS_LIB)

all: | program

upload: | all
	sshpass -p "onioneer" scp $(EXECUTABLE) root@192.168.1.150:/root/.

clean:
	rm -rf $(EXECUTABLE)
#	rm -rf $(LIB_NAME)
