AUTOMAKE_OPTIONS = subdir-objects
ACLOCAL_AMFLAGS = ${ACLOCAL_FLAGS} -I m4

if P44_BUILD_WIN

bin_PROGRAMS = p44mbutil

else

bin_PROGRAMS = kksdcmd p44mbutil

endif

# embedded libmodbus
if EMBEDDED_LIBMODBUS

LIBMODBUS_SOURCES = \
  src/p44utils/thirdparty/libmodbus/src/modbus-data.c \
  src/p44utils/thirdparty/libmodbus/src/modbus-private.h \
  src/p44utils/thirdparty/libmodbus/src/modbus-rtu-private.h \
  src/p44utils/thirdparty/libmodbus/src/modbus-rtu.c \
  src/p44utils/thirdparty/libmodbus/src/modbus-rtu.h \
  src/p44utils/thirdparty/libmodbus/src/modbus-tcp-private.h \
  src/p44utils/thirdparty/libmodbus/src/modbus-tcp.c \
  src/p44utils/thirdparty/libmodbus/src/modbus-tcp.h \
  src/p44utils/thirdparty/libmodbus/src/modbus-version.h \
  src/p44utils/thirdparty/libmodbus/src/modbus.c \
  src/p44utils/thirdparty/libmodbus/src/modbus.h

LIBMODBUS_CFLAGS = -I ${srcdir}/src/p44utils/thirdparty/libmodbus/src -D EMBEDDED_LIBMODBUS=1

else

LIBMODBUS_CFLAGS = ${MODBUS_CFLAGS}
LIBMODBUS_LIBS = ${MODBUS_LIBS}

endif


# ubus support via libubus
if ENABLE_UBUS

UBUS_SOURCES = \
  src/p44utils/ubus.cpp \
  src/p44utils/ubus.hpp
UBUS_FLAGS = -D ENABLE_UBUS=1

else

UBUS_FLAGS = -D ENABLE_UBUS=0

endif

# kksdcmd

if !P44_BUILD_WIN

if DEBUG
kksdcmd_DEBUG = -D DEBUG=1
endif

if P44_BUILD_OW
kksdcmd_PLATFORM = -D P44_BUILD_OW=1 -std=gnu++17
endif


# main

kksdcmd_LDADD = \
	${I2C_LIBS} \
	${JSONC_LIBS} \
	${LIBMODBUS_LIBS} \
	${PTHREAD_CFLAGS} \
	${PTHREAD_LIBS} \
  ${UBUS_LIBS} \
	${SQLITE3_LIBS}

kksdcmd_EXTRACFLAGS = -D NO_SSL_DL=1


kksdcmd_CPPFLAGS = \
  -I ${srcdir}/src/p44utils \
  -I ${srcdir}/src \
  -I ${srcdir}/src/p44utils/thirdparty/civetweb \
  ${BOOST_CPPFLAGS} \
  ${I2C_FLAGS} \
  ${JSONC_CFLAGS} \
  ${LIBMODBUS_CFLAGS} \
  ${PTHREAD_CFLAGS} \
  ${UBUS_FLAGS} \
  ${kksdcmd_EXTRACFLAGS} \
  ${kksdcmd_PLATFORM} \
  ${kksdcmd_DEBUG}



kksdcmd_SOURCES = \
  ${UBUS_SOURCES} \
  ${LIBMODBUS_SOURCES} \
  src/p44utils/analogio.cpp \
  src/p44utils/analogio.hpp \
  src/p44utils/application.cpp \
  src/p44utils/application.hpp \
  src/p44utils/consolekey.cpp \
  src/p44utils/consolekey.hpp \
  src/p44utils/crc32.cpp \
  src/p44utils/crc32.hpp \
  src/p44utils/digitalio.cpp \
  src/p44utils/digitalio.hpp \
  src/p44utils/error.cpp \
  src/p44utils/error.hpp \
  src/p44utils/p44script.cpp \
  src/p44utils/p44script.hpp \
  src/p44utils/fdcomm.cpp \
  src/p44utils/fdcomm.hpp \
  src/p44utils/fnv.cpp \
  src/p44utils/fnv.hpp \
  src/p44utils/gpio.cpp \
  src/p44utils/gpio.h \
  src/p44utils/gpio.hpp \
  src/p44utils/httpcomm.cpp\
  src/p44utils/httpcomm.hpp\
  src/p44utils/i2c.cpp \
  src/p44utils/i2c.hpp \
  src/p44utils/iopin.cpp \
  src/p44utils/iopin.hpp \
  src/p44utils/jsonobject.cpp \
  src/p44utils/jsonobject.hpp \
  src/p44utils/jsonwebclient.cpp\
  src/p44utils/jsonwebclient.hpp\
  src/p44utils/logger.cpp \
  src/p44utils/logger.hpp \
  src/p44utils/macaddress.cpp \
  src/p44utils/macaddress.hpp \
  src/p44utils/mainloop.cpp \
  src/p44utils/mainloop.hpp \
  src/p44utils/modbus.cpp \
  src/p44utils/modbus.hpp \
  src/p44utils/p44obj.cpp \
  src/p44utils/p44obj.hpp \
  src/p44utils/pwm.cpp \
  src/p44utils/pwm.hpp \
  src/p44utils/serialcomm.cpp \
  src/p44utils/serialcomm.hpp \
  src/p44utils/socketcomm.cpp \
  src/p44utils/socketcomm.hpp \
  src/p44utils/spi.cpp \
  src/p44utils/spi.hpp \
  src/p44utils/timeutils.cpp \
  src/p44utils/timeutils.hpp \
  src/p44utils/utils.cpp \
  src/p44utils/utils.hpp \
  src/p44utils/valueanimator.cpp \
  src/p44utils/valueanimator.hpp \
  src/p44utils/valueunits.cpp \
  src/p44utils/valueunits.hpp \
  src/p44utils/extutils.cpp \
  src/p44utils/extutils.hpp \
  src/p44utils/thirdparty/civetweb/civetweb.c \
  src/p44utils/thirdparty/civetweb/civetweb.h \
  src/p44utils/thirdparty/civetweb/handle_form.inl \
  src/p44utils/thirdparty/civetweb/md5.inl \
  src/p44utils/thirdparty/civetweb/sha1.inl \
  src/p44utils/thirdparty/civetweb/hostcheck.inl \
  src/p44utils/thirdparty/civetweb/openssl_hostname_validation.inl \
  src/p44utils/p44utils_common.hpp \
  src/p44utils_config.hpp \
  src/corespiproto.cpp \
  src/corespiproto.hpp \
  src/coreregmodel.cpp \
  src/coreregmodel.hpp \
  src/kksdcmd_main.cpp

endif

# p44mbutil

if DEBUG
p44mbutil_DEBUG = -D DEBUG=1
endif

if P44_BUILD_OW
p44mbutil_PLATFORM = -D P44_BUILD_OW=1
endif

if P44_BUILD_WIN
p44mbutil_PLATFORM = -D P44_BUILD_WIN=1 -static -static-libgcc -static-libstdc++
p44mbutil_LDFLAGS = -static -static-libgcc -static-libstdc++
endif


# main

p44mbutil_LDADD = ${LIBMODBUS_LIBS} ${PTHREAD_CFLAGS} ${PTHREAD_LIBS}
p44mbutil_EXTRACFLAGS = -D NO_SSL_DL=1 -D ENABLE_P44SCRIPT=0 -D ENABLE_JSON_APPLICATION=0


p44mbutil_CPPFLAGS = \
  -I ${srcdir}/src/p44utils \
  -I ${srcdir}/src \
  ${BOOST_CPPFLAGS} \
  ${LIBMODBUS_CFLAGS} \
  ${PTHREAD_CFLAGS} \
  ${p44mbutil_EXTRACFLAGS} \
  ${p44mbutil_PLATFORM} \
  ${p44mbutil_DEBUG}



p44mbutil_SOURCES = \
  ${LIBMODBUS_SOURCES} \
  src/p44utils/application.cpp \
  src/p44utils/application.hpp \
  src/p44utils/consolekey.cpp \
  src/p44utils/consolekey.hpp \
  src/p44utils/digitalio.cpp \
  src/p44utils/digitalio.hpp \
  src/p44utils/i2c.cpp \
  src/p44utils/i2c.hpp \
  src/p44utils/spi.cpp \
  src/p44utils/spi.hpp \
  src/p44utils/pwm.cpp \
  src/p44utils/pwm.hpp \
  src/p44utils/analogio.cpp \
  src/p44utils/analogio.hpp \
  src/p44utils/error.cpp \
  src/p44utils/error.hpp \
  src/p44utils/fnv.cpp \
  src/p44utils/fnv.hpp \
  src/p44utils/crc32.cpp \
  src/p44utils/crc32.hpp \
  src/p44utils/fdcomm.cpp \
  src/p44utils/fdcomm.hpp \
  src/p44utils/gpio.cpp \
  src/p44utils/gpio.h \
  src/p44utils/gpio.hpp \
  src/p44utils/iopin.cpp \
  src/p44utils/iopin.hpp \
  src/p44utils/logger.cpp \
  src/p44utils/logger.hpp \
  src/p44utils/macaddress.cpp \
  src/p44utils/macaddress.hpp \
  src/p44utils/mainloop.cpp \
  src/p44utils/mainloop.hpp \
  src/p44utils/p44obj.cpp \
  src/p44utils/p44obj.hpp \
  src/p44utils/serialcomm.cpp \
  src/p44utils/serialcomm.hpp \
  src/p44utils/modbus.cpp \
  src/p44utils/modbus.hpp \
  src/p44utils/utils.cpp \
  src/p44utils/utils.hpp \
  src/p44utils/valueanimator.cpp \
  src/p44utils/valueanimator.hpp \
  src/p44utils/extutils.cpp \
  src/p44utils/extutils.hpp \
  src/p44utils/p44utils_common.hpp \
  src/p44utils_config.hpp \
  src/p44mbutil_main.cpp
