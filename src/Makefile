BINARY = sat_firmware

OPENCM3_DIR=../libopencm3
LDSCRIPT = stm32f407vet6.ld

$(shell cp chan_fat/src/ff.c .)

OBJS += usart.o
OBJS += adc.o
OBJS += i2c.o
OBJS += bmp180.o
OBJS += eeprom.o
OBJS += main.o
OBJS += utils.o
OBJS += onewire.o
OBJS += ds18b20.o
OBJS += smbus.o
OBJS += mpu6050.o
OBJS += hmc5883l.o
OBJS += bh1750.o
OBJS += spi.o
OBJS += sdcard.o
OBJS += log.o
OBJS += madgwick_ahrs.o
OBJS += proto/telemetry.pb.o
OBJS += ../nanopb/pb_encode.o
OBJS += ../nanopb/pb_common.o

#OBJS += ff.o

PROTOBUFFERS += telemetry.pb

include libopencm3.target.mk
