include ../nanopb/extra/nanopb.mk

BINARY = sat_firmware

OPENCM3_DIR=../libopencm3
LDSCRIPT = stm32f407vet6.ld

OBJS += usart.o
OBJS += adc.o
OBJS += camera.o
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
OBJS += spi2.o
OBJS += sdcard.o
OBJS += sdcard2.o
OBJS += stuffer.o
OBJS += log.o
OBJS += radiation_sensor.o
OBJS += madgwick_ahrs.o
OBJS += mlx90614.o
OBJS += habduino.o
OBJS += proto/telemetry.pb.o
OBJS += ../nanopb/pb_encode.o
OBJS += ../nanopb/pb_common.o

include libopencm3.target.mk
