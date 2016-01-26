BINARY = sat_firmware

OPENCM3_DIR=../libopencm3
LDSCRIPT = stm32f407vet6.ld

OBJS += usart.o
OBJS += adc.o
OBJS += i2c.o
OBJS += bmp180.o
OBJS += eeprom.o
OBJS += main.o
OBJS += utils.o
OBJS += proto/telemetry.pb.o
OBJS += ../nanopb/pb_encode.o
OBJS += ../nanopb/pb_common.o

PROTOBUFFERS += telemetry.pb

include libopencm3.target.mk
