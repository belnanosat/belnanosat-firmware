/**
 * @author  Tilen Majerle
 * @email   tilen@majerle.eu
 * @website http://stm32f4-discovery.com
 * @link    http://stm32f4-discovery.com/2014/05/library-12-onewire-library-for-stm43f4xx/
 * @version v2.1
 * @ide     Keil uVision
 * @license GNU GPL v3
 * @brief   Onewire library for STM32F4 devices
 *
 @verbatim
 ----------------------------------------------------------------------
 Copyright (C) Tilen Majerle, 2015

 This program is free software: you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation, either version 3 of the License, or
 any later version.

 This program is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details.

 You should have received a copy of the GNU General Public License
 along with this program.  If not, see <http://www.gnu.org/licenses/>.
 ----------------------------------------------------------------------
 @endverbatim
*/
#ifndef ONEWIRE_H
#define ONEWIRE_H 210

/* C++ detection */
#ifdef __cplusplus
extern C {
#endif

/**
 * @addtogroup STM32F4xx_Libraries
 * @{
 */

/**
 * @defgroup ONEWIRE
 * @brief    Onewire library for STM32F4 devices - http://stm32f4-discovery.com/2014/05/library-12-onewire-library-for-stm43f4xx/
 * @{
 *
 * As of version 2.0 you can now use more than just one one-wire "port" on STM32F4. This allows you to group devices to separate ports.
 *
 * Because if you have a loot devices on one port, if one device fail, everything is failed. You can prevent this by use more than just one port.
 *
 * To set your port and pin for OneWire protocol, you can do this when calling @ref OneWire_Init function.
 *
 * \par Changelog
 *
 @verbatim
 Version 2.1
 - March 10, 2015
 - Added support for new GPIO library

 Version 2.0
 - January 04, 2015
 - New OneWire system
 - With support for multiple OneWire ports to separate group of devices

 Version 1.1
 - December 06, 2014
 - Added 8-bit CRC calculation for 1-Wire devices, algorithm from Dallas

 Version 1.0
 - First release
 @endverbatim
 *
 * \par Dependencies
 *
 @verbatim
 - STM32F4xx
 - STM32F4xx RCC
 - STM32F4xx GPIO
 - defines.h
 - TM DELAY
 - TM GPIO
 @endverbatim
*/

#include <libopencm3/stm32/gpio.h>

#include "utils.h"

/**
 * @defgroup ONEWIRE_Macros
 * @brief    Library defines
 * @{
 */

/* OneWire delay */
#define ONEWIRE_DELAY(x)				usleep(x)

/* Pin settings */
#define ONEWIRE_LOW(structure)			gpio_clear((structure)->gpio_port, (structure)->gpio_pin)
#define ONEWIRE_HIGH(structure)			gpio_set((structure)->gpio_port, (structure)->gpio_pin)
#define ONEWIRE_INPUT(structure)		gpio_mode_setup((structure)->gpio_port, GPIO_MODE_INPUT, GPIO_PUPD_PULLUP, (structure)->gpio_pin)
#define ONEWIRE_OUTPUT(structure)		gpio_mode_setup((structure)->gpio_port, GPIO_MODE_OUTPUT, GPIO_PUPD_PULLUP, (structure)->gpio_pin)
#define ONEWIRE_READ(structure)         gpio_get((structure)->gpio_port, (structure)->gpio_pin)

/* OneWire commands */
#define ONEWIRE_CMD_RSCRATCHPAD			0xBE
#define ONEWIRE_CMD_WSCRATCHPAD			0x4E
#define ONEWIRE_CMD_CPYSCRATCHPAD		0x48
#define ONEWIRE_CMD_RECEEPROM			0xB8
#define ONEWIRE_CMD_RPWRSUPPLY			0xB4
#define ONEWIRE_CMD_SEARCHROM			0xF0
#define ONEWIRE_CMD_READROM				0x33
#define ONEWIRE_CMD_MATCHROM			0x55
#define ONEWIRE_CMD_SKIPROM				0xCC

/**
 * @}
 */

/**
 * @defgroup ONEWIRE_Typedefs
 * @brief    Library Typedefs
 * @{
 */

/**
 * @brief  OneWire working struct
 * @note   Except ROM_NO member, everything is fully private and should not be touched by user
 */
	typedef struct {
		uint32_t gpio_port;           /*!< GPIOx port to be used for I/O functions */
		uint16_t gpio_pin;             /*!< GPIO Pin to be used for I/O functions */
		uint8_t LastDiscrepancy;       /*!< Search private */
		uint8_t LastFamilyDiscrepancy; /*!< Search private */
		uint8_t LastDeviceFlag;        /*!< Search private */
		uint8_t ROM_NO[8];             /*!< 8-bytes address of last search device */
	} OneWire_t;

/**
 * @}
 */

/**
 * @defgroup ONEWIRE_Functions
 * @brief    Library Functions
 * @{
 */

/**
 * @brief  Initializes OneWire bus
 * @param  *OneWireStruct: Pointer to @ref OneWire_t empty working onewire structure
 * @param  *Pointer to GPIO port used for onewire channel
 * @param  GPIO_Pin: GPIO Pin on specific GPIOx to be used for onewire channel
 * @retval None
 */
	void OneWire_Init(OneWire_t* OneWireStruct, uint32_t gpio_port, uint16_t gpio_pin);

/**
 * @brief  Resets OneWire bus
 *
 * @note   Sends reset command for OneWire
 * @param  *OneWireStruct: Pointer to @ref OneWire_t working onewire structure
 * @retval None
 */
	uint8_t OneWire_Reset(OneWire_t* OneWireStruct);

/**
 * @brief  Reads byte from one wire bus
 * @param  *OneWireStruct: Pointer to @ref OneWire_t working onewire structure
 * @retval Byte from read operation
 */
	uint8_t OneWire_ReadByte(OneWire_t* OneWireStruct);

/**
 * @brief  Writes byte to bus
 * @param  *OneWireStruct: Pointer to @ref OneWire_t working onewire structure
 * @param  byte: 8-bit value to write over OneWire protocol
 * @retval None
 */
	void OneWire_WriteByte(OneWire_t* OneWireStruct, uint8_t byte);

/**
 * @brief  Writes single bit to onewire bus
 * @param  *OneWireStruct: Pointer to @ref OneWire_t working onewire structure
 * @param  bit: Bit value to send, 1 or 0
 * @retval None
 */
	void OneWire_WriteBit(OneWire_t* OneWireStruct, uint8_t bit);

/**
 * @brief  Reads single bit from one wire bus
 * @param  *OneWireStruct: Pointer to @ref OneWire_t working onewire structure
 * @retval Bit value:
 *            - 0: Bit is low (zero)
 *            - > 0: Bit is high (one)
 */
	uint8_t OneWire_ReadBit(OneWire_t* OneWireStruct);

/**
 * @brief  Searches for OneWire devices on specific Onewire port
 * @note   Not meant for public use. Use @ref OneWire_First and @ref OneWire_Next for this.
 * @param  *OneWireStruct: Pointer to @ref OneWire_t working onewire structure where to search
 * @param  Device status:
 *            - 0: No devices detected
 *            - > 0: Device detected
 */
	uint8_t OneWire_Search(OneWire_t* OneWireStruct, uint8_t command);

/**
 * @brief  Resets search states
 * @param  *OneWireStruct: Pointer to @ref OneWire_t working onewire where to reset search values
 * @retval None
 */
	void OneWire_ResetSearch(OneWire_t* OneWireStruct);

/**
 * @brief  Starts search, reset states first
 * @note   When you want to search for ALL devices on one onewire port, you should first use this function.
 @verbatim
 /...Initialization before
 status = OneWire_First(&OneWireStruct);
 while (status) {
 //Save ROM number from device
 OneWire_GetFullROM(ROM_Array_Pointer);
 //Check for new device
 status = OneWire_Next(&OneWireStruct);
 }
 @endverbatim
 * @param  *OneWireStruct: Pointer to @ref OneWire_t working onewire where to reset search values
 * @param  Device status:
 *            - 0: No devices detected
 *            - > 0: Device detected
 */
	uint8_t OneWire_First(OneWire_t* OneWireStruct);

/**
 * @brief  Reads next device
 * @note   Use @ref OneWire_First to start searching
 * @param  *OneWireStruct: Pointer to @ref OneWire_t working onewire
 * @param  Device status:
 *            - 0: No devices detected any more
 *            - > 0: New device detected
 */
	uint8_t OneWire_Next(OneWire_t* OneWireStruct);

/**
 * @brief  Gets ROM number from device from search
 * @param  *OneWireStruct: Pointer to @ref OneWire_t working onewire
 * @param  index: Because each device has 8-bytes long ROm address, you have to call this 8 times, to get ROM bytes from 0 to 7
 * @reetval ROM byte for index (0 to 7) at current found device
 */
	uint8_t OneWire_GetROM(OneWire_t* OneWireStruct, uint8_t index);

/**
 * @brief  Gets all 8 bytes ROM value from device from search
 * @param  *OneWireStruct: Pointer to @ref OneWire_t working onewire
 * @param  *firstIndex: Pointer to first location for first byte, other bytes are automatically incremented
 * @retval None
 */
	void OneWire_GetFullROM(OneWire_t* OneWireStruct, uint8_t *firstIndex);

/**
 * @brief  Selects specific slave on bus
 * @param  *OneWireStruct: Pointer to @ref OneWire_t working onewire
 * @param  *addr: Pointer to first location of 8-bytes long ROM address
 * @retval None
 */
	void OneWire_Select(OneWire_t* OneWireStruct, uint8_t* addr);

/**
 * @brief  Selects specific slave on bus with pointer address
 * @param  *OneWireStruct: Pointer to @ref OneWire_t working onewire
 * @param  *ROM: Pointer to first byte of ROM address
 * @retval None
 */
	void OneWire_SelectWithPointer(OneWire_t* OneWireStruct, uint8_t* ROM);

/**
 * @brief  Calculates 8-bit CRC for 1-wire devices
 * @param  *addr: Pointer to 8-bit array of data to calculate CRC
 * @param  len: Number of bytes to check
 *
 * @retval Calculated CRC from input data
 */
	uint8_t OneWire_CRC8(uint8_t* addr, uint8_t len);

/**
 * @}
 */

/**
 * @}
 */

/**
 * @}
 */

/* C++ detection */
#ifdef __cplusplus
}
#endif

#endif
