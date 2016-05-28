/*
 * This file is part of belnanosat project.
 *
 * Copyright (C) 2015 Uladzislau Paulovich <selatnick@gmail.com>
 *
 * belnanosat is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * belnanosat is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with belnanosat.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef USART_H
#define USART_H

#define USART1_ID USART2
#define USART1_RCC_ID RCC_USART2
#define USART1_PORT GPIOA
#define USART1_RCC_PORT RCC_GPIOA
#define USART1_TX GPIO2
#define USART1_RX GPIO3
#define USART1_BAUD_RATE 38400

#define USART2_ID USART3
#define USART2_RCC_ID RCC_USART3
#define USART2_PORT GPIOB
#define USART2_RCC_PORT RCC_GPIOB
#define USART2_TX GPIO10
#define USART2_RX GPIO11
#define USART2_BAUD_RATE 38400

//#define USART_AUTO_ECHO

extern void usart1_setup(void);
extern void usart2_setup(void);

#endif
