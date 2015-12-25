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

#define USART_ID USART3
#define USART_RCC_ID RCC_USART3
#define USART_PORT GPIOB
#define USART_RCC_PORT RCC_GPIOB
#define USART_TX GPIO10
#define USART_RX GPIO11
#define USART_BAUD_RATE 38400

#define USART_AUTO_ECHO

extern void usart_setup(void);

#endif
