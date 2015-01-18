/*
 * usb.hpp
 * This file is part of minimal-cdc
 *
 * Copyright (C) 2015 - eKiwi <electron.kiwi@gmail.com>
 *
 * minimal-cdc is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * minimal-cdc is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with minimal-cdc. If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef USB_HPP
#define USB_HPP

#include <xpcc/architecture.hpp>
#include<stdint.h>

class
Usb
{
public:
	static const xpcc::stm32::TypeId::UsbDm Dm;
	static const xpcc::stm32::TypeId::UsbDp Dp;

public:
	static void
	initialize();

	static bool
	isConnected();

	/// send all buffered bytes if any
	static void
	flushTx();

	static bool
	write(uint8_t data);

	static bool
	read(uint8_t &data);

public:
	// TODO: make private
	static volatile bool
	connected;
};

#endif // USB_HPP
