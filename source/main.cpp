/*
 * main.cpp
 * This file is part of minmal-cdc
 *
 * Copyright (C) 2015 - eKiwi <electron.kiwi@gmail.com>
 *
 * minmal-cdc is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * minmal-cdc is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with minmal-cdc. If not, see <http://www.gnu.org/licenses/>.
 */

#include <xpcc/architecture.hpp>
#include <xpcc/debug/logger.hpp>
#include <xpcc/processing/timer.hpp>
#include "../xpcc/examples/stm32f3_discovery/stm32f3_discovery.hpp"
using namespace Board;
#include "usb.hpp"

xpcc::IODeviceWrapper<Usart2, xpcc::IOBuffer::DiscardIfFull> loggerDevice;
xpcc::log::Logger xpcc::log::debug(loggerDevice);
xpcc::log::Logger xpcc::log::info(loggerDevice);

static void printGpl();

xpcc::PeriodicTimer timer(500);

MAIN_FUNCTION
{
	systemClock::enable();
	xpcc::cortex::SysTickTimer::initialize<systemClock>();

	LedNorth::setOutput(xpcc::Gpio::Low);
	LedNorthEast::setOutput(xpcc::Gpio::Low);
	LedEast::setOutput(xpcc::Gpio::Low);
	LedSouthEast::setOutput(xpcc::Gpio::Low);
	LedSouth::setOutput(xpcc::Gpio::Low);
	LedSouthWest::setOutput(xpcc::Gpio::Low);
	LedWest::setOutput(xpcc::Gpio::Low);
	LedNorthWest::setOutput(xpcc::Gpio::Low);

	GpioOutputA2::connect(Usart2::Tx);
	GpioInputA3::connect(Usart2::Rx);
	Usart2::initialize<systemClock, 115200>(10);

	printGpl();

	GpioA11::connect(Usb::Dm);
	GpioA12::connect(Usb::Dp);
	Usb::initialize();

	// wait for usb to connect
	while(!Usb::isConnected())
		;

	XPCC_LOG_INFO << "Info: connected" << xpcc::endl;

	while (1)
	{
		uint8_t data;
		if(Usb::read(data)) {
			XPCC_LOG_INFO << "+1" << xpcc::endl;
			Usb::write(data);
		}
		if(timer.execute()) {
			LedSouth::toggle();
		}
	}

	return 0;
}

static inline void
printGpl()
{
	XPCC_LOG_INFO <<
	"minimal-cdc: minimal USB stack on STM32F3\n"
	"Copyright (C) 2015 eKiwi <electron.kiwi@gmail.com>\n"
	"\n"
	"This program is free software: you can redistribute it and/or modify\n"
	"it under the terms of the GNU General Public License as published by\n"
	"the Free Software Foundation, either version 3 of the License, or\n"
	"(at your option) any later version.\n"
	"\n"
	"This program is distributed in the hope that it will be useful,\n"
	"but WITHOUT ANY WARRANTY; without even the implied warranty of\n"
	"MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the\n"
	"GNU General Public License for more details.\n"
	"\n"
	"You should have received a copy of the GNU General Public License\n"
	"along with this program.  If not, see <http://www.gnu.org/licenses/>.\n";
}
