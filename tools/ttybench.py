#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# ttybench.py
# This file is part of minmal-cdc
#
# Copyright (C) 2015 - Kevin Laeufer <kevin.laeufer@rwth-aachen.de>
#
# minmal-cdc is free software; you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation; either version 2 of the License, or
# (at your option) any later version.
#
# minmal-cdc is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with minmal-cdc. If not, see <http://www.gnu.org/licenses/>.
#

from argparse import ArgumentParser
import serial, threading, time

class TtyBench(object):
	def __init__(self):
		parser = ArgumentParser()
		parser.add_argument("device", nargs=1, help="e.g. /dev/ttyACM0")
		parser.add_argument("-b", "--baudrate", dest="baudrate", default=115200,
			help="e.g. 115200", type=int)
		args = parser.parse_args()

		self.tx_data_size = 16
		self.tx_buffer_min  = 2048
		self.tx_max = 64

		# timeout of 0 means non blocking
		self.serial = serial.Serial(str(args.device[0]), args.baudrate, timeout=0)

		self.tx_data = bytes([(ii % 256) for ii in range(0,self.tx_data_size)])

		self.rx_count = 0
		self.tx_count = 0
		self.reading = False
		self.writing = False

		self.thread_read = threading.Thread(target=self.reader)
		self.thread_read.setDaemon(True)
		self.thread_write = threading.Thread(target=self.writer)
		self.thread_write.setDaemon(True)

	def run(self):
		self.reading = True
		self.thread_read.start()
		self.writing = True
		self.thread_write.start()
		time.sleep(1)
		self.writing = False
		time.sleep(0.1)
		self.reading = False
		self.thread_write.join()
		self.thread_read.join()
		print("tx_count: {} bit".format(self.tx_count * 8))
		print("rx_count: {} bit".format(self.rx_count * 8))

	def writer(self):
		while self.writing and self.tx_count < self.tx_max:
			if self.serial.outWaiting() < self.tx_buffer_min:
				n = self.serial.write(self.tx_data)
				self.tx_count += n

	def reader(self):
		while self.reading:
			data = self.serial.read(1)
			n = self.serial.inWaiting()
			if n:
				data = data + self.serial.read(n)
			if data:
				self.rx_count += len(data)
				# TODO: verify data

	def close(self):
		self.serial.close()

if __name__ == "__main__":
	tb = TtyBench()
	tb.run()
	tb.close()
