/*
 * usb.cpp
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

#include "usb.hpp"
#include <xpcc/debug/logger.hpp>
#include <xpcc/architecture/driver/atomic/lock.hpp>

#undef	XPCC_LOG_LEVEL
#define	XPCC_LOG_LEVEL xpcc::log::DEBUG
//#define	XPCC_LOG_LEVEL xpcc::log::ERROR

struct USB_EP_BUFFER_TypeDef
{
	volatile uint16_t ADDR_TX;
	volatile uint16_t R0;
	volatile uint16_t COUNT_TX;
	volatile uint16_t R1;
	volatile uint16_t ADDR_RX;
	volatile uint16_t R2;
	volatile uint16_t COUNT_RX;
	volatile uint16_t R3;
};

template<typename T>
static constexpr T
min(T a, T b) { return (a < b)? a : b; }

#define BL_SIZE (uint16_t(1<<15))

// some registers that or only valid for a buffer table offset of 0
#define USB_EP0 ((USB_EP_BUFFER_TypeDef*) USB_PMAADDR)
#define USB_EP1 ((USB_EP_BUFFER_TypeDef*) (USB_PMAADDR + 16))
#define USB_EP2 ((USB_EP_BUFFER_TypeDef*) (USB_PMAADDR + 16 * 2))
#define USB_EP3 ((USB_EP_BUFFER_TypeDef*) (USB_PMAADDR + 16 * 3))

static constexpr uint8_t FlushTimeMs = 100;		// do not wait more than 100ms untill sending data

static constexpr uint8_t  bAcmEndpointAddress = 1;	// Endpoint1
static constexpr uint8_t  bRxEndpointAddress  = 2;	// Endpoint2
static constexpr uint8_t  bTxEndpointAddress  = 3;	// Endpoint3
static constexpr uint16_t wAcmEndpointMaxPacketSize = 16;
static constexpr uint16_t wRxEndpointMaxPacketSize  = 4 * 32;	// should be a multiple of 32
static constexpr uint16_t wTxEndpointMaxPacketSize  = 4 * 32;	// should be a multiple of 32

static constexpr uint16_t EP0_TX_ADDR = 8 * 4;	// max 8 endpoints, 4 words (u16) per endpoint
static constexpr uint16_t EP0_SIZE = 64;	// bytes
static constexpr uint16_t EP0_RX_ADDR = EP0_TX_ADDR  + (EP0_SIZE / 2);
static constexpr uint16_t EP1_TX_ADDR = EP0_RX_ADDR  + (EP0_SIZE / 2);
static constexpr uint16_t EP2_RX_ADDR = EP1_TX_ADDR  + (wAcmEndpointMaxPacketSize / 2);
static constexpr uint16_t EP3_TX_ADDR = EP2_RX_ADDR  + (wRxEndpointMaxPacketSize  / 2);

struct Ep0
{
	static inline uint8_t readRx(uint8_t byte) {
		return *reinterpret_cast<uint8_t*>(USB_PMAADDR + EP0_RX_ADDR * 2 + byte + (byte / 2) * 2);
	}
	static inline uint16_t readRxWord(uint8_t byte) {
		return (readRx(byte) << 8) | readRx(byte-1);
	}
	static inline void writeTx(uint8_t word_index, uint8_t value0, uint8_t value1) {
		writeTx(word_index, value0 | (value1 << 8));
	}
	static inline void writeTx(uint8_t word_index, uint16_t value) {
		*reinterpret_cast<uint16_t*>(USB_PMAADDR + EP0_TX_ADDR * 2 + word_index * 4) = value;
	}
	static inline void setRxStatus(uint16_t status) {
		const uint16_t reg = USB->EP0R;
		USB->EP0R = ((reg & USB_EPRX_STAT) ^ status) | (reg & USB_EPREG_MASK);
	}
	static inline void setTxStatus(uint16_t status) {
		const uint16_t reg = USB->EP0R;
		USB->EP0R = ((reg & USB_EPTX_STAT) ^ status) | (reg & USB_EPREG_MASK);
	}
};

// ACM Endpoint (IN)
struct Ep1
{
	static inline void writeTx(uint8_t word_index, uint8_t value0, uint8_t value1) {
		writeTx(word_index, value0 | (value1 << 8));
	}
	static inline void writeTx(uint8_t word_index, uint16_t value) {
		*reinterpret_cast<uint16_t*>(USB_PMAADDR + EP1_TX_ADDR * 2 + word_index * 4) = value;
	}
	static inline void setTxStatus(uint16_t status) {
		const uint16_t reg = USB->EP1R;
		USB->EP1R = ((reg & USB_EPTX_STAT) ^ status) | (reg & USB_EPREG_MASK);
	}
};

// Rx Endpoint (OUT)
struct Ep2
{
	static inline uint8_t readRx(uint8_t byte) {
		return *reinterpret_cast<uint8_t*>(USB_PMAADDR + EP2_RX_ADDR * 2 + byte + (byte / 2) * 2);
	}
	static inline uint16_t readRxWord(uint8_t byte) {
		return (readRx(byte) << 8) | readRx(byte-1);
	}
	static inline void setRxStatus(uint16_t status) {
		const uint16_t reg = USB->EP2R;
		USB->EP2R = ((reg & USB_EPRX_STAT) ^ status) | (reg & USB_EPREG_MASK);
	}
};

// Tx Endpoint (IN)
struct Ep3
{
	static inline uint16_t readTx(uint8_t word_index) {
		return *reinterpret_cast<uint16_t*>(USB_PMAADDR + EP3_TX_ADDR * 2 + word_index * 4);
	}
	static inline void writeTxByte(uint16_t byte_index, uint8_t value) {
		uint16_t old = readTx(byte_index / 2);
		if((byte_index % 2) == 0) {
			writeTx(byte_index / 2, (old & (0xff << 8)) | value);
		} else {
			writeTx(byte_index / 2, (old & 0xff) | (value << 8));
		}
	}
	static inline void writeTx(uint8_t word_index, uint8_t value0, uint8_t value1) {
		writeTx(word_index, value0 | (value1 << 8));
	}
	static inline void writeTx(uint8_t word_index, uint16_t value) {
		*reinterpret_cast<uint16_t*>(USB_PMAADDR + EP3_TX_ADDR * 2 + word_index * 4) = value;
	}
	static inline bool isTransmitting() {
		return ((USB->EP3R & USB_EPTX_STAT) == USB_EP_TX_VALID);
	}
	static inline void setTxStatus(uint16_t status) {
		const uint16_t reg = USB->EP3R;
		USB->EP3R = ((reg & USB_EPTX_STAT) ^ status) | (reg & USB_EPREG_MASK);
	}
};

/// bRequest, see USB 2.0 specification p. 251
enum class
Request : uint8_t
{
	GetStatus        = 0x00,
	ClearFeature     = 0x01,	///< clear or request a specific feature
	// ReservedForFutureUse = 0x02,
	SetFeature       = 0x03,	///< returns the current device configuration value
	// ReservedForFutureUse = 0x04,
	SetAddress       = 0x05,
	GetDescriptor    = 0x06,	///< returns the specific descriptor if it exists
	SetDescriptor    = 0x07,
	GetConfiguration = 0x08,
	SetConfiguration = 0x09,
	// CDC Specific Requests
	CdcSetLineCoding       = 0x20,
	CdcGetLineCoding       = 0x21,
	CdcSetControlLineState = 0x22,
	CdcSendBreak           = 0x23,
};
	enum class
	Type : uint8_t
	{
		Device                  = 1,
		Configuration           = 2,
		String                  = 3,
		Interface               = 4,
		Endpoint                = 5,
		DeviceQualifier         = 6,
		OtherSpeedConfiguration = 7,
		InterfacePower          = 8,
		OnTheGo                 = 9,
	};

// FIXME: think about how much space we acutally need
static xpcc::atomic::Queue<uint8_t, 512> rxBuffer;
static xpcc::atomic::Queue<uint8_t, 512> txBuffer;

void
Usb::initialize()
{
	// enable clock
	RCC->APB1ENR |= RCC_APB1ENR_USBEN;
	// reset usb
	RCC->APB1RSTR |=  RCC_APB1RSTR_USBRST;
	RCC->APB1RSTR &= ~RCC_APB1RSTR_USBRST;
	// powerdown and reset
	USB->CNTR = USB_CNTR_PDWN | USB_CNTR_FRES;
	// clear powerdown
	USB->CNTR &= ~USB_CNTR_PDWN;
	// wait a little
	xpcc::delayMicroseconds(1);
	// clear reset
	USB->CNTR &= ~USB_CNTR_FRES;
	// enable interrupts
	static constexpr uint8_t priority = 1;
	NVIC_SetPriority(USB_LP_IRQn, priority);
	NVIC_EnableIRQ(USB_LP_IRQn);
	// only reset and correct tranfer interrupts will be handled
	// + start of frame for flush timer
	USB->CNTR |= USB_CNTR_CTRM | USB_CNTR_RESETM | USB_ISTR_SOF;
}

static inline void
writeDeviceDescriptor()
{
	Ep0::writeTx( 0, 18, 1);		// bLength and bDescriptorType: Device
	Ep0::writeTx( 1, 0x0200);		// bcdUSB: USB 2.0
	Ep0::writeTx( 2, 0x02, 0x00);	// bDeviceClass: CDC and bDeviceSubClass
	Ep0::writeTx( 3, 0x00, 64);		// bDeviceProtocol and bMaxPacketSize0
	Ep0::writeTx( 4, 0x0483);		// idVendor
	Ep0::writeTx( 5, 0x5740);		// idProduct
	Ep0::writeTx( 6, 0x0200);		// bcdDevice
	Ep0::writeTx( 7, 0, 0);			// iManufacturer (no str) and iProduct (no str)
	Ep0::writeTx( 8, 0, 1);			// iSerial (no str) and bNumConfigurations
}

static inline void
writeConfigurationDescriptor(const uint16_t wLength)
{
	static constexpr uint8_t bMasterInterface = 0;
	static constexpr uint8_t bSlaveInterface0 = 1;

	Ep0::writeTx( 0,  9, 0x02);		// bLength and bDescriptorType
	Ep0::writeTx( 1, 67);			// wTotalLength
	Ep0::writeTx( 2,  2, 1);		// bNumInterfaces and bConfigurationValue
	Ep0::writeTx( 3, 0x00, 0xc0);	// iConfiguration (no str) and bmAttributes (self powered)
	Ep0::writeTx( 4,   50,    9);	// MaxPower 100mA and bLength of ACM interface
	if(wLength > 10) {	// at least linux first requests only 9 bytes thus we do not need to write more
		// Acm Control Interface
		Ep0::writeTx( 5, 4, bMasterInterface);	// bDescriptorType (Interface) and bInterfaceNumber
		Ep0::writeTx( 6, 0, 1);		// bAlternateSetting and bNumEndpoints
		Ep0::writeTx( 7, 2, 2);		// bInterfaceClass and bInterfaceSubClass
		Ep0::writeTx( 8, 1, 0);		// bInterfaceProtocol (AT commands) and iInterface (no str)
		// CDC Header Functional Descriptor
		Ep0::writeTx( 9, 0x05, 0x24);	// bLength and bDescriptorType (CS_INTERFACE)
		Ep0::writeTx(10, 0x00, 0x10);	// bDescriptorSubtype and first part of bcdCDC
		// Call Management Functional Descripto
		Ep0::writeTx(11, 0x01, 0x05);	// second part of bcdCDC and bLength
		Ep0::writeTx(12, 0x24, 0x01);	// bDescriptorType and bDescriptorSubtype
		Ep0::writeTx(13, 0, 1);			// bmCapabilities and bDataInterface
		// Abstract Control Management Functional Descriptor
		Ep0::writeTx(14, 0x04, 0x24);	// bLength and bDescriptorType (CS_INTERFACE)
		Ep0::writeTx(15, 0x02, 0x06);	// bDescriptorSubtype and bmCapabilities // TODO: 0x06 or 0x02?
		// Union Functional Descriptor
		Ep0::writeTx(16, 0x05, 0x24);	// bLength and bDescriptorType (CS_INTERFACE)
		Ep0::writeTx(17, 0x06, bMasterInterface);	// bDescriptorSubtype and bMasterInterface
		Ep0::writeTx(18, bSlaveInterface0, 7);		// bSlaveInterface0 and bLength of AcmEndpoint
		// Acm Endpoint
		// the 0x80 is needed since it is an IN endpoint
		Ep0::writeTx(19, 5, bAcmEndpointAddress | 0x80);		// bDescriptorType and bEndpointAddress
		Ep0::writeTx(20, 3, wAcmEndpointMaxPacketSize & 0xff);	// bmAttributes: Interrupt and wMaxPacketSize
		Ep0::writeTx(21, (wAcmEndpointMaxPacketSize >> 8), 64);	// bIntervall

		// Data Interface
		Ep0::writeTx(22, 9, 4);					// bLength and bDescriptorType (Interface)
		Ep0::writeTx(23, bSlaveInterface0, 0);	// bInterfaceNumber and bAlternateSetting
		Ep0::writeTx(24, 2, 0x0a);				// bNumEndpoints and bInterfaceClass (CdcData)
		Ep0::writeTx(25, 0x00, 0x00);			// bInterfaceSubClass and bInterfaceProtocol
		Ep0::writeTx(26,    0, 7);				// iInterface (no str) and bLength of Rx Endpoint
		// Rx Endpoint
		Ep0::writeTx(27,    5, bRxEndpointAddress);					// bDescriptorType and bEndpointAddress
		Ep0::writeTx(28, 0x02, wRxEndpointMaxPacketSize & 0xff);	// bmAttributes: Bulk and wMaxPacketSize
		Ep0::writeTx(29, (wRxEndpointMaxPacketSize >> 8), 0);		// bIntervall
		// Tx Endpoint
		Ep0::writeTx(30, 7, 5);						// bLength and bDescriptorType
		// the 0x80 is needed since it is an IN endpoint
		Ep0::writeTx(31, bTxEndpointAddress | 0x80, 0x02);	// bEndpointAddress and bmAttributes: Bulk
		// will be sent in a seconds packet
		// Ep0::writeTx(32, wTxEndpointMaxPacketSize);			// wMaxPacketSize
		// Ep0::writeTx(33, 0, 0);								// bIntervall and dummy
	}
}

enum class
SetupState
{
	Idle,
	SetAddress,
	ConfigurationDescriptor,
	CdcSetLineCoding,
};

static inline void
setup()
{
	static uint16_t address = 0;
	static SetupState state = SetupState::Idle;

	if(USB->EP0R & USB_EP_CTR_RX) {
		USB->EP0R = USB->EP0R & ~USB_EP_CTR_RX & USB_EPREG_MASK;
		Ep0::setRxStatus(USB_EP_RX_NAK);

		// TODO: we should stall the enpoint if there is a request that we do
		//       not want to answer, e.g. the GET DEVICE QUALIFIER request

		if(USB->EP0R & USB_EP_SETUP) {
			const uint16_t wLength = Ep0::readRxWord(7);
			XPCC_LOG_DEBUG << "wLength: " << wLength << xpcc::endl;
			switch(static_cast<Request>(Ep0::readRx(1))) {
			case Request::GetStatus:
				XPCC_LOG_DEBUG << "* GetStatus" << xpcc::endl;
				break;
			case Request::ClearFeature:
				XPCC_LOG_DEBUG << "* ClearFeature" << xpcc::endl;
				break;
			case Request::SetFeature:
				XPCC_LOG_DEBUG << "* SetFeature" << xpcc::endl;
				break;
			case Request::SetAddress:
				address = Ep0::readRxWord(3);
				XPCC_LOG_DEBUG << "* SetAddress" << xpcc::endl;
				XPCC_LOG_DEBUG << "Device Address: " << Ep0::readRxWord(3) << xpcc::endl;
				state = SetupState::SetAddress;
				USB_EP0->COUNT_TX = 0;
				Ep0::setTxStatus(USB_EP_TX_VALID);
				break;
			case Request::GetDescriptor:
				XPCC_LOG_DEBUG << "* GetDescriptor" << xpcc::endl;
				// TODO: handle wLength
				switch(Ep0::readRx(3)) {
				case 1:	// Device
					XPCC_LOG_DEBUG << "  * Device" << xpcc::endl;
					// return device descriptor
					writeDeviceDescriptor();
					// send
					USB_EP0->COUNT_TX = 18;
					Ep0::setTxStatus(USB_EP_TX_VALID);
					break;

				case 2:	// Configuration
					XPCC_LOG_DEBUG << "  * Configuration" << xpcc::endl;
					// return device descriptor
					writeConfigurationDescriptor(wLength);
					// send
					USB_EP0->COUNT_TX = min(wLength, static_cast<uint16_t>(64));	// TODO: how can we send packets that are longer than 64 bytes
					if(wLength > 64) state = SetupState::ConfigurationDescriptor;	// write second part later
					Ep0::setTxStatus(USB_EP_TX_VALID);
					break;

				case 6:	// Device Qulifier
					// nobody seems to fucking care about this
					// neither ST in their ST Link v2 (as seen with wireshark)
					// nor Paul with his Teensy implement this
					Ep0::setTxStatus(USB_EP_TX_STALL);
					break;
				}
				break;
			case Request::SetDescriptor:
				XPCC_LOG_DEBUG << "* SetDescriptor" << xpcc::endl;
				break;
			case Request::GetConfiguration:
				XPCC_LOG_DEBUG << "* GetDescriptor" << xpcc::endl;
				break;
			case Request::SetConfiguration:
				XPCC_LOG_DEBUG << "* SetConfiguration" << xpcc::endl;
				// maybe we should check the configuration number, but since
				// we only have one and do not really strive to have
				// a standard compliant stack, let's just assume that
				// the host requests to activate the one configuration
				// that we provide
				// initialize endpoints that will be used

				// ACM Endpoint
				USB->EP1R = USB_EP_INTERRUPT | USB_EP_TX_NAK | bAcmEndpointAddress;
				USB_EP1->ADDR_TX = EP1_TX_ADDR;
				// standard response: SERIAL_STATE
				Ep1::writeTx(0, 0xa1, 0x20);	// bmRequestType, bRequest
				Ep1::writeTx(1, 0x00, 0x00);
				Ep1::writeTx(2, 0x00, 0x00);
				Ep1::writeTx(3, 0x02, 0x00);	// wLenght : 2
				Ep1::writeTx(4, 0x00, 0x00);	// no problems
				USB_EP1->COUNT_TX = 10;
				// USB_EP1->COUNT_TX = 0;
				Ep1::setTxStatus(USB_EP_TX_VALID);

				// Rx Endpoint
				USB->EP2R = USB_EP_BULK | USB_EP_RX_VALID | bRxEndpointAddress;
				USB_EP2->ADDR_RX = EP2_RX_ADDR;
				USB_EP2->COUNT_RX = BL_SIZE | (((wRxEndpointMaxPacketSize / 32) - 1)<<10);
				// Tx Endpoint
				USB->EP3R = USB_EP_BULK | USB_EP_TX_NAK | bTxEndpointAddress;
				USB_EP3->ADDR_TX = EP3_TX_ADDR;
				USB_EP3->COUNT_TX = 0;
				// the configuration was activated so we should be connected
				// TODO: are we really connected now?
				Usb::connected = true;
				// send 0 length packet
				USB_EP0->COUNT_TX = 0;
				Ep0::setTxStatus(USB_EP_TX_VALID);
				break;
			case Request::CdcSetLineCoding:
				XPCC_LOG_DEBUG << "* Cdc Set Line Coding" << xpcc::endl;
				// wait for data
				state = SetupState::CdcSetLineCoding;
				Ep0::setRxStatus(USB_EP_RX_VALID);
				break;
			case Request::CdcGetLineCoding:
				XPCC_LOG_DEBUG << "* Cdc Get Line Coding" << xpcc::endl;
				XPCC_LOG_DEBUG << "TODO: implement!" << xpcc::endl;
				break;
			case Request::CdcSetControlLineState:
				XPCC_LOG_DEBUG << "* Cdc Set Controll Line State" << xpcc::endl;
				// of cause we did what you told us .....
				// send 0 length packet
				USB_EP0->COUNT_TX = 0;
				Ep0::setTxStatus(USB_EP_TX_VALID);
				break;
			case Request::CdcSendBreak:
				XPCC_LOG_DEBUG << "* Cdc Send Break" << xpcc::endl;
				// of cause we did what you told us .....
				// send 0 length packet
				USB_EP0->COUNT_TX = 0;
				Ep0::setTxStatus(USB_EP_TX_VALID);
				break;
			}
		} else {
			if(state == SetupState::CdcSetLineCoding) {
				XPCC_LOG_DEBUG << "* Cdc Set Controll Line State Data" << xpcc::endl;
				// of cause we did what you told us .....
				// send 0 length packet
				USB_EP0->COUNT_TX = 0;
				Ep0::setTxStatus(USB_EP_TX_VALID);
				state = SetupState::Idle;
			} else {
				XPCC_LOG_DEBUG << "* unknown EP0 Data OUT" << xpcc::endl;
			}
		}
		// reset endpoint length
		USB_EP0->COUNT_RX = BL_SIZE | (1<<10);
	} else {
		USB->EP0R = USB->EP0R & ~USB_EP_CTR_TX & USB_EPREG_MASK;
		Ep0::setRxStatus(USB_EP_TX_NAK);

		switch(state) {
		case SetupState::Idle:
			XPCC_LOG_DEBUG << "* Setup in" << xpcc::endl;
			Ep0::setRxStatus(USB_EP_RX_VALID);
			break;
		case SetupState::SetAddress:
			XPCC_LOG_DEBUG << "Enabled Address: " << address << xpcc::endl;
			USB->DADDR = USB_DADDR_EF | address;
			Ep0::setRxStatus(USB_EP_RX_VALID);
			state = SetupState::Idle;
			break;
		case SetupState::ConfigurationDescriptor:
			// XPCC_LOG_DEBUG << "Second part of ConfigurationDescriptor" << xpcc::endl;
			Ep0::writeTx(32-32, wTxEndpointMaxPacketSize);	// wMaxPacketSize
			Ep0::writeTx(33-32, 0, 0);						// bIntervall and dummy
			// send
			USB_EP0->COUNT_TX = 3;
			Ep0::setTxStatus(USB_EP_TX_VALID);
			state = SetupState::Idle;
			break;
		}
	}
}

static inline void
acm_endpoint()
{
	if(USB->EP1R & USB_EP_CTR_RX) {
		USB->EP1R = USB->EP1R & ~USB_EP_CTR_RX & USB_EPREG_MASK;
		XPCC_LOG_DEBUG << "ACM OUT" << xpcc::endl;
	} else {
		USB->EP1R = USB->EP1R & ~USB_EP_CTR_TX & USB_EPREG_MASK;
		XPCC_LOG_DEBUG << "ACM IN" << xpcc::endl;
	}
}

static inline void
rx_endpoint()
{
	if(USB->EP2R & USB_EP_CTR_RX) {
		USB->EP2R = USB->EP2R & ~USB_EP_CTR_RX & USB_EPREG_MASK;
		Ep2::setRxStatus(USB_EP_RX_NAK);
//		XPCC_LOG_DEBUG << "RX OUT" << xpcc::endl;
		const uint16_t count = USB_EP2->COUNT_RX & ((1<<10)-1);
		for(int ii = 0; ii < count; ++ii) {
			uint8_t data = Ep2::readRx(ii);
			rxBuffer.push(data);
//			XPCC_LOG_DEBUG << data << ": 0x" << xpcc::hex << data << xpcc::endl;
		}
		// reset rx count
		USB_EP2->COUNT_RX = BL_SIZE | (((wRxEndpointMaxPacketSize / 32) - 1)<<10);
		Ep2::setRxStatus(USB_EP_RX_VALID);
	} else {
		USB->EP2R = USB->EP2R & ~USB_EP_CTR_TX & USB_EPREG_MASK;
		XPCC_LOG_DEBUG << "Error: RX IN" << xpcc::endl;
	}
}


static inline void
tx_endpoint()
{
	if(USB->EP3R & USB_EP_CTR_RX) {
		USB->EP3R = USB->EP3R & ~USB_EP_CTR_RX & USB_EPREG_MASK;
		XPCC_LOG_DEBUG << "Error: TX OUT" << xpcc::endl;
	} else {
		USB->EP3R = USB->EP3R & ~USB_EP_CTR_TX & USB_EPREG_MASK;
		Ep3::setTxStatus(USB_EP_TX_NAK);
		USB_EP3->COUNT_TX = 0;
		XPCC_LOG_DEBUG << "TX IN" << xpcc::endl;
		// write buffered data to USB buffer
		uint16_t count = 0;
		while(!txBuffer.isEmpty() && count <= wTxEndpointMaxPacketSize) {
			Ep3::writeTxByte(count, txBuffer.get());
			txBuffer.pop();
			count++;
		}
		USB_EP3->COUNT_TX = count;
		if(count == wTxEndpointMaxPacketSize) {
			Ep3::setTxStatus(USB_EP_TX_VALID);
		}
	}
}



static inline void
handleLowPriorityInterrupt()
{
	const uint16_t flag = USB->ISTR;

	if(flag & USB_ISTR_RESET) {
		USB->ISTR = ~USB_ISTR_RESET;
		USB->BTABLE = 0;	// see page 900
		// initialize endpoint 0
		USB->EP0R = USB_EP_CONTROL | USB_EP_TX_STALL;
		USB_EP0->ADDR_TX = EP0_TX_ADDR;
		USB_EP0->ADDR_RX = EP0_RX_ADDR;
		// 64 byte buffer (2*32) see table 125 page 925
		USB_EP0->COUNT_RX = BL_SIZE | (((EP0_SIZE / 32) - 1)<<10);
		USB->EP0R |= USB_EP_RX_VALID;
		// reset address
		USB->DADDR = USB_DADDR_EF;
		XPCC_LOG_DEBUG << "* Reset" << xpcc::endl;
	} else if(flag & USB_ISTR_CTR) {
		USB->ISTR = ~USB_ISTR_CTR;
		const uint16_t endpoint = flag & 0x0f;
		const bool out_not_in = static_cast<bool>(flag & USB_ISTR_DIR);
		switch(endpoint) {
		case 0:
			setup();
			break;
		case 1:
			acm_endpoint();
			break;
		case 2:
			rx_endpoint();
			break;
		case 3:
			tx_endpoint();
			break;
		default:
			if(out_not_in) {
				XPCC_LOG_DEBUG << "* OUT: EP" << endpoint << xpcc::endl;
			} else {
				XPCC_LOG_DEBUG << "* IN: EP" << endpoint << xpcc::endl;
			}
			break;
		}
	} else if(flag & USB_ISTR_SOF) {
		USB->ISTR = ~USB_ISTR_SOF;
		static uint8_t flushTimer = 0;
		flushTimer++;
		if(flushTimer >= FlushTimeMs) {
			Usb::flushTx();
			flushTimer = 0;
		}
	} else {
		XPCC_LOG_DEBUG << "?? " << xpcc::hex << flag << xpcc::endl;
	}
}

bool
Usb::isConnected()
{
	return connected;
}

void
Usb::flushTx()
{
	xpcc::atomic::Lock lock;

	if(USB_EP3->COUNT_TX > 0) {
		Ep3::setTxStatus(USB_EP_TX_VALID);
	}
}

bool
Usb::write(uint8_t data)
{
	xpcc::atomic::Lock lock;

	if(Ep3::isTransmitting()) {
		if (!txBuffer.push(data))
			return false;
	} else {
		uint16_t count = USB_EP3->COUNT_TX;
		Ep3::writeTxByte(count, data);
		count++;
		USB_EP3->COUNT_TX = count;
		if(count == wTxEndpointMaxPacketSize) {
			Ep3::setTxStatus(USB_EP_TX_VALID);
		}
	}
	return true;
}

bool
Usb::read(uint8_t &data)
{
	xpcc::atomic::Lock lock;
	if (rxBuffer.isEmpty()) {
		return false;
	} else {
		data = rxBuffer.get();
		rxBuffer.pop();
		return true;
	}
}

// ----- Interrupts ----
extern "C" void USB_LP_IRQHandler(void){ handleLowPriorityInterrupt(); }

// memory
volatile bool Usb::connected = false;
