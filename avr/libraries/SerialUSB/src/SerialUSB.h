/*

CDC Arduino Library by Osamu Tamura, Ihsan Kehribar (kehribar.me)
and Digistump LLC (digistump.com)
- all changes made under the same license as V-USB


*/
#pragma once

#include "Stream.h"

#ifndef HW_CDC_TX_BUF_SIZE    // Allow user to override our default.
#define HW_CDC_TX_BUF_SIZE    32
#endif
#ifndef HW_CDC_RX_BUF_SIZE    // Allow user to override our default.
#define HW_CDC_RX_BUF_SIZE    32
#endif

class SerialUSBDevice : public Stream {
public:
	void begin();
	void begin(unsigned long) { begin(); }
	void end();
	void refresh();
	void task();
	void delay(long millis);
	virtual int available(void);
	virtual int peek(void);
	virtual int read(void);
	virtual void flush(void);
	virtual size_t write(uint8_t);
	using Print::write;
	operator bool();
private:
	void usbBegin();
	void usbPollWrapper();
};

extern SerialUSBDevice SerialUSB;
