/*
	Cansat_RFM96 library,
	Written by NAROM, 2018.
	
	Todo: write a function that return the current
	buffer. This makes it easier for the user to output the buffer
	to a Serial.write to debug the program.
*/

#include "Arduino.h"
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include "Cansat_RFM96.h"

#define PRINT_VERBOSE	0

Cansat_RFM96* Cansat_RFM96::_deviceForInterrupt = 0;

Cansat_RFM96::Cansat_RFM96(unsigned long freq_kHz, bool use_sd, int cs_pin, int int_pin, int rst_pin) {
	_cs = cs_pin; //Cansat_RFM96_CS;
	_int = int_pin; //Cansat_RFM96_INT;
	_rst = rst_pin; //Cansat_RFM96_RST;
	
	if ((freq_kHz < 430000) || (freq_kHz > 436000))
		freq_kHz = 434000; // Set to default
	_userID = freq_kHz-420000;
	_freq = freq_kHz/1000.0;
	
	_tx_buf[0] = _userID&0xFF;
	_tx_buf[1] = _userID>>8;
	
	_use_sd = use_sd;
	_init_sd = false;
	
	_buf_len = 0;
	_buf_pos = 0;
	_tx_buf_len = 0;
	_last_RSSI = -137;
	_avail = false;
	_tx_done = false; // Not sending anything
	
	_use_timeouts = true;
	_tx_power = 13;
	
	_is_TX_timeout_on = false;
	_restarts = 0;
	_tx_timeout_vs_valid = 0;
	_rx_timeout_vs_valid = 0;
	_valid_TXs = 0;
	_timeouts = 0;
	
	_bw = 6; // We default this to 62.5 kHz
	_coding_rate = 1; // We default to 4/5
	_spreading_factor = 7; // We default to 128 chips/symbol
	_timeout_time = 20E6; // This will be changed at restart anyways...
	_use_lowopt = false;
}

bool Cansat_RFM96::init() {
	SPI_settings = SPISettings(8000000, MSBFIRST, SPI_MODE0);
	
	SPI.begin();
	
	digitalWriteFast(_cs, HIGH);
	pinMode(_cs, OUTPUT);
	pinMode(_int, INPUT);
	digitalWriteFast(_rst, HIGH);
	pinMode(_rst, OUTPUT);
	
	delay(10); // Let's do a reset first and then initialize
	restart();
	
	// Prepare for interrupts
	_deviceForInterrupt = this;
	attachInterrupt(_int, onPulse, RISING);
	SPI.usingInterrupt(_int);
	
	tid = millis();
	
	if (_use_sd) {
		_init_sd = SD.begin(BUILTIN_SDCARD);
		if (_init_sd) // Should only do this if, if init was a success
			return open_new_file();
		return _init_sd;
	}

	return true;
}

void Cansat_RFM96::restart() {
	_restarts++;
	
	digitalWriteFast(_rst, LOW);
	delay(10);
	digitalWriteFast(_rst, HIGH);
	delay(10);
	
	spiWrite(RFM96_REG_01_OP_MODE, RFM96_MODE_SLEEP | RFM96_LONG_RANGE_MODE);
	delay(20);
	
	spiWrite(RFM96_REG_0E_FIFO_TX_BASE_ADDR, 0);
    spiWrite(RFM96_REG_0F_FIFO_RX_BASE_ADDR, 0);
	spiWrite(RFM96_REG_01_OP_MODE, RFM96_MODE_STDBY);
	_mode = MODE_STDBY;
	
	spiWrite(RFM96_REG_26_MODEM_CONFIG3, 0x00);
	setModem(_bw, _coding_rate, _spreading_factor);
    
	
	spiWrite(RFM96_REG_20_PREAMBLE_MSB, 0x00);
    spiWrite(RFM96_REG_21_PREAMBLE_LSB, 0x08 & 0xff);
	
	spiWrite(RFM96_REG_4D_PA_DAC, RFM96_PA_DAC_DISABLE);
	//spiWrite(RFM96_REG_09_PA_CONFIG, RFM96_PA_SELECT | (13-5));
	
	//spiWrite(RFM96_REG_01_OP_MODE, RFM96_MODE_RXCONTINUOUS);
	//spiWrite(RFM96_REG_40_DIO_MAPPING1, 0x00); // Interrupt on RxDone
	
	spiWrite(RFM96_REG_0C_LNA, RFM96_LNA_GAIN | RFM96_LNA_BOOST_HF); // Maximum LNA gain and high HF boost
	
	
	setModeRx();
	
	setFrequency(_freq);
	setTxPower(_tx_power);
}

unsigned int Cansat_RFM96::actual_SF() { // in chips/symbol
	if (_spreading_factor == 6)
		return 64;
	if (_spreading_factor == 7)
		return 128;
	if (_spreading_factor == 8)
		return 256;
	if (_spreading_factor == 9)
		return 512;
	if (_spreading_factor == 10)
		return 1024;
	if (_spreading_factor == 11)
		return 2048;
	if (_spreading_factor == 12)
		return 4096;
	else
		return 1; // This should never happen
}

double Cansat_RFM96::actual_BW() { // in kHz
	if (_bw == 0)
		return 7.8;
	if (_bw == 1)
		return 10.4;
	if (_bw == 2)
		return 15.6;
	if (_bw == 3)
		return 20.8;
	if (_bw == 4)
		return 31.25;
	if (_bw == 5)
		return 41.7;
	if (_bw == 6)
		return 62.5;
	if (_bw == 7)
		return 125.0;
	if (_bw == 8)
		return 250.0;
	if (_bw == 9)
		return 500.0;
	else
		return 100000000.0; // This shold never happen
}

void Cansat_RFM96::spiWrite(uint8_t adr, uint8_t data) {
	SPI.beginTransaction(SPI_settings);
	digitalWriteFast(_cs, LOW);
	
	SPI.transfer((adr | SPI_WRITE_MASK) & 0xFF);
	SPI.transfer(data&0xFF);
	
	digitalWriteFast(_cs, HIGH);
	SPI.endTransaction();
}

void Cansat_RFM96::spiBurstWrite(uint8_t adr, uint8_t* data, uint8_t len) {
	SPI.beginTransaction(SPI_settings);
	digitalWriteFast(_cs, LOW);
	
	SPI.transfer((adr | SPI_WRITE_MASK) & 0xFF);
	for (uint8_t i=0; i<len; i++) {
		SPI.transfer(data[i]&0xFF);
	}
	
	digitalWriteFast(_cs, HIGH);
	SPI.endTransaction();
}

uint8_t Cansat_RFM96::spiRead(uint8_t adr) {
	uint8_t tmp_data;
	
	SPI.beginTransaction(SPI_settings);
	digitalWriteFast(_cs, LOW);
	
	SPI.transfer(adr & (~SPI_WRITE_MASK));
	tmp_data = SPI.transfer(0);
	
	digitalWriteFast(_cs, HIGH);
	SPI.endTransaction();
	
	return tmp_data;
}

void Cansat_RFM96::spiBurstRead(uint8_t adr, uint8_t* dest, uint8_t len) {
	SPI.beginTransaction(SPI_settings);
	digitalWriteFast(_cs, LOW);
	
	SPI.transfer(adr & ~SPI_WRITE_MASK);
	for (uint8_t i=0; i<len; i++)
		dest[i] = SPI.transfer(0);
	
	digitalWriteFast(_cs, HIGH);
	SPI.endTransaction();
}

void Cansat_RFM96::onPulse() {
	_deviceForInterrupt->handleInterrupt();
}

void Cansat_RFM96::onTimeout() {
	_deviceForInterrupt->handleTimeoutInterrupt();
}

void Cansat_RFM96::handleInterrupt() {
	uint8_t _tmp_buf_len = 0;
	clearIRQs();
	
	// Probably should check for TX/RX errors here, by maybe reading IRX registers
	if (_mode == MODE_RX_CONT) {
#if Cansat_RFM96_DEBUG
		tid = millis();
		Serial.print("Read: ");
		Serial.print(tid);
#endif
	
		_tmp_buf_len = spiRead(RFM96_REG_13_RX_NB_BYTES);
		if (_tmp_buf_len > 2) {
			spiWrite(RFM96_REG_0D_FIFO_ADDR_PTR,
				spiRead(RFM96_REG_10_FIFO_RX_CURRENT_ADDR));
			spiBurstRead(RFM96_REG_00_FIFO, _buf_ID, 2);
			_buf_pos = 0;
		
#if Cansat_RFM96_DEBUG
			Serial.print("   _buf_ID[0:1] : ");
			Serial.print(_buf_ID[0]);
			Serial.print(_buf_ID[1]);
			Serial.print("    ID: 0x");
			Serial.print(_buf_ID[0] | (_buf_ID[1]<<8), HEX);
#endif
			
			//if ((_buf_ID[0] == (_userID&0xFF)) && ((_buf_ID[1] == (_userID<<8)))) { //is this correct?
			if ((_buf_ID[0] == _tx_buf[0]) && (_buf_ID[1] == _tx_buf[1])) {
				end_timer();
				if (_rx_timeout_vs_valid > 2)
					_rx_timeout_vs_valid--;
				
#if Cansat_RFM96_DEBUG
				Serial.println("   (Same ID)");
#endif
				_buf_len = _tmp_buf_len;
				spiWrite(RFM96_REG_0D_FIFO_ADDR_PTR,
					spiRead(RFM96_REG_10_FIFO_RX_CURRENT_ADDR));
				spiBurstRead(RFM96_REG_00_FIFO, _buf, _buf_len);
	
				_last_RSSI = -137+spiRead(RFM96_REG_1A_PKT_RSSI_VALUE);
				_avail = true;
				_buf_pos = 0;
			}
			else {
#if Cansat_RFM96_DEBUG
				Serial.println("   (Wrong ID)");
#endif
			}
		}
		
		// To reset the RX buffer. Better way to do it?
		//setModeStdby();
		//setModeRx();
		start_timer();
		spiWrite(RFM96_REG_0F_FIFO_RX_BASE_ADDR, 0);
	}
	
	else if (_mode == MODE_TX) {
		end_timer(); // So not triggering a timeout
		if (_tx_timeout_vs_valid > 2);
			_tx_timeout_vs_valid--;
		setModeStdby(); // You have to manually set it to RX if that's what you want.
		_tx_done = true;
		_valid_TXs++;
	}
	
	clearIRQs();
}

void Cansat_RFM96::handleTimeoutInterrupt() {
	end_timer();
	_timeouts++;
	
	clearIRQs();
	
	if (whatMode() == MODE_TX) {
		//Serial.println("Timeout on tx");
		_tx_done = true;
		
		setModeStdby();
		_tx_timeout_vs_valid += 4;
	
		if (_tx_timeout_vs_valid > 20) {
			_tx_timeout_vs_valid = 0;
			restart(); // Ugly, but maybe needs to be done?
		}
	}
	
	if (whatMode() == MODE_RX_CONT) {
		setModeStdby();
		setModeRx();
		
		_rx_timeout_vs_valid += 4;
		
	
		if (_rx_timeout_vs_valid > 20) {
			_rx_timeout_vs_valid = 0;
			restart(); // Ugly, but maybe needs to be done?
		}
	}
}

uint8_t Cansat_RFM96::available() {
	if (!_avail)
		return 0;
	return _buf_len-2-_buf_pos;
}

int Cansat_RFM96::last_RSSI() {
	return _last_RSSI;
}

// Easy to make mistakes with this one, only use with care.
// Easier to use read()
uint8_t Cansat_RFM96::read(char* dest) {
	if (!_avail)
		return 0;
	
	noInterrupts()
	uint8_t buf_len = _buf_len-2;
	
	for (int i=0; i<(buf_len-2-_buf_pos); i++)
		dest[i] = _buf[_buf_pos+i+2];
	
	_buf_pos = 0;
	_buf_len = 2;
	_avail = false;
	interrupts();	
	
	// _buf_len is guaranteed to be larger than 0 (check at reading),
	// so we are not generating a false (error)	
	return buf_len;
}

char Cansat_RFM96::read(void) {
	if (!_avail)
		return 0;
	
	if (_buf_pos >= _buf_len-2)
		return 0;
	
	_buf_pos++;
	return(_buf[_buf_pos+2-1]);
}

void Cansat_RFM96::setFrequency(float centre) {
	_freq = centre;
	
	if ((_freq < 430.000) || (_freq > 436.000))
		_freq = 434.000; // Set to default
	_userID = (unsigned int)(_freq*1000)-420000;
	
	_tx_buf[0] = _userID&0xFF;
	_tx_buf[1] = _userID>>8;
	
    uint32_t frf = (centre * 1000000.0) / RFM96_FSTEP;
    spiWrite(RFM96_REG_06_FRF_MSB, (frf >> 16) & 0xff);
    spiWrite(RFM96_REG_07_FRF_MID, (frf >> 8) & 0xff);
    spiWrite(RFM96_REG_08_FRF_LSB, frf & 0xff);
}

void Cansat_RFM96::setTxPower(int8_t power) {
	if (power > 23)
		power = 23;
	if (power < 5) // Really, not lower than +5 dBm?
		power = 5;

	// For RFM96_PA_DAC_ENABLE, manual says '+20dBm on PA_BOOST when OutputPower=0xf'
	// RFM96_PA_DAC_ENABLE actually adds about 3dBm to all power levels.
	// We will us it for 21, 22 and 23dBm
	if (power > 20) {
		spiWrite(RFM96_REG_4D_PA_DAC, RFM96_PA_DAC_ENABLE);
		power -= 3;
	}
	else
		spiWrite(RFM96_REG_4D_PA_DAC, RFM96_PA_DAC_DISABLE);

	spiWrite(RFM96_REG_09_PA_CONFIG, RFM96_PA_SELECT | (power-5));
	
	_tx_power = power;
}

void Cansat_RFM96::setModeStdby() {
	end_timer();
    if (_mode != MODE_STDBY) {
		spiWrite(RFM96_REG_01_OP_MODE, RFM96_MODE_STDBY);
		_mode = MODE_STDBY;
    }
}

void Cansat_RFM96::setModeSleep() {
	end_timer();
    if (_mode != MODE_SLEEP) {
		spiWrite(RFM96_REG_01_OP_MODE, RFM96_MODE_SLEEP);
		_mode = MODE_SLEEP;
    }
}

void Cansat_RFM96::setModeRx() {
    if (_mode != MODE_RX_CONT) {
		start_timer();
		spiWrite(RFM96_REG_01_OP_MODE, RFM96_MODE_RXCONTINUOUS);
		spiWrite(RFM96_REG_40_DIO_MAPPING1, 0x00); // Interrupt on RxDone
		_mode = MODE_RX_CONT;
    }
}

void Cansat_RFM96::setModeTx() {
    if (_mode != MODE_TX) {
		end_timer();
		spiWrite(RFM96_REG_01_OP_MODE, RFM96_MODE_TX);
		spiWrite(RFM96_REG_40_DIO_MAPPING1, 0x40); // Interrupt on TxDone
			
		_mode = MODE_TX;
		_tx_done = false;
	}
}

uint8_t Cansat_RFM96::whatMode() {
	return _mode;
}

bool Cansat_RFM96::isTxReady() { 
	if (_mode != MODE_TX)
		return true; // Not sending anything (if not an error), so give it yes
    return _tx_done;
}

// Add data for transmission into buffer, but do NOT send yet.
uint8_t Cansat_RFM96::add(char* add_buf, uint8_t add_len) {
	int i;
	for (i=0; i<add_len; i++) {
		if (i+_tx_buf_len >= BUF_SIZE-2)
			return i;
		
		_tx_buf[i+_tx_buf_len+2] = add_buf[i];
	}
	
	_tx_buf_len += add_len;
	
	return add_len;
}

uint8_t Cansat_RFM96::send() {
	uint8_t len_buffer;
	while (_mode == MODE_TX); // Wait for any outgoing message (blocking)
	
	spiWrite(RFM96_REG_0D_FIFO_ADDR_PTR, 0);
	spiBurstWrite(RFM96_REG_00_FIFO, _tx_buf, _tx_buf_len+2); // +2 for ID bytes
	spiWrite(RFM96_REG_22_PAYLOAD_LENGTH, _tx_buf_len+2);
	
	len_buffer = _tx_buf_len+2;
	clear();
	
	setModeTx(); // Mark it for transmission
	
	_is_TX_timeout_on = timeoutTimer.begin(onTimeout, _timeout_time);
	
	return len_buffer; // Return the length of the sent buffer
}

uint8_t Cansat_RFM96::send(char* add_buf, uint8_t add_len) {
	add(add_buf, add_len);
	return send();
}

void Cansat_RFM96::clear() {
	_tx_buf_len = 0;
}

void Cansat_RFM96::printRegisters() {
    uint8_t registers[] = { 0x01, 0x06, 0x07, 0x08, 0x09, 0x0a, 0x0b, 0x0c, 0x0d, 0x0e, 0x0f, 0x10, 0x11, 0x12, 0x13, 0x014, 0x15, 0x16, 0x17, 0x18, 0x19, 0x1a, 0x1b, 0x1c, 0x1d, 0x1e, 0x1f, 0x20, 0x21, 0x22, 0x23, 0x24, 0x25, 0x26, 0x27};
	
	Serial.println("Registers:");
    for (unsigned int i = 0; i < sizeof(registers); i++) {
		Serial.print("0x");
		Serial.print(registers[i], HEX);
		Serial.print(":\t0x");
		Serial.println(spiRead(registers[i]), HEX);
    }
}

unsigned long Cansat_RFM96::numTimeouts() {
	return _timeouts;
}

unsigned long Cansat_RFM96::numTXs() {
	return _valid_TXs;
}

void Cansat_RFM96::end_timer() {
	if (_is_TX_timeout_on)
		timeoutTimer.end();
	_is_TX_timeout_on = false;
}

void Cansat_RFM96::start_timer() {
	if (_is_TX_timeout_on)
		timeoutTimer.end();
	
	if (_use_timeouts)
		_is_TX_timeout_on = timeoutTimer.begin(onTimeout, _timeout_time);
}

void Cansat_RFM96::start_timer(unsigned long timeout) {
	if (_is_TX_timeout_on)
		timeoutTimer.end();
	
	if (_use_timeouts)
		_is_TX_timeout_on = timeoutTimer.begin(onTimeout, timeout);
}

unsigned int Cansat_RFM96::numRestarts() {
	//Serial.println(_rx_timeout_vs_valid);
	return _restarts;
}

void Cansat_RFM96::useTimeouts(bool turn_on) {
	_use_timeouts = turn_on; // False if one wants to turn off
}

void Cansat_RFM96::printIRQFlag() {
	Serial.println(spiRead(RFM96_REG_12_IRQ_FLAGS));
}

void Cansat_RFM96::clearIRQs() {
	spiWrite(0x12, 0xFF);
	spiWrite(0x12, 0xFF);
}

void Cansat_RFM96::printInfo() {
	Serial.print("_rx_timeout_vs_valid=    ");
	Serial.println(_rx_timeout_vs_valid);
	Serial.print("_use_timeouts=           ");
	Serial.println(_use_timeouts);
	
	Serial.print("_timeouts=               ");
	Serial.println(_timeouts);
}

/*
For bw=6 (62.5 kHz)
	cr=4 (4/8)
	sf=8 256 chips/symbol

Time in ms: 110+8*num_bytes
Data rate at
	10 bytes/frame:		
*/
void Cansat_RFM96::setModem(unsigned int bw, unsigned int coding_rate,
	unsigned int spreading_factor) {
	
	if (bw > 9)
		_bw = 9;
	else
		_bw = bw;
	
	if (coding_rate == 0)
		_coding_rate = 1;
	else if (coding_rate > 4) // Not allowing more than 125 kHz in Cansat
		_coding_rate = 4;
	else
		_coding_rate = coding_rate;
	
	if (spreading_factor < 6) // in log2 form (2^6 = 64)
		_spreading_factor = 6;
	else if (spreading_factor > 12)
		_spreading_factor = 12;
	else
		_spreading_factor = spreading_factor;
	
	// everything else should be 0 in these registers, so we do not
	// need and-or on it to keep the other values
	spiWrite(RFM96_REG_1D_MODEM_CONFIG1, (_bw << 4) | (_coding_rate << 1));  
	spiWrite(RFM96_REG_1E_MODEM_CONFIG2, _spreading_factor << 4);
	
	/*
	if (actual_SF()/actual_BW() >= 16.0) { // This is not entirely correct, since it actually is depends on symbol rate being larger than 16 ms, which is also depends on bw(?). But let's just use this for now. See here for more: https://github.com/lupyuen/LoRaArduino and https://www.mouser.com/ds/2/761/sx1276-1278113.pdf It says in the SX1276 datasheet:  Its use is mandated when the symbol duration exceeds 16ms. Note that both the transmitter and the receiver must have the same setting for LowDataRateOptimize.
		_use_lowopt = true;
		spiWrite(RFM96_REG_26_MODEM_CONFIG3, RFM96_LOWDATARATEOPTIMIZE);
#if Cansat_RFM96_DEBUG
		Serial.print("Turning on LowDataRateOptimize, because SF/BW is ");
		Serial.print(actual_SF()/actual_BW());
		Serial.println(" ms (more than 16 ms)");
#endif
	}
	*/
	
	setTimeoutTime(); //_bw*1+_coding_rate*1+spreading_factor*2+100; // in us
}

unsigned long Cansat_RFM96::setTimeoutTime() { // this is in us
	if (_spreading_factor == 6)
		_timeout_time = 41;
	else if (_spreading_factor == 7)
		_timeout_time = 77;
	else if (_spreading_factor == 8)
		_timeout_time = 142;
	else if (_spreading_factor == 9)
		_timeout_time = 265;
	else if (_spreading_factor == 10)
		_timeout_time = 492;
	else if (_spreading_factor == 11)
		_timeout_time = 916;
	else if (_spreading_factor == 12)
		_timeout_time = 1700;
	else
		while(1); // Should never happen
	
	_timeout_time *= (20.+255)*(8.0-_bw)*1000.0/40.0;
	_timeout_time += 100000;
	
	return _timeout_time;
}

// SD functions
bool Cansat_RFM96::open_new_file(void) {
	unsigned long i;
	
	// Could do a padding here, but then must increase padding for 9->99->999...
	for (i=0; i<9999999; i++) {
		sprintf(_sd_filename, "C%07lu.txt", i);
		if (!SD.exists(_sd_filename)) {
			dataFile = SD.open(_sd_filename, FILE_WRITE);
			if (!dataFile) {
				_use_sd = false;
				return false; // Something weird happened
			}
			return true; // Success
		}
	}
	
	_use_sd = false;
	return false; // Could not find a proper filename
		// within 9999999 tries. Aborting...
}

uint8_t Cansat_RFM96::writeToFile(char* _data, uint8_t _data_len) {
	if (_use_sd) {
		uint8_t write_len=0;
		write_len = dataFile.write(_data, _data_len);
		dataFile.flush();
		return(write_len);
	}
	else
		return(0);
}

uint8_t Cansat_RFM96::writeToFile() {
	if (_use_sd) {
		uint8_t write_len=0;
		write_len = dataFile.write((char*)&_tx_buf[2], _tx_buf_len);
		dataFile.flush();
		return(write_len);
	}
	else
		return(0);
}

uint8_t Cansat_RFM96::writeToFile(char dat) {
	if (_use_sd) {
		dataFile.write(dat);
		if (dat == '\n')
			dataFile.flush(); // Super slow, but maybe needed?
		return 1;
	}
	else
		return(0);
}

uint8_t Cansat_RFM96::sendAndWriteToFile() {
	uint8_t tmp_length;
	tmp_length = writeToFile(); // This is always ready,
								// and will also flush every time
	
	if (isTxReady())
		tmp_length = send(); // IF we send, then this is the length we return
	
	clear(); // We empty the buffer and are ready to fill it up again
	return(tmp_length);
}

uint8_t Cansat_RFM96::flush() {
	if (_use_sd) {
		dataFile.flush();
		return(1);
	}
	else
		return(0);
}


//
// Print functions
//

size_t Cansat_RFM96::writeToBuffer(char *data) {
	uint8_t n=0, counter=0;
	while (data[counter++] != '\0')
		n++;
	
	return (size_t)add(data, n);
}

size_t Cansat_RFM96::writeToBuffer(const uint8_t buffer) {
	char buf[1];
	buf[0] = (char)buffer;
	return (size_t)add(buf, 1);
}

size_t Cansat_RFM96::writeToBuffer(const uint8_t *buffer, size_t size) {
	return (size_t)add((char*)buffer, (uint8_t)size);
}

// What does this do? Is it needed?
size_t Cansat_RFM96::printToBuffer(const __FlashStringHelper *ifsh) {
  //PGM_P p = reinterpret_cast<PGM_P>(ifsh);
  size_t n = 0;
  /*
  while (1) {
    unsigned char c = pgm_read_byte(p++);
    if (c == 0) break;
    if (writeToBuffer(c)) n++;
    else break;
  }
  */
  return n;
}

size_t Cansat_RFM96::printToBuffer(const String &s) {
	return add((char*)s.c_str(), s.length());
}

size_t Cansat_RFM96::printToBuffer(const char str[]) {
	return writeToBuffer((char*)str);
}

size_t Cansat_RFM96::printToBuffer(char c) {
	return writeToBuffer(c);
}

size_t Cansat_RFM96::printToBuffer(unsigned char b, int base) {
	return printToBuffer((unsigned long) b, base);
}

size_t Cansat_RFM96::printToBuffer(int n, int base) {
	return printToBuffer((long) n, base);
}

size_t Cansat_RFM96::printToBuffer(unsigned int n, int base) {
	return printToBuffer((unsigned long) n, base);
}

size_t Cansat_RFM96::printToBuffer(long n, int base) {
	if (base == 0) {
		return writeToBuffer(n);
	}
	else if (base == 10) {
		if (n < 0) {
			int t = printToBuffer('-');
			n = -n;
			return printNumber(n, 10) + t;
		}
		return printNumber(n, 10);
	}
	else {
		return printNumber(n, base);
	}
}

size_t Cansat_RFM96::printToBuffer(unsigned long n, int base) {
	if (base == 0)
		return writeToBuffer(n);
	else
		return printNumber(n, base);
}

size_t Cansat_RFM96::printToBuffer(double n, int digits) {
	return printFloat(n, digits);
}

size_t Cansat_RFM96::printlnToBuffer(const __FlashStringHelper *ifsh) {
	size_t n = printToBuffer(ifsh);
	n += printlnToBuffer();
	return n;
}

/*
size_t Cansat_RFM96::print(const Printable& x) {
	size_t n = 0;
	return n;
	//return x.printTo(*this);
}
*/

size_t Cansat_RFM96::printlnToBuffer(void) {
	return writeToBuffer((char*)"\r\n");
}

size_t Cansat_RFM96::printlnToBuffer(const String &s) {
	size_t n = printToBuffer(s);
	n += printlnToBuffer();
	return n;
}

size_t Cansat_RFM96::printlnToBuffer(const char c[]) {
	size_t n = printToBuffer(c);
	n += printlnToBuffer();
	return n;
}

size_t Cansat_RFM96::printlnToBuffer(char c) {
	size_t n = printToBuffer(c);
	n += printlnToBuffer();
	return n;
}

size_t Cansat_RFM96::printlnToBuffer(unsigned char b, int base) {
	size_t n = printToBuffer(b, base);
	n += printlnToBuffer();
	return n;
}

size_t Cansat_RFM96::printlnToBuffer(int num, int base) {
	size_t n = printToBuffer(num, base);
	n += printlnToBuffer();
	return n;
}

size_t Cansat_RFM96::printlnToBuffer(unsigned int num, int base) {
	size_t n = printToBuffer(num, base);
	n += printlnToBuffer();
	return n;
}

size_t Cansat_RFM96::printlnToBuffer(long num, int base) {
	size_t n = printToBuffer(num, base);
	n += printlnToBuffer();
	return n;
}

size_t Cansat_RFM96::printlnToBuffer(unsigned long num, int base) {
	size_t n = printToBuffer(num, base);
	n += printlnToBuffer();
	return n;
}

size_t Cansat_RFM96::printlnToBuffer(double num, int digits) {
	size_t n = printToBuffer(num, digits);
	n += printlnToBuffer();
	return n;
}

/*
size_t Cansat_RFM96::println(const Printable& x)
{
	size_t n = print(x);
	n += println();
	return n;
}
*/

// Private Methods /////////////////////////////////////////////////////////////

size_t Cansat_RFM96::printNumber(unsigned long n, uint8_t base) {
	char buf[8 * sizeof(long) + 1]; // Assumes 8-bit chars plus zero byte.
	char *str = &buf[sizeof(buf) - 1];

	*str = '\0';

	// prevent crash if called with base == 1
	if (base < 2) base = 10;

	do {
		char c = n % base;
		n /= base;

		*--str = c < 10 ? c + '0' : c + 'A' - 10;
	} while(n);

	return writeToBuffer(str);
}

size_t Cansat_RFM96::printFloat(double number, uint8_t digits)  { 
	size_t n = 0;
  
	if (isnan(number)) return printToBuffer("nan");
	if (isinf(number)) return printToBuffer("inf");
	if (number > 4294967040.0) return printToBuffer ("ovf");  // constant determined empirically
	if (number <-4294967040.0) return printToBuffer ("ovf");  // constant determined empirically
  
	// Handle negative numbers
	if (number < 0.0) {
		n += printToBuffer('-');
		number = -number;
	}

	// Round correctly so that print(1.999, 2) prints as "2.00"
	double rounding = 0.5;
	for (uint8_t i=0; i<digits; ++i)
		rounding /= 10.0;
  
	number += rounding;

	// Extract the integer part of the number and print it
	unsigned long int_part = (unsigned long)number;
	double remainder = number - (double)int_part;
	n += printToBuffer(int_part);

	// Print the decimal point, but only if there are digits beyond
	if (digits > 0) {
		n += printToBuffer('.'); 
	}

	// Extract digits from the remainder one at a time
	while (digits-- > 0) {
		remainder *= 10.0;
		unsigned long toPrint = (unsigned long)(remainder);
		n += printToBuffer(toPrint);
		remainder -= toPrint; 
	} 
  
	return n;
}

void Cansat_RFM96::printVariables() {
	Serial.print("_cs:\t\t");
	Serial.print(_cs);
	Serial.print("\t 0x");
	Serial.println(_cs, HEX);
	
	Serial.print("_int:\t\t");
	Serial.print(_int);
	Serial.print("\t 0x");
	Serial.println(_int, HEX);
	
	Serial.print("_rst:\t\t");
	Serial.print(_rst);
	Serial.print("\t 0x");
	Serial.println(_rst, HEX);
	
	Serial.print("_userID:\t\t");
	Serial.print(_userID);
	Serial.print("\t 0x");
	Serial.println(_userID, HEX);
	
	Serial.print("_use_sd:\t\t");
	Serial.print(_use_sd);
	Serial.print("\t 0x");
	Serial.println(_use_sd, HEX);
	
	Serial.print("_init_sd:\t\t");
	Serial.print(_init_sd);
	Serial.print("\t 0x");
	Serial.println(_init_sd, HEX);
	
	Serial.print("_mode:\t\t");
	Serial.print(_mode);
	Serial.print("\t 0x");
	Serial.println(_mode, HEX);
	
	Serial.print("_timeouts:\t\t");
	Serial.print(_timeouts);
	Serial.print("\t 0x");
	Serial.println(_timeouts, HEX);
	
	Serial.print("_freq:\t\t");
	Serial.print(_freq);
	Serial.print("\t 0x");
	Serial.println(_freq, HEX);
	
	Serial.print("_tx_power:\t\t");
	Serial.print(_tx_power);
	Serial.print("\t 0x");
	Serial.println(_tx_power, HEX);
	
	Serial.println();
}



