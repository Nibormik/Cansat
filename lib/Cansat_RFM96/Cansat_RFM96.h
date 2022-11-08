#ifndef Cansat_RFM96_h
#define Cansat_RFM96_h

#include "Arduino.h"
#include <SPI.h>
#include <SD.h>

#define Cansat_RFM96_RST								26
#define Cansat_RFM96_INT								25
#define Cansat_RFM96_CS									24
#define SPI_FREQ										8000000

#define Cansat_RFM96_DEBUG								0
#define BUF_SIZE										257

#define SPI_WRITE_MASK									0x80
#define RFM96_REG_0D_FIFO_ADDR_PTR						0x0D
#define RFM96_REG_00_FIFO								0x00
#define RFM96_REG_22_PAYLOAD_LENGTH						0x22
#define RFM96_REG_01_OP_MODE							0x01
#define RFM96_MODE_TX									0x03
#define RFM96_REG_40_DIO_MAPPING1						0x40
#define RFM96_MODE_STDBY								0x01
#define RFM96_MODE_SLEEP								0x00
#define RFM96_LONG_RANGE_MODE							0x80
#define RFM96_REG_0E_FIFO_TX_BASE_ADDR					0x0E
#define RFM96_REG_0F_FIFO_RX_BASE_ADDR					0x0F
#define RFM96_REG_1D_MODEM_CONFIG1						0x1D
#define RFM96_REG_1E_MODEM_CONFIG2						0x1E
#define RFM96_REG_26_MODEM_CONFIG3						0x26
#define RFM96_LOWDATARATEOPTIMIZE						0x10
#define RFM96_REG_20_PREAMBLE_MSB						0x20
#define RFM96_REG_21_PREAMBLE_LSB						0x21
#define RFM96_REG_4D_PA_DAC								0x4D
#define RFM96_PA_DAC_DISABLE							0x04
#define RFM96_PA_DAC_ENABLE								0x07
#define RFM96_REG_09_PA_CONFIG							0x09
#define RFM96_PA_SELECT									0x80
#define RFM96_MODE_RXCONTINUOUS							0x05
#define RFM96_MODE_RXSINGLE								0x06
#define RFM96_REG_13_RX_NB_BYTES						0x13
#define RFM96_REG_10_FIFO_RX_CURRENT_ADDR				0x10
#define RFM96_REG_1A_PKT_RSSI_VALUE						0x1A
#define RFM96_REG_06_FRF_MSB							0x06
#define RFM96_REG_07_FRF_MID							0x07
#define RFM96_REG_08_FRF_LSB							0x08
#define RFM96_PA_SELECT									0x80
#define RFM96_MAX_POWER									0x70
#define RFM96_OUTPUT_POWER								0x0f
#define RFM96_REG_11_IRQ_FLAGS_MASK						0x11
#define RFM96_REG_12_IRQ_FLAGS							0x12
#define RFM96_REG_0C_LNA								0x0C
#define RFM96_LNA_GAIN									0x20
#define RFM96_LNA_BOOST_HF								0x03

#define MODE_SLEEP										0x00
#define MODE_STDBY										0x01
#define MODE_TX											0x03
#define MODE_RX_CONT									0x05

#define RFM96_FXOSC										32000000.0
#define RFM96_FSTEP										(RFM96_FXOSC / 524288)

class Cansat_RFM96 {
	public:
		Cansat_RFM96(unsigned long freq_kHz, bool = true, int = Cansat_RFM96_CS, int = Cansat_RFM96_INT, int = Cansat_RFM96_RST);
		bool					init();
		void 					spiBurstWrite(uint8_t adr, uint8_t* data, uint8_t len);
		void 					spiWrite(uint8_t adr, uint8_t data);
		uint8_t 				spiRead(uint8_t adr);
		void 					spiBurstRead(uint8_t adr, uint8_t* dest, uint8_t len);
		uint8_t					available();
		int						last_RSSI();
		uint8_t					read(char* dest);
		char					read(void);
		void					setFrequency(float centre);
		void					setTxPower(int8_t power);
		uint8_t					add(char* add_buf, uint8_t add_len);
		uint8_t					send();
		uint8_t					send(char* add_buf, uint8_t add_len);
		void					clear();
		bool					isTxReady();
		void					printRegisters();
		void					setModem(unsigned int bw, unsigned int coding_rate,		unsigned int spreading_factor);
		unsigned int			actual_SF();
		double 					actual_BW();
		
		void					setModeStdby();
		void					setModeSleep();
		void					setModeRx();
		void					setModeTx();
		uint8_t					whatMode();
		void					useTimeouts(bool turn_on);
		void					printIRQFlag();
		void					clearIRQs();
		void					printInfo();
		unsigned long			setTimeoutTime();
		
		unsigned long			numTimeouts();
		unsigned long			numTXs();
		unsigned int			numRestarts();
		
		unsigned long 			tid;
		
		// SD functions
		uint8_t					writeToFile(char*, uint8_t);
		uint8_t					writeToFile();
		uint8_t					writeToFile(char);
		uint8_t					sendAndWriteToFile();
		uint8_t					flush();
		
		// Print functions
		size_t					writeToBuffer(char *);
		size_t					writeToBuffer(const uint8_t);
		size_t					writeToBuffer(const uint8_t*, size_t);
		size_t					printToBuffer(const __FlashStringHelper *ifsh);
		size_t					printToBuffer(const String &s);
		//size_t					print(char* str); //char* str
		size_t					printToBuffer(const char str[]);
		size_t					printToBuffer(char);
		size_t					printToBuffer(unsigned char, int = DEC);
		size_t					printToBuffer(int, int = DEC);
		size_t					printToBuffer(unsigned int, int = DEC);
		size_t					printToBuffer(long, int = DEC);
		size_t					printToBuffer(unsigned long, int = DEC);
		size_t					printToBuffer(double, int = 2);
		//size_t					print(unsigned long n);
		//size_t					print(long n);
		size_t					printlnToBuffer(const __FlashStringHelper *ifsh);
		//size_t					print(const Printable& x);
		size_t					printlnToBuffer(void);
		size_t					printlnToBuffer(const String &s);
		size_t					printlnToBuffer(const char c[]);
		size_t					printlnToBuffer(char);
		size_t					printlnToBuffer(unsigned char, int = DEC);
		size_t					printlnToBuffer(int, int = DEC);
		size_t					printlnToBuffer(unsigned int, int = DEC);
		size_t					printlnToBuffer(long, int = DEC);
		size_t					printlnToBuffer(unsigned long, int = DEC);
		size_t					printlnToBuffer(double, int = 2);
		//size_t					println(const Printable& x);
		void 					printVariables();
		
		
	protected:
		void 					handleInterrupt();
		void					handleTimeoutInterrupt();
	private:
		int 					_cs;
		int 					_int;
		int 					_rst;
		unsigned int			_userID;
		
		bool					_use_sd;
		bool					_init_sd;
		char					_sd_filename[8+1+3+1];
		File					dataFile;
		
		uint8_t					_buf[BUF_SIZE];
		uint8_t					_tx_buf[BUF_SIZE];
		uint8_t					_buf_ID[2];
		uint8_t					_buf_len;
		uint8_t					_buf_pos;
		uint8_t					_tx_buf_len;
		int						_last_RSSI;
		bool					_avail;
		bool					_tx_done;
		uint8_t					_mode;
		volatile bool			_is_TX_timeout_on;
		volatile unsigned long	_timeouts;
		volatile unsigned long	_valid_TXs;
		volatile unsigned int	_tx_timeout_vs_valid;
		volatile unsigned int	_rx_timeout_vs_valid;
		volatile unsigned int	_restarts;
		bool					_use_timeouts;
		float					_freq;
		int8_t					_tx_power;
		bool					_use_lowopt;
		
		SPISettings SPI_settings;
		
		static Cansat_RFM96*	_deviceForInterrupt;
		static Cansat_RFM96*	_deviceForTimerInterrupt;
		IntervalTimer			timeoutTimer;
		unsigned long			_timeout_time;
		
		static void				onPulse();
		static void				onTimeout();
		void					restart();
		void					end_timer();
		void					start_timer();
		void					start_timer(unsigned long timeout);
		
		unsigned int			_bw;
		unsigned int			_coding_rate;
		unsigned int			_spreading_factor;
		
		size_t					printNumber(unsigned long n, uint8_t base);
		size_t					printFloat(double number, uint8_t digits);
		
		bool					open_new_file(void);
};

#endif