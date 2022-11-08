/*
Copyright (C) Bill2462 from https://github.com/Bill2462
 This program is free software; you can redistribute it and/or
 modify it under the terms of the GNU General Public License
 version 3 as published by the Free Software Foundation.
*/

#include "Arduino.h"
#include "GY91.h"

GY91::GY91(uint8_t mpu_address, uint8_t mag_address, uint8_t bmp_address) {
	mpu_adr = mpu_address;
	mag_adr = mag_address;
	bmp_adr = bmp_address;
}

/* Initialize the chip
Arguments: None
Returns : None
*/
bool GY91::init() {
	Wire.begin();// enable I2C interface
	
	if ((mpu_adr != 0) || (mag_adr != 0))
		using_mpumag = true;
	else
		using_mpumag = false;
	
	if (bmp_adr != 0)
		using_bmp = true;
	else
		using_bmp = false;
	
	if (using_mpumag) {
		Hreset();//reset the chip
		
		acc_scale = 16384;
		gyro_scale = 131;
		magneto_scale = 4912.0/32760; // What is this - check the datasheet
		
		delay(10); // Wait for reset
		uint8_t mpu_who_am_i = read(mpu_adr, 0x75);
		//Serial.print("WHO_AM_I=0x"); Serial.println(mpu_who_am_i, HEX);
		if ((mpu_who_am_i != 0x71) && (mpu_who_am_i != 0x73)) // To support both MPU-9250 and 9255
			return false;
		
		write(mpu_adr,CONFIG, 0x03); // set DLPF_CFG to 11
		write(mpu_adr,SMPLRT_DIV, 0x04);// set prescaler sample rate to 4
		write(mpu_adr,GYRO_CONFIG, 0x01);// set gyro to 3.6 KHz bandwidth, and 0.11 ms using FCHOICE bits
		write(mpu_adr,INT_PIN_CFG, 0x02);// BYPASS ENABLE (Necessary for the magnetometer to function)
		write(mag_adr, CNTL, 0x16);//set magnetometer to read in mode 2 and enable 16 bit measurements
		
	}
		
	if (using_bmp) {
		if (read8(BMP280_REGISTER_CHIPID) != 0x58) // Sort of a who_am_i?
			return false;

		readCoefficients();
		write8(BMP280_REGISTER_CONTROL, 0x3F);
	}

	return true;
}


/* Read one byte of data from the sensor
Arguments:
- address - address of the device
- subAddress - address of the register
Returns : Contents of the readed register (one byte)
*/
uint8_t GY91::read(uint8_t address, uint8_t subAddress) {
	uint8_t data;
	Wire.beginTransmission(address);
	Wire.write(subAddress);
	Wire.endTransmission(false);
	Wire.requestFrom(address, (uint8_t) 1);
	data = Wire.read();
	return data;
}

/* Write one byte to the register
Arguments:
- address - address of the device
- subAddress - address of the register
- data - one byte of data that we want to put in the register
Returns : None
*/
void GY91::write(uint8_t address, uint8_t subAddress, uint8_t data) {
	Wire.beginTransmission(address);
	Wire.write(subAddress);
	Wire.write(data);
	Wire.endTransmission();
}

/* Perform OR operation on a register state and a data byte and write
result into the register.
t
Arguments:
- address - address of the device
- subAddress - address of the register
- data - one byte of data that we want to put in the register
Returns : None
*/
void GY91::write_OR(uint8_t address, uint8_t subAddress, uint8_t data) {
	uint8_t c = read(address,subAddress);
	c = c | data;
	write(address,subAddress,c);
}

/* Perform AND operation on a register state and a data byte and write
result into the register.
Arguments:
- address - address of the device
- subAddress - address of the register
- data - one byte of data that we want to put in the register
Returns : None
*/
void GY91::write_AND(uint8_t address, uint8_t subAddress, uint8_t data) {
	uint8_t c = read(address,subAddress);
	c = c & data;
	write(address,subAddress,c);
}

/* Set accelerometer scale
Arguments:
- Selected scale
Returns : None
*/
void GY91::set_acc_scale(unsigned char value) {
	uint8_t val = read(mpu_adr,ACCEL_CONFIG);//read old register value
	switch(value) {
		case 2:// +- 2g
		val &= ~((1<<3)|(1<<4));
		acc_scale = 16384.0;
		break;

		case 4:// +- 4g
		val &= ~(1<<4);
		val |= (1<<3);
		acc_scale = 8192.0;
		break;

		case 8:// +- 8g
		val &= ~(1<<3);
		val |= (1<<4);
		acc_scale = 4096.0;
		break;

		case 16:// +- 16g
		val |= (1<<4)|(1<<3);
		acc_scale = 2048.0;
		break;
	}
	write(mpu_adr,ACCEL_CONFIG,val);// commit changes
}

/* Set gyroscope scale
Arguments:
- Selected scale
Returns : None
*/
void GY91::set_gyro_scale(unsigned int value) {
	uint8_t val=read(mpu_adr,GYRO_CONFIG);//read old register value
	switch(value) {
		case 250:// +- 250 dps
		val &= ~((1<<3)|(1<<4));
		gyro_scale = 131;
		break;

		case 500:// +- 500 dps
		val &= ~(1<<4);
		val |= (1<<3);
		gyro_scale = 65.5;
		break;

		case 1000:// +- 1000 dps
		val &= ~(1<<3);
		val |= (1<<4);
		gyro_scale = 32.8;
		break;

		case 2000:// +- 2000 dps
		val |= (1<<4)|(1<<3);
		gyro_scale = 16.4;
		break;
	}
	write(mpu_adr,GYRO_CONFIG,val);// commit changes
}

/* Read data from the accelerometer
Arguments: None
Returns : None
*/
void GY91::read_acc() {
	//request data
	Wire.beginTransmission((int)mpu_adr);//begin transmission to the sensor
	Wire.write(ACCEL_XOUT_H);//write adress of the accelerometer data register to signalize that we want to read it
	Wire.endTransmission(false);//end transmission
	Wire.requestFrom((int)mpu_adr, 6);//request 6 bytes of data from the sensor

	//read data
	uint8_t i = 0;//index
	uint8_t rawData[6];//bufor
	while (Wire.available()) { //loop throught all received bytes
		rawData[i++] = Wire.read();//read byte and put it into rawData table
	}

	//dump reading into output variables
	ax_raw = ((int16_t)rawData[0] << 8) | rawData[1];
	ay_raw = ((int16_t)rawData[2] << 8) | rawData[3];
	az_raw = ((int16_t)rawData[4] << 8) | rawData[5];
  
	ax = ax_raw/acc_scale;
	ay = ay_raw/acc_scale;
	az = az_raw/acc_scale;
}

/* Read data from the gyroscope
Arguments: None
Returns : None
*/
void GY91::read_gyro() {
	Wire.beginTransmission((int)mpu_adr);//begin transmission to the sensor
	Wire.write(GYRO_XOUT_H);//write adress of the accelerometer data register to signalize that we want to read it
	Wire.endTransmission(false);//end transmission
	Wire.requestFrom((int)mpu_adr, 6);//request 6 bytes of data from the sensor

	uint8_t rawData[6];//bufor
	uint8_t i = 0;//index
	while (Wire.available()) { //loop throught all received bytes
		rawData[i++] = Wire.read();//read byte and put it into rawData table
	}

	//dump reading into output variables
	gx_raw = ((int16_t)rawData[0] << 8) | rawData[1];
	gy_raw = ((int16_t)rawData[2] << 8) | rawData[3];
	gz_raw = ((int16_t)rawData[4] << 8) | rawData[5];
  
	gx = gx_raw/gyro_scale;
	gy = gy_raw/gyro_scale;
	gz = gz_raw/gyro_scale;
}

/* Read data from the magnetometer
Arguments: None
Returns : None
*/
void GY91::read_mag() {
	Wire.beginTransmission((int)mag_adr);//begin transmission to the sensor
	Wire.write(MAG_XOUT_L);//write adress of the accelerometer data register to signalize that we want to read it
	Wire.endTransmission(false);//end transmission
	Wire.requestFrom((int)mag_adr, 8);//request 6 bytes of data from the sensor

	uint8_t rawData[6];//bufor
	uint8_t i = 0;//index
	while (Wire.available()) { //loop throught all received bytes
		rawData[i++] = Wire.read();//read byte and put it into rawData table
	}

	//dump reading into output variables
	mx_raw = (int16_t)((rawData[1] << 8) | rawData[0]);
	my_raw = (int16_t)((rawData[3] << 8) | rawData[2]);
	mz_raw = (int16_t)((rawData[5] << 8) | rawData[4]);
  
	mx = mx_raw*magneto_scale;
	my = my_raw*magneto_scale;
	mz = mz_raw*magneto_scale;
}

void GY91::read_pressure() { // a small wrapper function
	pressure = readPressure();
}

void GY91::read_gy91() { // a wrapper function to load in all the data at once
	read_acc();
	read_gyro();
	read_mag();
	read_pressure();
}

int16_t GY91::read_temp_mpumag() {
	Wire.beginTransmission((int)mpu_adr);//begin transmission to the sensor
	Wire.write(TEMP_OUT_H);//write adress of the thermometer data register to signalize that we want to read it
	Wire.endTransmission(false);//end transmission
	Wire.requestFrom((int)mpu_adr, 2);//request 2 bytes of data from the sensor

	uint8_t rawData[2];//bufor
	uint8_t i = 0;//index
	while (Wire.available()) { //loop throught all received bytes
	rawData[i++] = Wire.read();//read byte and put it into rawData table
	}

	int16_t temp=0;
	temp = ((int16_t)rawData[0] << 8) | rawData[1];//put together the output value
	return temp;//return raw data
}

/* Perform hard reset (basically reset everything). Call of init() function is required to use sensor afterwards
Arguments: None
Returns : None
*/
void GY91::Hreset() {
	write(mpu_adr,PWR_MGMT_1, 0x80);//write 1 to the hard reset bit in PWR_MGMT_1 register
}

/* Reset signal patch of the gyroscope
Arguments: None
Returns : None
*/
void GY91::gyro_RST() {
	write(mpu_adr,SIGNAL_PATH_RESET, 0x04);
}

/* Reset signal patch of the accelerometer
Arguments: None
Returns : None
*/
void GY91::acc_RST() {
	write(mpu_adr,SIGNAL_PATH_RESET, 0x02);
}

/* Reset signal patch of the thermometer
Arguments: None
Returns : None
*/
void GY91::temp_RST() {
	write(mpu_adr,SIGNAL_PATH_RESET, 0x01);
}

/* Reset accelerometer, gyroscope and thermometer sygnal patches and registers
Arguments: None
Returns : None
*/
void GY91::SIG_COND_RST() {
	uint8_t c = read(mpu_adr,USER_CTRL);//read old register value
	c = c | 0x01;//set bit 0 to 1
	write(mpu_adr,USER_CTRL, c);//commit changes
}

/* Perform software reset of the magnetometer
Arguments: None
Returns : None
*/
void GY91::mag_SoftRST() {
	write(mag_adr,CNTL2, 0x01);
}

/* Power down the magnetometer
Arguments: None
Returns : None
*/
void GY91::mag_PWRD() {
	write(mag_adr,CNTL, 0x00);
}

/* Power up magnetometer
Arguments: None
Returns : None
*/
void GY91::mag_PWRU() {
	write(mag_adr,CNTL, 0x16);
}

/* Put main chip in a sleep mode
Arguments: None
Returns : None
*/
void GY91::sleep_enable() {
	write(mpu_adr,PWR_MGMT_1, 0x40);//set sleep bit to 1
}

/* disable sleep mode
Arguments: None
Returns : None
*/
void GY91::sleep_disable() {
	write(mpu_adr,PWR_MGMT_1, 0x00);//clear the sleep bit
	delay(500);//wait until module stabilizes
}

/* disable accelerometer X axis
Arguments: None
Returns : None
*/
void GY91::disable_ax() {
	write_OR(mpu_adr,PWR_MGMT_2, 0x20);
}

/* disable accelerometer Y axis
Arguments: None
Returns : None
*/
void GY91::disable_ay() {
	write_OR(mpu_adr,PWR_MGMT_2, 0x10);
}

/* disable accelerometer Z axis
Arguments: None
Returns : None
*/
void GY91::disable_az() {
	write_OR(mpu_adr,PWR_MGMT_2, 0x08);
}

/* disable gyroscope X axis
Arguments: None
Returns : None
*/
void GY91::disable_gx() {
	write_OR(mpu_adr,PWR_MGMT_2, 0x04);
}

/* disable gyroscope Y axis
Arguments: None
Returns : None
*/
void GY91::disable_gy() {
	write_OR(mpu_adr,PWR_MGMT_2, 0x02);
}

/* disable gyroscope Z axis
Arguments: None
Returns : None
*/
void GY91::disable_gz() {
	write_OR(mpu_adr,PWR_MGMT_2, 0x01);
}

/* enable accelerometer X axis
Arguments: None
Returns : None
*/
void GY91::enable_ax() {
	write_AND(mpu_adr,PWR_MGMT_2, ~0x20);
}

/* enable accelerometer Y axis
Arguments: None
Returns : None
*/
void GY91::enable_ay() {
	write_AND(mpu_adr,PWR_MGMT_2, ~0x10);
}

/* enable accelerometer Y axis
Arguments: None
Returns : None
*/
void GY91::enable_az() {
	write_AND(mpu_adr,PWR_MGMT_2, ~0x8);
}

/* enable accelerometer Z axis
Arguments: None
Returns : None
*/
void GY91::enable_gx() {
	write_AND(mpu_adr,PWR_MGMT_2, ~0x04);
}

/* enable gyroscope Y axis
Arguments: None
Returns : None
*/
void GY91::enable_gy() {
	write_AND(mpu_adr,PWR_MGMT_2, ~0x02);
}

/* enable gyroscope Z axis
Arguments: None
Returns : None
*/
void GY91::enable_gz() {
	write_AND(mpu_adr,PWR_MGMT_2, ~0x01);
}


/*
	This is for the BMP280 sensor, the library is almost
	fully taken from Adafruit's library, but somewhat
	shortened as we only use I2C.
*/

void GY91::write8(byte reg, byte value) {
  Wire.beginTransmission((uint8_t)bmp_adr);
  Wire.write((uint8_t)reg);
  Wire.write((uint8_t)value);
  Wire.endTransmission();
}

uint8_t GY91::read8(byte reg) {
  uint8_t value;

  Wire.beginTransmission((uint8_t)bmp_adr);
  Wire.write((uint8_t)reg);
  Wire.endTransmission();
  Wire.requestFrom((uint8_t)bmp_adr, (byte)1);
  value = Wire.read();

  return value;
}

uint16_t GY91::read16(byte reg) {
  uint16_t value;

  Wire.beginTransmission((uint8_t)bmp_adr);
  Wire.write((uint8_t)reg);
  Wire.endTransmission();
  Wire.requestFrom((uint8_t)bmp_adr, (byte)2);
  value = (Wire.read() << 8) | Wire.read();

  return value;
}

uint16_t GY91::read16_LE(byte reg) {
  uint16_t temp = read16(reg);
  return (temp >> 8) | (temp << 8);

}

int16_t GY91::readS16(byte reg) {
  return (int16_t)read16(reg);

}

int16_t GY91::readS16_LE(byte reg) {
  return (int16_t)read16_LE(reg);

}

uint32_t GY91::read24(byte reg) {
  uint32_t value;

  Wire.beginTransmission((uint8_t)bmp_adr);
  Wire.write((uint8_t)reg);
  Wire.endTransmission();
  Wire.requestFrom((uint8_t)bmp_adr, (byte)3);
    
  value = Wire.read();
  value <<= 8;
  value |= Wire.read();
  value <<= 8;
  value |= Wire.read();

  return value;
}

void GY91::readCoefficients(void)
{
    dig_T1 = read16_LE(BMP280_REGISTER_DIG_T1);
    dig_T2 = readS16_LE(BMP280_REGISTER_DIG_T2);
    dig_T3 = readS16_LE(BMP280_REGISTER_DIG_T3);

    dig_P1 = read16_LE(BMP280_REGISTER_DIG_P1);
    dig_P2 = readS16_LE(BMP280_REGISTER_DIG_P2);
    dig_P3 = readS16_LE(BMP280_REGISTER_DIG_P3);
    dig_P4 = readS16_LE(BMP280_REGISTER_DIG_P4);
    dig_P5 = readS16_LE(BMP280_REGISTER_DIG_P5);
    dig_P6 = readS16_LE(BMP280_REGISTER_DIG_P6);
    dig_P7 = readS16_LE(BMP280_REGISTER_DIG_P7);
    dig_P8 = readS16_LE(BMP280_REGISTER_DIG_P8);
    dig_P9 = readS16_LE(BMP280_REGISTER_DIG_P9);
}

double GY91::readTemperature(void)
{
  int32_t var1, var2;

  int32_t adc_T = read24(BMP280_REGISTER_TEMPDATA);
  adc_T >>= 4;

  var1  = ((((adc_T>>3) - ((int32_t)dig_T1 <<1))) *
	   ((int32_t)dig_T2)) >> 11;

  var2  = (((((adc_T>>4) - ((int32_t)dig_T1)) *
	     ((adc_T>>4) - ((int32_t)dig_T1))) >> 12) *
	   ((int32_t)dig_T3)) >> 14;

  t_fine = var1 + var2;

  double T  = (t_fine * 5 + 128) >> 8;
  return T/100.0;
}

double GY91::readPressure(void) {
  int64_t var1, var2, p;

  // Must be done first to get the t_fine variable set up
  readTemperature();

  int32_t adc_P = read24(BMP280_REGISTER_PRESSUREDATA);
  adc_P >>= 4;

  var1 = ((int64_t)t_fine) - 128000;
  var2 = var1 * var1 * (int64_t)dig_P6;
  var2 = var2 + ((var1*(int64_t)dig_P5)<<17);
  var2 = var2 + (((int64_t)dig_P4)<<35);
  var1 = ((var1 * var1 * (int64_t)dig_P3)>>8) +
    ((var1 * (int64_t)dig_P2)<<12);
  var1 = (((((int64_t)1)<<47)+var1))*((int64_t)dig_P1)>>33;

  if (var1 == 0) {
    return 0;  // avoid exception caused by division by zero
  }
  p = 1048576 - adc_P;
  p = (((p<<31) - var2)*3125) / var1;
  var1 = (((int64_t)dig_P9) * (p>>13) * (p>>13)) >> 25;
  var2 = (((int64_t)dig_P8) * p) >> 19;

  p = ((p + var1 + var2) >> 8) + (((int64_t)dig_P7)<<4);
  return (double)p/256.0;
}