/*
Copyright (C) Bill2462 from https://github.com/Bill2462
 This program is free software; you can redistribute it and/or
 modify it under the terms of the GNU General Public License
 version 3 as published by the Free Software Foundation.
*/

#ifndef GY91_H
#define GY91_H

#include <Arduino.h>
#include <Wire.h>

//sensor adresses
#define MPU_ADDRESS 0x68//main chip
#define MAG_ADDRESS 0x0C//magnetometer
#define BMP_ADDRESS 0x76//pressure

//main chip
#define USER_CTRL  0x6A
#define PWR_MGMT_1 0x6B
#define PWR_MGMT_2 0x6C
#define SIGNAL_PATH_RESET 0x68
#define INT_PIN_CFG 0x37
#define ST1 0x02
#define ACCEL_CONFIG 0x1C
#define GYRO_CONFIG 0x1B
#define CONFIG 0x1A
#define SMPLRT_DIV 0x19
#define INT_ENABLE 0x38

//magnetometer
#define CNTL 0x0A
#define CNTL2 0x0B

#define BMP280_REGISTER_DIG_T1             0x88
#define BMP280_REGISTER_DIG_T2             0x8A
#define BMP280_REGISTER_DIG_T3             0x8C
#define BMP280_REGISTER_DIG_P1             0x8E
#define BMP280_REGISTER_DIG_P2             0x90
#define BMP280_REGISTER_DIG_P3             0x92
#define BMP280_REGISTER_DIG_P4             0x94
#define BMP280_REGISTER_DIG_P5             0x96
#define BMP280_REGISTER_DIG_P6             0x98
#define BMP280_REGISTER_DIG_P7             0x9A
#define BMP280_REGISTER_DIG_P8             0x9C
#define BMP280_REGISTER_DIG_P9             0x9E
#define BMP280_REGISTER_CHIPID             0xD0
#define BMP280_REGISTER_VERSION            0xD1
#define BMP280_REGISTER_SOFTRESET          0xE0
#define BMP280_REGISTER_CAL26              0xE1  // R calibration stored in 0xE1-0xF0
#define BMP280_REGISTER_CONTROL            0xF4
#define BMP280_REGISTER_CONFIG             0xF5
#define BMP280_REGISTER_PRESSUREDATA       0xF7
#define BMP280_REGISTER_TEMPDATA           0xFA

/// data registers
#define MAG_XOUT_L							0x03//magnetometer
#define GYRO_XOUT_H							0x43//gyro
#define ACCEL_XOUT_H						0x3B//accelerometer
#define TEMP_OUT_H							0x41//thermometer

class GY91
{
public:	
	//acceleration data
	int16_t ax_raw=0;//X axis
	int16_t ay_raw=0;//Y axis
	int16_t az_raw=0;//Z axis
	double ax=0;//X axis
	double ay=0;//Y axis
	double az=0;//Z axis

	//gyroscope data
	int16_t gx_raw=0;//X axis
	int16_t gy_raw=0;//Y axis
	int16_t gz_raw=0;//Z axis
	double gx=0;//X axis
	double gy=0;//Y axis
	double gz=0;//Z axis

	//magnetometer data
	int16_t mx_raw=0;//X axis
	int16_t my_raw=0;//Y axis
	int16_t mz_raw=0;//Z axis
	double mx=0;//X axis
	double my=0;//Y axis
	double mz=0;//Z axis
	
	double pressure=0;//pressure

	double acc_scale=0;
	double gyro_scale=0;
	double magneto_scale=0;
	
	// BMP280 calibration data
	uint16_t dig_T1;
    int16_t  dig_T2;
    int16_t  dig_T3;
    uint16_t dig_P1;
    int16_t  dig_P2;
    int16_t  dig_P3;
    int16_t  dig_P4;
    int16_t  dig_P5;
    int16_t  dig_P6;
    int16_t  dig_P7;
    int16_t  dig_P8;
    int16_t  dig_P9;

    uint8_t  dig_H1;
    int16_t  dig_H2;
    uint8_t  dig_H3;
    int16_t  dig_H4;
    int16_t  dig_H5;
    int8_t   dig_H6;
	
	//general control
	GY91(uint8_t mpu_address = MPU_ADDRESS, uint8_t mag_address = MAG_ADDRESS, uint8_t bmp_address = BMP_ADDRESS); // Constructor
	bool init();//Initialize magnetometer and IMU
	void set_acc_scale(unsigned char value);//Set accelerometer scale
	void set_gyro_scale(unsigned int value);//Set gyroscope scale

	//reset
	void Hreset();//Hard reset - Resets entire chip (call of init function is required to use chip afterwards)
	void gyro_RST();//reset gyroscope signal patch (registers values are preserved)
	void acc_RST();//reset accelerometer signal patch (registers values are preserved)
	void temp_RST();//reset termometer signal patch (registers values are preserved)
	void SIG_COND_RST();//reset all signal patchs and sensor registers in accelerometer gyroscope and termometer
	void mag_SoftRST();//reset Magnetometer

	//data read function
	void read_acc();//read data from accelerometer
	void read_gyro();//read data from gyroscope
	void read_mag();//read data from magnetometer
	void read_pressure();//read data from pressure
	void read_gy91(); // read all
	int16_t read_temp_mpumag();//read temperature from the internal sensor (EXPERIMENTAL!)

	//power control
	void mag_PWRD();//magnetometer powerdown
	void mag_PWRU();//magnetometer powerup
	void sleep_enable();//put main chip in sleep mode
	void sleep_disable();//disable sleep mode
	void disable_ax();//disable accelerometer X axis
	void disable_ay();//disable accelerometer Y axis
	void disable_az();//disable accelerometer Z axis
	void disable_gx();//disable gyroscope X axis
	void disable_gy();//disable gyroscope Y axis
	void disable_gz();//disable gyroscope Z axis
	void enable_ax();//enable accelerometer X axis
	void enable_ay();//enable accelerometer Y axis
	void enable_az();//enable accelerometer Z axis
	void enable_gx();//enable gyroscope X axis
	void enable_gy();//enable gyroscope Y axis
	void enable_gz();//enable gyroscope Z axis
	
	// For BMP280
	double readTemperature(void);
    double readPressure(void);


	private:
	uint8_t mpu_adr;
	uint8_t mag_adr;
	uint8_t bmp_adr;
	bool	using_mpumag;
	bool	using_bmp;
	
	void readCoefficients(void); // For BMP280

    void      write8(byte reg, byte value);
    uint8_t   read8(byte reg);
    uint16_t  read16(byte reg);
    uint32_t  read24(byte reg);
    int16_t   readS16(byte reg);
    uint16_t  read16_LE(byte reg); // little endian
    int16_t   readS16_LE(byte reg); // little endian
	int32_t   _sensorID;
    int32_t t_fine;
	
	uint8_t read(uint8_t address, uint8_t subAddress);//read one byte from register
	void write(uint8_t address, uint8_t subAddress, uint8_t data);//write one byte of data to the register
	void write_OR(uint8_t address, uint8_t subAddress, uint8_t data);//write one byte of data to the register (with OR operation)
	void write_AND(uint8_t address, uint8_t subAddress, uint8_t data);//write one byte of data to the register (with AND operation)
};

#endif
