//
// Created by Regner on 28/04/19.
//

#pragma once

#include <Arduino.h>
#include <Wire.h>


#define MPU6050_I2C_ADDRESS 0x68
#define SAMPLING_FREQ  100.0 // sample SAMPLING_FREQ in Hz


void setup_imu();

void imu();

void calibrate();

void read_sensor_data();

int i2c_read(int addr, int start, uint8_t *buffer, int size);

int i2c_write(int addr, int start, const uint8_t *pData, int size);

int i2c_write_reg(int addr, int reg, uint8_t data);