/**
* Universidade Federal de Santa Catarina
* Departamento de Automação e Sistemas - CTC
*
* Biblioteca mpu6050 / Controle Multivariável
* Autor: Alex Amadeu Cani
* Jan 2018
**/

#include "Mpu6050.h"

#include <math.h>
#include <Wire.h>

Mpu6050::Mpu6050()
 : filter_pos_x_{kCutFrequency, kQualityFactor}, filter_pos_y_{kCutFrequency, kQualityFactor},
   filter_speed_x_{kCutFrequency, kQualityFactor}, filter_speed_y_{kCutFrequency, kQualityFactor},
   filter_speed_z_{kCutFrequency, kQualityFactor}, x_speed_offset_(0.0), y_speed_offset_(0.0),
   z_speed_offset_(0.0), x_pos_offset_(0.0), y_pos_offset_(0.0)
{};

Mpu6050::~Mpu6050() {};

bool Mpu6050::Begin() {
    return SetUp();
}

bool Mpu6050::SetImuRegister(const byte &reg, const byte &value) {
    Wire.beginTransmission(kAddress);
    Wire.write(reg);
    Wire.write(value);
    return Wire.endTransmission();
}

/**
* Faz o setup e configuração inicial da MPU6050
* Registradores e opções de acordo com datasheet disponível em
* https://www.invensense.com/products/motion-tracking/6-axis/mpu-6050/
*
* Configuração:
* - FIFO desativado
* - Precisão do giroscópio (+-500 º/s)
* - Precisão do acelerômetro (+-2g)
* - Ativa filtro passa baixas interno
*
* return true se tudo deu certo
**/
bool Mpu6050::SetUp() {
    Wire.begin();
    
    int i = 0;
    i += (int)SetImuRegister(0x64, 0x00); // Desabilita FIFO
    i += (int)SetImuRegister(0x1B, 0x01); // Configura gyro
    i += (int)SetImuRegister(0x1C, 0x00); // Configura acelerômetro
    i += (int)SetImuRegister(0x1A, 0x06); // Filtro passa baixas interno, consultar datasheet
    
    if(i != 0) return false;
    
    SetImuRegister(0x6B, 0x00); // Ativa (sai do sleep mode)
    return true;
}

bool Mpu6050::ReadFromImu() {
    Wire.beginTransmission(kAddress);
    Wire.write(0x3B); // Começa no registrador ACCEL_XOUT_H
    if (Wire.endTransmission()) return false; // return != 0 significa falha
    Wire.requestFrom((int)kAddress, 6, true); // 6 bytes (3 acelerações), encerra após enviar
    
    sensor_values_.acceleration_x = (Wire.read()<<8) | Wire.read();
    sensor_values_.acceleration_y = (Wire.read()<<8) | Wire.read();
    sensor_values_.acceleration_z = (Wire.read()<<8) | Wire.read();
    
    Wire.beginTransmission(kAddress);
    Wire.write(0x43); // GYRO_XOUT_H
    if (Wire.endTransmission()) return false;
    Wire.requestFrom((int)kAddress, 6, true);
    
    sensor_values_.speed_x = (Wire.read()<<8) | Wire.read();
    sensor_values_.speed_y = (Wire.read()<<8) | Wire.read();
    sensor_values_.speed_z = (Wire.read()<<8) | Wire.read();
    
    return true;
}

void Mpu6050::Update() {
    while(!ReadFromImu()){};
    
    velocity_.x = filter_speed_x_.input((float)sensor_values_.speed_x*kGyroScaling + x_speed_offset_); // Preenche os structs para o usuario
    velocity_.y = filter_speed_y_.input((float)sensor_values_.speed_y*kGyroScaling + y_speed_offset_);
    velocity_.z = filter_speed_z_.input((float)sensor_values_.speed_z*kGyroScaling + z_speed_offset_);
    
    float normalized_x_accel = (float)sensor_values_.acceleration_x*kAccelScaling;
    float normalized_y_accel = (float)sensor_values_.acceleration_y*kAccelScaling;
    float normalized_z_accel = (float)sensor_values_.acceleration_z*kAccelScaling;
    
    float norm = sqrt(sq(normalized_x_accel) + sq(normalized_y_accel) + sq(normalized_z_accel));
    normalized_x_accel /= norm;
    normalized_y_accel /= norm;
    normalized_z_accel /= norm;
    
    // Calculos com base em https://www.nxp.com/docs/en/application-note/AN3461.pdf equações 37 e 38
    rotation_.x =  filter_pos_x_.input(kRadToDeg*atan(normalized_y_accel/(Sign(normalized_z_accel)*sqrt(sq(normalized_z_accel) + 0.005*sq(normalized_x_accel)))) + x_pos_offset_); // Roll
    rotation_.y =  filter_pos_y_.input(kRadToDeg*atan(-normalized_x_accel/sqrt(sq(normalized_y_accel) + sq(normalized_z_accel))) + y_pos_offset_); // Pitch
    
    return;
}

RawReadings Mpu6050::GetRawReadings() {
    return sensor_values_;
}

Rotation Mpu6050::GetRotation() {
    return rotation_;
}

Velocity Mpu6050::GetVelocity() {
    return velocity_;
}

void Mpu6050::SetXPosOffset(float offset) {
  x_pos_offset_ = offset;
}
void Mpu6050::SetYPosOffset(float offset) {
  y_pos_offset_ = offset;
}
void Mpu6050::SetXSpeedOffset(float offset) {
  x_speed_offset_ = offset;
}
void Mpu6050::SetYSpeedOffset(float offset) {
  y_speed_offset_ = offset;
}
void Mpu6050::SetZSpeedOffset(float offset) {
  z_speed_offset_ = offset;
}

