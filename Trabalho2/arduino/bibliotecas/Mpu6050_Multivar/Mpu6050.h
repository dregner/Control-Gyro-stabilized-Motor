/*
* Universidade Federal de Santa Catarina
* Departamento de Automação e Sistemas - CTC
*
* Biblioteca mpu6050 / Controle Multivariável
* Autor: Alex Amadeu Cani
* Jan 2018
*/

#ifndef MPU6050_MPU6050_H_
#define MPU6050_MPU6050_H_

#include <arduino.h>
#include <math.h>
#include <Filters.h>

struct RawReadings {
    int acceleration_x; // Na maioria das placas Arduino, int tem 16 bits, que é o tamanho do ADC da 6050
    int acceleration_y;
    int acceleration_z;
    int speed_x;
    int speed_y;
    int speed_z;
};

struct Rotation {
    float x;
    float y;
};

struct Velocity {
    float x;
    float y;
    float z;
};

class Mpu6050 {
  public:
    Mpu6050();
    ~Mpu6050();
    
    bool Begin();
    void Update();
    void SetXPosOffset(float offset);
    void SetYPosOffset(float offset);
    void SetXSpeedOffset(float offset);
    void SetYSpeedOffset(float offset);
    void SetZSpeedOffset(float offset);
    RawReadings GetRawReadings();
    Rotation GetRotation();
    Velocity GetVelocity();
    
  private:
    static constexpr byte kAddress = 0x68;
    static constexpr float kGyroScaling = 500.0/32767.0;
    static constexpr float kAccelScaling = 2.0/32767.0;
    static constexpr float kRadToDeg = 180.0/M_PI;
    static constexpr float kCutFrequency = 2.5;
    static constexpr float kQualityFactor = 1/sqrt(2);
    float x_pos_offset_, y_pos_offset_, x_speed_offset_, y_speed_offset_, z_speed_offset_;
    bool SetUp();
    bool ReadFromImu();
    bool SetImuRegister(const byte &reg, const byte &value);
    float Sign(float x){
        if(x==0.0) return 0.0; // Não usar com x=0 resultante de alguma operação, pois floats tem problemas com precisão!
        return x>0.0 ? 1 : -1;
    }
    
    RawReadings sensor_values_;
    Rotation rotation_;
    Velocity velocity_;
    FilterTwoPole filter_pos_x_;
    FilterTwoPole filter_pos_y_;
    FilterTwoPole filter_speed_x_;
    FilterTwoPole filter_speed_y_;
    FilterTwoPole filter_speed_z_;
};

#endif /* MPU6050_MPU6050_H_ */
