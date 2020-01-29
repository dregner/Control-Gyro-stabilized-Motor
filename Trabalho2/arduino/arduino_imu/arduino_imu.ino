#include <Wire.h>
//#include <SoftwareSerial.h>
#include "imu.h"
#include <Arduino.h>
#include <Servo.h>
#include <Mpu6050.h>

Servo myservo;

extern double gx;
extern double gy;
extern double gz;
double rho = 0;
double drho = 0;
double dtheta = 0;

float Ts = 100;
double prev_millis;


const bool EIXO_X = true; /* A IMU pode ser montada de duas formas,
                             ou seja, de modo que a posição angular
                             da moto corresponda ao eixo X da IMU ou
                             ao eixo Y.
                             Valores:
                             true = eixo de rotação da moto corresponde
                                    ao eixo X da IMU
                                    ao eixo Y da IMU
*/

float offset_posicao_x = 1;
float offset_posicao_y = 0.0;
float offset_velocidade_x = -4.56;
float offset_velocidade_y = 0.0; //  Para determinar os valores utilize o programa calibracao_imu

double x1, x2, x3, gx1, drho1;

const bool inverter_imu = false; //  Análogo ao inverter_direcao_servo. Ajustar de modo que o sentido de rotação
//  positivo da IMU seja o sentido positivo de rho

Mpu6050 imu1;
const float kGrausParaRadianos = M_PI / 180.0;
const float kRadianosParaGraus = 180.0 / M_PI;

int val;

void setup(){
    pinMode(7,OUTPUT);
  pinMode(8,OUTPUT);
  digitalWrite(7,LOW);
  digitalWrite(8,HIGH);
  myservo.attach(8);  // attaches the servo on pin 9 to the servo object
    int error;
    Serial.begin(115200);

    if (!imu1.Begin()) {
        Serial.println("Erro ao configurar IMU");
        Serial.end();
        while (true);
    }

    imu1.SetXPosOffset(offset_posicao_x);
    imu1.SetYPosOffset(offset_posicao_y);
    imu1.SetXSpeedOffset(offset_velocidade_x);
    imu1.SetYSpeedOffset(offset_velocidade_y);


    //BTSerial.begin(38400);

    prev_millis = millis();
    setup_imu();
    gx1 = gx;
    drho1 = drho;
    val = 120;
}

void loop(){
    imu();
    imu1.Update();
    
    Rotation rot = imu1.GetRotation();
    Velocity vel = imu1.GetVelocity();
//   gx = gx*3.1415/180;
    
    drho = (gx - gx1)*SAMPLING_FREQ;
//    drho = 0.99*drho1 + 0.01*drho;
//    drho1 = drho;
    
    x1 = EIXO_X ? rot.x : rot.y;
    x1 *= (inverter_imu ? -1 : 1);
    x1 *= kGrausParaRadianos;
    x3 = EIXO_X ? vel.x : vel.y;
    x3 *= (inverter_imu ? 8 - 1 : 1);
    x3 *= kGrausParaRadianos;
    if(millis()-prev_millis > Ts){
        Serial.print("will: ");
        Serial.print(gx);
        Serial.print(", ");
        Serial.print(val);
        Serial.print(", ");
        Serial.println(drho);
        Serial.print("Lib: ");
        Serial.print(x1);
        Serial.print(", ");
        Serial.print(val*kGrausParaRadianos);
        Serial.print(", ");
        Serial.println(x3);

        prev_millis = millis();


    }
    gx1 = gx;
      myservo.write(val);

}
