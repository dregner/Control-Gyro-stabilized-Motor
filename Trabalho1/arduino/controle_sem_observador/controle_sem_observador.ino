/*
* Universidade Federal de Santa Catarina
* Departamento de Automação e Sistemas - CTC
*
* Código de controle sem observador / Controle Multivariável
* Autor: Alex Amadeu Cani
* Jan 2018
*/

#include <Mpu6050.h>
#include <Servo.h>

// ======== CONFIGURAÇÕES =========
// Altere os valores conforme a montagem do seu projeto

// Pinos
const int pino_servo = 5;
const int pino_giroscopio = 9;

// Parâmetros servo
const int offset_servo = 85; /* Valor, em graus, que deixa o
                                giroscópio paralelo a moto.
                                Utilizar o programa teste_servo
                                para determinar o valor.
                             */
const int servo_maximo = 127; //  Limite superior da atuação do servo
const int servo_minimo = 47;  //  Limite inferior da atuação do servo
                              //  Novamente, utilize o arquivo teste_servo
                              //  para determinar o menor e maior valor para atuar

const bool inverter_direcao_servo = false; //  Modifique este parâmetro caso
                                           //  a direção da rotação do servo precise
                                           //  ser invertida (de modo que um aumento de u
                                           //  provoque um aumento de theta).

// Parâmetros IMU
const bool EIXO_X = true; /* A IMU pode ser montada de duas formas,
                             ou seja, de modo que a posição angular
                             da moto corresponda ao eixo X da IMU ou
                             ao eixo Y.
                             Valores:
                             true = eixo de rotação da moto corresponde
                                    ao eixo X da IMU
                             false = eixo de rotação da moto corresponde
                                    ao eixo Y da IMU
                           */
                         
float offset_posicao_x = 0.65;
float offset_posicao_y = 0.0;
float offset_velocidade_x = -10.58;
float offset_velocidade_y = 0.0; //  Para determinar os valores utilize o programa calibracao_imu


const bool inverter_imu = false; //  Análogo ao inverter_direcao_servo. Ajustar de modo que o sentido de rotação
                                 //  positivo da IMU seja o sentido positivo de rho
                                 
// Parâmetros de Controle
// Os estados do sistema, x1, x2 e x3 são realimentados
// através da lei de controle u = -K1*x1 + -K2*x2 + -K3*x3
// x1 -> Posição angular 'rho'
// x2 -> Posição angular do giroscópio 'theta'
// x3 -> Velocidade angular 'rho_ponto'

//r = 0.1
//const float K1 =  -7.2448;
//const float K2 = -3.1623;
//const float K3 = -3.2248;
//r = 0.3 valores rpm = 7200
//const float K1 =  -4.8461;
//const float K2 = -1.8257;
//const float K3 = -1.8769;
//r = 0.4 valores rpm = 6500 FUNCIONANDO REDONDO (23hrs 14/11)
//const float K1 =  -4.7122;
//const float K2 = -1.5811;
//const float K3 = -1.6387;
//r = 0.7 valores rpm = 6500
//const float K1 =  -4.0488;
//const float K2 = -1.1952;
//const float K3 = -1.2527;

//K DISCRETO r = 0.7
const float K1 = -2.6341;
const float K2 = -0.5648;
const float K3 = -0.6174;


const int Ts =  5;  // Período de amostragem, em ms.
const int Ts1 = 5;    // Menor valor aconselhado 5ms

// Opções de visualização
const bool plotar_referencia = false;
const bool plotar_x1 = true;
const bool plotar_x2 = true;
const bool plotar_x3 = true;
const bool plotar_x1e = true;
const bool plotar_x2e = true;
const bool plotar_x3e = true;
const bool plotar_atuacao_min_max = false;

//  ======= FIM DAS CONFIGURAÇÕES =========
//  Não é necessário editar nada abaixo desta linha

Mpu6050 imu;
Servo atuador;
Servo giroscopio;
unsigned long tempo_ultimo_controle;
float x1, x2, x3;
double x1d,x2d,x3d, xe1, xe2, xe3, y1, y2;
float du, dt;
float theta = 0;
float u;
int atuacao;

const float kGrausParaRadianos = M_PI/180.0;
const float kRadianosParaGraus = 180.0/M_PI;

void setup() {
  Serial.begin(115200);
  if(!imu.Begin()) {
    Serial.println("Erro ao configurar IMU");
    Serial.end();
    while(true);
  }
  //Condicoes Iniciais
    x1 = 0.29;
    x2 = theta;
    x3 = 0;
    
  imu.SetXPosOffset(offset_posicao_x);
  imu.SetYPosOffset(offset_posicao_y); 
  imu.SetXSpeedOffset(offset_velocidade_x);
  imu.SetYSpeedOffset(offset_velocidade_y);

  atuador.attach(pino_servo);
  atuador.write(offset_servo); //  Vai para a posição inicial
  
  giroscopio.attach(pino_giroscopio); //  Sequência de inicializaçao do giro
  giroscopio.writeMicroseconds(500);
  delay(3000);
  giroscopio.writeMicroseconds(1000);
  delay(3000);
  giroscopio.writeMicroseconds(2000);

  delay(12000); //  Espera o giroscópio acelerar para iniciar o controle


    
  tempo_ultimo_controle = millis();
}

void loop() {
  //  IMU deve ser atualizada o mais frequente possível devido aos filtros.
  imu.Update();
  dt = millis() - tempo_ultimo_controle;
  
  if (dt >= Ts) { // Loop de controle
    
    //sem_observador();
    observador();

    
    tempo_ultimo_controle = millis();
    
    // Plots
    if (plotar_referencia) {
      Serial.print(0);
      Serial.print('\t');
    }
    if (plotar_x1) {
      Serial.print(x1d);
      Serial.print('\t');
    }
    if (plotar_x2) {
      Serial.print(x2d);
      Serial.print('\t');
    }
    if (plotar_x3) {
      Serial.print(x3d);
      Serial.print('\t');
    }
        if (plotar_x1e) {
      Serial.print(xe1);
      Serial.print('\t');
    }
    if (plotar_x2e) {
      Serial.print(xe2);
      Serial.print('\t');
    }
    if (plotar_x3e) {
      Serial.print(xe3);
      Serial.print('\t');
    }
//    if (plotar_atuacao_min_max) {
//      Serial.print(kGrausParaRadianos*servo_maximo);
//      Serial.print('\t');
//      Serial.print(kGrausParaRadianos*servo_minimo);
//      Serial.print('\t');
//    }
    Serial.println();
  }
}

void observador(){

    u = -K1*x1 -K2*x2 -K3*x3;

    theta += (u*dt)/1000.0; //  Converter dt para segundos
    // Atua
    atuacao = static_cast<int>((inverter_direcao_servo ? -1 : 1)*kRadianosParaGraus*theta + offset_servo);
    if (atuacao > servo_maximo) {
      atuacao = servo_maximo;
      theta = kGrausParaRadianos*(servo_maximo-offset_servo); //  Não pode deixar theta ficar crescendo!
    } else if (atuacao < servo_minimo) {
      atuacao = servo_minimo;
      theta = kGrausParaRadianos*(servo_minimo-offset_servo);
    }
    
    atuador.write(atuacao);
    
   Rotation rot = imu.GetRotation();
   Velocity vel = imu.GetVelocity();
   
    x1d = EIXO_X ? rot.x : rot.y;
    x1d *= (inverter_imu ? -1 : 1);
    x1d *= kGrausParaRadianos;
    
    x2d = theta;
  
    x3d = EIXO_X ? vel.x : vel.y;
    x3d *= (inverter_imu ? -1 : 1);
    x3d *= kGrausParaRadianos;
  
    y1 = x1d;
    y2 = x2d;

        //DISCRETO L por LQR 0.01; C = [1 0 0; 0 1 0]
    xe1 = (Ts1*(-1.0421*x1-0.0062*x2+1*x3))/1000+(Ts1*1.0421*y1)/1000+x1;
    xe2 = (Ts1*(2.6341*x1-0.4254*x2+0.6174*x3))/1000+(Ts1*0.9902*y2)/1000+x2;
    xe3 = (Ts1*(-105.2803*x1-44.5130*x2-48.6636*x3))/1000+(Ts1*3.9974*y1)/1000+x3d;


    x1 = xe1;
    x2 = xe2;
    x3 = xe3;

    
}

void sem_observador(){

   Rotation rot = imu.GetRotation();
   Velocity vel = imu.GetVelocity();
   
    x1 = EIXO_X ? rot.x : rot.y;
    x1 *= (inverter_imu ? -1 : 1);
    x1 *= kGrausParaRadianos;
    
    x2 = theta;
  
    x3 = EIXO_X ? vel.x : vel.y;
    x3 *= (inverter_imu ? -1 : 1);
    x3 *= kGrausParaRadianos;
//
    u = -K1*x1 -K2*x2 -K3*x3;

    theta += (u*dt)/1000.0; //  Converter dt para segundos
    // Atua
    atuacao = static_cast<int>((inverter_direcao_servo ? -1 : 1)*kRadianosParaGraus*theta + offset_servo);
    if (atuacao > servo_maximo) {
      atuacao = servo_maximo;
      theta = kGrausParaRadianos*(servo_maximo-offset_servo); //  Não pode deixar theta ficar crescendo!
    } else if (atuacao < servo_minimo) {
      atuacao = servo_minimo;
      theta = kGrausParaRadianos*(servo_minimo-offset_servo);
    }
    
    atuador.write(atuacao);


}   


