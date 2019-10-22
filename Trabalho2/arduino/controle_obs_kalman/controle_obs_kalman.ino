#include <Mpu6050.h>
#include <Servo.h>
#include <Matrix.h>


// ======== CONFIGURAÇÕES FILTROD E KALMAN =========
// Tempo de amostragem 10 ms
//----------Jacobiano F e G------------
float jac_A[3][3] = {{1.0053,0,0.01},{0,1,0},{1.0652,0,1.053}};
Matrix<float> A(3,3,(float*)jac_A);
float jac_B[3][1] = {{-0.0039},{0.01},{-0.7896}};
Matrix<float> B(3,1,(float*)jac_B);

//--------SAIDAS h(x)--------
// DUAS SAIDAS POS MOTO E GIRO
float jac_C[2][3] = {{1,0,0},{0,1,0}}; 
Matrix<float> C(2,3,(float*)jac_C);

//--------Ruidos----------
// Ruido de entrada
float ruido_w[3][3] = {{10^-8,0,0},{0,10^-11,0},{0,0,10^-9}};
Matrix<float> w(3,3,(float*)ruido_w);
//Ruido duas saidas
// Duas saidas
float ruido_v[2][2] = {{10^-4,0},{0,10^-4}}; 
Matrix<float> v(2,2,(float*)ruido_v);
Matrix<float> S(2,2,'I');

//=====P inicial===========
Matrix<float> P(3,3,'I');

Matrix<float> xp(3,1,1);
Matrix<float> xap(3,1,1);
Matrix<float> Pp(3,3,1);
Matrix<float> x(3,1,1);
Matrix<float> K(3,2,1);


// ======== CONFIGURAÇÕES =========
// Altere os valores conforme a montagem do seu projeto

// Pinos
const int pino_servo = 8;
const int pino_giroscopio = 4;

// Parâmetros servo
const int offset_servo = 80; /* Valor, em graus, que deixa o
                                giroscópio paralelo a moto.
                                Utilizar o programa teste_servo
                                para determinar o valor.
                             */
const int servo_maximo = 120; 
const int servo_minimo = 40; /*//  Limite superior da atuação do servo
                               //  Limite inferior da atuação do servo
                              //  Novamente, utilize o arquivo teste_servo
                              //  para determinar o menor e maior valor para atuar*/
const bool inverter_direcao_servo = false; /*//  Modifique este parâmetro caso
                                           //  a direção da rotação do servo precise
                                           //  ser invertida (de modo que um aumento de u
                                           //  provoque um aumento de theta).*/
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
float offset_posicao_x = 1;
float offset_posicao_y = 0.0;
float offset_velocidade_x = -9;
float offset_velocidade_y = 0.0; //  Para determinar os valores utilize o programa calibracao_imu


const bool inverter_imu = false; //  Análogo ao inverter_direcao_servo. Ajustar de modo que o sentido de rotação
                                 //  positivo da IMU seja o sentido positivo de rho


// ======== CONFIGURAÇÕES CONTROLE =========                                
// Parâmetros de Controle
// Os estados do sistema, x1, x2 e x3 são realimentados
// através da lei de controle u = -K1*x1 + -K2*x2 + -K3*x3
// x1 -> Posição angular 'rho'
// x2 -> Posição angular do giroscópio 'theta'
// x3 -> Velocidade angular 'rho_ponto'

//K DISCRETO r = 0.7 Ts = 0.01
const float K1 = -3.0395;
const float K2 = -0.7455;
const float K3 = -0.7984;



//K DISCRETO r = 0.7 Ts = 0.005
//const float K1 = -3.4727;
//const float K2 = -0.9386;
//const float K3 = -0.9930;



const int Ts = 10;  // Período de amostragem, em ms.

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
double y1, y2, y3;
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
    x1 = 0;
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
    

    observador();

    tempo_ultimo_controle = millis();
    
    Serial.println();
  }
}

void plot_obs(){
      // Plots
    if (plotar_referencia) {
      Serial.print(0);
      Serial.print('\t');
    }
    if (plotar_x1) {
      Serial.print(y1);
      Serial.print('\t');
    }
    if (plotar_x2) {
      Serial.print(y2);
      Serial.print('\t');
    }
    if (plotar_x3) {
      Serial.print(y3);
      Serial.print('\t');
    }
        if (plotar_x1e) {
      Serial.print(x1);
      Serial.print('\t');
    }
    if (plotar_x2e) {
      Serial.print(x2);
      Serial.print('\t');
    }
    if (plotar_x3e) {
      Serial.print(x3);
      Serial.print('\t');
    }
    if (plotar_atuacao_min_max) {
      Serial.print(kGrausParaRadianos*servo_maximo);
      Serial.print('\t');
      Serial.print(kGrausParaRadianos*servo_minimo);
      Serial.print('\t');
    }
}


void observador(){

    u = -K1*x1 -K2*x2 -K3*x3;

    theta += (u*dt)/1000.0; //  Converter dt para segundos
    theta = min(0.7, max(-0.7, theta));
    atuacao = mapfloat(theta, -0.7, 0.7, 40, 120);
    atuador.write(atuacao);
    
   Rotation rot = imu.GetRotation();
   Velocity vel = imu.GetVelocity();
   
    y1 = EIXO_X ? rot.x : rot.y;
    y1 *= (inverter_imu ? -1 : 1);
    y1 *= kGrausParaRadianos;
    
    y2 = theta;
  
    y3 = EIXO_X ? vel.x : vel.y;
    y3 *= (inverter_imu ? -1 : 1);
    y3 *= kGrausParaRadianos;
  
    kalman_code();
    plot_obs();
    
}
void kalman_code(){
  
  float x_a[3][1] = {{x1},{x2},{x3}};
  Matrix<float> xa(3,1,(float*)x_a);

  float y_[2][1] = {{y1},{y2}};
  Matrix<float> y(2,1,(float*)y_);

  Matrix<float> Ct = Matrix<float>::transpose(C);
  Matrix<float> S = (C*P)*Ct+v;
  
  float S_inv[2][2] = {{1/S._entity[0][0],0},{0,1/S._entity[1][1]}};
  Matrix<float> Sinv(2,2,(float*)S_inv); //= Matrix<float>::inv(S);
  
  K = (P*Ct)*Sinv;
  xap = xa+K*(y-C*xa)
  ;
  Matrix<float> eye3(3,3,'I');
  Pp = (eye3 - K*C)*P;

  xp = A*xap+B*u;
  Matrix<float> At = Matrix<float>::transpose(A);
  P = ((A*Pp)*At)+w;
  
  x1 = xp._entity[0][0];
  x2 = xp._entity[1][0];
  x3 = xp._entity[2][0];

}

float mapfloat(float x, float in_min, float in_max, float out_min, float out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}
