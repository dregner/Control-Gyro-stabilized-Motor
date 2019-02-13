#include <Mpu6050.h>
#include <Servo.h>

// ======== CONFIGURAÇÕES =========
// Altere os valores conforme a montagem do seu projeto

// Pinos
const int pino_servo = 9;
const int pino_giroscopio = 4;

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


// ======== CONFIGURAÇÕES CONTROLE =========                                
// Parâmetros de Controle
// Os estados do sistema, x1, x2 e x3 são realimentados
// através da lei de controle u = -K1*x1 + -K2*x2 + -K3*x3
// x1 -> Posição angular 'rho'
// x2 -> Posição angular do giroscópio 'theta'
// x3 -> Velocidade angular 'rho_ponto'

//K DISCRETO r = 0.7 Ts = 0.01
//const float K1 = -3.0395;
//const float K2 = -0.7455;
//const float K3 = -0.7984;

/*K DISCRETO r = 0.3 Ts = 0.01
const float K1 = -3.2771;
const float K2 = -0.9223;
const float K3 = -0.9714;*/

//K DISCRETO r = 0.7 Ts = 0.005
const float K1 = -3.4727;
const float K2 = -0.9386;
const float K3 = -0.9930;

/*//K DISCRETO r = 0.4 Ts = 0.005
const float K1 = -3.8102;
const float K2 = -1.1549;
const float K3 = -1.2074;*/


const int Ts = 5;  // Período de amostragem, em ms.

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
double xe1, xe2, xe3, y1, y2, y3;
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
   
    y1 = EIXO_X ? rot.x : rot.y;
    y1 *= (inverter_imu ? -1 : 1);
    y1 *= kGrausParaRadianos;
    
    y2 = theta;
  
    y3 = EIXO_X ? vel.x : vel.y;
    y3 *= (inverter_imu ? -1 : 1);
    y3 *= kGrausParaRadianos;
  
    
      //DISCRETO L por LQR 0.01; C = [1 0 0; 0 1 0]  (Ad-Bd*Kd-Ld*C) + Ld*y Ld = lqr 0.01
//    xe1 = (-0.0562*x1-0.0062*x2+0.0100*x3)+1.0421*y1;
//    xe2 = (0.0439*x1+0.0192*x2+0.0103*x3)+0.9902*y2;
//    xe3 = (-5.6938*x1-0.7455*x2+0.1997*x3)+3.9974*y1;

//    //DISCRETO L por LQR 0.01; C = [1 0 0; 0 1 0]  (Ad-Bd*Kd-Ld*C) + Ld*y Ld = lqr 0.1 Kd = 0.7 ts 5
//    xe1 = (0.0738*x1-0.0009*x2+0.0040*x3)+0.9241*y1;
//    xe2 = (0.0174*x1+0.0886*x2+0.0050*x3)+0.9161*y2;
//    xe3 = (-2.5714*x1-0.3700*x2+0.6098*x3d)+1.7342*y1;

     //DISCRETO L por LQR q=1e-4 r = 1; C = [1 0 0; 0 1 0]  (Ad-Bd*Kd-Ld*C) + Ld*y Ld = lqr 0.1 Kd = 0.7 ts 5
    xe1 = (0.07865*x1-0.0029*x2+0.0069*x3)+0.2068*y1;
    xe2 = (0.0304*x1+0.9975*x2+0.0080*x3)+0.01*y2;
    xe3 = (-3.4652*x1-0.5886*x2+0.3749*x3)+2.1304*y1;


//   //DISCRETO L por LQR 0.01; C = [1 0 0; 0 1 0]  (Ad-Bd*Kd-Ld*C) + Ld*y Ld = lqr 0.001
//    xe1 = (-0.0089*x1-0.0009*x2+0.0040*x3)+1.0068*y1;
//    xe2 = (0.0174*x1+0.0057*x2+0.0050*x3)+0.999*y2;
//    xe3 = (-2.6669*x1-0.3700*x2+0.6098*x3)+1.8296*y1;


//   //DISCRETO L por LQR 0.01; C = eye(3) (Ad-Bd*Kd-Ld*C) + Ld*y Ld = lqr 0.1
//    xe1 = (0.0807*x1-0.0009*x2-0.0040*x3)+0.9172*y1+0.008*y3;
//    xe2 = (0.0174*x1+0.0886*x2+0.0050*x3)+0.9161*y2;
//    xe3 = (-1.3279*x1-0.3700*x2 -0.3110*x3)+0.4906*y1+0.9208*y3;

    x1 = xe1;
    x2 = xe2;
    x3 = xe3;
    plot_obs();
    
}
