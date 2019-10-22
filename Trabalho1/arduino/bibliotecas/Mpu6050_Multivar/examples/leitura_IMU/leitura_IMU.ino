/**
* Universidade Federal de Santa Catarina
* Departamento de Automação e Sistemas - CTC
*
* Biblioteca mpu6050 / Controle Multivariável
* Autor: Alex Amadeu Cani
* Jan 2018
**/

/*
 * Este exemplo serve como documentação de todas as funcionalidades da bilioteca para
 * uso da MPU6050.
 */
#include <Mpu6050.h>

Mpu6050 imu; // Declara a IMU

void setup() {
  Serial.begin(115200);
  if(!imu.Begin()) { // Configurações iniciais e conexões
    Serial.println("Falha na configuração da IMU");
  }


  // Existem 5 offsets, 2 em posição e 3 nas velocidades
  // O offset é o valor que é SOMADO ao valor obtido da leitura da IMU,
  // e serve como calibração.
  imu.SetXPosOffset(0.5); // Soma 0.5 na rotação X (em graus)
  imu.SetYPosOffset(-3.02); // Soma -3.02 na rotação Y (em graus)
  
  imu.SetXSpeedOffset(7.11); // Offsets para velocidade em graus/segundo
  imu.SetYSpeedOffset(3.58);
  imu.SetZSpeedOffset(3.58);
}

void loop() {
  /*
   * A utilização da biblioteca é simples. Para atualizar os valores basta
   * executar o método Update da arduino_imu. Os dados estarão disponíveis para uso
   * através dos métodos GetVelocity, GetRotation e GetRawReadings
   */
  imu.Update();

  Velocity gyro = imu.GetVelocity(); // Pega o conjunto de velocidades
  Rotation rot = imu.GetRotation(); // Pega o conjunto de posições angulares
  RawReadings raw = imu.GetRawReadings(); // Pega o conjunto de valores puros dos sensores,
                                      // antes de ser realizada a filtragem e escala correta

  // Posições, em GRAUS
  Serial.print(rot.x); // Rotação em X
  Serial.print('\t');
  Serial.print(rot.y); // Rotação em Y
  Serial.print('\t');

  // Velocidades, em GRAUS/SEGUNDO
  Serial.print(gyro.x); // Velocidade em X
  Serial.print('\t');
  Serial.print(gyro.y); // Velocidade em Y
  Serial.print('\t');
  Serial.print(gyro.z); // Velocidade em Z
  Serial.println();

  /**
   * Para os valores puros dos sensores, os campos são (note que a imu não fornece posição)
   * int acceleration_x;
   * int acceleration_y;
   * int acceleration_z;
   * int speed_x;
   * int speed_y;
   * int speed_z;
   * 
   * O ADC da MPU6050 tem 16 bits.
   * As configurações de precisão/escala e os filtros (passa baixas interno da MPU e o filtro de segunda ordem
   * após as medições) não estão disponíveis para alteração via interface com a class mpu6050, mas podem ser
   * alterados no arquivo fonte desta biblioteca, prestando-se a devida atenção ao datasheet e mapa de registradores.
   * O cálculo da posição é feito seguindo metodologia disponível em https://www.nxp.com/docs/en/application-note/AN3461.pdf ,
   * mais detalhes podem ser obtidos no código fonte.
   * 
   * O giroscópio está configurado para escala de +- 500º/s. Portanto precisão de 500/(2^15 -1) º/s por bit.
   * O acelerômetro está configurado para escala de +- 2g. Portanto precisão de 2/(2^15 -1) g por bit.
   */
  delay(10);
}
