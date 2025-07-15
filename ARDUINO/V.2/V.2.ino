/*
  Sketch Principal para Arduino Mega - Versão para Comunicação USB
  - Usa a biblioteca customizada IMU_Sensor para todos os sensores.
  - Distingue entre MPU-9250 e MPU-6050.
  - Implementa fusão de 9 eixos para os MPU-9250 para um Yaw preciso.
  - Zera a orientação ao iniciar e ao recalibrar.
  - Envia dados pela porta Serial principal (USB).
*/

#include "IMU_Sensor.h"   // Inclui a biblioteca customizada
#include "PushButton.h"   // Sua biblioteca do PushButton
#include <Wire.h>

// --- Configuração do Multiplexador ---
#define TCA_I2C_ADDRESS 0x70

// --- Definição das Portas (para clareza) ---
#define PORT_MPU9250_1 2 // MPU-9250
#define PORT_MPU6050   4 // MPU-6050
#define PORT_MPU9250_2 6 // MPU-9250

// --- Endereço I2C Padrão ---
#define MPU_DEFAULT_I2C_ADDRESS 0x68

// --- Criação dos Objetos dos Sensores ---
IMU_Sensor mpu1(TYPE_MPU9250, MPU_DEFAULT_I2C_ADDRESS, PORT_MPU9250_1);
IMU_Sensor mpu2(TYPE_MPU6050, MPU_DEFAULT_I2C_ADDRESS, PORT_MPU6050);
IMU_Sensor mpu3(TYPE_MPU9250, MPU_DEFAULT_I2C_ADDRESS, PORT_MPU9250_2);

PushButton recalibrateButton(7);

// --- Variáveis de Offset para "Zerar" a Posição ---
float roll_offset1 = 0, pitch_offset1 = 0, yaw_offset1 = 0;
float roll_offset2 = 0, pitch_offset2 = 0, yaw_offset2 = 0;
float roll_offset3 = 0, pitch_offset3 = 0, yaw_offset3 = 0;

void setup() {
  Serial.begin(115200);
  while (!Serial);
  Wire.begin();

  Serial.println("Inicializando sensores com biblioteca customizada aprimorada...");
  
  // --- Inicialização e Calibração ---
  
  // MPU-9250 na Porta 2
  Serial.println("\n--- MPU-9250 (Porta 2) ---");
  mpu1.begin();
  /* * !!! PASSO OBRIGATÓRIO !!!
   * Insira aqui os valores de calibração do magnetômetro para ESTE sensor.
   * Use um sketch de calibração para obtê-los.
   * Exemplo: mpu1.setMagCalibration(biasX, biasY, biasZ, scaleX, scaleY, scaleZ);
  */
  // mpu1.setMagCalibration(25.5, -50.2, 112.0, 1.02, 0.98, 1.00); // SUBSTITUA COM SEUS VALORES
  Serial.println("Calibrando Acel/Giro, deixe parado!");
  mpu1.calibrate(200);
  mpu1.reset();
  Serial.println("MPU na Porta 2 inicializado e calibrado.");

  // MPU-6050 na Porta 4
  Serial.println("\n--- MPU-6050 (Porta 4) ---");
  mpu2.begin();
  Serial.println("Calibrando Acel/Giro, deixe parado!");
  mpu2.calibrate(200);
  mpu2.reset();
  Serial.println("MPU na Porta 4 inicializado e calibrado.");

  // MPU-9250 na Porta 6
  Serial.println("\n--- MPU-9250 (Porta 6) ---");
  mpu3.begin();
  /* * !!! PASSO OBRIGATÓRIO !!!
   * Insira aqui os valores de calibração do magnetômetro para ESTE SEGUNDO sensor.
   * Use um sketch de calibração para obtê-los.
  */
  // mpu3.setMagCalibration(30.1, -45.8, 105.5, 1.01, 0.99, 1.03); // SUBSTITUA COM SEUS VALORES
  Serial.println("Calibrando Acel/Giro, deixe parado!");
  mpu3.calibrate(200);
  mpu3.reset();
  Serial.println("MPU na Porta 6 inicializado e calibrado.");

  Serial.println("\n------------------------------------");
  
  // Define a orientação inicial como o "zero"
  Serial.println("Definindo orientacao inicial como zero...");
  mpu1.loop();
  roll_offset1 = mpu1.getAngleX();
  pitch_offset1 = mpu1.getAngleY();
  yaw_offset1 = mpu1.getAngleZ();

  mpu2.loop();
  roll_offset2 = mpu2.getAngleX();
  pitch_offset2 = mpu2.getAngleY();
  yaw_offset2 = mpu2.getAngleZ();

  mpu3.loop();
  roll_offset3 = mpu3.getAngleX();
  pitch_offset3 = mpu3.getAngleY();
  yaw_offset3 = mpu3.getAngleZ();
  Serial.println("Orientacao inicial definida como zero.");
  
  Serial.println("\nTodos os MPUs prontos. Iniciando leitura.");
  Serial.println("FORMATO: <ID_SENSOR>,<ROLL>,<PITCH>,<YAW>");
}

void loop() {
  recalibrateButton.button_loop();

  // --- Leitura e Impressão com Correção de Offset ---

  // MPU 1 (Porta 2)
  mpu1.loop();
  float final_roll1 = mpu1.getAngleX() - roll_offset1;
  float final_pitch1 = mpu1.getAngleY() - pitch_offset1;
  float final_yaw1 = mpu1.getAngleZ() - yaw_offset1;
  Serial.print(PORT_MPU9250_1);
  Serial.print(",");
  Serial.print(final_roll1, 2);
  Serial.print(",");
  Serial.print(final_pitch1, 2);
  Serial.print(",");
  Serial.println(final_yaw1, 2);

  // MPU 2 (Porta 4)
  mpu2.loop();
  float final_roll2 = mpu2.getAngleX() - roll_offset2;
  float final_pitch2 = mpu2.getAngleY() - pitch_offset2;
  float final_yaw2 = mpu2.getAngleZ() - yaw_offset2;
  Serial.print(PORT_MPU6050);
  Serial.print(",");
  Serial.print(final_roll2, 2);
  Serial.print(",");
  Serial.print(final_pitch2, 2);
  Serial.print(",");
  Serial.println(final_yaw2, 2);

  // MPU 3 (Porta 6)
  mpu3.loop();
  float final_roll3 = mpu3.getAngleX() - roll_offset3;
  float final_pitch3 = mpu3.getAngleY() - pitch_offset3;
  float final_yaw3 = mpu3.getAngleZ() - yaw_offset3;
  Serial.print(PORT_MPU9250_2);
  Serial.print(",");
  Serial.print(final_roll3, 2);
  Serial.print(",");
  Serial.print(final_pitch3, 2);
  Serial.print(",");
  Serial.println(final_yaw3, 2);

  // --- Lógica de Recalibração e Zeramento ---
  if (recalibrateButton.pressed()) {
    Serial.println("RECALIBRATING...");
    // 1. Recalibra o bias dos sensores (requer que estejam parados)
    mpu1.calibrate(400); mpu1.reset();
    mpu2.calibrate(400); mpu2.reset();
    mpu3.calibrate(400); mpu3.reset();
    
    // 2. Define a orientação atual como o novo "zero"
    Serial.println("Definindo orientacao atual como zero...");
    mpu1.loop();
    roll_offset1 = mpu1.getAngleX();
    pitch_offset1 = mpu1.getAngleY();
    yaw_offset1 = mpu1.getAngleZ();

    mpu2.loop();
    roll_offset2 = mpu2.getAngleX();
    pitch_offset2 = mpu2.getAngleY();
    yaw_offset2 = mpu2.getAngleZ();

    mpu3.loop();
    roll_offset3 = mpu3.getAngleX();
    pitch_offset3 = mpu3.getAngleY();
    yaw_offset3 = mpu3.getAngleZ();

    Serial.println("Recalibracao finalizada.");
  }
  
  delay(10);
}
