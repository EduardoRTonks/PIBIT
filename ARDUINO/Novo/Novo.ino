/*
  Sketch Principal: Múltiplos MPU6050s via TCA9548A Multiplexador I2C
  Adaptado para saída CSV para plotagem em Python.
  MODIFICADO PARA INCLUIR UM 3º SENSOR NA PORTA 6.
*/

#include "MPU6050_Sensor.h" // Inclui a nova biblioteca da classe MPU6050_Sensor
#include "PushButton.h"     // Inclui sua biblioteca do PushButton
#include <Wire.h>           // Biblioteca padrão do Arduino para comunicação I2C

// --- Configuração do Multiplexador ---
#define TCA_I2C_ADDRESS 0x70

// --- Pinos do Multiplexador ---
#define MPU_PORT_1 2
#define MPU_PORT_2 4
#define MPU_PORT_3 6 // Adicionado para o MPU 6

// --- Endereços I2C dos MPU6050s ---
#define MPU6050_DEFAULT_I2C_ADDRESS 0x68

MPU6050_Sensor mpu1(MPU6050_DEFAULT_I2C_ADDRESS, MPU_PORT_1);
MPU6050_Sensor mpu2(MPU6050_DEFAULT_I2C_ADDRESS, MPU_PORT_2);
MPU6050_Sensor mpu3(MPU6050_DEFAULT_I2C_ADDRESS, MPU_PORT_3); // Adicionado para o MPU 6

PushButton recalibrateButton(7);

void setup() {
  Serial.begin(115200); // Aumentei o baud rate para uma comunicação mais rápida
  while (!Serial); // Espera a porta serial conectar (útil em algumas placas)
  Wire.begin();       // **IMPORTANTE**: Inicia a comunicação I2C principal do Arduino

  Serial.println("Inicializando MPU6050s via TCA9548A Multiplexador...");

  Wire.beginTransmission(TCA_I2C_ADDRESS);
  byte error = Wire.endTransmission();
  if (error == 0) {
    Serial.println("TCA9548A ENCONTRADO! Prosseguindo...");
  } else {
    Serial.print("TCA9548A NAO ENCONTRADO no endereco 0x");
    Serial.print(TCA_I2C_ADDRESS, HEX);
    Serial.println(". Verifique fiação e/ou endereço.");
    while (1);
  }
  Serial.println("------------------------------------");

  // --- Inicialização e Calibração do MPU na Porta 1 (MPU_PORT_1) ---
  Serial.print("\nInicializando MPU na Porta ");
  Serial.print(MPU_PORT_1);
  Serial.println("...");
  mpu1.begin();
  Serial.println("Calibrando MPU da Porta " + String(MPU_PORT_1) + ", deixe parado!");
  delay(1000);
  mpu1.calibrate(200);
  mpu1.reset();
  Serial.println("===== MPU da Porta " + String(MPU_PORT_1) + " Calibrado! =====");
  Serial.println("------------------------------------");

  // --- Inicialização e Calibração do MPU na Porta 2 (MPU_PORT_2) ---
  Serial.print("\nInicializando MPU na Porta ");
  Serial.print(MPU_PORT_2);
  Serial.println("...");
  mpu2.begin();
  Serial.println("Calibrando MPU da Porta " + String(MPU_PORT_2) + ", deixe parado!");
  delay(1000);
  mpu2.calibrate(200);
  mpu2.reset();
  Serial.println("===== MPU da Porta " + String(MPU_PORT_2) + " Calibrado! =====");
  Serial.println("------------------------------------");

  // --- Inicialização e Calibração do MPU na Porta 3 (MPU_PORT_3) --- // Adicionado para o MPU 6
  Serial.print("\nInicializando MPU na Porta ");
  Serial.print(MPU_PORT_3);
  Serial.println("...");
  mpu3.begin();
  Serial.println("Calibrando MPU da Porta " + String(MPU_PORT_3) + ", deixe parado!");
  delay(1000);
  mpu3.calibrate(200);
  mpu3.reset();
  Serial.println("===== MPU da Porta " + String(MPU_PORT_3) + " Calibrado! =====");
  Serial.println("------------------------------------");

  Serial.println("\nTodos os MPUs inicializados e calibrados. Pronto para ler!");
  Serial.println("FORMATO DE SAÍDA: <ID_SENSOR>,<ROLL>,<PITCH>,<YAW>");
  Serial.println("DATA_START");// Adicionado: Marcador para o Python saber que os dados CSV começam aqui
}

void loop() {
  recalibrateButton.button_loop();

  // --- Leitura e impressão do MPU da Porta 1 em formato CSV ---
  mpu1.loop();
  Serial.print(MPU_PORT_1); // ID do sensor (porta do MUX)
  Serial.print(",");
  Serial.print(mpu1.getAngleX(), 2); // Limita para 2 casas decimais
  Serial.print(",");
  Serial.print(mpu1.getAngleY(), 2); // Limita para 2 casas decimais
  Serial.print(",");
  Serial.println(mpu1.getAngleZ(), 2); // Limita para 2 casas decimais

  // --- Leitura e impressão do MPU da Porta 2 em formato CSV ---
  mpu2.loop();
  Serial.print(MPU_PORT_2); // ID do sensor (porta do MUX)
  Serial.print(",");
  Serial.print(mpu2.getAngleX(), 2);
  Serial.print(",");
  Serial.print(mpu2.getAngleY(), 2);
  Serial.print(",");
  Serial.println(mpu2.getAngleZ(), 2);

  // --- Leitura e impressão do MPU da Porta 3 em formato CSV --- // Adicionado para o MPU 6
  mpu3.loop();
  Serial.print(MPU_PORT_3); // ID do sensor (porta do MUX)
  Serial.print(",");
  Serial.print(mpu3.getAngleX(), 2);
  Serial.print(",");
  Serial.print(mpu3.getAngleY(), 2);
  Serial.print(",");
  Serial.println(mpu3.getAngleZ(), 2);


  if (recalibrateButton.pressed()) {
    Serial.println("RECALIBRATING_START");// Adicionado: Marcador para o Python
    Serial.println("Botao de recalibracao pressionado! Recalibrando todos os MPUs...");

    Serial.println("Recalibrando MPU da Porta " + String(MPU_PORT_1) + "...");
    mpu1.calibrate(400);
    mpu1.reset();
    Serial.println("MPU da Porta " + String(MPU_PORT_1) + " recalibrado.");

    Serial.println("Recalibrando MPU da Porta " + String(MPU_PORT_2) + "...");
    mpu2.calibrate(400);
    mpu2.reset();
    Serial.println("MPU da Porta " + String(MPU_PORT_2) + " recalibrado.");

    Serial.println("Recalibrando MPU da Porta " + String(MPU_PORT_3) + "..."); // Adicionado para o MPU 6
    mpu3.calibrate(400);
    mpu3.reset();
    Serial.println("MPU da Porta " + String(MPU_PORT_3) + " recalibrado."); // Adicionado para o MPU 6

    Serial.println("RECALIBRATING_END"); // Adicionado: Marcador para o Python
    Serial.println("Todos os MPUs recalibrados.");
    Serial.println("DATA_START"); // Adicionado: Recomeça o fluxo de dados CSV
  }
  delay(0); // Mantenha delay(0) para a maior velocidade possível

}