/*
  Sketch: Dois MPU6050s (Porta 5 e Porta 3) com TCA9548A
  Author: Brincando com Ideias (adaptado para dois MPUs via multiplexador)
*/

#include "mpu6050.h"   // Sua biblioteca para o MPU6050
#include "PushButton.h" // Sua biblioteca para o PushButton
#include <Wire.h>      // Biblioteca padrão do Arduino para comunicação I2C

// --- Configuração para o TCA9548A ---
// Endereço I2C do seu multiplexador.
// Se os pinos A0, A1, A2 do seu TCA9548A estiverem todos conectados ao GND,
// ou se estiverem flutuando (sem conexão), o endereço padrão é 0x70.
// Se você os conectou de outra forma, altere este valor.
#define TCAADDR 0x70

// Define as portas do multiplexador onde os MPU6050s estão conectados
#define MPU6050_MUX_PORT_A 6 // Primeiro MPU6050 na porta 5
#define MPU6050_MUX_PORT_B 4 // Segundo MPU6050 na porta 3

// Função auxiliar para selecionar a porta do multiplexador
// Esta função envia um comando I2C para o TCA9548A
// para que ele ative a conexão com a porta 'i' (0 a 7).
void tcaselect(uint8_t i) {
  if (i > 7) {
    return;
  }
  Wire.beginTransmission(TCAADDR);
  Wire.write(1 << i);
  Wire.endTransmission();
}
// --- Fim da Configuração para o TCA9548A ---


PushButton bot(7); // Instancia o PushButton conectado ao pino 7

void setup() {
  Serial.begin(9600);
  Wire.begin(); // **IMPORTANTE**: Inicia a comunicação I2C principal do Arduino

  Serial.println("Inicializando MPU6050s via TCA9548A...");

  // --- Inicialização do MPU6050 na PORTA A (Porta 5) ---
  Serial.println("\n--- MPU6050 na Porta " + String(MPU6050_MUX_PORT_A) + " ---");
  tcaselect(MPU6050_MUX_PORT_A); // Seleciona a porta 5
  mpu_begin();                   // Inicializa o MPU6050 da porta 5
  Serial.println("Calibrando MPU6050 da Porta " + String(MPU6050_MUX_PORT_A) + ", mantenha-o parado!");
  delay(1000);
  tcaselect(MPU6050_MUX_PORT_A); // Re-seleciona antes da calibração
  mpu_calibrate(200);
  tcaselect(MPU6050_MUX_PORT_A); // Re-seleciona antes do reset
  mpu_reset();
  Serial.println("===== MPU6050 da Porta " + String(MPU6050_MUX_PORT_A) + " Calibrado! =====");

  // --- Inicialização do MPU6050 na PORTA B (Porta 3) ---
  Serial.println("\n--- MPU6050 na Porta " + String(MPU6050_MUX_PORT_B) + " ---");
  tcaselect(MPU6050_MUX_PORT_B); // Seleciona a porta 3
  mpu_begin();                   // Inicializa o MPU6050 da porta 3
  Serial.println("Calibrando MPU6050 da Porta " + String(MPU6050_MUX_PORT_B) + ", mantenha-o parado!");
  delay(1000);
  tcaselect(MPU6050_MUX_PORT_B); // Re-seleciona antes da calibração
  mpu_calibrate(200);
  tcaselect(MPU6050_MUX_PORT_B); // Re-seleciona antes do reset
  mpu_reset();
  Serial.println("===== MPU6050 da Porta " + String(MPU6050_MUX_PORT_B) + " Calibrado! =====");
  Serial.println("\nTodos os MPU6050s inicializados e calibrados.");
}

void loop() {
  bot.button_loop(); // Mantém o funcionamento do botão

  // --- Leitura e impressão do MPU6050 na PORTA A (Porta 5) ---
  tcaselect(MPU6050_MUX_PORT_A); // Seleciona a porta 5 antes de interagir com MPU A
  mpu_loop(); // Realiza a leitura dos dados do MPU da porta 5

  Serial.print("MPU Porta 5 -> ");
  Serial.print("roll(x):"); Serial.print(getAngleX());
  Serial.print(" | pitch(y):"); Serial.print(getAngleY());
  Serial.print(" | yaw(z):"); Serial.println(getAngleZ());

  // --- Leitura e impressão do MPU6050 na PORTA B (Porta 3) ---
  tcaselect(MPU6050_MUX_PORT_B); // Seleciona a porta 3 antes de interagir com MPU B
  delay(500); // Pequeno atraso para não sobrecarregar o Serial Monitor

  mpu_loop(); // Realiza a leitura dos dados do MPU da porta 3

  Serial.print("MPU Porta 3 -> ");
  Serial.print("roll(x):"); Serial.print(getAngleX());
  Serial.print(" | pitch(y):"); Serial.print(getAngleY());
  Serial.print(" | yaw(z):"); Serial.println(getAngleZ());

  if (bot.pressed()) {
    Serial.println("\nBotao pressionado! Recalibrando ambos os MPUs...");

    // Recalibrar MPU6050 na Porta A
    Serial.println("Recalibrando MPU6050 da Porta " + String(MPU6050_MUX_PORT_A) + "...");
    tcaselect(MPU6050_MUX_PORT_A);
    mpu_calibrate(400);
    tcaselect(MPU6050_MUX_PORT_A);
    mpu_reset();
    Serial.println("MPU6050 da Porta " + String(MPU6050_MUX_PORT_A) + " recalibrado.");

    // Recalibrar MPU6050 na Porta B
    Serial.println("Recalibrando MPU6050 da Porta " + String(MPU6050_MUX_PORT_B) + "...");
    tcaselect(MPU6050_MUX_PORT_B);
    mpu_calibrate(400);
    tcaselect(MPU6050_MUX_PORT_B);
    mpu_reset();
    Serial.println("MPU6050 da Porta " + String(MPU6050_MUX_PORT_B) + " recalibrado.");

    Serial.println("Ambos os MPUs recalibrados.");
  }

  delay(500); // Pequeno atraso para não sobrecarregar o Serial Monitor
}