/*
  Sketch Principal: Múltiplos MPU6050s via TCA9548A Multiplexador I2C
  Adaptado para saída CSV para plotagem em Python.
*/

#include "MPU6050_Sensor.h" // Inclui a nova biblioteca da classe MPU6050_Sensor
#include <Wire.h>           // Biblioteca padrão do Arduino para comunicação I2C

// --- Configuração do Multiplexador ---
#define TCA_I2C_ADDRESS 0x70

// --- Pinos do Multiplexador (agora para portas 2, 4 e 6) ---
#define MPU_PORT_2 2
#define MPU_PORT_4 4
#define MPU_PORT_6 6

// --- Endereços I2C dos MPU6050s ---
#define MPU6050_DEFAULT_I2C_ADDRESS 0x68

MPU6050_Sensor mpu2(MPU6050_DEFAULT_I2C_ADDRESS, MPU_PORT_2);
MPU6050_Sensor mpu4(MPU6050_DEFAULT_I2C_ADDRESS, MPU_PORT_4);
MPU6050_Sensor mpu6(MPU6050_DEFAULT_I2C_ADDRESS, MPU_PORT_6);

// Variáveis para controlar a frequência das leituras
unsigned long lastReadTime = 0;
const long readInterval = 500; // 30 segundos para 2 leituras por minuto

String selectedSensor = ""; // Variável para armazenar a seleção do usuário

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
    while (1); // Trava aqui se o multiplexador não for encontrado
  }
  Serial.println("------------------------------------");

  // --- Inicialização e Calibração dos MPUs nas Portas 2, 4 e 6 ---
  Serial.println("\nInicializando e Calibrando MPUs, deixe parado!");

  // MPU na Porta 2
  Serial.print("Inicializando MPU na Porta ");
  Serial.print(MPU_PORT_2);
  Serial.println("...");
  mpu2.begin();
  Serial.println("Calibrando MPU da Porta " + String(MPU_PORT_2) + "...");
  delay(1000); // Pequeno atraso para estabilização
  mpu2.calibrate(200);
  mpu2.reset();
  Serial.println("===== MPU da Porta " + String(MPU_PORT_2) + " Calibrado! =====");
  Serial.println("------------------------------------");

  // MPU na Porta 4
  Serial.print("Inicializando MPU na Porta ");
  Serial.print(MPU_PORT_4);
  Serial.println("...");
  mpu4.begin();
  Serial.println("Calibrando MPU da Porta " + String(MPU_PORT_4) + "...");
  delay(1000); // Pequeno atraso para estabilização
  mpu4.calibrate(200);
  mpu4.reset();
  Serial.println("===== MPU da Porta " + String(MPU_PORT_4) + " Calibrado! =====");
  Serial.println("------------------------------------");

  // MPU na Porta 6
  Serial.print("Inicializando MPU na Porta ");
  Serial.print(MPU_PORT_6);
  Serial.println("...");
  mpu6.begin();
  Serial.println("Calibrando MPU da Porta " + String(MPU_PORT_6) + "...");
  delay(1000); // Pequeno atraso para estabilização
  mpu6.calibrate(200);
  mpu6.reset();
  Serial.println("===== MPU da Porta " + String(MPU_PORT_6) + " Calibrado! =====");
  Serial.println("------------------------------------");

  Serial.println("\nTodos os MPUs inicializados e calibrados.");
  Serial.println("\nSelecione qual sensor deseja imprimir os valores:");
  Serial.println("Digite '2' para MPU da Porta 2");
  Serial.println("Digite '4' para MPU da Porta 4");
  Serial.println("Digite '6' para MPU da Porta 6");
  Serial.println("Digite 'all' para todos os MPUs");
  Serial.println("Aguardando sua seleção...");

  // Espera por entrada serial
  while (Serial.available() == 0) {
    delay(100);
  }

  selectedSensor = Serial.readStringUntil('\n'); // Lê a entrada do usuário
  selectedSensor.trim(); // Remove espaços em branco ou caracteres de nova linha
  selectedSensor.toLowerCase(); // Converte para minúsculas para facilitar a comparação

  Serial.print("Você selecionou: ");
  Serial.println(selectedSensor);

  Serial.println("FORMATO DE SAÍDA: <ID_SENSOR>,<ROLL>,<PITCH>,<YAW>");
  Serial.println("DATA_START"); // Adicionado: Marcador para o Python saber que os dados CSV começam aqui

  lastReadTime = millis(); // Inicializa o tempo da última leitura
}

void loop() {
  unsigned long currentTime = millis();

  // Verifica se é hora de fazer uma nova leitura
  if (currentTime - lastReadTime >= readInterval) {
    lastReadTime = currentTime; // Atualiza o tempo da última leitura

    // --- Leitura e impressão do MPU da Porta 2 em formato CSV ---
    if (selectedSensor.equals("2") || selectedSensor.equals("all")) {
      mpu2.loop();
      Serial.print(MPU_PORT_2); // ID do sensor (porta do MUX)
      Serial.print(",");
      Serial.print(mpu2.getAngleX(), 2); // Limita para 2 casas decimais para reduzir o tamanho da string
      Serial.print(",");
      Serial.print(mpu2.getAngleY(), 2); // Limita para 2 casas decimais para reduzir o tamanho da string
      Serial.print(",");
      Serial.println(mpu2.getAngleZ(), 2); // Limita para 2 casas decimais para reduzir o tamanho da string
    }

    // --- Leitura e impressão do MPU da Porta 4 em formato CSV ---
    if (selectedSensor.equals("4") || selectedSensor.equals("all")) {
      mpu4.loop();
      Serial.print(MPU_PORT_4); // ID do sensor (porta do MUX)
      Serial.print(",");
      Serial.print(mpu4.getAngleX(), 2);
      Serial.print(",");
      Serial.print(mpu4.getAngleY(), 2);
      Serial.print(",");
      Serial.println(mpu4.getAngleZ(), 2);
    }

    // --- Leitura e impressão do MPU da Porta 6 em formato CSV ---
    if (selectedSensor.equals("6") || selectedSensor.equals("all")) {
      mpu6.loop();
      Serial.print(MPU_PORT_6); // ID do sensor (porta do MUX)
      Serial.print(",");
      Serial.print(mpu6.getAngleX(), 2);
      Serial.print(",");
      Serial.print(mpu6.getAngleY(), 2);
      Serial.print(",");
      Serial.println(mpu6.getAngleZ(), 2);
    }
  }

  // Pequeno atraso para não sobrecarregar o loop, mas ainda permitir a leitura serial rápida
  delay(10);
}