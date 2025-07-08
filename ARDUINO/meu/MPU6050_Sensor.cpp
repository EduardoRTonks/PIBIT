#include "MPU6050_Sensor.h"

// --- Endereço I2C do Multiplexador TCA9548A ---
// Este #define TCAADDR pode ser movido para o arquivo principal .ino
// ou para um arquivo de configuração global se você preferir.
// Por simplicidade, vou incluí-lo aqui também, mas cuidado com redefinições.
#define TCAADDR 0x70 // Endereço padrão do TCA9548A

// --- Função tcaselect() para o multiplexador (fora da classe, ou estática se preferir) ---
// É crucial que esta função seja chamada antes de qualquer operação I2C com o MPU
// É melhor deixá-la aqui para garantir que a classe controle a seleção da porta
// antes de tentar falar com o MPU.
void _tcaselect(uint8_t i) {
  if (i > 7) {
    return;
  }
  Wire.beginTransmission(TCAADDR);
  Wire.write(1 << i);
  Wire.endTransmission();
  delayMicroseconds(200); // Pequeno atraso para estabilização
}


// --- Implementação da Classe MPU6050_Sensor ---

// Construtor: Inicializa as variáveis membro com os valores passados
MPU6050_Sensor::MPU6050_Sensor(uint8_t mpu_i2c_address, uint8_t mux_port_number)
  : _mpu_i2c_address(mpu_i2c_address), _mux_port_number(mux_port_number) {
  // Inicializa outras variáveis de estado para cada instância
  _AccX = _AccY = _AccZ = 0;
  _GyroX = _GyroY = _GyroZ = 0;
  _AccErrorX = _AccErrorY = _GyroErrorX = _GyroErrorY = _GyroErrorZ = 0;
  _accAngleX = _accAngleY = _gyroAngleX = _gyroAngleY = _gyroAngleZ = 0;
  _roll = _pitch = _yaw = 0;
  _elapsedTime = _currentTime = _previousTime = 0;
}

// Configura o MPU
void MPU6050_Sensor::begin() {
  selectMuxPort(); // Seleciona a porta do multiplexador para ESTE MPU

  Wire.beginTransmission(_mpu_i2c_address);
  Wire.write(0x6B); // Reinicia o MPU
  Wire.write(0x00);
  Wire.endTransmission(true);

  // Configuração da faixa do Acelerômetro
  Wire.beginTransmission(_mpu_i2c_address);
  Wire.write(0x1C); // Registrador de conf. do acelerômetro
  Wire.write(0);    // Faixa: 2g
  Wire.endTransmission(true);

  // Configuração da faixa do Giroscópio
  Wire.beginTransmission(_mpu_i2c_address);
  Wire.write(0x1B); // Registrador de conf. do giroscópio
  Wire.write(0);    // faixa: 250°/s
  Wire.endTransmission(true);

  delay(20);
}

// Realiza a leitura do acelerômetro e salva os valores nas variáveis membro
void MPU6050_Sensor::_readAcel() {
  selectMuxPort(); // Garante que a porta correta esteja selecionada antes da leitura

  Wire.beginTransmission(_mpu_i2c_address);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom(_mpu_i2c_address, 6, true);

  _AccX = (Wire.read() << 8 | Wire.read()) / 16384.0f; // Para faixa de +-2g divisor = 16384
  _AccY = (Wire.read() << 8 | Wire.read()) / 16384.0f;
  _AccZ = (Wire.read() << 8 | Wire.read()) / 16384.0f;
}

// Realiza a leitura do giroscópio e salva os valores nas variáveis membro
void MPU6050_Sensor::_readGiro() {
  selectMuxPort(); // Garante que a porta correta esteja selecionada antes da leitura

  Wire.beginTransmission(_mpu_i2c_address);
  Wire.write(0x43);
  Wire.endTransmission(false);
  Wire.requestFrom(_mpu_i2c_address, 6, true);

  _GyroX = (Wire.read() << 8 | Wire.read()) / 131.0f; // Para faixa de +-250°/s divisor = 131.0
  _GyroY = (Wire.read() << 8 | Wire.read()) / 131.0f;
  _GyroZ = (Wire.read() << 8 | Wire.read()) / 131.0f;
}

// Calibra o MPU, realiza leituras e calcula os erros
void MPU6050_Sensor::calibrate(int num_readings) {
  _AccErrorX = 0;
  _AccErrorY = 0;
  _GyroErrorX = 0;
  _GyroErrorY = 0;
  _GyroErrorZ = 0;

  for (int nR = 0; nR < num_readings; nR++) {
    _readAcel(); // Usa o método privado para ler acelerômetro
    _AccErrorX += (atan((_AccY) / sqrt(pow((_AccX), 2) + pow((_AccZ), 2))) * 180 / PI);
    _AccErrorY += (atan(-1 * (_AccX) / sqrt(pow((_AccY), 2) + pow((_AccZ), 2))) * 180 / PI);
  }
  _AccErrorX = _AccErrorX / (float)num_readings;
  _AccErrorY = _AccErrorY / (float)num_readings;

  for (int nR = 0; nR < num_readings; nR++) {
    _readGiro(); // Usa o método privado para ler giroscópio
    _GyroErrorX += _GyroX;
    _GyroErrorY += _GyroY;
    _GyroErrorZ += _GyroZ;
  }
  _GyroErrorX = _GyroErrorX / (float)num_readings;
  _GyroErrorY = _GyroErrorY / (float)num_readings;
  _GyroErrorZ = _GyroErrorZ / (float)num_readings;
}

// Realiza as leituras e calcula os ângulos dos três eixos
void MPU6050_Sensor::loop() {
  _readAcel();

  _accAngleX = (atan(_AccY / sqrt(pow(_AccX, 2) + pow(_AccZ, 2))) * 180 / PI) - _AccErrorX;
  _accAngleY = (atan(-1 * _AccX / sqrt(pow(_AccY, 2) + pow(_AccZ, 2))) * 180 / PI) - _AccErrorY;

  _previousTime = _currentTime;
  _currentTime = micros();
  _elapsedTime = (_currentTime - _previousTime) / 1000000.0f;

  _readGiro();
  _GyroX -= _GyroErrorX;
  _GyroY -= _GyroErrorY;
  _GyroZ -= _GyroErrorZ;

  _gyroAngleX += _GyroX * _elapsedTime;
  _gyroAngleY += _GyroY * _elapsedTime;
  _gyroAngleZ += _GyroZ * _elapsedTime;

  _roll = 0.96 * _gyroAngleX + 0.04 * _accAngleX;
  _pitch = 0.96 * _gyroAngleY + 0.04 * _accAngleY;
  _yaw = _gyroAngleZ;
}

// Reinicia os ângulos do giroscópio
void MPU6050_Sensor::reset() {
  _gyroAngleX = 0;
  _gyroAngleY = 0;
  _gyroAngleZ = 0;

  _currentTime = micros();
  loop(); // Chame loop() para atualizar o tempo base e fazer uma leitura inicial
}

// Retorna o ângulo de giro em X
float MPU6050_Sensor::getAngleX() {
  return _roll;
}

// Retorna o ângulo de giro em Y
float MPU6050_Sensor::getAngleY() {
  return _pitch;
}

// Retorna o ângulo de giro em Z
float MPU6050_Sensor::getAngleZ() {
  return _yaw;
}

// Método para selecionar a porta do multiplexador para este sensor
void MPU6050_Sensor::selectMuxPort() {
  _tcaselect(_mux_port_number);
}