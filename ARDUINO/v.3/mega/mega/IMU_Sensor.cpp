#include "IMU_Sensor.h"

#define TCAADDR 0x70 // Endereço do multiplexador

// Função auxiliar para selecionar a porta do MUX
void _tcaselect(uint8_t i) {
  if (i > 7) return;
  Wire.beginTransmission(TCAADDR);
  Wire.write(1 << i);
  Wire.endTransmission();
  delayMicroseconds(200);
}

// Construtor
IMU_Sensor::IMU_Sensor(IMU_TYPE sensorType, uint8_t mpu_i2c_address, uint8_t mux_port_number)
  : _sensorType(sensorType), _mpu_i2c_address(mpu_i2c_address), _mux_port_number(mux_port_number) {
  // Inicializa variáveis
  _roll = _pitch = _yaw = 0;
  _currentTime = _previousTime = 0;
}

void IMU_Sensor::begin() {
  selectMuxPort();

  Wire.beginTransmission(_mpu_i2c_address);
  Wire.write(0x6B); // Registrador PWR_MGMT_1
  Wire.write(0x00); // Acorda o MPU
  Wire.endTransmission(true);

  Wire.beginTransmission(_mpu_i2c_address);
  Wire.write(0x1C); // Registrador ACCEL_CONFIG
  Wire.write(0x00); // Faixa Acelerômetro: ±2g
  Wire.endTransmission(true);

  Wire.beginTransmission(_mpu_i2c_address);
  Wire.write(0x1B); // Registrador GYRO_CONFIG
  Wire.write(0x00); // Faixa Giroscópio: ±250°/s
  Wire.endTransmission(true);

  // Se for um MPU-9250, inicializa o magnetômetro
  if (_sensorType == TYPE_MPU9250) {
    _initAK8963();
  }
  delay(20);
}

// Inicializa o magnetômetro AK8963
void IMU_Sensor::_initAK8963() {
  selectMuxPort();
  // Habilita o modo "passthrough" para a I2C auxiliar, permitindo acesso ao magnetômetro
  Wire.beginTransmission(_mpu_i2c_address);
  Wire.write(0x37); // Registrador INT_PIN_CFG
  Wire.write(0x02); // Seta o bit BYPASS_EN
  Wire.endTransmission();

  // Configura o magnetômetro para leituras contínuas de 16-bit a 100Hz
  Wire.beginTransmission(_mag_address);
  Wire.write(0x0A); // Registrador de Controle 1 (CNTL1)
  Wire.write(0x16); // Modo de fusão de 16-bit, 100Hz
  Wire.endTransmission();
}

void IMU_Sensor::_readAcel() {
  selectMuxPort();
  Wire.beginTransmission(_mpu_i2c_address);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom(_mpu_i2c_address, 6, true);
  _AccX = (int16_t)(Wire.read() << 8 | Wire.read()) / 16384.0f;
  _AccY = (int16_t)(Wire.read() << 8 | Wire.read()) / 16384.0f;
  _AccZ = (int16_t)(Wire.read() << 8 | Wire.read()) / 16384.0f;
}

void IMU_Sensor::_readGiro() {
  selectMuxPort();
  Wire.beginTransmission(_mpu_i2c_address);
  Wire.write(0x43);
  Wire.endTransmission(false);
  Wire.requestFrom(_mpu_i2c_address, 6, true);
  _GyroX = (int16_t)(Wire.read() << 8 | Wire.read()) / 131.0f;
  _GyroY = (int16_t)(Wire.read() << 8 | Wire.read()) / 131.0f;
  _GyroZ = (int16_t)(Wire.read() << 8 | Wire.read()) / 131.0f;
}

void IMU_Sensor::_readMag() {
  if (_sensorType != TYPE_MPU9250) return; // Só lê se for MPU9250

  selectMuxPort();
  Wire.beginTransmission(_mag_address);
  Wire.write(0x03); // Começa a leitura do registrador de dados do magnetômetro
  Wire.endTransmission();
  Wire.requestFrom(_mag_address, 7); // Pede 7 bytes de dados

  int16_t rawMagX = (int16_t)(Wire.read() | Wire.read() << 8);
  int16_t rawMagY = (int16_t)(Wire.read() | Wire.read() << 8);
  int16_t rawMagZ = (int16_t)(Wire.read() | Wire.read() << 8);
  
  // Aplica a calibração (bias e scale)
  _MagX = ((float)rawMagX - _magBias[0]) * _magScale[0];
  _MagY = ((float)rawMagY - _magBias[1]) * _magScale[1];
  _MagZ = ((float)rawMagZ - _magBias[2]) * _magScale[2];
}

void IMU_Sensor::calibrate(int num_readings) {
  _AccErrorX = 0; _AccErrorY = 0;
  _GyroErrorX = 0; _GyroErrorY = 0; _GyroErrorZ = 0;

  for (int i = 0; i < num_readings; i++) {
    _readAcel();
    _AccErrorX += (atan(_AccY / sqrt(pow(_AccX, 2) + pow(_AccZ, 2))) * 180 / PI);
    _AccErrorY += (atan(-1 * _AccX / sqrt(pow(_AccY, 2) + pow(_AccZ, 2))) * 180 / PI);
    _readGiro();
    _GyroErrorX += _GyroX;
    _GyroErrorY += _GyroY;
    _GyroErrorZ += _GyroZ;
    delay(1);
  }
  _AccErrorX /= num_readings;
  _AccErrorY /= num_readings;
  _GyroErrorX /= num_readings;
  _GyroErrorY /= num_readings;
  _GyroErrorZ /= num_readings;
}

void IMU_Sensor::loop() {
  // --- Cálculo de Tempo ---
  _previousTime = _currentTime;
  _currentTime = micros();
  _elapsedTime = (_currentTime - _previousTime) / 1000000.0f;

  // --- Leitura e Cálculo de Roll e Pitch (Filtro Complementar) ---
  _readAcel();
  _accAngleX = (atan(_AccY / sqrt(pow(_AccX, 2) + pow(_AccZ, 2))) * 180 / PI) - _AccErrorX;
  _accAngleY = (atan(-1 * _AccX / sqrt(pow(_AccY, 2) + pow(_AccZ, 2))) * 180 / PI) - _AccErrorY;

  _readGiro();
  _GyroX -= _GyroErrorX;
  _GyroY -= _GyroErrorY;
  _GyroZ -= _GyroErrorZ;

  _gyroAngleX += _GyroX * _elapsedTime;
  _gyroAngleY += _GyroY * _elapsedTime;

  _roll = 0.96 * _gyroAngleX + 0.04 * _accAngleX;
  _pitch = 0.96 * _gyroAngleY + 0.04 * _accAngleY;

  // --- Cálculo de Yaw (com correção do Magnetômetro) ---
  if (_sensorType == TYPE_MPU9250) {
    _readMag();
    
    // Tilt Compensation: corrige a leitura do magnetômetro com base na inclinação (roll/pitch)
    float magX_comp = _MagX * cos(radians(_pitch)) + _MagY * sin(radians(_roll)) * sin(radians(_pitch)) - _MagZ * cos(radians(_roll)) * sin(radians(_pitch));
    float magY_comp = _MagY * cos(radians(_roll)) + _MagZ * sin(radians(_roll));
    
    // Calcula o heading (Yaw) a partir do magnetômetro compensado
    float magYaw = atan2(magY_comp, magX_comp) * 180 / PI;

    _gyroAngleZ += _GyroZ * _elapsedTime; // Integra o giroscópio
    
    // Filtro complementar para o Yaw: funde o giroscópio com o magnetômetro
    _yaw = 0.95 * _gyroAngleZ + 0.05 * magYaw;

  } else {
    // Para o MPU-6050, o Yaw é apenas o giroscópio (sofrerá drift)
    _gyroAngleZ += _GyroZ * _elapsedTime;
    _yaw = _gyroAngleZ;
  }
}

void IMU_Sensor::setMagCalibration(float magBiasX, float magBiasY, float magBiasZ, float magScaleX, float magScaleY, float magScaleZ) {
    _magBias[0] = magBiasX;
    _magBias[1] = magBiasY;
    _magBias[2] = magBiasZ;
    _magScale[0] = magScaleX;
    _magScale[1] = magScaleY;
    _magScale[2] = magScaleZ;
}

void IMU_Sensor::reset() {
  _gyroAngleX = 0; _gyroAngleY = 0; _gyroAngleZ = 0;
  _currentTime = micros();
  loop();
}

float IMU_Sensor::getAngleX() { return _roll; }
float IMU_Sensor::getAngleY() { return _pitch; }
float IMU_Sensor::getAngleZ() { return _yaw; }

void IMU_Sensor::selectMuxPort() {
  _tcaselect(_mux_port_number);
}
