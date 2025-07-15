#ifndef IMU_Sensor_h
#define IMU_Sensor_h

#include "Arduino.h"
#include <Wire.h>

// Enum para identificar o tipo de sensor
enum IMU_TYPE {
  TYPE_MPU6050,
  TYPE_MPU9250
};

class IMU_Sensor {
public:
  // Construtor modificado para aceitar o tipo de sensor
  IMU_Sensor(IMU_TYPE sensorType, uint8_t mpu_i2c_address, uint8_t mux_port_number);

  void begin();
  void calibrate(int num_readings);
  void loop();
  void reset();

  // Métodos para aplicar a calibração do magnetômetro (valores obtidos externamente)
  void setMagCalibration(float magBiasX, float magBiasY, float magBiasZ, float magScaleX, float magScaleY, float magScaleZ);

  float getAngleX(); // Roll
  float getAngleY(); // Pitch
  float getAngleZ(); // Yaw

private:
  void selectMuxPort();
  void _readAcel();
  void _readGiro();
  void _readMag(); // Novo método para ler o magnetômetro
  void _initAK8963(); // Novo método para inicializar o magnetômetro do MPU9250

  const IMU_TYPE _sensorType;
  const int _mpu_i2c_address;
  const uint8_t _mux_port_number;
  
  // Endereço do magnetômetro dentro do MPU9250
  const uint8_t _mag_address = 0x0C;

  // Variáveis de estado
  float _AccX, _AccY, _AccZ;
  float _GyroX, _GyroY, _GyroZ;
  float _MagX, _MagY, _MagZ; // Variáveis para o magnetômetro

  // Variáveis de calibração
  float _AccErrorX, _AccErrorY;
  float _GyroErrorX, _GyroErrorY, _GyroErrorZ;
  float _magBias[3] = {0, 0, 0};   // Bias do magnetômetro
  float _magScale[3] = {1, 1, 1}; // Escala do magnetômetro

  // Variáveis de ângulo
  float _accAngleX, _accAngleY;
  float _gyroAngleX, _gyroAngleY, _gyroAngleZ;
  float _roll, _pitch, _yaw;
  
  // Variáveis de tempo
  float _elapsedTime, _currentTime, _previousTime;
};

#endif
