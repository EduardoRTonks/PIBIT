#ifndef MPU6050_Sensor_h
#define MPU6050_Sensor_h

#include "Arduino.h"
#include <Wire.h> // A biblioteca Wire será usada internamente pela classe

class MPU6050_Sensor {
public:
  // Construtor: Passa o endereço I2C do MPU (geralmente 0x68 ou 0x69)
  // e o pino do multiplexador onde este MPU está conectado.
  MPU6050_Sensor(uint8_t mpu_i2c_address, uint8_t mux_port_number);

  void begin();
  void calibrate(int num_readings);
  void loop(); // Para ler dados e calcular ângulos continuamente
  void reset();

  // Métodos para obter os ângulos calculados
  float getAngleX();
  float getAngleY();
  float getAngleZ();

  // Método para setar a porta do multiplexador para este sensor
  // Esta função é chamada internamente por outros métodos da classe.
  void selectMuxPort();

private:
  const int _mpu_i2c_address; // Endereço I2C do MPU6050 (0x68 ou 0x69)
  const uint8_t _mux_port_number; // Porta do multiplexador (0-7)

  // Variáveis de estado para CADA MPU
  float _AccX, _AccY, _AccZ;
  float _GyroX, _GyroY, _GyroZ;
  float _AccErrorX, _AccErrorY, _GyroErrorX, _GyroErrorY, _GyroErrorZ;
  float _accAngleX, _accAngleY, _gyroAngleX, _gyroAngleY, _gyroAngleZ;
  float _roll, _pitch, _yaw;
  float _elapsedTime, _currentTime, _previousTime;

  // Métodos privados para leitura de dados brutos
  void _readAcel();
  void _readGiro();
};

#endif