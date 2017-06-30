/**
   ----------------------------------------------------------------------------
   FEDERAL UNIVERSITY OF UBERLANDIA
   Faculty of Electrical Engineering
   Uberlândia, Brazil
   ----------------------------------------------------------------------------
   Authors: Italo G S Fernandes,
   contact: italogsfernandes@gmail.com
   URL: https://github.com/italogfernandes/cobec-project
   ------------------------------------------------------------------------------
   Description:
   Este projeto se trata de um sistema de coleta para o rastreamento da posição
   do pé durante a marcha.
   O que faz:
      Realiza leitura de 1 sensor inercial e envia via serial port
   ------------------------------------------------------------------------------
   TODO:
     Adicionar Timer de aquisicao confiavel
     Obter offsets do sensor inercial
     Verificar "NOTE" espalhados no codigo.
     Verificar "TODO" espalhados no codigo.
   ------------------------------------------------------------------------------
   Pacotes:
   Inercial(Accel-Gyro): (14 bytes)
   ['$'] [X_AC_H] [X_AC_L] [Y_AC_H] [Y_AC_L] [Z_AC_H] [Z_AC_L]
         [X_GY_H] [X_GY_L] [Y_GY_H] [Y_GY_L] [Z_GY_H] [Y_GY_L] ['\n']

    Esquema de montagem:
    Arduino - Dispositivo
    13      - LED
    A4      - SDA do GY-521
    A5      - SCL do GY-521
    5V      - VCC do GY-521
    GND     - GND do GY-521
   Para visualizar de forma visivel ao ser humano
   Altere o comentario em: #define DEBUG_MODE
*/
//#define OUTPUT_READABLE_ACCELGYRO
#define OUTPUT_BINARY_ACCELGYRO

#include "I2Cdev.h"
#include "MPU6050.h"
#include "Timer.h"

// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation
// is used in I2Cdev.h
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
#include "Wire.h"
#endif

#define LED_PIN 13
#define UART_BAUDRATE 115200
#define PeriodoAquisicao 5 //5ms = 200Hz

Timer t;
int timer_id;
bool blinkState = false;

MPU6050 accelgyro;

int16_t ax, ay, az;
int16_t gx, gy, gz;

const int offsets[6] = {  -1628, -886, 1122, 83,   -35,  21}; //TODO calculate
String serialOp;

void setup() {
  pinMode(LED_PIN, OUTPUT);
  Wire.begin();
  Serial.begin(UART_BAUDRATE);
  iniciar_sensor();
  Serial.println("Envie um comando: CMDSTART, CMDSTOP...");
  timer_id = t.every(PeriodoAquisicao, ler_sensor);

}

void loop() {
  //Menu
  if (Serial.available() > 0) {
    serialOp = Serial.readString();
    if (serialOp == "CMDSTART") {
      Serial.print("['$'] [X_AC_H] [X_AC_L] [Y_AC_H] [Y_AC_L] [Z_AC_H] [Z_AC_L] [X_GY_H] [X_GY_L] [Y_GY_H] [Y_GY_L] [Z_GY_H] [Y_GY_L] ['\n']");
      timer_id = t.every(PeriodoAquisicao, ler_sensor);
    }
    else if (serialOp == "CMDSTOP") {
      t.stop(timer_id);
      Serial.print("************************************************************************************************************************");
      digitalWrite(LED_PIN, LOW);
    }
  }
  t.update();
}

void iniciar_sensor() {
  if (accelgyro.testConnection()) {
    accelgyro.initialize(); //Initializes the IMU
    accelgyro.setFullScaleGyroRange(MPU6050_GYRO_FS_500); //+-500degress/s
    accelgyro.setFullScaleAccelRange(MPU6050_ACCEL_FS_4); //+- 4g
    accelgyro.setXAccelOffset(offsets[0]); accelgyro.setYAccelOffset(offsets[1]); accelgyro.setZAccelOffset(offsets[2]);
    accelgyro.setXGyroOffset(offsets[3]); accelgyro.setYGyroOffset(offsets[4]); accelgyro.setZGyroOffset(offsets[5]);
    Serial.print("Sensor configurado com sucesso.\n");
  } else {
    Serial.print("Erro na conexao do sensor.\n");
  }
}

void ler_sensor() {
  accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
  enviar_dados();
  blinkState = !blinkState;
  digitalWrite(LED_PIN, blinkState);
}

void enviar_dados() {
#ifdef OUTPUT_READABLE_ACCELGYRO
  // display tab-separated accel/gyro x/y/z values
  //Serial.print("a/g:\t");
  Serial.print(ax / 16384.00,5);  Serial.print(" ");  Serial.print(ay / 16384.00,5); Serial.print(" ");  Serial.print(az / 16384.00,5); Serial.print(" ");
  Serial.print(gx / 131.00,5);    Serial.print(" ");  Serial.print(gy / 131.00,5);   Serial.print(" ");  Serial.print(gz / 131.00,5);
  Serial.print("\n");
#endif
#ifdef OUTPUT_BINARY_ACCELGYRO
  Serial.write(0x24);
  Serial.write((uint8_t)(ax >> 8)); Serial.write((uint8_t)(ax & 0xFF));
  Serial.write((uint8_t)(ay >> 8)); Serial.write((uint8_t)(ay & 0xFF));
  Serial.write((uint8_t)(az >> 8)); Serial.write((uint8_t)(az & 0xFF));
  Serial.write((uint8_t)(gx >> 8)); Serial.write((uint8_t)(gx & 0xFF));
  Serial.write((uint8_t)(gy >> 8)); Serial.write((uint8_t)(gy & 0xFF));
  Serial.write((uint8_t)(gz >> 8)); Serial.write((uint8_t)(gz & 0xFF));
  Serial.write(0x0A);
#endif
}

