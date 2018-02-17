/* UNIVERSIDADE FEDERAL DE UBERLANDIA
   BIOLAB - Biomedical Engineering Lab

   Autor: Ítalo G S Fernandes
   contact: italogsfernandes@gmail.com
   Git: https://github.com/italogfernandes/
   This code is part of the real time gate tracking project.
   It reads a mpu6050 imu and send the through the serial port (usb).
   It can also be usued with a hc-05 bluetooth module.
   Packets:
   Default:
   [Starter] [Quat] [Accel] [Gyro] [End]
   More descritive way:
   ['$'] | [WH] [WL] [XH] [XL] [YH] [YL] [ZH] [ZL] | [AcXH] [AcXL] ... ['\n']

    Hardware Connections:
    Arduino - Device
    13      - LED
    A4      - SDA do GY-521
    A5      - SCL do GY-521
    5V      - VCC do GY-521
    GND     - GND do GY-521

    For easy visualization uncomment the DEBUG_MODE define.
    So the output will be in ASCII instead of binary data.
*/
#define ACCEL_GYRO_FROM_DMP //If enabled the accel and gyro data will come from the DMP
#define DEBUG_MODE

//Se estiver no modo debug printa as msg debug, se nao estiver nao printa
#ifdef DEBUG_MODE
#define DEBUG_PRINT_(x) Serial.print(x)
#endif
#ifndef DEBUG_MODE
#define DEBUG_PRINT_(x)
#endif

//////////////
// Libraries//
//////////////
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"

// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation
// is used in I2Cdev.h
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
#include "Wire.h"
#endif

///////////////
// Constants //
///////////////
#define LED_PIN 13

#define UART_BAUDRATE 115200
#define PACKET_START  '$'
#define PACKET_END    '\n'

#define PSDMP 42 //Packet Size DMP - tam do pacote interno da mpu-6050

///////////////////////
// Functions Headers //
///////////////////////
void init_inertial_sensor();
void read_inertial_sensor();
void send_packet();

/////////////////
// Global Data //
/////////////////
// Application Control
char serialOp; // Storages the cmd send by serial interface (start,stop,etc)
bool acquisition_running = true; // flag indication if the acquisition is running or not

MPU6050 mpu(0x68); // MPU6050 object
const int offsets[6] = { -1196,  -88, 445, 73,  -46, 22}; // Callibration Offsets, you should calculate this with the calibration sketch (github.com/italogfernandes/libraries)
uint8_t fifoBuffer[42]; // FIFO storages fifoBuffer of mpu
int numbPackets; // Amount of packets to be read in the mpu buffer

///////////
// SETUP //
///////////
void setup() {
  //GPIO:
  pinMode(LED_PIN, OUTPUT);

  //Serial Communication:
  Serial.begin(UART_BAUDRATE);
  DEBUG_PRINT_("Available commands: s - Start, q - stop.\n");

  //Inertial Sensor
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
  Wire.begin();
  Wire.setClock(400000); // Note: Sometimes it did'nt work very well
#elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
  Fastwire::setup(400, true);
#endif

  init_inertial_sensor();
}

//////////
// LOOP //
//////////
void loop() {
  //Menu
  if (Serial.available() > 0) {
    serialOp = Serial.read();
    if (serialOp == 's') {
      digitalWrite(LED_PIN, HIGH);
      mpu.resetFIFO();
      delay(5);
      acquisition_running = true;
    } else if (serialOp == 'q') {
      digitalWrite(LED_PIN, LOW);
      acquisition_running = false;
    }
  }

  // Acquisition
  if (acquisition_running) {
    read_inertial_sensor(); // Read and send the data from the inertial sensor
  }
}

////////////////////
//Sensor Inercial //
////////////////////

/**
  Firt setup of the inertial sensor.
  Only works if the connection recognizes the correct device id.
  First it initilizes the MPU for continuous measuriments.
  So it will configure the DMP (Digital Motion Processor)
  and the offsets.
*/
void init_inertial_sensor() {
  if (mpu.testConnection()) {
    mpu.initialize(); //Initializes the IMU
    uint8_t ret = mpu.dmpInitialize(); //Initializes the DMP
    delay(50);
    if (ret == 0) {
      mpu.setDMPEnabled(true);
      mpu.setXAccelOffset(offsets[0]); mpu.setYAccelOffset(offsets[1]); mpu.setZAccelOffset(offsets[2]);
      mpu.setXGyroOffset(offsets[3]); mpu.setYGyroOffset(offsets[4]); mpu.setZGyroOffset(offsets[5]);
      DEBUG_PRINT_("Sensor Inercial configurado com sucesso.\n");
    } else {
      //TODO: adicionar uma forma melhor de aviso. outro led?
      DEBUG_PRINT_("Erro na inicializacao do sensor Inercial!\n");
    }
  } else {
    DEBUG_PRINT_("Erro na conexao do sensor Inercial.\n");
  }
}

/**
   Verifies the amount of packets in the inertial sensor fifo,
   and read every one.
*/
void read_inertial_sensor() {
  numbPackets = floor(mpu.getFIFOCount() / PSDMP);
  if (numbPackets >= 24) {
    mpu.resetFIFO();
    DEBUG_PRINT_("FIFO sensor 1 overflow!\n"); //NOTE: may put a led for indicating is a good idea
  } else {
    while (numbPackets > 0) {
      DEBUG_PRINT_(numbPackets); DEBUG_PRINT_(" - ");
      numbPackets--;
      mpu.getFIFOBytes(fifoBuffer, PSDMP);
#ifndef ACCEL_GYRO_FROM_DMP
      int16_t ax, ay, az, gx, gy, gz;
      mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
      fifoBuffer[28] = (uint8_t) (ax >> 8); //ac_x_msb
      fifoBuffer[29] = (uint8_t) (ax && 0x00FF); //ac_x_lsb

      fifoBuffer[32] = (uint8_t) (ay >> 8); //ac_y_msb
      fifoBuffer[33] = (uint8_t) (ay && 0x00FF); //ac_y_lsb

      fifoBuffer[36] = (uint8_t) (az >> 8); //ac_z_msb
      fifoBuffer[37] = (uint8_t) (az && 0x00FF); //ac_z_lsb

      fifoBuffer[16] = (uint8_t) (gx >> 8); //gy_x_msb
      fifoBuffer[17] = (uint8_t) (gx && 0x00FF); //gy_x_lsb

      fifoBuffer[20] = (uint8_t) (gy >> 8); //gy_y_msb
      fifoBuffer[21] = (uint8_t) (gy && 0x00FF); //gy_y_lsb

      fifoBuffer[24] = (uint8_t) (gz >> 8); //gy_z_msb
      fifoBuffer[25] = (uint8_t) (gz && 0x00FF); //gy_z_lsb
#endif
      send_packet();
    }
  }
}

/**
   Assembles the packets.
*/
void send_packet() {
#ifndef DEBUG_MODE
  //Assembling packet and sending
  Serial.write(PACKET_START);
  Serial.write(fifoBuffer[0]); //qw_msb
  Serial.write(fifoBuffer[1]); //qw_lsb
  Serial.write(fifoBuffer[4]); //qx_msb
  Serial.write(fifoBuffer[5]); //qx_lsb
  Serial.write(fifoBuffer[8]); //qy_msb
  Serial.write(fifoBuffer[9]); //qy_lsb
  Serial.write(fifoBuffer[12]); //qz_msb
  Serial.write(fifoBuffer[13]); //qz_lsb
  Serial.write(fifoBuffer[28]); //ac_x_msb
  Serial.write(fifoBuffer[29]); //ac_x_lsb
  Serial.write(fifoBuffer[32]); //ac_y_msb
  Serial.write(fifoBuffer[33]); //ac_y_lsb
  Serial.write(fifoBuffer[36]); //ac_z_msb
  Serial.write(fifoBuffer[37]); //ac_z_lsb
  Serial.write(fifoBuffer[16]); //gy_x_msb
  Serial.write(fifoBuffer[17]); //gy_x_lsb
  Serial.write(fifoBuffer[20]); //gy_y_msb
  Serial.write(fifoBuffer[21]); //gy_y_lsb
  Serial.write(fifoBuffer[24]); //gy_z_msb
  Serial.write(fifoBuffer[25]); //gy_z_lsb
  Serial.write(PACKET_END);
#endif /*DEBUG_MODE*/
#ifdef DEBUG_MODE
  float q[4], a[3], g[3];
  //Quaternion
  q[0] = (float) ((fifoBuffer[0] << 8) | fifoBuffer[1]) / 16384.0f;
  q[1] = (float) ((fifoBuffer[4] << 8) | fifoBuffer[5]) / 16384.0f;
  q[2] = (float) ((fifoBuffer[8] << 8) | fifoBuffer[9]) / 16384.0f;
  q[3] = (float) ((fifoBuffer[12] << 8) | fifoBuffer[13]) / 16384.0f;
  //Aceleracao
  a[0] = (float) ((fifoBuffer[28] << 8) | fifoBuffer[29]) / 8192.0f;
  a[1] = (float) ((fifoBuffer[32] << 8) | fifoBuffer[33]) / 8192.0f;
  a[2] = (float) ((fifoBuffer[36] << 8) | fifoBuffer[37]) / 8192.0f;
  //Giroscopio
  g[0] = (float) ((fifoBuffer[16] << 8) | fifoBuffer[17]) / 131.0f;
  g[1] = (float) ((fifoBuffer[20] << 8) | fifoBuffer[21]) / 131.0f;
  g[2] = (float) ((fifoBuffer[24] << 8) | fifoBuffer[25]) / 131.0f;

  DEBUG_PRINT_("Quat - Accel - Gyro: \t");
  //Quaternions
  DEBUG_PRINT_(q[0]);
  DEBUG_PRINT_("\t");
  DEBUG_PRINT_(q[1]);
  DEBUG_PRINT_("\t");
  DEBUG_PRINT_(q[2]);
  DEBUG_PRINT_("\t");
  DEBUG_PRINT_(q[3]);
  DEBUG_PRINT_("\t - \t");
  //accel in G
  DEBUG_PRINT_(a[0]);
  DEBUG_PRINT_("\t");
  DEBUG_PRINT_(a[1]);
  DEBUG_PRINT_("\t");
  DEBUG_PRINT_(a[2]);
  DEBUG_PRINT_("\t - \t");
  //gyro in degrees/s
  DEBUG_PRINT_(g[0]);
  DEBUG_PRINT_("\t");
  DEBUG_PRINT_(g[1]);
  DEBUG_PRINT_("\t");
  DEBUG_PRINT_(g[2]);
  DEBUG_PRINT_("\n");
#endif /*DEBUG_MODE*/
}



