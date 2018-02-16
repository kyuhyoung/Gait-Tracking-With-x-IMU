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
   [Starter] [Quat] [Accel] [Gyro] [Mag] [End]
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

///////////
//Timers //
///////////
#define FREQ_AQUIRE          200                    //Frequency in Hz
#define INTERVAL_MS_AQUIRE   1000 / FREQ_AQUIRE     //Interval in milliseconds
#define INTERVAL_US_AQUIRE   1000000 / FREQ_AQUIRE  //Interval in microseconds

// Arduino DUE
#if defined(__arm__) && defined(__SAM3X8E__)
#include<DueTimer.h>
#define SETUP_TIMER()   Timer3.attachInterrupt(timerDataAcq).setFrequency(FREQ_AQUIRE)
#define START_TIMER()   Timer3.start()
#define STOP_TIMER()    Timer3.stop()
#endif

// Arduino UNO, NANO (and others 328P based boards), MEGA e ATtiny85
#if defined(__AVR_ATmega328P__) || defined(__AVR_ATmega2560__) || defined(__AVR_ATtiny85__)
#include<TimerOne.h>
#define SETUP_TIMER()   Timer1.initialize()
#define START_TIMER()   Timer1.attachInterrupt(timerDataAcq,INTERVAL_US_AQUIRE)
#define STOP_TIMER()    Timer1.stop()
#endif

///////////////
// Constants //
///////////////
#define LED_PIN 13


#define UART_BAUDRATE 115200
#define PACKET_START  '$'
#define PACKET_END    '\n'
#define PIN_ADC A0


#define MPUsampFreq 200
#define mpu_interval 10000 //Each 10ms

#define QPMM 10 // Qnt de pontos media movel - Sera feita a media de 10 pontos
#define PSDMP 42 //Packet Size DMP - tam do pacote interno da mpu-6050

#define UART_BAUDRATE 115200
#define UART_START '$' //Inicio do pacote
#define UART_PQUAT 'Q' //Se for um pacote quaternion
#define UART_PEMG 'E' //Se for um pacote emg
#define UART_END '\n' //Fim do pacote

//Variaveis Gerais
uint8_t serial_buffer_out[11];
String serialOp;
//TODO: trocar esses millis por timer
unsigned long currentMillis = 0;
unsigned long previousEMGMillis = 0;
unsigned long previousMPUMillis = 0;
bool aquisition_running = false;

//Variaveis Inercial
MPU6050 mpu(0x68);
const int offsets[6] = {  -602, 2823, 1234, 16,   109,  33};
uint8_t fifoBuffer[42]; // FIFO storage fifoBuffer of mpu
int numbPackets;
//Variaveis EMG
uint16_t valor_media_movel;
uint16_t medial_movel_counter;

void setup() {
  //GPIO:
  pinMode(LED_PIN, OUTPUT); pinMode(EMG_PIN, INPUT);

  //Sensor Inercial
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
  Wire.begin();
  Wire.setClock(200000); //NOTE: Ajustar de acordo com arduino utilizado
#elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
  Fastwire::setup(400, true);
#endif
  iniciar_sensor_inercial();

  //Serial:
  Serial.begin(UART_BAUDRATE);
  DEBUG_PRINT_("Insira um comando: CMDSTART, CMDSTOP.\n");
}

void loop() {
  //Menu
  if (Serial.available() > 0)
  {
    serialOp = Serial.readString();
    if (serialOp == "CMDSTART")
    {
      digitalWrite(LED_PIN, HIGH);
      mpu.resetFIFO();
      delay(5);
      aquisition_running = true; //TODO: Utilizar Timer
    }
    else if (serialOp == "CMDSTOP")
    {
      digitalWrite(LED_PIN, LOW);
      aquisition_running = false;
    }
  }
  //TODO: substituir por funcao takereading num timer
  if (aquisition_running) {
    currentMillis = millis();
    if (currentMillis - previousEMGMillis >= emg_interval) {
      previousEMGMillis = currentMillis;
      //Realiza leitura, media movel e envia pacote(ou mostra) dados
      ler_emg(); //TODO: Revisar
    }
    if (currentMillis - previousMPUMillis >= mpu_interval) {
      previousMPUMillis = currentMillis;
      ler_sensor_inercial(); //Realiza leitura e envia pacote(ou mostra) dados
    }
  }
}


////////////////////
//Sensor Inercial //
////////////////////

void iniciar_sensor_inercial() {
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

void ler_sensor_inercial() {
   numbPackets = floor(mpu.getFIFOCount() / PSDMP);
  DEBUG_PRINT_(numbPackets); DEBUG_PRINT_(" - ");
  if (numbPackets >= 24) {
    mpu.resetFIFO();
    DEBUG_PRINT_("FIFO sensor 1 overflow!\n"); //TODO: mostrar isso de alguma forma. outro led?
  } else {
    while (numbPackets > 0) {
      mpu.getFIFOBytes(fifoBuffer, PSDMP);
      enviar_pacote_inercial();
      numbPackets--;
    }
  }
}

void enviar_pacote_inercial() {
#ifndef DEBUG_MODE
  //Assembling packet and sending
  serial_buffer_out[0] = UART_START;
  serial_buffer_out[1] = UART_PQUAT;
  serial_buffer_out[2] = fifoBuffer[0]; //qw_msb
  serial_buffer_out[3] = fifoBuffer[1]; //qw_lsb
  serial_buffer_out[4] = fifoBuffer[4]; //qx_msb
  serial_buffer_out[5] = fifoBuffer[5]; //qx_lsb
  serial_buffer_out[6] = fifoBuffer[8]; //qy_msb
  serial_buffer_out[7] = fifoBuffer[9]; //qy_lsb
  serial_buffer_out[8] = fifoBuffer[12]; //qz_msb
  serial_buffer_out[9] = fifoBuffer[13]; //qz_lsb
  serial_buffer_out[10] = UART_END;
  Serial.write(serial_buffer_out, 11);
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
  DEBUG_PRINT_("Quat-Accel-Gyro:\t");
  //Quaternions
  DEBUG_PRINT_(q[0]);
  DEBUG_PRINT_("\t");
  DEBUG_PRINT_(q[1]);
  DEBUG_PRINT_("\t");
  DEBUG_PRINT_(q[2]);
  DEBUG_PRINT_("\t");
  DEBUG_PRINT_(q[3]);
  DEBUG_PRINT_("\t-\t");
  //accel in G
  DEBUG_PRINT_(a[0]);
  DEBUG_PRINT_("\t");
  DEBUG_PRINT_(a[1]);
  DEBUG_PRINT_("\t");
  DEBUG_PRINT_(a[2]);
  DEBUG_PRINT_("\t-\t");
  //g[1]ro in degrees/s
  DEBUG_PRINT_(g[0]);
  DEBUG_PRINT_("\t");
  DEBUG_PRINT_(g[1]);
  DEBUG_PRINT_("\t");
  DEBUG_PRINT_(g[2]);
  DEBUG_PRINT_("\n");
#endif /*DEBUG_MODE*/
}

//////////////
//Sinal EMG //
//////////////
void ler_emg() {
  //Esta variavel nao vai estourar pois QPMM = 10. 10*1023 = 10 230 < 65 535
  valor_media_movel +=  analogRead(EMG_PIN); //Lendo a cada 1ms
  medial_movel_counter++;
  if (medial_movel_counter >= QPMM) { // a cada 10ms envia media
    medial_movel_counter = 0; //Reinicia contador
    valor_media_movel /= QPMM; //Divide a media movel
    enviar_pacote_emg();
  }
}

void enviar_pacote_emg() {
#ifndef DEBUG_MODE
  //Assembling packet and sending
  serial_buffer_out[0] = UART_START;
  serial_buffer_out[1] = UART_PEMG;
  serial_buffer_out[2] = (uint8_t) (valor_media_movel >> 8); //emg_msb
  serial_buffer_out[3] = (uint8_t) valor_media_movel; //emg_lsb
  serial_buffer_out[4] = UART_END;
  Serial.write(serial_buffer_out, 5);
#endif /*DEBUG_MODE*/
#ifdef DEBUG_MODE
  DEBUG_PRINT_("EMG:\t");
  DEBUG_PRINT_(valor_media_movel);
  DEBUG_PRINT_("\n");
#endif /*DEBUG_MODE*/
}


