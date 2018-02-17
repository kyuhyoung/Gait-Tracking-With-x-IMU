/* UNIVERSIDADE FEDERAL DE UBERLANDIA
   Biomedical Engineering Lab
   Autor: Ítalo G S Fernandes
   contact: italogsfernandes@gmail.com
   URLs: https://github.com/italogfernandes/

   This code is part of the real time gate tracking project.
   It reads a mpu6050 imu and send the through the serial port (usb).
   It communicates with one nrf24le1 module properly configured.
   Packets:
   Default:
   [Starter] [Quat] [Accel] [Gyro] [Mag] [End]
   More descritive way:
   ['$'] | [WH] [WL] [XH] [XL] [YH] [YL] [ZH] [ZL] | [AcXH] [AcXL] ... ['\n']

    Hardware Connections:
    Arduino - Dispositivo
    7       - Led RGB Vermelho
    6       - Led RGB Verde
    5       - Led RGB Azul
    GND     - GND comum

    3.3V      - VCC do nrf24le01
    ICSP6/GND - Gnd do nrf24le01
    3       - CSN do nrf24le01
    4       - CE do nrf24le01
    ICSP4/MOSI - MOSI do nrf24le01
    ICSP3/SCK - SCK do nrf24le01
    2       - IRQ do nrf24le01
    ICSP1/MISO - MISO do nrf24le01

    Comandos:
 	 CMD_i2c_mpu_writeBytes:

 					Pacote:
 						SUB_ADDR | CMD | regAddr | data_len | DATA_1 | ... | DATA_N
 					Exemplo:

 	 CMD_i2c_mpu_readBytes:
 					Pacote:
 						SUB_ADDR | CMD | regAddr | data_len
                     Resposta:
                         SUB_ADDR | CMD | DATA_1 | ... | DATA_N
 					Exemplo:

       CMD_hal_w2_write:

 					Pacote:
 						SUB_ADDR | CMD | address | data_len | DATA_1 | ... | DATA_N
 					Exemplo:

 	 CMD_hal_w2_read:
 					Pacote:
 						SUB_ADDR | CMD | address | data_len
                     Resposta:
                         SUB_ADDR | CMD | DATA_1 | ... | DATA_N
 					Exemplo:

       CMD_FMW_VERSION:
 					Retorna a versao do firmware, util para handshake.
 					Pacote:
 						SUB_ADDR | CMD
                     Resposta:
                         SUB_ADDR | CMD | FIRMWARE_VERSION
 					Exemplo:

       CMD_STATUS_LED
                     Coloca o led de status no nivel definido
                     ESTADO = 0 -> apagado; 1 -> acesso; 2 -> Inverter; 3 -> Do nothing
                     Pacote:
                         SUB_ADDR | CMD | ESTADO
                     Resposta:
                         SUB_ADDR | CMD | ESTADO

    For easy visualization uncomment the DEBUG_MODE define.
    So the output will be in ASCII instead of binary data.
*/
//////////
//Debug //
//////////
#define DEBUG_MODE
#ifdef DEBUG_MODE
#define DEBUG_PRINT(x) Serial.print(x)
#define DEBUG_PRINTLN(x) Serial.println(x)
#define DEBUG_WRITE(x,y) Serial.write(x,y)
#define DEBUG_PRINT_HEX(x) Serial.print(x,HEX)
#endif
#ifndef DEBUG_MODE
#define DEBUG_PRINT(x)
#define DEBUG_PRINTLN(x)
#define DEBUG_WRITE(x,y)
#define DEBUG_PRINT_HEX(x)
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

//////////////
//Libraries //
//////////////
#include "nrf24le01Module.h"
#include "led_rgb.h"
#include "REG_MPU6050.h"

////////////
//Defines //
////////////
//COMANDOS da SPI-RF-BRIDGE
#define CMD_i2c_mpu_writeBytes  0x01
#define CMD_i2c_mpu_readBytes   0x02
#define CMD_hal_w2_write        0x03
#define CMD_hal_w2_read         0x04
#define CMD_FMW_VERSION  		0x05
#define CMD_STATUS_LED  		0x06

//////////////
//Constants //
//////////////
#define FIRMWARE_VERSION        0x01

//Protocolo
#define INDEX_SUB_ADDR  0
#define INDEX_CMD       1
#define INDEX_ARG1      2
#define INDEX_ARG2      3
#define INDEX_ARG3      4
//Pinos
#define LED_RED     7
#define LED_GREEN   6
#define LED_BLUE    5
#define NRF_CSN     3
#define NRF_CE      4
#define NRF_IRQ     2
//UART
#define UART_BAUDRATE 115200

////////////////
//Global Data //
////////////////
uint8_t sensor_addrs[1] = {0x00};
led_rgb status_led(LED_RED, LED_GREEN, LED_BLUE); //R_pin,G_pin,B_pin
nrf24le01Module host_nrf(NRF_IRQ, NRF_CE, NRF_CSN); //IRQ, CE, CSN
bool handshake_ok = false;
char serialOp, serialArg;

unsigned long received_millis, sent_millis , timeout_millis, actual_millis;

void setup() {
  Serial.begin(UART_BAUDRATE);
#ifdef DEBUG_MODE
  SerialUSB.begin(UART_BAUDRATE);
#endif

  status_led.init();
  host_nrf.rf_init(host_nrf.ADDR_HOST, host_nrf.ADDR_HOST, 92, RF_DATA_RATE_2Mbps, RF_TX_POWER_0dBm); //RF Communication
  attachInterrupt(digitalPinToInterrupt(host_nrf.RFIRQ), rf_interrupt, FALLING); //Todo: put inside library
  piscas_iniciais_led();
  Serial.flush();
  DEBUG_PRINTLN("Host Iniciado.");
  mostrar_informacoes_rede();
  DEBUG_PRINTLN("Modo RX.");

  DEBUG_PRINTLN("Commands:");
  DEBUG_PRINTLN("\t 1-Firmware version.");
  DEBUG_PRINTLN("\t 2-Acender Led");
  DEBUG_PRINTLN("\t 3-Apagar Led");
  DEBUG_PRINTLN("\t 4-Inverter Led");
  DEBUG_PRINTLN("\t 5-Status Led");

  DEBUG_PRINTLN("Codificacao:");
  DEBUG_PRINTLN("\t CMD_i2c_mpu_writeBytes  0x01");
  DEBUG_PRINTLN("\t CMD_i2c_mpu_readBytes   0x02");
  DEBUG_PRINTLN("\t CMD_hal_w2_write        0x03");
  DEBUG_PRINTLN("\t CMD_hal_w2_read         0x04");
  DEBUG_PRINTLN("\t CMD_FMW_VERSION      0x05");
  DEBUG_PRINTLN("\t CMD_STATUS_LED      0x06");
}

void mostrar_informacoes_rede() {
  DEBUG_PRINTLN("*****************CONFIGURAÇÕES DA REDE*****************");
  DEBUG_PRINTLN("Tamanho do Endereço: 5.");
  DEBUG_PRINTLN("Endereço RX: 0xE7 | 0xE7 | 0xE7 | 0xE7 | 0xE7.");
  DEBUG_PRINTLN("Endereço TX: 0xE7 | 0xE7 | 0xE7 | 0xE7 | 0xE7.");
  DEBUG_PRINTLN("Frequência: 2.4 GHz.");
  DEBUG_PRINTLN("Canal (0 - 125): 92.");
  DEBUG_PRINTLN("Taxa de Dados (250 Kbps, 1 Mbps, 2 Mbps): 2 Mbps.");
  DEBUG_PRINTLN("Potência (-18 dBm, -12 dBm, -6 dBm, 0dBm): 0dBm.");
  DEBUG_PRINTLN("********************************************************");
}

void loop() {
  actual_millis = millis();
  if (host_nrf.newPayload) {
    received_millis = actual_millis;
    host_nrf.newPayload = 0;
    status_led.acender(LED_COLOR_AQUA);
    Serial.write(host_nrf.rx_buf, host_nrf.payloadWidth);
#ifdef DEBUG_MODE
    DEBUG_PRINTLN("**Payload Recebida**");
    DEBUG_PRINT("Payload Width: "); DEBUG_PRINTLN(host_nrf.payloadWidth);
    DEBUG_PRINT("Payload Data (ASCII): "); DEBUG_WRITE(host_nrf.rx_buf, host_nrf.payloadWidth);
    DEBUG_PRINT("\nPayload Data (HEX): ");
    for (int i = 0; i < host_nrf.payloadWidth; i++) {
      DEBUG_PRINT("0x"); DEBUG_PRINT_HEX(host_nrf.rx_buf[i]); DEBUG_PRINT(" ");
    }
    DEBUG_PRINT("\n");
    int16_t x = host_nrf.rx_buf[2] << 8 | host_nrf.rx_buf[3];
    int16_t y = host_nrf.rx_buf[4] << 8 | host_nrf.rx_buf[5];
    int16_t z = host_nrf.rx_buf[6] << 8 | host_nrf.rx_buf[7];
    
    DEBUG_PRINTLN(x / 16384.0f);
    DEBUG_PRINTLN(y / 16384.0f);
    DEBUG_PRINTLN(z / 16384.0f);
#endif
  }

  if (Serial.available() > 0) {
    serialOp = Serial.read();
    switch (serialOp) {
      case 'h': //handshake
        DEBUG_PRINTLN("Handshake");
        host_nrf.tx_buf[INDEX_SUB_ADDR] = sensor_addrs[0];
        host_nrf.tx_buf[INDEX_CMD] = CMD_FMW_VERSION;
        host_nrf.TX_Mode_NOACK(2);
        DEBUG_PRINTLN("Payload Enviada: 0x00, 0x05");
        break;
      case 'l': //Led
        while (!Serial.available());
        serialArg = Serial.read();
        switch (serialArg) {
          case '0':
            DEBUG_PRINTLN("Apagar led");
            host_nrf.tx_buf[INDEX_SUB_ADDR] = sensor_addrs[0];
            host_nrf.tx_buf[INDEX_CMD] = CMD_STATUS_LED;
            host_nrf.tx_buf[INDEX_ARG1] = 0;
            host_nrf.TX_Mode_NOACK(3);
            DEBUG_PRINTLN("Payload Enviada: 0x00, 0x06, 0");
            break;
          case '1': //Acender led
            DEBUG_PRINTLN("Acender led");
            host_nrf.tx_buf[INDEX_SUB_ADDR] = sensor_addrs[0];
            host_nrf.tx_buf[INDEX_CMD] = CMD_STATUS_LED;
            host_nrf.tx_buf[INDEX_ARG1] = 1;
            host_nrf.TX_Mode_NOACK(3);
            DEBUG_PRINTLN("Payload Enviada: 0x00, 0x06, 1");
            break;
          case '2': //Inverter led
            DEBUG_PRINTLN("Inverter led");
            host_nrf.tx_buf[INDEX_SUB_ADDR] = sensor_addrs[0];
            host_nrf.tx_buf[INDEX_CMD] = CMD_STATUS_LED;
            host_nrf.tx_buf[INDEX_ARG1] = 2;
            host_nrf.TX_Mode_NOACK(3);
            DEBUG_PRINTLN("Payload Enviada: 0x00, 0x06, 2");
            break;
          default: //Status do led
            DEBUG_PRINTLN("Status led");
            host_nrf.tx_buf[INDEX_SUB_ADDR] = sensor_addrs[0];
            host_nrf.tx_buf[INDEX_CMD] = CMD_STATUS_LED;
            host_nrf.tx_buf[INDEX_ARG1] = 3;
            host_nrf.TX_Mode_NOACK(3);
            DEBUG_PRINTLN("Payload Enviada: 0x00, 0x06, 3");
            break;
        } /*SWITCH LED*/
        break;
      case 'i': //device id
        DEBUG_PRINTLN("Get device id");
        host_nrf.tx_buf[INDEX_SUB_ADDR] = sensor_addrs[0];
        host_nrf.tx_buf[INDEX_CMD] = CMD_i2c_mpu_readBytes;
        host_nrf.tx_buf[INDEX_ARG1] = MPU6050_RA_WHO_AM_I;
        host_nrf.tx_buf[INDEX_ARG2] = 1;
        host_nrf.TX_Mode_NOACK(4);
        DEBUG_PRINTLN("Payload Enviada: 0x00, CMD_i2c_mpu_readBytes, MPU6050_RA_WHO_AM_I, 1");
        break;
      case 'r': //read device memory
        DEBUG_PRINTLN("read device memory");
        host_nrf.tx_buf[INDEX_SUB_ADDR] = sensor_addrs[0];
        host_nrf.tx_buf[INDEX_CMD] = CMD_i2c_mpu_readBytes;
        host_nrf.tx_buf[INDEX_ARG1] = 0x00;
        host_nrf.tx_buf[INDEX_ARG2] = 16;
        host_nrf.TX_Mode_NOACK(4);
        DEBUG_PRINTLN("Payload Enviada: 0x00, CMD_i2c_mpu_readBytes, 0x00, 16");
        break;
      case 'a': //accel
        DEBUG_PRINTLN("read device memory");
        host_nrf.tx_buf[INDEX_SUB_ADDR] = sensor_addrs[0];
        host_nrf.tx_buf[INDEX_CMD] = CMD_i2c_mpu_readBytes;
        host_nrf.tx_buf[INDEX_ARG1] = 0x3B;
        host_nrf.tx_buf[INDEX_ARG2] = 6;
        host_nrf.TX_Mode_NOACK(4);
        DEBUG_PRINTLN("Payload Enviada: 0x00,CMD_i2c_mpu_readBytes, 0x3B, 6");
        break;
      case '0':
        //SUB_ADDR | CMD | regAddr | data_len | DATA_1 | ... | DATA_N
        DEBUG_PRINTLN("pwrm1 = 0");
        host_nrf.tx_buf[INDEX_SUB_ADDR] = sensor_addrs[0];
        host_nrf.tx_buf[INDEX_CMD] = CMD_i2c_mpu_writeBytes;
        host_nrf.tx_buf[INDEX_ARG1] = 0x6B;
        host_nrf.tx_buf[INDEX_ARG2] = 1;
        host_nrf.tx_buf[INDEX_ARG3] = 0x00;
        host_nrf.TX_Mode_NOACK(5);
        DEBUG_PRINTLN("Payload Enviada: 0x00,CMD_i2c_mpu_writeBytes, 0x6B, 1, 0x00");
        break;
      default:
        break;
    }
  } /*If Serial Available*/


  //Pisca leds
  if (actual_millis - received_millis  > 250) {
    status_led.apagar();
  }
  if (actual_millis - sent_millis > 250) {
    status_led.apagar();
  }
  //if (!digitalRead(2)) {
  //  host_nrf.RF_IRQ();
  //}
}

/**
   Interrupcao do rf
*/
void rf_interrupt() {
  host_nrf.RF_IRQ();
}

/**
   Envia uma string pela interface rf
   @param str2send string para ser enviada
*/
void rfSendString(const char * str2send) {
  int i = 0;
  for (i = 0; str2send[i] != '\0'; i++) {
    host_nrf.tx_buf[i] = str2send[i];
  }
  host_nrf.TX_Mode_NOACK(i);
  sent_millis = actual_millis;
  status_led.acender(LED_COLOR_YELLOW);
}

/**
   Sequencia de inicializacao.
*/
void piscas_iniciais_led() {
  status_led.acender(LED_COLOR_RED); delay(250);
  status_led.apagar(); delay(100);
  status_led.acender(LED_COLOR_GREEN); delay(250);
  status_led.apagar(); delay(100);
  status_led.acender(LED_COLOR_BLUE); delay(250);
  status_led.apagar(); delay(100);
}
