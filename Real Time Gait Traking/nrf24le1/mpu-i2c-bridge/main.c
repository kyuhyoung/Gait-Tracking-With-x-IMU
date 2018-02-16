/* UNIVERSIDADE FEDERAL DE UBERLANDIA
   BIOLAB - Biomedical Engineering Lab
   Autor: Ítalo G S Fernandes
   contact: italogsfernandes@gmail.com
   URLs: https://github.com/italogfernandes
   Description
   Comandos:
	* CMD_i2c_mpu_writeBytes:

					Pacote:
						SUB_ADDR | CMD | regAddr | data_len | DATA_1 | ... | DATA_N
					Exemplo:

	* CMD_i2c_mpu_readBytes:
					Pacote:
						SUB_ADDR | CMD | regAddr | data_len
                    Resposta:
                        SUB_ADDR | CMD | DATA_1 | ... | DATA_N
					Exemplo:

    * CMD_hal_w2_write:

					Pacote:
						SUB_ADDR | CMD | address | data_len | DATA_1 | ... | DATA_N
					Exemplo:

	* CMD_hal_w2_read:
					Pacote:
						SUB_ADDR | CMD | address | data_len
                    Resposta:
                        SUB_ADDR | CMD | DATA_1 | ... | DATA_N
					Exemplo:

    * CMD_FMW_VERSION:
					Retorna a versao do firmware, util para handshake.
					Pacote:
						SUB_ADDR | CMD
                    Resposta:
                        SUB_ADDR | CMD | FIRMWARE_VERSION
					Exemplo:

    * CMD_STATUS_LED
                    Coloca o led de status no nivel definido
                    ESTADO = 0 -> apagado; 1 -> acesso; 2 -> Inverter; 3 -> Do nothing
                    Pacote:
                        SUB_ADDR | CMD | ESTADO
                    Resposta:
                        SUB_ADDR | CMD | ESTADO

*/
//////////////
//Libraries //
//////////////
#include "nrf24le1.h"
#include "reg24le1.h" //Definicoes de muitos endere�os de registradores.
#include "nRF-SPICommands.h" //Comunicacao RF
#include "hal_delay.h" //Delays
#include "hal_w2_isr.h" //I2C
#include "string.h" //Funcao sendString utiliza strlen

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

//Constantes
#define FIRMWARE_VERSION        0x01
#define MY_SUB_ADDR             0x00
#define MPU6050_ADDRESS_AD0_LOW     0x68 // address pin low (GND), default for InvenSense evaluation board
#define MPU6050_ADDRESS_AD0_HIGH    0x69 // address pin high (VCC)
#define MPU6050_DEFAULT_ADDRESS     MPU6050_ADDRESS_AD0_LOW

//Protocolo
#define INDEX_SUB_ADDR  0
#define INDEX_CMD       1
#define INDEX_ARG1      2
#define INDEX_ARG2      3
#define INDEX_ARG3      4
//Pinos
#define STATUS_LED P02

////////////////
//Global Data //
////////////////
bool handshake_ok = false;

///////////////////
//Implementation //
///////////////////
void sendString(const char* msg);
void iniciarIO(void);
void piscar_led(uint8_t repeat, uin16_t delay_time);

void setup() {
    //Pinos
    iniciarIO(); piscar_led(1,250);
    //Radio Frequencia
    rf_init(ADDR_HOST,ADDR_HOST,92,RF_DATA_RATE_2Mbps,RF_TX_POWER_0dBm);
	sendString("\nNRF_V1\n\0"); delay_ms(10); piscar_led(1,250);
    //Interface I2C com os sensores
    hal_w2_configure_master(HAL_W2_100KHZ); piscar_led(1,250);
}

void main(void) {
    setup();
    while(1){ //Loop
        if(!handshake_ok){
            piscar_led(2,500);
        }
		if(newPayload){ //Se recebeu um novo pacote via RF
            sta = 0; newPayload = 0; //Limpa as flags de status
            if(rx_buf[INDEX_SUB_ADDR] == MY_SUB_ADDR){ //verifica se o sinal eh direcionado para mim
                switch(rx_buf[INDEX_CMD]){
                    case CMD_i2c_mpu_writeBytes:
                        i2c_mpu_writeBytes(
                            MPU6050_DEFAULT_ADDRESS,
                            rx_buf[INDEX_ARG1],
                            rx_buf[INDEX_ARG2],
                            &rx_buf[INDEX_ARG3]);
                        break;
                    case CMD_i2c_mpu_readBytes:
                        tx_buf[INDEX_SUB_ADDR] = MY_SUB_ADDR;
                        tx_buf[INDEX_CMD] = CMD_i2c_mpu_readBytes;
                        i2c_mpu_readBytes(
                            MPU6050_DEFAULT_ADDRESS,
                            rx_buf[INDEX_ARG1],
                            rx_buf[INDEX_ARG2],
                            &tx_buf[INDEX_ARG1]);
                        TX_Mode_NOACK(rx_buf[INDEX_ARG2]+2); //Len = Len + 2 bytes
                        break;
                    case CMD_hal_w2_write:
                        hal_w2_write(
                            rx_buf[INDEX_ARG1],
                            &rx_buf[INDEX_ARG3],
                            rx_buf[INDEX_ARG2])
                        break;
                    case CMD_hal_w2_read:
                        tx_buf[INDEX_SUB_ADDR] = MY_SUB_ADDR;
                        tx_buf[INDEX_CMD] = CMD_hal_w2_read;
                        hal_w2_read(
                            rx_buf[INDEX_ARG1],
                            &tx_buf[INDEX_ARG1],
                            rx_buf[INDEX_ARG2]);
                        TX_Mode_NOACK(rx_buf[INDEX_ARG2]+2); //Len = Len + 2 bytes
                        break;
                    case CMD_FMW_VERSION:
                        tx_buf[INDEX_SUB_ADDR] = MY_SUB_ADDR;
                        tx_buf[INDEX_CMD] = CMD_STATUS_LED;
                        tx_buf[INDEX_ARG1] = FIRMWARE_VERSION;
                        TX_Mode_NOACK(3);
                        handshake_ok = true;
                        break;
					case CMD_STATUS_LED:
                        switch (rx_buf[INDEX_ARG1]) {
                            case 0: //Apagar
                                STAUTS_LED = 0;
                                break;
                            case 1: //Acender
                                STATUS_LED = 1;
                                break;
                            case 2: //Inverter
                                STATUS_LED = !STATUS_LED;
                                break;
                            default: //Do Nothing
                                break;
                        }
                        tx_buf[INDEX_SUB_ADDR] = MY_SUB_ADDR;
                        tx_buf[INDEX_CMD] = CMD_STATUS_LED;
                        tx_buf[INDEX_ARG1] = STATUS_LED;
                        TX_Mode_NOACK(3);
                        break;
					default:
                        piscar_led(2,250); //Pisca led 2 vezes indicando erro
                        break;
                } /*switch(rx_buf[INDEX_CMD])*/
            } /*if(rx_buf[INDEX_SUB_ADDR] == MY_SUB_ADDR)*/
        } /*if(newPayload)*/
    } /*while(1)*/
} /*void main(void)*/

/**
* Iniciar os pinos de In e Out
* Padrão = 0 = OUT
* P00 e P01 = IN = Cristal - Built-In
* P02 = 1 = IN = STAUTS_LED
* P04 = 0 = IN = SCL
*/
void iniciarIO(void){
   P0DIR = 0x00;    // Tudo output
   P1DIR = 0x00;    // Tudo output
   P0CON = 0x00;
   P1CON = 0x00;    //Reseting PxCON registers

   P0DIR &= ~(1<<4);//P04 = w2scl = output
   P0DIR &= ~(1<<2);//P02 = Status led = output

   P1CON |= 0x53; // All general I/O 0101 0011
}

/**
 * Pisca o led de status (P02)
 * @param repeat     [description]
 * @param delay_time [description]
 */
void piscar_led( uint8_t repeat, uin16_t delay_time){
    while(repeat>0){
        STATUS_LED = !STATUS_LED; delay_ms(delay_time);
        STATUS_LED = !STATUS_LED; delay_ms(delay_time);
        repeat--;
    }
}

/**
 * Envia uma string como um pacote RF
 * @param msg string a ser enviada
 */
void sendString(const char* msg){
	uint8_t msglen = strlen(msg);
	uint8_t i = 0;
	for(i = 0; i < msglen; i++){
		tx_buf[i] = msg[i];
	}
	TX_Mode_NOACK(msglen);
}

/**
 * Interrupção do I2C - NOTE: Don't know why this should be here, but it works
 */
void I2C_IRQ (void) interrupt INTERRUPT_SERIAL {
    I2C_IRQ_handler();
}
