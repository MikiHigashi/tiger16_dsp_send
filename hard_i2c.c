#include <xc.h>
#include <stdio.h>
#include <string.h>

#define FCY 69784687UL
#include <libpic30.h>
#include "hard_i2c.h"
#include "mcc_generated_files/mcc.h"


#define TIMEOUT_CNT 300


void IdleI2C1(void)
{
	while(I2C1CON1bits.SEN || I2C1CON1bits.PEN || I2C1CON1bits.RCEN || 
		I2C1CON1bits.RSEN || I2C1CON1bits.ACKEN || I2C1STATbits.TRSTAT);
}


// ==================== I2C Start =============================
void I2C_start() {
    uint16_t cnt = 0;
	i2c1_driver_start();
    for (cnt=0; cnt<TIMEOUT_CNT; cnt++) {
        if (I2C1CONLbits.SEN == 0) return;
        __delay_us(1);
    }
}

// ==================== I2C Stop ==============================
void I2C_stop() {
    uint16_t cnt = 0;
    i2c1_driver_stop();
    for (cnt=0; cnt<TIMEOUT_CNT; cnt++) {
        if (I2C1CONLbits.PEN == 0) return;
        __delay_us(1);
    }
}

// ==================== I2C Send ==============================
void I2C_send(unsigned char send_data) {
    uint16_t cnt = 0;
    
    for (cnt=0; cnt<TIMEOUT_CNT; cnt++) {
        I2C1STATbits.IWCOL = 0;
        i2c1_driver_TXData(send_data);
        if (I2C1STATbits.IWCOL == 0) break;
        __delay_us(1);
    }
    if (cnt>=TIMEOUT_CNT) {
    	i2c1_driver_stop();
        return;
    }

    for (cnt=0; cnt<TIMEOUT_CNT; cnt++) {
        if (I2C1STATbits.TBF) break;
        __delay_us(1);
    }
    if (cnt>=TIMEOUT_CNT) {
    	i2c1_driver_stop();
        return;
    }
}

// ==================== I2C Recive ============================
unsigned char I2C_rcv() {
	return 0;
}


// ==================== I2C ACK check =========================
unsigned char I2C_ackchk() {
    uint16_t cnt = 0;

    for (cnt=0; cnt<TIMEOUT_CNT; cnt++) {
        if (I2C1STATbits.TRSTAT == 0) break;
        __delay_us(1);
    }
    if (cnt>=TIMEOUT_CNT) {
    	i2c1_driver_stop();
        return 0;
    }

    if (i2c1_driver_isNACK()) {
        return 1;
    }
    return 0;
}

// ==================== I2C ACK send ==========================
void I2C_acksnd() {
}

// ==================== I2C NACK send =========================
void I2C_nacksnd() {
}

