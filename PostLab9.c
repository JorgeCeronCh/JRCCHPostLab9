/*
 * File:   PostLab9.c
 * Author: jorge
 *
 * Created on 26 de abril de 2022, 11:53 PM
 */
// PIC16F887 Configuration Bit Settings
// 'C' source line config statements
// CONFIG1
#pragma config FOSC = INTRC_NOCLKOUT// Oscillator Selection bits (INTOSC oscillator: CLKOUT function on RA6/OSC2/CLKOUT pin, I/O function on RA7/OSC1/CLKIN)
#pragma config WDTE = OFF       // Watchdog Timer Enable bit (WDT disabled and can be enabled by SWDTEN bit of the WDTCON register)
#pragma config PWRTE = OFF      // Power-up Timer Enable bit (PWRT disabled)
#pragma config MCLRE = OFF      // RE3/MCLR pin function select bit (RE3/MCLR pin function is digital input, MCLR internally tied to VDD)
#pragma config CP = OFF         // Code Protection bit (Program memory code protection is disabled)
#pragma config CPD = OFF        // Data Code Protection bit (Data memory code protection is disabled)
#pragma config BOREN = OFF      // Brown Out Reset Selection bits (BOR disabled)
#pragma config IESO = OFF       // Internal External Switchover bit (Internal/External Switchover mode is disabled)
#pragma config FCMEN = OFF      // Fail-Safe Clock Monitor Enabled bit (Fail-Safe Clock Monitor is disabled)
#pragma config LVP = OFF        // Low Voltage Programming Enable bit (RB3 pin has digital I/O, HV on MCLR must be used for programming)
// CONFIG2
#pragma config BOR4V = BOR40V   // Brown-out Reset Selection bit (Brown-out Reset set to 4.0V)
#pragma config WRT = OFF        // Flash Program Memory Self Write Enable bits (Write protection off)
// #pragma config statements should precede project file includes.
// Use project enums instead of #define for ON and OFF.

#include <xc.h>
#include <stdint.h>             // int8_t, unit8_t

#define _tmr0_value 249         // Valor para 0.015 ms
#define _XTAL_FREQ 4000000
#define IN_MIN 0                
#define IN_MAX 255              // Valores de entrada a Potenciometro
#define OUT_MIN 100
#define OUT_MAX 650             // Valores para el Servomotor

uint8_t bandera;
unsigned short CCPR = 0;
unsigned short CCPR_2 = 0;
uint8_t cont1 = 0;
int PWMTIMER = 0;

void setup(void);
unsigned short map(uint8_t val, uint8_t in_min, uint8_t in_max, 
            unsigned short out_min, unsigned short out_max);

void __interrupt() isr (void){
    if(PIR1bits.ADIF){                                              // Interrupción por ADC
        if(ADCON0bits.CHS == 0){                                    // Interrupción por AN0
            CCPR = map(ADRESH, IN_MIN, IN_MAX, OUT_MIN, OUT_MAX);   // Valor de ancho de pulso
            CCPR1L = (uint8_t)(CCPR>>2);                            // 8 bits mas significativos en CPR1L
            CCP1CONbits.DC1B = CCPR & 0b11;                         // 2 bits menos significativos en DC1B
        }
        else if (ADCON0bits.CHS == 1){
            CCPR_2 = map(ADRESH, IN_MIN, IN_MAX, OUT_MIN, OUT_MAX);   // Valor de ancho de pulso
            CCPR2L = (uint8_t)(CCPR_2>>2);                            // 8 bits mas significativos en CPR1L
            CCP2CONbits.DC2B0 = CCPR_2 & 0b01; 
            CCP2CONbits.DC2B1 = CCPR_2 & 0b10;                        // 2 bits menos significativos en DC2B0 y DC2B1
        }
        else if (ADCON0bits.CHS == 2){
            PWMTIMER = ADRESH;
        }
        PIR1bits.ADIF = 0;              // Limpiar la bandera de ADC
    }
    if(INTCONbits.T0IF){                // Verificar si se activa la bandera de interrupcion de TMR0
        cont1++;
        if (cont1 < PWMTIMER){
            PORTCbits.RC3 = 1;
        }
        else if (PWMTIMER == 0){
            PORTCbits.RC3 = 0;
        }
        else {
            PORTCbits.RC3 = 0;
        }
        INTCONbits.T0IF = 0;            // Limpiar la bandera de TMR0
        TMR0 = _tmr0_value;             // Valor de retraso para reinicio de TMR0
    }
    return;
}

void tmr0_setup(void){
    OPTION_REGbits.T0CS = 0;     // Temporizador
    OPTION_REGbits.PSA = 0;      // Prescaler a TMR0
    OPTION_REGbits.PS0 = 0;
    OPTION_REGbits.PS1 = 0;
    OPTION_REGbits.PS2 = 0;      // Prescaler 1:2
    INTCONbits.T0IF = 0;         // Limpiar la bandera de TMR0
    TMR0 = _tmr0_value;          // Valor de retraso (_tmr0_value = 249 para 0.015 ms)
    return;
}

void main(void) {
    setup();
    tmr0_setup();
    while(1){
        if (ADCON0bits.GO == 0){
            if(ADCON0bits.CHS == 0){        // Interrupción por AN0
                ADCON0bits.CHS = 1;         // Cambio de AN0 a AN1
                 __delay_us(40);            // Sample time
            }
            else if (ADCON0bits.CHS == 1){  // Interrupción por AN1
                ADCON0bits.CHS = 2;         // Cambio de AN1 a AN2
                 __delay_us(40);            // Sample time
            }
            else if (ADCON0bits.CHS == 2){  // Interrupción por AN1
                ADCON0bits.CHS = 0;         // Cambio de AN2 a AN0
                 __delay_us(40);            // Sample time
            }
             __delay_us(40);                // Sample time
            ADCON0bits.GO = 1;              // On
        }
    }
    return;
}



void setup(void){
    ANSEL = 0b00000111;         // AN0, AN1 y AN2 como entradas analógicas
    ANSELH = 0;                 // I/O digitales
    
    TRISA = 0b00000111;         // RA0, RA1 y RA2 como entradas analógicas y demás como salidas
    
    PORTA = 0x00;               // Se limpia PORTA
    
    
    // Configuración de interrupciones
    PIR1bits.ADIF = 0;          // Limpiar la bandera de ADC
    PIE1bits.ADIE = 1;          // Habilitar interrupciones de ADC
    INTCONbits.PEIE = 1;        // Habilitar interrupciones de periféricos
    INTCONbits.GIE = 1;         // Habilitar interrupciones globales
    INTCONbits.T0IE = 1;        // Habilitar interrupciones de TMR0
    INTCONbits.T0IF = 0;        // Limpiar la bandera de TMR0
    
    // Configuración del oscilador
    OSCCONbits.IRCF = 0b0110;   // 4 MHz
    OSCCONbits.SCS = 1;         // Oscilador interno
    
    // Configuración ADC
    ADCON0bits.ADCS = 0b10;         // FOSC/32
    ADCON1bits.VCFG0 = 0;           // VDD
    ADCON1bits.VCFG1 = 0;           // VSS
    ADCON0bits.CHS = 0b0000;        // Selecciona el AN0
    ADCON1bits.ADFM = 0;            // Justificador a la izquierda
    ADCON0bits.ADON = 1;            // Habilitar modulo ADC
    __delay_us(320);                 // Sample time
    
    // Configuración PWM
    TRISCbits.TRISC2 = 1;           // Deshabilitar salida de CCP1 (Se pone como entrada)
    TRISCbits.TRISC1 = 1;           // Deshabilitar salida de CCP2 (Se pone como entrada)
    PR2 = 249;                      // Periodo de 4 ms
    
    // Configuracion CCP
    CCP1CON = 0;                    // Apagar CCP1
    CCP2CON = 0;
    CCP1CONbits.P1M = 0;            // Modo sigle output
    CCP1CONbits.CCP1M = 0b1100;     // PWM
    CCP2CONbits.CCP2M = 0b1100;     // PWM
    
    CCPR1L = 250>>2;
    CCP1CONbits.DC1B = 250 & 0b11;  // 
    CCPR2L = 250>>2;
    CCP2CONbits.DC2B0 = 250 & 0b01; 
    CCP2CONbits.DC2B1 = 250 & 0b10; // 
    
    PIR1bits.TMR2IF = 0;            // Limpiar bandera de TMR2
    T2CONbits.T2CKPS = 0b11;        // Prescaler 1:16
    T2CONbits.TMR2ON = 1;           // Encender TMR2
    while (!PIR1bits.TMR2IF);       // Esperar un ciclo del TMR2
    PIR1bits.TMR2IF = 0;
    
    TRISCbits.TRISC2 = 0;           // Habilitar salida de PWM
    TRISCbits.TRISC1 = 0;           // Habilitar salida de PWM
    TRISCbits.TRISC3 = 0;
    return;
}

unsigned short map(uint8_t x, uint8_t x0, uint8_t x1, 
            unsigned short y0, unsigned short y1){
    return (unsigned short)(y0+((float)(y1-y0)/(x1-x0))*(x-x0));
}
