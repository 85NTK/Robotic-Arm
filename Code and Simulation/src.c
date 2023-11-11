/* 
 * File:   src.c
 * Author: MPHUC
 *
 */

#pragma config FOSC = HS        // Oscillator Selection bits (HS oscillator)
#pragma config WDTE = OFF       // Watchdog Timer Enable bit (WDT disabled)
#pragma config PWRTE = ON       // Power-up Timer Enable bit (PWRT enabled)
#pragma config BOREN = ON       // Brown-out Reset Enable bit (BOR enabled)
#pragma config LVP = OFF        // Low-Voltage (Single-Supply) In-Circuit Serial Programming Enable bit (RB3 is digital I/O, HV on MCLR must be used for programming)
#pragma config CPD = OFF        // Data EEPROM Memory Code Protection bit (Data EEPROM code protection off)
#pragma config WRT = OFF        // Flash Program Memory Write Enable bits (Write protection off; all program memory may be written to by EECON control)
#pragma config CP = OFF         // Flash Program Memory Code Protection bit (Code protection off)

#include <xc.h>
#define _XTAL_FREQ 20000000 //Crystak Freq is 20MHz
#define PWM_Frequency 0.05 // in KHz (50Hz)

int POT_val; //variable to store value from ADC
int count; //timer variable 

// T_TOTAL = (1/PWM_Frequency); //calculate Total Time from frequency (in milli sec)) //20msec
int Duty_cycle;
int T_ON[4];

char servo;

void ADC_Initialize() {
  ADCON0 = 0b01000001;

  ADCON1 = 0b11000000;
}

unsigned int ADC_Read(unsigned char channel) {
  ADCON0 &= 0x11000101;
  ADCON0 |= channel<<3;
  __delay_ms(2);

  GO_nDONE = 1;
  while(GO_nDONE);

  return ((ADRESH<<8) + ADRESL);
}


void interrupt timer_isr() {  
    // Overflow for every 0.05ms
    if(TMR0IF == 1){
        TMR0 = 248;
        TMR0IF = 0;
        
        count++; 
    } 

    int servo_code[] = {0b00000001, 0b00000010, 0b00000100, 0b00001000};

    if (count >= 400)
        count = 0;

     if (count <= (T_ON[servo]) )
         PORTC = PORTC | servo_code[servo];
     else 
         PORTC = PORTC & ~(servo_code[servo]);
}

void main() {

    /*****Port Configuration for Timer ******/
    OPTION_REG = 0b00000100;  // Timer0 with external freq and 32 as prescalar
    TMR0 = 248;       // Load the time value for 0.0001s; delayValue can be between 0-256 only
    TMR0IE = 1;       //Enable timer interrupt bit in PIE1 register
    GIE = 1;          //Enable Global Interrupt
    PEIE = 1;         //Enable the Peripheral Interrupt
    /***********______***********/   

    /*****Port Configuration for I/O ******/
    TRISC = 0x00;

    PORTC = 0x00;
    /***********______***********/ 
    
    ADC_Initialize();
    
    while(1) {
        for (int pot_num = 0; pot_num <= 3; pot_num++) {
            int prev_ton = T_ON[pot_num];
            
            POT_val = (ADC_Read(pot_num));
            Duty_cycle = (POT_val * 0.0976); //Map 0 to 1024 to 0 to 100
            T_ON[pot_num] = Duty_cycle * 0.4;
            
            if (T_ON[pot_num] != prev_ton) 
                servo = pot_num;
            
        }
    }
}
