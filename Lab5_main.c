//*****************************************************************************
//
// Lab 5 - "Low Power"
// 2/6/22
// Jannel Bennett
//
// For this lab; code was written on the MSP432 system to implement a basic remote temperature measurement system, measure the power consumption
//              to verify that the specifications are met. This system is designed to measure the temperature in an eagle nest as part of a study to
//              monitor their habits.
//
// ** PIN 2.7 Not Working
//
// Initialize GPIO. Setup inputs and outputs as follows:
//
//  Port pin    in/out  Pullup/down     Connect
//    P4.0        In      Up            Ext. Sig. (radio)
//    P4.4        In      N/A            A9 (TMP36 Sensor)
//
//  *********   Nokia LCD interface reference   **************
//
// Red SparkFun Nokia 5110 (LCD-10168)
// -----------------------------------
// Signal        (Nokia 5110) LaunchPad pin
// 3.3V          (VCC, pin 1) power
// Ground        (GND, pin 2) ground
// UCA3STE       (SCE, pin 3) connected to P9.4
// Reset         (RST, pin 4) connected to P9.3
// Data/Command  (D/C, pin 5) connected to P9.2
// UCA3SIMO      (DN,  pin 6) connected to P9.7
// UCA3CLK       (SCLK, pin 7) connected to P9.5
// back light    (LED, pin 8) not connected, consists of 4 3.3 V white LEDs which draw ~80mA total

//****************************************************************************
//****************************************************************************

#include <stdint.h>
#include "stdint.h"
#include "msp.h"
#include "msoe_lib_clk.h"
#include "msoe_lib_lcd.h"
#include "msoe_lib_delay.h"

//Function Prototypes

void init_gpio(void);
void init_A2D(void);
void init_WDT_A(void);
void init_LP(void);
void stupLCD(void);

#define INT_ADC14_BIT (1<<24);
#define lock 0;

//global variables
uint32_t converting=1; // A2D in process
int ADC=0;
float degreesC = 0;
float degreesF = 0;


/**
 * main.c
 */

void main(void){

    init_gpio();
    init_A2D();
    init_WDT_A(); // watchdog in timer mode before we can LPM3
    init_LP();
    stupLCD();


    ADC14->CTL0|=ADC14_CTL0_SC; // start ADC conversion

    // system control block -> System contr0l register
    SCB->SCR=SCB_SCR_SLEEPDEEP_Msk; // precondition device in preparation for LPM3 mode by SLEEPDEEP = 1;


    while(1){

        __wfi(); // wait for interrupt
        while (converting==1){}; // wait for A2D flag
        converting = 1;


        LCD_goto_xy(1,5);
        LCD_print_udec5(ADC);


        P4->OUT&=~(BIT4); // radio signal

    }
} // END
//********************FUNCTIONS**********************************************//
// initialize ports
void init_gpio(void)
{
    // set unused pins to pullup/down enabled to avoid floating inputs
  P1->REN |= 0xFF;
  P2->REN |= 0xFF;
  P3->REN |= 0xFF;
  P4->REN |= 0xFF;
  P5->REN |= 0xFF;
  P6->REN |= 0xFF;
  P7->REN |= 0xFF;
  P8->REN |= 0xFF;
  P9->REN |= 0xFF;
  P10->REN |= 0xFF;

//  //R&B LED's init
//  P1->DIR |= (BIT0);  // output
//  P1->OUT &=~ (0x00);  //start off

  //R&B LED's init
  P2->DIR |= (BIT0 | BIT2 | BIT1);  // output
  P2->OUT &=~ (BIT0 | BIT2 | BIT1);  //start off

    // A9 (P4.4) ADC input setup
    P4->SEL1 |= (BIT4);
    P4->SEL0 |= (BIT4);

      return;
}
// initialize LCD
void stupLCD(void){
    LCD_Config();
    LCD_clear();
}
// initialize A2D
void init_A2D(void) // check settings
{
    // Sampling time: S&H=96 | Sample&Hold pulse mode | MCLK select | ADC14 on | single channel / single conversion
    ADC14->CTL0 |= ADC14_CTL0_SHT0_5 | ADC14_CTL0_SHP | ADC14_CTL0_SSEL_3 | ADC14_CTL0_ON | ADC14_CTL0_CONSEQ_0;


    ADC14->CTL1 |= ADC14_CTL1_RES_1 ; // 10-bit conversion (11cycle conversion)
    ADC14->CTL1 &= ~ADC14_CTL1_RES_2;  // turn off 12-bit conversion (ADC14RES defaults to 11b or 14bits)
    ADC14->CTL1 |= (3 << ADC14_CTL1_CSTARTADD_OFS);  // start w/ MEM[3]`

    // input on A9 to mem3 (P4.4) | End of sequence
    ADC14->MCTL[3] |= ADC14_MCTLN_INCH_9 | ADC14_MCTLN_EOS;

    ADC14->IER0 |= ADC14_IER0_IE3;  // enable interrupt for MEM3

    ADC14->CTL0 |= ADC14_CTL0_ENC;  // enable conversion

    NVIC->ISER[0] |= INT_ADC14_BIT;  // enable ADC interrupt in NVIC

}
// ADC IR handler
void ADC14_IRQHandler(void) // adc update
  {
      // reading clears flag
      ADC = ADC14->IV; // store val to ADC (reads 18)??
      ADC14->CLRIFGR0 = ADC14_IFGR0_IFG3 ; // clear pending interrupt flag tied to mem3 (changed from IFG0 )

   // if the interupt source: ADCMEM3 interrupt flag; interrupt flag: ADC14IFG3
          ADC = ADC14->MEM[3];

          float voltage = ADC * (3.3/1023.0); // 3.3V/1023 = 0.0032258064516129 mV per step (10bit conv.)

          // he reason 0.5V is subtracted from the calculated voltage is because there is a 0.5V offset,
          // TMP36 has an output scale factor of 0.01V/degC
          // mentioned on page 8 of the TMP36 datasheet.
           degreesC = ((voltage - 0.5)/0.01);
           degreesF = degreesC * (9.0/5.0) + 32.0;

      converting = 0; // A2D done
  }
// initialize WDT_A
void init_WDT_A(void){

                // WDT_A key value for WDT_A write access| hold WDT_A | BCLK source |
                // WDT CLK Interval Select for interrupt  (00:04:16 at 32.768 kHz) | WDT_A timer selected | clear counter
    WDT_A->CTL = WDT_A_CTL_PW | WDT_A_CTL_SSEL__BCLK | WDT_A_CTL_IS_2 | WDT_A_CTL_TMSEL | WDT_A_CTL_CNTCL;

}
// WDT_A handler
void WDT_A_IRQHandler(void){
    P4->OUT |= BIT0; // radio signal on

                // enable ADC14 conversion | start ADC14 conversion
    ADC14->CTL0 |= ADC14_CTL0_ENC | ADC14_CTL0_SC;

}
// initialize low power
void init_LP (void){

    // key to access CTL0 -- Direct Write!
    // ALSO..... LPMR = 0h in the PCMCTL0 in order to select LPM3.
    // active mode request AM_LF_VCORE0 (Low-Frequency Active Mode at Core voltage setting 0.)
    // AMR has to be written at time of key or will not change ?? or maybe CS??
    PCM->CTL0 = PCM_CTL0_KEY_VAL | PCM_CTL0_AMR__AM_LF_VCORE0;

    while(PCM->CTL1 &= PCM_CTL1_PMR_BUSY){} // PMR_BUSY flag set. writes to PCMCTL0 or CS ignored while PMR_BUSY==1

    // PCM_CTL1_PMR_BUSY==0 and PCMCTL0 and CS reg's unlocked

// /*Check Point*/
//            PCM->IE |= PCM_IE_AM_INVALID_TR_IE; // AM invalid transition interrupt enable
//            if((PCM->IFG & PCM_IFG_AM_INVALID_TR_IFG) ==1 ){ // invalid transition occurred
//
//                    LCD_goto_xy(0,0);
//                    LCD_print_str("NOPE");
//                PCM->CLRIFG |= PCM_CLRIFG_CLR_AM_INVALID_TR_IFG; // clear ActiveMode invalid transition flag
//}

    CS->KEY = CS_KEY_VAL; // key to access

    CS->CTL1 = CS_CTL1_SELM__REFOCLK | CS_CTL1_SELS__REFOCLK | CS_CTL1_SELA__REFOCLK;

    uint32_t status = CS->STAT;

    CS->KEY=lock; // lock key

    while(! CS_STAT_MCLK_READY ){} /// wait for MCLK to be good

}
