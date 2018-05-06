#include "currentcontrol.h"
#include <xc.h>





void current_control_init(void)
{
    //Initialize 5kHz Timer2, 20kHz Timer3 + OC1/RD0, and Digital I/O RC14

    //Configure Timer 2 for 5kHz interrupts
    T2CONbits.TCKPS = 0;
    PR2 = 15999;
    TMR2 = 0;

    //Set interrupt Priority
    IPC2bits.T2IP = 5;
    IPC2bits.T2IS = 0;
    IFS0bits.T2IF = 0;          //Clear Flag

    //Configure Timer 3 and OC1 for 20kHz PWM
    T3CONbits.TCKPS = 0;        //Prescaler of 1
    PR3 = 3999;                 //Period of Timer 3
    TMR3 = 0;                   //Set timer to 0
    OC1CONbits.OCTSEL = 1;      //OC1 will use Timer 3
    OC1CONbits.OCM = 0b110;     //PWM mode, no fault pin
    OC1RS = 3000;               //Duty Cycle = OC1RS/(PR3+1) = 75%
    OC1R = 3000;

    //Configure RC14 for digital output
    TRISCbits.TRISC14 = 0;      //RC14 to Output
    LATCbits.LATC14 = 0;        //RC14 set to 0

    //Activate PWM Generator
    T3CONbits.ON = 1;           //Enable Timer 3
    OC1CONbits.ON = 1;          //Enable OC1

    //Activate Current Controller
    IEC0bits.T2IE = 1;          //Enable TImer 2 interrupt
    T2CONbits.ON = 1;           //Enable Timer 2
}
