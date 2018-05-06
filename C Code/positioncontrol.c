#include "positioncontrol.h"
#include <xc.h>







void position_control_init(void)
{
    //Configure Timer 1 for 200 Hz interrupt
    //Period of 5,000,000 nanoseconds
    //T / (N*12.5)  - 1 = P
    //N = 8, P = 49999
    T1CONbits.TCKPS = 1;        //Prescaler of 8
    PR1 = 49999;                //Period of 50,000 ticks
    TMR1 = 0;

    //Set interrupt Priority
    IPC1bits.T1IP = 4;          //Priority 4, so that it won't interrupt current control. As long as it has time to run between current control loops?
    IPC1bits.T1IS = 0;
    IFS0bits.T1IF = 0;          //Clear Flag



    //Set up SPI4
    //Already done in encoder.c


    //Activate Current Controller
    IEC0bits.T1IE = 1;          //Enable TImer 1 interrupt
    T1CONbits.ON = 1;           //Enable Timer 1

    //Activate SPI4
    //Already done in encoder.c
}






