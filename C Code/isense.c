#include "isense.h"
#include <xc.h>




void init_ADC(void)
{
    //Congigure AN10/RB10
    AD1PCFGbits.PCFG10 = 0;         //Configure Pin RB10 as an analog input.
    AD1CON3bits.ADCS = 2;           //ADC clock Tad = 2*(this + 1)*12.5 = 75ns
    AD1CON1bits.SSRC = 0b111;       //Activate Auto-Conversion
    AD1CHSbits.CH0SA = 10;          //Set sample voltage source to pin RB10.
    AD1CON1bits.ON = 1;             //Activate ADC

    //It's possible to generate an interrupt after a certain number of readings, which will be stored in buffers. Could use this to make the program code simpler.
}

int read_ADC(void)
{
    //Simply read the ADC and return the reading
    AD1CON1bits.SAMP = 1;           //Begin the read
    while(!AD1CON1bits.DONE)        //Delay until Conversion is done
        {;}
    return ADC1BUF0;                //Return result, should be 0-1023
}

int read_current(void)
{
    int sum = read_ADC();
    sum += read_ADC();
    sum += read_ADC();
    sum /= 3;

    return (int)(1.9626031036 * (float)sum - 998.8327484971);

}



