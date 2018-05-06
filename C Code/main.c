#include "NU32.h"          // config bits, constants, funcs for startup and UART
#include "encoder.h"
#include "utilities.h"
#include <stdio.h>
#include "isense.h"
#include "positioncontrol.h"
// include other header files here

#define BUF_SIZE 200
#define DATA_BUF 100

int DUTY = 0;          //In percentage
int DIRECTION = 0;    //0 for forward, 1 for reverse
float Pc = .7;//.5
float Ic = .1;//.15
float Pp = 25.;
float Dp = 800.;
float Ip = .3;
int current[DATA_BUF];
int reference[DATA_BUF];
float position_array[4000];
float return_array[4000];
int reference_current = -200;   //Todo should be static in interrupt

int desired_position = 0;        //Degrees
int desired_current = 0;
int trajectory_length = 0;

int recording = 0;
int testing = 0;

char dbmsg[200];

void set_PWM(int percentage);

//TOdo
//Break out the tracking calculations into functions, then modules



void __ISR(_TIMER_1_VECTOR, IPL4SOFT) Position_Controller(void) //vector = 4
{ //Frequency validated at 5ms period with oscilloscope.

  switch(get_mode())
  {
    case HOLD:
    {
      //Establish variables
      int actual_position, position_error;
      static int position_eint;
      static int position_eprev = 0;

      //read encoder
      (encoder_counts()-32768)/(MAXCOUNT/360);           //Queue current position
      actual_position = (encoder_counts()-32768)/(MAXCOUNT/360);//Count in degrees

      //calculate e = desired - actual and eint
      position_error = desired_position - actual_position;
      position_eint += position_error;
      if(position_eint > 200)
        position_eint = 200;
      else if(position_eint < -200)
        position_eint = -200;

      //calculate reference current with control gains
      //Positive current means positive motion, negative current means negatie motion
      //Ostensibly current varies in a range like -600,600
      //If The angle is less than it should be, e>0
      //If more, e<0
      desired_current = Pp*position_error + Dp*(position_error-position_eprev) + Ip*position_eint;
      if(desired_current > 600)
        desired_current = 600;
      else if(desired_current < -600)
        desired_current = -600;


      // sprintf(dbmsg,"%d\r\n",desired_current);   //Debug print
      // NU32_WriteUART3(dbmsg);


      position_eprev = position_error;

      break;
    }
    case TRACK:
    {
      //Establish variables
      int actual_position, position_error;
      static int position_eint;
      static int position_eprev = 0;
      static int counter = 0;


      //cycle through position array
      desired_position = position_array[counter];


      //read encoder
      (encoder_counts()-32768)/(MAXCOUNT/360);           //Queue current position
      actual_position = (encoder_counts()-32768)/(MAXCOUNT/360);//Count in degrees

      //calculate e = desired - actual and eint
      position_error = desired_position - actual_position;
      position_eint += position_error;
      if(position_eint > 200)
        position_eint = 200;
      else if(position_eint < -200)
        position_eint = -200;

      //calculate reference current with control gains
      //Positive current means positive motion, negative current means negative motion
      //Ostensibly current varies in a range like -600,600
      //If The angle is less than it should be, e>0
      //If more, e<0
      desired_current = Pp*position_error + Dp*(position_error-position_eprev) + Ip*position_eint;
      if(desired_current > 600)
        desired_current = 600;
      else if(desired_current < -600)
        desired_current = -600;


      position_eprev = position_error;

      //Collect angle data
      return_array[counter] = actual_position;

      counter++;

      if(counter == trajectory_length)
      {
        counter = 0;
        recording = 0;
        set_mode(HOLD);
        //tracking variable set to low to trigger transmit
      }

      break;
    }
  }

  //Clear Flag IFS0<4>
  IFS0CLR = 1 << 4;
}


void __ISR(_TIMER_2_VECTOR, IPL5SOFT) Current_Controller(void) //vector = 8
{
  //Set duty cycle and direction based on mode
  switch(get_mode())
  {                   //Pg. 444 for H-bridge instructions
    case IDLE:
    {
        OC1RS = 0;    //0%, Braking
        DIRECTION = 0;
        LATCbits.LATC14 = DIRECTION;
        break;
    }
    case PWM:
    {
        OC1RS = DUTY * (PR3+1)/100;
        LATCbits.LATC14 = DIRECTION;
        break;
    }
    case ITEST:
    {
      int current_error, actual_current, u;
      static int current_eint = 0;
      static int counter = 0;


      //Determine reference current for this iteration
      if(counter % 25 == 0)
      {
        reference_current *= -1;
      }
      actual_current = read_current();

      //PI controller tries to reduce error between current_sense & reference_current by adjusting PWM
      current_error = reference_current - actual_current;
      current_eint = current_eint + current_error;
      if(current_eint > 500)
        current_eint = 500;
      else if(current_eint < -500)
        current_eint = -500;

      u = Pc*current_error + Ic*current_eint;
      if(u > 100.0)
      {
          u = 100.0;
      }else if(u < -100.0)
      {
          u = -100.0;
      }
      set_PWM(u);
      OC1RS = DUTY * (PR3+1)/100;
      LATCbits.LATC14 = DIRECTION;

      //Record reference and actual current values in arrays
      reference[counter] = reference_current;
      current[counter]   = actual_current;

      counter++;
      if(counter == DATA_BUF)
      {
        set_mode(IDLE);
        counter = 0;
        testing = 0;
      }
      break;
    }
    case HOLD:
    {
      int current_error, actual_current, u;
      static int current_eint;
      //perform current control with desired current as reference
      actual_current = read_current();

      //PI controller tries to reduce error between current_sense & reference_current by adjusting PWM
      current_error = desired_current - actual_current;
      current_eint = current_eint + current_error;
      if(current_eint > 500)
        current_eint = 500;
      else if(current_eint < -500)
        current_eint = -500;

      //Calculate and limit desired Duty cycle
      u = Pc*current_error + Ic*current_eint;
      if(u > 100.0)
      {
          u = 100.0;
      }else if(u < -100.0)
      {
          u = -100.0;
      }

      //Set the motor
      set_PWM(u);
      OC1RS = DUTY * (PR3+1)/100;
      LATCbits.LATC14 = DIRECTION;


      // sprintf(dbmsg,"%d\r\n",u);   //Debug print
      // NU32_WriteUART3(dbmsg);

      break;
    }
    case TRACK:
    {

      int current_error, actual_current, u;
      static int current_eint;
      //perform current control with desired current as reference
      actual_current = read_current();

      //PI controller tries to reduce error between current_sense & reference_current by adjusting PWM
      current_error = desired_current - actual_current;
      current_eint = current_eint + current_error;
      if(current_eint > 500)
        current_eint = 500;
      else if(current_eint < -500)
        current_eint = -500;

      //Calculate and limit desired Duty cycle
      u = Pc*current_error + Ic*current_eint;
      if(u > 100.0)
      {
          u = 100.0;
      }else if(u < -100.0)
      {
          u = -100.0;
      }

      //Set the motor
      set_PWM(u);
      OC1RS = DUTY * (PR3+1)/100;
      LATCbits.LATC14 = DIRECTION;
      break;
    }
  }


  //Clear Flag IFS0<8>
  IFS0CLR = 1 << 8;
}


int main()
{
  char buffer[BUF_SIZE];
  NU32_Startup(); // cache on, min flash wait, interrupts on, LED/button init, UART init
  NU32_LED1 = 1;  // turn off the LEDs
  NU32_LED2 = 1;
  set_mode(IDLE);

  __builtin_disable_interrupts();
  // Initialize modules and peripherals
  encoder_init();
  init_ADC();
  current_control_init();
  position_control_init();
  __builtin_enable_interrupts();

  while(1)
  {
    NU32_ReadUART3(buffer,BUF_SIZE); // we expect the next character to be a menu command
    NU32_LED2 = 1;                   // clear the error LED
    switch (buffer[0]) {
      case 'a':
      {
        int n;
        //Get voltage read from ADC
        n = read_ADC();
        //Get voltage read from ADC
        n += read_ADC();
        //Get voltage read from ADC
        n += read_ADC();
        //Average voltage reads
        n /= 3;
        //Return Read
        sprintf(buffer,"%d\r\n",n);
        NU32_WriteUART3(buffer);
        break;
      }
      case 'b':
      {
        int n;
        n = read_current();
        sprintf(buffer, "%d\r\n", n);
        NU32_WriteUART3(buffer);
        break;
      }
      case 'f':
      {
        int n;
        NU32_ReadUART3(buffer,BUF_SIZE);
        sscanf(buffer,"%d",&n);
        set_PWM(n);

        set_mode(PWM);
        break;
      }
      case 'g':
      {
        float v1, v2;
        NU32_ReadUART3(buffer,BUF_SIZE);
        sscanf(buffer,"%f %f",&v1, &v2);
        Pc = v1;
        Ic = v2;
        break;
      }
      case 'h':
      {
        sprintf(buffer,"%f %f\r\n",Pc,Ic);
        NU32_WriteUART3(buffer);
        break;
      }
      case 'i':
      {
        float v1, v2, v3;
        NU32_ReadUART3(buffer,BUF_SIZE);
        sscanf(buffer,"%f %f %f",&v1,&v2,&v3);
        Pp = v1;
        Dp = v2;
        Ip = v3;
        break;
      }
      case 'j':
      {
        sprintf(buffer,"%f %f %f\r\n",Pp,Dp,Ip);
        NU32_WriteUART3(buffer);
        break;
      }
      case 'k':
      {
        int i;
        set_mode(ITEST);
        testing = 1;
        while(testing)    //Wait for test to finish Todo check IDLE instead?
          {;}

        //Send number of samples
        sprintf(buffer,"%d\r\n",DATA_BUF);
        NU32_WriteUART3(buffer);
        //Send back pairs of integers
        for(i=0; i<DATA_BUF; i++)
        {
          sprintf(buffer,"%d %d\r\n",reference[i],current[i]);
          NU32_WriteUART3(buffer);
        }
        break;
      }
      case 'l':
      {
        int n;
        //Receive desired angle from user
        NU32_ReadUART3(buffer,BUF_SIZE);
        sscanf(buffer,"%d",&n);

        //Set desired position
        desired_position = n;

        //go to HOLD mode
        set_mode(HOLD);
        break;
      }
      case 'm':
      {
        int n;
        float v;
        int i;
        //receive transmission length
        NU32_ReadUART3(buffer,BUF_SIZE);
        sscanf(buffer,"%d",&n);
        //store length
        trajectory_length = n;
        //receive transmission int by int and store in position_array
        for(i=0;i<trajectory_length;i++)
        {
          NU32_ReadUART3(buffer,BUF_SIZE);
          sscanf(buffer,"%f",&v);
          position_array[i] = v;
        }
        break;
      }
      case 'n':
      {
        int n;
        float v;
        int i;
        //receive transmission length
        NU32_ReadUART3(buffer,BUF_SIZE);
        sscanf(buffer,"%d",&n);
        //store length
        trajectory_length = n;
        // sprintf(buffer,"%d\r\n",trajectory_length);
        // NU32_WriteUART3(buffer);
        //receive transmission int by int and store in position_array
        for(i=0;i<trajectory_length;i++)
        {
          NU32_ReadUART3(buffer,BUF_SIZE);
          sscanf(buffer,"%f",&v);
          position_array[i] = v;
        }
        // for(i=0;i<trajectory_length;i++)
        // {
        //   sprintf(buffer,"%f\r\n",position_array[i]);
        //   NU32_WriteUART3(buffer);
        // }
        break;
      }
      case 'o':
      {
        int i;
        recording = 1;
        set_mode(TRACK);

        while(recording)    //Wait for tracking to finish
          {;}

        //Transmit # of samples
        sprintf(buffer,"%d\r\n",trajectory_length);
        NU32_WriteUART3(buffer);

        //Transmit Data
        for(i=0;i<trajectory_length;i++)
        {
          sprintf(buffer,"%f %f\r\n",position_array[i],return_array[i]);
          NU32_WriteUART3(buffer);
        }

        break;
      }
      case 'p':
      {
        set_mode(IDLE);
        break;
      }
      case 'q':
      {
        set_mode(IDLE); //Todo Do I really need both p and q?
        break;
      }
      case 'x':
      {
        int n1=0, n2=0;
        NU32_ReadUART3(buffer,BUF_SIZE);
        sscanf(buffer, "%d %d",&n1,&n2);
        sprintf(buffer, "%d\r\n", n1+n2);
        NU32_WriteUART3(buffer);
        break;
      }
      case 'c':
      {
        sprintf(buffer,"%d\r\n",encoder_counts());
        sprintf(buffer,"%d\r\n",encoder_counts());
        NU32_WriteUART3(buffer);    //Encoder count to client
        break;
      }
      case 'e':
      {
        sprintf(buffer,"%d\r\n",encoder_reset());
        sprintf(buffer,"%d\r\n",encoder_reset());
        NU32_WriteUART3(buffer);    //Reset count to client
        break;
      }
      case 'd':
      {
        sprintf(buffer,"%d\r\n",(encoder_counts()-32768)/(MAXCOUNT/360));
        sprintf(buffer,"%d\r\n",(encoder_counts()-32768)/(MAXCOUNT/360));
        NU32_WriteUART3(buffer);    //Encoder degrees to client
        break;
      }
      case 'r':
      {
        switch (get_mode())
        {
            case IDLE:
            {
                sprintf(buffer,"IDLE\r\n");
                break;
            }
            case PWM:
            {
                sprintf(buffer,"PWM\r\n");
                break;
            }
            case ITEST:
            {
                sprintf(buffer,"ITEST\r\n");
                break;
            }
            case HOLD:
            {
                sprintf(buffer,"HOLD\r\n");
                break;
            }
            case TRACK:
            {
                sprintf(buffer,"TRACK\r\n");
                break;
            }
        }
        NU32_WriteUART3(buffer);
        break;
      }
      default:
      {
        NU32_WriteUART3("To end screen, use ctrl+a, k\r\n");
        NU32_LED2 = 0;  // turn on LED2 to indicate an error
        break;
      }
    }
  }
  return 0;
}

void set_PWM(int percentage)    //percentage is a number [-100,100]
{
        if(percentage<0)     //Place this into a module so I can do ITEST properly
        {
          DIRECTION = 1;
          percentage *= -1;
        }
        else
        {
          DIRECTION = 0;
        }
        DUTY = percentage;
}
