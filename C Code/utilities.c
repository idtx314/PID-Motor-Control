#include "utilities.h"


// enum mode_select{
//     IDLE,
//     PWM,
//     ITEST,
//     HOLD,
//     TRACK
// } MODE = 0;



enum mode_select get_mode(void)
{
    return MODE;
}

void set_mode(int setting)
{
    MODE = setting;
}




