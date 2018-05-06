#ifndef UTLITIES__H__
#define UTLITIES__H__




enum mode_select{
    IDLE,
    PWM,
    ITEST,
    HOLD,
    TRACK
} MODE = 0;

enum mode_select get_mode(void);
void set_mode(int setting);


#endif // UTLITIES__H__
