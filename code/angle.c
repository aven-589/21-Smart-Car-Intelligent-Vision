#include "angle.h"
#include <math.h>

extern float yaw;
float an_kp=4.5,an_ki=0,an_kd=2;
float an_thiserror,an_lasterror,an_errorsum;
float an_actual,an_out;
float target_angle=0;
float angle_pid(float an_target)
 {
    an_lasterror=an_thiserror;
    an_thiserror=an_target-yaw;
    an_errorsum+=an_thiserror;
    if(an_errorsum>180)an_errorsum=180;
    if(an_errorsum<-180)an_errorsum=-180;
    an_out=an_kp*an_thiserror+an_ki*an_errorsum+an_kd*(an_thiserror-an_lasterror);
    if(an_out>300)an_out=300;
    if(an_out<-300)an_out=-300;
    return an_out;
 }
