#include "move.h"

#define PWM_MAX 8000
void motor_init(void)
{
	
                                                        // ×ó²àÕý×ª
        
   pwm_init(MOTOR1_PWM1, 17000, 0);											        // PWM³õÊ¼»¯ÆµÂÊ 17KHz Õ¼¿Õ±È³õÊ¼Îª 0
	pwm_init(MOTOR1_PWM2, 17000, 0);											        // PWM³õÊ¼»¯ÆµÂÊ 17KHz Õ¼¿Õ±È³õÊ¼Îª 0
    
    pwm_init(MOTOR2_PWM1, 17000, 0);											        // PWM³õÊ¼»¯ÆµÂÊ 17KHz Õ¼¿Õ±È³õÊ¼Îª 0
	pwm_init(MOTOR2_PWM2, 17000, 0);											        // PWM³õÊ¼»¯ÆµÂÊ 17KHz Õ¼¿Õ±È³õÊ¼Îª 0
    
    pwm_init(MOTOR3_PWM1, 17000, 0);											        // PWM³õÊ¼»¯ÆµÂÊ 17KHz Õ¼¿Õ±È³õÊ¼Îª 0
	pwm_init(MOTOR3_PWM2, 17000, 0);											        // PWM³õÊ¼»¯ÆµÂÊ 17KHz Õ¼¿Õ±È³õÊ¼Îª 0
    
    pwm_init(MOTOR4_PWM1, 17000, 0);											        // PWM³õÊ¼»¯ÆµÂÊ 17KHz Õ¼¿Õ±È³õÊ¼Îª 0
	pwm_init(MOTOR4_PWM2, 17000, 0);			

		
}




void motor_c(int pwm)
{
	//Õý×ª
	if(pwm>PWM_MAX) pwm=PWM_MAX;
    if(pwm<-PWM_MAX) pwm=-PWM_MAX;
	if(0<=pwm)
	{
		pwm_set_duty(MOTOR1_PWM1,pwm);
        pwm_set_duty(MOTOR1_PWM2, 0);
	}
	//·´×ª
	else
	{
		pwm_set_duty(MOTOR1_PWM2, -pwm);
		pwm_set_duty(MOTOR1_PWM1, 0);
	}
}

void motor_d(int pwm)
{
	if(pwm>PWM_MAX) pwm=PWM_MAX;
    if(pwm<-PWM_MAX) pwm=-PWM_MAX;
	if(0<=pwm)
	{
		pwm_set_duty(MOTOR2_PWM1,pwm);
    pwm_set_duty(MOTOR2_PWM2, 0);
	}
	//·´×ª
	else
	{
		    pwm_set_duty(MOTOR2_PWM2, -pwm);
		    pwm_set_duty(MOTOR2_PWM1, 0);
	}
}

void motor_b(int pwm)
{
	if(pwm>PWM_MAX) pwm=PWM_MAX;
    if(pwm<-PWM_MAX) pwm=-PWM_MAX;
	if(0<=pwm)
	{

		pwm_set_duty(MOTOR3_PWM1, pwm);
    pwm_set_duty(MOTOR3_PWM2, 0);
	}
	//·´×ª
	else
	{
		  pwm_set_duty(MOTOR3_PWM1, 0);
      pwm_set_duty(MOTOR3_PWM2, -pwm);

	}
}

void motor_a(int pwm)
{
	if(pwm>PWM_MAX) pwm=PWM_MAX;
    if(pwm<-PWM_MAX) pwm=-PWM_MAX;
	if(0<=pwm)
	{

		pwm_set_duty(MOTOR4_PWM1, 0);
    pwm_set_duty(MOTOR4_PWM2, pwm);
           
	}
	//·´×ª
	else
	{
		pwm_set_duty(MOTOR4_PWM1, -pwm);
    pwm_set_duty(MOTOR4_PWM2, 0);

	}
}


