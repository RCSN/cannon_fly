/*
                   _ooOoo_
                  o8888888o
                  88" . "88
                  (| -_- |)
                  O\  =  /O
               ____/`---'\____
             .'  \\|     |//  `.
            /  \\|||  :  |||//  \
           /  _||||| -:- |||||-  \
           |   | \\\  -  /// |   |
           | \_|  ''\---/''  |   |
           \  .-\__  `-`  ___/-. /
         ___`. .'  /--.--\  `. . __
      ."" '<  `.___\_<|>_/___.'  >'"".
     | | :  `- \`.;`\ _ /`;.`/ - ` : | |
     \  \ `-.   \_ __\ /__ _/   .-` /  /
======`-.____`-.___\_____/___.-`____.-'======
                   `=---='
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
        佛祖保佑   永无BUG   永不宕机

   佛曰:
        写字楼里写字间，写字间里程序员；
        程序人员写程序，又拿程序换酒钱。
        酒醒只在网上坐，酒醉还来网下眠；
        酒醉酒醒日复日，网上网下年复年。
        但愿老死电脑间，不愿鞠躬老板前；
        奔驰宝马贵者趣，公交自行程序员。
				别人笑我忒疯癫，我笑自己命太贱；
        不见满街漂亮妹，哪个归得程序员？
*/

#include "fly_control_pwm.h"

Motor_Pwm_Typedef Motor_PWM_Duty;

void Motor_Control(Motor_Pwm_Typedef *motor_pwm)
{
	if(motor_pwm->Motor1>1000)
		motor_pwm->Motor1=1000;
	
	if(motor_pwm->Motor2>1000)
		motor_pwm->Motor1=1000;
	
	if(motor_pwm->Motor3>1000)
		motor_pwm->Motor1=1000;
	
	if(motor_pwm->Motor4>1000)
		motor_pwm->Motor1=1000;
	
	
	TIM2->CCR1 = motor_pwm->Motor1;// CH1
	TIM2->CCR2 = motor_pwm->Motor2;// CH2	
	TIM3->CCR3 = motor_pwm->Motor3;// CH3	
	TIM3->CCR2 = motor_pwm->Motor4;// CH4
}

