#include "control.h"
#include "ZL5SERVO.h"
#include "PickPlane.h"
#include "bsp_usart.h"
#include "led.h"
#include "can1.h"
#include "can2.h"
#include "comunication.h"
#include "hmcsensor.h"

uint8_t g_release_flag = NONE;
void Auto_Release_Plane(void)
{
	switch(g_release_flag){
		case ONE:
			PP_Motor_Pull();
			if(Eject.puspul0==2&&Eject.puspul1==2){
				PP_Motor_Stop();
				g_release_flag = TWO;
			}
		break;
		case TWO:
			UD_Motor_Down();
			if(Eject.updown0==1&&Eject.updown1==1){
				UD_Motor_Stop();
				g_release_flag = THREE;
			}
		break;
		case THREE:
			Catch_Motor_Open();
			if(Eject.catch0==1&&Eject.catch1==1){
				Catch_Motor_Stop();
				g_release_flag = FOUR;
				u1_printf("release plane is ok...\r\n");
			}
		break;
		case FOUR:
			g_release_flag = DONE;
		break;
	}
}

uint8_t g_auto_pick_state = NONE;
void Auto_Pick_Plane(void)
{
	if(T_Plane.g_plane_flag1==1&&T_Plane.g_plane_flag2==1\
&&T_Plane.last_plane_flag1==0&&T_Plane.last_plane_flag2==0){
		g_auto_pick_state = ONE;
		T_Plane.last_plane_flag1=1;
		T_Plane.last_plane_flag2=1;
	}
	switch(g_auto_pick_state){
		case ONE:
			UD_Motor_Up();
			if(Eject.updown0==2&&Eject.updown1==2){
				UD_Motor_Stop();
				g_auto_pick_state = TWO;
			}
		break;
		case TWO:
			PP_Motor_Push();
			Catch_Motor_Close();
			if(Eject.catch0==2&&Eject.catch1==2&&Eject.puspul0==1&&Eject.puspul1==1){
				PP_Motor_Stop();
				Catch_Motor_Stop();
				g_auto_pick_state = THREE;
			}
		break;
		case THREE:
			g_auto_pick_state = DONE;
			CV.cv_cmd=2;//GOHOME mode
		break;
	}
}

extern volatile uint8_t rc_data_flag;
uint8_t g_agv_work_mode = STANDBY;
void AGV_Work_Mode_Choose(void)
{
	if(rc_data_flag==1){
		g_agv_work_mode = REMOTE_CTR;
	}
	else if(CV.cv_cmd==1){
		g_agv_work_mode = CV_CTR;
	}
	else if(CV.cv_cmd==2){
		g_agv_work_mode = GO_HOME;
	}
	else if((rc_data_flag==0&&CV.cv_cmd==0)||(AGV_CMD.agv_charge==1)){
		g_agv_work_mode = STANDBY;
	}
}

uint8_t g_rgb_cmd = 0;
void RGB_Ctr_Task(void)
{
	
}

T_PLANE T_Plane = {0};
void Plane_Check_Task(void)
{
	if(READ_EXTI1()==0){
		T_Plane.g_plane_flag1 = 1;
	}
	else{
		T_Plane.g_plane_flag1 = 0;
	}
	if(READ_EXTI2()==0){
		T_Plane.g_plane_flag2 = 1;
	}
	else{
		T_Plane.g_plane_flag2 = 0;
	}
}

static uint8_t auto_stop_flag = 0,auto_stop_cnt = 0;
void Auto_GoHome_Task(void)
{
	int16_t speed = 0;
	float omg = 0;
	static uint16_t runcnt = 0;
	int8_t turn_flag = 0;
	
	runcnt++;
	if(runcnt>60){
		runcnt = 0;
		
		if(g_release_flag!=NONE && g_release_flag!=DONE){//还没有放完飞机
			speed = 0;omg = 0;turn_flag = 0;
		}
		else if(g_auto_pick_state!=NONE && g_auto_pick_state!=DONE){//还没有抬起飞机
			speed = 0;omg = 0;turn_flag = 0;
		}
		else if(T_hmcb.flag>60000){//遇到横线
			if(auto_stop_flag==0){
				auto_stop_flag = 1;
				g_release_flag = ONE;
				{speed = 0;omg = 0;turn_flag = 0;}
				u1_printf("first line stop,release plane...\r\n");
			}
			else if(auto_stop_flag==1){
				auto_stop_cnt++;
				if(auto_stop_cnt>2){
					auto_stop_cnt = 2;
					auto_stop_flag = 2;
					{speed = 0;omg = 0;turn_flag = 0;}
					g_agv_work_mode = STANDBY;
					u1_printf("second line stop,agv change...\r\n");
				}
				else{
					speed = -150;
					omg = 0;
					turn_flag = 0;
					u1_printf("go back,find second line...\r\n");
				}
			}
		}
		else if(T_hmcb.flag>2032 && T_hmcf.flag<=2016 && T_hmcf.flag>0){
			speed = 70;
			omg = 45;
			turn_flag = 1;//turn riht
u1_printf("turn right hmcf:%d hmcb:%d\r\n",T_hmcf.flag,T_hmcb.flag);
		}
		else if(T_hmcb.flag<1984&&T_hmcb.flag>0&&T_hmcf.flag>=1984){
			speed = 70;
			omg = 45;
			turn_flag = -1;//turn left
u1_printf("turn left hmcf:%d hmcb:%d\r\n",T_hmcf.flag,T_hmcb.flag);
		}
		else if(T_hmcf.flag==0||T_hmcb.flag==0){
			{speed = 0;omg = 0;turn_flag = 0;}
		}
		else{
			speed = -150;
			omg = 0;
			turn_flag = 0;
u1_printf("go back hmcf:%d hmcb:%d\r\n",T_hmcf.flag,T_hmcb.flag);
		}
		if(turn_flag == 1){//turn riht
			motor1_speed = speed;
			motor2_speed = speed;
			motor3_speed = speed;
			motor4_speed = speed;
			omgset_pos1 = -Limit(omg*LSB,_90_ANGLE);
			omgset_pos2 = Limit(omg*LSB,_90_ANGLE);
			omgset_pos3 = -Limit(omg*LSB,_90_ANGLE);
			omgset_pos4 = Limit(omg*LSB,_90_ANGLE);
		}
		else if(turn_flag == -1){//turn left
			motor1_speed = -speed;
			motor2_speed = -speed;
			motor3_speed = -speed;
			motor4_speed = -speed;
			omgset_pos1 = -Limit(omg*LSB,_90_ANGLE);
			omgset_pos2 = Limit(omg*LSB,_90_ANGLE);
			omgset_pos3 = -Limit(omg*LSB,_90_ANGLE);
			omgset_pos4 = Limit(omg*LSB,_90_ANGLE);
		}
		else{
			motor1_speed = -speed;
			motor2_speed = -speed;
			motor3_speed = speed;
			motor4_speed = speed;
			omgset_pos1 = Limit(omg*LSB,_90_ANGLE);
			omgset_pos2 = Limit(omg*LSB,_90_ANGLE);
			omgset_pos3 = Limit(omg*LSB,_90_ANGLE);
			omgset_pos4 = Limit(omg*LSB,_90_ANGLE);
		}
	}
}

int Limit(int datain,int max)
{
	if(datain>max){
		datain = max;
	}
	else if(datain<-max){
		datain = -max;
	}
	return datain;
}


