#include "config.h"
/***
修改更新记录见log.txt
参数配置详见config.h
//use 10CH_T8FB Remote controler
*/

//------------------------------------------------------------------------------
void bsp_init(void)
{
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
	delay_init(168);
	LED_GPIO_Config();
	Relay_Init();
	Exti_Gpio_Init();
	bsp_InitUart(115200);//usart1 115200bps for debug
	RC_Init();
	delay_ms(1000);
	CAN1_Configuration(0X01);
	CAN2_Configuration(0X501);
	delay_ms(100);
	TIM5_Init(2000-1,84-1);//2ms  SYS TIMER
	u1_printf("sys init finish!\r\n");
}

#define DEAD_ZONE 300
#define LSB 2275.55f//外部电机轴转过1度，内部磁编码器要走的值
#define RAD_TO_DU 57.32f
#define RATIO	50
#define _90_ANGLE	204800

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

const int32_t step = 16384;//14位编码器	204800
uint8_t chassic_ctr_cnt = 0;
static int16_t delaycnt = 0;
float womg = 0;
static float last_womg = 0;
void chassic_control_task(int16_t x,int16_t y,int16_t w)
{
	int16_t speed = 0;
	int16_t ch1 = 0,ch0 = 0,ch3 = 0;
	int32_t omgea1 = 0,omgea2 = 0,omgea3 = 0,omgea4 = 0;
	
		ch1 = y-1000;	ch0 = x-1000;
		ch3 = w-1000;

		if(myabs(ch0)>DEAD_ZONE && myabs(ch1)>DEAD_ZONE){
			last_womg = womg;
			if(ch0 > DEAD_ZONE && ch1 < -DEAD_ZONE){
				womg = -45;
				speed = -(myabs(ch0)+myabs(ch1))*0.6f;
			}
			else if(ch0 < -DEAD_ZONE && ch1 < -DEAD_ZONE){
				womg = 45;
				speed = -(myabs(ch0)+myabs(ch1))*0.6f;
			}
			else if(ch0 < -DEAD_ZONE && ch1 > DEAD_ZONE){
				womg = -45;
				speed = (myabs(ch0)+myabs(ch1))*0.6f;
			}
			else if(ch0 > DEAD_ZONE && ch1 > DEAD_ZONE){
				womg = 45;
				speed = (myabs(ch0)+myabs(ch1))*0.6f;
			}
		}
		else if(myabs(ch0)>DEAD_ZONE && myabs(ch1)<DEAD_ZONE){
			last_womg = womg;
			if(ch0>DEAD_ZONE){
				womg = 90;
				speed = (ch0-DEAD_ZONE);
			}
			else if(ch0<-DEAD_ZONE){
				womg = -90;
				speed = -(ch0+DEAD_ZONE);
			}
		}
		else if(myabs(ch0)<DEAD_ZONE && myabs(ch1)>DEAD_ZONE){
			last_womg = womg;
			womg = 0;
			if(ch1>DEAD_ZONE){
				speed = ch1-DEAD_ZONE;
			}
			else{
				speed = ch1+DEAD_ZONE;
			}
		}
		
		if(myabs(ch3)>DEAD_ZONE){
			
			if(ch3>DEAD_ZONE){
				speed = ch3-DEAD_ZONE;
			}
			else{
				speed = ch3+DEAD_ZONE;
			}
			last_womg = womg;
			womg = 45;
			if(myabs(last_womg-womg)>20.0f){
				delaycnt = 25;
			}
			delaycnt--;
			if(delaycnt<=0){
				delaycnt = 0;
				motor1_speed = speed*3;
				motor2_speed = speed*3;
				motor3_speed = speed*3;
				motor4_speed = speed*3;
				omgea1 = -womg*LSB;
				omgea2 =  womg*LSB;
				omgea3 = -womg*LSB;
				omgea4 =  womg*LSB;
			}
		}
		else if(myabs(ch0)<DEAD_ZONE && myabs(ch1)<DEAD_ZONE){
			womg = 0;
			speed = 0;
			motor1_speed = -speed*5;
			motor2_speed = -speed*5;
			motor3_speed = speed*5;
			motor4_speed = speed*5;
			omgea1 = womg*LSB;
			omgea2 = womg*LSB;
			omgea3 = womg*LSB;
			omgea4 = womg*LSB;
		}
		else{
			if(myabs(last_womg-womg)>60.0f){
				delaycnt = 32;
			}
			else if(myabs(last_womg-womg)>30.0f){
				delaycnt = 20;
			}
			delaycnt--;
			if(delaycnt<=0){
				delaycnt = 0;
				motor1_speed = -speed*6;
				motor2_speed = -speed*6;
				motor3_speed = speed*6;
				motor4_speed = speed*6;
			}
			omgea1 = womg*LSB;
			omgea2 = womg*LSB;
			omgea3 = womg*LSB;
			omgea4 = womg*LSB;
		}
		
		omgset_pos1 = Limit(omgea1,_90_ANGLE);
		omgset_pos2 = Limit(omgea2,_90_ANGLE);
		omgset_pos3 = Limit(omgea3,_90_ANGLE);
		omgset_pos4 = Limit(omgea4,_90_ANGLE);
}

void Pick_Plane_Ctr_Task(int16_t x,int16_t y,int16_t w)
{
	int16_t s_ch0 = 0,s_ch1 = 0,s_ch2 = 0;
	s_ch2 = w-1000,s_ch1 = y-1000; s_ch0 = x-1000;
		if(s_ch0>DEAD_ZONE)
		{
			CAMotor0_Sp = 200;
			CAMotor1_Sp = -200;
		}
		else if(s_ch0<-DEAD_ZONE)
		{
			CAMotor0_Sp = -200;
			CAMotor1_Sp = 200;
		}
		else 
		{
			CAMotor0_Sp = 0;
			CAMotor1_Sp = 0;
		}
		
		if(s_ch1>DEAD_ZONE)
		{
			PPMotor0_Sp = -220;
			PPMotor1_Sp = 200;
		}
		else if(s_ch1<-DEAD_ZONE)
		{
			PPMotor0_Sp = 220;
			PPMotor1_Sp = -200;
		}
		else 
		{
			PPMotor0_Sp = 0;
			PPMotor1_Sp = 0;
		}
		
		if(s_ch2>DEAD_ZONE)
		{
			UDMotor0_Sp = 370;
			UDMotor1_Sp = 400;
		}
		else if(s_ch2<-DEAD_ZONE)
		{
			UDMotor0_Sp = -435;
			UDMotor1_Sp = -380;
		}
		else 
		{
			UDMotor0_Sp = 0;
			UDMotor1_Sp = 0;
		}
}

void Auto_GoHome_Task(void)
{
	int16_t speed = 0;
	float omg = 0;
	static uint16_t runcnt = 0;
	int8_t turn_flag = 0;
	runcnt++;
	if(runcnt>60){
		runcnt = 0;
		if(T_hmcb.flag>=1920 && T_hmcf.flag>=1920){
			speed = 70;
			omg = 45;
			turn_flag = 1;
		}
		else if(T_hmcb.flag<=960&&T_hmcb.flag>0&&T_hmcf.flag<=960 && T_hmcf.flag>0){
			speed = 70;
			omg = 45;
			turn_flag = -1;
		}
		else if(T_hmcb.flag<800 && T_hmcf.flag>=1920){
			speed = 70;
			omg = 45;
			turn_flag = -1;
//				u1_printf("1\r\n");
		}
		else if(T_hmcf.flag<800 && T_hmcb.flag>=1920){
			speed = 70;
			omg = 45;
			turn_flag = 1;
//				u1_printf("2\r\n");
		}
		else if(T_hmcf.flag>=1984){
			speed = 70;
			omg = 90;
			turn_flag = 0;
		}	
		else if(T_hmcf.flag<960&&T_hmcf.flag>0){
			speed = 70;
			omg = -90;
			turn_flag = 0;
		}
		else if(T_hmcf.flag==0||T_hmcb.flag==0){
			speed = 0;
			omg = 0;
			turn_flag = 0;
		}
		else{
			speed = -150;
			omg = 0;
			turn_flag = 0;
//				u1_printf("line on mid,car run back\r\n");
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


typedef struct{
	uint8_t g_plane_flag1;
	uint8_t g_plane_flag2;
	uint8_t last_plane_flag1;
	uint8_t last_plane_flag2;
}T_PLANE;
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

uint8_t g_rgb_cmd = 0;
void RGB_Ctr_Task(void)
{
	
}

uint8_t g_agv_work_mode = STANDBY;
void AGV_Work_Mode_Choose(void)
{
	if(rc_data_flag==1&&CV.cv_cmd==0){
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

void debug(void)
{
	static uint8_t dbgcnt = 0;
	dbgcnt++;
	if(dbgcnt>50){
		u1_printf("mode:%d cvx:%.2f cvy:%.2f cvw:%.2f\r\n",g_agv_work_mode,CV.agv_spx,CV.agv_spy,CV.agv_spw);
//		u1_printf("ch0:%d ch1:%d ch3:%d sp:%d omg:%d bat:%.2f cur:%.2f soc:%.2f\r\n",\
//Channel_0,Channel_1,Channel_3,motor1_speed,omgset_pos1,Battery_Msg.Voltage,Battery_Msg.Current,Battery_Msg.Soc);
		dbgcnt = 0;
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
			UDMotor0_Sp = 370;
			UDMotor1_Sp = 400;
			if(Eject.updown0==2&&Eject.updown1==2){
				UDMotor0_Sp = 0;
				UDMotor1_Sp = 0;
				g_auto_pick_state = TWO;
			}
		break;
		case TWO:
			PPMotor0_Sp = -220;
			PPMotor1_Sp = 200;
			CAMotor0_Sp = -200;//收紧
			CAMotor1_Sp = 200;
			if(Eject.catch0==2&&Eject.catch1==2&&Eject.puspul0==1&&Eject.puspul1==1){
				PPMotor0_Sp = 0;
				PPMotor1_Sp = 0;
				CAMotor0_Sp = 0;//收紧
				CAMotor1_Sp = 0;
				g_auto_pick_state = THREE;
			}
		break;
		case THREE:
			CV.cv_cmd=2;//GOHOME mode
			g_agv_work_mode = GO_HOME;
		break;
	}
}

int main(void)
{
	bsp_init();
	Enable_All_Motor_Modbus();
	delay_ms(100);
	while(1)
	{
		if(Task_timer_flag){//20HZ
			AGV_Work_Mode_Choose();
			switch(g_agv_work_mode)
			{
				case STANDBY:
					Stop_All_Chassicmotor();
					Stop_All_Bldcmotor();
					u1_printf("standby\r\n");
				break;
				case REMOTE_CTR:
					if(right_Switch==right_Switch_UP){
						chassic_control_task(Channel_0,Channel_1,Channel_3);
//						Stop_All_Bldcmotor();
						Auto_Pick_Plane();
					}
					else if(right_Switch==right_Switch_DOWN){
						Pick_Plane_Ctr_Task(Channel_0,Channel_1,Channel_2);
						Stop_All_Chassicmotor();
					}
					else if(right_Switch==right_Switch_MID){
						Auto_GoHome_Task();
					}
				break;
				case GO_HOME:
//					u1_printf("gohome\r\n");
					Auto_GoHome_Task();
					//RTK
				break;
				case CV_CTR:
//					u1_printf("cv_ctr\r\n");
					Plane_Check_Task();
					chassic_control_task(CV.agv_spx,CV.agv_spy,CV.agv_spw);
					Pick_Plane_Ctr_Task(CV.pick_spcatch,CV.pick_sppp,CV.pick_spupdown);
				break;
			}
			Plane_Check_Task();
			RGB_Ctr_Task();
			DR16_Unlink_Check();
//			NX_Data_Return();
//			Data_Upload();
			debug();
			Task_timer_flag = 0;
		}
	}
}



