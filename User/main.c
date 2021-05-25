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
	delay_ms(200);
	CAN2_Configuration(0X501);
	delay_ms(100);
	TIM5_Init(2000-1,84-1);//2ms  SYS TIMER
	u1_printf("sys init finish!\r\n");
}

const int32_t step = 16384;//14位编码器	204800
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
		Catch_Motor_Open();
	}
	else if(s_ch0<-DEAD_ZONE)
	{
		Catch_Motor_Close();
	}
	else 
	{
		Catch_Motor_Stop();
	}
	
	if(s_ch1>DEAD_ZONE)
	{
		PP_Motor_Push();
	}
	else if(s_ch1<-DEAD_ZONE)
	{
		PP_Motor_Pull();
	}
	else 
	{
		PP_Motor_Stop();
	}
	
	if(s_ch2>DEAD_ZONE)
	{
		UD_Motor_Up();
	}
	else if(s_ch2<-DEAD_ZONE)
	{
		UD_Motor_Down();
	}
	else 
	{
		UD_Motor_Stop();
	}
}


void debug(void)
{
	static uint8_t dbgcnt = 0;
	dbgcnt++;
	if(dbgcnt>20){
		if(right_Switch==right_Switch_UP&&rc_data_flag!=0){
			u1_printf("mode:%d hmcf:%d hmcb:%d\r\n",g_agv_work_mode,T_hmcf.flag,T_hmcb.flag);
//		u1_printf("mode:%d cvx:%.2f cvy:%.2f cvw:%.2f\r\n",g_agv_work_mode,CV.agv_spx,CV.agv_spy,CV.agv_spw);
//		u1_printf("ch0:%d ch1:%d ch3:%d sp:%d omg:%d bat:%.2f cur:%.2f soc:%.2f\r\n",\
//Channel_0,Channel_1,Channel_3,motor1_speed,omgset_pos1,Battery_Msg.Voltage,Battery_Msg.Current,Battery_Msg.Soc);
		dbgcnt = 0;
		}
	}
}

int main(void)
{
	double dis = 0;
	bsp_init();
	Enable_All_Motor_Modbus();
//	Auto_Release_Plane(INIT);
	delay_ms(100);
	while(1)
	{
		if(Task_timer_flag){//20HZ
			AGV_Work_Mode_Choose();
			switch(g_agv_work_mode)
			{
				case STANDBY:
					dis = gps_get_distance(118.47810568,30.516160653,118.478099582,30.516160595);
					Stop_All_Chassicmotor();
					Stop_All_Bldcmotor();
					delay_ms(100);
					u1_printf("standby,dis is:%f\r\n",dis);
				break;
				case REMOTE_CTR:
					if(right_Switch==right_Switch_UP){
						chassic_control_task(Channel_0,Channel_1,Channel_3);
//						Auto_Pick_Plane(FIND);//手动模式下调试自动
					}
					else if(right_Switch==right_Switch_DOWN){
						Pick_Plane_Ctr_Task(Channel_0,Channel_1,Channel_2);
						Stop_All_Chassicmotor();
					}
					else if(right_Switch==right_Switch_MID){
						Auto_FollowLine_Task(OUT,WITHPLANE);
					}
					if(left_Wheel>100){
					  //3051.6160569,N,11847.8117588
//						dis = gps_get_distance(118.7968904,30.8602454,118.7968626466,30.860267615);
						dis = rtk_dis_analysis(118.47813424,30.51614724,118.478117588,30.516160569);//0.579
						double ang = gps_get_angle(118.7968904,30.8602454,118.7968626466,30.860267615);
						delay_ms(100);
						u1_printf("standby,dis is:%lfm,ang:%lf\r\n",dis,ang);
					}

				break;
				case AUTO_CTR:
					if(AGV_CMD.agv_cmd!=0 && AGV_CMD.last_agv_cmd==0&&g_agv_task_state==TASK_OK_CHARGE){
						g_agv_task_state = PICK_PLANE_OUT;
						AGV_CMD.last_agv_cmd = AGV_CMD.agv_cmd;
					}
					switch(g_agv_task_state){
						case TASK_OK_CHARGE:
							delay_ms(100);
							Follow_Line_Clear();
							u1_printf("task ok,plane in home,agv is charging now...\r\n");
						break;
						case PICK_PLANE_OUT:
							Auto_FollowLine_Task(OUT,WITHPLANE);
							Auto_Pick_Plane(FIND,OUT);
							Auto_Release_Plane(OUT);
						break;
						case GOHOME_WITHOUT_PLANE:
							Auto_FollowLine_Task(IN,NOPLANE);//black line or until charge flag
							if(g_auto_pick_state!=NONE&&g_auto_pick_state!=DONE){
								Auto_Pick_Plane(CLOSE,IN);
							}
						break;
						case CHARGE_WITHOUT_PLANE:
							delay_ms(100);
							u1_printf("task ok,plane out home,agv is charging now...\r\n");
//							g_agv_task_state = GET_OUT_FIND_PLANE;
						break;
						case GET_OUT_FIND_PLANE:
							Auto_FollowLine_Task(OUT,NOPLANE);
							if(g_release_flag!=NONE && g_release_flag!=DONE){
								Auto_Release_Plane(INIT);
							}
							Auto_Pick_Plane(FIND,IN);
							if(g_auto_pick_state==DONE){
								g_agv_task_state = PICK_PLANE_IN;
							}
						break;
//						case CVRTK_FIND_PLANE:
//							Auto_Pick_Plane(FIND);
//							g_agv_task_state = PICK_PLANE_IN;	
//						break;
						case PICK_PLANE_IN:
							Auto_FollowLine_Task(IN,WITHPLANE);
							Auto_Release_Plane(IN);
						break;
						case RELEASE_PLANE_IN:
							g_agv_task_state = TASK_OK_CHARGE;	
						break;
					}
				break;
				case CV_CTR:
					chassic_control_task(CV.agv_spx,CV.agv_spy,CV.agv_spw);
					Pick_Plane_Ctr_Task(CV.pick_spcatch,CV.pick_sppp,CV.pick_spupdown);
				break;
			}
			Plane_Check_Task();
			RGB_Ctr_Task();
			DR16_Unlink_Check();
//			NX_Data_Return();
//			Data_Upload();
//			debug();
			Task_timer_flag = 0;
		}
	}
}



