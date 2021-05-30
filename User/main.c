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
	bsp_InitUart(115200);// for server
	delay_ms(100);
	RC_Init();
	delay_ms(100);
	usart3_init(115200);
	delay_ms(100);
	usart4_init(115200);
	delay_ms(100);
	usart5_init(115200);//debug
	server_485_init();
	delay_ms(1000);
	CAN1_Configuration(0X01);
	delay_ms(200);
	CAN2_Configuration(0X501);
	delay_ms(100);
	TIM5_Init(2000-1,84-1);//2ms  SYS TIMER
	u5_printf("sys init finish!\r\n");
}



void debug(void)
{
	static uint8_t dbgcnt = 0;
	dbgcnt++;
	if(dbgcnt>20){
		if(right_Switch==right_Switch_UP&&rc_data_flag!=0){
			u5_printf("mode:%d hmcf:%d hmcb:%d\r\n",g_agv_work_mode,T_hmcf.flag,T_hmcb.flag);
//		u5_printf("mode:%d cvx:%.2f cvy:%.2f cvw:%.2f\r\n",g_agv_work_mode,CV.agv_spx,CV.agv_spy,CV.agv_spw);
//		u5_printf("ch0:%d ch1:%d ch3:%d sp:%d omg:%d bat:%.2f cur:%.2f soc:%.2f\r\n",\
//Channel_0,Channel_1,Channel_3,motor1_speed,omgset_pos1,Battery_Msg.Voltage,Battery_Msg.Current,Battery_Msg.Soc);
		dbgcnt = 0;
		}
	}
}

int main(void)
{
	static uint8_t decnt = 0;
	bsp_init();
	Enable_All_Motor_Modbus();
	Auto_Release_Plane(INIT);
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
					delay_ms(100);
					u5_printf("standby...\r\n");
				break;
				case REMOTE_CTR:
					if(right_Switch==right_Switch_UP){
						chassic_control_task(Channel_0,Channel_1,Channel_3);
						Stop_All_Bldcmotor();
					}
					else if(right_Switch==right_Switch_DOWN){
						Pick_Plane_Ctr_Task(Channel_0,Channel_1,Channel_2);
						Stop_All_Chassicmotor();
					}
					else if(right_Switch==right_Switch_MID){
						Auto_FollowLine_Task(OUT,WITHPLANE);
					}
					if(left_Wheel>100){//手动调试
						if(g_agv_task_state==CHARGE_WITHOUT_PLANE){
//							g_agv_task_state = GET_OUT_FIND_PLANE;//手动调试切换状态
							g_agv_task_state = CVRTK_FIND_PLANE;
						}
						g_agv_task_state = CVRTK_FIND_PLANE;
						g_cvrtk_findplane_state = ONE;
					}
					else{
							decnt++;
							if(decnt>20){
					u5_printf("agv_lon:%lf,agv_lat:%lf,agv_ang:%lf mode:%d\r\n",agvrtk.lon*100,agvrtk.lat*100,agvrtk.ang,agvrtk.datamode);
								decnt=0;
							}
						}
				break;
				case AUTO_CTR:
					if(AGV_CMD.agv_cmd!=0 && AGV_CMD.last_agv_cmd==0&&g_agv_task_state==TASK_OK_CHARGE){
						g_agv_task_state = PICK_PLANE_OUT;
						AGV_CMD.last_agv_cmd = AGV_CMD.agv_cmd;
					}
					switch(g_agv_task_state){
						case TASK_OK_CHARGE:
							decnt++;
							if(decnt>20){
								decnt = 0;
								Follow_Line_Clear();
								Auto_CVRTK_FindState_Clear();
							u5_printf("task ok,plane in home,agv is charging now...\r\n");
							}
						break;
						case PICK_PLANE_OUT:
							Auto_FollowLine_Task(OUT,WITHPLANE);
							Auto_Pick_Plane(FIND,OUT);
							Auto_Release_Plane(OUT);//g_release_flag = DONE;
						break;
						case GOHOME_WITHOUT_PLANE:
							Auto_FollowLine_Task(IN,NOPLANE);//black line or until charge flag
							if(g_pick_state!=NONE&&g_pick_state!=DONE){
								Auto_Pick_Plane(CLOSE,IN);//g_pick_state = NONE;//清空状态机标志
							}
						break;
						case CHARGE_WITHOUT_PLANE:
							decnt++;
							if(decnt>20){
								decnt = 0;
								u5_printf("task ok,plane out home,agv is charging now...\r\n");
//								g_agv_task_state = GET_OUT_FIND_PLANE;
							}
						break;
						case GET_OUT_FIND_PLANE:
							Auto_FollowLine_Task(OUT,NOPLANE);
							if(g_release_flag!=NONE&&g_release_flag!=DONE){
								Auto_Release_Plane(INIT);//g_release_flag==DONE
							}
							#if 0
							if(g_release_flag==DONE){
								Auto_Pick_Plane(FIND,IN);//g_pick_state = DONE;
							}
							if(g_pick_state==DONE){
								g_agv_task_state = PICK_PLANE_IN;
							}
							#else
							if(g_release_flag==DONE){
								g_agv_task_state = CVRTK_FIND_PLANE;
							}
							#endif
						break;
						case CVRTK_FIND_PLANE://视觉 RTK搜索龙鱼
							Auto_CVRTK_FindPlane();
							Auto_Pick_Plane(FIND,IN);//g_pick_state=DONE
							if(g_pick_state==DONE){
								//find black line and go back
								g_find_blackline_state = ONE;
								g_agv_task_state = FIIND_BLACK_LINE;
							}
						break;
						case FIIND_BLACK_LINE:
							Auto_Find_BlackLine();
							if(g_find_blackline_state==DONE){
								g_agv_task_state = PICK_PLANE_IN;
							}
						break;
						case PICK_PLANE_IN:
							Auto_FollowLine_Task(IN,WITHPLANE);
							Auto_Release_Plane(IN);//g_release_flag = DONE;
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
			DR16_Unlink_Check();
			NX_Data_Return();
			AGV_Data_Upload();
//			debug();
			Task_timer_flag = 0;
		}
	}
}



