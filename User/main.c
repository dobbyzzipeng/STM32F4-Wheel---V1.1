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
	if(dbgcnt>30){
		if(rc_data_flag==0){
			u5_printf("mode:%d  auto_state:%d\r\n",g_agv_work_mode,g_agv_task_state);
//pos_analysis(agvrtk.lon,agvrtk.lat,agvrtk.ang,drgrtk.lon,drgrtk.lat,drgrtk.ang,&Pos);
//u5_printf("agvx:%.2f agvy:%.2f agvang:%.2f drgx:%.2f drgy:%.2f drgang:%.2f Qx:%.2f Qy:%.2f ver:%.2f agv2drg:%.2f agv2Q:%.2f workmode:%d\r\n", \
//Pos.agv_x,Pos.agv_y,Pos.agv_ang,Pos.drg_x,Pos.drg_y,Pos.drg_ang,Pos.Q_x,Pos.Q_y,Pos.vert_to_drg2Q,Pos.agv2drg,Pos.agv2Q,g_agv_work_mode);
//u5_printf("agvx:%.2f agvy:%.2f agvang:%.2f drgx:%.2f drgy:%.2f drgang:%.2f Qx:%.2f Qy:%.2f ver:%.2f agv2drg:%.2f agv2Q:%.2f agv_lon:%lf,agv_lat:%lf,agv_ang:%.2f mode:%d drg_lon:%lf,drg_lat:%lf,drg_ang:%.2f\r\n", \
//Pos.agv_x,Pos.agv_y,Pos.agv_ang,Pos.drg_x,Pos.drg_y,Pos.drg_ang,Pos.Q_x,Pos.Q_y,Pos.vert_to_drg2Q,Pos.agv2drg,Pos.agv2Q,agvrtk.lon*100,agvrtk.lat*100,agvrtk.ang,agvrtk.datamode,drgrtk.lon*100,drgrtk.lat*100,drgrtk.ang);
		dbgcnt = 0;
		}
	}
}

int main(void)
{
	uint8_t rel_flag = 0;
	static uint8_t decnt = 0;
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
						Stop_All_Chassicmotor();
						Stop_All_Bldcmotor();
					}
					if(right_Wheel==RW_DOWN){
						if(left_Wheel>100){//手动调试
							g_agv_task_state = GOHOME_WITHOUT_PLANE;
						}
						else{
							g_agv_task_state = PICK_PLANE_OUT;
						}
					}
					else if(right_Wheel==RW_UP){
						if(left_Wheel>100){//手动调试
							g_agv_task_state = GET_OUT_FIND_PLANE;
						}
						else{
							g_agv_task_state = CVRTK_FIND_PLANE;
						}
					}
				break;
				case AUTO_CTR:
					switch(g_agv_task_state){
						case TASK_OK_CHARGE:
							decnt++;
							if(decnt>20){
								decnt = 0;
								Stop_All_Chassicmotor();Stop_All_Bldcmotor();
								Follow_Line_Clear();Auto_CVRTK_FindState_Clear();
								Auto_Find_BlackLine_State_Clear();
					u5_printf("task ok,plane in home,agv is charging now...\r\n");
							}
						break;
						// 1. task one
						case PICK_PLANE_OUT:
						case GOHOME_WITHOUT_PLANE:
							if(rel_flag!=1){
								if(is_release()!=1){
									Auto_Release_Plane(INIT);
								}
								else{
									rel_flag = 1;
								}
							}
							else{
								Auto_FollowLine_Task(OUT,WITHPLANE);
								Auto_Pick_Plane(FIND,OUT);
								Auto_Release_Plane(OUT);//g_release_flag = DONE;
							}
						break;
						case GOBACK_FOR_SPACE:
							Auto_FollowLine_Task(REBACK,NOPLANE);
							rel_flag = 0;
//							Auto_Pick_Plane(CLOSE,IN);
//							if(g_pick_state == DONE){
//								g_agv_task_state = GOHOME_WITHOUT_PLANE;
//							}
						break;
						
						// 2. task two
//						case GOHOME_WITHOUT_PLANE:
//							if(g_pick_state != DONE){
//								Auto_Pick_Plane(CLOSE,IN);
//							}
//							else{
//								Auto_FollowLine_Task(IN,NOPLANE);//black line or until charge flag
//							}
//						break;
						case CHARGE_WITHOUT_PLANE:
							decnt++;
							if(decnt>20){
								decnt = 0;
					u5_printf("task ok,plane out home,agv is charging now...\r\n");
							}
						break;
							
						// 3. task three
						case GET_OUT_FIND_PLANE:
							Auto_FollowLine_Task(OUT,NOPLANE);
//							if(g_release_flag!=NONE&&g_release_flag!=DONE){
//								Auto_Release_Plane(INIT);//g_release_flag==DONE
//							}
//							else if(g_release_flag==DONE){
//								g_agv_task_state = CVRTK_FIND_PLANE;//切换到视觉RTK搜索龙鱼
//								g_dragonfish_flag = 1;//要搜索龙鱼
//							}
						break;
						case GOFRONT_FOR_SPACE:
							Auto_FollowLine_Task(GOFRONT,NOPLANE);
						    agv_open_arm();
						    if (g_agv_arm_flag == AGV_ARM_OPEN)
							{
								u5_printf("agv arm open at start point!\r\n");
							}
						break;
						
						// 4. task four
						case CVRTK_FIND_PLANE://视觉 RTK搜索龙鱼
							if(rel_flag!=1){
								if(is_release()!=1){
									Auto_Release_Plane(INIT);
								}
								else{
									rel_flag =1;
								}
							}
							 else{
								Auto_CVRTK_FindPlane();
								Auto_Pick_Plane(FIND,IN);//g_pick_state=DONE
								if(g_pick_state==DONE){
									rel_flag = 0;
									g_agv_task_state = FIIND_BLACK_LINE;
								}
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
			debug();
			Plane_Check_Task();
			DR16_Unlink_Check();
			NX_Data_Return();
			AGV_Data_Upload();
			Task_timer_flag = 0;
		}
	}
}



