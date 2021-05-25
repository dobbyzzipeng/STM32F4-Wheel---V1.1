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
void Auto_Release_Plane(uint8_t agvdir)
{
	if(agvdir!=INIT)
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
				if(agvdir==OUT){
					g_agv_task_state = GOHOME_WITHOUT_PLANE;//first release,next goback for space
				}
//				else if(agvdir==IN){
//					g_agv_task_state = RELEASE_PLANE_IN;
//				}
			break;
		}
	}
	else{
		do{
			PP_Motor_Pull();
			UD_Motor_Down();
			Catch_Motor_Open();
		}
		while((Eject.puspul0!=2||Eject.puspul1!=2)||(Eject.updown0!=1||Eject.updown1!=1)||(Eject.catch0!=1||Eject.catch1!=1));
		PP_Motor_Stop();
		UD_Motor_Stop();
		Catch_Motor_Stop();
		u1_printf("pickplane is open ok...\r\n");
	}
}

uint8_t g_auto_pick_state = NONE;
void Auto_Pick_Plane(uint8_t flag,uint8_t dir)
{
	if(flag==FIND){
		if(T_Plane.g_plane_flag1==1&&T_Plane.g_plane_flag2==1\
	&&T_Plane.last_plane_flag1==0&&T_Plane.last_plane_flag2==0){
			g_auto_pick_state = ONE;
			T_Plane.last_plane_flag1=1;
			T_Plane.last_plane_flag2=1;
			u1_printf("find plane is ok...\r\n");
		}
		if(dir==IN){
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
						u1_printf("pick plane is ok...\r\n");
					}
				break;
				case THREE:
					g_auto_pick_state = DONE;
				break;
			}
		}
		else if(dir==OUT){
			switch(g_auto_pick_state){
				case ONE:
					Catch_Motor_Close();
					if(Eject.catch0==2&&Eject.catch1==2){
						Catch_Motor_Stop();
						g_auto_pick_state = TWO;
					}
				break;
				case TWO:
					PP_Motor_Push();
					UD_Motor_Up();
					if(Eject.updown0==2&&Eject.updown1==2&&Eject.puspul0==1&&Eject.puspul1==1){
						PP_Motor_Stop();
						UD_Motor_Stop();
						g_auto_pick_state = THREE;
						u1_printf("pick plane is ok...\r\n");
					}
				break;
				case THREE:
					g_auto_pick_state = DONE;
				break;
			}
		}
	}
	else if(flag==CLOSE){
		do{
			UD_Motor_Up();
			Catch_Motor_Close();
		}while((Eject.updown0!=2||Eject.updown1!=2)||(Eject.catch0!=2||Eject.catch1!=2));
		UD_Motor_Stop();
		PP_Motor_Stop();
		Catch_Motor_Stop();
		g_auto_pick_state = NONE;//清空状态机标志
		u1_printf("pick machine is close...\r\n");
	}
}

extern volatile uint8_t rc_data_flag;
uint8_t g_agv_work_mode = STANDBY;
uint8_t g_agv_task_state = TASK_OK_CHARGE;
void AGV_Work_Mode_Choose(void)
{
	if(rc_data_flag==1){
		g_agv_work_mode = REMOTE_CTR;
	}
	else if(AGV_CMD.agv_cmd==RUN){
		g_agv_work_mode = AUTO_CTR;
	}
	else if(AGV_CMD.agv_cmd==PAUSE){
		g_agv_work_mode = STANDBY;
	}
	else if(AGV_CMD.agv_cmd==CANCEL){
		g_agv_work_mode = STANDBY;
		g_agv_task_state = TASK_OK_CHARGE;
	}
	else if((rc_data_flag==0&&AGV_CMD.agv_cmd==NOTASK)/*||(AGV_CMD.agv_charge==1)*/){
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

static uint8_t in_black_line_flag = 0,in_stop_cnt = 0;//in
static uint8_t out_black_line_flag = 0,out_stop_cnt = 0;//out
static uint8_t in_black_line_flag1 = 0,in_stop_cnt1 = 0;//in
static uint8_t out_black_line_flag1 = 0,out_stop_cnt1 = 0;//out
void Follow_Line_Clear(void)
{
	in_black_line_flag = 0;in_stop_cnt = 0;
	out_black_line_flag = 0;out_stop_cnt = 0;
	in_black_line_flag1 = 0;in_stop_cnt1 = 0;
	out_black_line_flag1 = 0;out_stop_cnt1 = 0;
}
	
void Auto_FollowLine_Task(uint8_t dir,uint8_t plane)
{
	int16_t speed = 0;
	float omg = 0;
	static uint16_t runcnt = 0;
	int8_t turn_flag = 0;
	runcnt++;
	if(runcnt>60){//不可修改
		runcnt = 0;
		Line_Analysis(T_hmcf.flag,&Linef);
		Line_Analysis(T_hmcb.flag,&Lineb);
		
		if(g_release_flag!=NONE && g_release_flag!=DONE){//还没有放完飞机
			speed = 0;omg = 0;turn_flag = 0;
		}
		else if(g_auto_pick_state!=NONE && g_auto_pick_state!=DONE){//还没有抬起飞机
			speed = 0;omg = 0;turn_flag = 0;
		}
//		else if((T_hmcb.flag&0xE187)==0XE187)
		else if(Lineb.wlineflag==1)
		{//遇到横线
			if(plane==WITHPLANE)
			{
				if(dir==IN)
				{
					if(in_black_line_flag==0){
						in_stop_cnt++;
						if(in_stop_cnt>1){
							in_stop_cnt = 1;
							in_black_line_flag = 1;
						}
						{speed = -150;omg = 0;turn_flag = 0;}
						u1_printf("first line,not stop still go back...\r\n");
					}
					else if(in_black_line_flag==1){
						in_stop_cnt++;
						if(in_stop_cnt>=3){
							in_stop_cnt = 3;
							in_black_line_flag = 2;
							{speed = 0;omg = 0;turn_flag = 0;}
							g_release_flag = ONE;//agv stop and release plane
							u1_printf("find second line,agv stop and release plane\r\n");
						}
						else{
							{speed = -150;omg = 0;turn_flag = 0;}
							u1_printf("go back,find second line...\r\n");
						}
					}
					else if(in_black_line_flag==2){
						in_stop_cnt++;
						if(in_stop_cnt>=5){
							in_stop_cnt = 5;
							in_black_line_flag = 3;
							{speed = 0;omg = 0;turn_flag = 0;}
							g_agv_task_state = RELEASE_PLANE_IN;//agv stop and charge
							u1_printf("find third line stop,agv change...\r\n");
						}
						else{
							{speed = -150;omg = 0;turn_flag = 0;}
							u1_printf("go back,find third line...\r\n");
						}
					}
				}
				else if(dir==OUT){
					if(out_black_line_flag==0){
						out_stop_cnt++;
						if(out_stop_cnt>1){
							out_stop_cnt = 1;
							out_black_line_flag = 1;
							g_auto_pick_state = ONE;//夹取无人机
							{speed = 0;omg = 0;turn_flag = 0;}
							u1_printf("first line stop,pick plane...\r\n");
						}
						else{
							{speed = 150;omg = 0;turn_flag = 0;}
							u1_printf("first line find...\r\n");
						}
					}
					else if(out_black_line_flag==1){
						out_stop_cnt++;
						if(out_stop_cnt>2){
							out_stop_cnt = 2;
							out_black_line_flag = 2;
						}
						{speed = 150;omg = 0;turn_flag = 0;}
						u1_printf("second line not stop,go out...\r\n");
					}
					else if(out_black_line_flag==2){
						out_stop_cnt++;
						if(out_stop_cnt>4){
							out_stop_cnt = 4;
							out_black_line_flag = 3;
							{speed = 0;omg = 0;turn_flag = 0;}
							g_release_flag = ONE;//释放无人机
							u1_printf("third line stop,release plane...\r\n");
						}
						else{
							{speed = 100;omg = 0;turn_flag = 0;}
							u1_printf("go out,find third line...\r\n");
						}
					}
				}
			}
			else if(plane==NOPLANE)
			{
				if(dir==IN){
					if(in_black_line_flag1==0){
						in_stop_cnt1++;
						if(in_stop_cnt1>1){
							in_stop_cnt1 = 1;
							in_black_line_flag1 = 1;
							{speed = 0;omg = 0;turn_flag = 0;}
							g_auto_pick_state = ONE;//CLOSE PICK MACHINE
							u1_printf("first line,agv stop and close pick...\r\n");
						}
						else{
							u1_printf("first line find...\r\n");
							{speed = -150;omg = 0;turn_flag = 0;}
						}
					}
					else if(in_black_line_flag1==1){
						in_stop_cnt1++;
						if(in_stop_cnt1>=3){
							in_stop_cnt1 = 3;
							in_black_line_flag1 = 2;
							{speed = -150;omg = 0;turn_flag = 0;}
							u1_printf("second line,agv still go back...\r\n");
						}
						else{
							{speed = -150;omg = 0;turn_flag = 0;}
							u1_printf("go back,find second line...\r\n");
						}
					}
					else if(in_black_line_flag1==2){
						in_stop_cnt1++;
						if(in_stop_cnt1>=5){
							in_stop_cnt1 = 5;
							in_black_line_flag1 = 3;
							{speed = 0;omg = 0;turn_flag = 0;}
							g_agv_task_state = CHARGE_WITHOUT_PLANE;//切换到充电状态
							u1_printf("third line,agv stop and charge,plane is outside...\r\n");
						}
						else{
							{speed = -150;omg = 0;turn_flag = 0;}
							u1_printf("go back,find third line...\r\n");
						}
					}
				}
				else if(dir==OUT){
					if(out_black_line_flag1==0){
						out_stop_cnt1++;
						if(out_stop_cnt1>1){
							out_stop_cnt1 = 1;
							out_black_line_flag1 = 1;
						}
						u1_printf("first line,agv is still go out...\r\n");
						{speed = 150;omg = 0;turn_flag = 0;}
					}
					else if(out_black_line_flag1==1){
						out_stop_cnt1++;
						if(out_stop_cnt1>=3){
							out_stop_cnt1 = 3;
							out_black_line_flag1 = 2;
							g_release_flag = ONE;//open pick
							u1_printf("find second line,ready to open pick...\r\n");
							{speed = 0;omg = 0;turn_flag = 0;}
						}
						else{
							u1_printf("go out,find second line...\r\n");
							{speed = 150;omg = 0;turn_flag = 0;}
						}
					}
					else if(out_black_line_flag1==2){
						out_stop_cnt1++;
						if(out_stop_cnt1>=5){
							out_stop_cnt1 = 5;
							out_black_line_flag1 = 3;
							//g_agv_task_state = CVRTK_FIND_PLANE;
							u1_printf("find third line,ready to pick plane...\r\n");
							{speed = 0;omg = 0;turn_flag = 0;}
						}
						else{
							u1_printf("go out,find third line...\r\n");
							{speed = 150;omg = 0;turn_flag = 0;}
						}
					}
				}
			}
		}
		else if((Lineb.err<-1&&Linef.err<-1)||(Lineb.err<=1&&Lineb.err>=-1&&Linef.err<-1)||(Linef.err<=1&&Linef.err>=-1&&Lineb.err<-1)){
			{speed = 70;omg = 45;turn_flag = 1;}//turn riht
			u1_printf("turn right hmcf:%d hmcb:%d\r\n",Linef.mid,Lineb.mid);
		}
		else if((Lineb.err>1&&Linef.err>1)||(Lineb.err<=1&&Lineb.err>=-1&&Linef.err>1)||(Linef.err<=1&&Linef.err>=-1&&Lineb.err>1)){
			{speed = 70;omg = 45;turn_flag = -1;}//turn left
			u1_printf("turn left hmcf:%d hmcb:%d\r\n",Linef.mid,Lineb.mid);
		}
		else if(Lineb.err>1&&Linef.err<-1){
			{speed = 70;omg = 90;turn_flag = 0;}//run riht
			u1_printf("run right hmcf:%d hmcb:%d\r\n",Linef.mid,Lineb.mid);
		}
		else if(Lineb.err<-1&&Linef.err>1){
			{speed = 70;omg = -90;turn_flag = 0;}//run left
			u1_printf("run left hmcf:%d hmcb:%d\r\n",Linef.mid,Lineb.mid);
		}
		else if(T_hmcf.flag==0&&T_hmcb.flag==0){
			{speed = 0;omg = 0;turn_flag = 0;}
			u1_printf("miss line,stop\r\n");
		}
		else{
			if(dir==OUT){
				speed = 150;
				u1_printf("go out hmcf:%d hmcb:%d\r\n",T_hmcf.flag,T_hmcb.flag);
			}
			else if(dir==IN){
				speed = -150;
				u1_printf("go back hmcf:%d hmcb:%d\r\n",T_hmcf.flag,T_hmcb.flag);
			}
			{omg = 0;turn_flag = 0;}
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


