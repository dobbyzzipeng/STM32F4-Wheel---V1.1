#include "control.h"
#include "ZL5SERVO.h"
#include "PickPlane.h"
#include "usart.h"
#include "led.h"
#include "can1.h"
#include "can2.h"
#include "comunication.h"
#include "hmcsensor.h"
#include "postion.h"
#include "rtk.h"
#include "gps.h"
#include <math.h>

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
					u5_printf("release plane is ok...\r\n");
				}
			break;
			case FOUR:
				g_release_flag = DONE;
				if(agvdir==OUT){
					g_agv_task_state = GOHOME_WITHOUT_PLANE;//first release,next goback for space
				}
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
		g_release_flag = DONE;
		u5_printf("pickplane is open ok...\r\n");
	}
}

uint8_t g_pick_state = NONE;
uint8_t g_dragonfish_flag = 1;//搜索龙鱼标志 0搜索到,1没有搜索到
void Auto_Pick_Plane(uint8_t flag,uint8_t dir)
{
	static uint16_t catime = 0,catflag = 0;
	if(flag==FIND){
		if(T_Plane.g_plane_flag1==1&&T_Plane.g_plane_flag2==1\
	&&T_Plane.last_plane_flag1==0&&T_Plane.last_plane_flag2==0){
			g_pick_state = ONE;
			g_dragonfish_flag = 0;//搜索到龙鱼
			T_Plane.last_plane_flag1=1;
			T_Plane.last_plane_flag2=1;
			u5_printf("find plane is ok...\r\n");
		}
		else if(T_Plane.g_plane_flag1==0&&T_Plane.g_plane_flag2==0){
			T_Plane.last_plane_flag1=0;
			T_Plane.last_plane_flag2=0;
		}
		if(dir==IN){
			switch(g_pick_state){
				case ONE:
					UD_Motor_Up();
					if(Eject.updown0==2&&Eject.updown1==2){
						UD_Motor_Stop();
						g_pick_state = TWO;
					}
				break;
				case TWO:
					Catch_Motor_Close();
					if(Eject.catch0==2&&Eject.catch1==2){
						Catch_Motor_Stop();
						g_pick_state = THREE;
						u5_printf("pick plane is ok...\r\n");
					}
				break;
				case THREE:
					PP_Motor_Push();
					if(Eject.puspul0==1&&Eject.puspul1==1){
						PP_Motor_Stop();
						g_pick_state = FOUR;
						u5_printf("pick plane is ok...\r\n");
					}
				break;
				case FOUR:
					g_pick_state = DONE;
				break;
			}
		}
		else if(dir==OUT){
			switch(g_pick_state){
				case ONE:
					if(catflag==0){
						catflag = 1;
						Catch_Motor_Close();
					}
					else{
						catime++;
						if(catime>340){//17s
							catime = 0;//只关一半
							Catch_Motor_Stop();
							g_pick_state = TWO;
						}
					}
				break;
				case TWO:
					UD_Motor_Up();
					if(Eject.updown0==2&&Eject.updown1==2){
						UD_Motor_Stop();
						g_pick_state = THREE;
					}
				break;
				case THREE:
					PP_Motor_Push();
					Catch_Motor_Close();
					if(Eject.puspul0==1&&Eject.puspul1==1&&Eject.catch0==2&&Eject.catch1==2){
						Catch_Motor_Stop();
						PP_Motor_Stop();
						g_pick_state = FOUR;
					}
				break;
				case FOUR:
					u5_printf("pick plane is ok...\r\n");
					g_pick_state = DONE;
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
		g_pick_state = NONE;//清空状态机标志
		u5_printf("pick machine is close...\r\n");
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
	else if((rc_data_flag==0&&AGV_CMD.agv_cmd==NOTASK)){
		g_agv_work_mode = STANDBY;
	}
}

uint8_t g_rgb_cmd = 0;
void RGB_Ctr_Task(void)
{
	static uint8_t cnt = 0;
	uint8_t buf[8] = {0};
	uint8_t warm = 0,cold = 0x50,red = 0xff,green = 0xff,blue = 0xff;
	cnt++;
	if(cnt>20){
		cnt = 0;
		buf[0] = _1HZ;
		buf[1] = 254;
		buf[2] = _BREATH;
		buf[3] = cold;
		buf[4] = warm;
		buf[5] = blue;
		buf[6] = green;
		buf[7] = red;
		CAN2_TX_PACKET(0X301,buf,sizeof(buf));
	}
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

static int8_t in_black_line_state = -3;//in
static int8_t out_black_line_state = -3;//out
static int8_t in_black_line_state1 = -3;//in
static int8_t out_black_line_state1 = -3;//out
void Follow_Line_Clear(void)
{
	in_black_line_state = -3;
	out_black_line_state = -3;
	in_black_line_state1 = -3;
	out_black_line_state1 = -3;
}

#define SPEED	450
#define SLOW_SPEED	220
#define TURN_SPEED	100
static int16_t speed = 0;
static float omg = 0;
static int8_t turn_flag = 0;//自旋标志
void FollowLine_process(uint8_t dir,uint8_t lrlimit)
{
	static uint8_t follow_flag = STOP;//,last_follow_flag = STOP;
	if(Lineb.wlineflag!=1)
	{
//		float f_pos = Linef.err;
//		float b_pos = -1.0f * Lineb.err;
		
//		last_follow_flag = follow_flag;
		if (follow_flag == RUN_RIGHT)
		{
			{speed = TURN_SPEED;omg = 90;turn_flag = 0;}//run riht
			u5_printf("run right hmcf_mid:%.1f hmcb_mid:%.1f\r\n",Linef.mid,Lineb.mid);
			if (Lineb.err < 0 || Linef.err > 0)
			{
				follow_flag = GO;
			}
			return;
		}
		else if (follow_flag == RUN_LEFT)
		{
			{speed = TURN_SPEED;omg = -90;turn_flag = 0;}//run left
			u5_printf("run left hmcf_mid:%.1f hmcb_mid:%.1f\r\n",Linef.mid,Lineb.mid);
			if (Lineb.err > 0 || Linef.err < 0)
			{
				follow_flag = GO;
			}
			return;
		}
		
		if(T_hmcf.flag==0&&T_hmcb.flag==0){
			follow_flag = STOP;
			{speed = 0;omg = 0;turn_flag = 0;}
			u5_printf("miss line,stop\r\n");
		}
		else if((Lineb.err<=-2&&Linef.err<=-2)||(Lineb.err<=1&&Lineb.err>=-1&&Linef.err<-1)||(Linef.err<=1&&Linef.err>=-1&&Lineb.err<-1)){
			follow_flag = TURN_RIGHT;
			{speed = TURN_SPEED;omg = 45;turn_flag = 1;}//turn riht
			u5_printf("turn right hmcf_mid:%.1f hmcb_mid:%.1f\r\n",Linef.mid,Lineb.mid);
		}
		else if((Lineb.err>=2&&Linef.err>=2)||(Lineb.err<=1&&Lineb.err>=-1&&Linef.err>1)||(Linef.err<=1&&Linef.err>=-1&&Lineb.err>1)){
			follow_flag = TURN_LEFT;
			{speed = TURN_SPEED;omg = 45;turn_flag = -1;}//turn left
			u5_printf("turn left hmcf_mid:%.1f hmcb_mid:%.1f\r\n",Linef.mid,Lineb.mid);
		}
		else if(Lineb.err>(3-lrlimit) && Linef.err<-(3-lrlimit)){
			follow_flag = RUN_RIGHT;
		}
		else if(Lineb.err<-(3-lrlimit) && Linef.err>(3-lrlimit)){
			follow_flag = RUN_LEFT;
		}
		else{
			if(dir==OUT){
				follow_flag = GO;
				speed = SPEED;
				u5_printf("go out hmcf_mid:%.1f hmcb_mid:%.1f\r\n",Linef.mid,Lineb.mid);
			}
			else if(dir==IN){
				follow_flag = GO;
				speed = -SPEED;
				u5_printf("go back hmcf_mid:%.1f hmcb_mid:%.1f\r\n",Linef.mid,Lineb.mid);
			}
			{omg = 0;turn_flag = 0;}
		}
	}
	else{
		if(dir==OUT){
			follow_flag = GO;
			speed = SPEED;
			u5_printf("go out hmcf_mid:%.1f hmcb_mid:%.1f\r\n",Linef.mid,Lineb.mid);
		}
		else if(dir==IN){
			follow_flag = GO;
			speed = -SPEED;
			u5_printf("go back hmcf_mid:%.1f hmcb_mid:%.1f\r\n",Linef.mid,Lineb.mid);
		}
		{omg = 0;turn_flag = 0;}
	}
}

static uint8_t line_flag = 1,last_line_flag = 0;//后边磁导轨传感器遇到横线标志，1为横线
void Auto_FollowLine_Task(uint8_t dir,uint8_t plane)
{
	static uint16_t runcnt = 0;
	runcnt++;
	if(runcnt>10){//2HZ不可修改
		runcnt = 0;
		Line_Analysis(T_hmcf.flag,&Linef);
		Line_Analysis(T_hmcb.flag,&Lineb);
		last_line_flag = line_flag;
		line_flag = Lineb.wlineflag;
		
		if(g_release_flag!=NONE && g_release_flag!=DONE){//还没有放完飞机
			speed = 0;omg = 0;turn_flag = 0;
		}
		else if(g_pick_state!=NONE && g_pick_state!=DONE){//还没有抬起飞机
			speed = 0;omg = 0;turn_flag = 0;
		}
		else if(plane==WITHPLANE)
		{
			if(dir==IN)
			{
				if(in_black_line_state==-3&&line_flag==1&&last_line_flag!=1){
					in_black_line_state = -2;
					{speed = -SPEED;omg = 0;turn_flag = 0;}
					u5_printf("release plane line find...\r\n");
				}
				else if(in_black_line_state==-2&&line_flag!=1&&last_line_flag==1){
					in_black_line_state = -1;
					{speed = -SPEED;omg = 0;turn_flag = 0;}
					u5_printf("release plane line miss...\r\n");
				}
				else if(in_black_line_state==-1&&line_flag==1&&last_line_flag!=1){
					in_black_line_state = 0;
					{speed = -SPEED;omg = 0;turn_flag = 0;}
					u5_printf("first line find,agv not stop still go back...\r\n");
				}
				else if(in_black_line_state==0&&line_flag!=1&&last_line_flag==1){
					in_black_line_state = 1;
					{speed = -SLOW_SPEED;omg = 0;turn_flag = 0;}
					u5_printf("first line miss,agv not stop still go back...\r\n");
				}
				else if(in_black_line_state==1&&line_flag==1&&last_line_flag!=1){
					in_black_line_state = 2;
					g_release_flag = ONE;//agv 停止，释放龙鱼
					{speed = 0;omg = 0;turn_flag = 0;}
					u5_printf("second line find,agv stop and release plane\r\n");
				}
				else if(in_black_line_state==2&&line_flag!=1&&last_line_flag==1){
					in_black_line_state = 3;
					{speed = -SPEED;omg = 0;turn_flag = 0;}
					u5_printf("second line miss,agv go back home\r\n");
				}
				else if(in_black_line_state==3&&line_flag==1&&last_line_flag!=1){
					in_black_line_state = 4;
					{speed = -SLOW_SPEED;omg = 0;turn_flag = 0;}
					u5_printf("third line find...\r\n");
				}
				else if(in_black_line_state==4&&line_flag!=1&&last_line_flag==1){
					in_black_line_state = 5;
					{speed = 0;omg = 0;turn_flag = 0;}
					g_agv_task_state = RELEASE_PLANE_IN;//agv 停止，充电
					u5_printf("third line miss,agv stop change...\r\n");
				}
				else{
					FollowLine_process(IN,LIMIT);
				}
			}
			else if(dir==OUT){
				if(out_black_line_state==-3&&line_flag==1&&last_line_flag!=1){
					out_black_line_state = -2;
					{speed = SPEED;omg = 0;turn_flag = 0;}
					u5_printf("charge line find...\r\n");
				}
				else if(out_black_line_state==-2&&line_flag!=1&&last_line_flag==1){
					out_black_line_state = -1;
					{speed = SPEED;omg = 0;turn_flag = 0;}
					u5_printf("charge line miss...\r\n");
				}
				else if(out_black_line_state==-1&&line_flag==1&&last_line_flag!=1){
					out_black_line_state = 0;
					{speed = SLOW_SPEED;omg = 0;turn_flag = 0;}
					u5_printf("first line find...\r\n");
				}
				else if(out_black_line_state==0&&line_flag!=1&&last_line_flag==1){
					out_black_line_state = 1;
					g_pick_state = ONE;//夹取无人机
					{speed = 0;omg = 0;turn_flag = 0;}
					u5_printf("first line miss,agv stop and pick plane...\r\n");
				}
				else if(out_black_line_state==1&&line_flag==1&&last_line_flag!=1){
					out_black_line_state = 2;
					{speed = SPEED;omg = 0;turn_flag = 0;}
					u5_printf("second line find...\r\n");
				}
				else if(out_black_line_state==2&&line_flag!=1&&last_line_flag==1){
					out_black_line_state = 3;
					{speed = SPEED;omg = 0;turn_flag = 0;}
					u5_printf("second line miss,agv still go out...\r\n");
				}
				else if(out_black_line_state==3&&line_flag==1&&last_line_flag!=1){
					out_black_line_state = 4;
					{speed = SPEED;omg = 0;turn_flag = 0;}
					u5_printf("third line find...\r\n");
				}
				else if(out_black_line_state==4&&line_flag!=1&&last_line_flag==1){
					out_black_line_state = 5;
					{speed = 0;omg = 0;turn_flag = 0;}
					g_release_flag = ONE;//释放无人机
					u5_printf("third line miss,agv stop and release plane...\r\n");
				}
				else{
					FollowLine_process(OUT,SONG);
				}
			}
		}
		else if(plane==NOPLANE)
		{
			if(dir==IN){
				if(in_black_line_state1==-3&&line_flag==1&&last_line_flag!=1){
					in_black_line_state1 = -2;
					{speed = -SPEED;omg = 0;turn_flag = 0;}
					u5_printf("release plane line find...\r\n");
				}
				else if(in_black_line_state1==-2&&line_flag!=1&&last_line_flag==1){
					in_black_line_state1 = -1;
					{speed = -SPEED;omg = 0;turn_flag = 0;}
					u5_printf("release plane line miss...\r\n");
				}
				else if(in_black_line_state1==-1&&line_flag==1&&last_line_flag!=1){
					in_black_line_state1 = 0;
					g_pick_state = ONE;//收起夹具
					{speed = 0;omg = 0;turn_flag = 0;}
					u5_printf("first line find,agv stop and close pick...\r\n");
				}
				else if(in_black_line_state1==0&&line_flag!=1&&last_line_flag==1){
					in_black_line_state1 = 1;
					{speed = -SPEED;omg = 0;turn_flag = 0;}
					u5_printf("first line miss...\r\n");
				}
				else if(in_black_line_state1==1&&line_flag==1&&last_line_flag!=1){
					in_black_line_state1 = 2;
					{speed = -SPEED;omg = 0;turn_flag = 0;}
					u5_printf("second line find...\r\n");
				}
				else if(in_black_line_state1==2&&line_flag!=1&&last_line_flag==1){
					in_black_line_state1 = 3;
					{speed = -SPEED;omg = 0;turn_flag = 0;}
					u5_printf("second line miss,agv still go back...\r\n");
				}
				else if(in_black_line_state1==3&&line_flag==1&&last_line_flag!=1){
					in_black_line_state1 = 4;
					{speed = -SPEED;omg = 0;turn_flag = 0;}
					u5_printf("third line find...\r\n");
				}
				else if(in_black_line_state1==4&&line_flag!=1&&last_line_flag==1){
					in_black_line_state1 = 5;
					{speed = 0;omg = 0;turn_flag = 0;}
					g_agv_task_state = CHARGE_WITHOUT_PLANE;//切换到充电状态
					u5_printf("third line miss,agv stop and charge,plane is outside...\r\n");
				}
				else{
					FollowLine_process(IN,LIMIT);
				}
			}
			else if(dir==OUT){
				if(out_black_line_state1==-3&&line_flag==1&&last_line_flag!=1){
					out_black_line_state1 = -2;
					{speed = SPEED;omg = 0;turn_flag = 0;}
					u5_printf("charge line find...\r\n");
				}
				else if(out_black_line_state1==-2&&line_flag!=1&&last_line_flag==1){
					out_black_line_state1 = -1;
					{speed = SPEED;omg = 0;turn_flag = 0;}
					u5_printf("charge line miss...\r\n");
				}
				else if(out_black_line_state1==-1&&line_flag==1&&last_line_flag!=1){
					out_black_line_state1 = 0;
					{speed = SPEED;omg = 0;turn_flag = 0;}
					u5_printf("first line find...\r\n");
				}
				else if(out_black_line_state1==0&&line_flag!=1&&last_line_flag==1){
					out_black_line_state1 = 1;
					u5_printf("first line miss,agv is still go out...\r\n");
					{speed = SPEED;omg = 0;turn_flag = 0;}
				}
				else if(out_black_line_state1==1&&line_flag==1&&last_line_flag!=1){
					out_black_line_state1 = 2;
					u5_printf("second line find...\r\n");
					{speed = SPEED;omg = 0;turn_flag = 0;}
				}
				else if(out_black_line_state1==2&&line_flag!=1&&last_line_flag==1){
					out_black_line_state1 = 3;
					g_release_flag = ONE;//打开夹具
					u5_printf("second line miss,ready to open pick...\r\n");
					{speed = 0;omg = 0;turn_flag = 0;}
				}
				else if(out_black_line_state1==3&&line_flag==1&&last_line_flag!=1){
					out_black_line_state1 = 4;
					u5_printf("third line find...\r\n");
					{speed = SPEED;omg = 0;turn_flag = 0;}
				}
				else if(out_black_line_state1==4&&line_flag!=1&&last_line_flag==1){
					out_black_line_state1 = 5;
					g_agv_task_state = CVRTK_FIND_PLANE;//切换到视觉RTK搜索龙鱼
					g_dragonfish_flag = 1;//要搜索龙鱼
					u5_printf("third line miss,ready to pick plane...\r\n");
					{speed = 0;omg = 0;turn_flag = 0;}
				}
				else{
					FollowLine_process(OUT,SONG);
				}
			}
		}
		Chassic_Motor_Ctr(speed,omg,turn_flag);
	}
}

void Chassic_Motor_Ctr(int16_t sp,float w,int8_t flag)
{
	if(flag != 0){//turn riht or turn left
		motor1_speed = flag*sp;
		motor2_speed = flag*sp;
		motor3_speed = flag*sp;
		motor4_speed = flag*sp;
		omgset_pos1 = -Limit(w*LSB,_90_ANGLE);
		omgset_pos2 = Limit(w*LSB,_90_ANGLE);
		omgset_pos3 = -Limit(w*LSB,_90_ANGLE);
		omgset_pos4 = Limit(w*LSB,_90_ANGLE);
	}
	else{//left right front back
		motor1_speed = -sp;
		motor2_speed = -sp;
		motor3_speed = sp;
		motor4_speed = sp;
		omgset_pos1 = Limit(w*LSB,_90_ANGLE);
		omgset_pos2 = Limit(w*LSB,_90_ANGLE);
		omgset_pos3 = Limit(w*LSB,_90_ANGLE);
		omgset_pos4 = Limit(w*LSB,_90_ANGLE);
	}
}

#define OMG_ERR	2.0f//角度死区 °
#define DIS_ERR	0.20f//距离偏差 m
const float p_gain = 15;
uint8_t Chassic_Pid_Ctr(float diserr,float omgerr,int8_t turn)//flag:1,自旋，0前进，2,左右
{
	uint8_t res = 0;
	int8_t flag = 0;
	double pint = 0;
	
	if(turn==TURN){//自旋
		if(myabs(omgerr) > OMG_ERR){
			if(omgerr > 0){
				flag = 1;//往右转
			}
			else if(omgerr < 0){
				flag = -1;////往左转
			}
			pint = myabs(omgerr*p_gain);
			if(pint>1000){pint = 1000;}
			else if(pint<-1000){pint = -1000;}
			Chassic_Motor_Ctr(pint,45.0f,flag);
//			u5_printf("agv is controled by w pid,werr:%.2f agvang:%.2f agvx:%.2f agvy:%.2f...\r\n",omgerr,Pos.agv_ang,Pos.agv_x,Pos.agv_y);
			res = 0;
		}
		else{
			res = 1;
		}
	}
	else if(turn==LR){//left right
		if (myabs(diserr) > DIS_ERR)
		{
			if(diserr > 0.0f){
				pint = 500;
				Chassic_Motor_Ctr(pint,90.0f,0);
//				u5_printf("agv is controled by lr pid,diserr:%.2f agvang:%.2f agvx:%.2f agvy:%.2f...\r\n",diserr,Pos.agv_ang,Pos.agv_x,Pos.agv_y);
			}
			else{
				pint = 500;
				Chassic_Motor_Ctr(pint,-90.0f,0);
//				u5_printf("agv is controled by lr pid,diserr:%.2f agvang:%.2f agvx:%.2f agvy:%.2f...\r\n",diserr,Pos.agv_ang,Pos.agv_x,Pos.agv_y);
			}
			res = 0;
		}
		else{
			res = 1;
		}
	}
	else if(turn==FB){//fb
		if(myabs(diserr - DIS_ERR) > 0.1f){
			if(diserr - DIS_ERR > 0.1f){
				pint = 500;
				Chassic_Motor_Ctr(-pint,0.0f,0);
//				u5_printf("agv is controled by fb pid,diserr:%.2f agvang:%.2f agvx:%.2f agvy:%.2f...\r\n",diserr,Pos.agv_ang,Pos.agv_x,Pos.agv_y);
			}
			else if(diserr - DIS_ERR < -0.1f){
				pint = 500;
				Chassic_Motor_Ctr(pint,0.0f,0);
//				u5_printf("agv is controled by fb pid,diserr:%.2f agvang:%.2f agvx:%.2f agvy:%.2f...\r\n",diserr,Pos.agv_ang,Pos.agv_x,Pos.agv_y);
			}
			res = 0;
		}
		else{
			res = 1;
		}
	}
	return res;
}

uint8_t g_cvrtk_findplane_state = NONE,g_cvrtk_findplane_flag = 0;
void Auto_CVRTK_FindPlane(void)
{
	static uint8_t rcnt = 0;
	double angw_err = 0;

	rcnt++;
	if(rcnt<5){return;}
	else{rcnt = 0;}
	
	if(g_cvrtk_findplane_flag==0){
		g_cvrtk_findplane_state = ONE;
		g_cvrtk_findplane_flag = 1;
	}
	else{
		if(agvrtk.good_ok!=0 && drgrtk.good_ok!=0){
			pos_analysis(agvrtk.lon,agvrtk.lat,agvrtk.ang,drgrtk.lon,drgrtk.lat,drgrtk.ang,&Pos);
			angw_err = Pos.drg_ang_org - Pos.agv_ang_org;
//			g_cvrtk_findplane_state = 0;
u5_printf("agvx:%.2f agvy:%.2f agvang:%.2f drgx:%.2f drgy:%.2f drgang:%.2f Qx:%.2f Qy:%.2f ver:%.2f agv2drg:%.2f agv2Q:%.2f\r\n", \
Pos.agv_x,Pos.agv_y,Pos.agv_ang,Pos.drg_x,Pos.drg_y,Pos.drg_ang,Pos.Q_x,Pos.Q_y,Pos.vert_to_drg2Q,Pos.agv2drg,Pos.agv2Q);
			
			switch(g_cvrtk_findplane_state){
				case ONE:
				if(Chassic_Pid_Ctr(0,angw_err,TURN)){
					u5_printf("agv is arrived at drgon fish's head...\r\n");
					g_cvrtk_findplane_state = TWO;
				}
				break;
				case TWO:
					Chassic_Motor_Ctr(0,0,0);
					g_cvrtk_findplane_state = THREE;
				break;
				case THREE:
					if(Chassic_Pid_Ctr(Pos.vert_to_drg2Q,0,LR)){//left right
						u5_printf("agv is arrived at drgon fish's xpos...\r\n");
						g_cvrtk_findplane_state = FOUR;
					}
				break;
				case FOUR:
					Chassic_Motor_Ctr(0,0,0);
					g_cvrtk_findplane_state = FIVE;
				break;
				case FIVE:
					if(Chassic_Pid_Ctr(0,angw_err,TURN)){
						u5_printf("agv is arrived at drgon fish's head...\r\n");
						g_cvrtk_findplane_state = SIX;
					}
				break;
				case SIX:
					Chassic_Motor_Ctr(0,0,0);
					g_cvrtk_findplane_state = SEVEN;
				break;
				case SEVEN:
					if(g_dragonfish_flag!=0){//还没找到龙鱼
						Chassic_Motor_Ctr(SPEED,0,0);
						u5_printf("agv go ahead,try to pick drgon fish...\r\n");
					}
					else{
						Chassic_Motor_Ctr(0,0,0);//找到龙鱼，停止AGV
						u5_printf("drgon fish is find,agv stop and pick...\r\n");
						g_cvrtk_findplane_state = EIGHT;
					}
				break;
				case EIGHT:
				g_cvrtk_findplane_state = DONE;
				break;
			}
		}
		else{
		u5_printf("rtk signal is not good!!,please wait...\r\n");
			Chassic_Motor_Ctr(0,0,0);
		}
	}
//	chassic_control_task(CV.agv_spx,CV.agv_spy,CV.agv_spw);
//	Pick_Plane_Ctr_Task(CV.pick_spcatch,CV.pick_sppp,CV.pick_spupdown);
}

void Auto_CVRTK_FindState_Clear(void)
{
	g_cvrtk_findplane_state = NONE;
	g_cvrtk_findplane_flag = 0;
}

uint8_t g_find_blackline_state = NONE,g_find_blackline_flag = 0;
void Auto_Find_BlackLine(void)
{
	static uint8_t rttcnt = 0;
	double w_err = 0,x3 = 0,y3 = 0;
	rttcnt++;
	if(rttcnt<5){return;}
	else{rttcnt = 0;}
	if(g_find_blackline_flag==0){
		g_find_blackline_flag = 1;
		g_find_blackline_state = ONE;
	}
	else{
		if(agvrtk.good_ok!=0 && drgrtk.good_ok!=0){
			pos_analysis(agvrtk.lon,agvrtk.lat,agvrtk.ang,drgrtk.lon,drgrtk.lat,drgrtk.ang,&Pos);
			w_err = 270.0f - Pos.agv_ang_org;
			x3 = 0.5f - Pos.agv_x;
			y3 = -0.1f - Pos.agv_y;
			
			switch(g_find_blackline_state){
				case ONE:
					if(Chassic_Pid_Ctr(0,w_err,TURN)){
						u5_printf("agv is come back,arrived at blackline's mid...\r\n");
						g_find_blackline_state = TWO;
					}
				break;
				case TWO:
					Chassic_Motor_Ctr(0,0,0);
					g_find_blackline_state = THREE;
				break;
				case THREE:
					if(Chassic_Pid_Ctr(-y3,0,LR)){
						u5_printf("agv is arrived at blackline's xpos...\r\n");
						g_find_blackline_state = FOUR;
					}
				break;
				case FOUR:
					Chassic_Motor_Ctr(0,0,0);
					g_find_blackline_state = FIVE;
				break;
				case FIVE:
					if(Chassic_Pid_Ctr(-x3,0,FB)){
						u5_printf("agv is arrived at blackline's ypos...\r\n");
						g_find_blackline_state = SIX;
					}
				break;
				case SIX:
					Chassic_Motor_Ctr(0,0,0);
					g_find_blackline_state = SEVEN;
				break;
				case SEVEN:
					u5_printf("agv is arrived at blackline's release_plane point...\r\n");
					g_find_blackline_state = DONE;
				break;
			}
		}
		else{
			u5_printf("rtk signal is not good!!,please wait...\r\n");
			Chassic_Motor_Ctr(0,0,0);
		}
	}
}

void Auto_Find_BlackLine_State_Clear(void)
{
	g_find_blackline_state = NONE;
	g_find_blackline_flag = 0;
}

int Limit(int data,int max)
{
	if(data>max){
		data = max;
	}
	else if(data<-max){
		data = -max;
	}
	return data;
}

//static const int32_t step = 16384;//14位编码器	204800
static int16_t delaycnt = 0;
static float womg = 0;
static float last_womg = 0;
void chassic_control_task(int16_t x,int16_t y,int16_t w)//x left and right,y:front and back,w
{
	int16_t speed = 0;
	int16_t ch1 = 0,ch0 = 0,ch3 = 0;
	int32_t omgea1 = 0,omgea2 = 0,omgea3 = 0,omgea4 = 0;
	
	ch1 = y-1000;//f b
	ch0 = x-1000;//l r
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
			delaycnt = 20;
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

void Pick_Plane_Ctr_Task(int16_t x,int16_t y,int16_t w)//x left and right,y:front and back,w
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

