#include "control.h"
#include "ZL5SERVO.h"
#include "PickPlane.h"
#include "bsp_usart.h"
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
					u1_printf("release plane is ok...\r\n");
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
		u1_printf("pickplane is open ok...\r\n");
	}
}

uint8_t g_pick_state = NONE;
uint8_t g_dragonfish_flag = 0;//搜索龙鱼标志 0默认搜索到,1没有搜索到
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
			u1_printf("find plane is ok...\r\n");
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
					PP_Motor_Push();
					Catch_Motor_Close();
					if(Eject.catch0==2&&Eject.catch1==2&&Eject.puspul0==1&&Eject.puspul1==1){
						PP_Motor_Stop();
						Catch_Motor_Stop();
						g_pick_state = THREE;
						u1_printf("pick plane is ok...\r\n");
					}
				break;
				case THREE:
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
					u1_printf("pick plane is ok...\r\n");
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
	else if((rc_data_flag==0&&AGV_CMD.agv_cmd==NOTASK)){
		g_agv_work_mode = STANDBY;
	}
}

uint8_t g_rgb_cmd = 0;
void RGB_Ctr_Task(void)
{
	static uint8_t cnt = 0;
	uint8_t buf[8] = {0};
	uint8_t warm = 0,cold = 0x50,red = 0,green = 0,blue = 0xff;
	cnt++;
	if(cnt>20){
		cnt = 0;
		buf[0] = 0;
		buf[1] = _1HZ;
		buf[2] = _BREATH;
		buf[3] = warm;
		buf[4] = cold;
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

#define SPEED	180
#define SLOW_SPEED	100
#define TURN_SPEED	80
static int16_t speed = 0;
static float omg = 0;
static int8_t turn_flag = 0;//自旋标志
void FollowLine_process(uint8_t dir)
{
	if(Lineb.wlineflag!=1)
	{
		if((Lineb.err<=-2&&Linef.err<=-2)||(Lineb.err<=1&&Lineb.err>=-1&&Linef.err<-1)||(Linef.err<=1&&Linef.err>=-1&&Lineb.err<-1)){
			{speed = TURN_SPEED;omg = 45;turn_flag = 1;}//turn riht
			u1_printf("turn right hmcf_mid:%d hmcb_mid:%d\r\n",Linef.mid,Lineb.mid);
		}
		else if((Lineb.err>=2&&Linef.err>=2)||(Lineb.err<=1&&Lineb.err>=-1&&Linef.err>1)||(Linef.err<=1&&Linef.err>=-1&&Lineb.err>1)){
			{speed = TURN_SPEED;omg = 45;turn_flag = -1;}//turn left
			u1_printf("turn left hmcf_mid:%d hmcb_mid:%d\r\n",Linef.mid,Lineb.mid);
		}
		else if(Lineb.err>=2&&Linef.err<-2){
			{speed = TURN_SPEED;omg = 90;turn_flag = 0;}//run riht
			u1_printf("run right hmcf_mid:%d hmcb_mid:%d\r\n",Linef.mid,Lineb.mid);
		}
		else if(Lineb.err<=-2&&Linef.err>2){
			{speed = TURN_SPEED;omg = -90;turn_flag = 0;}//run left
			u1_printf("run left hmcf_mid:%d hmcb_mid:%d\r\n",Linef.mid,Lineb.mid);
		}
		else if(T_hmcf.flag==0&&T_hmcb.flag==0){
			{speed = 0;omg = 0;turn_flag = 0;}
			u1_printf("miss line,stop\r\n");
		}
		else{
			if(dir==OUT){
				speed = SPEED;
				u1_printf("go out hmcf_mid:%d hmcb_mid:%d\r\n",Linef.mid,Lineb.mid);
			}
			else if(dir==IN){
				speed = -SPEED;
				u1_printf("go back hmcf_mid:%d hmcb_mid:%d\r\n",Linef.mid,Lineb.mid);
			}
			{omg = 0;turn_flag = 0;}
		}
	}
	else{
		if(dir==OUT){
			speed = SPEED;
			u1_printf("go out hmcf_mid:%d hmcb_mid:%d\r\n",Linef.mid,Lineb.mid);
		}
		else if(dir==IN){
			speed = -SPEED;
			u1_printf("go back hmcf_mid:%d hmcb_mid:%d\r\n",Linef.mid,Lineb.mid);
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
		else if(g_dragonfish_flag==1){//在等待搜索飞机
			speed = 0;omg = 0;turn_flag = 0;
		}
		else if(plane==WITHPLANE)
		{
			if(dir==IN)
			{
				if(in_black_line_state==-3&&line_flag==1&&last_line_flag!=1){
					in_black_line_state = -2;
					{speed = -SPEED;omg = 0;turn_flag = 0;}
					u1_printf("release plane line find...\r\n");
				}
				else if(in_black_line_state==-2&&line_flag!=1&&last_line_flag==1){
					in_black_line_state = -1;
					{speed = -SPEED;omg = 0;turn_flag = 0;}
					u1_printf("release plane line miss...\r\n");
				}
				else if(in_black_line_state==-1&&line_flag==1&&last_line_flag!=1){
					in_black_line_state = 0;
					{speed = -SPEED;omg = 0;turn_flag = 0;}
					u1_printf("first line find,agv not stop still go back...\r\n");
				}
				else if(in_black_line_state==0&&line_flag!=1&&last_line_flag==1){
					in_black_line_state = 1;
					{speed = -SLOW_SPEED;omg = 0;turn_flag = 0;}
					u1_printf("first line miss,agv not stop still go back...\r\n");
				}
				else if(in_black_line_state==1&&line_flag==1&&last_line_flag!=1){
					in_black_line_state = 2;
					g_release_flag = ONE;//agv 停止，释放龙鱼
					{speed = 0;omg = 0;turn_flag = 0;}
					u1_printf("second line find,agv stop and release plane\r\n");
				}
				else if(in_black_line_state==2&&line_flag!=1&&last_line_flag==1){
					in_black_line_state = 3;
					{speed = -SPEED;omg = 0;turn_flag = 0;}
					u1_printf("second line miss,agv go back home\r\n");
				}
				else if(in_black_line_state==3&&line_flag==1&&last_line_flag!=1){
					in_black_line_state = 4;
					{speed = -SPEED;omg = 0;turn_flag = 0;}
					u1_printf("third line find...\r\n");
				}
				else if(in_black_line_state==4&&line_flag!=1&&last_line_flag==1){
					in_black_line_state = 5;
					{speed = 0;omg = 0;turn_flag = 0;}
					g_agv_task_state = RELEASE_PLANE_IN;//agv 停止，充电
					u1_printf("third line miss,agv stop change...\r\n");
				}
				else{
					FollowLine_process(IN);
				}
			}
			else if(dir==OUT){
				if(out_black_line_state==-3&&line_flag==1&&last_line_flag!=1){
					out_black_line_state = -2;
					{speed = SPEED;omg = 0;turn_flag = 0;}
					u1_printf("charge line find...\r\n");
				}
				else if(out_black_line_state==-2&&line_flag!=1&&last_line_flag==1){
					out_black_line_state = -1;
					{speed = SPEED;omg = 0;turn_flag = 0;}
					u1_printf("charge line miss...\r\n");
				}
				else if(out_black_line_state==-1&&line_flag==1&&last_line_flag!=1){
					out_black_line_state = 0;
					{speed = SLOW_SPEED;omg = 0;turn_flag = 0;}
					u1_printf("first line find...\r\n");
				}
				else if(out_black_line_state==0&&line_flag!=1&&last_line_flag==1){
					out_black_line_state = 1;
					g_pick_state = ONE;//夹取无人机
					{speed = 0;omg = 0;turn_flag = 0;}
					u1_printf("first line miss,agv stop and pick plane...\r\n");
				}
				else if(out_black_line_state==1&&line_flag==1&&last_line_flag!=1){
					out_black_line_state = 2;
					{speed = SPEED;omg = 0;turn_flag = 0;}
					u1_printf("second line find...\r\n");
				}
				else if(out_black_line_state==2&&line_flag!=1&&last_line_flag==1){
					out_black_line_state = 3;
					{speed = SPEED;omg = 0;turn_flag = 0;}
					u1_printf("second line miss,agv still go out...\r\n");
				}
				else if(out_black_line_state==3&&line_flag==1&&last_line_flag!=1){
					out_black_line_state = 4;
					{speed = SPEED;omg = 0;turn_flag = 0;}
					u1_printf("third line find...\r\n");
				}
				else if(out_black_line_state==4&&line_flag!=1&&last_line_flag==1){
					out_black_line_state = 5;
					{speed = 0;omg = 0;turn_flag = 0;}
					g_release_flag = ONE;//释放无人机
					u1_printf("third line miss,agv stop and release plane...\r\n");
				}
				else{
					FollowLine_process(OUT);
				}
			}
		}
		else if(plane==NOPLANE)
		{
			if(dir==IN){
				if(in_black_line_state1==-3&&line_flag==1&&last_line_flag!=1){
					in_black_line_state1 = -2;
					{speed = -SPEED;omg = 0;turn_flag = 0;}
					u1_printf("release plane line find...\r\n");
				}
				else if(in_black_line_state1==-2&&line_flag!=1&&last_line_flag==1){
					in_black_line_state1 = -1;
					{speed = -SPEED;omg = 0;turn_flag = 0;}
					u1_printf("release plane line miss...\r\n");
				}
				else if(in_black_line_state1==-1&&line_flag==1&&last_line_flag!=1){
					in_black_line_state1 = 0;
					g_pick_state = ONE;//收起夹具
					{speed = 0;omg = 0;turn_flag = 0;}
					u1_printf("first line find,agv stop and close pick...\r\n");
				}
				else if(in_black_line_state1==0&&line_flag!=1&&last_line_flag==1){
					in_black_line_state1 = 1;
					{speed = -SPEED;omg = 0;turn_flag = 0;}
					u1_printf("first line miss...\r\n");
				}
				else if(in_black_line_state1==1&&line_flag==1&&last_line_flag!=1){
					in_black_line_state1 = 2;
					{speed = -SPEED;omg = 0;turn_flag = 0;}
					u1_printf("second line find...\r\n");
				}
				else if(in_black_line_state1==2&&line_flag!=1&&last_line_flag==1){
					in_black_line_state1 = 3;
					{speed = -SPEED;omg = 0;turn_flag = 0;}
					u1_printf("second line miss,agv still go back...\r\n");
				}
				else if(in_black_line_state1==3&&line_flag==1&&last_line_flag!=1){
					in_black_line_state1 = 4;
					{speed = -SPEED;omg = 0;turn_flag = 0;}
					u1_printf("third line find...\r\n");
				}
				else if(in_black_line_state1==4&&line_flag!=1&&last_line_flag==1){
					in_black_line_state1 = 5;
					{speed = 0;omg = 0;turn_flag = 0;}
					g_agv_task_state = CHARGE_WITHOUT_PLANE;//切换到充电状态
					u1_printf("third line miss,agv stop and charge,plane is outside...\r\n");
				}
				else{
					FollowLine_process(IN);
				}
			}
			else if(dir==OUT){
				if(out_black_line_state1==-3&&line_flag==1&&last_line_flag!=1){
					out_black_line_state1 = -2;
					{speed = SPEED;omg = 0;turn_flag = 0;}
					u1_printf("charge line find...\r\n");
				}
				else if(out_black_line_state1==-2&&line_flag!=1&&last_line_flag==1){
					out_black_line_state1 = -1;
					{speed = SPEED;omg = 0;turn_flag = 0;}
					u1_printf("charge line miss...\r\n");
				}
				else if(out_black_line_state1==-1&&line_flag==1&&last_line_flag!=1){
					out_black_line_state1 = 0;
					{speed = SPEED;omg = 0;turn_flag = 0;}
					u1_printf("first line find...\r\n");
				}
				else if(out_black_line_state1==0&&line_flag!=1&&last_line_flag==1){
					out_black_line_state1 = 1;
					u1_printf("first line miss,agv is still go out...\r\n");
					{speed = SPEED;omg = 0;turn_flag = 0;}
				}
				else if(out_black_line_state1==1&&line_flag==1&&last_line_flag!=1){
					out_black_line_state1 = 2;
					u1_printf("second line find...\r\n");
					{speed = SPEED;omg = 0;turn_flag = 0;}
				}
				else if(out_black_line_state1==2&&line_flag!=1&&last_line_flag==1){
					out_black_line_state1 = 3;
					g_release_flag = ONE;//打开夹具
					u1_printf("second line miss,ready to open pick...\r\n");
					{speed = 0;omg = 0;turn_flag = 0;}
				}
				else if(out_black_line_state1==3&&line_flag==1&&last_line_flag!=1){
					out_black_line_state1 = 4;
					u1_printf("third line find...\r\n");
					{speed = SPEED;omg = 0;turn_flag = 0;}
				}
				else if(out_black_line_state1==4&&line_flag!=1&&last_line_flag==1){
					out_black_line_state1 = 5;
					//g_agv_task_state = CVRTK_FIND_PLANE;//切换到视觉RTK搜索龙鱼
					g_dragonfish_flag = 1;//要搜索龙鱼
					u1_printf("third line miss,ready to pick plane...\r\n");
					{speed = 0;omg = 0;turn_flag = 0;}
				}
				else{
					FollowLine_process(OUT);
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

#define OMG_ERR	3.0f//角度死区 °
#define DIS_ERR	5.0f//距离偏差 cm
const float p_gain = 2,d_gain = 10;
uint8_t Chassic_Pid_Ctr(float diserr,float omgerr,int8_t flag)
{
	uint8_t res = 0;
	int8_t turn = 0;
	if(flag!=0){//自旋
		if(myabs(omgerr) - OMG_ERR > 0.001f){
			if(omgerr - OMG_ERR > 0.001f){
				turn = 1;
			}
			else if(omgerr - OMG_ERR < -0.001f){
				turn = -1;
			}
			Chassic_Motor_Ctr(myabs(omgerr*p_gain),45.0f,turn);
			u1_printf("agv is controled by w pid,werr:%.2f...\r\n",omgerr);
			res = 0;
		}
		else{
			res = 1;
		}
	}
	else{
		if(myabs(diserr) - DIS_ERR > 0.001f){
			if(diserr - DIS_ERR > 0.001f){
				Chassic_Motor_Ctr(myabs(diserr*p_gain),90.0f,0);
				u1_printf("agv is controled by dis pid,diserr:%.2f...\r\n",diserr);
			}
			else if(diserr - DIS_ERR < -0.001f){
				Chassic_Motor_Ctr(myabs(diserr*p_gain),-90.0f,0);
				u1_printf("agv is controled by dis pid,diserr:%.2f...\r\n",diserr);
			}
			res = 0;
		}
		else{
			res = 1;
		}
	}
	return res;
}

const float agv_drg_dis = 1.2f;//m AGV距离龙鱼
uint8_t g_cvrtk_findplane_state = NONE,g_cvrtk_findplane_flag = 0;
void Auto_CVRTK_FindPlane(void)
{
	double disx_err = 0,disy_err = 0,angw_err = 0;
	double dis_err = 0,w_err = 0,/*ang_err = 0,*/x3 = 0,y3 = 0;
	if(g_cvrtk_findplane_flag==0){
		g_cvrtk_findplane_flag = 1;
	}
	if(g_cvrtk_findplane_flag!=0){
		pos_analysis(agvrtk.lon,agvrtk.lat,agvrtk.ang,drgrtk.lon,drgrtk.lat,drgrtk.ang,&Pos);
		disx_err = Pos.drg_x - Pos.agv_x;
		disy_err = Pos.drg_y - Pos.agv_y;
		angw_err  = Pos.drg_ang - Pos.agv_ang;
		
		w_err = drgrtk.ang-270.0f;
		x3 = Pos.drg_x + agv_drg_dis*cos(w_err);
		x3 = Pos.drg_y - agv_drg_dis*sin(w_err);
		dis_err = sqrt(pow((x3-Pos.agv_x),2)+pow((y3-Pos.agv_y),2))*100.0f;//cm

		switch(g_cvrtk_findplane_state){
			case ONE:
				if(Chassic_Pid_Ctr(0,angw_err,1)){
					u1_printf("agv is arrived at drgon fish's head...\r\n");
					g_cvrtk_findplane_state = TWO;
				}
			break;
			case TWO:
				if(Chassic_Pid_Ctr(dis_err,0,0)){
					u1_printf("agv is arrived at drgon fish's front...\r\n");
					g_cvrtk_findplane_state = THREE;
				}	
			break;
			case THREE:
				if(g_dragonfish_flag!=0){//还没找到龙鱼
					Chassic_Motor_Ctr(SPEED,0,0);
					u1_printf("agv go ahead,try to pick drgon fish...\r\n");
				}
				else{
					Chassic_Motor_Ctr(0,0,0);//找到龙鱼，停止AGV
					u1_printf("drgon fish is find,agv stop and pick...\r\n");
					g_cvrtk_findplane_state = FOUR;
				}
			break;
			case FOUR:
				
			g_cvrtk_findplane_state = DONE;
			break;
		}
//	chassic_control_task(CV.agv_spx,CV.agv_spy,CV.agv_spw);
//	Pick_Plane_Ctr_Task(CV.pick_spcatch,CV.pick_sppp,CV.pick_spupdown);
	}
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

