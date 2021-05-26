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

uint8_t g_auto_pick_state = NONE;
uint8_t g_dragonfish_flag = 0;//搜索龙鱼标志
void Auto_Pick_Plane(uint8_t flag,uint8_t dir)
{
	if(flag==FIND){
		if(T_Plane.g_plane_flag1==1&&T_Plane.g_plane_flag2==1\
	&&T_Plane.last_plane_flag1==0&&T_Plane.last_plane_flag2==0){
			g_auto_pick_state = ONE;
			g_dragonfish_flag = 0;
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
	else if((rc_data_flag==0&&AGV_CMD.agv_cmd==NOTASK)){
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

static int16_t speed = 0;
static float omg = 0;
static int8_t turn_flag = 0;//自旋标志
void FollowLine_process(uint8_t dir)
{
	if(Lineb.wlineflag!=1)
	{
		if((Lineb.err<-2&&Linef.err<-2)||(Lineb.err<=1&&Lineb.err>=-1&&Linef.err<-1)||(Linef.err<=1&&Linef.err>=-1&&Lineb.err<-1)){
			{speed = 70;omg = 45;turn_flag = 1;}//turn riht
			u1_printf("turn right hmcf:%d hmcb:%d\r\n",Linef.mid,Lineb.mid);
		}
		else if((Lineb.err>2&&Linef.err>2)||(Lineb.err<=1&&Lineb.err>=-1&&Linef.err>1)||(Linef.err<=1&&Linef.err>=-1&&Lineb.err>1)){
			{speed = 70;omg = 45;turn_flag = -1;}//turn left
			u1_printf("turn left hmcf:%d hmcb:%d\r\n",Linef.mid,Lineb.mid);
		}
		else if(Lineb.err>2&&Linef.err<-2){
			{speed = 70;omg = 90;turn_flag = 0;}//run riht
			u1_printf("run right hmcf:%d hmcb:%d\r\n",Linef.mid,Lineb.mid);
		}
		else if(Lineb.err<-2&&Linef.err>2){
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
		else if(g_auto_pick_state!=NONE && g_auto_pick_state!=DONE){//还没有抬起飞机
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
					{speed = -150;omg = 0;turn_flag = 0;}
					u1_printf("release plane line find...\r\n");
				}
				else if(in_black_line_state==-2&&line_flag!=1&&last_line_flag==1){
					in_black_line_state = -1;
					{speed = -150;omg = 0;turn_flag = 0;}
					u1_printf("release plane line miss...\r\n");
				}
				else if(in_black_line_state==-1&&line_flag==1&&last_line_flag!=1){
					in_black_line_state = 0;
					{speed = -150;omg = 0;turn_flag = 0;}
					u1_printf("first line find,agv not stop still go back...\r\n");
				}
				else if(in_black_line_state==0&&line_flag!=1&&last_line_flag==1){
					in_black_line_state = 1;
					{speed = -150;omg = 0;turn_flag = 0;}
					u1_printf("first line miss,agv not stop still go back...\r\n");
				}
				else if(in_black_line_state==1&&line_flag==1&&last_line_flag!=1){
					in_black_line_state = 2;
					{speed = -150;omg = 0;turn_flag = 0;}
					u1_printf("second line find,agv ready to release plane\r\n");
				}
				else if(in_black_line_state==2&&line_flag!=1&&last_line_flag==1){
					in_black_line_state = 3;
					{speed = 0;omg = 0;turn_flag = 0;}
					g_release_flag = ONE;//agv 停止，释放龙鱼
					u1_printf("second line miss,agv stop and release plane\r\n");
				}
				else if(in_black_line_state==3&&line_flag==1&&last_line_flag!=1){
					in_black_line_state = 4;
					{speed = -100;omg = 0;turn_flag = 0;}
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
					{speed = 150;omg = 0;turn_flag = 0;}
					u1_printf("charge line find...\r\n");
				}
				else if(out_black_line_state==-2&&line_flag!=1&&last_line_flag==1){
					out_black_line_state = -1;
					{speed = 150;omg = 0;turn_flag = 0;}
					u1_printf("charge line miss...\r\n");
				}
				else if(out_black_line_state==-1&&line_flag==1&&last_line_flag!=1){
					out_black_line_state = 0;
					{speed = 150;omg = 0;turn_flag = 0;}
					u1_printf("first line find...\r\n");
				}
				else if(out_black_line_state==0&&line_flag!=1&&last_line_flag==1){
					out_black_line_state = 1;
					g_auto_pick_state = ONE;//夹取无人机
					{speed = 0;omg = 0;turn_flag = 0;}
					u1_printf("first line miss,,agv stop and pick plane...\r\n");
				}
				else if(out_black_line_state==1&&line_flag==1&&last_line_flag!=1){
					out_black_line_state = 2;
					{speed = 150;omg = 0;turn_flag = 0;}
					u1_printf("second line find...\r\n");
				}
				else if(out_black_line_state==2&&line_flag!=1&&last_line_flag==1){
					out_black_line_state = 3;
					{speed = 150;omg = 0;turn_flag = 0;}
					u1_printf("second line miss,agv still go out...\r\n");
				}
				else if(out_black_line_state==3&&line_flag==1&&last_line_flag!=1){
					out_black_line_state = 4;
					{speed = 150;omg = 0;turn_flag = 0;}
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
					{speed = -150;omg = 0;turn_flag = 0;}
					u1_printf("release plane line find...\r\n");
				}
				else if(in_black_line_state1==-2&&line_flag!=1&&last_line_flag==1){
					in_black_line_state1 = -1;
					{speed = -150;omg = 0;turn_flag = 0;}
					u1_printf("release plane line miss...\r\n");
				}
				else if(in_black_line_state1==-1&&line_flag==1&&last_line_flag!=1){
					in_black_line_state1 = 0;
					g_auto_pick_state = ONE;//收起夹具
					{speed = 0;omg = 0;turn_flag = 0;}
					u1_printf("first line find,agv stop and close pick...\r\n");
				}
				else if(in_black_line_state1==0&&line_flag!=1&&last_line_flag==1){
					in_black_line_state1 = 1;
					{speed = -150;omg = 0;turn_flag = 0;}
					u1_printf("first line miss...\r\n");
				}
				else if(in_black_line_state1==1&&line_flag==1&&last_line_flag!=1){
					in_black_line_state1 = 2;
					{speed = -150;omg = 0;turn_flag = 0;}
					u1_printf("second line find...\r\n");
				}
				else if(in_black_line_state1==2&&line_flag!=1&&last_line_flag==1){
					in_black_line_state1 = 3;
					{speed = -150;omg = 0;turn_flag = 0;}
					u1_printf("second line miss,agv still go back...\r\n");
				}
				else if(in_black_line_state1==3&&line_flag==1&&last_line_flag!=1){
					in_black_line_state1 = 4;
					{speed = -150;omg = 0;turn_flag = 0;}
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
					{speed = 150;omg = 0;turn_flag = 0;}
					u1_printf("charge line find...\r\n");
				}
				else if(out_black_line_state1==-2&&line_flag!=1&&last_line_flag==1){
					out_black_line_state1 = -1;
					{speed = 150;omg = 0;turn_flag = 0;}
					u1_printf("charge line miss...\r\n");
				}
				else if(out_black_line_state1==-1&&line_flag==1&&last_line_flag!=1){
					out_black_line_state1 = 0;
					{speed = 150;omg = 0;turn_flag = 0;}
					u1_printf("first line find...\r\n");
				}
				else if(out_black_line_state1==0&&line_flag!=1&&last_line_flag==1){
					out_black_line_state1 = 1;
					u1_printf("first line miss,agv is still go out...\r\n");
					{speed = 150;omg = 0;turn_flag = 0;}
				}
				else if(out_black_line_state1==1&&line_flag==1&&last_line_flag!=1){
					out_black_line_state1 = 2;
					u1_printf("second line find...\r\n");
					{speed = 150;omg = 0;turn_flag = 0;}
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
					{speed = 150;omg = 0;turn_flag = 0;}
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

const int32_t step = 16384;//14位编码器	204800
static int16_t delaycnt = 0;
float womg = 0;
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

