/* **************************************
2019-10-2: 包建飞  针对芦墟的方案
* 仅包含2个A车，arduino控制2个A车以恒定的速度向前前进
* 外设包括：双路电机驱动板1块，库伦计1块，4路继电器1块，遥控开关继电器1个
* 实现功能：
*     停/起功能：在主控板的控制下，执行清洗过程（固定速度前进 + 刷子旋转）或停止清洗（停止前进+刷子停止）
*     电量检测:  将电量上报给主控板，将是否充电状态上报给主控板
*     
****************************************/

//#define test_motor // 测试电机，去除的话，电机未收到消息不会转动

//#define test_seri_0 // 测试串口 0 打印 传感器消息 ros不能用 
// mega 2560  

//#define USE_PID  //如果还没有接码盘，需要注释掉这个

#if (ARDUINO >= 100)
 #include <Arduino.h>
#else
 #include <WProgram.h>
#endif

//#define USE_USBCON
#include <ros.h>
#include <std_msgs/UInt32.h>
#include <FlexiTimer2.h>// 系统任务定时器 
#include <TimerThree.h> //for test
#include <TimerOne.h>		// http://www.arduino.cc/playground/Code/Timer1
#include "pid.h"
#include<avr/wdt.h>  //watchdog

#define SERIAL_BUFFER_SIZE 128
#define SERIAL_RX_BUFFER_SIZE 512
#define SERIAL_TX_BUFFER_SIZE 512

int motor_driver_type = 1;  //0 旧电机驱动板  1 新电机驱动板
int enable_motor = false;

int PWM_STEP = 20;   //PWM的变化频度

//--------------------------------------- 硬件引脚定义----------------------------------------------------------
// ---- digital table -----
// 	0: 串口0	 1:串口0		2: 	 					3: 							4: 
// 	5:			 6:				7:	自测跳线 			8: 右前码盘							9: 左前轮方向
// 	10:	右前方向 11:右PWM		12:	左PWM				13:	LED						14:串口3
//	15:串口3	 16:串口2		17:串口2				18:串口1					19:串口1
//	20:			 21:			22:充电继电器
//	25:
//	30:			 31:			32:	保留				33:						34:		
//	35:	侧刷电机 36:			37:前刷电机				38:						39:
//	40:			 41:			42:						43:						44:
//	45:			 46:			47:						48:左轮使能
//	50:右轮使能  51:			52:						53:					
//
// ---- Analog table -----
//  A0:				A1:					A2:					A3:						A4:
//	A5:				A6: 				A7:					A8:						A9:
//	A10:		    A11:		
//  
//----  其他 -----
// 串口3：		    串口2: 电流计   		串口1: 



/* 前进电机引脚定义 */
#define PWM_left       12//7  // B-L
#define PWM_right      11//  8  // B-R
#define DIR_left        9 // 左轮方向 IO口  //old 10
#define DIR_right        10//13 // 右轮方向 IO口
#define EN_B_LEFT_PWM  		48   //左轮
#define EN_B_RIGHT_PWM  	50   //右轮



//普通电机口=====================================
#define motor_2    35//36    //后刷电机  （37）
#define motor_1    37// 38 //    //前刷电机  （35）

/* B车前进的编码器 */
#define B_LF_ENCODE 8 // 左车编码器
#define B_RF_ENCODE 8 // 右车编码器

/* LED管脚 */
const int ledPin =  LED_BUILTIN;//LED管脚

/* 自测跳线 */
#define SELFTEST_PIN  7

#define CHONGDIAN_NODE 22  //充电状态检测继电器节点

/* EEPROM 地址偏移量定义  */
int addr = 0;

int pause_response_firefly_counter = 0;
int prev_battery_state = 0;

//---------------------------- 全局变量定义---------------------------------------------------------------------
/////////  ROS和上位机通信相关控制全局变量
int Motor_contrl_msg_count = 0;// ros话题接收消息次数 计数
int Motor_contrl_msg_count_100ms = 0;// 100ms时的计数
bool master_firefly_online = 0;// 主机在线标志，在发送消息

/////// 车速度和方向控制的全局变量
float vel_l = 0;//左轮转速
float vel_r = 0;//右轮转速
int PWM_left_front_value = 0;
int PWM_right_front_value = 0;


//从上位机接收到的指令解析后的数据
float angle_speed = 0.0;//车角速度   bit6 - bit 0 （3.14弧度/s 等分128）
bool direction_angle_speed = 0;//  bit 7 0-- 逆时针  1  -- 顺时针转  
float liner_speed = 0.0;//车线速度  bit15 - bit8 0.2m/s 等分256
bool direction = 0;//车前进方向 bit26  1 向前    0 向后

// 发送消息的变量====================
//sensor
int battery_percent = 0;//0-100表示电池电量百分比  bit6-bit0
bool battery_state = 0;//bit 7  0 -- 放电中  1 -- 充电中
int	 cur_temperature = 20;  //当前温度


//编码器相关全局变量
int encode_lf_status = 0;
int encode_lf_counter = 0;
int encode_lf_counter_prev = 0;
int b_l_encoder_count_diff = 0;

int encode_rf_status = 0;
int encode_rf_counter = 0;
int encode_rf_counter_prev = 0;
int b_r_encoder_count_diff = 0;



//利用码盘对速度进行调整
float cur_liner_speed = 0.0;
float weight = 1.5;
int adjust_val_switch = 0;

//LED参数
bool led_on_off_flag=0;

// 串口2接收============== 
//电源模块,充电时，电源模块红线接充电器的负，黑线接电池的负
//unsigned char data[15];//保存传输过来的15个字节
unsigned char data;//保存传输过来的15个字节
unsigned char raw_data[15];
unsigned char checksum = 0;
unsigned char count=0;//计数
int eletric_percent;//
int serial2_data_state = 0;
int battery_state_from_node = 0;
int battery_state_charge_count = 0;

//Task任务控制全局变量==============================
int send_ros_count = 0;  //用于向上位机发送数据的周期控制计数器
int Motor_contrl_msg_time_count=0;  //用于判断上位机是否离线
int motor_count = 0;  //Task的周期计数变量

//  ROS参数定义=======================
ros::NodeHandle   nh;                               //创建ROS句柄
void Motor_contrl( const std_msgs::UInt32& cmd_msg);//函数声明
void Setup_contrl( const std_msgs::UInt32& cmd_msg);//函数声明
void Motor_contrl_local();


//定义编码器脉冲广播消息格式
std_msgs::UInt32  Sensor_msg; 
// 定义向上位机发送指令，Publisher-发送，Sensor-，"Sensor"-消息头，Sensor_msg-待发送的消息结构
ros::Publisher Sensor("Sensor", &Sensor_msg); 

std_msgs::UInt32  Sensor_msg_1;              
ros::Publisher Sensor_1("Sensor_1", &Sensor_msg_1);  


// 定义从上位机接收话题，Subscriber-接收，std_msgs-，"motor"-消息头，Motor_contrl-得到上位机指令后调用的函数
ros::Subscriber<std_msgs::UInt32> sub("motor", Motor_contrl);
ros::Subscriber<std_msgs::UInt32> sub_setup("setup", Setup_contrl);
bool send_ros_flag = false;

bool check_battery_flag = false;

//从上位机接收消息
bool motor_1_value = 0;//清洗面6的电机    bit28      1 旋转  0 不转
bool motor_2_value = 0;//清洗面5/7的电机    bit29      1 旋转  0 不转
std_msgs::UInt32 localctrl_msg;  //控制消息

int delay_start_motor_timer = 0;  //逐一启动电机

//PWM分频
int DIVISOR = 1;  //1

//温度
short tempture=0;
short current = 0;
short voltage = 0;

bool stay_on_charge_site = true;  //true表示当前停在充电桩处
float  go_distance = 0;   //离开充电桩处的前进距离

int selftest_mode = 0; //是否处于自测状态/

//---------------------------- 函数声明和定义 --------------------------------------------------------------
void send_ros();
void setPwmFrequency2560(int pin, int divisor);
void task();
void setPwmFrequency2560(int pin, int divisor);
void Motor_contrl( const std_msgs::UInt32& cmd_msg);
void read_encode_value();

void(* resetFunc) (void) = 0;

//读取光电编码器的计数
void read_encode_value()
{
	int value;

	//读取左轮读数
	value = digitalRead(B_LF_ENCODE);
	if (value==1 && encode_lf_status==0)
	{
		encode_lf_counter ++;
	}

	encode_lf_status = value;

/*	
	//读取右轮读数
	value = digitalRead(B_RF_ENCODE);
	if (value==1 && encode_rf_status==0)
	{
		encode_rf_counter ++;
	}
	encode_rf_status = value;
*/	
		
	
}

/////////////10ms 执行一次定时器中断服务程序=======================
void task(){
	wdt_reset();
	
	//LED闪烁5s	
	if (motor_count%500==0  && selftest_mode==0)
	{
		digitalWrite(LED_BUILTIN,led_on_off_flag);
		led_on_off_flag = !led_on_off_flag;
	}
	if ( selftest_mode==1)
	{
		if (!master_firefly_online)
		{
			if (motor_count%10==0)
			{
				digitalWrite(LED_BUILTIN,led_on_off_flag);
				led_on_off_flag = !led_on_off_flag;
			}
		}
		else if (voltage==0)
		{
			if (motor_count%100==0)
			{
				digitalWrite(LED_BUILTIN,led_on_off_flag);
				led_on_off_flag = !led_on_off_flag;
			}
			
		}
		
	}
	
	//每10ms读一次编码器的电平
	read_encode_value();
	
	//每100ms控制一下车子的前进（仅在调试时使用，通过修改localctrl_msg的值来测试）
	if (motor_count%100==0 && selftest_mode==1)
	{
		Motor_contrl_local();
	}


	if (pause_response_firefly_counter>0)
		pause_response_firefly_counter--;

	//每100ms对ros消息上传
	//每100ms对延迟启动计数器进行递减
	if (motor_count%10==0)  //10*10ms
	{
		send_ros_flag = true;  

		if (delay_start_motor_timer>0)
			delay_start_motor_timer --;
	
		
	}
	
	//每100ms查询一下是否处于充电状态
	if (motor_count%10==0)  //10*10ms
	{
		check_battery_flag = true;  
	}
	
	//每5s统计一下当前右轮的转速
	if (motor_count%500==0)  //5s更新一次，速度
	{
		adjust_val_switch = 1; //测速完成

		//左轮测速
		b_l_encoder_count_diff = encode_lf_counter - encode_lf_counter_prev;
		encode_lf_counter_prev = encode_lf_counter;
		if (fabs(b_l_encoder_count_diff)>100000) 		//越界的判断
			b_l_encoder_count_diff = 0;

		//Serial.println(encode_lf_counter);
		
		//右轮测速
/*		
		b_r_encoder_count_diff = encode_rf_counter - encode_rf_counter_prev;
		encode_rf_counter_prev = encode_rf_counter;
		if (fabs(b_r_encoder_count_diff)>100000) 		//越界的判断
			b_r_encoder_count_diff = 0;

		Serial.print("\t");
		Serial.println(encode_rf_counter);
*/		
	}


	//循环计时器累加 10ms*1000 = 10s
	motor_count ++;
	if (motor_count>=1000)
		motor_count = 0;
	
// 主机在线离线判断===============================
// 注：本地遥控模式下，不进行离线判断
#ifndef test_motor
	Motor_contrl_msg_time_count ++;  //+=1
	if(Motor_contrl_msg_time_count == 10)     
		Motor_contrl_msg_count_100ms = Motor_contrl_msg_count;// 100ms接收消息次数
	if(Motor_contrl_msg_time_count >= 100) { // 1000ms
		if( Motor_contrl_msg_count > Motor_contrl_msg_count_100ms) 
			master_firefly_online = 1;// 主机在线
		else 
			master_firefly_online = 0;// 主机离线
     
		Motor_contrl_msg_time_count=0;// 时间计数清零   500ms周期判断主机是否在线        
		Motor_contrl_msg_count = 0;   // 消息计数清零
	}
#endif
	
	
	if (motor_count%100==0)  //1s更新一次前进的距离
	{
		go_distance += liner_speed;
	}
	
	
}

/////////////////////////////PWM实现函数//////////////////////////////////////////
void setPwmFrequency2560(int pin, int divisor) {
  byte mode;
  if((pin >= 2 && pin <= 13) || (pin >= 44 && pin <= 46)) 
  {
    switch(divisor) {
      case 1: mode = 0x01; break;
      case 8: mode = 0x02; break;
      case 64: mode = 0x03; break;
      case 256: mode = 0x04; break;
      case 1024: mode = 0x05; break;
      default: return;
    }
      if(pin == 4 || pin == 13)//Timer0
      {
        TCCR0B = TCCR0B & 0b11111000 | mode;  
      }
      else if(pin == 11 || pin == 12 || pin == 13)//Timer1
      {
        TCCR1B = TCCR1B & 0b11111000 | mode;  
      }
      else if(pin == 8 || pin == 9)//Timer2
      {
        TCCR2B = TCCR2B & 0b11111000 | mode;  
      }
      else if(pin == 5 || pin == 2 || pin == 3)//Timer3
      {
        TCCR3B = TCCR3B & 0b11111000 | mode;  
      }
      else if(pin == 6 || pin == 7 || pin == 8)//Timer4
      {
        TCCR4B = TCCR4B & 0b11111000 | mode;    
      }
      else if(pin == 46 || pin == 45 || pin == 44)//Timer5
      {
        TCCR5B = TCCR5B & 0b11111000 | mode;    
      }
  } 
}

//本地测试
void Motor_contrl_local()
{
	digitalWrite(motor_1, LOW); //芦墟现在用的继电器是低电平有效
	digitalWrite(motor_2, LOW); //芦墟现在用的继电器是低电平有效

	if (motor_driver_type==0) {
		digitalWrite(DIR_left, HIGH); // LOW 后退   HIGH向前  
		digitalWrite(DIR_right, HIGH); // LOW 后退   HIGH向前  
	}
	else if (motor_driver_type==1)
	{
		digitalWrite(DIR_left, LOW); // INA  = LOW 前进   HIGH 后退  
		digitalWrite(EN_B_LEFT_PWM, HIGH); // INB  = HIGH 前进   LOW 后退   

		digitalWrite(DIR_right, LOW); // INA  = LOW 前进   HIGH 后退  
		digitalWrite(EN_B_RIGHT_PWM, HIGH); // INB  = HIGH 前进   LOW 后退   
	}
	analogWrite(PWM_left,150);          
	analogWrite(PWM_right, 150);
	
}

// 订阅话题的回调函数(得到上位机指令后，调用该函数进行解析)
void Motor_contrl( const std_msgs::UInt32& cmd_msg)
{  
	float feedback_vel_l = 0;
	float feedback_vel_r = 0;
	bool old_value;

	//如果此时处于临时不响应Firefly指令状态
	if (pause_response_firefly_counter>0)
		return;
	
	if (selftest_mode==1) {
		Motor_contrl_msg_count++; // 接收上位机消息次数++，用于判断 上位机是否在发送消息=====
		return;
	}

	direction_angle_speed	 = (cmd_msg.data & 0x00000080)>>7;
	liner_speed =       	   ((cmd_msg.data & 0x0000ff00)>>8)*0.2/256;
	direction =             (cmd_msg.data & 0x04000000)>>26;
	angle_speed =         0;  //  (cmd_msg.data & 0x0000007f)*3.14/128; --- 只支持车子向前走
 
	//速度解析==========================================	
	if(direction_angle_speed)
	{
		vel_l = liner_speed;//左轮转速(如果发现左右轮转速有不一致，在这里乘以系数)
		vel_r = liner_speed;//右轮转速(如果发现左右轮转速有不一致，在这里乘以系数)
	}
	else
	{
		vel_l = liner_speed;//左轮转速
		vel_r = liner_speed;//右轮转速
	}

	//如果测速已经完成，则根据码盘读数对vel_l进行调整 (每5秒调整一次）
	if (adjust_val_switch==1 && vel_l!=0)
	{
		//通过左侧码盘对车子的整体速度进行调整
		adjust_val_switch = 0;  //等下一次5秒之后再进行速度权重调整
		feedback_vel_l = b_l_encoder_count_diff*0.004361;  
		if ((feedback_vel_l-vel_l) >0.01) //实际速度偏快
			weight -=  0.01/vel_l;//减小速度
		else if ((feedback_vel_l-vel_l) <-0.01) //实际速度偏慢
			weight += 0.03/vel_l; //增加速度
		//Serial.print(feedback_vel_l);
		//Serial.print('\t');
		//Serial.print(vel_r);
		//Serial.print('\t');
		//Serial.println(weight);
	}

/*	
	//如果测速已经完成，则根据码盘读数对vel_r进行调整 (每5秒调整一次）
	if (adjust_val_switch==1 && vel_r!=0)
	{
		//通过右侧码盘对车子的整体速度进行调整
		adjust_val_switch = 0;  //等下一次5秒之后再进行速度权重调整
		feedback_vel_r = b_r_encoder_count_diff*0.004361;  
		if ((feedback_vel_r-vel_r) >0.01) //实际速度偏快
			weight -=  0.01/vel_r;//减小速度
		else if ((feedback_vel_r-vel_r) <-0.01) //实际速度偏慢
			weight += 0.01/vel_r; //增加速度
		//Serial.print(feedback_vel_r);
		//Serial.print('\t');
		//Serial.print(vel_r);
		//Serial.print('\t');
		//Serial.println(weight);
	}
*/
		

	//这里需要单独控制 ---- 2019-10-01
	float val =  abs(vel_l) * weight * 1333;
	PWM_left_front_value = (int)val;
	val  = abs(vel_r) * weight * 1333*0.88;
	PWM_right_front_value = (int)val;

	if (PWM_left_front_value>220) 
		PWM_left_front_value = 220;
	else if (PWM_left_front_value==0)
		PWM_left_front_value = 0;
	else if (PWM_left_front_value<35)
		PWM_left_front_value = 35;
		
	
		
	if (PWM_right_front_value>220) 
		PWM_right_front_value = 220;
	else if (PWM_right_front_value==0)
		PWM_right_front_value = 0;
	else if (PWM_right_front_value<31) //31 --- 0.89    28 --- 0.8
		PWM_right_front_value = 31;
	
 
	//普通电机口===================================== 
	//延时启动
	old_value = motor_1_value;
	motor_1_value = (cmd_msg.data & 0x40000000)>>30;
	if (motor_1_value && !old_value) {
		if (delay_start_motor_timer==0) {
			delay_start_motor_timer = 10;
		}
		else 
		{
			motor_1_value = old_value;
		}
	}	
	old_value = motor_2_value;
	motor_2_value = (cmd_msg.data & 0x80000000)>>31;
	if (motor_2_value && !old_value) {
		if (delay_start_motor_timer==0) {
			delay_start_motor_timer = 10;
		}
		else 
		{
			motor_2_value = old_value;
		}
	}
	
	if(motor_1_value) 
		digitalWrite(motor_1, LOW); //芦墟现在用的继电器是低电平有效
	else
		digitalWrite(motor_1, HIGH);

	if(motor_2_value) 
		digitalWrite(motor_2, LOW);
	else 
		digitalWrite(motor_2, HIGH);


	//B车电机方向口======================================
	if(direction)
	{
		if (vel_l >= 0) 
		{
			if (motor_driver_type==0) {
				digitalWrite(DIR_left, HIGH); // LOW 后退   HIGH向前  
			}
			else if (motor_driver_type==1 && enable_motor==true)
			{
				digitalWrite(DIR_left, LOW); // INA  = LOW 前进   HIGH 后退  
				digitalWrite(EN_B_LEFT_PWM, HIGH); // INB  = HIGH 前进   LOW 后退  
			}
		}
		else
		{
			if (motor_driver_type==0) {
				digitalWrite(DIR_left, LOW); // LOW 后退   HIGH向前  
			}
			else if (motor_driver_type==1 && enable_motor==true)
			{
				digitalWrite(DIR_left, HIGH); // INA  = LOW 前进   HIGH 后退  
				digitalWrite(EN_B_LEFT_PWM, LOW); // INB  = HIGH 前进   LOW 后退   
			}
		}
		if (vel_r >= 0) 
		{
			if (motor_driver_type==0) {
				digitalWrite(DIR_right, HIGH); // LOW 后退   HIGH向前  
			}
			else if (motor_driver_type==1 && enable_motor==true)
			{
				digitalWrite(DIR_right, LOW); // INA  = LOW 前进   HIGH 后退  
				digitalWrite(EN_B_RIGHT_PWM, HIGH); // INB  = HIGH 前进   LOW 后退  
			}
		}
		else
		{
			if (motor_driver_type==0) {
				digitalWrite(DIR_right, LOW); // LOW 后退   HIGH向前  
			}
			else if (motor_driver_type==1 && enable_motor==true)
			{
				digitalWrite(DIR_right, HIGH); // INA  = LOW 前进   HIGH 后退  
				digitalWrite(EN_B_RIGHT_PWM, LOW); // INB  = HIGH 前进   LOW 后退   
			}
		}
	}
	else
	{
		if (vel_l >=0) 
		{
			if (motor_driver_type==0) {
				digitalWrite(DIR_left, LOW); // LOW 后退   HIGH向前  
			}
			else if (motor_driver_type==1 && enable_motor==true)
			{
				digitalWrite(DIR_left, HIGH); // INA  = LOW 前进   HIGH 后退  
				digitalWrite(EN_B_LEFT_PWM, LOW); // INB  = HIGH 前进   LOW 后退   
			}
		}
		else
		{
			if (motor_driver_type==0 ) {
				digitalWrite(DIR_left, HIGH); // LOW 后退   HIGH向前  
			}
			else if (motor_driver_type==1 && enable_motor==true)
			{
				digitalWrite(DIR_left, LOW); // INA  = LOW 前进   HIGH 后退  
				digitalWrite(EN_B_LEFT_PWM, HIGH); // INB  = HIGH 前进   LOW 后退  
			}
		}
		
		if (vel_r >= 0) 
		{
			if (motor_driver_type==0) {
				digitalWrite(DIR_right, LOW); // LOW 后退   HIGH向前  
			}
			else if (motor_driver_type==1 && enable_motor==true)
			{
				digitalWrite(DIR_right, HIGH); // INA  = LOW 前进   HIGH 后退  
				digitalWrite(EN_B_RIGHT_PWM, LOW); // INB  = HIGH 前进   LOW 后退   
			}
		}
		else {
			if (motor_driver_type==0) {
				digitalWrite(DIR_right, HIGH); // LOW 后退   HIGH向前  
			}
			else if (motor_driver_type==1 && enable_motor==true)
			{
				digitalWrite(DIR_right, LOW); // INA  = LOW 前进   HIGH 后退  
				digitalWrite(EN_B_RIGHT_PWM, HIGH); // INB  = HIGH 前进   LOW 后退  
			}
		}
	}

	
	
	//等刷子启动之后再启动行走电机
	if (delay_start_motor_timer==0) {
		if (motor_driver_type==0)
		{
			//B车电机PWM口 ===========================
			digitalWrite(EN_B_LEFT_PWM, LOW);
			digitalWrite(EN_B_RIGHT_PWM, LOW);
		}
		else if (motor_driver_type==1)
		{
			enable_motor = true;
		}
		
		analogWrite(PWM_left,PWM_left_front_value);          
		analogWrite(PWM_right, PWM_right_front_value);
//		analogWrite(PWM_left,150);          
//		analogWrite(PWM_right, 150);
	}
 
	Motor_contrl_msg_count++; // 接收上位机消息次数++，用于判断 上位机是否在发送消息=====
}


// 订阅话题的回调函数(得到上位机指令后，调用该函数进行解析)
void Setup_contrl( const std_msgs::UInt32& cmd_msg)
{  
	int cmd = (cmd_msg.data & 0x000000ff);
	if (cmd==1) //表示是要求Arduino重启的命令
	{
		resetFunc(); //重启Arduino
	}
	
	
}


//发送消息的函数,
void send_ros(){
	//温度正负
	int temp_status = (tempture&0x8000)!=0 ?1:0;  //1表示是负温度
	int report_temp = 0;
	int report_voltage = 0;
	int report_current = 0;
	int battery_state_to_web = 0;
	
	report_temp = tempture & 0x0FFF;
	report_temp = report_temp / 10;
	if (report_temp>255)
		report_temp = 255;
	
	report_current = current/10;  //电流换算成0.1A基本单位
	if (report_current>255)
		report_current = 255;
	
	report_voltage = voltage/10 - 200;  //电压换算为0.1V基本单位 （然后更换量程 20V~30V）
	if (report_voltage>255)
		report_voltage = 255;
	
	battery_percent = eletric_percent;
	
	if (battery_state==1 || (battery_state_from_node==1 && battery_state_charge_count>=10) )
		battery_state_to_web = 1;
	
	
	Sensor_msg.data= (long(fabs(report_voltage)           ) & 0xffffffff)<<24
						 |(long(temp_status				        ) & 0xffffffff)<<23 
                         |(long(report_temp)            & 0xffffffff)<<16 
                         |(long(fabs(report_current)           ) & 0xffffffff)<<8 
                         |(long( battery_state_to_web           ) & 0xffffffff)<<7
                         |(long( battery_percent         ) & 0xffffffff);    
                       //  |(long( pwm_set          ) & 0xffffffff);    

	Sensor.publish(&Sensor_msg);

	Sensor_msg_1.data=  battery_state_from_node<<16 
		|((unsigned short)(go_distance*10        ) & 0xffffffff);                  
	Sensor_1.publish(&Sensor_msg_1);
	
}

void check_battery()
{
	int value = digitalRead(CHONGDIAN_NODE);
	
	if (value) {
		battery_state_from_node = 0;  //未充电
		battery_state_charge_count = 0;
	}
	else {
		battery_state_from_node = 1; //充电中
		battery_state_charge_count++;
	}		
	//Serial.println(	battery_state_from_node);
/*	
	Serial2.write(0x5A);
//	delay(1);
	Serial2.write(0xA5);
//	delay(1);
	Serial2.write(0x10);
//	delay(1);
	Serial2.write(0x00);
//	delay(1);
	Serial2.write(0x00);
//	delay(1);
	Serial2.write(0x00);
//	delay(1);
	Serial2.write(0x00);
//	delay(1);
	Serial2.write(0x0F);
*/
/*
	unsigned char buf[8];
	buf[0] = 0x5a;
	buf[1] = 0xa5;
	buf[2] = 0x09;
	buf[3] = 0x00;
	buf[4] = 0x00;
	buf[5] = 0x00;
	buf[6] = 0x00;
	buf[7] = 0x08;
	int len = Serial2.write(buf,8);
	Serial.println(len);
*/	
/*	
	Serial2.write(0x5A);
//	delay(1);
	Serial2.write(0xA5);
//	delay(1);
	Serial2.write(0x09);
//	delay(1);
	Serial2.write(0x00);
//	delay(1);
	Serial2.write(0x00);
//	delay(1);
	Serial2.write(0x00);
//	delay(1);
	Serial2.write(0x00);
//	delay(1);
	Serial2.write(0x08);
*/ 
}

void setup()
{
	pinMode(LED_BUILTIN, OUTPUT);//系统运行指示灯
	Serial2.begin(9600);//串口初始化 库仑计信号
	// delay(5000);

	// 编码器口初始化================================
	pinMode(B_LF_ENCODE, INPUT);
	pinMode(B_RF_ENCODE, INPUT);
	
	pinMode(SELFTEST_PIN,INPUT_PULLUP);
	
	//充电状态继电器
	pinMode(CHONGDIAN_NODE,INPUT_PULLUP);

	// 普通电机口初始化================================
	pinMode(motor_1, OUTPUT);
	digitalWrite(motor_1, HIGH);  //注：芦墟现在用的继电器是低电平有效

	pinMode(motor_2, OUTPUT);
	digitalWrite(motor_2, HIGH);

	if (motor_driver_type==0)
	{
		//车的2个行走电机使能信号初始化（关断）
		pinMode(EN_B_LEFT_PWM, OUTPUT);
		digitalWrite(EN_B_LEFT_PWM, HIGH);
		pinMode(EN_B_RIGHT_PWM, OUTPUT);
		digitalWrite(EN_B_RIGHT_PWM, HIGH);
	
		// B车初始化================================
		pinMode(DIR_left, OUTPUT);     // 左轮方向口   10   (PWM 7)
		digitalWrite(DIR_left, LOW);   // LOW 后退   HIGH向前  
		pinMode(DIR_right, OUTPUT);    // 右轮方向口   13
		digitalWrite(DIR_right,LOW ); // LOW 后退   HIGH向前  
	}
	else if (motor_driver_type==1)
	{
		pinMode(DIR_left, OUTPUT);     // 左轮方向口   10   (PWM 7)
		digitalWrite(DIR_left, HIGH);   // INA INB = HIGH HIGH 滑行 
		pinMode(EN_B_LEFT_PWM, OUTPUT);
		digitalWrite(EN_B_LEFT_PWM, HIGH);

		pinMode(DIR_right, OUTPUT);    // 右轮方向口   13
		digitalWrite(DIR_right,HIGH ); // // INA INB = HIGH HIGH 滑行 
		pinMode(EN_B_RIGHT_PWM, OUTPUT);
		digitalWrite(EN_B_RIGHT_PWM, HIGH);
	
		
	}
	
    setPwmFrequency2560(PWM_left,DIVISOR);
    pinMode(PWM_left, OUTPUT);     // 左轮方向口——7
    analogWrite(PWM_left,0);                  //左电机控制占孔比初始化 
    setPwmFrequency2560(PWM_right,DIVISOR);
    pinMode(PWM_right, OUTPUT);     // 左轮方向口——8
    analogWrite(PWM_right,0);                 //左电机控制占孔比初始化

#ifndef test_motor	
	nh.initNode(); //ROS节点初始化   Serial0  9600     
	// delay(3000); 
	//消息初始化===============================
	nh.advertise(Sensor);    //消息发布初始化
	nh.advertise(Sensor_1);    //消息发布初始化
	
	nh.subscribe(sub);        //消息订阅初始化,会影响串口
	nh.subscribe(sub_setup);        //消息订阅初始化,会影响串口
#else
	Serial.begin(9600);
#endif


//test new motor driver
/*	digitalWrite(DIR_left, LOW);
	digitalWrite(EN_B_LEFT_PWM, HIGH);
	analogWrite(PWM_left,40);
	
	digitalWrite(DIR_right, LOW);
	digitalWrite(EN_B_RIGHT_PWM, HIGH);
	analogWrite(PWM_right,40);
*/
	
	//for test
//	localctrl_msg.data = 0x04008000;
//	digitalWrite(EN_B_RIGHT_PWM, LOW);
//	analogWrite(PWM_right,150);
//	digitalWrite(EN_B_LEFT_PWM, LOW);
//	analogWrite(PWM_left,150);
	//localctrl_mode = 1;
	
//	digitalWrite(motor_1, LOW); //芦墟现在用的继电器是低电平有效
//	digitalWrite(motor_2, LOW); //芦墟现在用的继电器是低电平有效
//	delay(1000000);
	
	
	Timer3.initialize(10000);  //10ms
	Timer3.attachInterrupt(task);
	Timer3.start();
	

	wdt_enable(WDTO_250MS); //120
}

//// 主循环 //////
void loop()
{
	//判断是否处于自测状态
	int val = digitalRead(SELFTEST_PIN);
	if (val==0)
	{
		selftest_mode = 1;	
		localctrl_msg.data = 0xC4008000;
			
	}
/*	else {
		selftest_mode = 0;
		localctrl_msg.data = 0x00000000;
	}
*/	
	
#ifndef test_motor	
	nh.spinOnce();

	if (send_ros_flag==true)
	{
		send_ros_flag = false;
		send_ros();           // ros消息上传
	}
#endif	

#ifndef test_motor  
	if(!master_firefly_online  	// 主机离线 关机所有电机==============
		&& selftest_mode==0
	){
		// 可能还需要一个标志，确保master_firefly_online在变换时，只执行一次
		analogWrite(PWM_left,0);
		analogWrite(PWM_right,0);
	
		if (motor_driver_type==0 )
		{
			//B车使能信号拉高（关断）
			digitalWrite(EN_B_LEFT_PWM, HIGH);
			digitalWrite(EN_B_RIGHT_PWM, HIGH);
		}
		else if (motor_driver_type==1)
		{
			enable_motor = false;
		}
		
		// 关闭 继电器电机=========
		digitalWrite(motor_1, HIGH); //芦墟现在用的继电器是低电平有效
		digitalWrite(motor_2, HIGH);
	}
#endif

	if (check_battery_flag==true)
	{
		check_battery_flag = false;
		check_battery();
	}
	
	if ( (battery_state==1 || battery_state_from_node==1) &&  prev_battery_state==0 && go_distance>=1.0 )  //如果当前在充电中而之前不在充电中，并且车子已经前进超过1米 将机器先停下来10秒种
	{
		if (motor_driver_type==0)
		{
			//B车使能信号拉高（关断）
			digitalWrite(EN_B_LEFT_PWM, HIGH);
			digitalWrite(EN_B_RIGHT_PWM, HIGH);
		}
		else if (motor_driver_type==1)
		{
			enable_motor = false;
		}
		
		digitalWrite(motor_1, HIGH);
		digitalWrite(motor_2, HIGH);
		pause_response_firefly_counter = 1000;	 //临时不响应Firefly的控制指令的计数器
		
		go_distance = 0.0;  //车子已走到充电桩处
	}
	prev_battery_state = battery_state;
//	prev_battery_state = prev_battery_state;

	//串口连接的是电流计
	while (Serial2.available()) {
//		Serial.println(Serial2.read());
		
		data = (unsigned char)Serial2.read();//读取一个新字节
		
		if(serial2_data_state == 0 && data==0x5A) { //读取首标志0x5A状态
			serial2_data_state = 1; 
			checksum = data;
			continue; //return;//如果读取的第一个字节是0x5A，状态修改为读取A5
		}

		if (serial2_data_state == 1 ) { //读取第二标志0xA5状态
			if (data==0xA5) {
				serial2_data_state = 2;
				checksum += data;
				count = 0;
			}
			else
				serial2_data_state = 0; 

			continue;	
		}		
		
		if(serial2_data_state == 2) {  //寻找checksum阶段
			raw_data[count] = data;
			count++;
			
			if (count>13)
			{
				serial2_data_state = 0;
				continue;
			}
			
			if (checksum == data)  {
				serial2_data_state = 3;  //找到疑似checksum阶段
				continue;
			}
			else {
				checksum += data;
				continue;
			}
		}
		
		if (serial2_data_state==3) {  //找到疑似checksum阶段
			if (data==0x5A) { //表示完整收到了数据
				//处理收到的数据
				if (count==13) {  //我需要处理的数据
					eletric_percent = raw_data[1];
					tempture = (raw_data[8]<<8)|raw_data[9];
					voltage = (raw_data[2] <<8)|raw_data[3];
					current = (raw_data[4] <<8)|raw_data[5];
					battery_state = raw_data[10];
//					Serial.println(eletric_percent);
//					Serial.println(battery_state);
					//Serial.print(raw_data[2] );
					//Serial.print('\t');
					//Serial.println(current );
					
				}
				serial2_data_state = 1; 
				checksum = data;
				continue; //return;//如果读取的第一个字节是0x5A，状态修改为读取A5
			}
			else //如果后面不是0x5A，表示这个数据不是checksum，恢复到寻找checksum状态
			{
				raw_data[count] = data;
				count++;
				serial2_data_state = 2;
				continue;
			}
		}
		delay(1);
		
/*		
		data[count] = (unsigned char)Serial2.read();//读取一个新字节
		if(count==0 && data[0]!=0x5A) 
			continue; //return;//如果读取的第一个字节不是0x5A，跳出去
		count++;
		if(count == 14){//当读满13个字节
			count = 0;
			if(data[2] == 0x10)//当第三个字节为0x10，读取接收到的消息
			{
				eletric_percent = data[3];
				battery_percent = eletric_percent;
				battery_state = data[12];
				tempture = ( (data[10] | 0x80) <<8) | data[11];
				
				
				
			}
		}
*/		
	}
	
}

