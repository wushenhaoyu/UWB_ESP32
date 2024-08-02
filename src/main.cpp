#include <Arduino.h>
/*! ----------------------------------------------------------------------------
 * @app PG定位系统 
 * @author 广州联网科技有限公司
 * @web www.gzlwkj.com
 */

/*
   2023/04/13 v1.0
   使用Arduino串口接收LinkPGV5.8 MODBUS协议数据并解析打印
   Arduino   LinkPG
      TX --->  RX
      RX --->  TX
      5V --->  5V
      GND--->  GND
   Arduino串口发送PGSTA 开始定位
              发送PGSTO 取消定位
*/
#include <HardwareSerial.h>


#define SERVICE_UUID        "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define CHARACTERISTIC_UUID "beb5483e-36e1-4688-b7f5-ea07361b26a8"
#ifndef uS_TO_S_FACTOR
#define uS_TO_S_FACTOR 1000000
#endif
#define SERIAL2_RX_PIN 4
#define SERIAL2_TX_PIN 5
#define SERIAL1_RX_PIN 16
#define SERIAL1_TX_PIN 17
#define BEEP_PIN 14 //蜂鸣器
#define LASER_PIN 13

#define IDLE 0
#define CHECK 1
#define LEN 2
#define WORK 3
#define END 4

#define Serial2_RX_MAX   128   //串口数据接收缓存长度
// HC_SR04_BASE - https://github.com/bjoernboeckle/HC_SR04
// Copyright © 2022, Björn Böckle
// MIT License





// Initialize HC_SR04 sensor
byte variable[Serial2_RX_MAX]; //串口接收缓存区
u8_t state = IDLE;
u8_t RX_buffer[256];
u8_t len_ = 0;
u8_t func = 0;
u8_t len_reg = 0;
unsigned long laserOnTime = 0;
bool isLaserOn = false;
unsigned long beepOnTime = 0;
bool isBeepOn = false;

int recvIndex = 0;
int ReceieveByte_length;



byte start_modbus[11] = {0x01,0x10,0x00,0x3B,0x00,0x01,0x02,0x00,0x04,0xA3,0x18};//开始定位
byte stop_modbus[11] = {0x01,0x10,0x00,0x3B,0x00,0x01,0x02,0x00,0x00,0xA2,0xDB};//取消定位

#define ANCHOR_LIST_COUNT 16
#define ANCHOR_MAX_COUNT 16

uint16_t Analyze_TagID = 0x00; //监听标签的ID号
void Rtls_DataRecv(byte *buff);
void Tag_RtlsDataRecv(byte *buff);
void Start_Location();
void Stop_Location();
void Get_Location();
void Deal_With_Data();
void Deal_With_Recieve_Data(u8_t func,u8_t len,u8_t *data);
void beepOn(unsigned long duration);
// 关闭蜂鸣器的函数
void beepOff();
void initHC_SR04Sensors();
// Perform sensor measurements and print results
void measureAndPrint();
// Helper to print measured data for all sensors
void printSensorMeasurements();
void Start_Laser();
void bee_3();
typedef struct{
  uint16_t Tag_ID;  //标签ID
 	int16_t x;         //计算出的x坐标 单位cm
	int16_t y;         //计算出的y坐标 单位cm
	int16_t z;         //计算出的z坐标 单位cm
	uint32_t Cal_Flag;  //测距成功标志位 第8位 1：定位成功 第0-7：1分别代表A-H基站测距成功
  uint16_t Dist[ANCHOR_LIST_COUNT];     //测得标签与A-H基站的距离
  uint32_t Time_ts[6];      //时间戳
}Cal_data_t;

typedef struct {
  uint16_t Max_noise;
  uint16_t Std_noise;
  uint16_t Fp_amp1;
  uint16_t Fp_amp2;
  uint16_t Fp_amp3;
  uint16_t Max_growthCIR;
  uint16_t Rx_preambleCount;
  uint16_t Fp;
  double Fp_power;
  double Rx_power;
}Rx_diag_t;

Cal_data_t Last_cal_data_hds;  //基站位置列表
Rx_diag_t Rx_diag;    //基站接收信息列表

const int ANC_PROTOCAL_RTLS = 0;      //基站定位数据标志位
const int ANC_PROTOCAL_DIST = 1;      //基站测距数据标志位
const int ANC_PROTOCAL_RXDIAG = 2;    //基站接收信息标志位
const int ANC_PROTOCAL_TIMESTAMP = 3; //基站时间戳信息标志位

#define TAG_OUTPUT_DIST   0
#define TAG_OUTPUT_RTLS   1




const byte auchCRCHi[] = 
{ 	 
	0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
	0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 
	0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
	0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 
	0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 
	0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 
	0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 
	0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 
	0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 
	0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 
	0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 
	0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 
	0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 
	0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 
	0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 
	0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40 
} ; 

const byte auchCRCLo[] = 
{ 
	0x00, 0xC0, 0xC1, 0x01, 0xC3, 0x03, 0x02, 0xC2, 0xC6, 0x06, 0x07, 0xC7, 0x05, 0xC5, 0xC4, 0x04, 0xCC,
	0x0C, 0x0D, 0xCD, 0x0F, 0xCF, 0xCE, 0x0E, 0x0A, 0xCA, 0xCB, 0x0B, 0xC9, 0x09, 0x08, 0xC8, 0xD8, 
	0x18, 0x19, 0xD9, 0x1B, 0xDB, 0xDA, 0x1A, 0x1E, 0xDE, 0xDF, 0x1F, 0xDD, 0x1D, 0x1C, 0xDC, 0x14, 
	0xD4, 0xD5, 0x15, 0xD7, 0x17, 0x16, 0xD6, 0xD2, 0x12, 0x13, 0xD3, 0x11, 0xD1, 0xD0, 0x10, 0xF0, 
	0x30, 0x31, 0xF1, 0x33, 0xF3, 0xF2, 0x32, 0x36, 0xF6, 0xF7, 0x37, 0xF5, 0x35, 0x34, 0xF4, 0x3C, 
	0xFC, 0xFD, 0x3D, 0xFF, 0x3F, 0x3E, 0xFE, 0xFA, 0x3A, 0x3B, 0xFB, 0x39, 0xF9, 0xF8, 0x38, 0x28, 
	0xE8, 0xE9, 0x29, 0xEB, 0x2B, 0x2A, 0xEA, 0xEE, 0x2E, 0x2F, 0xEF, 0x2D, 0xED, 0xEC, 0x2C, 0xE4, 
	0x24, 0x25, 0xE5, 0x27, 0xE7, 0xE6, 0x26, 0x22, 0xE2, 0xE3, 0x23, 0xE1, 0x21, 0x20, 0xE0, 0xA0, 
	0x60, 0x61, 0xA1, 0x63, 0xA3, 0xA2, 0x62, 0x66, 0xA6, 0xA7, 0x67, 0xA5, 0x65, 0x64, 0xA4, 0x6C, 
	0xAC, 0xAD, 0x6D, 0xAF, 0x6F, 0x6E, 0xAE, 0xAA, 0x6A, 0x6B, 0xAB, 0x69, 0xA9, 0xA8, 0x68, 0x78, 
	0xB8, 0xB9, 0x79, 0xBB, 0x7B, 0x7A, 0xBA, 0xBE, 0x7E, 0x7F, 0xBF, 0x7D, 0xBD, 0xBC, 0x7C, 0xB4, 
	0x74, 0x75, 0xB5, 0x77, 0xB7, 0xB6, 0x76, 0x72, 0xB2, 0xB3, 0x73, 0xB1, 0x71, 0x70, 0xB0, 0x50, 
	0x90, 0x91, 0x51, 0x93, 0x53, 0x52, 0x92, 0x96, 0x56, 0x57, 0x97, 0x55, 0x95, 0x94, 0x54, 0x9C, 
	0x5C, 0x5D, 0x9D, 0x5F, 0x9F, 0x9E, 0x5E, 0x5A, 0x9A, 0x9B, 0x5B, 0x99, 0x59, 0x58, 0x98, 0x88, 
	0x48, 0x49, 0x89, 0x4B, 0x8B, 0x8A, 0x4A, 0x4E, 0x8E, 0x8F, 0x4F, 0x8D, 0x4D, 0x4C, 0x8C, 0x44, 
	0x84, 0x85, 0x45, 0x87, 0x47, 0x46, 0x86, 0x82, 0x42, 0x43, 0x83, 0x41, 0x81, 0x80, 0x40
} ;	 

// CRC校验
// <param name="pdata">要处理的数据
// <param name="num">数据长度
// <returns>CRC校验码
uint16_t CRC_Calculate(byte *pdata,int num)
{
  byte uchCRCHi = 0xFF ;               
  byte  uchCRCLo = 0xFF ;               
	int urecvIndex ;                
	while(num --)                    
	{
		urecvIndex = uchCRCHi^*pdata++ ;           
		uchCRCHi = uchCRCLo^auchCRCHi[urecvIndex];
		uchCRCLo = auchCRCLo[urecvIndex];
	}
	return (uint16_t)(uchCRCHi << 8 | uchCRCLo) ;
}


void setup() 
{
  Serial.begin(9600);
  Serial1.begin(115200, SERIAL_8N1, SERIAL1_RX_PIN, SERIAL1_TX_PIN);

  // 配置Serial2使用自定义引脚
  Serial2.begin(115200, SERIAL_8N1, SERIAL2_RX_PIN, SERIAL2_TX_PIN);
  pinMode(BEEP_PIN, OUTPUT);
  pinMode(LASER_PIN, OUTPUT);
  digitalWrite(LASER_PIN, LOW);
  digitalWrite(BEEP_PIN, HIGH);

  Start_Location();
  bee_3();
}

void loop() 
{
    Get_Location();
    Deal_With_Data();
    if(isBeepOn && (millis() - beepOnTime >= 500)){
          digitalWrite(BEEP_PIN, HIGH); // 关闭蜂鸣器
           digitalWrite(LASER_PIN,LOW);
          isBeepOn = false;
    }
}
void Deal_With_Data() {
  while (Serial1.available()) { // 检查Serial1是否有数据可读
    int incomingByte = Serial1.read(); // 读取一个字节的数据
    Serial.print(incomingByte);
    if (incomingByte != -1) {
        switch (state)
    {

    case IDLE:
        if(incomingByte == 0xAA)
            state = CHECK;
        break;
    case CHECK:
            func = incomingByte;
            state = LEN;
        break;
    case LEN:
        len_= incomingByte;
        len_reg = 0;
        state = WORK;
        break;
    case WORK:
        if(len_reg < len_)
        {
            RX_buffer[len_reg] = incomingByte;
            len_reg++;
        }
        if(len_reg == len_)
        {
            state = END;
        }
        break;
    case END:
        if(incomingByte == 0xAF)
        {
            Deal_With_Recieve_Data(func,len_,RX_buffer);
        }
        len_ = 0;
        state = IDLE;
        len_reg = 0;
        func = 0;
        break;

    }
    }
  }
}
void Deal_With_Recieve_Data(u8_t func,u8_t len,u8_t *data)
{
  switch (func)
  {
    case 0x01://蜂鸣器 bee bee bee
          bee_3();
      break;
    case 0x02: //控制激光
          bee_3();
      break;
    case 0x03://定位控制
      if(data[0])
      {
        Start_Location();
      }
      else
      {
        Stop_Location();
      }
  }
}
void bee_3()
{

        if (!isBeepOn) {
        digitalWrite(LASER_PIN,HIGH);
        digitalWrite(BEEP_PIN, LOW); // 开启激光
        beepOnTime = millis(); // 记录激光开启的时间
        isBeepOn = true;
      }
}

void Start_Laser()
{

        if (!isLaserOn) {
        digitalWrite(BEEP_PIN, HIGH); // 开启激光
        laserOnTime = millis(); // 记录激光开启的时间
        isLaserOn = true;
      }
}



void Start_Location()
{
        Serial2.write(start_modbus,11);
        recvIndex = 0;
}

void Stop_Location()
{
        Serial2.write(stop_modbus,11);
        recvIndex = 0;
}
void Get_Location()//获取uwb信息并发送通过serial1给飞控
{
  while(Serial2.available() > 0)
  {
    variable[recvIndex] = Serial2.read();
    recvIndex++;
    if(recvIndex > 4)
    {
      if(variable[0] == 'P' && variable[1] == 'G' && variable[2] == 'S' && variable[3] == 'T' && variable[4] == 'A')//发送定位命令
      {
        Serial2.write(start_modbus,11);
        recvIndex = 0;
      }
      else if(variable[0] == 'P' && variable[1] == 'G' && variable[2] == 'S' && variable[3] == 'T' && variable[4] == 'O')//停止定位命令
      {
        Serial2.write(stop_modbus,11);
        recvIndex = 0;
      }
      else if(variable[0] == 0x01 && variable[1] == 0x03)  //modbusID号
      {
        int len = variable[2];
        ReceieveByte_length = len + 5;
        if(ReceieveByte_length > recvIndex)  //还没接收完全
          break;
        uint16_t crc = CRC_Calculate(variable,ReceieveByte_length-2);
        if (crc == (uint16_t)((uint16_t)(variable[ReceieveByte_length - 2] << 8) | (uint16_t)variable[ReceieveByte_length - 1]))
        {
          Serial.print(crc,HEX);
          Serial.print("-");
          Serial.print((uint16_t)((uint16_t)(variable[ReceieveByte_length - 2] << 8) | (uint16_t)variable[ReceieveByte_length - 1]),HEX);
          Serial.println("  CRC校验成功");

          if(variable[3] == 0xCA && variable[4] == 0xDA)
            Rtls_DataRecv(variable);  //主基站数据解算
          if(variable[3] == 0xAC && variable[4] == 0xDA)
            Tag_RtlsDataRecv(variable); //标签数据解算
        }
        recvIndex = 0;
      }
      else
        recvIndex = 0;
    }
  }
}

// 检查字对应位是否为1
// <param name="data">要检查的字节
// <param name="b">第几位
// <returns>true则为1 否则为false
bool Check_BitIsTrue(uint32_t data, int b)
{
  return ((data >> b) & 0x01) == 0x01;
}

// 接收基站定位信息处理
// <param name="buff">要处理的数据
// <returns>无
void Rtls_DataRecv(byte *buff)
{
  int Number = 5;
  
  uint16_t output_protocal = (uint16_t)((uint16_t)buff[Number++] << 8 | (uint16_t)buff[Number++]);
  Serial.print(" 输出指示:");
  Serial.println(output_protocal,HEX);
  Last_cal_data_hds.Tag_ID = (uint16_t)((uint16_t)buff[Number++] << 8 | (uint16_t)buff[Number++]);
  Serial.print(" TagID:");
  Serial.println(Last_cal_data_hds.Tag_ID);

  //状态标志位
  uint32_t Cal_Flag = (uint32_t)((uint32_t)buff[Number++] << 24 | (uint32_t)buff[Number++] << 16 | (uint16_t)buff[Number++] << 8 | (uint16_t)buff[Number++]);
  
  if (Check_BitIsTrue(output_protocal, ANC_PROTOCAL_RTLS))  //定位数据
  {
    if (!(Check_BitIsTrue(Cal_Flag, 16)))   //定位解算没有计算成功   ((Cal_Flag >> 8) & 0x01) == 0x00 
    {
      Serial.println("定位解算失败");
      Number += 6;
    }
    else
    {
      Serial.println("定位解算成功");
      Last_cal_data_hds.x = (int)(((uint16_t)((uint16_t)buff[Number++] << 8) & 0xFF00) | (uint16_t)buff[Number++]);
      Last_cal_data_hds.y = (int)(((uint16_t)((uint16_t)buff[Number++] << 8) & 0xFF00) | (uint16_t)buff[Number++]);
      Last_cal_data_hds.z = (int)(((uint16_t)((uint16_t)buff[Number++] << 8) & 0xFF00) | (uint16_t)buff[Number++]);
    }

    Serial.print(" Rtls:X = ");
    Serial.print(Last_cal_data_hds.x,DEC);Serial.print(" cm");
    Serial.print(" Y = ");
    Serial.print(Last_cal_data_hds.y,DEC);Serial.print(" cm");
    Serial.print(" Z = ");
    Serial.print(Last_cal_data_hds.z,DEC);Serial.print(" cm");
    Serial.println("");
    
  }

  if (Check_BitIsTrue(output_protocal, ANC_PROTOCAL_DIST))  //距离可输出
  {
    //获取距离值
    for (int i = 0; i < ANCHOR_MAX_COUNT; i++)
    {
      if (Check_BitIsTrue(Cal_Flag, i)) //((Cal_Flag >> i) & 0x01) == 0x01
      {
          Last_cal_data_hds.Dist[i] = (uint16_t)(((uint16_t)(buff[Number++] << 8) & 0xFF00) | (uint16_t)buff[Number++]);
      }
      else
      {
          Last_cal_data_hds.Dist[i] = 0;
          Number += 2;
      }
    }
    Serial.print(" Dist:AnchorA: ");
    Serial.print(Last_cal_data_hds.Dist[0],DEC);Serial.print(" cm");
    Serial.print(" AnchorB: ");
    Serial.print(Last_cal_data_hds.Dist[1],DEC);Serial.print(" cm");
    Serial.print(" AnchorC: ");
    Serial.print(Last_cal_data_hds.Dist[2],DEC);Serial.print(" cm");
    Serial.print(" AnchorD: ");
    Serial.print(Last_cal_data_hds.Dist[3],DEC);Serial.print(" cm");
    Serial.print(" AnchorE: ");
    Serial.print(Last_cal_data_hds.Dist[4],DEC);Serial.print(" cm");
    Serial.println("");
    Serial.print(" AnchorF: ");
    Serial.print(Last_cal_data_hds.Dist[5],DEC);Serial.print(" cm");
    Serial.print(" AnchorG: ");
    Serial.print(Last_cal_data_hds.Dist[6],DEC);Serial.print(" cm");
    Serial.print(" AnchorH: ");
    Serial.print(Last_cal_data_hds.Dist[7],DEC);Serial.print(" cm");
    Serial.print(" AnchorI: ");
    Serial.print(Last_cal_data_hds.Dist[8],DEC);Serial.print(" cm");
    Serial.print(" AnchorJ: ");
    Serial.print(Last_cal_data_hds.Dist[9],DEC);Serial.print(" cm");
    Serial.println("");
    Serial.print(" AnchorK: ");
    Serial.print(Last_cal_data_hds.Dist[10],DEC);Serial.print(" cm");
    Serial.print(" AnchorL: ");
    Serial.print(Last_cal_data_hds.Dist[11],DEC);Serial.print(" cm");
    Serial.print(" AnchorM: ");
    Serial.print(Last_cal_data_hds.Dist[12],DEC);Serial.print(" cm");
    Serial.print(" AnchorN: ");
    Serial.print(Last_cal_data_hds.Dist[13],DEC);Serial.print(" cm");
    Serial.print(" AnchorO: ");
    Serial.print(Last_cal_data_hds.Dist[14],DEC);Serial.print(" cm");
    Serial.print(" AnchorP: ");
    Serial.print(Last_cal_data_hds.Dist[15],DEC);Serial.print(" cm");
    Serial.println("");
  }

  if (Check_BitIsTrue(output_protocal, ANC_PROTOCAL_RXDIAG))  //接收强度信息
  {
    if(Last_cal_data_hds.Tag_ID == Analyze_TagID)
    {
      Rx_diag.Max_noise = (uint16_t)((uint16_t)buff[Number++] << 8 | (uint16_t)buff[Number++]);
      Rx_diag.Std_noise = (uint16_t)((uint16_t)buff[Number++] << 8 | (uint16_t)buff[Number++]);
      Rx_diag.Fp_amp1 = (uint16_t)((uint16_t)buff[Number++] << 8 | (uint16_t)buff[Number++]);
      Rx_diag.Fp_amp2 = (uint16_t)((uint16_t)buff[Number++] << 8 | (uint16_t)buff[Number++]);
      Rx_diag.Fp_amp3 = (uint16_t)((uint16_t)buff[Number++] << 8 | (uint16_t)buff[Number++]);
      Rx_diag.Max_growthCIR = (uint16_t)((uint16_t)buff[Number++] << 8 | (uint16_t)buff[Number++]);
      Rx_diag.Rx_preambleCount = (uint16_t)((uint16_t)buff[Number++] << 8 | (uint16_t)buff[Number++]);
      Rx_diag.Fp = (uint16_t)(((uint16_t)buff[Number++] << 8 | (uint16_t)buff[Number++]) / 64);  //除以64 取实数部分
    }
    else 
    {
      Number += 16;
    }
    Serial.print(" Max_noise: ");
    Serial.print(Rx_diag.Max_noise,HEX);
    Serial.print(" Std_noise: ");
    Serial.print(Rx_diag.Std_noise,HEX);
    Serial.print(" Fp_amp1: ");
    Serial.print(Rx_diag.Fp_amp1,HEX);
    Serial.print(" Fp_amp2: ");
    Serial.print(Rx_diag.Fp_amp2,HEX);
    Serial.println("");
    Serial.print(" Fp_amp3: ");
    Serial.print(Rx_diag.Fp_amp3,HEX);
    Serial.print(" Max_growthCIR: ");
    Serial.print(Rx_diag.Max_growthCIR,HEX);
    Serial.print(" Rx_preambleCount: ");
    Serial.print(Rx_diag.Rx_preambleCount,HEX);
    Serial.print(" Fp: ");
    Serial.print(Rx_diag.Fp,HEX);
    Serial.println("");
  }

  if (Check_BitIsTrue(output_protocal, ANC_PROTOCAL_TIMESTAMP))
  {
    // uint32_t Time_ts[6];
    Serial.print(" Number: ");
    Serial.println(Number);
    if(Last_cal_data_hds.Tag_ID == Analyze_TagID)
    {
        for (int j = 0; j < 6; j++)
        {
            Last_cal_data_hds.Time_ts[j] = (uint32_t)((uint32_t)buff[Number++] << 24 | (uint32_t)buff[Number++] << 16 | (uint16_t)buff[Number++] << 8 | (uint16_t)buff[Number++]);
        }
    }
    
    Serial.print(" t1: ");
    Serial.print(Last_cal_data_hds.Time_ts[0]);
    Serial.print(" t2: ");
    Serial.print(Last_cal_data_hds.Time_ts[1]);
    Serial.print(" t3: ");
    Serial.print(Last_cal_data_hds.Time_ts[2]);
    Serial.println("");
    Serial.print(" t4: ");
    Serial.print(Last_cal_data_hds.Time_ts[3]);
    Serial.print(" t5: ");
    Serial.print(Last_cal_data_hds.Time_ts[4]);
    Serial.print(" t6: ");
    Serial.print(Last_cal_data_hds.Time_ts[5]);
    Serial.println("");
    Serial.println("");
  }
}

// <summary>
// 标签接收到定位信息的处理
// </summary>
// <param name="buff">要解析的数据</param>
void Tag_RtlsDataRecv(byte *buff)
{
  int Number = 5;
  bool ok_flags[ANCHOR_LIST_COUNT + 1];

  uint16_t output_protocal = (uint16_t)((uint16_t)buff[Number++] << 8 | (uint16_t)buff[Number++]);
  Serial.print(" 输出指示:");
  Serial.println(output_protocal,HEX);

  if (Check_BitIsTrue(output_protocal, TAG_OUTPUT_DIST))
  {
    uint16_t dist_flag = (uint16_t)((uint16_t)((buff[Number++] << 8) & 0xFF00) | (uint16_t)buff[Number++]);
     //测距使能
    for(int i = 0; i < ANCHOR_MAX_COUNT; i++)
    {
      ok_flags[i + 1] = Check_BitIsTrue(dist_flag, i);
      if(ok_flags[i + 1])
        Last_cal_data_hds.Dist[i] = (uint16_t)((uint16_t)((buff[Number++] << 8) & 0xFF00) | (uint16_t)buff[Number++]);
      else
       Number += 2;
    }
      
    
    Serial.print(" Dist:AnchorA: ");
    Serial.print(Last_cal_data_hds.Dist[0],DEC);Serial.print(" cm");
    Serial.print(" AnchorB: ");
    Serial.print(Last_cal_data_hds.Dist[1],DEC);Serial.print(" cm");
    Serial.print(" AnchorC: ");
    Serial.print(Last_cal_data_hds.Dist[2],DEC);Serial.print(" cm");
    Serial.print(" AnchorD: ");
    Serial.print(Last_cal_data_hds.Dist[3],DEC);Serial.print(" cm");
    Serial.print(" AnchorE: ");
    Serial.print(Last_cal_data_hds.Dist[4],DEC);Serial.print(" cm");
    Serial.println("");
    Serial.print(" AnchorF: ");
    Serial.print(Last_cal_data_hds.Dist[5],DEC);Serial.print(" cm");
    Serial.print(" AnchorG: ");
    Serial.print(Last_cal_data_hds.Dist[6],DEC);Serial.print(" cm");
    Serial.print(" AnchorH: ");
    Serial.print(Last_cal_data_hds.Dist[7],DEC);Serial.print(" cm");
    Serial.print(" AnchorI: ");
    Serial.print(Last_cal_data_hds.Dist[8],DEC);Serial.print(" cm");
    Serial.print(" AnchorJ: ");
    Serial.print(Last_cal_data_hds.Dist[9],DEC);Serial.print(" cm");
    Serial.println("");
    Serial.print(" AnchorK: ");
    Serial.print(Last_cal_data_hds.Dist[10],DEC);Serial.print(" cm");
    Serial.print(" AnchorL: ");
    Serial.print(Last_cal_data_hds.Dist[11],DEC);Serial.print(" cm");
    Serial.print(" AnchorM: ");
    Serial.print(Last_cal_data_hds.Dist[12],DEC);Serial.print(" cm");
    Serial.print(" AnchorN: ");
    Serial.print(Last_cal_data_hds.Dist[13],DEC);Serial.print(" cm");
    Serial.print(" AnchorO: ");
    Serial.print(Last_cal_data_hds.Dist[14],DEC);Serial.print(" cm");
    Serial.print(" AnchorP: ");
    Serial.print(Last_cal_data_hds.Dist[15],DEC);Serial.print(" cm");
    Serial.println("");
  }
  if (Check_BitIsTrue(output_protocal, TAG_OUTPUT_RTLS))
  {
    ok_flags[0] = ((buff[Number++] << 8) & 0xFF00 | buff[Number++]) == 1;
     if (ok_flags[0])
     {
      //定位使能
      Last_cal_data_hds.x = (int)((uint16_t)((buff[Number++] << 8) & 0xFF00) | (uint16_t)buff[Number++]);
      Last_cal_data_hds.y = (int)((uint16_t)((buff[Number++] << 8) & 0xFF00) | (uint16_t)buff[Number++]);
      Last_cal_data_hds.z = (int)((uint16_t)((buff[Number++] << 8) & 0xFF00) | (uint16_t)buff[Number++]);
     }

    Serial.print(" Rtls:X = ");
    Serial.print(Last_cal_data_hds.x,DEC);Serial.print(" cm");
    Serial.print(" Y = ");
    Serial.print(Last_cal_data_hds.y,DEC);Serial.print(" cm");
    Serial.print(" Z = ");
    Serial.print(Last_cal_data_hds.z,DEC);Serial.print(" cm");
    Serial.println("");
    Serial1.write(0xAA);//发送xy坐标给飞控
    Serial1.write(0x01);
    Serial1.write(0x04);
  int16_t x = Last_cal_data_hds.x;
  int16_t y = Last_cal_data_hds.y;

// 分别发送 x 的高位字节和低位字节
Serial1.write((x >> 8) & 0xFF); // 发送 x 的高位字节
Serial1.write(x & 0xFF);        // 发送 x 的低位字节
//bee_3();
// 分别发送 y 的高位字节和低位字节
Serial1.write((y >> 8) & 0xFF); // 发送 y 的高位字节
Serial1.write(y & 0xFF);
    Serial1.write(0xAF);
  }
}


