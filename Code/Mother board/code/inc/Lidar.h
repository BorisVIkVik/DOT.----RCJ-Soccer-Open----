#ifndef _LIDAR_H
#define _LIDAR_H


#include <stdint.h>
//#include "bsp_uart.h"
#include <stm32f407_UART.h>
#include "check.h"

/********************************************************************************************************************/
//通讯相关参数定义
/********************************************************************************************************************/
#define FRAME_PARAM_MAX_RX_LEN		            (2000 + 10)	                                	//接收数据参数的最大长度
#define FRAME_PARAM_MAX_TX_LEN		            100		                                			//发送数据参数的最大长度

#define FRAME_HEAD		                        	0xAA                                        //帧头
#define FRAME_PROTOCAL_VERSION	  	            0x00                                        //协议版本                                     
#define FRAME_TYPE						               		0x61                                        //帧类型

#define FRAME_MEASURE_INFO											0xAD
#define FRAME_DEVICE_HEALTH_INFO								0xAE

//雷达扫描状态
enum SCANSTATE
{
	GRAB_SCAN_FIRST = 0,
	GRAB_SCAN_ELSE_DATA
};	
enum SCANRESULT
{
	LIDAR_GRAB_ING = 0,
	LIDAR_GRAB_SUCESS,
	LIDAR_GRAB_ERRO,
	LIDAR_GRAB_ELSE
};

//定义雷达齿轮个数
#define  TOOTH_NUM																	4

/********************************************************************************************************************/
//通讯帧定义
/********************************************************************************************************************/
#pragma pack (1)
typedef struct
{
	uint8_t Header;
	uint16_t Len;
	uint8_t Addr;
	uint8_t CmdType;
	uint8_t CmdId;
	uint16_t ParamLen;
	uint8_t *Data;
}T_PROTOCOL;

//点信息帧定义
typedef struct 
{
	float Angle;
	float Distance;
}T_POINT;
//测量帧定义
typedef struct 
{
	float RotateSpeed;
	float ZeroOffset;
	float FrameStartAngle;
	uint8_t PointNum;//一帧的点数
	T_POINT Point[100];//一帧 点信息
}T_FRAME_MEAS_INFO;

//设备健康信息帧
typedef struct 
{
	uint8_t ErrCode;
}T_DEVICE_HEALTH_INFO;

//雷达扫描信息定义
typedef struct
{
	uint8_t State;
	uint8_t Result;
	float LastScanAngle;
	uint8_t ToothCount;
	uint16_t OneCriclePointNum;//一圈点数
	T_FRAME_MEAS_INFO FrameMeasInfo;//一帧测量信息
	T_POINT OneCriclePoint[1000];//一圈测量点信息：从零点开始，由16帧点信息组成
}T_LIDARSCANINFO;
#pragma pack ()

//T_LIDARSCANINFO lidarscaninfo;

//void Lidarscaninfo_Init(void);
//uint8_t P_Cmd_Process(void);

//void Lidarscaninfo_Init(void)
//{
//	lidarscaninfo.State = GRAB_SCAN_FIRST;
//	lidarscaninfo.ToothCount=0;
//	lidarscaninfo.LastScanAngle=0.0;
//	lidarscaninfo.Result = LIDAR_GRAB_ING;
//	lidarscaninfo.OneCriclePointNum=0;
//}

uint16_t Little2BigEndian_u16(uint16_t dat)
{
	return ((dat>>8)|(dat<<8));
}
/********************************************************************************************/
//2个字符转换成uint16_t函数
//参数:     无
//返回值:   无
/*****************************************************************************************/
uint16_t Strto_u16(uint8_t* str)
{
	return ((*str << 8) | *(str+1));
}

void LittleCopy_u16(uint8_t *dest,uint8_t *src,uint16_t len)
{
	while(len)
  {
		*dest = *(src+1);
		*(dest+1) = *src;
		dest+=2;src+=2;
		len--;
	}
}

uint8_t isFirstGratingScan(float angle)
{
	return angle<0.0001? 1:0;
}



class Lidar
{
	public:
		void LidarInit(uint32_t uartLIDAR)
		{
			this->uartLIDAR = uartLIDAR; 
			initUART(uartLIDAR, 230400);
		}
		void Lidarscaninfo_Init(void)
		{
			LSI.State = GRAB_SCAN_FIRST;
			LSI.ToothCount=0;
			LSI.LastScanAngle=0.0;
			LSI.Result = LIDAR_GRAB_ING;
			LSI.OneCriclePointNum=0;
		}
		
		void OneCriclePoint_insert(T_FRAME_MEAS_INFO one_meas_info)
		{
			uint8_t i=0;
			for(i=0;i<one_meas_info.PointNum;i++)
			{
				LSI.OneCriclePoint[LSI.OneCriclePointNum].Angle=one_meas_info.Point[i].Angle;
				LSI.OneCriclePoint[LSI.OneCriclePointNum].Distance=one_meas_info.Point[i].Distance;
				LSI.OneCriclePointNum++;
			}
		}
		
		void grabFirstGratingScan(T_FRAME_MEAS_INFO one_meas_info)
		{
			LSI.State = GRAB_SCAN_ELSE_DATA;
			LSI.ToothCount=1;
			LSI.LastScanAngle=one_meas_info.FrameStartAngle;
			OneCriclePoint_insert(one_meas_info);
		}
		
		void ScanOneCircle(T_FRAME_MEAS_INFO one_meas_info)
		{
			switch(LSI.State)
			{
				case GRAB_SCAN_FIRST:
						//first tooth scan come
						if(isFirstGratingScan(one_meas_info.FrameStartAngle))
						{
							Lidarscaninfo_Init();
							grabFirstGratingScan(one_meas_info);
						}
						else
						{
							//printf("[Lidar] GRAB_SCAN_FIRST is not zero; scan angle %5.2f!\n",one_meas_info.FrameStartAngle);
						}
						break;
				
				case GRAB_SCAN_ELSE_DATA:
						/* Handle angle suddenly reduces */
						if(one_meas_info.FrameStartAngle < LSI.LastScanAngle) 
						{    
								//printf("[Lidar] Restart scan, FrameStartAngle: %5.2f, LastScanAngle: %5.2f!\n",one_meas_info.FrameStartAngle, lidarscaninfo.LastScanAngle); 
								//这次角度小于上一次角度，且角度为零：表示这一圈数据不完整，刚好从零点重新下一圈(这圈数据丢掉)
								if(isFirstGratingScan(one_meas_info.FrameStartAngle))
								{
									Lidarscaninfo_Init();
									grabFirstGratingScan(one_meas_info);
								}
								else //这次角度小于上一次角度，且角度不为零：表示这一圈和下圈数据都不完整，重新矫正到零点开始扫描，这一圈和下圈数据都丢掉
								{
									LSI.State = GRAB_SCAN_FIRST;
								}
								return;				
						}
						OneCriclePoint_insert(one_meas_info);
						LSI.ToothCount++;
						LSI.LastScanAngle = one_meas_info.FrameStartAngle;

						if(LSI.ToothCount == TOOTH_NUM)//Scan One Circle
						{
							LSI.ToothCount=0;
							LSI.State = GRAB_SCAN_FIRST;
							LSI.Result =  LIDAR_GRAB_SUCESS;
							return;
						}
						break;			
				default:
						//printf("[Lidar] Uknow grab scan data state\n");
						break;
			}
		}

		void AnalysisMeasureInfo(T_PROTOCOL* Preq)
		{
			uint8_t data_head_offset=0;
			uint8_t i = 0;
			float per_angle_offset = 0;

			Preq->ParamLen = Little2BigEndian_u16(Preq->ParamLen);
			
			LSI.FrameMeasInfo.RotateSpeed=(*(Preq->Data))*0.05f;
			data_head_offset +=1;
			LSI.FrameMeasInfo.ZeroOffset=Strto_u16(Preq->Data+data_head_offset)*0.01f;
			data_head_offset +=2;   
			LSI.FrameMeasInfo.FrameStartAngle=Strto_u16(Preq->Data+data_head_offset)*0.01f;
			data_head_offset +=2;
			LSI.FrameMeasInfo.PointNum = (Preq->ParamLen-data_head_offset)/3;
			
			per_angle_offset=22.5f/LSI.FrameMeasInfo.PointNum;
			for(i=0;i<LSI.FrameMeasInfo.PointNum;i++)
			{
				LSI.FrameMeasInfo.Point[i].Distance = Strto_u16(Preq->Data+data_head_offset+3*i+1)*0.25f;
				LSI.FrameMeasInfo.Point[i].Angle =	LSI.FrameMeasInfo.FrameStartAngle+i*per_angle_offset;
			}
			
			ScanOneCircle(LSI.FrameMeasInfo);
		}
		
		
		void AnalysisDeviceHealthInfo(T_PROTOCOL* Preq)
		{
			float speed = 0;
			speed = *(Preq->Data)*0.05f;
			//printf("[Lidar] Speed error!\n");
		}
		
		uint8_t FrameIsRight(T_PROTOCOL Preq)
		{
			uint16_t Temp = 0,CalcCRC,Len;
			Len = Little2BigEndian_u16(Preq.Len);
			if(Preq.Header != 0xAA) 
			{	
				return 0;
			}
			else if(Len > RxBuffer.Len)
			{
				return 0;
			}
			else if(Preq.CmdType != FRAME_TYPE)
			{
				return 0;
			} 
			else 
			{	
				//1.crc校验
				//CalcCRC = CRC16(RxBuffer.Buff,Len);
				//2.累加校验和
				CalcCRC	= Calc_CheckSum(RxBuffer.Buff,Len);
				
				LittleCopy_u16((uint8_t *)&Temp,&RxBuffer.Buff[Len],1);
				if(CalcCRC != Temp)
				{
//					printf("[Lidar] Frame is CRC error!\n");
					return 0;
				}
			}
			return 1;
		}
		
		
		void FrameAnalysis(T_PROTOCOL Preq)
		{
			switch(Preq.CmdId) 
			{
				case FRAME_MEASURE_INFO://雷达测量信息帧
					AnalysisMeasureInfo(&Preq);
					break;
				case FRAME_DEVICE_HEALTH_INFO://雷达设备健康信息帧
					AnalysisDeviceHealthInfo(&Preq);
					break;
				default:
					return;
			}
		}
		
		void ProcessUartRxData(void)
		{
			if(RxBuffer.Rdy == 0)
				return;

			if(RxBuffer.Rdy > 0)
				P_Cmd_Process();
			RxBuffer.Len = 0;
			RxBuffer.Rdy = 0;
		}
		
		uint8_t P_Cmd_Process(void)
		{
			T_PROTOCOL Preq;
				
			memcpy((uint8_t *)&Preq,(uint8_t * )RxBuffer.Buff,8);
			Preq.Data = &RxBuffer.Buff[8];
			
			if(FrameIsRight(Preq)==0)
			{
				return 0;
			}
			FrameAnalysis(Preq);
			return 1;
		}
		
		T_LIDARSCANINFO LSI;
	private:
			
			uint32_t uartLIDAR;
			
			
};

#endif
