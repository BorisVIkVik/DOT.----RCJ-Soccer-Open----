#ifndef _LIDAR_H
#define _LIDAR_H


#include <stdint.h>
//#include "bsp_uart.h"
#include <stm32f407_UART.h>
#include "check.h"

/********************************************************************************************************************/
//ͨѶ��ز�������
/********************************************************************************************************************/
#define FRAME_PARAM_MAX_RX_LEN		            (2000 + 10)	                                	//�������ݲ�������󳤶�
#define FRAME_PARAM_MAX_TX_LEN		            100		                                			//�������ݲ�������󳤶�

#define FRAME_HEAD		                        	0xAA                                        //֡ͷ
#define FRAME_PROTOCAL_VERSION	  	            0x00                                        //Э��汾                                     
#define FRAME_TYPE						               		0x61                                        //֡����

#define FRAME_MEASURE_INFO											0xAD
#define FRAME_DEVICE_HEALTH_INFO								0xAE

//�״�ɨ��״̬
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

//�����״���ָ���
#define  TOOTH_NUM																	4

/********************************************************************************************************************/
//ͨѶ֡����
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

//����Ϣ֡����
typedef struct 
{
	float Angle;
	float Distance;
}T_POINT;
//����֡����
typedef struct 
{
	float RotateSpeed;
	float ZeroOffset;
	float FrameStartAngle;
	uint8_t PointNum;//һ֡�ĵ���
	T_POINT Point[100];//һ֡ ����Ϣ
}T_FRAME_MEAS_INFO;

//�豸������Ϣ֡
typedef struct 
{
	uint8_t ErrCode;
}T_DEVICE_HEALTH_INFO;

//�״�ɨ����Ϣ����
typedef struct
{
	uint8_t State;
	uint8_t Result;
	float LastScanAngle;
	uint8_t ToothCount;
	uint16_t OneCriclePointNum;//һȦ����
	T_FRAME_MEAS_INFO FrameMeasInfo;//һ֡������Ϣ
	T_POINT OneCriclePoint[1000];//һȦ��������Ϣ������㿪ʼ����16֡����Ϣ���
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
//2���ַ�ת����uint16_t����
//����:     ��
//����ֵ:   ��
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
								//��νǶ�С����һ�νǶȣ��ҽǶ�Ϊ�㣺��ʾ��һȦ���ݲ��������պô����������һȦ(��Ȧ���ݶ���)
								if(isFirstGratingScan(one_meas_info.FrameStartAngle))
								{
									Lidarscaninfo_Init();
									grabFirstGratingScan(one_meas_info);
								}
								else //��νǶ�С����һ�νǶȣ��ҽǶȲ�Ϊ�㣺��ʾ��һȦ����Ȧ���ݶ������������½�������㿪ʼɨ�裬��һȦ����Ȧ���ݶ�����
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
				//1.crcУ��
				//CalcCRC = CRC16(RxBuffer.Buff,Len);
				//2.�ۼ�У���
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
				case FRAME_MEASURE_INFO://�״������Ϣ֡
					AnalysisMeasureInfo(&Preq);
					break;
				case FRAME_DEVICE_HEALTH_INFO://�״��豸������Ϣ֡
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
