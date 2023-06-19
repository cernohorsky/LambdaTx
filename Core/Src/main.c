/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <stdlib.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

//**********************start user define************!!!!!
// cpu clock 84 for stm32f401
#define SYS_CLK_MHZ 84

//#define EYE
// use internal timers for sound out
#define   USE_PWM
//#define USE_I2S
//#define   USE_SPDIF

//slower for float - faster for integer
//#define USE_FLOAT_SIGMA

// integration order 0 , 1 or 2
//#define ORDER 1
#define ORDER 1


// if disabled - direct output (order 0)

// if enabled - use bilinear interpolation
//#define LINEAR_INTERPOLATION

//**********************end user define area************!!!!!!!!

#define TIMER_CLOCK_FREQ 400000.0f

#ifdef  USE_PWM
#if     (ORDER!=0)
#define FREQ  (48000*6)
#define MAX_VOL  (SYS_CLK_MHZ*1000000/FREQ)
#else
#define FREQ  (44100)
#define MAX_VOL = (SYS_CLK_MHZ*1000000/FREQ)
#endif
#endif

#ifdef  USE_I2S  //No PWM, I2S output to external DAC
#define MAX_VOL (65536)
#define FREQ   (2*96000)
#endif

#ifdef  USE_SPDIF  //No PWM, I2S output to external DAC
#define MAX_VOL (65536)
#define FREQ   (96000/2)
#endif

#define DBL_SAMPL 1


#define SIGMA_BITS  16
#define SIGMA       (1<<SIGMA_BITS)


#define USB_DATA_BITS   16
#define USB_DATA_BITS_H (USB_DATA_BITS-1)


#define SPDIF_FRAMES 192
#ifdef  USE_SPDIF
#define N_SIZE      (SPDIF_FRAMES*4)
#else
#define N_SIZE_BITS (8)
#define N_SIZE (1<<N_SIZE_BITS)
#endif


#define ASBUF_SIZE     (N_SIZE*2)
int16_t baudio_buffer[ASBUF_SIZE];

// between [14  20]
#define TIME_BIT_SCALE_FACT 18u
#define TIME_SCALE_FACT     (1u<<TIME_BIT_SCALE_FACT)

//linear is more precision
#ifdef LINEAR_INTERPOLATION
#define BIT_SHIFT_SCALE_FACT 12
#define SHIFT_SCALE_FACT  (1<<BIT_SHIFT_SCALE_FACT)
#endif


//#define MAX_VOL (2046/TIME_SCALE_FACT)


/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2S_HandleTypeDef hi2s2;
DMA_HandleTypeDef hdma_spi2_tx;

SPI_HandleTypeDef hspi3;
DMA_HandleTypeDef hdma_spi3_tx;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_I2S2_Init(void);
static void MX_SPI3_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM2_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

uint16_t* VoiceBuff0 = (uint16_t*)&baudio_buffer[0];
uint16_t* VoiceBuff1 = (uint16_t*)&baudio_buffer[N_SIZE];


void TIM1_TE1()
{
}
void TIM1_TC2()
{
}
void TIM1_HT2()
{
}
void TIM1_TE2()
{
}


void init_timers()
{

	  TIM1->ARR = MAX_VOL-1;
	  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_DMA2);
	  LL_DMA_ConfigAddresses(DMA2, LL_DMA_STREAM_1, (uint32_t)&VoiceBuff0[0], (uint32_t)&TIM1->CCR1, LL_DMA_GetDataTransferDirection(DMA2, LL_DMA_STREAM_1));
	  LL_DMA_SetDataLength(DMA2, LL_DMA_STREAM_1, N_SIZE);
	  LL_DMA_EnableIT_TC(DMA2, LL_DMA_STREAM_1);
	  LL_DMA_EnableIT_TE(DMA2, LL_DMA_STREAM_1);
	  LL_DMA_EnableIT_HT(DMA2, LL_DMA_STREAM_1);

	  LL_DMA_ConfigAddresses(DMA2, LL_DMA_STREAM_2, (uint32_t)&VoiceBuff1[0], (uint32_t)&TIM1->CCR2, LL_DMA_GetDataTransferDirection(DMA2, LL_DMA_STREAM_2));
	  LL_DMA_SetDataLength(DMA2, LL_DMA_STREAM_2, N_SIZE);
	  LL_DMA_EnableIT_TC(DMA2, LL_DMA_STREAM_2);
	  LL_DMA_EnableIT_HT(DMA2, LL_DMA_STREAM_2);
	  LL_DMA_EnableIT_TE(DMA2, LL_DMA_STREAM_2);



	  /***************************/
	  /* Enable the DMA transfer */
	  /***************************/
	  LL_DMA_EnableStream(DMA2, LL_DMA_STREAM_1);
	  LL_DMA_EnableStream(DMA2, LL_DMA_STREAM_2);

	  LL_TIM_EnableDMAReq_CC1(TIM1);
	  LL_TIM_EnableDMAReq_CC2(TIM1);

	  LL_TIM_CC_EnableChannel(TIM1, LL_TIM_CHANNEL_CH1);
	  LL_TIM_CC_EnableChannel(TIM1, LL_TIM_CHANNEL_CH2);



	  LL_TIM_OC_EnablePreload(TIM1, LL_TIM_CHANNEL_CH1);
	  LL_TIM_OC_EnablePreload(TIM1, LL_TIM_CHANNEL_CH2);

	  LL_TIM_CC_EnableChannel(TIM1, LL_TIM_CHANNEL_CH1N);
	  LL_TIM_CC_EnableChannel(TIM1, LL_TIM_CHANNEL_CH2N);
#if 0
	  //LL_TIM_OC_SetPolarity(TIM1, LL_TIM_CHANNEL_CH1,LL_TIM_OCPOLARITY_HIGH);
	  //LL_TIM_OC_SetPolarity(TIM1, LL_TIM_CHANNEL_CH1N,LL_TIM_OCPOLARITY_LOW);
	  LL_TIM_OC_SetPolarity(TIM1, LL_TIM_CHANNEL_CH1,LL_TIM_OCPOLARITY_HIGH);
	  LL_TIM_OC_SetPolarity(TIM1, LL_TIM_CHANNEL_CH1N,LL_TIM_OCPOLARITY_HIGH);
	  LL_TIM_OC_SetPolarity(TIM1, LL_TIM_CHANNEL_CH2,LL_TIM_OCPOLARITY_HIGH);
	  LL_TIM_OC_SetPolarity(TIM1, LL_TIM_CHANNEL_CH2N,LL_TIM_OCPOLARITY_HIGH);
      int res;
	  res = LL_TIM_OC_GetPolarity(TIM1, LL_TIM_CHANNEL_CH1);
	  printf("OC_GetPolarity %d\n",res);
	  res = LL_TIM_OC_GetPolarity(TIM1, LL_TIM_CHANNEL_CH1N);
	  printf("OC_GetPolarity %d\n",res);
#endif


	  LL_TIM_EnableCounter(TIM1);

	  LL_TIM_GenerateEvent_UPDATE(TIM1);
	  LL_TIM_GenerateEvent_CC1(TIM1);
	  

}
//void init_timer3LL()
//{
//	LL_TIM_EnableCounter(TIM3);
//	LL_TIM_GenerateEvent_UPDATE(TIM3);
//	LL_TIM_GenerateEvent_CC1(TIM3);
//}

uint32_t AUDIO_PeriodicTC_FS_Counter;
void retarget_put_char(char p)
{
	HAL_UART_Transmit(&huart2, &p, 1, 0xffffff); // send message via UART
}
int _write(int fd, char* ptr, int len)
{
    (void)fd;
    int i = 0;
    while (ptr[i] && (i < len))
    {
    	if (ptr[i] == '\r')
    	{

    	}
    	else
    	{
			retarget_put_char((int)ptr[i]);
			if (ptr[i] == '\n')
			{
				retarget_put_char((int)'\r');
			}
    	}
        i++;
    }
    return len;
}







volatile int AUDIO_OUT_Play_Counter;
volatile int AUDIO_OUT_ChangeBuffer_Counter;
volatile int TransferComplete_CallBack_FS_Counter;
volatile int HalfTransfer_CallBack_FS_Counter;



//volatile fl = 0;
uint32_t  samplesInBuff        = 0;
int32_t   samplesInBuffH       = 0;
int16_t * usb_SndBuffer        = 0;


uint32_t  samplesInBuffScaled  = 0;
uint32_t  readPositionXScaled  = 0;  //readPosition<<12;
uint32_t  readSpeedXScaled     = 1.001*TIME_SCALE_FACT*USBD_AUDIO_FREQ/(float)FREQ;
int frequency=123;

struct LR
{
	int16_t L;
	int16_t R;
	uint32_t freq;
};
volatile int32_t ampL = 0;
volatile int32_t ampR = 0;
volatile int32_t LLL = 0;
volatile int32_t RRR = 0;
volatile int UsbSamplesAvail = 0;
int zeroSamplePosition=0;
int lastZeroSamplePosition=0;

struct LR getNextSampleLR()
{
	struct LR res ;
	res.L = 0;
	res.R = 0;
	//res.freq = 0;
	//if(usb_SndBuffer)
	HAL_GPIO_WritePin(LED_GPIO_Port,LED_Pin,GPIO_PIN_SET);

	if(UsbSamplesAvail)
	{
		HAL_GPIO_WritePin(LED_GPIO_Port,LED_Pin,GPIO_PIN_RESET);
		readPositionXScaled += readSpeedXScaled;
		if(readPositionXScaled >= samplesInBuffScaled)
		{
			readPositionXScaled -= samplesInBuffScaled;
		}

		uint32_t readPositionIntC = readPositionXScaled>>TIME_BIT_SCALE_FACT;
		//uint32_t readPositionIntN = readPositionIntC+1;

		int16_t L  = usb_SndBuffer[readPositionIntC*2+0];//+usb_SndBuffer[readPositionIntN*2+0];
		int16_t R  = usb_SndBuffer[readPositionIntC*2+1];//+usb_SndBuffer[readPositionIntN*2+1];

		// zero cross
		int16_t L_previous  = usb_SndBuffer[readPositionIntC*2+2];

		if(L<0 && L_previous>0 && L!=0 && L!=-1 && L_previous!=0 && L_previous!=-1)
		{
			zeroSamplePosition=readPositionIntC*2;
		}

		int samplesCount = (zeroSamplePosition-lastZeroSamplePosition)/2;

		if(samplesCount>15 && samplesCount<480)
		{
			int16_t LastSample = usb_SndBuffer[lastZeroSamplePosition+0];
			int16_t nextToLastSample = usb_SndBuffer[lastZeroSamplePosition+2];

			//linear interpolation
			int32_t startOffset = -(nextToLastSample*20833)/(LastSample-nextToLastSample);
			int32_t endOffset = -(L*20833)/(L_previous-L);
			int32_t period= startOffset + (samplesCount-1) * 20833 + endOffset;

			frequency=100000000000/period;

			/*
			int samples[20];
			int j = 0;

			for (int i=0;i<40;i=i+2)
			{
				samples[j] = usb_SndBuffer[lastZeroSamplePosition+i];
				j++;
			}

			j=0;
			*/
			lastZeroSamplePosition=0;
			zeroSamplePosition=0;
		}
		else
		{
			lastZeroSamplePosition=zeroSamplePosition;
		}

		if (L==0 && L_previous==0)
		{
			res.freq = 0;
		}
		else
		{
			res.freq = frequency;
		}

		res.L = L;
		res.R = R;
	}

    return res;
}
void  AUDIO_Init(uint32_t AudioFreq, uint32_t Volume, uint32_t options)
{
	printf("AUDIO_Init %d %d %d\n",AudioFreq,Volume,options);
}

void AUDIO_OUT_Start(uint16_t* pBuffer, uint32_t Size)
{
	printf("AUDIO_OUT_Start %p Size = %d\n",pBuffer,Size);
	samplesInBuff = Size/4; //(L & R channels short)
	samplesInBuffH = samplesInBuff/2;

	usb_SndBuffer = pBuffer;
	AUDIO_OUT_Play_Counter++;

	samplesInBuffScaled = samplesInBuff<<TIME_BIT_SCALE_FACT;
	readPositionXScaled = samplesInBuffScaled/2;

}

uint16_t  lastDmaAccessTime;
int       lastDmaPos;
uint32_t   outDmaSpeedScaled;
int       sForcedSpeed;
uint16_t  lastAudioUsbTimeStamp;
float inputSpeed = 0;

int prevPos = -1 ;

int  median3(int  a, int  b, int  c)
{
   return (b<a)
              ?   (b<c)  ?  (c<a) ? c : a  :  b
              :   (a<c)  ?  (c<b) ? c : b  :  a;
}

int timeForRecivedSamples_mean[3];
int pnt_timeForRecivedSamples_mean = 0;
int appDistErr = 0;


void AUDIO_OUT_Periodic(uint16_t* pBuffer, uint32_t Size)
{
	AUDIO_PeriodicTC_FS_Counter++;
	if(!usb_SndBuffer) return ;
	int cPos  = (pBuffer - (uint16_t*)usb_SndBuffer)/2;
	uint16_t lastAudioUsbTimeStampNew = TIM3->CNT;
	if(cPos==0)
	{
		uint16_t timeForRecivedSamples = lastAudioUsbTimeStampNew - lastAudioUsbTimeStamp;

		timeForRecivedSamples_mean [pnt_timeForRecivedSamples_mean++] = timeForRecivedSamples;
		if(pnt_timeForRecivedSamples_mean>2)pnt_timeForRecivedSamples_mean = 0;


        lastAudioUsbTimeStamp = lastAudioUsbTimeStampNew;

	}

	if(cPos==0)
	{

		//


		int  timeForRecivedSamples = median3(timeForRecivedSamples_mean[0],timeForRecivedSamples_mean[1],timeForRecivedSamples_mean[2]);
		//(TIMER_CLOCK_FREQ*(float)outDmaSpeedScaled)/TIME_SCALE_FACT;
		if(timeForRecivedSamples)
			inputSpeed = samplesInBuff*TIMER_CLOCK_FREQ/timeForRecivedSamples;


		//sForcedSpeed = (int)(samplesInBuff*TIME_SCALE_FACT*mean/(timeForRecivedSamples*N_SIZE);

		uint16_t timeFromLastDMA = lastAudioUsbTimeStampNew - lastDmaAccessTime;


		//where i am ?

		int approximateSamplesOutedFromLastDMA  = ((float)timeFromLastDMA/TIMER_CLOCK_FREQ)*inputSpeed;



		int appDistance  =  (int)(lastDmaPos + approximateSamplesOutedFromLastDMA )-cPos;
		if(appDistance    < 0 ) appDistance += samplesInBuff;
		//while(appDistance > samplesInBuff)  appDistance -= samplesInBuff;
		int err = appDistance - samplesInBuffH;


        if(UsbSamplesAvail)
		{
        	if(timeForRecivedSamples)
        	{
				int dC = appDistance - prevPos;
				while(dC>samplesInBuffH)  dC-=samplesInBuff;
				while(dC<-samplesInBuffH) dC+=samplesInBuff;
				appDistErr = err;
				if(err > samplesInBuffH/2 || err <-samplesInBuffH/2 ) //seems completely lost sync , force set frequency
				{
					float outSpeed = (TIMER_CLOCK_FREQ*(float)outDmaSpeedScaled)/TIME_SCALE_FACT;
					sForcedSpeed = (int)(inputSpeed*TIME_SCALE_FACT/outSpeed);
					readSpeedXScaled = sForcedSpeed;
					readPositionXScaled = samplesInBuffScaled/2;
				}
				else
				{
					//ok - only phase tune
					readSpeedXScaled -= dC + err/256;// + 8*err/samplesInBuff ;//- ((err>0)?1:(err<0)?-1:0);
				}
        	}
		}
        else
        {
        	readPositionXScaled = samplesInBuffScaled/2;
        }
		prevPos =  appDistance;
		UsbSamplesAvail = samplesInBuff*((int)((float)FREQ/USBD_AUDIO_FREQ+0.5f))*2;
	}

}

typedef struct
{
	float x;
	float y;
} vec2f;

typedef float mat4f[4] ;
void printMat(mat4f m2)
{
	printf("%1.3f %1.3f\n",m2[0],m2[1]);
	printf("%1.3f %1.3f\n",m2[2],m2[3]);
}

void printVec(vec2f v)
{
	printf("x=%1.3f y=%1.3f\n",v.x,v.y);
}



vec2f applyMat(mat4f m2,vec2f v)
{
	vec2f res;
	res.x = v.x*m2[0]+v.y*m2[1];
	res.y = v.x*m2[2]+v.y*m2[3];
	return res;
}



void makeRotMat(mat4f xx,float alpha)
{
	float cosA = cosf(alpha);
	float sinA = sinf(alpha);
	xx[0]  = cosA; xx[1]  = -sinA;
	xx[2]  = sinA; xx[3]  = cosA;
}
void AUDIO_OUT_ChangeBuffer(uint16_t *pBuffer, uint16_t Size)
{
	printf("%p SizeC = %d\n",pBuffer,Size);
		AUDIO_OUT_ChangeBuffer_Counter++;
		//fl = 1;
		//HAL_I2S_Transmit_DMA(&hi2s2, pBuffer,Size);
}
void cleanAudioBuffer()
{
#ifdef PWM
	for(int k=0;k<N_SIZE;k++)
	{
		VoiceBuff0[k] = MAX_VOL/2;
		VoiceBuff1[k] = MAX_VOL/2;
	}
#else
	for(int k=0;k<N_SIZE;k++)
	{
		VoiceBuff0[k] = 0;
		VoiceBuff1[k] = 0;
	}
#endif
}

struct sigmaDeltaStorage
{
	float integral;
	float y;
};
struct sigmaDeltaStorage2
{
	float integral0;
	float integral1;
	float y;
};


float funcN1(float x,int N)
{
	//float y = 2.0f*floorf(x*0.5f)+1.0f;
	float y = floorf(x+0.5f);
	//saturate
	if(y>N) y = N;
	if(y<-N) y = -N;
	return y;
}


float sigma_delta_slow(struct sigmaDeltaStorage* st,float x)
{
	st->integral+= x - st->y;
	st->y = funcN1(st->integral,(int)(MAX_VOL/2));
	return st->y;
}



struct  sigmaDeltaStorage_SCALED
{
	int integral;
//	int y;
};
struct  sigmaDeltaStorage2_SCALED
{
	int integral0;
	int integral1;
	//int y;
};

float sigma_delta(struct sigmaDeltaStorage* st,float x)
{
	st->integral+= x - st->y;
	st->y = floorf(st->integral+0.5f);
	if(st->y<0)st->y = 0;
	if(st->y>MAX_VOL)st->y = MAX_VOL;

	return st->y;
}

float sigma_delta2(struct sigmaDeltaStorage2* st,float x)
{
	st->integral0+= x             - st->y;
	st->integral1+= st->integral0 - st->y;
	st->y = floorf(st->integral1+0.5f);
	if(st->y<0)st->y = 0;
	if(st->y>MAX_VOL)st->y = MAX_VOL;

	return st->y;
}


int sigma_delta_SCALED(struct sigmaDeltaStorage_SCALED* st,int x_SCALED)
{
	int y		 =st->integral>>SIGMA_BITS;
	if(y < 0)
		y = 0;
	if(y > MAX_VOL)
		y = MAX_VOL;

	st->integral+= x_SCALED - (y<<SIGMA_BITS);
	return y;
}

int sigma_delta2_SCALED(struct sigmaDeltaStorage2_SCALED* st,int x_SCALED)
{
	int y		 =st->integral1>>SIGMA_BITS;
	if(y < 0)
		y = 0;
	if(y > MAX_VOL)
		y = MAX_VOL;

	st->integral0+= x_SCALED      - (y<<SIGMA_BITS);
	st->integral1+= st->integral0 - (y<<SIGMA_BITS);
	//st->y		 =(st->integral1+SIGMA/2)/SIGMA;
	return y;
}

struct sigmaDeltaStorage static_L_channel;
struct sigmaDeltaStorage static_R_channel;

struct sigmaDeltaStorage_SCALED static_L_channel_SCALED;
struct sigmaDeltaStorage_SCALED static_R_channel_SCALED;

struct sigmaDeltaStorage2 static_L_channel2;
struct sigmaDeltaStorage2 static_R_channel2;

struct sigmaDeltaStorage2_SCALED static_L_channel2_SCALED;
struct sigmaDeltaStorage2_SCALED static_R_channel2_SCALED;

int tfl_mean[3];
int pnt_mean = 0;
void checkTime()
{
	uint16_t prevTime;
	uint16_t  tfl;
	prevTime = lastDmaAccessTime;
	lastDmaAccessTime = TIM3->CNT;
	//lastDmaPos  = readPositionXScaled>>TIME_BIT_SCALE_FACT;
	//tfl = lastDmaAccessTime -prevTime;
	//outDmaSpeedScaled =  TIME_SCALE_FACT*N_SIZE/(tfl);

	lastDmaPos  = readPositionXScaled>>TIME_BIT_SCALE_FACT;
	tfl      = lastDmaAccessTime -prevTime;

	tfl_mean[pnt_mean] = tfl;
	pnt_mean++;
	if(pnt_mean>2) pnt_mean =0;

    int mean = median3(tfl_mean[0],tfl_mean[1],tfl_mean[2]);
    if (!mean) mean = 1;

#ifdef USE_SPDIF
	outDmaSpeedScaled =  TIME_SCALE_FACT*N_SIZE/4*DBL_SAMPL/(mean);
#else
	outDmaSpeedScaled =  TIME_SCALE_FACT*N_SIZE*DBL_SAMPL/(mean);
#endif
}
void readDataTim(int offset)
{
#if    (ORDER==0)
	for(int k=0;k<N_SIZE/2;k++)
	{
		struct LR tt =getNextSampleLR(/*k+ASBUF_SIZE/4*/);
		VoiceBuff0[k+offset] = MAX_VOL*(tt.L+(1<<USB_DATA_BITS_H))>>USB_DATA_BITS;
		VoiceBuff1[k+offset] = MAX_VOL*(tt.R+(1<<USB_DATA_BITS_H))>>USB_DATA_BITS;
	}
#endif
#ifdef USE_FLOAT_SIGMA
#if    (ORDER==1)
	float a_scale = MAX_VOL/65536.0f;
	for(int k=0;k<N_SIZE/2;k++)
	{
		struct LR tt =getNextSampleLR(/*k+ASBUF_SIZE/4*/);
		VoiceBuff0[k+offset] =( int)(sigma_delta2(&static_L_channel2,a_scale*tt.L+ MAX_VOL/2));
		VoiceBuff1[k+offset] =( int)(sigma_delta2(&static_R_channel2,a_scale*tt.R+ MAX_VOL/2));
	}
#if    (ORDER==2)
	float a_scale = MAX_VOL/65536.0f;
	for(int k=0;k<N_SIZE/2;k++)
	{
		struct LR tt =getNextSampleLR(/*k+ASBUF_SIZE/4*/);
		VoiceBuff0[k+offset] =( int)(sigma_delta(&static_L_channel,a_scale*tt.L+ MAX_VOL/2));
		VoiceBuff1[k+offset] =( int)(sigma_delta(&static_R_channel,a_scale*tt.R+ MAX_VOL/2));
	}
#endif
#endif
#else //integer sigma
#if (ORDER==1)
	VoiceBuff0[0] = 0;
	VoiceBuff1[0] = 0;
	struct LR tt =getNextSampleLR(/*k+ASBUF_SIZE/4*/);
	//for(int k=0;k<N_SIZE/2;k++)
	//{
	//	struct LR tt =getNextSampleLR(/*k+ASBUF_SIZE/4*/);
	//	VoiceBuff0[k+offset] =sigma_delta_SCALED(&static_L_channel_SCALED,((MAX_VOL)*(tt.L+(1<<USB_DATA_BITS_H)))>>(USB_DATA_BITS-SIGMA_BITS));
	//	VoiceBuff1[k+offset] =sigma_delta_SCALED(&static_R_channel_SCALED,((MAX_VOL)*(tt.R+(1<<USB_DATA_BITS_H)))>>(USB_DATA_BITS-SIGMA_BITS));
	//}

#endif
#if (ORDER==2)
	for(int k=0;k<N_SIZE/2;k++)
	{
		struct LR tt =getNextSampleLR(/*k+ASBUF_SIZE/4*/);
		VoiceBuff0[k+offset] =sigma_delta2_SCALED(&static_L_channel2_SCALED,(MAX_VOL*(tt.L+(1<<USB_DATA_BITS_H)))>>(USB_DATA_BITS-SIGMA_BITS));
		VoiceBuff1[k+offset] =sigma_delta2_SCALED(&static_R_channel2_SCALED,(MAX_VOL*(tt.R+(1<<USB_DATA_BITS_H)))>>(USB_DATA_BITS-SIGMA_BITS));
	}
#endif
#endif
}
void TIM1_TC1()
{

    checkTime();
    /*
    if(UsbSamplesAvail > N_SIZE/2)
    {
    	UsbSamplesAvail -= N_SIZE/2;
    }
    else
    {
    	UsbSamplesAvail = 0;
    }
    */
	TransferComplete_CallBack_FS_Counter++;
	readDataTim(N_SIZE/2);
}
void TIM1_HT1()
{
	HalfTransfer_CallBack_FS_Counter++;
    if(UsbSamplesAvail > N_SIZE/2)
    {
    	UsbSamplesAvail -= N_SIZE/2;
    }
    else
    {
    	UsbSamplesAvail = 0;
    }
    readDataTim(0);
}


//http://www.hardwarebook.info/S/PDIF
//https://sigrok.org/wiki/Protocol_decoder:Spdif
//uint8_t chanel_bit[SPDIF_FRAMES] = {0,0,1,0,0,0};
uint8_t chanel_bit[SPDIF_FRAMES] = {0,0,0,0,0,0};
int     last_phase = 0;

uint16_t bitTable[256];
void makePTable()
{
	for(int k=0;k<256;k++)
	{
		int flag = 0;
		uint16_t num =0;
		for(int bit=0;bit<8;bit++)
		{
			int ind = 15-bit*2;
			if(k&(1<<bit))
			{
				num |= (!flag)<<(ind);
				num |=   flag <<(ind-1);
			}
			else
			{
				flag= !flag;
				num |= flag<<(ind);
				num |= flag<<(ind-1);
			}
		}
		bitTable[k] = num;
	}
}
#define MARKER_B 0b1110100011001100
#define MARKER_M 0b1110001011001100
#define MARKER_W 0b1110010011001100
#define VALIDITY     (28u)
#define SUB     	 (29u)
#define CH           (30u)
#define PARITY       (31u)
uint32_t parity(uint32_t val)
{
	return __builtin_parity(val);
}
void HAL_I2S_TxCpltCallback(I2S_HandleTypeDef *hi2s)
{
    checkTime();
	int FACTOR= 1;
#ifdef USE_SPDIF
	FACTOR= 4;
#endif
    if(UsbSamplesAvail > N_SIZE/2/FACTOR)
    {
    	UsbSamplesAvail -= N_SIZE/2/FACTOR;
    }
    else
    {
    	UsbSamplesAvail = 0;
    }



	TransferComplete_CallBack_FS_Counter++;
#ifdef USE_I2S
    struct LR * bfr = ((struct LR *) baudio_buffer)+ASBUF_SIZE/4;
	for(int k=0;k<ASBUF_SIZE/4;k++)
	{
		bfr[k] = getNextSampleLR(/*k+ASBUF_SIZE/4*/);

	}
#endif
#ifdef USE_SPDIF
	uint16_t *bbuf = (uint16_t *)&baudio_buffer[ASBUF_SIZE/2];
	for(int k=0;k<SPDIF_FRAMES/2;k++)
	{
		struct LR lr = getNextSampleLR(/*k*/);
//		uint32_t lv = ((int)(lr.L)+(1<<USB_DATA_BITS_H))<<(26-USB_DATA_BITS_H);
//		uint32_t rv = ((int)(lr.R)+(1<<USB_DATA_BITS_H))<<(26-USB_DATA_BITS_H);
		uint16_t L = lr.L;
		uint16_t R = lr.R;

		uint32_t lv = ((uint32_t)(L))<<(27-USB_DATA_BITS_H);
		uint32_t rv = ((uint32_t)(R))<<(27-USB_DATA_BITS_H);

		lv |= (chanel_bit[k+SPDIF_FRAMES/2]<<CH)+(0<<VALIDITY)+(0<<SUB);
		lv |= parity(lv)<<PARITY;
		//b
		{
			uint16_t nm0 = MARKER_M;
			if(last_phase) nm0 = ~nm0;
			last_phase = nm0 & 1;
			uint16_t nm1 = bitTable[(lv>>(8))&0xff];
			if(last_phase) nm1 = ~nm1;
			last_phase = nm1 & 1;
			uint16_t nm2 = bitTable[(lv>>(16))&0xff];
			if(last_phase) nm2 = ~nm2;
			last_phase = nm2 & 1;
			uint16_t nm3 = bitTable[(lv>>(24))&0xff];
			if(last_phase) nm3 = ~nm3;
			last_phase = nm3 & 1;

			bbuf[k*8+0] = nm0;
			bbuf[k*8+1] = nm1;
			bbuf[k*8+2] = nm2;
			bbuf[k*8+3] = nm3;
		}

		rv |= (chanel_bit[k+SPDIF_FRAMES/2]<<CH)+(0<<VALIDITY)+(0<<SUB);
		rv |= parity(rv)<<PARITY;
		{
			uint16_t nm0 = MARKER_W;
			if(last_phase) nm0 = ~nm0;
			last_phase = nm0 & 1;
			uint16_t nm1 = bitTable[(rv>>(8))&0xff];
			if(last_phase) nm1 = ~nm1;
			last_phase = nm1 & 1;
			uint16_t nm2 = bitTable[(rv>>(16))&0xff];
			if(last_phase) nm2 = ~nm2;
			last_phase = nm2 & 1;
			uint16_t nm3 = bitTable[(rv>>(24))&0xff];
			if(last_phase) nm3 = ~nm3;
			last_phase = nm3 & 1;

			bbuf[k*8+4] = nm0;
			bbuf[k*8+5] = nm1;
			bbuf[k*8+6] = nm2;
			bbuf[k*8+7] = nm3;
		}
	}
#endif

}

//void HAL_I2S_TxHalfCpltCallback(I2S_HandleTypeDef *hi2s)
void HAL_I2S_TxHalfCpltCallback(I2S_HandleTypeDef *hi2s)
{
	int FACTOR= 1;
#ifdef USE_SPDIF
	FACTOR= 4;
#endif
    if(UsbSamplesAvail > N_SIZE/2/FACTOR)
    {
    	UsbSamplesAvail -= N_SIZE/2/FACTOR;
    }
    else
    {
    	UsbSamplesAvail = 0;
    }

    HalfTransfer_CallBack_FS_Counter++;
#ifdef USE_I2S
	struct LR * bfr = ((struct LR *) baudio_buffer);
	for(int k=0;k<ASBUF_SIZE/4;k++)
		bfr[k] = getNextSampleLR(/*k*/);
#endif
#ifdef USE_SPDIF
	uint16_t *bbuf = (uint16_t *)&baudio_buffer[0];
	for(int k=0;k<SPDIF_FRAMES/2;k++)
	{
		struct LR lr = getNextSampleLR(/*k*/);
//		uint32_t lv = ((int)(lr.L)+(1<<USB_DATA_BITS_H))<<(26-USB_DATA_BITS_H);
//		uint32_t rv = ((int)(lr.R)+(1<<USB_DATA_BITS_H))<<(26-USB_DATA_BITS_H);
		uint16_t L = lr.L;
		uint16_t R = lr.R;

		uint32_t lv = ((uint32_t)(L))<<(27-USB_DATA_BITS_H);
		uint32_t rv = ((uint32_t)(R))<<(27-USB_DATA_BITS_H);

		lv |= (chanel_bit[k]<<CH)+(0<<VALIDITY)+(0<<SUB);
		lv |= parity(lv)<<PARITY;
		//b
		{
			uint16_t nm0 = k==0?MARKER_B:MARKER_M;
			if(last_phase) nm0 = ~nm0;
			last_phase = nm0 & 1;
			uint16_t nm1 = bitTable[(lv>>(8))&0xff];
			if(last_phase) nm1 = ~nm1;
			last_phase = nm1 & 1;
			uint16_t nm2 = bitTable[(lv>>(16))&0xff];
			if(last_phase) nm2 = ~nm2;
			last_phase = nm2 & 1;
			uint16_t nm3 = bitTable[(lv>>(24))&0xff];
			if(last_phase) nm3 = ~nm3;
			last_phase = nm3 & 1;

			bbuf[k*8+0] = nm0;
			bbuf[k*8+1] = nm1;
			bbuf[k*8+2] = nm2;
			bbuf[k*8+3] = nm3;
		}

		rv |= (chanel_bit[k]<<CH)+(0<<VALIDITY)+(0<<SUB);
		rv |= parity(rv)<<PARITY;
		{
			uint16_t nm0 = MARKER_W;
			if(last_phase) nm0 = ~nm0;
			last_phase = nm0 & 1;
			uint16_t nm1 = bitTable[(rv>>(8))&0xff];
			if(last_phase) nm1 = ~nm1;
			last_phase = nm1 & 1;
			uint16_t nm2 = bitTable[(rv>>(16))&0xff];
			if(last_phase) nm2 = ~nm2;
			last_phase = nm2 & 1;
			uint16_t nm3 = bitTable[(rv>>(24))&0xff];
			if(last_phase) nm3 = ~nm3;
			last_phase = nm3 & 1;

			bbuf[k*8+4] = nm0;
			bbuf[k*8+5] = nm1;
			bbuf[k*8+6] = nm2;
			bbuf[k*8+7] = nm3;
		}
	}
#endif

	//HalfTransfer_CallBack_FS();
}


void HAL_I2S_ErrorCallback(I2S_HandleTypeDef *hi2s)
{
	printf("HAL_I2S_ErrorCallback !!!!!!!!!!!!!!!\n");
  //BSP_AUDIO_OUT_Error_CallBack();
}


#define PI 3.14159265358979323846
#define TAU (2.0 * PI)


/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_I2S2_Init();
  MX_SPI3_Init();
  MX_TIM1_Init();
  MX_TIM3_Init();
  MX_USB_DEVICE_Init();
  MX_TIM2_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
  //0 1 2 3 0

  printf("\nStart program\n");

  HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);
  cleanAudioBuffer();
  TIM3->PSC  = (int)(SYS_CLK_MHZ*1000000.0f/TIMER_CLOCK_FREQ)-1;
  HAL_TIM_Base_Start(&htim3);
  HAL_StatusTypeDef stat;

#ifdef  USE_PWM
  LL_TIM_DisableAllOutputs(TIM1);
  init_timers();
  HAL_Delay(100);
  LL_TIM_EnableAllOutputs(TIM1);
#endif
#ifdef  USE_I2S
  stat = HAL_I2S_Transmit_DMA(&hi2s2, baudio_buffer,ASBUF_SIZE);
  HAL_Delay(100);
#endif
#ifdef  USE_SPDIF
  makePTable();
  stat = HAL_I2S_Transmit_DMA(&hi2s2, baudio_buffer,ASBUF_SIZE);
  HAL_Delay(100);
#endif
  printf("Start Speed %x\n",readSpeedXScaled);
  printf("Freq  = %f,sSpeed= %x %04d %04d %04d speed=%x\n",SYS_CLK_MHZ*1000000.0/(MAX_VOL),sForcedSpeed,HalfTransfer_CallBack_FS_Counter,TransferComplete_CallBack_FS_Counter,AUDIO_PeriodicTC_FS_Counter,readSpeedXScaled);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  //int timcnt =0;
  int ko = 0;
  printf("freq = %d hz \n",frequency);

  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  if(HAL_GetTick()/204!=ko)
	  {
		  ko = HAL_GetTick()/204;
		  // printf("%04d %04d %04d %04d %04d\n",HalfTransfer_CallBack_FS_Counter,TransferComplete_CallBack_FS_Counter,AUDIO_PeriodicTC_FS_Counter,AUDIO_OUT_Play_Counter,AUDIO_OUT_ChangeBuffer_Counter);
		  //printf("MAX_VOL = %d INSpeed=%0.01f Hz outSpeed=%0.01f Hz %x Delta %d SMPInH %d elaps = %d mS,ForcedSpeed = %x  %04d PLLspeed = %x %01d %07d s=%d \n",MAX_VOL,inputSpeed,(TIMER_CLOCK_FREQ*(float)outDmaSpeedScaled)/TIME_SCALE_FACT,outDmaSpeedScaled,appDistErr,samplesInBuffH,elapsed_time_ticks,sForcedSpeed,AUDIO_PeriodicTC_FS_Counter,readSpeedXScaled,HAL_GPIO_ReadPin(KEY_GPIO_Port,KEY_Pin),TIM2->CNT,UsbSamplesAvail);
		  uint32_t out = frequency;
		  printf("freq = %d Hz \n",out/100);
	  }
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2S2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2S2_Init(void)
{

  /* USER CODE BEGIN I2S2_Init 0 */

  /* USER CODE END I2S2_Init 0 */

  /* USER CODE BEGIN I2S2_Init 1 */

  /* USER CODE END I2S2_Init 1 */
  hi2s2.Instance = SPI2;
  hi2s2.Init.Mode = I2S_MODE_MASTER_TX;
  hi2s2.Init.Standard = I2S_STANDARD_PHILIPS;
  hi2s2.Init.DataFormat = I2S_DATAFORMAT_16B;
  hi2s2.Init.MCLKOutput = I2S_MCLKOUTPUT_DISABLE;
  hi2s2.Init.AudioFreq = I2S_AUDIOFREQ_192K;
  hi2s2.Init.CPOL = I2S_CPOL_LOW;
  hi2s2.Init.ClockSource = I2S_CLOCK_PLL;
  hi2s2.Init.FullDuplexMode = I2S_FULLDUPLEXMODE_DISABLE;
  if (HAL_I2S_Init(&hi2s2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2S2_Init 2 */

  /* USER CODE END I2S2_Init 2 */

}

/**
  * @brief SPI3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI3_Init(void)
{

  /* USER CODE BEGIN SPI3_Init 0 */

  /* USER CODE END SPI3_Init 0 */

  /* USER CODE BEGIN SPI3_Init 1 */

  /* USER CODE END SPI3_Init 1 */
  /* SPI3 parameter configuration*/
  hspi3.Instance = SPI3;
  hspi3.Init.Mode = SPI_MODE_MASTER;
  hspi3.Init.Direction = SPI_DIRECTION_2LINES;
  hspi3.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi3.Init.CLKPolarity = SPI_POLARITY_HIGH;
  hspi3.Init.CLKPhase = SPI_PHASE_2EDGE;
  hspi3.Init.NSS = SPI_NSS_SOFT;
  hspi3.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi3.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi3.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi3.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi3.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI3_Init 2 */

  /* USER CODE END SPI3_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  LL_TIM_InitTypeDef TIM_InitStruct = {0};
  LL_TIM_OC_InitTypeDef TIM_OC_InitStruct = {0};
  LL_TIM_BDTR_InitTypeDef TIM_BDTRInitStruct = {0};

  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* Peripheral clock enable */
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_TIM1);

  /* TIM1 DMA Init */

  /* TIM1_CH1 Init */
  LL_DMA_SetChannelSelection(DMA2, LL_DMA_STREAM_1, LL_DMA_CHANNEL_6);

  LL_DMA_SetDataTransferDirection(DMA2, LL_DMA_STREAM_1, LL_DMA_DIRECTION_MEMORY_TO_PERIPH);

  LL_DMA_SetStreamPriorityLevel(DMA2, LL_DMA_STREAM_1, LL_DMA_PRIORITY_HIGH);

  LL_DMA_SetMode(DMA2, LL_DMA_STREAM_1, LL_DMA_MODE_CIRCULAR);

  LL_DMA_SetPeriphIncMode(DMA2, LL_DMA_STREAM_1, LL_DMA_PERIPH_NOINCREMENT);

  LL_DMA_SetMemoryIncMode(DMA2, LL_DMA_STREAM_1, LL_DMA_MEMORY_INCREMENT);

  LL_DMA_SetPeriphSize(DMA2, LL_DMA_STREAM_1, LL_DMA_PDATAALIGN_HALFWORD);

  LL_DMA_SetMemorySize(DMA2, LL_DMA_STREAM_1, LL_DMA_MDATAALIGN_HALFWORD);

  LL_DMA_EnableFifoMode(DMA2, LL_DMA_STREAM_1);

  LL_DMA_SetFIFOThreshold(DMA2, LL_DMA_STREAM_1, LL_DMA_FIFOTHRESHOLD_FULL);

  LL_DMA_SetMemoryBurstxfer(DMA2, LL_DMA_STREAM_1, LL_DMA_MBURST_INC8);

  LL_DMA_SetPeriphBurstxfer(DMA2, LL_DMA_STREAM_1, LL_DMA_PBURST_SINGLE);

  /* TIM1_CH2 Init */
  LL_DMA_SetChannelSelection(DMA2, LL_DMA_STREAM_2, LL_DMA_CHANNEL_6);

  LL_DMA_SetDataTransferDirection(DMA2, LL_DMA_STREAM_2, LL_DMA_DIRECTION_MEMORY_TO_PERIPH);

  LL_DMA_SetStreamPriorityLevel(DMA2, LL_DMA_STREAM_2, LL_DMA_PRIORITY_HIGH);

  LL_DMA_SetMode(DMA2, LL_DMA_STREAM_2, LL_DMA_MODE_CIRCULAR);

  LL_DMA_SetPeriphIncMode(DMA2, LL_DMA_STREAM_2, LL_DMA_PERIPH_NOINCREMENT);

  LL_DMA_SetMemoryIncMode(DMA2, LL_DMA_STREAM_2, LL_DMA_MEMORY_INCREMENT);

  LL_DMA_SetPeriphSize(DMA2, LL_DMA_STREAM_2, LL_DMA_PDATAALIGN_HALFWORD);

  LL_DMA_SetMemorySize(DMA2, LL_DMA_STREAM_2, LL_DMA_MDATAALIGN_HALFWORD);

  LL_DMA_EnableFifoMode(DMA2, LL_DMA_STREAM_2);

  LL_DMA_SetFIFOThreshold(DMA2, LL_DMA_STREAM_2, LL_DMA_FIFOTHRESHOLD_FULL);

  LL_DMA_SetMemoryBurstxfer(DMA2, LL_DMA_STREAM_2, LL_DMA_MBURST_INC8);

  LL_DMA_SetPeriphBurstxfer(DMA2, LL_DMA_STREAM_2, LL_DMA_PBURST_SINGLE);

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  TIM_InitStruct.Prescaler = 0;
  TIM_InitStruct.CounterMode = LL_TIM_COUNTERMODE_UP;
  TIM_InitStruct.Autoreload = 65535;
  TIM_InitStruct.ClockDivision = LL_TIM_CLOCKDIVISION_DIV1;
  TIM_InitStruct.RepetitionCounter = 0;
  LL_TIM_Init(TIM1, &TIM_InitStruct);
  LL_TIM_DisableARRPreload(TIM1);
  LL_TIM_OC_EnablePreload(TIM1, LL_TIM_CHANNEL_CH1);
  TIM_OC_InitStruct.OCMode = LL_TIM_OCMODE_PWM1;
  TIM_OC_InitStruct.OCState = LL_TIM_OCSTATE_DISABLE;
  TIM_OC_InitStruct.OCNState = LL_TIM_OCSTATE_DISABLE;
  TIM_OC_InitStruct.CompareValue = 0;
  TIM_OC_InitStruct.OCPolarity = LL_TIM_OCPOLARITY_HIGH;
  TIM_OC_InitStruct.OCNPolarity = LL_TIM_OCPOLARITY_HIGH;
  TIM_OC_InitStruct.OCIdleState = LL_TIM_OCIDLESTATE_LOW;
  TIM_OC_InitStruct.OCNIdleState = LL_TIM_OCIDLESTATE_LOW;
  LL_TIM_OC_Init(TIM1, LL_TIM_CHANNEL_CH1, &TIM_OC_InitStruct);
  LL_TIM_OC_DisableFast(TIM1, LL_TIM_CHANNEL_CH1);
  LL_TIM_OC_EnablePreload(TIM1, LL_TIM_CHANNEL_CH2);
  LL_TIM_OC_Init(TIM1, LL_TIM_CHANNEL_CH2, &TIM_OC_InitStruct);
  LL_TIM_OC_DisableFast(TIM1, LL_TIM_CHANNEL_CH2);
  LL_TIM_SetTriggerOutput(TIM1, LL_TIM_TRGO_RESET);
  LL_TIM_DisableMasterSlaveMode(TIM1);
  TIM_BDTRInitStruct.OSSRState = LL_TIM_OSSR_DISABLE;
  TIM_BDTRInitStruct.OSSIState = LL_TIM_OSSI_DISABLE;
  TIM_BDTRInitStruct.LockLevel = LL_TIM_LOCKLEVEL_OFF;
  TIM_BDTRInitStruct.DeadTime = 0;
  TIM_BDTRInitStruct.BreakState = LL_TIM_BREAK_DISABLE;
  TIM_BDTRInitStruct.BreakPolarity = LL_TIM_BREAK_POLARITY_HIGH;
  TIM_BDTRInitStruct.AutomaticOutput = LL_TIM_AUTOMATICOUTPUT_DISABLE;
  LL_TIM_BDTR_Init(TIM1, &TIM_BDTRInitStruct);
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOB);
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOA);
  /**TIM1 GPIO Configuration
  PB0   ------> TIM1_CH2N
  PB13   ------> TIM1_CH1N
  PA8   ------> TIM1_CH1
  PA9   ------> TIM1_CH2
  */
  GPIO_InitStruct.Pin = LL_GPIO_PIN_0|LL_GPIO_PIN_13;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_1;
  LL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = LL_GPIO_PIN_8|LL_GPIO_PIN_9;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_OPENDRAIN;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_UP;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_1;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 4294967295;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 10;
  if (HAL_TIM_Encoder_Init(&htim2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 840/4-1;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65535;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream4_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream4_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream4_IRQn);
  /* DMA1_Stream5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream5_IRQn);
  /* DMA2_Stream1_IRQn interrupt configuration */
  NVIC_SetPriority(DMA2_Stream1_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),0, 0));
  NVIC_EnableIRQ(DMA2_Stream1_IRQn);
  /* DMA2_Stream2_IRQn interrupt configuration */
  NVIC_SetPriority(DMA2_Stream2_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),0, 0));
  NVIC_EnableIRQ(DMA2_Stream2_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, LED_Pin|DUMMY_OUT_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(MUTE_OUT_GPIO_Port, MUTE_OUT_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : Button_Pin */
  GPIO_InitStruct.Pin = Button_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(Button_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LED_Pin DUMMY_OUT_Pin */
  GPIO_InitStruct.Pin = LED_Pin|DUMMY_OUT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : MUTE_OUT_Pin */
  GPIO_InitStruct.Pin = MUTE_OUT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(MUTE_OUT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : KEY_Pin */
  GPIO_InitStruct.Pin = KEY_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(KEY_GPIO_Port, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
