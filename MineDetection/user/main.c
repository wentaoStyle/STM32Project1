/****************************************************************************
* Copyright (C), 2013 奋斗嵌入式工作室 www.ourstm.net
*											    
* 本例程在 奋斗版STM32开发板V2,2.1,V3,V5,MINI上调试通过           
* QQ: 9191274, 旺旺：sun68, Email: sun68@163.com 
* 淘宝店铺：ourstm.taobao.com  
* 奋斗板论坛：www.ourstm.net  
*
* 文件名: main.c
* 内容简述:
*       本例程演示了在3寸TFT屏上显示一副16位色图片，并在图片上透明叠加两个不同显示方向的字符串
*       图片大小范围400X240之内。 字符串取模尺寸：单色400X240 之内 取模X轴长度为8的整数倍数。
*       图片取模软件：img2lcd
*       字符取模软件：ZIMO3
*
* 文件历史:
* 版本号  日期       作者    说明
* v0.1    2011-12-5 sun68  创建该文件
*/
//Vh = analog_quantity1*3.3/4095
//impedance = (51000*5)/Vh-51000;//Rf = 51K (Rh+Rf)/Rf=Vcc/Vh;  Rh=Rf*Vcc/Vh-Rf;

/* Includes ------------------------------------------------------------------*/
#include "fsmc_sram.h" 
#include "stm32f10x_fsmc.h"
#include "stm32f10x_adc.h"
#include "misc.h"
#include "stm32f10x_dma.h"
#include "stm32f10x_usart.h"


#include  <stdarg.h>

#define ADC1_DR_Address    ((u32)0x4001244C)	 			  //偏移量0x4C

int rec_f=0;
unsigned char TxBuffer1[12];
unsigned char RxBuffer1[12];
unsigned char ReceivedData;
 

vu16 get_31HUMI_vule(vu16 impedance);

/* Private function prototypes -----------------------------------------------*/
vu16 ADC_ConvertedValue[2],analog_quantity1,analog_quantity2;
unsigned int temperature,humidity,people,i;

void NVIC_Configuration(void);
//void USART1_IRQHandler(void);
void RCC_Configuration(void); 
void ADC_Configuration(void); 
void GPIO_Configuration(void);
void Usart1_Init(void);
void USART_OUT(USART_TypeDef* USARTx, uint8_t *Data,...); 
extern void LCD_Init(void);
extern void LCD_test(unsigned int temperature,unsigned int humidity,int people);
void lcd_rst(void);
void Delay(__IO uint32_t nCount);

/////////////主函数////////////////////////////
int main(void)	
{  
  vu16 impedance,Vh; 

  RCC_Configuration();   					 //系统时钟配置为72MHz
  FSMC_LCD_Init();							 //FSMC总线配置		FSMC是stm32的一个接口
  Usart1_Init();		             	     //串口1初始化
  NVIC_Configuration();						 //NVIC中断初始化
  GPIO_Configuration();  					 //led初始化
  LCD_Init();								 //液晶初始化	
  ADC_Configuration();				 //ADC初始化
 
  while (1)
  {		
    //ADC_ConvertedValue = ADC_GetConversionValue(ADC1);
	analog_quantity1 = ADC_ConvertedValue[0];		//获取16号通道 温度传感器的模拟量																	//转化为电压值
	temperature = ((1.43-(analog_quantity1*3.3/4095))/0.0043)+ 25 ;      //温度(°C) = {(V25 - VSENSE) / Avg_Slope} + 25 
	
	
	analog_quantity2 = ADC_ConvertedValue[1];	//获取7号通道 PA7的模拟量
	Vh = analog_quantity2*3.3/4095;	 //电压Vh
	impedance = (51000*5)/Vh-51000;  //阻抗Rh		Rf = 51KΩ  Rh=Rf*Vcc/Vh-Rf;
	humidity = get_31HUMI_vule(impedance);	//计算湿度
			
    
  

	if(GPIO_ReadInputDataBit(GPIOD,GPIO_Pin_2)){
		people=1;
	 }
	 else {
		people=0;
	}

	 if(humidity >= 40){
	 	GPIO_SetBits(GPIOD,GPIO_Pin_6);
	 }else{
		 GPIO_ResetBits(GPIOD,GPIO_Pin_6) ;
	 }


	 if( people== 1){
	 	GPIO_SetBits(GPIOB,GPIO_Pin_5);
	 }else{
		 GPIO_ResetBits(GPIOB,GPIO_Pin_5) ;
	 }


 

		USART_OUT(USART1,"\r\nThe current TempAnalog = %d , temperature = %d , HumiAnalog = %d , humidity = %d，people = %d  \r\n", analog_quantity1, temperature, analog_quantity2, humidity,people);
		LCD_test(temperature,humidity,people);	//向LCD_test传入温度、湿度	人员 
  }
}

/****************************************************************************
* 名    称：void ADC_Configuration(void)
* 功    能：ADC 配置函数
* 入口参数：无
* 出口参数：无
* 说    明：
* 调用方法：
****************************************************************************/ 
void ADC_Configuration(void)
{
	ADC_InitTypeDef ADC_InitStructure;
	GPIO_InitTypeDef GPIO_InitStructure;
	DMA_InitTypeDef DMA_InitStructure;

    //设置AD模拟输入端口为输入 1路AD 规则通道
  	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7;
  	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;	 //模拟量  区间	
  	GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);																																																																																																																																																																																																																																																																																																																																														  
   /* Enable ADC1 and GPIOC clock */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1 , ENABLE);	
  
	/* DMA channel1 configuration ----------------------------------------------*/
	//使能DMA
	
	DMA_DeInit(DMA1_Channel1);
	DMA_InitStructure.DMA_PeripheralBaseAddr = ADC1_DR_Address;			            //DMA通道1的地址 
	DMA_InitStructure.DMA_MemoryBaseAddr = (u32)&ADC_ConvertedValue;	            //DMA传送地址
											//DR和ADC_ConvertedValue搭建DMA通道，向全局变量ADC_ConvertedValue传值
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;					            //传送方向
								//设置 外设为源
	DMA_InitStructure.DMA_BufferSize = 2;								            //传送内存大小，2个16位
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;	 			//外设不递增
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;				            //传送内存地址递增
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;		//ADC1转换的数据是16位 半个字-2字节
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;				//传送的目的地址是16位宽度
	DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;									//循环
	DMA_InitStructure.DMA_Priority = DMA_Priority_High;
	DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
	DMA_Init(DMA1_Channel1, &DMA_InitStructure);
     
	/* 允许DMA1通道1传输结束中断 */
	//DMA_ITConfig(DMA1_Channel1,DMA_IT_TC, ENABLE);


	//使能DMA通道1
	DMA_Cmd(DMA1_Channel1, ENABLE);

	//ADC配置
	/* ADC转换时间： ─ STM32F103xx增强型产品：时钟为56MHz时为1μs(时钟为72MHz为1.17μs)
	ADC采样范围0-3.3V    */
	RCC_ADCCLKConfig(RCC_PCLK2_Div6);                   //设置ADC的时钟为72MHZ/6=12M 

	ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;	//ADC1工作在独立模式
	ADC_InitStructure.ADC_ScanConvMode = ENABLE;		//模数转换工作在扫描模式（多通道）还是单次（单通道）模式
	ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;	//模数转换工作在连续模式，还是单次模式
	ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;//转换由软件而不是外部触发启动 0x000E0000    E:1110   其中111设置软件触发
	ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;//ADC数据右对齐
	ADC_InitStructure.ADC_NbrOfChannel = 2;               //规定了顺序进行规则转换的ADC通道的数目。这个数目的取值范围是1到16
												   //通道16
	ADC_Init(ADC1, &ADC_InitStructure);
	
	/* ADC1 regular channels configuration [规则模式通道配置]*/ 

	//ADC1 规则通道配置
  	ADC_RegularChannelConfig(ADC1, ADC_Channel_16, 1, ADC_SampleTime_55Cycles5);	  //通道11采样时间 55.5周期
	ADC_RegularChannelConfig(ADC1, ADC_Channel_7, 2, ADC_SampleTime_55Cycles5);		 
							   //通道11配置为规则组1号

	//使能ADC1 DMA 
	ADC_DMACmd(ADC1, ENABLE);

	ADC_TempSensorVrefintCmd(ENABLE); //开启内部温度传感器

	//使能ADC1
	ADC_Cmd(ADC1, ENABLE);	
	
	// 初始化ADC1校准寄存器
	ADC_ResetCalibration(ADC1);
	//检测ADC1校准寄存器初始化是否完成
	while(ADC_GetResetCalibrationStatus(ADC1));
	
	//开始校准ADC1
	ADC_StartCalibration(ADC1);
	//检测是否完成校准
	while(ADC_GetCalibrationStatus(ADC1));
	
	//ADC1转换启动
	ADC_SoftwareStartConvCmd(ADC1, ENABLE);	 
}

//设置系统时钟，通过9倍频，将系统时钟设置为72MHz
void RCC_Configuration(void)
{   
  SystemInit();
  RCC_ADCCLKConfig(RCC_PCLK2_Div6); 
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1 , ENABLE);
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB | RCC_APB2Periph_GPIOC 
  						| RCC_APB2Periph_GPIOD| RCC_APB2Periph_GPIOE , ENABLE);
}											

//NVIC中断设置
void NVIC_Configuration(void)
{
	NVIC_InitTypeDef  NVIC_InitStruct;
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_0);	    
	NVIC_InitStruct.NVIC_IRQChannel = USART1_IRQn;
	NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 1;	 
	NVIC_InitStruct.NVIC_IRQChannelSubPriority = 2;	  
	NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE; 
	NVIC_Init(&NVIC_InitStruct);
}

//led 123初始化
void GPIO_Configuration(void)
{		
	  GPIO_InitTypeDef GPIO_InitStructure;
	  //初始化led1 2 3
	  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5; //LED1
      GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	  GPIO_Init(GPIOB, &GPIO_InitStructure);			
	  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6|GPIO_Pin_3; //LED2、3
	  GPIO_Init(GPIOD, &GPIO_InitStructure);	

	 //hongwaixian
	  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;//感应高电平，是输入下拉吧
      GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
     GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
     GPIO_Init(GPIOD,&GPIO_InitStructure);
}

//////////FSMC 接口设置///////////////////////////////////////////
void FSMC_LCD_Init(void)
{
  FSMC_NORSRAMInitTypeDef  FSMC_NORSRAMInitStructure;
  FSMC_NORSRAMTimingInitTypeDef  p;	
  GPIO_InitTypeDef GPIO_InitStructure;	    
  //使能FSMC外设时钟
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_FSMC, ENABLE);   
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB | RCC_APB2Periph_GPIOC |
                         RCC_APB2Periph_GPIOD | RCC_APB2Periph_GPIOE , ENABLE);  
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13; 			  //LCD背光控制
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_Init(GPIOD, &GPIO_InitStructure);

  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1 ; 	 		  //LCD复位
  GPIO_Init(GPIOE, &GPIO_InitStructure);   	   	
  // 复用端口为FSMC接口 FSMC-D0--D15
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_4 | GPIO_Pin_5 |
                                GPIO_Pin_8 | GPIO_Pin_9 | GPIO_Pin_10 | GPIO_Pin_14 | 
                                GPIO_Pin_15;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
  GPIO_Init(GPIOD, &GPIO_InitStructure);   
   
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7 | GPIO_Pin_8 | GPIO_Pin_9 | GPIO_Pin_10 | 
                                GPIO_Pin_11 | GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_14 | 
                                GPIO_Pin_15;
  GPIO_Init(GPIOE, &GPIO_InitStructure);    
  //FSMC NE1  LCD片选
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7; 
  GPIO_Init(GPIOD, &GPIO_InitStructure);
  
  //FSMC RS---LCD指令 指令/数据	切换
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11 ; 
  GPIO_Init(GPIOD, &GPIO_InitStructure); 	
  GPIO_SetBits(GPIOD, GPIO_Pin_13);			           //LCD背光打开
  
  
  //FSMC接口特性配置
  p.FSMC_AddressSetupTime = 0x02;
  p.FSMC_AddressHoldTime = 0x00;
  p.FSMC_DataSetupTime = 0x05;
  p.FSMC_BusTurnAroundDuration = 0x00;
  p.FSMC_CLKDivision = 0x00;
  p.FSMC_DataLatency = 0x00;
  p.FSMC_AccessMode = FSMC_AccessMode_B;

 
  FSMC_NORSRAMInitStructure.FSMC_Bank = FSMC_Bank1_NORSRAM1;
  FSMC_NORSRAMInitStructure.FSMC_DataAddressMux = FSMC_DataAddressMux_Disable;
  FSMC_NORSRAMInitStructure.FSMC_MemoryType = FSMC_MemoryType_NOR;
  FSMC_NORSRAMInitStructure.FSMC_MemoryDataWidth = FSMC_MemoryDataWidth_16b;
  FSMC_NORSRAMInitStructure.FSMC_BurstAccessMode = FSMC_BurstAccessMode_Disable;
  FSMC_NORSRAMInitStructure.FSMC_WaitSignalPolarity = FSMC_WaitSignalPolarity_Low;
  FSMC_NORSRAMInitStructure.FSMC_WrapMode = FSMC_WrapMode_Disable;
  FSMC_NORSRAMInitStructure.FSMC_WaitSignalActive = FSMC_WaitSignalActive_BeforeWaitState;
  FSMC_NORSRAMInitStructure.FSMC_WriteOperation = FSMC_WriteOperation_Enable;
  FSMC_NORSRAMInitStructure.FSMC_WaitSignal = FSMC_WaitSignal_Disable;
  FSMC_NORSRAMInitStructure.FSMC_ExtendedMode = FSMC_ExtendedMode_Disable;
  FSMC_NORSRAMInitStructure.FSMC_WriteBurst = FSMC_WriteBurst_Disable;
  FSMC_NORSRAMInitStructure.FSMC_ReadWriteTimingStruct = &p;
  FSMC_NORSRAMInitStructure.FSMC_WriteTimingStruct = &p;
 
  FSMC_NORSRAMInit(&FSMC_NORSRAMInitStructure); 		
  /* Enable FSMC Bank1_SRAM Bank */
  FSMC_NORSRAMCmd(FSMC_Bank1_NORSRAM1, ENABLE);  
}


// ++++++++++++++++TFT 复位操作
void lcd_rst(void){
	GPIO_ResetBits(GPIOE, GPIO_Pin_1);	      //PE1 为LCD 复位信号
    Delay(0xAFFFFf);					   
    GPIO_SetBits(GPIOE, GPIO_Pin_1 );		 	 
	Delay(0xAFFFFf);	
}

void Delay(__IO uint32_t nCount)
{
  for(; nCount != 0; nCount--);
}

/****************************************************************************
* 名    称：void Usart1_Init(void)
* 功    能：串口1初始化函数
* 入口参数：无
* 出口参数：无
* 说    明：
* 调用方法：无 
****************************************************************************/
void Usart1_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;
  USART_InitTypeDef USART_InitStructure;
 
  RCC_APB2PeriphClockCmd( RCC_APB2Periph_USART1 , ENABLE);	 //使能串口1时钟


  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;	         		 //USART1 TX
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;    		 //复用推挽输出
  GPIO_Init(GPIOA, &GPIO_InitStructure);		    		 //A端口 

  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;	         	 //USART1 RX
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;   	 //复用开漏输入
  GPIO_Init(GPIOA, &GPIO_InitStructure);		         	 //A端口 

  USART_InitStructure.USART_BaudRate = 115200;						//速率115200bps
  USART_InitStructure.USART_WordLength = USART_WordLength_8b;		//数据位8位
  USART_InitStructure.USART_StopBits = USART_StopBits_1;			//停止位1位
  USART_InitStructure.USART_Parity = USART_Parity_No;				//无校验位
  USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;   //无硬件流控
  USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;					//收发模式

  /* Configure USART1 */
  USART_Init(USART1, &USART_InitStructure);							//配置串口参数函数   
  
  USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);  //未知
   /* Enable the USART1 */
  USART_Cmd(USART1, ENABLE);	
  
}
/******************************************************
		整形数据转字符串函数
        char *itoa(int value, char *string, int radix)
		radix=10 标示是10进制	非十进制，转换结果为0;  

	    例：d=-379;
		执行	itoa(d, buf, 10); 后
		
		buf="-379"							   			  
**********************************************************/
char *itoa(int value, char *string, int radix)
{
    int     i, d;
    int     flag = 0;
    char    *ptr = string;

    /* This implementation only works for decimal numbers. */
    if (radix != 10)
    {
        *ptr = 0;
        return string;
    }

    if (!value)
    {
        *ptr++ = 0x30;
        *ptr = 0;
        return string;
    }

    /* if this is a negative value insert the minus sign. */
    if (value < 0)
    {
        *ptr++ = '-';

        /* Make the value positive. */
        value *= -1;
    }

    for (i = 10000; i > 0; i /= 10)
    {
        d = value / i;

        if (d || flag)
        {
            *ptr++ = (char)(d + 0x30);
            value -= (d * i);
            flag = 1;
        }
    }

    /* Null terminate the string. */
    *ptr = 0;

    return string;

} /* NCL_Itoa */

void USART1_IRQHandler(void) 
{
 	if(USART_GetITStatus(USART1, USART_IT_RXNE) == SET)	  
	{	
    	//将读寄存器的数据缓存到接收缓冲区里
		RxBuffer1[0] = USART_ReceiveData(USART1);   	
		//将接收缓冲器的数据转到发送缓冲区，准备转发	
		TxBuffer1[0] = RxBuffer1[0]; 	
	  	rec_f=1;//接收成功标志
		USART_SendData(USART1, 8);// 发送一个字符	  
    }
 
}

/****************************************************************************
* 名    称：void USART_OUT(USART_TypeDef* USARTx, uint8_t *Data,...)
* 功    能：格式化串口输出函数
* 入口参数：USARTx:  指定串口
			Data：   发送数组
			...:     不定参数
* 出口参数：无
* 说    明：格式化串口输出函数
        	"\r"	回车符	   USART_OUT(USART1, "abcdefg\r")   
			"\n"	换行符	   USART_OUT(USART1, "abcdefg\r\n")
			"%s"	字符串	   USART_OUT(USART1, "字符串是：%s","abcdefg")
			"%d"	十进制	   USART_OUT(USART1, "a=%d",10)
* 调用方法：无 
****************************************************************************/
void USART_OUT(USART_TypeDef* USARTx, uint8_t *Data,...){ 

	const char *s;
    int d;
   
    char buf[16];
    va_list ap;
    va_start(ap, Data);

	while(*Data!=0){				                          //判断是否到达字符串结束符
		if(*Data==0x5c){									  //'\'
			switch (*++Data){
				case 'r':							          //回车符
					USART_SendData(USARTx, 0x0d);	   

					Data++;
					break;
				case 'n':							          //换行符
					USART_SendData(USARTx, 0x0a);	
					Data++;
					break;
				
				default:
					Data++;
				    break;
			}
			
			 
		}
		else if(*Data=='%'){									  //
			switch (*++Data){				
				case 's':										  //字符串
                	s = va_arg(ap, const char *);
                	for ( ; *s; s++) {
                    	USART_SendData(USARTx,*s);
						while(USART_GetFlagStatus(USARTx, USART_FLAG_TC)==RESET);
                	}
					Data++;
                	break;
            	case 'd':										  //十进制
                	d = va_arg(ap, int);
                	itoa(d, buf, 10);
                	for (s = buf; *s; s++) {
                    	USART_SendData(USARTx,*s);
						while(USART_GetFlagStatus(USARTx, USART_FLAG_TC)==RESET);
                	}
					Data++;
                	break;
				default:
					Data++;
				    break;
			}		 
		}
		else USART_SendData(USARTx, *Data++);
		while(USART_GetFlagStatus(USARTx, USART_FLAG_TC)==RESET);
	}
}

vu16 get_31HUMI_vule(vu16 impedance)
{
  vu16 R_Humideal_int = impedance;
  //分段计算湿度值 
  if((R_Humideal_int/1000)>=2600)    R_Humideal_int=(146000-10*(R_Humideal_int/1000))*10/4800; //20-25
  else if(1300<=(R_Humideal_int/1000)&(R_Humideal_int/1000)<2600)  R_Humideal_int=(91000-10*(R_Humideal_int/1000))*10/26000; //25-30
  else if(630<=(R_Humideal_int/1000)&(R_Humideal_int/1000)<1300)  R_Humideal_int=(53200-10*(R_Humideal_int/1000))*10/13400; //30-35
  else if(310<=(R_Humideal_int/1000)&(R_Humideal_int/1000)<630)   R_Humideal_int=(28700-10*(R_Humideal_int/1000))*10/6400; //35-40  
  else if(160<=(R_Humideal_int/1000)&(R_Humideal_int/1000)<310)  R_Humideal_int=(15100-10*(R_Humideal_int/1000))*10/3000; //40-45 
  else if(87<=(R_Humideal_int/1000)&(R_Humideal_int/1000)<160)  R_Humideal_int=(8170-10*(R_Humideal_int/1000))*10/1460; //45-50
  else if(49<=(R_Humideal_int/1000)&(R_Humideal_int/1000)<87)  R_Humideal_int=(4670-10*(R_Humideal_int/1000))*10/760; //50-55
  else if(31<=(R_Humideal_int/1000)&(R_Humideal_int/1000)<49)  R_Humideal_int=(2470-10*(R_Humideal_int/1000))*10/360; //55-60     
  else if(20<=(R_Humideal_int/1000)&(R_Humideal_int/1000)<31)  R_Humideal_int=(1630-10*(R_Humideal_int/1000))*10/220; //60-65  
  else if(13<=(R_Humideal_int/1000)&(R_Humideal_int/1000)<20)  R_Humideal_int=(1110-10*(R_Humideal_int/1000))*10/140; //65-70
  else if(8<=(R_Humideal_int/1000)&(R_Humideal_int/1000)<13)   R_Humideal_int=(7740-100*(R_Humideal_int/1000))*10/920;//70-75
  else if(6<=(R_Humideal_int/1000)&(R_Humideal_int/1000)<8)   R_Humideal_int=(4890-100*(R_Humideal_int/1000))*10/540;//75-80
  //else if(5<=(R_Humideal_int/1000)&(R_Humideal_int/1000)<6)  HUMI_vule=(2010-100*(R_Humideal_int/1000))*10/18;//80-85
  else if((R_Humideal_int/1000)<6)  R_Humideal_int=(388-10*(R_Humideal_int/1000))*10/4;//85-90
  
  R_Humideal_int = R_Humideal_int-30;
  return R_Humideal_int;
}

 
 
 

