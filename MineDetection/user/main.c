/****************************************************************************
* Copyright (C), 2013 �ܶ�Ƕ��ʽ������ www.ourstm.net
*											    
* �������� �ܶ���STM32������V2,2.1,V3,V5,MINI�ϵ���ͨ��           
* QQ: 9191274, ������sun68, Email: sun68@163.com 
* �Ա����̣�ourstm.taobao.com  
* �ܶ�����̳��www.ourstm.net  
*
* �ļ���: main.c
* ���ݼ���:
*       ��������ʾ����3��TFT������ʾһ��16λɫͼƬ������ͼƬ��͸������������ͬ��ʾ������ַ���
*       ͼƬ��С��Χ400X240֮�ڡ� �ַ���ȡģ�ߴ磺��ɫ400X240 ֮�� ȡģX�᳤��Ϊ8������������
*       ͼƬȡģ�����img2lcd
*       �ַ�ȡģ�����ZIMO3
*
* �ļ���ʷ:
* �汾��  ����       ����    ˵��
* v0.1    2011-12-5 sun68  �������ļ�
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

#define ADC1_DR_Address    ((u32)0x4001244C)	 			  //ƫ����0x4C

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

/////////////������////////////////////////////
int main(void)	
{  
  vu16 impedance,Vh; 

  RCC_Configuration();   					 //ϵͳʱ������Ϊ72MHz
  FSMC_LCD_Init();							 //FSMC��������		FSMC��stm32��һ���ӿ�
  Usart1_Init();		             	     //����1��ʼ��
  NVIC_Configuration();						 //NVIC�жϳ�ʼ��
  GPIO_Configuration();  					 //led��ʼ��
  LCD_Init();								 //Һ����ʼ��	
  ADC_Configuration();				 //ADC��ʼ��
 
  while (1)
  {		
    //ADC_ConvertedValue = ADC_GetConversionValue(ADC1);
	analog_quantity1 = ADC_ConvertedValue[0];		//��ȡ16��ͨ�� �¶ȴ�������ģ����																	//ת��Ϊ��ѹֵ
	temperature = ((1.43-(analog_quantity1*3.3/4095))/0.0043)+ 25 ;      //�¶�(��C) = {(V25 - VSENSE) / Avg_Slope} + 25 
	
	
	analog_quantity2 = ADC_ConvertedValue[1];	//��ȡ7��ͨ�� PA7��ģ����
	Vh = analog_quantity2*3.3/4095;	 //��ѹVh
	impedance = (51000*5)/Vh-51000;  //�迹Rh		Rf = 51K��  Rh=Rf*Vcc/Vh-Rf;
	humidity = get_31HUMI_vule(impedance);	//����ʪ��
			
    
  

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


 

		USART_OUT(USART1,"\r\nThe current TempAnalog = %d , temperature = %d , HumiAnalog = %d , humidity = %d��people = %d  \r\n", analog_quantity1, temperature, analog_quantity2, humidity,people);
		LCD_test(temperature,humidity,people);	//��LCD_test�����¶ȡ�ʪ��	��Ա 
  }
}

/****************************************************************************
* ��    �ƣ�void ADC_Configuration(void)
* ��    �ܣ�ADC ���ú���
* ��ڲ�������
* ���ڲ�������
* ˵    ����
* ���÷�����
****************************************************************************/ 
void ADC_Configuration(void)
{
	ADC_InitTypeDef ADC_InitStructure;
	GPIO_InitTypeDef GPIO_InitStructure;
	DMA_InitTypeDef DMA_InitStructure;

    //����ADģ������˿�Ϊ���� 1·AD ����ͨ��
  	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7;
  	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;	 //ģ����  ����	
  	GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);																																																																																																																																																																																																																																																																																																																																														  
   /* Enable ADC1 and GPIOC clock */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1 , ENABLE);	
  
	/* DMA channel1 configuration ----------------------------------------------*/
	//ʹ��DMA
	
	DMA_DeInit(DMA1_Channel1);
	DMA_InitStructure.DMA_PeripheralBaseAddr = ADC1_DR_Address;			            //DMAͨ��1�ĵ�ַ 
	DMA_InitStructure.DMA_MemoryBaseAddr = (u32)&ADC_ConvertedValue;	            //DMA���͵�ַ
											//DR��ADC_ConvertedValue�DMAͨ������ȫ�ֱ���ADC_ConvertedValue��ֵ
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;					            //���ͷ���
								//���� ����ΪԴ
	DMA_InitStructure.DMA_BufferSize = 2;								            //�����ڴ��С��2��16λ
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;	 			//���費����
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;				            //�����ڴ��ַ����
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;		//ADC1ת����������16λ �����-2�ֽ�
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;				//���͵�Ŀ�ĵ�ַ��16λ���
	DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;									//ѭ��
	DMA_InitStructure.DMA_Priority = DMA_Priority_High;
	DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
	DMA_Init(DMA1_Channel1, &DMA_InitStructure);
     
	/* ����DMA1ͨ��1��������ж� */
	//DMA_ITConfig(DMA1_Channel1,DMA_IT_TC, ENABLE);


	//ʹ��DMAͨ��1
	DMA_Cmd(DMA1_Channel1, ENABLE);

	//ADC����
	/* ADCת��ʱ�䣺 �� STM32F103xx��ǿ�Ͳ�Ʒ��ʱ��Ϊ56MHzʱΪ1��s(ʱ��Ϊ72MHzΪ1.17��s)
	ADC������Χ0-3.3V    */
	RCC_ADCCLKConfig(RCC_PCLK2_Div6);                   //����ADC��ʱ��Ϊ72MHZ/6=12M 

	ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;	//ADC1�����ڶ���ģʽ
	ADC_InitStructure.ADC_ScanConvMode = ENABLE;		//ģ��ת��������ɨ��ģʽ����ͨ�������ǵ��Σ���ͨ����ģʽ
	ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;	//ģ��ת������������ģʽ�����ǵ���ģʽ
	ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;//ת��������������ⲿ�������� 0x000E0000    E:1110   ����111�����������
	ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;//ADC�����Ҷ���
	ADC_InitStructure.ADC_NbrOfChannel = 2;               //�涨��˳����й���ת����ADCͨ������Ŀ�������Ŀ��ȡֵ��Χ��1��16
												   //ͨ��16
	ADC_Init(ADC1, &ADC_InitStructure);
	
	/* ADC1 regular channels configuration [����ģʽͨ������]*/ 

	//ADC1 ����ͨ������
  	ADC_RegularChannelConfig(ADC1, ADC_Channel_16, 1, ADC_SampleTime_55Cycles5);	  //ͨ��11����ʱ�� 55.5����
	ADC_RegularChannelConfig(ADC1, ADC_Channel_7, 2, ADC_SampleTime_55Cycles5);		 
							   //ͨ��11����Ϊ������1��

	//ʹ��ADC1 DMA 
	ADC_DMACmd(ADC1, ENABLE);

	ADC_TempSensorVrefintCmd(ENABLE); //�����ڲ��¶ȴ�����

	//ʹ��ADC1
	ADC_Cmd(ADC1, ENABLE);	
	
	// ��ʼ��ADC1У׼�Ĵ���
	ADC_ResetCalibration(ADC1);
	//���ADC1У׼�Ĵ�����ʼ���Ƿ����
	while(ADC_GetResetCalibrationStatus(ADC1));
	
	//��ʼУ׼ADC1
	ADC_StartCalibration(ADC1);
	//����Ƿ����У׼
	while(ADC_GetCalibrationStatus(ADC1));
	
	//ADC1ת������
	ADC_SoftwareStartConvCmd(ADC1, ENABLE);	 
}

//����ϵͳʱ�ӣ�ͨ��9��Ƶ����ϵͳʱ������Ϊ72MHz
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

//NVIC�ж�����
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

//led 123��ʼ��
void GPIO_Configuration(void)
{		
	  GPIO_InitTypeDef GPIO_InitStructure;
	  //��ʼ��led1 2 3
	  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5; //LED1
      GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	  GPIO_Init(GPIOB, &GPIO_InitStructure);			
	  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6|GPIO_Pin_3; //LED2��3
	  GPIO_Init(GPIOD, &GPIO_InitStructure);	

	 //hongwaixian
	  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;//��Ӧ�ߵ�ƽ��������������
      GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
     GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
     GPIO_Init(GPIOD,&GPIO_InitStructure);
}

//////////FSMC �ӿ�����///////////////////////////////////////////
void FSMC_LCD_Init(void)
{
  FSMC_NORSRAMInitTypeDef  FSMC_NORSRAMInitStructure;
  FSMC_NORSRAMTimingInitTypeDef  p;	
  GPIO_InitTypeDef GPIO_InitStructure;	    
  //ʹ��FSMC����ʱ��
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_FSMC, ENABLE);   
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB | RCC_APB2Periph_GPIOC |
                         RCC_APB2Periph_GPIOD | RCC_APB2Periph_GPIOE , ENABLE);  
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13; 			  //LCD�������
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_Init(GPIOD, &GPIO_InitStructure);

  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1 ; 	 		  //LCD��λ
  GPIO_Init(GPIOE, &GPIO_InitStructure);   	   	
  // ���ö˿�ΪFSMC�ӿ� FSMC-D0--D15
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
  //FSMC NE1  LCDƬѡ
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7; 
  GPIO_Init(GPIOD, &GPIO_InitStructure);
  
  //FSMC RS---LCDָ�� ָ��/����	�л�
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11 ; 
  GPIO_Init(GPIOD, &GPIO_InitStructure); 	
  GPIO_SetBits(GPIOD, GPIO_Pin_13);			           //LCD�����
  
  
  //FSMC�ӿ���������
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


// ++++++++++++++++TFT ��λ����
void lcd_rst(void){
	GPIO_ResetBits(GPIOE, GPIO_Pin_1);	      //PE1 ΪLCD ��λ�ź�
    Delay(0xAFFFFf);					   
    GPIO_SetBits(GPIOE, GPIO_Pin_1 );		 	 
	Delay(0xAFFFFf);	
}

void Delay(__IO uint32_t nCount)
{
  for(; nCount != 0; nCount--);
}

/****************************************************************************
* ��    �ƣ�void Usart1_Init(void)
* ��    �ܣ�����1��ʼ������
* ��ڲ�������
* ���ڲ�������
* ˵    ����
* ���÷������� 
****************************************************************************/
void Usart1_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;
  USART_InitTypeDef USART_InitStructure;
 
  RCC_APB2PeriphClockCmd( RCC_APB2Periph_USART1 , ENABLE);	 //ʹ�ܴ���1ʱ��


  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;	         		 //USART1 TX
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;    		 //�����������
  GPIO_Init(GPIOA, &GPIO_InitStructure);		    		 //A�˿� 

  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;	         	 //USART1 RX
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;   	 //���ÿ�©����
  GPIO_Init(GPIOA, &GPIO_InitStructure);		         	 //A�˿� 

  USART_InitStructure.USART_BaudRate = 115200;						//����115200bps
  USART_InitStructure.USART_WordLength = USART_WordLength_8b;		//����λ8λ
  USART_InitStructure.USART_StopBits = USART_StopBits_1;			//ֹͣλ1λ
  USART_InitStructure.USART_Parity = USART_Parity_No;				//��У��λ
  USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;   //��Ӳ������
  USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;					//�շ�ģʽ

  /* Configure USART1 */
  USART_Init(USART1, &USART_InitStructure);							//���ô��ڲ�������   
  
  USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);  //δ֪
   /* Enable the USART1 */
  USART_Cmd(USART1, ENABLE);	
  
}
/******************************************************
		��������ת�ַ�������
        char *itoa(int value, char *string, int radix)
		radix=10 ��ʾ��10����	��ʮ���ƣ�ת�����Ϊ0;  

	    ����d=-379;
		ִ��	itoa(d, buf, 10); ��
		
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
    	//�����Ĵ��������ݻ��浽���ջ�������
		RxBuffer1[0] = USART_ReceiveData(USART1);   	
		//�����ջ�����������ת�����ͻ�������׼��ת��	
		TxBuffer1[0] = RxBuffer1[0]; 	
	  	rec_f=1;//���ճɹ���־
		USART_SendData(USART1, 8);// ����һ���ַ�	  
    }
 
}

/****************************************************************************
* ��    �ƣ�void USART_OUT(USART_TypeDef* USARTx, uint8_t *Data,...)
* ��    �ܣ���ʽ�������������
* ��ڲ�����USARTx:  ָ������
			Data��   ��������
			...:     ��������
* ���ڲ�������
* ˵    ������ʽ�������������
        	"\r"	�س���	   USART_OUT(USART1, "abcdefg\r")   
			"\n"	���з�	   USART_OUT(USART1, "abcdefg\r\n")
			"%s"	�ַ���	   USART_OUT(USART1, "�ַ����ǣ�%s","abcdefg")
			"%d"	ʮ����	   USART_OUT(USART1, "a=%d",10)
* ���÷������� 
****************************************************************************/
void USART_OUT(USART_TypeDef* USARTx, uint8_t *Data,...){ 

	const char *s;
    int d;
   
    char buf[16];
    va_list ap;
    va_start(ap, Data);

	while(*Data!=0){				                          //�ж��Ƿ񵽴��ַ���������
		if(*Data==0x5c){									  //'\'
			switch (*++Data){
				case 'r':							          //�س���
					USART_SendData(USARTx, 0x0d);	   

					Data++;
					break;
				case 'n':							          //���з�
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
				case 's':										  //�ַ���
                	s = va_arg(ap, const char *);
                	for ( ; *s; s++) {
                    	USART_SendData(USARTx,*s);
						while(USART_GetFlagStatus(USARTx, USART_FLAG_TC)==RESET);
                	}
					Data++;
                	break;
            	case 'd':										  //ʮ����
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
  //�ֶμ���ʪ��ֵ 
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

 
 
 

