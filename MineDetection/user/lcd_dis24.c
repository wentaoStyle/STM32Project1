/****************************************************************************
* Copyright (C), 2011 �ܶ�Ƕ��ʽ������ www.ourstm.net
*
* �������� �ܶ���STM32������V2,2.1,V3,MINI�ϵ���ͨ��           
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


#include "fsmc_sram.h"

#define Bank1_LCD_D    ((uint32_t)0x60020000)    //��ʾ�����ݵ�ַ	  
#define Bank1_LCD_C    ((uint32_t)0x60000000)	 //��ʾ��ָ���ַ

unsigned long color1=63489;
unsigned int peo;

void LCD_Init(void);
void LCD_WR_REG(unsigned int index);
void LCD_WR_CMD(unsigned int index,unsigned int val);
void LCD_WR_Data(unsigned int val);

void LCD_test(unsigned int temperature,unsigned int humidity,unsigned int people);
void LCD_clear(unsigned int p);
void lcd_wr_zf(u16 StartX, u16 StartY, u16 X, u16 Y, u16 Color, u8 Dir, u8 *chr); 
unsigned char *num_pub(unsigned int a);


extern unsigned char zm9[];
extern unsigned char zm8[];
extern unsigned char zm7[];
extern unsigned char zm6[];
extern unsigned char zm5[];
extern unsigned char zm4[];
extern unsigned char zm3[];
extern unsigned char zm2[];
extern unsigned char zm1[];
extern unsigned char zm0[];
extern unsigned char a1[];

extern unsigned char zf2[];
extern unsigned char zf3[];
extern unsigned char zf4[];
extern unsigned char zf5[];	
extern unsigned char zf6[];
extern unsigned char zf7[];
extern unsigned char zf8[];
extern unsigned char pic[];
extern unsigned char a2[];
extern unsigned char a3[];

unsigned int LCD_RD_data(void);
extern void lcd_rst(void);
extern void Delay(__IO uint32_t nCount);


/****************************************************************************
* ��    �ƣ�void lcd_DrawPicture(u16 StartX,u16 StartY, u8 Dir��u8 *pic)
* ��    �ܣ���ָ�����귶Χ��ʾһ��ͼƬ
* ��ڲ�����StartX     ����ʼ����
*           StartY     ����ʼ����
*			Dir		   ͼ����ʾ����       
*           pic        ͼƬͷָ��
* ���ڲ�������
* ˵    ����ͼƬȡģ��ʽΪˮƽɨ�裬16λ��ɫģʽ  ȡģ���img2LCD
* ���÷�����lcd_DrawPicture(0,0,0,(u16*)demo);
****************************************************************************/
void lcd_DrawPicture(u16 StartX,u16 StartY,u8 Dir,u8 *pic)
{
  u32  i=8, len;	  //i+8����ͼ��ͷ��Ϣ
  u16 temp,x,y;
  	  
  //��ͼƬģ����ǰ8λ�õ����ȸ߶�
  x=((uint16_t)(pic[2]<<8)+pic[3])-1;	  	//��ͼ��������ȡ��ͼ��ĳ���
  y=((uint16_t)(pic[4]<<8)+pic[5])-1;	  	//��ͼ��������ȡ��ͼ��ĸ߶�
  if(Dir==0){
    //3�żĴ���ΪҺ������ʾģ���������дֵ 1030
	LCD_WR_CMD(0x0003,0x1030);   			//ͼ����ʾ����Ϊ������  �е���  �еݼ�
    LCD_WR_CMD(0x0210, StartX); 			//ˮƽ��ʾ����ʼ��ַ 0-239
  	LCD_WR_CMD(0x0211, StartX+x);           //ˮƽ��ʾ��������ַ 0-239
  	LCD_WR_CMD(0x0212, StartY);     		//��ֱ��ʾ����ʼ��ַ 0-399
  	LCD_WR_CMD(0x0213, StartY+y);         	//��ֱ��ʾ��������ַ 0-399
  
  	LCD_WR_CMD(0x0200, StartX);		          	//ˮƽ��ʾ����ַ
  	LCD_WR_CMD(0x0201, StartY);		      		//��ֱ��ʾ����ַ
  }	 	   
  else if(Dir==1){
	LCD_WR_CMD(0x0003,0x1018);   			//ͼ����ʾ����Ϊ������  �е���  �еݼ�
    LCD_WR_CMD(0x0210, StartY); 			//ˮƽ��ʾ����ʼ��ַ 0-239
  	LCD_WR_CMD(0x0211, StartY+y);           //ˮƽ��ʾ��������ַ 0-239
  	LCD_WR_CMD(0x0212, 399-(x+StartX));     //��ֱ��ʾ����ʼ��ַ 0-399
  	LCD_WR_CMD(0x0213, 399-StartX);         //��ֱ��ʾ��������ַ 0-399
  
  	LCD_WR_CMD(0x200, StartY);		          	//ˮƽ��ʾ����ַ
  	LCD_WR_CMD(0x201, 399-StartX);		      	//��ֱ��ʾ����ַ
  }	 

  //���
  LCD_WR_REG(0x0202);				          	//д���ݵ���ʾ��			 REG����д Dataдʲô
			//202ΪGRAM�Դ�  д��ĵ�ַ 
  len=2*((uint16_t)(pic[2]<<8)+pic[3])*((uint16_t)(pic[4]<<8)+pic[5]);   //�����ͼ����ռ���ֽ���  
  while(i<(len+8)) {							 //��ͼ������ĵ�9λ��ʼ����
  	temp=(uint16_t)( pic[i]<<8)+pic[i+1];		 //16λ���ߣ� ��Ҫһ�η���2���ֽڵ�����
  	LCD_WR_Data(temp);							 //��ȡ����16λ��������������ʾ��
	i=i+2;										 //ȡģλ�ü�2����Ϊ��ȡ��һ����������
  }
}

//д�Ĵ�����ַ����
void LCD_WR_REG(unsigned int index)
{
	*(__IO uint16_t *) (Bank1_LCD_C)= index;  

}



//д�Ĵ������ݺ���
//���룺dbw ����λ����1Ϊ16λ��0Ϊ8λ��
void LCD_WR_CMD(unsigned int index,unsigned int val)
{								 //Bank1_LCD_C�Դ�д����ĵ�ַ 0x60000000
	*(__IO uint16_t *) (Bank1_LCD_C)= index; //Ҫд��ļĴ�������Ϣ	
	*(__IO uint16_t *) (Bank1_LCD_D)= val;
}							 //0x60020000 �Դ�д���ݵĵ�ַ

//����ʾ��ַ����
unsigned int LCD_RD_data(void){
	unsigned int a=0;
	a=*(__IO uint16_t *) (Bank1_LCD_D);   //�ղ���
	a=*(__IO uint16_t *) (Bank1_LCD_D);   //������ʵ��16λ��������	  
	return(a);	
}

/****************************************************************************
* ��    �ƣ�u16 ili9320_BGR2RGB(u16 c)
* ��    �ܣ�RRRRRGGGGGGBBBBB ��Ϊ BBBBBGGGGGGRRRRR ��ʽ
* ��ڲ�����c      BRG ��ɫֵ
* ���ڲ�����RGB ��ɫֵ
* ˵    �����ڲ���������
* ���÷�����
****************************************************************************/
u16 lcd_BGR2RGB(u16 c)
{
  u16  r, g, b;

  b = (c>>0)  & 0x1f;
  g = (c>>5)  & 0x3f;
  r = (c>>11) & 0x1f;
  
  return( (b<<11) + (g<<5) + (r<<0) );
}

//д16λ�������ݺ���
void    LCD_WR_Data(unsigned int val)
{   
	*(__IO uint16_t *) (Bank1_LCD_D)= val; 	
}



/****************************************************************************
* ��    �ƣ�void LCD_Init(void)
* ��    �ܣ�3 ��TFT(R61509) ��ʼ������
* ��ڲ�������
* ���ڲ�������
* ˵    ����
* ���÷������� 
****************************************************************************/ 
void LCD_Init(void)
{
	unsigned int i;
	GPIO_ResetBits(GPIOE, GPIO_Pin_1);	  //Ӳ����λ
    Delay(0x1AFFf);			   
    GPIO_SetBits(GPIOE, GPIO_Pin_1 );		 
	Delay(0x1AFFf);	

	LCD_WR_CMD(0x0000, 0x00000);
    LCD_WR_CMD(0x0000, 0x00000);
	LCD_WR_CMD(0x0000, 0x00000);
	LCD_WR_CMD(0x0000, 0x00000);

	LCD_WR_CMD(0x0400, 0x06200);
	LCD_WR_CMD(0x0008, 0x00808);
	LCD_WR_CMD(0x0300, 0x00C00);//gamma
	LCD_WR_CMD(0x0301, 0x05A0B);
	LCD_WR_CMD(0x0302, 0x00906);
	LCD_WR_CMD(0x0303, 0x01017);
	LCD_WR_CMD(0x0304, 0x02300);
	LCD_WR_CMD(0x0305, 0x01700);
	LCD_WR_CMD(0x0306, 0x06309);
	LCD_WR_CMD(0x0307, 0x00C09);
	LCD_WR_CMD(0x0308, 0x0100C);
	LCD_WR_CMD(0x0309, 0x02232);
	LCD_WR_CMD(0x0010, 0x00016);//69.5Hz
	LCD_WR_CMD(0x0011, 0x00101);//
	LCD_WR_CMD(0x0012, 0x00000);//
	LCD_WR_CMD(0x0013, 0x00001);//
	LCD_WR_CMD(0x0100, 0x00330);//BT,AP
	LCD_WR_CMD(0x0101, 0x00237);//DC0,DC1,VC
	LCD_WR_CMD(0x0103, 0x00F00);//VDV
	LCD_WR_CMD(0x0280, 0x06100);//VCM
	LCD_WR_CMD(0x0102, 0x0C1B0);//VRH,VCMR,PSON,PON

	LCD_WR_CMD(0x0001, 0x00100);
	LCD_WR_CMD(0x0002, 0x00100);
	LCD_WR_CMD(0x0003, 0x01030);
	LCD_WR_CMD(0x0009, 0x00001);
	LCD_WR_CMD(0x000C, 0x00000);
	LCD_WR_CMD(0x0090, 0x08000);
	LCD_WR_CMD(0x000F, 0x00000);
	LCD_WR_CMD(0x0210, 0x00000);
	LCD_WR_CMD(0x0211, 0x000EF);
	LCD_WR_CMD(0x0212, 0x00000);
	LCD_WR_CMD(0x0213, 0x0018F);//432=1AF, 400=18F
	LCD_WR_CMD(0x0500, 0x00000);
	LCD_WR_CMD(0x0501, 0x00000);
	LCD_WR_CMD(0x0502, 0x0005F);
	LCD_WR_CMD(0x0401, 0x00001);
	LCD_WR_CMD(0x0404, 0x00000);

	LCD_WR_CMD(0x0007, 0x00100);//BASEE

	LCD_WR_CMD(0x0200, 0x00000);
	LCD_WR_CMD(0x0201, 0x00000);
    LCD_WR_CMD(0x200, 0);
    LCD_WR_CMD(0x201, 0);
	*(__IO uint16_t *) (Bank1_LCD_C)= 0x202;	 //׼��д������ʾ��
	for(i=0;i<96000;i++)
	{
	  LCD_WR_Data(0xffff);					 //�ú�ɫ����
	}  
	color1=0; 
}



/****************************************************************************
* ��    �ƣ�lcd_wr_zf(u16 StartX, u16 StartY, u16 X, u16 Y, u16 Color, u8 Dir, u8 *chr) 
* ��    �ܣ���ָ��������ʾһ���ַ�͸�������ڱ���ͼƬ��
* ��ڲ�����StartX     ����ʼ����	 0-239
*           StartY     ����ʼ����	 0-399
*           X          ��(Ϊ8�ı�����0-400
*           Y          ��			 0-240
*			Color      ��ɫ0-65535
*           Dir		   ͼ����ʾ����
*           chr        �ַ���ָ��
* ���ڲ�������
* ˵    �����ַ�ȡģ��ʽΪ��ɫ��ģ������ȡģ���ֽ�����  ȡģ�����ZIMO3
* ���÷�����lcd_wr_zf(0,0,100,100,(u16*)demo);
****************************************************************************/
//+++++++++++++++++++++++LCDд�ַ��ӳ���
void lcd_wr_zf(u16 StartX, u16 StartY, u16 X, u16 Y, u16 Color, u8 Dir, u8 *chr)
{	unsigned int temp=0,num,R_dis_mem=0,Size=0,x=0,y=0,i=0;


	if(Dir==2) LCD_WR_CMD(0x0003,0x1010);   //ͼ����ʾ����Ϊ������  �еݼ�  �е���  AM=0  I/D[1:0]=00	<--
	else if(Dir==3) LCD_WR_CMD(0x0003,0x1028);   //ͼ����ʾ����Ϊ������  �еݼ�  �е���  AM=1  I/D[1:0]=10	V
  	if(Dir==0){
		LCD_WR_CMD(0x0003,0x1030);		  //ͼ����ʾ����Ϊ������  �е���  �е���  AM=0  I/D[1:0]=11  -->
		LCD_WR_CMD(0x0210, StartX); 	  //ˮƽ��ʾ����ʼ��ַ 0-239
  		LCD_WR_CMD(0x0211, StartX+X-1);   //ˮƽ��ʾ��������ַ 0-239
  		LCD_WR_CMD(0x0212, StartY);       //��ֱ��ʾ����ʼ��ַ 0-399
  		LCD_WR_CMD(0x0213, StartY+Y-1);   //��ֱ��ʾ��������ַ 0-399
		LCD_WR_CMD(0x0200, StartX);		  //ˮƽ��ʾ����ַ
  		LCD_WR_CMD(0x0201, StartY);		  //��ֱ��ʾ����ַ	
		LCD_WR_REG(0x0202);               //׼��д������ʾ��
		Size=X*Y;						  //�ַ������ַ�ռ�õ����سߴ�
		while(i<Size){					  
			temp=*chr++;				  //һ���ֽڴ���8�����أ���˼�1������������8������
			for(num=0; num<8; num++){	  //�����ÿ���ֽڴ�����8������	    
				if((temp&0x80)>0){		  //���ֽڵĸ�λ�����жϣ�Ϊ1���ô��������16λ��ɫֵ��ʾ��д�뵽����λ�á�						 
				
					LCD_WR_Data(Color); 		
				}
				else{
					LCD_WR_CMD(0x0200, StartX+x);		//ˮƽ��ʾ����ַ
  					LCD_WR_CMD(0x0201, StartY+y);		//��ֱ��ʾ����ַ								
					LCD_WR_REG(0x0202);					//׼����������ʾ��
					R_dis_mem=LCD_RD_data();		  	//��ȡ����ɫ��Ϊ���Ӳ���͸��Ч����׼��	
					LCD_WR_Data(R_dis_mem); 		//���ֽڵĸ�λ�����жϣ�Ϊ0���õ�ǰ��������16λ��ɫֵ��ʾ�� 						
				}
				temp=temp<<1; 		  				//�ֽڸ�λ���Ƴ�
				x++;  //����
				//��һ������ʼ
				if(x>=X){x=0; y++;}				    //�������ص���Ϊ��ǰ��x��y��Ϊ��ǰ���ض�������ɫ��׼��
				i++; //��ǰ��ռ����	
			}				  
							
		}
	}
	else if(Dir==1){
		LCD_WR_CMD(0x0003,0x1018);   	  		//ͼ����ʾ����Ϊ������  �е���  �еݼ�  AM=1  I/D[1:0]=01	A
		LCD_WR_CMD(0x0210, StartY); 	  		//ˮƽ��ʾ����ʼ��ַ 0-239
  		LCD_WR_CMD(0x0211, StartY+Y-1);    		//ˮƽ��ʾ��������ַ 0-239
  		LCD_WR_CMD(0x0212, 399-(StartX+X-1));   //��ֱ��ʾ����ʼ��ַ 0-399
  		LCD_WR_CMD(0x0213, 399-StartX);    		//��ֱ��ʾ��������ַ 0-399
		LCD_WR_CMD(0x0200, StartY);		  	  	//ˮƽ��ʾ����ַ
  		LCD_WR_CMD(0x0201, 399-StartX);	 	  	//��ֱ��ʾ����ַ	
		LCD_WR_REG(0x0202);                   	//׼��д������ʾ��

		Size=X*Y;						  		//�ַ������ַ�ռ�õ����سߴ�
		while(i<Size){					  
			temp=*chr++;				  		//һ���ֽڴ���8�����أ���˼�1������������8������
			for(num=0; num<8; num++){	  		//�����ÿ���ֽڴ�����8������	    
				if((temp&0x80)>0){		  		//���ֽڵĸ�λ�����жϣ�Ϊ1���ô��������16λ��ɫֵ��ʾ��д�뵽����λ�á�						 
				
					LCD_WR_Data(Color); 		
				}
				else{
					LCD_WR_CMD(0x0200, StartY+y);		//ˮƽ��ʾ����ַ
  					LCD_WR_CMD(0x0201, 399-(StartX+x));	//��ֱ��ʾ����ַ								
					LCD_WR_REG(0x0202);					//׼����������ʾ��
					R_dis_mem=LCD_RD_data();		  	//��ȡ����ɫ��Ϊ���Ӳ���͸��Ч����׼��	
					LCD_WR_Data(R_dis_mem); 		//���ֽڵĸ�λ�����жϣ�Ϊ0���õ�ǰ��������16λ��ɫֵ��ʾ�� 						
				}
				temp=temp<<1; 		  				//�ֽڸ�λ���Ƴ�
				x++;
				if(x>=X){x=0; y++;}				    //�������ص���Ϊ��ǰ��x��y��Ϊ��ǰ���ض�������ɫ��׼��
				i++;	
			}							
		}
	}
}


//��ʾ����
void LCD_test(unsigned int temperature,unsigned int humidity,unsigned int people)
{

	unsigned char *p,*q;
    unsigned int finalTem,finalHum,i;
	finalTem = temperature;
	finalHum = 	humidity;
	i = people;
	//1,2λΪ��ʼλ�ã�����λ0��ʾ������1��ʾ����������λ��ʾ����ģ
	lcd_DrawPicture(0,0,1,a2);	    	//��ĳһָ��λ����ʾͼƬ�� ͼƬ�ĳߴ�Ҫ��240X400��Χ�ڡ�  
     
	Delay(0xafffff);

	lcd_wr_zf(100,60,80,32,63489,1,zf4);  		//��ʾ�¶�����
	lcd_wr_zf(100,120,80,32,63489,1,zf5);  		//��ʾʪ������

    lcd_wr_zf(100,0,280,32,color1,1,zf3);  		//��ʾ�༶����ѧ��
	//���Դ���280x32��bitλ     	

	if(people == 1){
		lcd_wr_zf(100,180,80,32,63489,1,zf6);  		//��ʾ�Ƿ�����
	
	}
	

	
	p=num_pub(finalTem/10);
	lcd_wr_zf(170,60,24,32,63489,1,p);  

	p=num_pub((finalTem%10));
	lcd_wr_zf(194,60,24,32,63489,1,p);  
	
	q=num_pub(finalHum/10);
	lcd_wr_zf(170,120,24,32,63489,1,q);  

	q=num_pub((finalHum%10));
	lcd_wr_zf(194,120,24,32,63489,1,q);
	
	
	  
	lcd_wr_zf(222,60,40,32,63489,1,zf7);  		//��ʾ��C
	lcd_wr_zf(222,120,40,32,63489,1,zf8);  		//��ʾ%RH

 
//			p=num_pub(i%10);							                                                   
//			lcd_wr_zf(200,180,24,32,63489,1,p);
 
 
	Delay(0xafffff);			
}



//++++++++������ֵ��ȡ��λ�������ַ������׵�ַ+++++++++++++++++++++++++++++++++++++++++++
unsigned char *num_pub(unsigned  int a){
	unsigned char *b;
	switch(a){
		case 0x01: b=&zm1[0]; break;
 		case 0x02: b=&zm2[0]; break;
		case 0x03: b=&zm3[0]; break;
		case 0x04: b=&zm4[0]; break;
		case 0x05: b=&zm5[0]; break;
		case 0x06: b=&zm6[0]; break;
		case 0x07: b=&zm7[0]; break;
		case 0x08: b=&zm8[0]; break;
		case 0x09: b=&zm9[0]; break;
		case 0x00: b=&zm0[0]; break;
		default: b=&zm0[0];break;
	}
	return(b);
}

/******************* (C) COPYRIGHT 2011 STMicroelectronics *****END OF FILE****/
