/********************************** (C) COPYRIGHT *******************************
* File Name          : main.c
* Author             :  MT@ https://b23.tv/EKGuesl (记得三连哦!)
* Version            : FFTBOX V1.1
* Date               : 2023/04/29
* Description        : Main program body.
*********************************************************************************


    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.


 *@Note
 SPI DMA, master/slave mode transceiver routine:



 接线:
SPI1_SCK(PA5)\SPI1_CS(PA4)\SPI1_DIN(PA7).
麦克风: PA1
触摸按键: PA2
*/

#include "debug.h"
#include "max7219.h"
#include "LedControl.h"
#include "arduinoFFT.h"
#include "flash.h"

#define num_samples 128
#define SAMPLE_SIZE 64
//#define maxFreq 22667

//uint16_t adc_addr[num_samples]; // point to the address of ADC continuously fast sampling output
//uint16_t buffer[num_samples]; // point to the address of ADC continuously fast sampling output
//uint16_t adc_num = num_samples; // sampling number of ADC continuously fast sampling, range [1, 65535]
//uint8_t adc_clk_div = 16; // ADC working clock = 80M/adc_clk_div, range [4, 16], the recommended value is 8
const uint16_t samples = 64; //This value MUST ALWAYS be a power of 2
uint32_t TxBuf[SAMPLE_SIZE * 3];
const float maxFrequency = 22667 ;
//const float maxFrequency = 11333 ;

int ratio = 0;
//int skipStep;
int levelMax[8] = {0};
int levelVal[8] ={0};
int matrix_pos[8] = {0};

int changeFlag =0;

//int SoundIndex = 0, direction = 0, sensity = 0;
int modeFlag = 0; // 0:running,  1:sensity setting, 2:latency setting
//int frameRate = 0;
int effectMode =0;
//unsigned long timmer1 = 0, timmer2 = 0, latency = 0;
unsigned long delaytime=0; //delay for spi
int sound_level = 0;
int SoundIndex = 0, direction = 0, sensity = 0, sensityLevel = 5, latency = 0, latencyLevel = 5;
u_int16_t flashData[3];

int  key_flag = 0;
int new_data = 0;

const byte s[3]={0B0010010,0B00101010,0B00100100};
const  byte l[3]={0B00111110,0B00000010,0B00000010};
const  byte e[3]={0B00111110,0B00101010,0B00101010};
const  byte _0[4]={0B00111100,0B01000010,0B01000010,0B00111100};
const  byte _1[4]={0B00000000,0B00100000,0B01111110,0B00000000};
const  byte _2[4]={0B00100110,0B01001010,0B01001010,0B00110010};
const  byte _3[4]={0B01000010,0B01010010,0B01010010,0B00101100};
const  byte _4[4]={0B00011100,0B00100100,0B01111110,0B00000100};
const  byte _5[4]={0B01110010,0B01010010,0B01010010,0B01001100};
const  byte _6[4]={0B00111100,0B01010010,0B01010010,0B00001100};
const  byte _7[4]={0B01000000,0B01001110,0B01010000,0B01100000};
const  byte _8[4]={0B00101100,0B01010010,0B01010010,0B00101100};
const  byte _9[4]={0B00110000,0B01001010,0B01001010,0B00111100};

const byte lv[9] ={0B00000000,0B10000000,0B11000000,0B11100000,0B11110000,0B11111000,0B11111100,0B11111110,0B11111111};
const byte rlv[9] = {0B00000000,0B00000001,0B00000011,0B00000111,0B00001111,0B00011111,0B00111111,0B01111111,0B11111111};

void DMA1_INT_INIT(void)
{

    NVIC_InitTypeDef NVIC_InitStructure = {0};

    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
    DMA_ITConfig(DMA1_Channel1, DMA_IT_TC, ENABLE);

    NVIC_InitStructure.NVIC_IRQChannel = DMA1_Channel1_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
}


u16 Touch_Key_Adc(u8 ch)
{
   TKey1->CTLR1 |= (1 << 11);
   TKey1->CTLR1 |= (1<<24); //Enable touch key

  ADC_RegularChannelConfig(ADC1, ch, 1, ADC_SampleTime_13Cycles5);
  TKey1->IDATAR1 =0x4;  //Charging Time 0x10
  TKey1->RDATAR =0x4;   //Discharging Time 0x8
  ADC_SoftwareStartConvCmd(ADC1, ENABLE); //使能ADC1软件启动转换
  while(!ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC ));
//  printf("Touch_Key_Adc::%d\r\n",TKey1->RDATAR);
  return (uint16_t) TKey1->RDATAR;
//  Delay_Ms(500);
}

void TIM1_Init(void)
{
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);
    TIM_CounterModeConfig(TIM1, TIM_CounterMode_Up);
    TIM_SetAutoreload(TIM1, 0xFFFF); //65535 reload
    TIM_PrescalerConfig(TIM1, 60000 - 1, TIM_PSCReloadMode_Immediate);
//    TIM_SelectOutputTrigger(TIM1, TIM_TRGOSource_Enable);
    TIM_Cmd(TIM1, ENABLE);
}

void TIM2_Init(void)
{
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
    TIM_CounterModeConfig(TIM2, TIM_CounterMode_Up);
    TIM_SetAutoreload(TIM2, 0xFFFF); //65535 reload
    TIM_PrescalerConfig(TIM2, 16000 - 1, TIM_PSCReloadMode_Immediate);
//    TIM_SelectOutputTrigger(TIM1, TIM_TRGOSource_Enable);
    TIM_Cmd(TIM2, ENABLE);
}

void ADC_Function_Init(void)
{
    ADC_InitTypeDef  ADC_InitStructure = {0};
    GPIO_InitTypeDef GPIO_InitStructure = {0};

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);
    RCC_ADCCLKConfig(RCC_PCLK2_Div8);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    ADC_DeInit(ADC1);
    ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;
    ADC_InitStructure.ADC_ScanConvMode = DISABLE;
    ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;
    ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;
    ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
    ADC_InitStructure.ADC_NbrOfChannel = 1;
    ADC_InitStructure.ADC_Pga=ADC_Pga_4;
    ADC_InitStructure.ADC_OutputBuffer = ADC_OutputBuffer_Enable;
    ADC_Init(ADC1, &ADC_InitStructure);

    ADC_DMACmd(ADC1, ENABLE);
    ADC_Cmd(ADC1, ENABLE);

//    ADC_BufferCmd(ADC1, ENABLE); //disable buffer
//    ADC_ResetCalibration(ADC1);
//    while(ADC_GetResetCalibrationStatus(ADC1));
//    ADC_StartCalibration(ADC1);
//    while(ADC_GetCalibrationStatus(ADC1));
//    Calibrattion_Val = Get_CalibrationValue(ADC1);

}

void Set_New_Data_Flag(void){
    new_data = 1;
}



void DMA_Tx_Init(DMA_Channel_TypeDef *DMA_CHx, u32 ppadr, u32 memadr, u16 bufsize)
{
    DMA_InitTypeDef DMA_InitStructure = {0};

    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);

    DMA_DeInit(DMA_CHx);
    DMA_InitStructure.DMA_PeripheralBaseAddr = ppadr;
    DMA_InitStructure.DMA_MemoryBaseAddr = memadr;
    DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;
    DMA_InitStructure.DMA_BufferSize = bufsize;
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Word;
    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Word;
    DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
//    DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
    DMA_InitStructure.DMA_Priority = DMA_Priority_VeryHigh;
    DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
    DMA_Init(DMA_CHx, &DMA_InitStructure);
}


void get_adc_Setup(u8 ch)
{
    TKey1->CTLR1 &= ~(1 << 11); //使能连续模式
    TKey1->CTLR1 &= ~(1<<24);  //禁用touch-key
    ADC_DMACmd(ADC1, ENABLE);   //DMA
    ADC_RegularChannelConfig(ADC1, ADC_Channel_1, 1, ADC_SampleTime_239Cycles5 );
//    ADC_BufferCmd(ADC1, ENABLE); //disable buffer
}

int16_t get_adc(void)
{
    ADC_SoftwareStartConvCmd(ADC1, ENABLE); //使能ADC1软件启动转换
    while(!ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC )); //等待转换结束
    return ADC_GetConversionValue(ADC1) ; //返回常规通道的最后一个ADC1转换结果数据
}

// function for FFT debug
 #define SCL_INDEX 0x00
 #define SCL_TIME 0x01
 #define SCL_FREQUENCY 0x02
 #define SCL_PLOT 0x03
void PrintRawData(float_t *vData, uint16_t bufferSize){
                for (int i=0; i < SAMPLE_SIZE; i+=1)
                 printf("%f\r\n",vData[i]);
}

void PrintVector(float_t *vData, uint16_t bufferSize, uint8_t scaleType)
{
  for (uint16_t i = 0; i < bufferSize; i++)
  {
      float_t abscissa = 0;
    switch (scaleType)
    {
      case SCL_INDEX:
        abscissa = (i * 1.0);
    break;
      case SCL_TIME:
        abscissa = ((i * 1.0) / maxFrequency);
    break;
      case SCL_FREQUENCY:
        abscissa = ((i * 1.0 * maxFrequency) / SAMPLE_SIZE);
    break;
    }
    printf("%f", abscissa);
    if(scaleType==SCL_FREQUENCY)
      printf("Hz");
    printf("@%f\r\n", vData[i]);
  }
  printf("\r\n");
}



void setRatio(){
  ratio = 60 + (9 - sensityLevel)*5; //1024 + (7 - sensity)*200;  //80
//    ratio = 60;  //10-30
}

void setLatency(){
    int val = 0;
    switch(effectMode){
    case 0:
        val = 700;
    break;
    case 1:
        val = 270;
    break;
    case 2:
        val = 200;
    break;
    case 3:
        val = 100;
    break;
    default:
        break;
    }
    latency = val + latencyLevel *val>>2;
}

void setDirection(){
  const byte showLine = 0B11111111;
  for (int i = 0; i < 3; i ++) {
  switch(direction){
    case 0:
      setColumn(0,0,showLine);
      break;
    case 1:
      setRow(0,0,showLine);
      break;
    case 2:
      setColumn(0,7,showLine);
      break;
    case 3:
      setRow(0,7,showLine);
      break;
    default:
      break;
  }
  delay(200);
  clearDisplay(0);
  delay(200);
  }
}


void animationEffect(int col, const byte *shiftFrom, const byte *shiftTo){
  short int twoByte[4];
  for (int i = 0; i<4; i++){
  twoByte[i] = shiftTo[i];
  twoByte[i] = twoByte[i] << 8;
  twoByte[i] = twoByte[i] + shiftFrom[i];
  // Serial.println(twoByte[i],0BIN);
  }
  for (int j = 0; j < 8; j++) {

  for (int i = 0; i < sizeof(shiftFrom); i ++) {
    twoByte[i] =  twoByte[i] >> 1;
    setColumn(0, col + i, (byte)twoByte[i]);

  }
  delay(50);
  }
}

void showAnimation(int digit){
  switch (digit){
  case 0:
    animationEffect(4,_9,_0);
    break;
  case 1:
    animationEffect(4,_0,_1);
    break;
  case 2:
    animationEffect(4,_1,_2);
    break;
  case 3:
    animationEffect(4,_2,_3);
    break;
  case 4:
    animationEffect(4,_3,_4);
    break;
  case 5:
    animationEffect(4,_4,_5);
    break;
  case 6:
    animationEffect(4,_5,_6);
    break;
  case 7:
    animationEffect(4,_6,_7);
    break;
  case 8:
    animationEffect(4,_7,_8);
    break;
  case 9:
    animationEffect(4,_8,_9);
    break;
  default:
  break;
  }
}

void valueSetting(int mode, int value, bool animationFlag){
mode = modeFlag;
switch (mode){
  case 1:
  setColumn(0,0,s[0]);
  setColumn(0,1,s[1]);
  setColumn(0,2,s[2]);
  break;
  case 2:
  setColumn(0,0,l[0]);
  setColumn(0,1,l[1]);
  setColumn(0,2,l[2]);
  break;
  case 3:
  setColumn(0,0,e[0]);
  setColumn(0,1,e[1]);
  setColumn(0,2,e[2]);
  break;
  default:
  break;
}

switch (value){
  case 0/* constant-expression */:
  if (animationFlag) showAnimation(0);
  else{
      setColumn(0,4,_0[0]);
      setColumn(0,5,_0[1]);
      setColumn(0,6,_0[2]);
      setColumn(0,7,_0[3]);
      }
    break;
 case 1/* constant-expression */:
  if (animationFlag) showAnimation(1);
  else{
      setColumn(0,4,_1[0]);
      setColumn(0,5,_1[1]);
      setColumn(0,6,_1[2]);
      setColumn(0,7,_1[3]);
      }
    break;
  case 2/* constant-expression */:
  if (animationFlag) showAnimation(2);
  else{
      setColumn(0,4,_2[0]);
      setColumn(0,5,_2[1]);
      setColumn(0,6,_2[2]);
      setColumn(0,7,_2[3]);
      }
    break;
  case 3/* constant-expression */:
  if (animationFlag) showAnimation(3);
  else{
      setColumn(0,4,_3[0]);
      setColumn(0,5,_3[1]);
      setColumn(0,6,_3[2]);
      setColumn(0,7,_3[3]);
      }
    break;
  case 4/* constant-expression */:
    if (animationFlag) showAnimation(4);
  else{
      setColumn(0,4,_4[0]);
      setColumn(0,5,_4[1]);
      setColumn(0,6,_4[2]);
      setColumn(0,7,_4[3]);
      }
    break;
  case 5/* constant-expression */:
    if (animationFlag) showAnimation(5);
  else{
      setColumn(0,4,_5[0]);
      setColumn(0,5,_5[1]);
      setColumn(0,6,_5[2]);
      setColumn(0,7,_5[3]);
      }
    break;
  case 6/* constant-expression */:
    if (animationFlag) showAnimation(6);
  else{
      setColumn(0,4,_6[0]);
      setColumn(0,5,_6[1]);
      setColumn(0,6,_6[2]);
      setColumn(0,7,_6[3]);
      }
    break;
  case 7/* constant-expression */:
   if (animationFlag) showAnimation(7);
  else{
      setColumn(0,4,_7[0]);
      setColumn(0,5,_7[1]);
      setColumn(0,6,_7[2]);
      setColumn(0,7,_7[3]);
      }
    break;
  case 8/* constant-expression */:
   if (animationFlag) showAnimation(8);
  else{
      setColumn(0,4,_8[0]);
      setColumn(0,5,_8[1]);
      setColumn(0,6,_8[2]);
      setColumn(0,7,_8[3]);
      }
    break;
  case 9/* constant-expression */:
    if (animationFlag) showAnimation(9);
  else{
      setColumn(0,4,_9[0]);
      setColumn(0,5,_9[1]);
      setColumn(0,6,_9[2]);
      setColumn(0,7,_9[3]);
       }
    break;
  default:
    break;
  }
}


void LedLevelSet(int index, int level, int direction){
//  if(level>0) level --;
  switch (direction) {
   case 0:
    setRow(0,index,lv[level]);
    break;
   case 1:
    setColumn(0,(7-index),lv[level]);
    break;
   case 2:
    setRow(0,(7-index),rlv[level]);
    break;
   case 3:
    setColumn(0,index,rlv[level]);
  }
}


void LedMaxSet(int index, int level, int direction){
  int val = 1;
  if(level == 0) val = 0;
  if (level > 0) level--;
   switch (direction) {
   case 0:
    setLed(0,index,level,val);
    break;
   case 1:
    setLed(0,level,(7-index),val);
    break;
   case 2:
    setLed(0,(7-index),(7-level),val);
    break;
   case 3:
    setLed(0,(7-level),index,val);
  }
}

void matrixRoll(int index, int level, int direction){
//    level = 2;
    int dis =( ((rand() &0xff) &  (rand() &0xff)) & (rand() &0xff) ) | (1<< (8-level));
    int cls = (1<<(8-level));
    printf("level=%d, dir=%d\r\n",level, dis);
//    Delay_Ms(500);
    switch (direction) {
     case 0:
//         for (int i = 0; i < 5; i++)
//      setRow(0,index,cls);
//      Delay_Us(5000);
//      if (changeFlag) {
//          changeFlag ++;
//          if (changeFlag == 9) changeFlag = 0;
          setRow(0,index,dis);
//      }
      break;
     case 1:
     setColumn(0,(7-index),cls);
      setColumn(0,(7-index),dis);
      break;
     case 2:
     setRow(0,(7-index),cls);
      setRow(0,(7-index),dis);
      break;
     case 3:
     setColumn(0,index,cls);
      setColumn(0,index,dis);
    }
}

//long press key fuction
void longPressed(){
  clearDisplay(0);
  switch (modeFlag)
  {
  case 0:
    modeFlag = 1;
    valueSetting(0, sensityLevel, false);
    break;
  case 1:
    modeFlag = 2;
    valueSetting(1, latencyLevel, false);
    break;
  case 2:
    modeFlag = 3;
    valueSetting(1, effectMode, false);
    break;
  case 3:
      flashData[0] = direction;
      flashData[1] = sensityLevel;
      flashData[2] = latencyLevel;
      flashData[3] = effectMode;
//      printf("Write flashData = %d,%d,%d\r\n", flashData[0], flashData[1], flashData[2]);
      Flash_Write(flashData, 4);
      modeFlag = 0;
      get_adc_Setup(ADC_Channel_1);
//Save option and out

    break;
//  case 3:
//    modeFlag = 0;
//    break;
  default:
    break;
  }
  delay(500);
  key_flag = 0;
}

//short press key fuction
void shortPressed(){
  switch (modeFlag)
  {
  case 0:
    direction ++;
    if (direction > 3) direction = 0;
    setDirection();
    break;
  case 1:
    sensityLevel ++;
    if (sensityLevel > 9) sensityLevel = 0;
    valueSetting(modeFlag, sensityLevel, true);
    break;
  case 2:
    latencyLevel ++;
    if (latencyLevel > 9) latencyLevel = 0;
    valueSetting(modeFlag, latencyLevel, true);
    break;
  case 3:
     effectMode ++;
     if (effectMode > 3) effectMode = 0;
     valueSetting(modeFlag, effectMode, true);
     break;
  default:
    break;
  }
  // delay(500);
  setRatio();
  setLatency();
  key_flag = 0;
}

void Key_Detection(void) {
//key detection
    if (key_flag ==0){
        if (Touch_Key_Adc(ADC_Channel_2) < 3080) {

          Delay_Ms(600);
          if (Touch_Key_Adc(ADC_Channel_2) < 3080) {
//            longPressed();
              key_flag = 2;
          }
//          else shortPressed();
          else key_flag = 1;
        }
    }
}

void Switch_Key(void){
    switch (key_flag){
    case 1:
        shortPressed();
        break;
    case 2:
        longPressed();
        break;
    default:
        break;
    }
}

int getMid(int a, int b, int c){
    if (((b > a) &&(c>b))||((b>c)&&a>b)) return b;
    else if (((a>b)&&(c>a)) || ((a> c)&&(b>a))) return a;
    else  return c;
}

int getMid2(int a, int b, int c){
//    int max;
//    max = a>b?a:b;
//    max = max > c? max: c;

    int min;
    min = a > b? b:a;
    min = min > c?c:min;

    return (a + b + c - min)/2;
   }

int16_t  adc[SAMPLE_SIZE * 3];
float_t vReal[SAMPLE_SIZE];
float_t vImag[SAMPLE_SIZE];

int main(void)
{
    Delay_Init();
    TIM1_Init();
    TIM2_Init();
    USART_Printf_Init(115200);
    Max7219_GPIO_Init();
    Delay_Ms(500);
    Init_MAX7219();
    LedControl_Init(SPI_MOSI, SPI_CLK, SPI_CS, 1);
//    adc_Init();
    ADC_Function_Init();
    DMA_Tx_Init(DMA1_Channel1, (u32)&ADC1->RDATAR, (u32)TxBuf, SAMPLE_SIZE * 3);
    DMA_Cmd(DMA1_Channel1, ENABLE);
    DMA1_INT_INIT();

    ADC_RegularChannelConfig(ADC1, ADC_Channel_1, 1, ADC_SampleTime_239Cycles5);
    ADC_SoftwareStartConvCmd(ADC1, ENABLE);
//    Write_Max7219(2,0xFF);
//    Delay_Ms(1000);
//while(1);
    Flash_Read(flashData, 4);
//    printf("Read flashData = %d,%d,%d\r\n", flashData[0], flashData[1], flashData[2]);
    if(flashData[0]>0 && flashData[0] < 4) direction = flashData[0];
    else direction = 0;
    if(flashData[1]>0 && flashData[1] < 10) sensityLevel = flashData[1];
    else sensityLevel = 5;
    if(flashData[2]>0 && flashData[2] < 10) latencyLevel = flashData[2];
    else latencyLevel = 5;
    if(flashData[3]>0 && flashData[3] < 3) effectMode = flashData[3];
    else effectMode = 0;

    setDirection();
    setRatio();
    setLatency();


//#define SPEED_TEST
//5.7(175fps) or 6.6s(151fps) //5.61 & 6.16s //5.53 &6.08(or 180 & 164fps)/2.7&3.2ms(370/307fps)
#ifdef SPEED_TEST
    int iCount =0;
#endif
  while(1)
    {
      Switch_Key();
      if (modeFlag == 0) {
#ifdef SPEED_TEST

          iCount ++;
          if (iCount> 10000) {
              printf("10000fps!\r\n");
              iCount = 0;
          }
#endif

//          millis();
//      get_adc_Setup(ADC_Channel_1);
//          for (int i=0; i < SAMPLE_SIZE*3; i ++)  adc[i] = get_adc();
          int j =0;
          for (int i=0; i < SAMPLE_SIZE*3; i += 3){
//              vReal[j] = adc[i];
              vReal[j] = (TxBuf[i]+ TxBuf[i+1] +TxBuf[i+2]) / 3 ;
//              millis();
//              vReal[j] = getMid(TxBuf[i], TxBuf[i+1],TxBuf[i+2]);
//              printf("getMid = %f", millis());
//              millis();
//              vReal[j] = getMid2(TxBuf[i], TxBuf[i+1],TxBuf[i+2]);
//              printf("getMid2 = %f\r\n", millis());
//              vReal[j] = TxBuf[i+2] ;
              vImag[j] = 0;
              j++;
//Delay_Ms(500);
          }

//          printf("%f\r\n", millis());
//          Delay_Ms(2000);

 //   DCRemoval(vReal,samples);
 //   Windowing(vReal, samples, FFT_WIN_TYP_HAMMING, FFT_FORWARD); /* Weigh data */
 //   Compute(vReal, vImag, samples, FFT_FORWARD); //Compute FFT
 //   ComplexToMagnitude(vReal, vImag, samples); // Compute magnitudes
 //  //FFT debug here
//             millis();
//          DMA_Cmd(DMA1_Channel1, ENABLE);
          if (new_data) {
                  new_data = 0;
//                 PrintRawData(vReal, SAMPLE_SIZE);
                 FFT_DCRemoval(vReal,SAMPLE_SIZE); //0.113777 vs 0.07522
    //             PrintVector(vReal, SAMPLE_SIZE, SCL_TIME);
    //             FFT_Windowing(vReal, SAMPLE_SIZE, FFT_WIN_TYP_HAMMING, FFT_FORWARD);  /* Weigh data */ //1.230444 vs 1.168722
                 FFT_Windowing_Fast64(vReal,SAMPLE_SIZE); //0.060777
    //             PrintVector(vReal, SAMPLE_SIZE, SCL_TIME);
//                 PrintRawData(vReal, SAMPLE_SIZE);
//                 millis();
                 FFT_Compute(vReal, vImag, SAMPLE_SIZE, FFT_FORWARD); //Compute FFT//2.483666 vs 1.755888 vs. 1.693166 vs. 1.40688
//                 printf("%f\r\n", millis());
                 FFT_ComplexToMagnitude(vReal, vImag, SAMPLE_SIZE); //0.552833 vs 0.536944
    //             PrintVector(vReal, (SAMPLE_SIZE >> 1), SCL_FREQUENCY);

//                 printf("time=%f\r\n", millis());  //Total: 2.64996
//

//                 Delay_Ms(2000);



   // 8 segments led display
    for (int i = 0; i < (samples >> 1); i++)
    {
         int j= i>>2 ;
         if(j < 8) {
           int level = vReal[i]/ratio;
           if (level > 8) level = 8;
           if (level > levelMax[j]) {
               levelMax[j] = level;
               TIM1->CNT = 0;
           }
           if (level > levelVal[j]){
               levelVal[j] = level;
//               TIM2->CNT = 0;
           }
         }
   }

 }
       //set led 6fps@128p
       for (int i = 0; i < 8; i ++) {
           switch (effectMode) {
               case 0:
                   LedLevelSet(i, levelVal[i], direction);
                   break;
               case 1:
                   LedLevelSet(i, levelVal[i], direction);
                   LedMaxSet(i, levelMax[i],direction);
                   break;
               case 2:
                   LedLevelSet(i, 0, direction);
                   LedMaxSet(i, levelMax[i],direction);
                   break;
               case 3:
//                   LedLevelSet(i, 0, direction);
                   matrixRoll(i,levelMax[i],direction);
//                   LedMaxSet(i, levelMax[i],direction);
//                   LedMaxSet(i, levelMax[i],direction);
//                   setColumn(0, i,rand()|(1>>levelMax[i]) );
                   break;
               default:
                   break;
           };
//
//           if(effectMode != 2) LedLevelSet(i, levelVal[i], direction);
//           else  LedLevelSet(i, 0, direction);
//           if (effectMode != 0) LedMaxSet(i, levelMax[i],direction);
       }

       // fading effect 0fps
        if ((TIM1->CNT)>latency) {
              TIM1->CNT = 0;
             for (int i = 0; i < 8; i ++) {
               if (levelMax[i]> 0) levelMax[i] --;
             }
       }
        if ((TIM2->CNT)>latency) {
              TIM2->CNT = 0;
             for (int i = 0; i < 8; i ++) {
               if (levelVal[i]> 0) levelVal[i] --;
               if (matrix_pos[i] < 8) matrix_pos[i] ++;
               else {
                   matrix_pos[i] = 0;
                   changeFlag = 1;
               }
             }
       }
      }
     }

 }



