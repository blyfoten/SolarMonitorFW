//
// This file is part of the GNU ARM Eclipse distribution.
// Copyright (c) 2014 Liviu Ionescu.
//

// ----------------------------------------------------------------------------

#include <stdio.h>
#include "diag/Trace.h"

#include "Timer.h"
#include "BlinkLed.h"
#include "stm32f10x_conf.h"



// Keep the LED on for 2/3 of a second.
#define BLINK_ON_TICKS  (TIMER_FREQUENCY_HZ * 2 / 3)
#define BLINK_OFF_TICKS (TIMER_FREQUENCY_HZ - BLINK_ON_TICKS)

// ----- main() ---------------------------------------------------------------

// Sample pragmas to cope with warnings. Please note the related line at
// the end of this function, used to pop the compiler diagnostics status.
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-parameter"
#pragma GCC diagnostic ignored "-Wmissing-declarations"
#pragma GCC diagnostic ignored "-Wreturn-type"

uint16_t ADC_ConvertedValue[8];
volatile uint8_t flag = 0;

int main(int argc, char* argv[])
{
  int i;
  DMA_InitTypeDef  DMA_InitStructure;
  GPIO_InitTypeDef GPIO_InitStructure;
  ADC_InitTypeDef  ADC_InitStructure;
  NVIC_InitTypeDef NVIC_InitStructure;

  RCC_LSEConfig(RCC_LSE_ON);
  RCC_HCLKConfig(RCC_SYSCLK_Div1);
  RCC_PCLK1Config(RCC_HCLK_Div1);
  RCC_PCLK2Config(RCC_HCLK_Div1);
  RCC_ADCCLKConfig(RCC_PCLK2_Div2);

  /* Enable ADC1, GPIOA, GPIO Peripheral clock and DMA1*/
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1 | RCC_APB2Periph_GPIOA | BLINK_RCC_MASKx(BLINK_PORT_NUMBER), ENABLE);

  //NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);

  /* Configure and enable ADC interrupt */

  NVIC_InitStructure.NVIC_IRQChannel = ADC1_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);

  timer_start();


  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1|GPIO_Pin_2|GPIO_Pin_3|GPIO_Pin_6;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
  GPIO_Init(GPIOA, &GPIO_InitStructure);

  // Configure pin in output push/pull mode
  GPIO_InitStructure.GPIO_Pin = BLINK_PIN_MASK(BLINK_PIN_NUMBER) | BLINK_PIN_MASK(BLINK_PIN_NUMBER-1);
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_Init(BLINK_GPIOx(BLINK_PORT_NUMBER), &GPIO_InitStructure);


  /* DMA Channel1 Configuration ----------------------------------------------*/

  DMA_DeInit(DMA1_Channel1);
  DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&ADC1->DR;
  DMA_InitStructure.DMA_MemoryBaseAddr     = (uint32_t) &ADC_ConvertedValue;
  DMA_InitStructure.DMA_DIR                = DMA_DIR_PeripheralSRC;
  DMA_InitStructure.DMA_BufferSize         = 1;
  DMA_InitStructure.DMA_PeripheralInc      = DMA_PeripheralInc_Disable;
  DMA_InitStructure.DMA_MemoryInc          = DMA_MemoryInc_Disable;
  DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
  DMA_InitStructure.DMA_MemoryDataSize     = DMA_MemoryDataSize_HalfWord;
  DMA_InitStructure.DMA_Mode               = DMA_Mode_Normal;
  DMA_InitStructure.DMA_Priority           = DMA_Priority_High;
  DMA_InitStructure.DMA_M2M                = DMA_M2M_Disable;
  DMA_Init(DMA1_Channel1, &DMA_InitStructure);

  DMA_Cmd(DMA1_Channel1, ENABLE);




  // Start with led turned off
  blink_led_off();


  ADC_DeInit(ADC1);
  ADC_InitStructure.ADC_Mode               = ADC_Mode_Independent;
  ADC_InitStructure.ADC_ScanConvMode       = DISABLE;
  ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;
  ADC_InitStructure.ADC_ExternalTrigConv   = ADC_ExternalTrigConv_None;
  ADC_InitStructure.ADC_DataAlign          = ADC_DataAlign_Right;
  ADC_InitStructure.ADC_NbrOfChannel       = 1;
  ADC_Init(ADC1, &ADC_InitStructure);


  ADC_RegularChannelConfig(ADC1, ADC_Channel_TempSensor, 1, ADC_SampleTime_1Cycles5);
  ADC_TempSensorVrefintCmd(ENABLE);

  //ADC_RegularChannelConfig(ADC1, ADC_Channel_2, 2, ADC_SampleTime_1Cycles5);
  //ADC_RegularChannelConfig(ADC1, ADC_Channel_3, 3, ADC_SampleTime_1Cycles5);
  //ADC_RegularChannelConfig(ADC1, ADC_Channel_6, 4, ADC_SampleTime_1Cycles5);


  /* Enable ADCx's DMA interface */
  //ADC_DMACmd(ADC1, ENABLE);

  ADC_ITConfig(ADC1,ADC_IT_EOC,ENABLE);

  /* Enable ADCx */
  ADC_Cmd(ADC1, ENABLE);


  ADC_ResetCalibration(ADC1);                     // Enable ADC1 reset calibration register
  while(ADC_GetResetCalibrationStatus(ADC1));     // Check the end of ADC1 reset calibration register

  ADC_StartCalibration(ADC1);                     // Start ADC1 calibration
  while(ADC_GetCalibrationStatus(ADC1));          // Check the end of ADC1 calibration

  //ADC_SoftwareStartConvCmd(ADC1, ENABLE);

  /* Test on channel1 transfer complete flag */
  //while(!DMA_GetFlagStatus(DMA1_FLAG_TC1));
  /* Clear channel1 transfer complete flag */
  //DMA_ClearFlag(DMA1_FLAG_TC1);
  //blink_led_on();


  // Infinite loop
  while (1)
    {
      //if (flag == 1)
      for (i=0;i<1000000;i++)
    	  asm("nop");
//      blink_led2_on();
//      blink_led2_off();
      blink_led2_on();
//      blink_led2_on();
//      blink_led2_off();
    //flag = 0;
	  /* Start ADC1 Software Conversion */
      //timer_sleep(BLINK_ON_TICKS);
      for (i=0;i<1000000;i++)
    	  asm("nop");
      blink_led2_off();
      //timer_sleep(BLINK_OFF_TICKS);


      // Count seconds on the trace device.
      //  trace_printf("Second %u\n", seconds);
    }
  // Infinite loop, never return.

}



void ADC1_IRQHandler(void)
{
    flag = 1;
    ADC_ClearITPendingBit(ADC1, ADC_IT_EOC);
    blink_led_on();
}

#pragma GCC diagnostic pop

// ----------------------------------------------------------------------------
