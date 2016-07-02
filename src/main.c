// ----------------------------------------------------------------------------
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-parameter"
#pragma GCC diagnostic ignored "-Wmissing-declarations"
#pragma GCC diagnostic ignored "-Wreturn-type"

#include <stdio.h>
#include <stdlib.h>
#include "diag/Trace.h"

#include "resources.h"

#include <math.h>
#include <stm32f0xx_gpio.h>
#include <stm32f0xx_rcc.h>
#include <stm32f0xx_tim.h>
#include <stm32f0xx_dac.h>
#include <stm32f0xx_dma.h>


// diodes
#define GreenLED GPIO_Pin_9
#define BlueLED GPIO_Pin_8
#define LEDGPIO GPIOC

//Define Push button
#define PushButton_Pin GPIO_Pin_0
#define PushButton_GPIO GPIOA

//Initialization structs

GPIO_InitTypeDef			GPIO_InitStructure;
GPIO_InitTypeDef         	LEDs;
GPIO_InitTypeDef         	Buttons;
GPIO_InitTypeDef         	Pins;
TIM_TimeBaseInitTypeDef 	TTB;
DAC_InitTypeDef         	DAC_InitStructure;
NVIC_InitTypeDef         	NVIC_InitStructure;


int lutIndex =0;
int lutStep =12;
int handleTIM =1;
int usingLeds =1;
int usingDAC=0;

uint16_t outputDAC=4095;


uint32_t accumulator=0;
uint32_t accumulatorStep=5;
uint16_t CurrentTimerVal = 0;

#define ONE ((int)(2*256*256*256))

uint32_t  feedback_i = ONE*0.25/32768;
uint32_t  accumulator_i = ONE*1;
uint32_t  accumulatorDelta_i = ONE*0.005;


// configure board
void  InitBoard()
{


        SystemInit();
        SystemCoreClockUpdate();

        //Enable GPIO Clock
        RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOC, ENABLE);
        // enable Timer
        //RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);
        //RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
        RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3 ,ENABLE);
        // enable DAC


        RCC_APB1PeriphClockCmd(RCC_APB1Periph_DAC, ENABLE);
        //Initialize LEDs
        GPIO_StructInit(&LEDs);
        LEDs.GPIO_Pin = GreenLED | BlueLED;
        LEDs.GPIO_Mode = GPIO_Mode_OUT;
        LEDs.GPIO_OType = GPIO_OType_PP;
        LEDs.GPIO_PuPd = GPIO_PuPd_NOPULL;
        LEDs.GPIO_Speed = GPIO_Speed_Level_3; //50MHz
        GPIO_Init(LEDGPIO, &LEDs);
        // turn on green  and blue Leds
        GPIO_SetBits(LEDGPIO, GreenLED);
        GPIO_ResetBits(LEDGPIO, BlueLED);


}
// configure DAC
void InitDAC(void)
{

	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);
  /* DAC Periph clock enable */
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_DAC, ENABLE);



  // Configure PA.04/05 (DAC) as output -------------------------
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4 | GPIO_Pin_5;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOA, &GPIO_InitStructure);


  /* Fill DAC InitStructure */
  //DAC_InitStructure.DAC_Trigger = DAC_Trigger_T2_TRGO; /* Select the receiving end */
  //DAC_InitStructure.DAC_OutputBuffer = DAC_OutputBuffer_Disable;

  	  DAC_InitStructure.DAC_Trigger = DAC_Trigger_T3_TRGO;
  	  //DAC_InitStructure.DAC_Trigger = DAC_Trigger_Software;
  	  //DAC_InitStructure.DAC_Trigger = DAC_Trigger_None;
 	  DAC_InitStructure.DAC_OutputBuffer = DAC_OutputBuffer_Disable;//DAC_OutputBuffer_Enable;


 	  //DAC_InitStructure.DAC_WaveGeneration = DAC_WaveGeneration_Triangle;
 	  //DAC_InitStructure.DAC_LFSRUnmask_TriangleAmplitude = DAC_TriangleAmplitude_4095;

 	  DAC_Init(DAC_Channel_1, &DAC_InitStructure);

 	  //(+) Enable the DAC channel using DAC_Cmd()
 	  DAC_Cmd(DAC_Channel_1, ENABLE);


  /* DAC channel1 Configuration */
 	  DAC_Init(DAC_Channel_2, &DAC_InitStructure);

  /* Enable DAC Channel1: Once the DAC channel1 is enabled, PA.04 is
     automatically connected to the DAC converter. */
 	  DAC_Cmd(DAC_Channel_2, ENABLE);



}




void InitTimer()
{

	 	 	SystemInit(); //Ensure CPU is running at correctly set clock speed
	     	SystemCoreClockUpdate(); //Update SystemCoreClock variable to current clock speed
	     	//SysTick_Config(SystemCoreClock/1000); //Set up a systick interrupt every millisecond

            TTB.TIM_CounterMode = TIM_CounterMode_Up;
            TTB.TIM_Prescaler = 10-1; //  4800 kHz
            TTB.TIM_Period = 10-1; //1Hz;
            TTB.TIM_RepetitionCounter = 0;
            TIM_TimeBaseInit(TIM3, &TTB);
            TIM_Cmd(TIM3, ENABLE);



            /* http://visualgdb.com/tutorials/arm/stm32/timers/ */
            TIM_ITConfig(TIM3, TIM_IT_Update, ENABLE);
            TIM_SelectOutputTrigger(TIM3, TIM_TRGOSource_Update);
            /* http://forbot.pl/blog/artykuly/programowanie/kurs-stm32-7-liczniki-timery-w-praktyce-pwm-id8459 */



            NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn;
            NVIC_InitStructure.NVIC_IRQChannelPriority = 0;
            NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
            NVIC_Init(&NVIC_InitStructure);

}




void TIM3_IRQHandler()
{



    if (TIM_GetITStatus(TIM3, TIM_IT_Update) == SET)
    {


    	if (usingLeds)
    	{

        if (GPIO_ReadOutputDataBit(LEDGPIO, BlueLED))
                    GPIO_ResetBits(LEDGPIO, BlueLED);
                else
                    GPIO_SetBits(LEDGPIO, BlueLED);

        if (GPIO_ReadOutputDataBit(LEDGPIO, GreenLED))
                    GPIO_ResetBits(LEDGPIO, GreenLED);
                else
                    GPIO_SetBits(LEDGPIO, GreenLED);

    	}


    	// index modulo number of samples

    	//lutIndex=(uint16_t) (lutIndex%1024);

/*
    	    	if (lutIndex>=1024)
    	    	{
    	    		lutIndex =0;
    	    	}


    	DAC_SetChannel1Data(DAC_Align_12b_R,(uint16_t) Sine1024_12bit[lutIndex] );
*/
    	//lutIndex+=lutStep;



    	if (lutIndex>=1024)
    	{
    		//lutIndex= lutIndex%1024;
    		lutIndex=0;
    	}

		DAC_SetChannel1Data(DAC_Align_12b_R,(uint16_t) Sine1024_12bit[lutIndex] );
		lutIndex+=lutStep;

    	/*
    	 http://stackoverflow.com/questions/16889426/fm-synthesis-using-phase-accumulator
    	 */



       TIM_ClearITPendingBit(TIM3, TIM_IT_Update);

    }

}


int
main(int argc, char* argv[])
{


    InitBoard();
    InitDAC();
    InitTimer();

    //trace_printf("Hello\n");

  // Infinite loop
  while (1)
  {


  }

  return 0;
}




#ifdef  USE_FULL_ASSERT
void assert_failed(uint8_t* file, uint32_t line) {
  while (1) {
  }
}
#endif

#pragma GCC diagnostic pop

// ----------------------------------------------------------------------------

// ----------------------------------------------------------------------------
