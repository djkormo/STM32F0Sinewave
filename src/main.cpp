// ----------------------------------------------------------------------------
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-parameter"
#pragma GCC diagnostic ignored "-Wmissing-declarations"
#pragma GCC diagnostic ignored "-Wreturn-type"

#include <stdio.h>
#include <stdlib.h>
#include "diag/Trace.h"

#include <math.h>
#include "stm32f0xx.h"
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



// table for SIN with 12-bit resolution 2^12=4096, minimal 0, maximal - 4095
const uint16_t Sine12bit[32] = {
                      2047,
					  2447,
					  2831,
					  3185,
					  3498,
					  3750,
					  3939,
					  4056,
					  4095,
					  4056,
                      3939,
					  3750,
					  3495,
					  3185,
					  2831,
					  2447,
					  2047,
					  1647,
					  1263,
					  909,
                      599,
					  344,
					  155,
					  38,
					  0,
					  38,
					  155,
					  344,
					  599,
					  909,
					  1263,
					  1647};


const uint16_t Sine1024[1024] =
{
		2048	,
		2061	,
		2073	,
		2086	,
		2098	,
		2111	,
		2123	,
		2136	,
		2148	,
		2161	,
		2174	,
		2186	,
		2199	,
		2211	,
		2224	,
		2236	,
		2249	,
		2261	,
		2274	,
		2286	,
		2299	,
		2311	,
		2324	,
		2336	,
		2349	,
		2361	,
		2373	,
		2386	,
		2398	,
		2411	,
		2423	,
		2435	,
		2448	,
		2460	,
		2472	,
		2484	,
		2497	,
		2509	,
		2521	,
		2533	,
		2546	,
		2558	,
		2570	,
		2582	,
		2594	,
		2606	,
		2618	,
		2630	,
		2643	,
		2655	,
		2667	,
		2678	,
		2690	,
		2702	,
		2714	,
		2726	,
		2738	,
		2750	,
		2762	,
		2773	,
		2785	,
		2797	,
		2808	,
		2820	,
		2832	,
		2843	,
		2855	,
		2866	,
		2878	,
		2889	,
		2901	,
		2912	,
		2924	,
		2935	,
		2946	,
		2958	,
		2969	,
		2980	,
		2991	,
		3002	,
		3013	,
		3024	,
		3036	,
		3047	,
		3057	,
		3068	,
		3079	,
		3090	,
		3101	,
		3112	,
		3122	,
		3133	,
		3144	,
		3154	,
		3165	,
		3175	,
		3186	,
		3196	,
		3207	,
		3217	,
		3227	,
		3238	,
		3248	,
		3258	,
		3268	,
		3278	,
		3288	,
		3298	,
		3308	,
		3318	,
		3328	,
		3337	,
		3347	,
		3357	,
		3367	,
		3376	,
		3386	,
		3395	,
		3405	,
		3414	,
		3423	,
		3433	,
		3442	,
		3451	,
		3460	,
		3469	,
		3478	,
		3487	,
		3496	,
		3505	,
		3514	,
		3523	,
		3531	,
		3540	,
		3548	,
		3557	,
		3565	,
		3574	,
		3582	,
		3591	,
		3599	,
		3607	,
		3615	,
		3623	,
		3631	,
		3639	,
		3647	,
		3655	,
		3663	,
		3670	,
		3678	,
		3685	,
		3693	,
		3700	,
		3708	,
		3715	,
		3722	,
		3730	,
		3737	,
		3744	,
		3751	,
		3758	,
		3765	,
		3772	,
		3778	,
		3785	,
		3792	,
		3798	,
		3805	,
		3811	,
		3817	,
		3824	,
		3830	,
		3836	,
		3842	,
		3848	,
		3854	,
		3860	,
		3866	,
		3872	,
		3877	,
		3883	,
		3888	,
		3894	,
		3899	,
		3905	,
		3910	,
		3915	,
		3920	,
		3925	,
		3930	,
		3935	,
		3940	,
		3945	,
		3950	,
		3954	,
		3959	,
		3963	,
		3968	,
		3972	,
		3976	,
		3980	,
		3985	,
		3989	,
		3993	,
		3997	,
		4000	,
		4004	,
		4008	,
		4011	,
		4015	,
		4018	,
		4022	,
		4025	,
		4028	,
		4032	,
		4035	,
		4038	,
		4041	,
		4043	,
		4046	,
		4049	,
		4052	,
		4054	,
		4057	,
		4059	,
		4061	,
		4064	,
		4066	,
		4068	,
		4070	,
		4072	,
		4074	,
		4076	,
		4077	,
		4079	,
		4081	,
		4082	,
		4084	,
		4085	,
		4086	,
		4087	,
		4088	,
		4089	,
		4090	,
		4091	,
		4092	,
		4093	,
		4094	,
		4094	,
		4095	,
		4095	,
		4095	,
		4096	,
		4096	,
		4096	,
		4096	,
		4096	,
		4096	,
		4096	,
		4095	,
		4095	,
		4095	,
		4094	,
		4094	,
		4093	,
		4092	,
		4091	,
		4090	,
		4089	,
		4088	,
		4087	,
		4086	,
		4085	,
		4084	,
		4082	,
		4081	,
		4079	,
		4077	,
		4076	,
		4074	,
		4072	,
		4070	,
		4068	,
		4066	,
		4064	,
		4061	,
		4059	,
		4057	,
		4054	,
		4052	,
		4049	,
		4046	,
		4043	,
		4041	,
		4038	,
		4035	,
		4032	,
		4028	,
		4025	,
		4022	,
		4018	,
		4015	,
		4011	,
		4008	,
		4004	,
		4000	,
		3997	,
		3993	,
		3989	,
		3985	,
		3980	,
		3976	,
		3972	,
		3968	,
		3963	,
		3959	,
		3954	,
		3950	,
		3945	,
		3940	,
		3935	,
		3930	,
		3925	,
		3920	,
		3915	,
		3910	,
		3905	,
		3899	,
		3894	,
		3888	,
		3883	,
		3877	,
		3872	,
		3866	,
		3860	,
		3854	,
		3848	,
		3842	,
		3836	,
		3830	,
		3824	,
		3817	,
		3811	,
		3805	,
		3798	,
		3792	,
		3785	,
		3778	,
		3772	,
		3765	,
		3758	,
		3751	,
		3744	,
		3737	,
		3730	,
		3722	,
		3715	,
		3708	,
		3700	,
		3693	,
		3685	,
		3678	,
		3670	,
		3663	,
		3655	,
		3647	,
		3639	,
		3631	,
		3623	,
		3615	,
		3607	,
		3599	,
		3591	,
		3582	,
		3574	,
		3565	,
		3557	,
		3548	,
		3540	,
		3531	,
		3523	,
		3514	,
		3505	,
		3496	,
		3487	,
		3478	,
		3469	,
		3460	,
		3451	,
		3442	,
		3433	,
		3423	,
		3414	,
		3405	,
		3395	,
		3386	,
		3376	,
		3367	,
		3357	,
		3347	,
		3337	,
		3328	,
		3318	,
		3308	,
		3298	,
		3288	,
		3278	,
		3268	,
		3258	,
		3248	,
		3238	,
		3227	,
		3217	,
		3207	,
		3196	,
		3186	,
		3175	,
		3165	,
		3154	,
		3144	,
		3133	,
		3122	,
		3112	,
		3101	,
		3090	,
		3079	,
		3068	,
		3057	,
		3047	,
		3036	,
		3024	,
		3013	,
		3002	,
		2991	,
		2980	,
		2969	,
		2958	,
		2946	,
		2935	,
		2924	,
		2912	,
		2901	,
		2889	,
		2878	,
		2866	,
		2855	,
		2843	,
		2832	,
		2820	,
		2808	,
		2797	,
		2785	,
		2773	,
		2762	,
		2750	,
		2738	,
		2726	,
		2714	,
		2702	,
		2690	,
		2678	,
		2667	,
		2655	,
		2643	,
		2630	,
		2618	,
		2606	,
		2594	,
		2582	,
		2570	,
		2558	,
		2546	,
		2533	,
		2521	,
		2509	,
		2497	,
		2484	,
		2472	,
		2460	,
		2448	,
		2435	,
		2423	,
		2411	,
		2398	,
		2386	,
		2373	,
		2361	,
		2349	,
		2336	,
		2324	,
		2311	,
		2299	,
		2286	,
		2274	,
		2261	,
		2249	,
		2236	,
		2224	,
		2211	,
		2199	,
		2186	,
		2174	,
		2161	,
		2148	,
		2136	,
		2123	,
		2111	,
		2098	,
		2086	,
		2073	,
		2061	,
		2048	,
		2035	,
		2023	,
		2010	,
		1998	,
		1985	,
		1973	,
		1960	,
		1948	,
		1935	,
		1922	,
		1910	,
		1897	,
		1885	,
		1872	,
		1860	,
		1847	,
		1835	,
		1822	,
		1810	,
		1797	,
		1785	,
		1772	,
		1760	,
		1747	,
		1735	,
		1723	,
		1710	,
		1698	,
		1685	,
		1673	,
		1661	,
		1648	,
		1636	,
		1624	,
		1612	,
		1599	,
		1587	,
		1575	,
		1563	,
		1550	,
		1538	,
		1526	,
		1514	,
		1502	,
		1490	,
		1478	,
		1466	,
		1453	,
		1441	,
		1429	,
		1418	,
		1406	,
		1394	,
		1382	,
		1370	,
		1358	,
		1346	,
		1334	,
		1323	,
		1311	,
		1299	,
		1288	,
		1276	,
		1264	,
		1253	,
		1241	,
		1230	,
		1218	,
		1207	,
		1195	,
		1184	,
		1172	,
		1161	,
		1150	,
		1138	,
		1127	,
		1116	,
		1105	,
		1094	,
		1083	,
		1072	,
		1060	,
		1049	,
		1039	,
		1028	,
		1017	,
		1006	,
		995	,
		984	,
		974	,
		963	,
		952	,
		942	,
		931	,
		921	,
		910	,
		900	,
		889	,
		879	,
		869	,
		858	,
		848	,
		838	,
		828	,
		818	,
		808	,
		798	,
		788	,
		778	,
		768	,
		759	,
		749	,
		739	,
		729	,
		720	,
		710	,
		701	,
		691	,
		682	,
		673	,
		663	,
		654	,
		645	,
		636	,
		627	,
		618	,
		609	,
		600	,
		591	,
		582	,
		573	,
		565	,
		556	,
		548	,
		539	,
		531	,
		522	,
		514	,
		505	,
		497	,
		489	,
		481	,
		473	,
		465	,
		457	,
		449	,
		441	,
		433	,
		426	,
		418	,
		411	,
		403	,
		396	,
		388	,
		381	,
		374	,
		366	,
		359	,
		352	,
		345	,
		338	,
		331	,
		324	,
		318	,
		311	,
		304	,
		298	,
		291	,
		285	,
		279	,
		272	,
		266	,
		260	,
		254	,
		248	,
		242	,
		236	,
		230	,
		224	,
		219	,
		213	,
		208	,
		202	,
		197	,
		191	,
		186	,
		181	,
		176	,
		171	,
		166	,
		161	,
		156	,
		151	,
		146	,
		142	,
		137	,
		133	,
		128	,
		124	,
		120	,
		116	,
		111	,
		107	,
		103	,
		99	,
		96	,
		92	,
		88	,
		85	,
		81	,
		78	,
		74	,
		71	,
		68	,
		64	,
		61	,
		58	,
		55	,
		53	,
		50	,
		47	,
		44	,
		42	,
		39	,
		37	,
		35	,
		32	,
		30	,
		28	,
		26	,
		24	,
		22	,
		20	,
		19	,
		17	,
		15	,
		14	,
		12	,
		11	,
		10	,
		9	,
		8	,
		7	,
		6	,
		5	,
		4	,
		3	,
		2	,
		2	,
		1	,
		1	,
		1	,
		0	,
		0	,
		0	,
		0	,
		0	,
		0	,
		0	,
		1	,
		1	,
		1	,
		2	,
		2	,
		3	,
		4	,
		5	,
		6	,
		7	,
		8	,
		9	,
		10	,
		11	,
		12	,
		14	,
		15	,
		17	,
		19	,
		20	,
		22	,
		24	,
		26	,
		28	,
		30	,
		32	,
		35	,
		37	,
		39	,
		42	,
		44	,
		47	,
		50	,
		53	,
		55	,
		58	,
		61	,
		64	,
		68	,
		71	,
		74	,
		78	,
		81	,
		85	,
		88	,
		92	,
		96	,
		99	,
		103	,
		107	,
		111	,
		116	,
		120	,
		124	,
		128	,
		133	,
		137	,
		142	,
		146	,
		151	,
		156	,
		161	,
		166	,
		171	,
		176	,
		181	,
		186	,
		191	,
		197	,
		202	,
		208	,
		213	,
		219	,
		224	,
		230	,
		236	,
		242	,
		248	,
		254	,
		260	,
		266	,
		272	,
		279	,
		285	,
		291	,
		298	,
		304	,
		311	,
		318	,
		324	,
		331	,
		338	,
		345	,
		352	,
		359	,
		366	,
		374	,
		381	,
		388	,
		396	,
		403	,
		411	,
		418	,
		426	,
		433	,
		441	,
		449	,
		457	,
		465	,
		473	,
		481	,
		489	,
		497	,
		505	,
		514	,
		522	,
		531	,
		539	,
		548	,
		556	,
		565	,
		573	,
		582	,
		591	,
		600	,
		609	,
		618	,
		627	,
		636	,
		645	,
		654	,
		663	,
		673	,
		682	,
		691	,
		701	,
		710	,
		720	,
		729	,
		739	,
		749	,
		759	,
		768	,
		778	,
		788	,
		798	,
		808	,
		818	,
		828	,
		838	,
		848	,
		858	,
		869	,
		879	,
		889	,
		900	,
		910	,
		921	,
		931	,
		942	,
		952	,
		963	,
		974	,
		984	,
		995	,
		1006	,
		1017	,
		1028	,
		1039	,
		1049	,
		1060	,
		1072	,
		1083	,
		1094	,
		1105	,
		1116	,
		1127	,
		1138	,
		1150	,
		1161	,
		1172	,
		1184	,
		1195	,
		1207	,
		1218	,
		1230	,
		1241	,
		1253	,
		1264	,
		1276	,
		1288	,
		1299	,
		1311	,
		1323	,
		1334	,
		1346	,
		1358	,
		1370	,
		1382	,
		1394	,
		1406	,
		1418	,
		1429	,
		1441	,
		1453	,
		1466	,
		1478	,
		1490	,
		1502	,
		1514	,
		1526	,
		1538	,
		1550	,
		1563	,
		1575	,
		1587	,
		1599	,
		1612	,
		1624	,
		1636	,
		1648	,
		1661	,
		1673	,
		1685	,
		1698	,
		1710	,
		1723	,
		1735	,
		1747	,
		1760	,
		1772	,
		1785	,
		1797	,
		1810	,
		1822	,
		1835	,
		1847	,
		1860	,
		1872	,
		1885	,
		1897	,
		1910	,
		1922	,
		1935	,
		1948	,
		1960	,
		1973	,
		1985	,
		1998	,
		2010	,
		2023	,
		2035

};


int lutIndex =0;
int handleTIM =1;
int usingLeds =1;
// shape of sine
/* #define shapeSteps 32

uint16_t sin_wave [shapeSteps];
*/
uint32_t accumulator=0;
uint32_t accumulatorStep=50;
uint16_t CurrentTimerVal = 0;
uint16_t outputDac=0.0;

// generate Lookup Table


void MakeShapes()
{
/*
    for (int i=0;i<shapeSteps;i++)
    {
        sin_wave[i]= (2048)+(uint16_t) 4096*sin(M_PI*i/(shapeSteps)/2); // no values below zero

    }
*/
}
// configure board
void  InitBoard()
{


        SystemInit();
        SystemCoreClockUpdate();

        //Enable GPIO Clock
        RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOC, ENABLE);
        // enable Timer
        RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);
        RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
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
        //GPIO_SetBits(LEDGPIO, BlueLED);
        /*
        GPIO_StructInit(&Buttons);
        Buttons.GPIO_Pin = PushButton_Pin; //Set pins inside the struct
        Buttons.GPIO_Mode = GPIO_Mode_IN; //Set GPIO pins as INPUT
        Buttons.GPIO_PuPd = GPIO_PuPd_NOPULL; //No pullup required as pullup is external
        GPIO_Init(GPIO_Init, &Buttons); //Assign struct to LED_GPIO
		*/


}
// configure DAC
void InitDAC(void)
{

	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);
  /* DAC Periph clock enable */
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_DAC, ENABLE);



  GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_4;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;

  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_Level_3; //50MHz
  GPIO_Init(GPIOA, &GPIO_InitStructure);



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


 	 // DAC_InitStructure.DAC_WaveGeneration = DAC_WaveGeneration_Triangle;
 	 // DAC_InitStructure.DAC_LFSRUnmask_TriangleAmplitude = DAC_TriangleAmplitude_4095;

 	  DAC_Init(DAC_Channel_1, &DAC_InitStructure);

 	  //(+) Enable the DAC channel using DAC_Cmd()
 	  DAC_Cmd(DAC_Channel_1, ENABLE);


  /* DAC channel1 Configuration */
 	  DAC_Init(DAC_Channel_1, &DAC_InitStructure);

  /* Enable DAC Channel1: Once the DAC channel1 is enabled, PA.04 is
     automatically connected to the DAC converter. */
 	  DAC_Cmd(DAC_Channel_1, ENABLE);



}




void InitTimer()
{

	 	 	 SystemInit(); //Ensure CPU is running at correctly set clock speed
	     	 SystemCoreClockUpdate(); //Update SystemCoreClock variable to current clock speed
	     	 SysTick_Config(SystemCoreClock);

            TTB.TIM_CounterMode = TIM_CounterMode_Up;
            TTB.TIM_Prescaler = 48000; //  1 kHz
            TTB.TIM_Period = 1000; //1Hz;
            //TTB.TIM_Period = 500-1; //100Hz;
            //TTB.TIM_Period = 50-1; //1kHz
            //TTB.TIM_Period = 1; //50kHz
            TTB.TIM_RepetitionCounter = 0;

            TIM_TimeBaseInit(TIM3, &TTB);
            TIM_Cmd(TIM3, ENABLE);



            /* http://visualgdb.com/tutorials/arm/stm32/timers/ */
            TIM_ITConfig(TIM3, TIM_IT_Update, ENABLE);
            TIM_SelectOutputTrigger(TIM3, TIM_TRGOSource_Update);
            /* http://forbot.pl/blog/artykuly/programowanie/kurs-stm32-7-liczniki-timery-w-praktyce-pwm-id8459 */


            //NVIC_InitStructure.NVIC_IRQChannel = TIM1_BRK_UP_TRG_COM_IRQn;
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

             lutIndex+=accumulatorStep;
             if (lutIndex>=32)
             {
            	 lutIndex=0;
             }

             // send to output
             DAC_SetChannel1Data(DAC_Align_12b_R,(uint16_t) Sine12bit[lutIndex] );
             DAC_SetChannel1Data(DAC_Align_12b_R,(uint16_t) 4095  );
             DAC_SetChannel2Data(DAC_Align_12b_L,(uint16_t) 0  );

        	 //DAC_SetChannel1Data(DAC_Align_12b_R, 0x07FF);
             DAC_SoftwareTriggerCmd(DAC_Channel_1, ENABLE);
             //TIM_ClearITPendingBit(TIM3, TIM_IT_Update);

    }

}


int
main(int argc, char* argv[])
{

    MakeShapes();
    InitBoard();
    InitDAC();
    InitTimer();


  // Infinite loop
  while (1)
  {
	  /*
      lutIndex+=accumulatorStep;
      if (lutIndex>=32)
      {
     	 lutIndex=0;
      }

      // send to output
      DAC_SetChannel1Data(DAC_Align_12b_R,(uint16_t) Sine12bit[lutIndex] );
      DAC_SetChannel1Data(DAC_Align_12b_R, 0x07FF);
      DAC_SoftwareTriggerCmd(DAC_Channel_1, ENABLE);
      //DAC_SetChannel1Data(DAC_Align_12b_R,(uint16_t) 4095  );
      //DAC_SetChannel1Data(DAC_Align_12b_R,(uint16_t) 0  );
	*/

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
