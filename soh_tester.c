
//[II] SOH tester

/*SIDDESHWAR RAGHAVAN*/

#include "stm32f0xx.h"
#include "stm32f0xx_gpio.h"
#include "stm32f0xx_tim.h"
#include "stm32f0xx_misc.h"
#include "stm32f0xx_syscfg.h"
#include "stm32f0xx_exti.h"
#include "stm32f0xx_tim.h"

#define KEY_PORT GPIOA
#define KEY GPIO_Pin_0
int flag = 0;
volatile int BitSet=0;
/*void GPIO_ToggleBits(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin)
{
  /* Check the parameters
  assert_param(IS_GPIO_ALL_PERIPH(GPIOx));

  GPIOx->ODR ^= GPIO_Pin;
}
 */

void EnableButtonInterrupt()
{
    EXTI_InitTypeDef EXTI_InitStruct;
    NVIC_InitTypeDef NVIC_InitStruct;
    GPIO_InitTypeDef gpioUserButton;

    // initializing the user button
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);
    gpioUserButton.GPIO_Pin = KEY;
    gpioUserButton.GPIO_Mode = GPIO_Mode_IN;
    gpioUserButton.GPIO_OType = GPIO_OType_PP;
    gpioUserButton.GPIO_Speed = GPIO_Speed_50MHz;
    gpioUserButton.GPIO_PuPd = GPIO_PuPd_DOWN;
    GPIO_Init(KEY_PORT, &gpioUserButton);

    SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOA, EXTI_PinSource0);    //Inform system that PA0 is EXTI line
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);
    EXTI_InitStruct.EXTI_Line = EXTI_Line0;    //PA0 is connected to EXTI_Line0
    EXTI_InitStruct.EXTI_LineCmd = ENABLE;     //Enable interrupt
    EXTI_InitStruct.EXTI_Mode = EXTI_Mode_Interrupt;  //Interrupt mode
    EXTI_InitStruct.EXTI_Trigger = EXTI_Trigger_Rising_Falling;  //Triggers on falling edge
    EXTI_Init(&EXTI_InitStruct);

    NVIC_InitStruct.NVIC_IRQChannel = EXTI0_1_IRQn ;
    NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStruct);
}

void InitializePINs()
{
  // Pin PB8
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOB, ENABLE);
     GPIO_InitTypeDef gpioPinPB8;
    gpioPinPB8.GPIO_Pin = GPIO_Pin_8;
    gpioPinPB8.GPIO_Mode = GPIO_Mode_AF;
    gpioPinPB8.GPIO_Speed = GPIO_Speed_50MHz;
    gpioPinPB8.GPIO_OType = GPIO_OType_PP;
    gpioPinPB8.GPIO_PuPd = GPIO_PuPd_DOWN;
    GPIO_Init(GPIOB, &gpioPinPB8);
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource8, GPIO_AF_2);

}

void InitializeLEDs()
{
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOC, ENABLE);

    GPIO_InitTypeDef gpioLed1;
    gpioLed1.GPIO_Pin = GPIO_Pin_9;
    gpioLed1.GPIO_Mode = GPIO_Mode_AF;   //AF= alternate function like pwm,i2c,usart,etc.
    gpioLed1.GPIO_Speed = GPIO_Speed_50MHz;
    gpioLed1.GPIO_PuPd = GPIO_PuPd_DOWN;

    GPIO_Init(GPIOC, &gpioLed1);
    GPIO_WriteBit(GPIOC, GPIO_Pin_9, Bit_RESET);
    GPIO_PinAFConfig(GPIOC, GPIO_PinSource9, GPIO_AF_0);


}

/*
void InitializeTimer2(int state)
{
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);

    TIM_TimeBaseInitTypeDef timerInitStructure;
    timerInitStructure.TIM_Prescaler = 480;
    timerInitStructure.TIM_CounterMode = TIM_CounterMode_Up;
    timerInitStructure.TIM_Period = 100000;     //100 millisecond
    timerInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    timerInitStructure.TIM_RepetitionCounter = 0;
    TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE);
    TIM_TimeBaseInit(TIM2, &timerInitStructure);
    TIM_Cmd(TIM2, state);
}
void EnableTimerInterrupt2()
{
    NVIC_InitTypeDef nvicStructure;
    //NVIC_ClearPendingIRQ(TIM2_IRQHandler);
   nvicStructure.NVIC_IRQChannel = TIM2_IRQn;
   nvicStructure.NVIC_IRQChannelPriority = 0;
    nvicStructure.NVIC_IRQChannelCmd = ENABLE;
   NVIC_Init(&nvicStructure);
}
 */
void InitializeTimer3()
{
   // for led pc9
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);

    TIM_TimeBaseInitTypeDef timerInitStructure;
    timerInitStructure.TIM_Prescaler = 4;
    timerInitStructure.TIM_CounterMode = TIM_CounterMode_Up;
    timerInitStructure.TIM_Period = 1000;
    timerInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    //timerInitStructure.TIM_RepetitionCounter = 0;
    TIM_TimeBaseInit(TIM3, &timerInitStructure);
    TIM_Cmd(TIM3, ENABLE);


}

void InitializeTimer16()
{
   // pin PB8

    // 1 sec ON 1 sec OFF --> Timer2 for PB8
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM16, ENABLE);

    TIM_Cmd(TIM16, DISABLE);
    TIM_TimeBaseInitTypeDef timerInitStructure;
    timerInitStructure.TIM_Prescaler = 4;
    timerInitStructure.TIM_CounterMode = TIM_CounterMode_Up;
    timerInitStructure.TIM_Period = 2000;
    timerInitStructure.TIM_ClockDivision = 0;
    timerInitStructure.TIM_RepetitionCounter = 0;
    TIM_TimeBaseInit(TIM16, &timerInitStructure);


}

void InitializePWMChannelPin()
{


    TIM_OCInitTypeDef outputChannelInit ;
    outputChannelInit.TIM_OCMode = TIM_OCMode_PWM2;
    outputChannelInit.TIM_Pulse = 1000;
    outputChannelInit.TIM_OutputState = TIM_OutputState_Enable;
    outputChannelInit.TIM_OutputNState = TIM_OutputNState_Disable;
    outputChannelInit.TIM_OCPolarity = TIM_OCPolarity_Low;
    TIM_OC1Init(TIM16, &outputChannelInit);   //TIM_OC1Init -> the 1 represents channel = TIM16_CH1
    //TIM_OC1PreloadConfig(TIM16, TIM_OCPreload_Enable);
     TIM_Cmd(TIM16, ENABLE);
    TIM_CtrlPWMOutputs(TIM16, ENABLE);


}


void InitializePWMChannelLed(int pulse)
{
    // 2 sec ON 2 sec OFF --> timer1 for led PC9
    TIM_OCInitTypeDef outputChannelInit;
    outputChannelInit.TIM_OCMode = TIM_OCMode_PWM1;
    outputChannelInit.TIM_Pulse = pulse;
    outputChannelInit.TIM_OutputState = TIM_OutputState_Enable;
    outputChannelInit.TIM_OCPolarity = TIM_OCPolarity_High;
    TIM_OC4Init(TIM3, &outputChannelInit);     //TIM_OC4Init -> the 4 represents channel = TIM3_CH4
    //TIM_OC4PreloadConfig(TIM3, TIM_OCPreload_Enable);
    TIM_Cmd(TIM3, ENABLE);
    TIM_CtrlPWMOutputs(TIM3, ENABLE);


}int main()
{

   // int BitSet=0; // Set for duty cycle

    InitializeLEDs();
    InitializePINs();
    InitializeTimer3();
    InitializeTimer16();
    InitializePWMChannelPin();
    EnableButtonInterrupt();




    for (;;)
    {

      //waiting for interrupt 2
     /* while(flag==0)
       {

       }
       //TIM_Cmd(TIM2, DISABLE);
        flag=0;
        /* Clear interrupt flag */



    }
    }





void EXTI0_1_IRQHandler(void)
{
   // BitSet for duty cycle
    if (EXTI_GetITStatus(EXTI_Line0) != RESET)
    {
      EXTI_ClearITPendingBit(EXTI_Line0);
        /* Do code when PA0 is changed */
       //  if(GPIO_ReadInputDataBit(KEY_PORT, KEY))
        if(keypress==0)
        {
          keypress=1;       // to find btw keypress and release interrupts
        }
        else
        {
         keypress=0;
        ++BitSet;
        if(BitSet>25)
        {
                BitSet=0;
        }
        else
        {
                InitializePWMChannelLed(BitSet*40);
        }
      //  while(GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_0));
        }
        TIM_Cmd(TIM2, ENABLE);
        }

}

void TIM2_IRQHandler()   // timer 2 to introduce delay for key press jitter removal
{
    if (TIM_GetITStatus(TIM2, TIM_IT_Update) != RESET)
    {
      flag=1;

    }
    TIM_ClearITPendingBit(TIM2, TIM_IT_Update);
}


[III] Basic timer functionality

/*SIDDESHWAR RAGHAVAN*/

#include "stm32f0xx.h"
#include "stm32f0xx_gpio.h"
#include "stm32f0xx_tim.h"


void InitializeLEDs()
{
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOC, ENABLE);

    GPIO_InitTypeDef gpioStructure;
    gpioStructure.GPIO_Pin = GPIO_Pin_8;
    gpioStructure.GPIO_Mode = GPIO_Mode_OUT;
    gpioStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOC, &gpioStructure);

    GPIO_WriteBit(GPIOC, GPIO_Pin_8, Bit_RESET);
}

void InitializeTimer()
{
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);

    TIM_TimeBaseInitTypeDef timerInitStructure;
    timerInitStructure.TIM_Prescaler = 40000;
    timerInitStructure.TIM_CounterMode = TIM_CounterMode_Up;
    timerInitStructure.TIM_Period = 500;
    timerInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    timerInitStructure.TIM_RepetitionCounter = 0;
    TIM_TimeBaseInit(TIM2, &timerInitStructure);
    TIM_Cmd(TIM2, ENABLE);
}

int main()
{
    InitializeLEDs();
    InitializeTimer();

    for (;;)
    {
        int timerValue = TIM_GetCounter(TIM2);
        if (timerValue == 400)
            GPIO_WriteBit(GPIOC, GPIO_Pin_8, Bit_SET);
        else if (timerValue == 500)
            GPIO_WriteBit(GPIOC, GPIO_Pin_8, Bit_RESET);
    }
}
