/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2018 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#define Button1 GPIO_PIN_15//B15
#define Button2 GPIO_PIN_14//B14
#define Button3 GPIO_PIN_13//B13
#define Button4 GPIO_PIN_12//B12
#define user_delay(x) (x*10300)//1~1mS  if HCLK to core = 75 MHz  x must be <=?416987
#define alarm_on 1
#define alarm_off 0
//1uS~10,3 cycles
//1mS~10300 cycles
//10mS~103000 cycles
//160ms~1648000 cycles


char alarm_on_off=0;
uint32_t time=0;
uint32_t buffer=0;

uint32_t seconds=0;
uint32_t minutes=22;
uint32_t hours=22;

uint32_t Discharge_selection=1;
uint32_t Discharge_selection_Flag=0;
uint32_t Timer_started_Flag=0;

uint32_t DesHrs=0;
uint32_t EdHrs=0;
uint32_t DesMin=0;
uint32_t EdMin=0;


uint32_t array_numbers[16]=
{
  0xFC, //"0"
  0x30, //"1"
  0x6E,
  0x7A,
  0xB2,
  0xDA,
  0xDE,
  0x70,
  0xFE,
  0xFA,
  0xF6,
  0x9E,
  0xCC,
  0x3E,
  0xCE,
  0xC6
};
uint32_t array_Rotation[6]={0x80,0x04,0x08,0x10,0x20,0x40};
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
RTC_HandleTypeDef hrtc;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;

/* USER CODE BEGIN PV */
//RTC_TimeTypeDef sTime = {0};
//RTC_DateTypeDef DateToUpdate = {0};
RTC_TimeTypeDef sTime;
RTC_TimeTypeDef user_alarm_Time;
RTC_DateTypeDef sData;
RTC_DateTypeDef DateToUpdate;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
static void MX_RTC_Init(void);
/* USER CODE BEGIN PFP */
void checking_the_buttons_pressed(void);
void rotation_effect(void);
void write_hello(void);
void write_0toF(void);
void write_Fto0(void);
void numbers_selection(void);
void Lath_up_down(void);
void write_time_register(uint32_t value);
uint32_t time_shaper(uint32_t Buff_hours, uint32_t Buff_minutes, uint32_t Buff_seconds);
void delay_without_interruptions(unsigned int ij);//10,3~1us 160ms->ij~1648000
void menu_selection_mode(void);
void numbers_selection(void);
void start_the_alarm_signal(void);
void numbers_selection_signal(void);
void disable_signal(void);
void refresh_time(void);

static void MX_TIM2_Init_50(void);
static void MX_TIM2_Init_100(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

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
  //__HAL_RCC_PWR_CLK_ENABLE();
  //HAL_PWR_EnableBkUpAccess();
  //  __HAL_RCC_BKP_CLK_ENABLE();//!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
  //HAL_PWR_EnableBkUpAccess();//!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
  MX_GPIO_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_RTC_Init();
  /* USER CODE BEGIN 2 */
  

  //HAL_TIM_Base_Stop_IT(&htim1);          //stop the timer1
  HAL_TIM_Base_Stop_IT(&htim2);          //stop the timer2
  //__HAL_TIM_CLEAR_IT(&htim1, TIM_IT_UPDATE);//Clear interrupt FUCKING BEATCH!!!
  __HAL_TIM_CLEAR_IT(&htim2, TIM_IT_UPDATE);//Clear interrupt FUCKING BEATCH!!!
  
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    
    //####################################################Инициализация сдвиговых регистров или ХЗ чё это  
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);//Reset
    HAL_Delay(5);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_RESET);//CLK
    HAL_Delay(5);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, GPIO_PIN_RESET);//DS
    HAL_Delay(5);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, GPIO_PIN_RESET);//Lath
    HAL_Delay(5);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);//Reset UP
    HAL_Delay(5);
 
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, GPIO_PIN_SET);//DS
    HAL_Delay(5);
   
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);//Reset DOWN
    HAL_Delay(5);
    Lath_up_down();
    HAL_Delay(5);

    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);//Reset UP
    HAL_Delay(5);

    //####################################################Rotation
    rotation_effect();
        
    while(1)
    {
    //####################################################Checking buttons
    //######Button1#############################
    if( HAL_GPIO_ReadPin(GPIOB, Button1) == 1 )//Если кнопка нажата
    {   
        write_hello();
    }  
    //######Button2#############################
    if( HAL_GPIO_ReadPin(GPIOB, Button2) == 1 )//Если кнопка нажата
    {  
        write_0toF();
    }
    //######Button3#############################
    if( HAL_GPIO_ReadPin(GPIOB, Button3) == 1 )//Если кнопка нажата
    {  
        write_Fto0();
    }
    //######Button4#############################MENU
    if( HAL_GPIO_ReadPin(GPIOB, Button4) == 1 )//Если кнопка нажата
    {        
        delay_without_interruptions(user_delay(200));
        menu_selection_mode();
    } 
    
    //###################################################Backup time
    //Чтобы время продолжало считать (при подключенной VBAT) даже после отключения питания, надо в функции MX_RTC_Init(); запретить обнулять время!!!
    
    //###################################################Refresh time
    //HAL_RTCEx_BKUPWrite(&hrtc, RTC_BKP_DR1, 1234);
    refresh_time();

    
    //###################################################Check the alarm
    //if(AlarmTime==time)
    //start_sound_on_the_10_minutes();
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE|RCC_OSCILLATORTYPE_LSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_RTC;
  PeriphClkInit.RTCClockSelection = RCC_RTCCLKSOURCE_LSE;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief RTC Initialization Function
  * @param None
  * @retval None
  */
static void MX_RTC_Init(void)
{

  /* USER CODE BEGIN RTC_Init 0 */
  
  /* USER CODE END RTC_Init 0 */

  RTC_TimeTypeDef sTime;
  RTC_DateTypeDef DateToUpdate = {0};

  /* USER CODE BEGIN RTC_Init 1 */

  /* USER CODE END RTC_Init 1 */
  /** Initialize RTC Only
  */
  hrtc.Instance = RTC;
  hrtc.Init.AsynchPrediv = RTC_AUTO_1_SECOND;
  hrtc.Init.OutPut = RTC_OUTPUTSOURCE_ALARM;
  if (HAL_RTC_Init(&hrtc) != HAL_OK)
  {
    Error_Handler();
  }

  /* USER CODE BEGIN Check_RTC_BKUP */
    
  /* USER CODE END Check_RTC_BKUP */

  /** Initialize RTC and set the Time and Date
  */
  /*sTime.Hours = 0;
  sTime.Minutes = 0;
  sTime.Seconds = 0;

  if (HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BIN) != HAL_OK)
  {
    Error_Handler();
  }*/
  DateToUpdate.WeekDay = RTC_WEEKDAY_MONDAY;
  DateToUpdate.Month = RTC_MONTH_JANUARY;
  DateToUpdate.Date = 1;
  DateToUpdate.Year = 0;

  if (HAL_RTC_SetDate(&hrtc, &DateToUpdate, RTC_FORMAT_BIN) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN RTC_Init 2 */
  HAL_RTC_MspInit(&hrtc);
  /* USER CODE END RTC_Init 2 */

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

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 7199;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 1000;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

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

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 7199;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 500;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
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
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_RESET);

  /*Configure GPIO pins : PA1 PA2 PA3 PA4 */
  GPIO_InitStruct.Pin = GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB12 PB13 PB14 PB15 */
  GPIO_InitStruct.Pin = GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PB9 */
  GPIO_InitStruct.Pin = GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
    void refresh_time(void)
    {
      HAL_RTC_GetTime(&hrtc, &sTime, RTC_FORMAT_BIN);  
      HAL_RTC_GetDate(&hrtc, &sData, RTC_FORMAT_BIN);
      if(seconds!=sTime.Seconds)     
      {
        seconds=sTime.Seconds;     
        minutes=sTime.Minutes;           
        hours=sTime.Hours; 
        
        time_shaper(hours,minutes,seconds);
        time = buffer;
        write_time_register(time);
        
        //checking the alarm
        if(alarm_on_off&&
           //(seconds==user_alarm_Time.Seconds)&&
             (minutes==user_alarm_Time.Minutes)&&
               (hours==user_alarm_Time.Hours))
        {start_the_alarm_signal();}
      }
    }


void delay_without_interruptions(unsigned int ij)//unsigned int ij=user_delay(x)
{
    while(ij)
    {--ij;}
}

uint32_t time_shaper(uint32_t Buff_hours, uint32_t Buff_minutes, uint32_t Buff_seconds)
{
//#######################################################################_HOURS
if (Buff_hours<=9)//0..9 hours
  {
    buffer=array_numbers[Buff_hours];
    buffer=(buffer<<8);
  }
if ((Buff_hours>9)&&(Buff_hours<=19))//10..19 hours
  {
    buffer=0x30;//"1"
    buffer=(buffer<<8);
    buffer+=array_numbers[Buff_hours-10];
    buffer=(buffer<<8);
  }
if (Buff_hours>19)//20..23 hours
  {
    buffer=0x6E;//"2"
    buffer=(buffer<<8);
    buffer+=array_numbers[Buff_hours-20];
    buffer=(buffer<<8);
  }
//#######################################################################_MINUTES

buffer+=array_numbers[(Buff_minutes)/10];//dozens
buffer=(buffer<<8);
buffer+=array_numbers[Buff_minutes%10];//units

//#######################################################################_SECONDS
//empty
return 0;
}

  void rotation_effect(void)//######Rotation
        {
        for(int jk=0;jk<7;jk++)
        {
        buffer |= array_Rotation[jk];
        buffer =(buffer<<24);
        write_time_register(buffer);
        buffer =(buffer>>24);
        HAL_Delay(60); 
        }
        buffer =(buffer<<8);
        HAL_Delay(80);

        for(int jk=0;jk<7;jk++)
        {
        buffer |= array_Rotation[jk];
        buffer =(buffer<<16);
        write_time_register(buffer);
        buffer =(buffer>>16);
        HAL_Delay(60); 
        }
        buffer =(buffer<<8);
        HAL_Delay(80);
        
        for(int jk=0;jk<7;jk++)
        {
        buffer |= array_Rotation[jk];
        buffer =(buffer<<8);
        write_time_register(buffer);
        buffer =(buffer>>8);
        HAL_Delay(60); 
        }
        buffer =(buffer<<8);
        HAL_Delay(80);
        
        for(int jk=0;jk<7;jk++)
        {
        buffer |= array_Rotation[jk];
        write_time_register(buffer);
        HAL_Delay(60); 
        }
        HAL_Delay(80); 
        }

static void MX_TIM2_Init_50(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 7199;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 500;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
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

void Lath_up_down(void)
{
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, GPIO_PIN_SET);  //Lath UP
    //HAL_Delay(5);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, GPIO_PIN_RESET);//Lath DOWN
    //HAL_Delay(5);
}
void write_time_register(uint32_t value)
{
  for(int i=0; i<32; i++)
  {
    if((value&0x01)==0x01)
    {
      HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, GPIO_PIN_SET);//DS UP
    }
    else
    {
      HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, GPIO_PIN_RESET);//DS DOWN
    }
    //HAL_Delay(5);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_SET);   //CLK UP
    //HAL_Delay(5);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_RESET); //CLK DOWN
    //HAL_Delay(5);
    
    value=(value>>1);
  }
  Lath_up_down();
}

      void write_hello(void)
      {
        for(int jk=0;jk<4;jk++)
        {
        buffer = 0xB6CEB4FC;//"Hello"
        time = buffer;
        write_time_register(time);
        HAL_Delay(250);
        
        buffer = 0x00;//
        time = buffer;//"0000"
        write_time_register(time);
        HAL_Delay(450);
        }
      }

      void write_0toF(void)
        {
        for(int jk=0;jk<16;jk++)
        {
        buffer=0; 
        
            for(int p=0;p<3;p++)
            {
              buffer += array_numbers[jk];//"0-F"
              buffer = (buffer<<8);
            }
        buffer += array_numbers[jk];//"0-F"  
        time = buffer;
        write_time_register(time);    
        HAL_Delay(100);
        
        time = 0x00;
        write_time_register(time);
        delay_without_interruptions(user_delay(160));//~160mS
        }
        }

      void write_Fto0(void)
      {
      for(int jk=16;jk!=0;jk--)
        {
        buffer=0; 
        
            for(int p=0;p<3;p++)
            {
              buffer += array_numbers[jk-1];//"F-0"
              buffer = (buffer<<8);
            }
        buffer += array_numbers[jk-1];//"F-0"  
        time = buffer;
        write_time_register(time);    
        HAL_Delay(100);
        
        time = 0x00;
        write_time_register(time);
        delay_without_interruptions(user_delay(160));//~160mS
        }
      }

//numbers selection
  void numbers_selection(void)
      {
        while(HAL_GPIO_ReadPin(GPIOB, Button4) == 1)//Ждать отжатия кнопки
        {;}
        delay_without_interruptions(user_delay(250));//~250mS
        
        Timer_started_Flag=0;//for blinking
        //####Launch menu####
        while(HAL_GPIO_ReadPin(GPIOB, Button4) == 0)//While the button do not press
        {
            //####Menu B3####Select
            if(HAL_GPIO_ReadPin(GPIOB, Button3) == 1 )//Button 3 pressed
            {
                Discharge_selection++;
                if(Discharge_selection>4)
                {Discharge_selection=1;}

             while(HAL_GPIO_ReadPin(GPIOB, Button3) == 1)//Ждать отжатия кнопки
             {;}
             delay_without_interruptions(user_delay(250));//~250mS
            } 
            //####Menu B2####--
            if(HAL_GPIO_ReadPin(GPIOB, Button2) == 1 )//Button 2 pressed
            {  
                if(Discharge_selection==1)
                {       
                  if((hours/10)>0)
                  {DesHrs--;}
                }  
                
                if(Discharge_selection==2)
                {
                  if((hours%10)>0)
                  {EdHrs--;}
                }
                  
                if(Discharge_selection==3)
                {
                  if((minutes/10)>0)
                  {DesMin--;}
                }  
                if(Discharge_selection==4)
                {
                  if((minutes%10)>0)
                  {EdMin--;}
                }
              
             while(HAL_GPIO_ReadPin(GPIOB, Button2) == 1)//Ждать отжатия кнопки
             {;}
             delay_without_interruptions(user_delay(250));//~250mS

             hours=DesHrs*10+EdHrs;
             minutes=DesMin*10+EdMin;
             seconds=0;
             
             time_shaper(hours,minutes,seconds);
             write_time_register(buffer);
            }
            //####Menu B1####++
             if( HAL_GPIO_ReadPin(GPIOB, Button1) == 1 )//Button 1 pressed
            {        
                if(Discharge_selection==1)
                {       
                  if((hours/10)<2)
                  {DesHrs++;}
                  if((DesHrs*10+EdHrs)>23)
                  {EdHrs=0;}
                }  
                
                if(Discharge_selection==2)
                {
                  if(((hours/10)<2)&&((hours%10)<9))
                  {EdHrs++;}
                  if(((hours/10)==2)&&((hours%10)<3))
                  {EdHrs++;}
                }
                  
                if(Discharge_selection==3)
                {
                  if((minutes/10)<5)
                  {DesMin++;}
                }  
                if(Discharge_selection==4)
                {
                  if((minutes%10)<9)
                  {EdMin++;}
                }
                
                while(HAL_GPIO_ReadPin(GPIOB, Button1) == 1)//Ждать отжатия кнопки
                {;}
                delay_without_interruptions(user_delay(250));//~250mS
             
             hours=DesHrs*10+EdHrs;
             minutes=DesMin*10+EdMin;
             seconds=0;
             
             time_shaper(hours,minutes,seconds);
             write_time_register(buffer);
            }  
            //#####################Blinking  
            if(Timer_started_Flag==0)
            {
            Timer_started_Flag=1;
              if(Discharge_selection_Flag==1)
              {
                MX_TIM2_Init_50();                     //init timer_2
                __HAL_TIM_CLEAR_IT(&htim2, TIM_IT_UPDATE);//Clear interrupt FUCKING BEATCH!!!
                HAL_TIM_Base_Start_IT(&htim2);          //start timer_2              
              }  
              
              if(Discharge_selection_Flag==0)  
              {
                MX_TIM2_Init_100();                     //init timer_2 
                __HAL_TIM_CLEAR_IT(&htim2, TIM_IT_UPDATE);//Clear interrupt FUCKING BEATCH!!!
                HAL_TIM_Base_Start_IT(&htim2);          //start timer_2              
              }
            }
        }
            HAL_TIM_Base_Stop_IT(&htim2);          //stop the timer2
            __HAL_TIM_CLEAR_IT(&htim2, TIM_IT_UPDATE);//Clear interrupt FUCKING BEATCH!!!
      }

static void MX_TIM2_Init_100(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 7199;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 500;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
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

        void menu_selection_mode(void)
    {
        unsigned int press_duration=0;
        while((HAL_GPIO_ReadPin(GPIOB, Button4) == 1)&&(press_duration<4294967295))//how long the button is pressing?
        {
          ++press_duration;
          if(press_duration>4090000)//alarm off
            {
              numbers_selection_signal();//Звук отключения будильника
              numbers_selection_signal();//
              numbers_selection_signal();//
              numbers_selection_signal();//              
              alarm_on_off=alarm_off;//disable the alarm
              break;
            }
        }
        
        if((press_duration>1030000)&&(press_duration<=4090000))//alarm setting
          {
            DesHrs=user_alarm_Time.Hours/10;//восстаноаление цифр
            EdHrs=user_alarm_Time.Hours%10;
            DesMin=user_alarm_Time.Minutes/10;
            EdMin=user_alarm_Time.Minutes%10;
            hours=DesHrs*10+EdHrs;
            minutes=DesMin*10+EdMin;
            seconds=0;
            time_shaper(hours,minutes,seconds);
            write_time_register(buffer);
            
            numbers_selection_signal();//Звук начала установки будильника
            numbers_selection_signal();//a single sound signal
            numbers_selection();//alarm setting  
            
            user_alarm_Time.Seconds=seconds;
            user_alarm_Time.Minutes=minutes;
            user_alarm_Time.Hours=hours;
            alarm_on_off=alarm_on;//enable the alarm
            numbers_selection_signal();//Звук выхода из установки будильника
            numbers_selection_signal();
          }                                                   
        else if(press_duration<=1030000)
          {
            DesHrs=hours/10;//восстаноаление цифр
            EdHrs=hours%10;
            DesMin=minutes/10;
            EdMin=minutes%10;
            
            numbers_selection_signal();//Звук начала установки времени
            numbers_selection();
            //###########################################SET TIME
            sTime.Seconds=seconds;    
            sTime.Minutes=minutes;     
            sTime.Hours=hours;
            HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BIN);
            
            time_shaper(hours,minutes,seconds);
            time = buffer;
            write_time_register(time);
            numbers_selection_signal();//Звук выхода из установки времени
          }
        
          while(HAL_GPIO_ReadPin(GPIOB, Button4) == 1)//Ждать отжатия кнопки
          {;}            
          delay_without_interruptions(user_delay(250));//~250mS игнорирование дребезга контактов
    }

void disable_signal(void)
{
    //####################################################Sound signal   
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_SET);
    delay_without_interruptions(user_delay(200));
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_RESET);
    delay_without_interruptions(user_delay(600));
    
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_SET);
    delay_without_interruptions(user_delay(100));
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_RESET);
    delay_without_interruptions(user_delay(200));
    
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_SET);
    delay_without_interruptions(user_delay(100));
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_RESET);    

}
void numbers_selection_signal(void)
{  
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_SET);
    delay_without_interruptions(user_delay(80));
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_RESET);
    delay_without_interruptions(user_delay(80));
}

void start_the_alarm_signal(void)
{   
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_SET);
    delay_without_interruptions(user_delay(80));
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_RESET);
    delay_without_interruptions(user_delay(80));
    
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_SET);
    delay_without_interruptions(user_delay(80));
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_RESET);
    delay_without_interruptions(user_delay(200));
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
