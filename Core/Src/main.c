#include "stm32l073xx.h"
#include "core_cm0plus.h"
#include <string.h>
#include <stdio.h>
#include <math.h>
uint32_t SystemCoreClock = 2097152U;
void SystemInit (void){}
	
volatile uint32_t uwTick;
	void SysTick_Handler(){
		uwTick+=1;
	}
	void TIM2_us_init(void) {
    // Enable TIM2 clock (APB1)
    RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;

    // reset timer
    TIM2->CR1 = 0;
    TIM2->PSC = (SystemCoreClock / 1000000UL) - 1; // prescaler for 1 MHz -> 1 tick = 1 us
    TIM2->ARR = 0xFFFF; // max
    TIM2->EGR = TIM_EGR_UG;
    TIM2->CR1 |= TIM_CR1_CEN; // enable
}
void delay_us(uint16_t us)
{
    TIM6->CNT = 0;
    while (TIM6->CNT < us);
}
	void delayMicroseconds(uint32_t us)
{
    uint32_t startTick = SysTick->VAL;  // Ð?c giá tr? hi?n t?i c?a SysTick
    uint32_t ticks_per_us = SystemCoreClock / 1000000U; // s? chu k? trong 1 µs
    uint32_t load = SysTick->LOAD + 1;  // Giá tr? d?m t?i da
    uint32_t us_ticks = us * ticks_per_us; // t?ng s? tick c?n ch?

    uint32_t elapsedTicks = 0;
    uint32_t oldTick = startTick;

    while (elapsedTicks < us_ticks)
    {
        uint32_t newTick = SysTick->VAL;
        if (newTick <= oldTick)
            elapsedTicks += (oldTick - newTick);
        else
            elapsedTicks += (load - newTick + oldTick);
        oldTick = newTick;
    }
}
		#define DHT22_PORT GPIOA
#define DHT22_PIN  (0x0002U)
	
	
	typedef enum
{
  GPIO_PIN_RESET = 0U,
  GPIO_PIN_SET
} GPIO_PinState;
	GPIO_PinState HAL_GPIO_ReadPin(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin)
{
  GPIO_PinState bitstatus;

  if ((GPIOx->IDR & GPIO_Pin) != (uint32_t)GPIO_PIN_RESET)
  {
    bitstatus = GPIO_PIN_SET;
  }
  else
  {
    bitstatus = GPIO_PIN_RESET;
  }
  return bitstatus;
}
typedef struct
{
  uint32_t Pin;       /*!< Specifies the GPIO pins to be configured.
                           This parameter can be a combination of @ref GPIO_pins_define */

  uint32_t Mode;      /*!< Specifies the operating mode for the selected pins.
                           This parameter can be a value of @ref GPIO_mode_define */

  uint32_t Pull;      /*!< Specifies the Pull-up or Pull-Down activation for the selected pins.
                           This parameter can be a value of @ref GPIO_pull_define */

  uint32_t Speed;     /*!< Specifies the speed for the selected pins.
                           This parameter can be a value of @ref GPIO_speed_define */

  uint32_t Alternate;  /*!< Peripheral to be connected to the selected pins
                            This parameter can be a value of @ref GPIOEx_Alternate_function_selection */
} GPIO_InitTypeDef;
#define GPIO_MODE_Pos                           0U
#define GPIO_MODE                               (0x3UL << GPIO_MODE_Pos)
#define MODE_INPUT                              (0x0UL << GPIO_MODE_Pos)
#define MODE_OUTPUT                             (0x1UL << GPIO_MODE_Pos)
#define MODE_AF                                 (0x2UL << GPIO_MODE_Pos)
#define MODE_ANALOG                             (0x3UL << GPIO_MODE_Pos)
#define OUTPUT_TYPE_Pos                         4U
#define OUTPUT_TYPE                             (0x1UL << OUTPUT_TYPE_Pos)
#define OUTPUT_PP                               (0x0UL << OUTPUT_TYPE_Pos)
#define  GPIO_MODE_INPUT                        MODE_INPUT                                                  /*!< Input Floating Mode                   */
#define  GPIO_MODE_OUTPUT_PP                    (MODE_OUTPUT | OUTPUT_PP) 
#define  GPIO_SPEED_FREQ_LOW              (0x00000000U)  
#define EXTI_MODE_Pos                           16U
#define EXTI_MODE                               (0x3UL << EXTI_MODE_Pos)
#define EXTI_IT                                 (0x1UL << EXTI_MODE_Pos)
#define EXTI_EVT                                (0x2UL << EXTI_MODE_Pos)
#define SET_BIT(REG, BIT)     ((REG) |= (BIT))
#define CLEAR_BIT(REG, BIT)   ((REG) &= ~(BIT))
#define __HAL_RCC_SYSCFG_CLK_ENABLE()   SET_BIT(RCC->APB2ENR, (RCC_APB2ENR_SYSCFGEN))
#define GPIO_GET_INDEX(__GPIOx__)    (((__GPIOx__) == (GPIOA))? 0U :\
                                      ((__GPIOx__) == (GPIOB))? 1U :\
                                      ((__GPIOx__) == (GPIOC))? 2U :\
                                      ((__GPIOx__) == (GPIOD))? 3U :\
                                      ((__GPIOx__) == (GPIOE))? 4U :\
                                      ((__GPIOx__) == (GPIOH))? 5U : 6U)
																			
#define TRIGGER_MODE_Pos                         20U
#define TRIGGER_MODE                            (0x7UL << TRIGGER_MODE_Pos)
#define TRIGGER_RISING                          (0x1UL << TRIGGER_MODE_Pos)
#define TRIGGER_FALLING                         (0x2UL << TRIGGER_MODE_Pos)
#define  GPIO_NOPULL        (0x00000000U) 
void HAL_GPIO_Init(GPIO_TypeDef  *GPIOx, GPIO_InitTypeDef *GPIO_Init)
{
  uint32_t position = 0x00U;
  uint32_t iocurrent = 0x00U;
  uint32_t temp = 0x00U;


  /* Configure the port pins */
  while (((GPIO_Init->Pin) >> position) != 0)
  {
    /* Get the IO position */
    iocurrent = (GPIO_Init->Pin) & (1U << position);

    if (iocurrent)
    {
      /*--------------------- GPIO Mode Configuration ------------------------*/
      /* In case of Output or Alternate function mode selection */
      if (((GPIO_Init->Mode & GPIO_MODE) == MODE_OUTPUT) ||
          ((GPIO_Init->Mode & GPIO_MODE) == MODE_AF))
      {

        /* Configure the IO Speed */
        temp = GPIOx->OSPEEDR;
        temp &= ~(GPIO_OSPEEDER_OSPEED0 << (position * 2U));
        temp |= (GPIO_Init->Speed << (position * 2U));
        GPIOx->OSPEEDR = temp;

        /* Configure the IO Output Type */
        temp = GPIOx->OTYPER;
        temp &= ~(GPIO_OTYPER_OT_0 << position) ;
        temp |= (((GPIO_Init->Mode & OUTPUT_TYPE) >> OUTPUT_TYPE_Pos) << position);
        GPIOx->OTYPER = temp;
      }

      if ((GPIO_Init->Mode & GPIO_MODE) != MODE_ANALOG)
      {


        /* Activate the Pull-up or Pull down resistor for the current IO */
        temp = GPIOx->PUPDR;
        temp &= ~(GPIO_PUPDR_PUPD0 << (position * 2U));
        temp |= ((GPIO_Init->Pull) << (position * 2U));
        GPIOx->PUPDR = temp;
      }

      /* In case of Alternate function mode selection */
      if ((GPIO_Init->Mode & GPIO_MODE) == MODE_AF)
      {


        /* Configure Alternate function mapped with the current IO */
        temp = GPIOx->AFR[position >> 3U];
        temp &= ~(0xFUL << ((uint32_t)(position & 0x07UL) * 4U));
        temp |= ((uint32_t)(GPIO_Init->Alternate) << (((uint32_t)position & (uint32_t)0x07U) * 4U));
        GPIOx->AFR[position >> 3U] = temp;
      }

      /* Configure IO Direction mode (Input, Output, Alternate or Analog) */
      temp = GPIOx->MODER;
      temp &= ~(GPIO_MODER_MODE0 << (position * 2U));
      temp |= ((GPIO_Init->Mode & GPIO_MODE) << (position * 2U));
      GPIOx->MODER = temp;

      /*--------------------- EXTI Mode Configuration ------------------------*/
      /* Configure the External Interrupt or event for the current IO */
      if ((GPIO_Init->Mode & EXTI_MODE) != 0x00U)
      {
        /* Enable SYSCFG Clock */
        __HAL_RCC_SYSCFG_CLK_ENABLE();

        temp = SYSCFG->EXTICR[position >> 2U];
        CLEAR_BIT(temp, (0x0FUL) << (4U * (position & 0x03U)));
        SET_BIT(temp, (GPIO_GET_INDEX(GPIOx)) << (4 * (position & 0x03U)));
        SYSCFG->EXTICR[position >> 2U] = temp;

        /* Clear Rising Falling edge configuration */
        temp = EXTI->RTSR;
        temp &= ~((uint32_t)iocurrent);
        if ((GPIO_Init->Mode & TRIGGER_RISING) != 0x00U)
        {
          temp |= iocurrent;
        }
        EXTI->RTSR = temp;

        temp = EXTI->FTSR;
        temp &= ~((uint32_t)iocurrent);
        if ((GPIO_Init->Mode & TRIGGER_FALLING) != 0x00U)
        {
          temp |= iocurrent;
        }
        EXTI->FTSR = temp;

        temp = EXTI->EMR;
        temp &= ~((uint32_t)iocurrent);
        if ((GPIO_Init->Mode & EXTI_EVT) != 0x00U)
        {
          temp |= iocurrent;
        }
        EXTI->EMR = temp;

        /* Clear EXTI line configuration */
        temp = EXTI->IMR;
        temp &= ~((uint32_t)iocurrent);
        if ((GPIO_Init->Mode & EXTI_IT) != 0x00U)
        {
          temp |= iocurrent;
        }
        EXTI->IMR = temp;
      }
    }
    position++;
  }
}
void HAL_GPIO_WritePin(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin, GPIO_PinState PinState)
{


  if (PinState != GPIO_PIN_RESET)
  {
    GPIOx->BSRR = GPIO_Pin;
  }
  else
  {
    GPIOx->BRR = GPIO_Pin ;
  }
}
void Set_Pin_Output(GPIO_TypeDef *GPIOx,uint16_t GPIO_Pin)
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};
	GPIO_InitStruct.Pin = GPIO_Pin;
GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOx, &GPIO_InitStruct);
}
void Set_Pin_Input(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin)
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};
	GPIO_InitStruct.Pin = GPIO_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOx, &GPIO_InitStruct);
}

void DHT22_Start (void)
{
	Set_Pin_Output(DHT22_PORT, DHT22_PIN); // set the pin as output
	HAL_GPIO_WritePin (DHT22_PORT, DHT22_PIN, 0);   // pull the pin low
	delay_us(1200);   // wait for > 1ms

	HAL_GPIO_WritePin (DHT22_PORT, DHT22_PIN, 1);   // pull the pin high
	delay_us (20);   // wait for 30us
	Set_Pin_Input(DHT22_PORT, DHT22_PIN);   // set as input
}
uint8_t DHT22_Check_Response (void)
{
	Set_Pin_Input(DHT22_PORT, DHT22_PIN);   // set as input
	uint8_t Response = 0;
	delay_us (40);  // wait for 40us
	if (!(HAL_GPIO_ReadPin (DHT22_PORT, DHT22_PIN))) // if the pin is low
	{
		delay_us (80);   // wait for 80us

		if ((HAL_GPIO_ReadPin (DHT22_PORT, DHT22_PIN))) Response = 1;  // if the pin is high, response is ok
		else Response = -1;
	}

	while ((HAL_GPIO_ReadPin (DHT22_PORT, DHT22_PIN)));   // wait for the pin to go low
	return Response;
}
uint8_t DHT22_Read (void)
{
	uint8_t i,j;
	for (j=0;j<8;j++)
	{
		while (!(HAL_GPIO_ReadPin (DHT22_PORT, DHT22_PIN)));   // wait for the pin to go high
		delay_us (40);   // wait for 40 us

		if (!(HAL_GPIO_ReadPin (DHT22_PORT, DHT22_PIN)))   // if the pin is low
		{
			i&= ~(1<<(7-j));   // write 0
		}
		else i|= (1<<(7-j));  // if the pin is high, write 1
		while ((HAL_GPIO_ReadPin (DHT22_PORT, DHT22_PIN)));  // wait for the pin to go low
	}

	return i;
}

void ADC1_Init(void)
{
    // 1?? B?t clock cho GPIOA và ADC1
    RCC->IOPENR |= (1 << 0);     // GPIOA clock enable
    RCC->APB2ENR |= (1 << 9);    // ADC1 clock enable

    // 2?? B?t HSI16 làm ngu?n clock ADC (m?c d?nh ADC dùng HSI16)
    RCC->CR |= RCC_CR_HSION;          // B?t HSI16
    while (!(RCC->CR & RCC_CR_HSIRDY)); // Ch? HSI16 s?n sàng

    // 3?? C?u hình PA0 làm Analog (ADC_IN0)
    GPIOA->MODER |= (0b11 << (0 * 2));  // Analog mode
    GPIOA->PUPDR &= ~(0b11 << (0 * 2)); // No pull-up/pull-down

    // 4?? Ð?m b?o ADC t?t tru?c khi hi?u chu?n
    if (ADC1->CR & ADC_CR_ADEN)
    {
        ADC1->CR |= ADC_CR_ADDIS;
        while (ADC1->CR & ADC_CR_ADEN);
    }

    // 5?? Hi?u chu?n ADC
    ADC1->CR |= ADC_CR_ADCAL;
    while (ADC1->CR & ADC_CR_ADCAL); // Ch? hi?u chu?n xong

    // 6?? B?t ADC
    ADC1->CR |= ADC_CR_ADEN;
    while (!(ADC1->ISR & ADC_ISR_ADRDY)); // Ð?i ADC s?n sàng

    // 7?? Ch?n kênh PA0 (ADC_IN0)
    ADC1->CHSELR = ADC_CHSELR_CHSEL0;

    // 8?? Ch?n th?i gian l?y m?u dài (?n d?nh hon)
    ADC1->SMPR |= ADC_SMPR_SMP_2;
}


uint16_t ADC1_Read(void)
{
    ADC1->CR |= ADC_CR_ADSTART;           // B?t d?u chuy?n d?i
    while (!(ADC1->ISR & ADC_ISR_EOC));   // Ch? chuy?n d?i xong
    return (uint16_t)ADC1->DR;            // Tr? v? giá tr? 12-bit
}

void PWM_Init(void)
{
    // 1?? B?t clock cho GPIOB và TIM2
    RCC->IOPENR |= (1 << 1);    // GPIOB clock enable
    RCC->APB1ENR |= (1 << 0);   // TIM2 clock enable

    // 2?? C?u hình PB3 làm Alternate Function (AF2: TIM2_CH2)
    GPIOB->MODER &= ~(0b11 << (3 * 2));
    GPIOB->MODER |=  (0b10 << (3 * 2));   // AF mode
    GPIOB->AFR[0]  &= ~(0xF << (3 * 4));
    GPIOB->AFR[0]  |=  (2 << (3 * 4));    // AF2 for TIM2_CH2

    // 3?? C?u hình Timer2 cho PWM mode 1 (CH2)
    TIM2->PSC = (SystemCoreClock / 1000000U) - 1;  // prescale d? timer tick = 1 MHz
    TIM2->ARR = 1000 - 1;     // chu k? PWM = 1000 µs = 1 kHz
    TIM2->CCR2 = 500;         // duty 50%

    // PWM Mode 1 (OC2M = 110)
    TIM2->CCMR1 &= ~(7 << 12);
    TIM2->CCMR1 |=  (6 << 12);
    TIM2->CCMR1 |=  (1 << 11);     // preload enable

    TIM2->CCER  |=  (1 << 4);      // enable output CH2
    TIM2->CR1   |=  (1 << 7);      // ARPE enable
    TIM2->EGR   |=  (1 << 0);      // update event
    TIM2->CR1   |=  (1 << 0);      // start counter
}

typedef struct {
    float setpoint;       // Giá tr? m?c tiêu
    float gain;           // Ð? nh?y (tuong t? Kp)
		float ki;             // Ki (M?i thêm)
		float integral;       // Tích luy l?i (M?i thêm)
    float dead_zone;      // Vùng ch?t (không ph?n ?ng n?u error nh?)
    float output_min;
    float output_max;
    float offset;         // Offset output (giá tr? baseline)
} Tanh_Controller;

void Tanh_Init(Tanh_Controller *ctrl, float setpoint, float gain, float ki,
               float dead_zone, float out_min, float out_max, float offset)
{
    ctrl->setpoint = setpoint;
    ctrl->gain = gain;
		ctrl->ki = ki;            // M?i
		ctrl->integral = 0.0f;    // Reset tích luy
    ctrl->dead_zone = dead_zone;
    ctrl->output_min = out_min;
    ctrl->output_max = out_max;
    ctrl->offset = offset;
}

float Tanh_Compute(Tanh_Controller *ctrl, float measured_value)
{
    float error = ctrl->setpoint - measured_value;
    
    // Áp d?ng vùng ch?t
    if (fabs(error) < ctrl->dead_zone) {
        error = 0.0f;
    }
    ctrl->integral += error * ctrl->ki;
		// Ch?ng bão hòa tích phân (Anti-windup):
    // Gi?i h?n integral d? nó không c?ng d?n quá l?n gây v?t l?
    float max_integral = (ctrl->output_max - ctrl->output_min) / 2.0f; 
    if (ctrl->integral > max_integral) ctrl->integral = max_integral;
    if (ctrl->integral < -max_integral) ctrl->integral = -max_integral;
    // Hàm tanh: smooth, bounded, t? nhiên
    // tanh(x) cho output t? -1 d?n +1
    float tanh_output = tanh(ctrl->gain * error);
    
    // Scale v? range mong mu?n
    float range = (ctrl->output_max - ctrl->output_min) / 2.0f;
    float output = ctrl->offset + range * tanh_output + ctrl->integral;
    
    // Ð?m b?o trong gi?i h?n
    if (output > ctrl->output_max) output = ctrl->output_max;
    if (output < ctrl->output_min) output = ctrl->output_min;
    
    return output;
}
Tanh_Controller light_tanh;

void LightControl_Init(uint16_t target)
{
    // gain=0.002: error ±500 ADC ? tanh(±1) ? output ±76%
    Tanh_Init(&light_tanh, (float)target, 0.002f, 0.05f, 0.0f, 0.0f, 1000.0f, 500.0f);
}
// Thêm vào d?u file
float adc_filtered = 0.0f;
uint8_t adc_initialized = 0;

uint16_t ADC1_Read_EMA(void)
{
    uint16_t raw = ADC1_Read();
    
    if (!adc_initialized) {
        adc_filtered = (float)raw;
        adc_initialized = 1;
        return raw;
    }
    
    // alpha nh? = l?c m?nh = mu?t hon
    float alpha = 0.2f;  // Gi?m t? 0.3 xu?ng 0.2
    adc_filtered = alpha * raw + (1.0f - alpha) * adc_filtered;
    
    return (uint16_t)adc_filtered;
}
//UART_HandleTypeDef huart2;
uint8_t tx_buffer[27]="Welcome\n\r";
uint8_t rx_indx;
char rx_data[8];
char rx_buffer[100];
char *str = rx_data;
char *cmd = rx_buffer;
uint8_t Rh_byte1,Rh_byte2,Temp_byte1,Temp_byte2;
uint16_t SUM,TEMP,RH;
float Temperature=0;
float Humidity=0;
uint8_t Presence=0;
void TIM6_Init(void)
{
    RCC->APB1ENR |= RCC_APB1ENR_TIM6EN;
    TIM6->PSC = (SystemCoreClock / 1000000U) - 1; // prescaler d? 1 tick = 1 µs
    TIM6->ARR = 0xFFFF;
    TIM6->CR1 |= TIM_CR1_CEN;
}
char c = 0;   // khai báo ngoài
uint8_t mode = 1;
int main(void)
{	
	TIM2_us_init();
	TIM6_Init();
	SysTick->LOAD = (SystemCoreClock / 1000) - 1;
	SysTick->VAL = 0;
	SysTick->CTRL=0b111;
	
  RCC->IOPENR |=1;
	RCC->APB1ENR |= (1 << 17); 
	RCC->APB2ENR |= (1 << 14); //USART1
	GPIOA->MODER &= ~((0b11 <<(5*2))|(0b11 << (6*2))|(0b11 << (2*2))|(0b11 << (3*2))|(0b11 << (9*2))|(0b11 << (10*2)));
	GPIOA->MODER |= (1<<(5*2))|(1 << (6*2))|(0b10 << (2*2))|(0b10 <<(3*2))|(0b10 << (9*2))|(0b10 << (10*2));
	// Ð?m b?o không có pull-up trên PA5
GPIOA->PUPDR &= ~(0b11 << (5*2));  // No pull-up, no pull-down
GPIOA->AFR[0] |= (4 << (2 * 4)) | (4 << (3 * 4));
	GPIOA->AFR[1] &= ~((0xF << ((9-8)*4)) | (0xF << ((10-8)*4)));
GPIOA->AFR[1] |=  (4 << ((9-8)*4)) | (4 << ((10-8)*4)); // AF4 = USART1
	USART1->BRR = SystemCoreClock / 9600;
USART1->CR1 = (1<<3) | (1<<2) | 1 | 1<<5;
	NVIC->ISER[0] = (1 << USART1_IRQn); // Kích ho?t ng?t USART2
USART2->BRR = SystemCoreClock / 9600;
USART2->CR1 = (1<<3) | (1<<2) | 1 | 1<<5;
NVIC->ISER[0] = (1 << USART2_IRQn); // Kích ho?t ng?t USART2
		float temp, hum;
		char text[32];
	char t[32];
	ADC1_Init();
PWM_Init();
LightControl_Init(2000);
GPIOA->BRR = 1 << 5;
	uint32_t last = 0;
  //HAL_TIM_Base_Start(&htim6);  // for us Delay
//  uint32_t s = uwTick;
//		while((uwTick - s) <2000){}
	
  while (1)
  {
		//DHT22_ReadData(&temp, &hum);

    //sprintf(text, "T=%.1fC, H=%.1f%%\r\n", temp, hum);
//		
		if(uwTick - last >= 2000){
		DHT22_Start();
      Presence = DHT22_Check_Response();
      Rh_byte1 = DHT22_Read ();
      Rh_byte2 = DHT22_Read ();
      Temp_byte1 = DHT22_Read ();
      Temp_byte2 = DHT22_Read ();
      SUM = DHT22_Read();

      TEMP = ((Temp_byte1<<8)|Temp_byte2);
      RH = ((Rh_byte1<<8)|Rh_byte2);

      Temperature = (float) (TEMP/10.0);
      Humidity = (float) (RH/10.0);
			last = uwTick;
		}
      //HAL_Delay(2000);
			//delay_us(1000);   // <--- thêm dòng này
			uint16_t adc_val = ADC1_Read_EMA();
		sprintf(text, "TEMP=%.1f; HUMI=%.1f; ADC=%u\r\n", Temperature, Humidity, adc_val);
		GPIOA->BSRR = 1 << 5;
		//delay_us(20);            // Delay cho IC RS485
		
    for (int i = 0; i < strlen(text); i++) {
        USART2->TDR = text[i];
        while (!(USART2->ISR & (1 << 7))); // ch? TDR tr?ng
				USART1->TDR = text[i];
        while (!(USART1->ISR & (1 << 7))); // ch? TDR tr?ng
    }
		while (!(USART2->ISR & (1 << 6)));     // Ch? TC (transmission complete)
        
        //delay_us(50);              // Ð?m b?o byte cu?i ra kh?i shift register
		GPIOA->BRR = 1 << 5;
		//uint32_t z = uwTick;
		//while ((uwTick - z) < 500){}
if (mode == 1){
uint16_t pwm_duty = (uint16_t)Tanh_Compute(&light_tanh, (float)adc_val);
TIM2->CCR2 = pwm_duty;
}
//if (USART2->ISR & (1 << 5)) {
//    c = USART2->RDR;  // d?c d? li?u
//	GPIOA->BRR = 1 << 4;
//}
//else {
//	GPIOA->BSRR = 1 << 4;
//}
//if (USART1->ISR & (1 << 5)) {
//    c = USART1->RDR;  // d?c d? li?u
//}
//if (c == '1') {
//    GPIOA->BSRR = 1 << 5; // bat LED
//}
//else if (c == '0') {
//    GPIOA->BRR = 1 << 5; // tat LED
//}
//else if(c == 'r') {
//	GPIOA->BRR = 1 << 4;
//}
//else if (c == 't') {
//	GPIOA->BSRR = 1 << 4;
//}

		
//		//delay_us(2000);
//GPIOA->BSRR = 1 << 5; // Set
//delay_us(65000);
////		uint32_t start = uwTick;
////		while((uwTick - start) <65){}
//			GPIOA->BRR = 1<<5 ;//reset
//delay_us(65000);
//			uint32_t s = uwTick;
//		while((uwTick - s) <65){}
			
		//GPIOA->ODR ^= (1<<5);
		uint32_t start = uwTick;
		while((uwTick - start) <500){}
  }
}
void USART1_IRQHandler(void)
{
	c = USART1->RDR;
	TIM2->CCR2 = (c - '0')*100;
	if (c == '1') {
    GPIOA->BSRR = 1 << 6; // bat LED
}
else if (c == '0') {
    GPIOA->BRR = 1 << 6; // tat LED
}
else if (c == 'm'){
		mode = 0;
}
else if (c == 'a') {
		mode = 1;
}
	}
void USART2_IRQHandler(void)
{
	c = USART2->RDR;
	TIM2->CCR2 = (c - '0')*100;
	if (c == '1') {
    GPIOA->BSRR = 1 << 6; // bat LED
}
else if (c == '0') {
    GPIOA->BRR = 1 << 6; // tat LED
}
		
	}

/*
void USART2_IRQHandler(void)
{
    
   *str = USART2->RDR;
	uint8_t i;
	  if(rx_indx == 0){
		for(i=0;i<100;i++)
			rx_buffer[i]=0;
			GPIOA->BRR = 1<<5 ;//reset
		}
		if(*str != 13){ // rxdata[0] !=13
		   rx_buffer[rx_indx++] = rx_data[0];
		}
		else{
		rx_indx = 0;
			USART2->TDR = *"\n\r";//Transmit
			const char *cmd = "LED ON";
			char *ptr = rx_buffer;  
			while (*ptr && (*ptr == *cmd)) { 
				ptr++;
				cmd++;
			}
			if (*cmd == '\0') {  // N?u duy?t h?t "LED ON" mà không có sai khác
				GPIOA->BSRR = 1 << 5; // Set
			}
		}	
		USART2->TDR = *str;//Transmit
}
*/