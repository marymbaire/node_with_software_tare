/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "lora.h"
#include "Flash.h"
#include <string.h>
#include <stdio.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define FLASH_LAST_BYTE_ADDRESS 0x0800FFFF
#define MAX_MESSAGES 50 // Define the maximum number of messages to store
#define R1 10000.0  // Resistance value in ohms
#define R2 10000.0 // Resistance value in ohms
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

IWDG_HandleTypeDef hiwdg;

RTC_HandleTypeDef hrtc;

SPI_HandleTypeDef hspi2;

TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
int packetReceived=0;
int tare_reset=0;
uint32_t tare_received=0;
int milligram;
char previous_state = '0';  // Initial previous state
lora_pins_t lora_pins;	    // Structure variable for lora pins
lora_t lora;				// Structure variable for lora
// Define global variables for tare and coefficient
uint32_t tare = 0;
float coefficient = 1.0;
float coefficient1;
uint8_t ret;
float weight, prev_weight, difference;
volatile uint8_t tare_button_pressed = 0;
float weight_tare=0.0;
float battery_percentage;
float Vout;
float Vin;
char uid_str[40];
char msg[256];
char msg8[256];
char buf[256];
char buff[256];
uint16_t count =0;
uint16_t messageCount = 0;
int gateway_available=1;
uint32_t main_counter = 0;
uint32_t rtc_counter = 0;
uint32_t function_counter=0;
volatile int rtc_check=0;
typedef struct {
    char message[256];
} Message;
Message messageQueue[MAX_MESSAGES];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_RTC_Init(void);
static void MX_SPI2_Init(void);
static void MX_TIM2_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */
static void MX_IWDG_Init(void);
void calibrate(void);
uint32_t HX711_Tare(void); // Function prototype for HX711_Tare
void readTarebutton(void);
float HX711_GetWeight(uint8_t times);
void Enter_Stop_Mode(void);
void check_gateway(void);
void store_message(const char* msg);
void send_stored_messages(void);
static void MX_ADC1_Init(void);
void checkbatlevel(void);
void senddata(void);
void send_weight(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void microDelay(uint16_t delay)
{
  __HAL_TIM_SET_COUNTER(&htim2, 0);
  while (__HAL_TIM_GET_COUNTER(&htim2) < delay);
}

int32_t getHX711(void)
{
	  uint32_t data = 0;
	  uint32_t startTime = HAL_GetTick();
	  while(HAL_GPIO_ReadPin(GPIOB, DT_Pin) == GPIO_PIN_SET)
	  {
	    if(HAL_GetTick() - startTime > 200)
	      return 0;
	  }
	  for(int8_t len=0; len<24 ; len++)
	  {
	    HAL_GPIO_WritePin(GPIOB, SCK_Pin, GPIO_PIN_SET);
	    microDelay(1);
	    data = data << 1;
	    HAL_GPIO_WritePin(GPIOB, SCK_Pin, GPIO_PIN_RESET);
	    microDelay(1);
	    if(HAL_GPIO_ReadPin(GPIOB, DT_Pin) == GPIO_PIN_SET)
	      data ++;
	  }
	  data = data ^ 0x800000;
	  HAL_GPIO_WritePin(GPIOB, SCK_Pin, GPIO_PIN_SET);
	  microDelay(1);
	  HAL_GPIO_WritePin(GPIOB, SCK_Pin, GPIO_PIN_RESET);
	  microDelay(1);
	  return data;
}

uint32_t HX711_Tare() {
    for (int i = 0; i < 2; i++) {
        HAL_GPIO_TogglePin(GPIOB, RED_Pin);
        HAL_Delay(100);
        HAL_GPIO_TogglePin(GPIOB, RED_Pin);
        HAL_Delay(100);
    }
    for (int i = 0; i < 2; i++) {
        HAL_GPIO_TogglePin(GPIOB, GREEN_Pin);
        HAL_Delay(100);
        HAL_GPIO_TogglePin(GPIOB, GREEN_Pin);
        HAL_Delay(100);
    }
    for (int i = 0; i < 2; i++) {
        HAL_GPIO_TogglePin(GPIOB, BLUE_Pin);
        HAL_Delay(100);
        HAL_GPIO_TogglePin(GPIOB, BLUE_Pin);
        HAL_Delay(100);
    }

	int32_t total = 0;
	char msg1[] = "Taring in progress.....\r\n";
	HAL_UART_Transmit(&huart1, (uint8_t *)msg1-1, sizeof(msg1), HAL_MAX_DELAY);
	int32_t samples = 50;
	for(uint16_t i = 0; i < samples; i++) {
		total += getHX711();
	}
	uint32_t average = (int32_t)(total / samples);
	return average;
}

int32_t read_average(int8_t times)
{
  if (times < 1) times = 1;
  int32_t sum = 0;
  for (uint8_t i = 0; i < times; i++)
  {
    sum += getHX711();
  }
  return sum / times;
}


float HX711_GetWeight(uint8_t times) {
  int32_t  total = 0;
  for(uint16_t i=0 ; i<times ; i++)
  {
	  total += getHX711();
  }
  int32_t average = (int32_t)(total / times);
  milligram = (int)(average-tare)*coefficient1;
  return milligram;
}

void startup(){
    for (int i = 0; i < 2; i++) {
        HAL_GPIO_TogglePin(GPIOB, RED_Pin);
        HAL_Delay(100);
        HAL_GPIO_TogglePin(GPIOB, RED_Pin);
        HAL_Delay(100);
    }
    for (int i = 0; i < 2; i++) {
        HAL_GPIO_TogglePin(GPIOB, GREEN_Pin);
        HAL_Delay(100);
        HAL_GPIO_TogglePin(GPIOB, GREEN_Pin);
        HAL_Delay(100);
    }
    for (int i = 0; i < 2; i++) {
        HAL_GPIO_TogglePin(GPIOB, BLUE_Pin);
        HAL_Delay(100);
        HAL_GPIO_TogglePin(GPIOB, BLUE_Pin);
        HAL_Delay(100);
    }

    HAL_GPIO_WritePin(GPIOB, RED_Pin, GPIO_PIN_SET);
	HAL_Delay(6000); // Place weight
	HAL_GPIO_WritePin(GPIOB, RED_Pin, GPIO_PIN_RESET);
	HAL_Delay(3000);
}

void calibrate(void) {
	HAL_IWDG_Refresh(&hiwdg); // Refresh at the start
	HAL_GPIO_WritePin(GPIOA, HX711_ACT_Pin, GPIO_PIN_SET);
	HAL_Delay(1000);
    HAL_GPIO_WritePin(GPIOB, GREEN_Pin, GPIO_PIN_SET);
    HAL_Delay(5000);
    HAL_GPIO_WritePin(GPIOB, GREEN_Pin, GPIO_PIN_RESET);
    HAL_Delay(2000);
    HAL_IWDG_Refresh(&hiwdg); // Refresh at the start
    // Tare the scale
    tare= HX711_Tare();
    HAL_IWDG_Refresh(&hiwdg); // Refresh at the start
    Flash_Write_Data(0x0801F800,&tare,1);
    char msg1[] = "Calibrating.....\r\n";
    HAL_UART_Transmit(&huart1, (uint8_t *)msg1, sizeof(msg1)-1, HAL_MAX_DELAY);
    HAL_IWDG_Refresh(&hiwdg); // Refresh at the start
    // Place known weights for calibration
    const int weights[] = {0, 50000, 100000, 200000, 500000, 1000000, 2000000};
    //float sum_coefficients = 0.0;
    int num_calibrations = sizeof(weights) / sizeof(weights[0]);

    for (int i = 0; i < num_calibrations; i++) {
        for (int j = 0; j < 4; j++) {
            HAL_GPIO_TogglePin(GPIOB, BLUE_Pin);
            HAL_Delay(250);
        }
        HAL_GPIO_WritePin(GPIOB, BLUE_Pin, GPIO_PIN_SET);
        HAL_Delay(6000); // Place weight
        HAL_GPIO_WritePin(GPIOB, BLUE_Pin, GPIO_PIN_RESET);
        HAL_Delay(3000);
        int32_t currentvalue= (int32_t)read_average(50)-(int32_t)tare;
        float current_coefficient = 1.0;
        HAL_IWDG_Refresh(&hiwdg); // Refresh at the start
        if (weights[i] != 0) {
        	current_coefficient=((1*(float)weights[i])/currentvalue);
        	coefficient=current_coefficient;
        }
    }
    HAL_IWDG_Refresh(&hiwdg); // Refresh at the start
	char msg9[512];
    sprintf(msg9, "here is the coeff being stored to memory:%lu\r\n", coefficient);
	HAL_UART_Transmit(&huart1, (uint8_t*)msg9, strlen(msg9), HAL_MAX_DELAY);
	HAL_Delay(2000);

    // Calibration done indicator LEDs
    for (int i = 0; i < 2; i++) {
        HAL_GPIO_TogglePin(GPIOB, RED_Pin);
        HAL_Delay(100);
        HAL_GPIO_TogglePin(GPIOB, RED_Pin);
        HAL_Delay(100);
    }
    for (int i = 0; i < 2; i++) {
        HAL_GPIO_TogglePin(GPIOB, GREEN_Pin);
        HAL_Delay(100);
        HAL_GPIO_TogglePin(GPIOB, GREEN_Pin);
        HAL_Delay(100);
    }
    for (int i = 0; i < 2; i++) {
        HAL_GPIO_TogglePin(GPIOB, BLUE_Pin);
        HAL_Delay(100);
        HAL_GPIO_TogglePin(GPIOB, BLUE_Pin);
        HAL_Delay(100);
    }
    HAL_IWDG_Refresh(&hiwdg); // Refresh at the start
    Flash_Write_NUM (0x0801FC00, coefficient);
	char msg8[512];
    sprintf(msg8, "here is the coeff1 being stored to memory:%lu\r\n", coefficient);
	HAL_UART_Transmit(&huart1, (uint8_t*)msg8, strlen(msg8), HAL_MAX_DELAY);
	HAL_Delay(2000);
	HAL_GPIO_WritePin(GPIOA, HX711_ACT_Pin, GPIO_PIN_RESET);
	HAL_IWDG_Refresh(&hiwdg); // Refresh at the start

}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  if (GPIO_Pin == DIO0_Pin)
  {
	  __HAL_GPIO_EXTI_CLEAR_IT(GPIO_Pin);  // Clear the interrupt flag
		packetReceived = 1;
  }
}

void HAL_RTC_AlarmAEventCallback(RTC_HandleTypeDef *hrtc)
{
	rtc_check=1;
    // Clear the RTC Alarm A flag
    __HAL_RTC_ALARM_CLEAR_FLAG(hrtc, RTC_FLAG_ALRAF);
}

//void checkbatlevel(){
//	  // Start ADC conversion
//	  HAL_ADC_PollForConversion(&hadc1, 1000);
//	  HAL_IWDG_Refresh(&hiwdg);
//	  uint32_t ADC_value = HAL_ADC_GetValue(&hadc1);
//	  HAL_IWDG_Refresh(&hiwdg);
//	  // Convert ADC value to corresponding voltage
//	  Vout = (ADC_value / 4095.0) * 3.3; // assuming Vref is 3.3V
//	  // Calculate the actual battery voltage
//	  Vin = Vout * 2; // Since the voltage divider divides by 2
//	  HAL_IWDG_Refresh(&hiwdg);
//	  // Calculate battery percentage
//	  battery_percentage = ((Vin - 3.0) / (4.2 - 3.0)) * 100.0;
//	  HAL_IWDG_Refresh(&hiwdg);
//	  // Constrain the percentage between 0% and 100%
//	  if (battery_percentage < 0) battery_percentage = 0;
//	  if (battery_percentage > 100) battery_percentage = 100;
//	  // Debugging message
//	  char msg[512];
//	  sprintf(msg, "Current battery level: %.2f%%\r\n", battery_percentage);
//	  HAL_UART_Transmit(&huart1, (uint8_t *)msg, strlen(msg), 1000);
//}

void checkbatlevel() {
    // Start a new ADC conversion
    HAL_ADC_Start(&hadc1);

    // Wait for the ADC conversion to complete
    if (HAL_ADC_PollForConversion(&hadc1, 1000) == HAL_OK) {
        HAL_IWDG_Refresh(&hiwdg);

        // Read the ADC value
        uint32_t ADC_value = HAL_ADC_GetValue(&hadc1);

        // Convert ADC value to corresponding voltage
        Vout = (ADC_value / 4095.0) * 3.3; // assuming Vref is 3.3V

        // Calculate the actual battery voltage
        Vin = Vout * 2; // Since the voltage divider divides by 2

        // Calculate battery percentage
        battery_percentage = ((Vin - 3.0) / (4.2 - 3.0)) * 100.0;

        // Constrain the percentage between 0% and 100%
        if (battery_percentage < 0) battery_percentage = 0;
        if (battery_percentage > 100) battery_percentage = 100;

        // Debugging message
        char msg[512];
        sprintf(msg, "Current battery level: %.2f%%\r\n", battery_percentage);
        HAL_UART_Transmit(&huart1, (uint8_t *)msg, strlen(msg), 1000);
    }

    // Stop the ADC after reading
    HAL_ADC_Stop(&hadc1);
    HAL_IWDG_Refresh(&hiwdg);
}


void senddata(){
	  // Prepare weight data for LoRa transmission
	  sprintf(buf, "weight:%.2f% Kg, count:%d, node battery level: %.2f%%", weight,count,battery_percentage);
	  strcat(buf,uid_str);
	  // Temporarily disable the interrupt while transmitting
	  HAL_NVIC_DisableIRQ(EXTI15_10_IRQn);
	  lora_begin_packet(&lora);
	  lora_tx(&lora, (uint8_t *)buf, strlen(buf));
	  lora_end_packet(&lora);
	  sprintf(msg,"Sending packet %d\r\n",count);
	  HAL_UART_Transmit(&huart1,(uint8_t *)msg,strlen(msg),1000);
	  HAL_IWDG_Refresh(&hiwdg);
	  lora_start_receiving(&lora);
	  HAL_Delay(10);
	  check_gateway();
	  HAL_IWDG_Refresh(&hiwdg);
	  if(gateway_available==0){
		  sprintf(msg, "Storing packet %d\r\n", count);
		  HAL_UART_Transmit(&huart1, (uint8_t *)msg, strlen(msg), 1000);
		  store_message(buf);
		  count++; // Increment the counter after storing the message
		  HAL_IWDG_Refresh(&hiwdg);
	  }
	  else if(gateway_available==1){
		  send_stored_messages();
		  count++;
		  HAL_IWDG_Refresh(&hiwdg);
	  }
	  HAL_IWDG_Refresh(&hiwdg);
	  // Re-enable the interrupt after transmitting
	  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);
	  // Also transmit the message via UART for debugging
	  char uart_msg[50];
	  sprintf(uart_msg, "Transmitted successfully \r\n");
	  HAL_UART_Transmit(&huart1, (uint8_t*)uart_msg, strlen(uart_msg), HAL_MAX_DELAY);
}

void send_weight(){
	  HAL_IWDG_Refresh(&hiwdg);
	  HAL_GPIO_WritePin(GPIOA, HX711_ACT_Pin, GPIO_PIN_SET);
	  HAL_Delay(1000);
	  weight = HX711_GetWeight(50); // in milligram
	  weight=weight/1000000;
	  if(weight<=0.0){
		  weight=0.0;
	  }
	  if(prev_weight>weight){
		  difference = prev_weight - weight;
		  if(difference>=5.5){
			  tare= HX711_Tare(); // Call tare() function if drastic weight fluctuation
			  Flash_Write_Data(0x0801F800,&tare,1);
		  }
	  }
	  prev_weight= weight;
	  HAL_IWDG_Refresh(&hiwdg);
	  // Transmit weight over USART1
	  char weight_str[50];
	  sprintf(weight_str, "Weight:%.2f% Kg\r\n", weight);
	  HAL_UART_Transmit(&huart1, (uint8_t*)weight_str, strlen(weight_str), HAL_MAX_DELAY);
	  HAL_IWDG_Refresh(&hiwdg);
	  HAL_GPIO_WritePin(GPIOA, HX711_ACT_Pin, GPIO_PIN_RESET);
	  HAL_Delay(1000);
	  HAL_IWDG_Refresh(&hiwdg);
}

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
  MX_GPIO_Init();
  MX_ADC1_Init();
  MX_RTC_Init();
  MX_SPI2_Init();
  MX_TIM2_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
  sprintf(msg, "Initializing....\r\n");
  HAL_UART_Transmit(&huart1, (uint8_t*)msg, strlen(msg), 1000);

  MX_IWDG_Init();
  HAL_ADC_Start(&hadc1);
  lora_pins.dio0.port  = DIO0_GPIO_Port;
  lora_pins.dio0.pin   = DIO0_Pin;
  lora_pins.nss.port   = NSS_GPIO_Port;				// NSS pin to which port is connected
  lora_pins.nss.pin    = NSS_Pin;					// NSS pin to which pin is connected
  lora_pins.reset.port = RESET_GPIO_Port;			// RESET pin to which port is connected
  lora_pins.reset.pin  = RESET_Pin;			// RESET pin to which pin is connected
  lora_pins.spi  		= &hspi2;

  lora.pin = &lora_pins;
  lora.frequency = FREQ_866MHZ;								// 866MHZ Frequency

  sprintf(msg,"Configuring LoRa module\r\n");
  HAL_UART_Transmit(&huart1,(uint8_t *)msg,strlen(msg),1000);

  while(lora_init(&lora)){										// Initialize the lora module
	sprintf(msg,"LoRa Failed\r\n");
	HAL_UART_Transmit(&huart1,(uint8_t *)msg,strlen(msg),1000);
	HAL_Delay(1000);
  }
  sprintf(msg,"Done configuring LoRaModule\r\n");
  HAL_UART_Transmit(&huart1,(uint8_t *)msg,strlen(msg),1000);
  HAL_IWDG_Refresh(&hiwdg); // Refresh at the start
  HAL_GPIO_WritePin(GPIOA, HX711_ACT_Pin, GPIO_PIN_SET);
  HAL_Delay(1000);
  HAL_TIM_Base_Start(&htim2);
  HAL_GPIO_WritePin(GPIOB, SCK_Pin, GPIO_PIN_SET);
  HAL_Delay(10);
  HAL_GPIO_WritePin(GPIOB, SCK_Pin, GPIO_PIN_RESET);
  HAL_Delay(10);
  coefficient1=Flash_Read_NUM (0x0801FC00);
  sprintf(msg, "here is the coeff gotten from memory:%lu\r\n", coefficient1);
  HAL_UART_Transmit(&huart1, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
  HAL_IWDG_Refresh(&hiwdg); // Refresh at the start
  HAL_Delay(2000);
  /* Check if memory location is empty */
  uint32_t backupData;
  Flash_Read_Data(0x0801F800,&backupData,1);
  if (backupData == 0xFFFFFFFF || backupData == 0x00000000)
  {
	sprintf(msg,"No data available in memory\r\n");
	HAL_UART_Transmit(&huart1,(uint8_t *)msg,strlen(msg),1000);
	// To store a 32-bit value
	tare=HX711_Tare();
	Flash_Write_Data(0x0801F800,&tare,1);
  }
  else
  {
	tare= backupData;
	sprintf(msg, "Stored value: %lu\r\n",tare);
	HAL_UART_Transmit(&huart1,(uint8_t *)msg,strlen(msg),1000);
  }
  HAL_IWDG_Refresh(&hiwdg); // Refresh at the start
  //Calibrate when needed
  //calibrate();
  HAL_GPIO_WritePin(GPIOA, HX711_ACT_Pin, GPIO_PIN_RESET);
  HAL_Delay(1000);
  startup();
  HAL_IWDG_Refresh(&hiwdg); // Refresh at the start
  // Temporarily disable the interrupt while transmitting
  HAL_NVIC_DisableIRQ(EXTI15_10_IRQn);
  // Array to store the UID
  uint32_t UID[3];
  // Read the UID from the memory locations
  UID[0] = *(uint32_t *)0x1FFFF7E8;
  UID[1] = *(uint32_t *)0x1FFFF7EC;
  UID[2] = *(uint32_t *)0x1FFFF7F0;

  // Convert the UID to a string for transmission
  snprintf(uid_str, sizeof(uid_str), "Node ID: %08X%08X%08X\r\n", UID[2], UID[1], UID[0]);
  HAL_IWDG_Refresh(&hiwdg); // Refresh at the start
  // Transmit the UID via UART1
  HAL_UART_Transmit(&huart1, (uint8_t*)uid_str, strlen(uid_str), HAL_MAX_DELAY);
  // Re-enable the interrupt after transmitting
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);
  lora_start_receiving(&lora);
  HAL_IWDG_Refresh(&hiwdg); // Refresh at the start
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
      // Refresh before entering stop mode
      HAL_IWDG_Refresh(&hiwdg);
      Enter_Stop_Mode();
      // Refresh after waking up from stop mode
      HAL_IWDG_Refresh(&hiwdg);
      if (rtc_check == 1) {
          rtc_check = 0;
          function_counter += 15;
          sprintf(msg, "function count is %lu\r\n", function_counter);
          HAL_UART_Transmit(&huart1, (uint8_t*)msg, strlen(msg), 1000);
          HAL_IWDG_Refresh(&hiwdg);
          if (function_counter >= 30) {
        	  function_counter=0;
              main_counter = 1; // Signal main loop execution
          } else {
              rtc_counter = 1; // Signal RTC wakeup
          }
          HAL_IWDG_Refresh(&hiwdg);
      }
      HAL_IWDG_Refresh(&hiwdg);
      if (rtc_counter == 1) {
    	  HAL_IWDG_Refresh(&hiwdg);
          rtc_counter = 0;
          sprintf(msg8, "Woke up from RTC\r\n");
          HAL_UART_Transmit(&huart1, (uint8_t*)msg8, strlen(msg8), 1000);
          send_weight();
      }
      else if (main_counter == 1 || packetReceived==1) {
    	  HAL_IWDG_Refresh(&hiwdg);
          main_counter = 0;
          sprintf(msg8, "Woke up full program!\r\n");
          HAL_UART_Transmit(&huart1, (uint8_t*)msg8, strlen(msg8), 1000);
          HAL_IWDG_Refresh(&hiwdg);
//    	  if (packetReceived==1){
//    		  packetReceived=0;
//    		  ret = lora_prasePacket(&lora);
//    		  if(ret){
//    			  uint8_t i=0;
//    			  while(lora_available(&lora)){
//    				  buff[i] = lora_read(&lora);
//    				  i++;
//    			  }
//    			  HAL_IWDG_Refresh(&hiwdg);
//    			  buff[i] = '\0';
//    			  memset(msg, 0,256);
//    			  sprintf(msg,"%s\r\n",buff);
//    			  // Check if msg ends with "state:0" or "state:1"
//    			  if (strstr(msg, "ate:0") != NULL || strstr(msg, "ate:1") != NULL) {
//    				  HAL_GPIO_WritePin(GPIOA, HX711_ACT_Pin, GPIO_PIN_SET);
//    				  HAL_Delay(1000);
//    				  tare= HX711_Tare(); // Call tare() function if state changed
//    				  Flash_Write_Data(0x0801F800,&tare,1);
//    				  weight = HX711_GetWeight(50); // in milligram
//    				  weight=weight/1000000;
//    				  if(weight<=0.0){
//    					  weight=0.0;
//    				  }
//    				  // Transmit weight over USART1
//    				  char weight_str1[50];
//    				  sprintf(weight_str1, "Weight after tare: %.2f% Kg\r\n", weight);
//    				  HAL_UART_Transmit(&huart1, (uint8_t*)weight_str1, strlen(weight_str1), HAL_MAX_DELAY);
//    				  HAL_GPIO_WritePin(GPIOA, HX711_ACT_Pin, GPIO_PIN_RESET);
//    				  HAL_Delay(1000);
//    				  HAL_IWDG_Refresh(&hiwdg);
//    				  checkbatlevel();
//    				  HAL_IWDG_Refresh(&hiwdg);
//    				  senddata();
//    			  }
//    			  HAL_IWDG_Refresh(&hiwdg);
//    			  memset(buff, 0,sizeof(buff));
//    			  HAL_UART_Transmit(&huart1,(uint8_t *)msg,strlen(msg),HAL_MAX_DELAY);
//
//    		}
//		}
    	  HAL_IWDG_Refresh(&hiwdg);
    	  checkbatlevel();
    	  send_weight();
    	  senddata();
      }
      HAL_IWDG_Refresh(&hiwdg);
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_RTC|RCC_PERIPHCLK_ADC;
  PeriphClkInit.RTCClockSelection = RCC_RTCCLKSOURCE_LSI;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief IWDG Initialization Function
  * @param None
  * @retval None
  */
static void MX_IWDG_Init(void)
{

  /* USER CODE BEGIN IWDG_Init 0 */

  /* USER CODE END IWDG_Init 0 */

  /* USER CODE BEGIN IWDG_Init 1 */

  /* USER CODE END IWDG_Init 1 */
  hiwdg.Instance = IWDG;
  hiwdg.Init.Prescaler = IWDG_PRESCALER_256;
  hiwdg.Init.Reload = 4095;
  if (HAL_IWDG_Init(&hiwdg) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN IWDG_Init 2 */

  /* USER CODE END IWDG_Init 2 */

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

  RTC_TimeTypeDef sTime = {0};
  RTC_DateTypeDef DateToUpdate = {0};
  RTC_AlarmTypeDef sAlarm = {0};

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
  sTime.Hours = 0x0;
  sTime.Minutes = 0x0;
  sTime.Seconds = 0x0;

  if (HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BCD) != HAL_OK)
  {
    Error_Handler();
  }
  DateToUpdate.WeekDay = RTC_WEEKDAY_MONDAY;
  DateToUpdate.Month = RTC_MONTH_JANUARY;
  DateToUpdate.Date = 0x1;
  DateToUpdate.Year = 0x0;

  if (HAL_RTC_SetDate(&hrtc, &DateToUpdate, RTC_FORMAT_BCD) != HAL_OK)
  {
    Error_Handler();
  }

  /** Enable the Alarm A
  */
  sAlarm.AlarmTime.Hours = 0x0;
  sAlarm.AlarmTime.Minutes = 0x0;
  sAlarm.AlarmTime.Seconds = 0x0;
  sAlarm.Alarm = RTC_ALARM_A;
  if (HAL_RTC_SetAlarm_IT(&hrtc, &sAlarm, RTC_FORMAT_BCD) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN RTC_Init 2 */

  /* USER CODE END RTC_Init 2 */

}

/**
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI2_Init(void)
{

  /* USER CODE BEGIN SPI2_Init 0 */

  /* USER CODE END SPI2_Init 0 */

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_64;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

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
  htim2.Init.Prescaler = 71;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 65535;
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
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, BLUE_Pin|GREEN_Pin|RED_Pin|SCK_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, RESET_Pin|NSS_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(HX711_ACT_GPIO_Port, HX711_ACT_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : BLUE_Pin GREEN_Pin RED_Pin SCK_Pin */
  GPIO_InitStruct.Pin = BLUE_Pin|GREEN_Pin|RED_Pin|SCK_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : RESET_Pin */
  GPIO_InitStruct.Pin = RESET_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(RESET_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : DIO0_Pin */
  GPIO_InitStruct.Pin = DIO0_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(DIO0_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : NSS_Pin HX711_ACT_Pin */
  GPIO_InitStruct.Pin = NSS_Pin|HX711_ACT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : DT_Pin */
  GPIO_InitStruct.Pin = DT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(DT_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void Enter_Stop_Mode(void)
{
    // Get the current time
    RTC_TimeTypeDef currentTime = {0};
    HAL_RTC_SetTime(&hrtc, &currentTime, RTC_FORMAT_BIN);
    RTC_AlarmTypeDef sAlarm = {0};

    // Set the alarm time to 15 seconds from now
    uint8_t alarmSeconds = currentTime.Seconds + 15;
    if (alarmSeconds  >= 60) {
    	alarmSeconds  -= 60;
    }

    sAlarm.AlarmTime.Seconds = alarmSeconds ;
    sAlarm.AlarmTime.Minutes = currentTime.Minutes;
    sAlarm.AlarmTime.Hours = currentTime.Hours;
    sAlarm.Alarm = RTC_ALARM_A;

    if (HAL_RTC_SetAlarm_IT(&hrtc, &sAlarm, RTC_FORMAT_BIN) != HAL_OK) {
        Error_Handler(); // Handle any error that occurs
    }

   // __HAL_PWR_CLEAR_FLAG(PWR_FLAG_WU);
    // Clear any previous wakeup flags
    __HAL_PWR_CLEAR_FLAG(PWR_FLAG_WU | PWR_FLAG_SB);
    // Enter Stop mode, MCU will halt here until RTC alarm triggers
    HAL_SuspendTick();
    HAL_PWR_EnterSTOPMode(PWR_LOWPOWERREGULATOR_ON, PWR_STOPENTRY_WFI);
    // Reconfigure the system clock after waking up
    SystemClock_Config();
    HAL_ResumeTick();
}

void check_gateway(void) {
    uint32_t timeout = HAL_GetTick() + 5000;  // 5 seconds timeout
    while (HAL_GetTick() < timeout) {
        ret = lora_prasePacket(&lora);
        if (ret) {
            uint8_t i = 0;
            while (lora_available(&lora)) {
                buff[i] = lora_read(&lora);
                i++;
            }
            buff[i] = '\0';  // Null-terminate the received string
            sprintf(msg, "Received feedback: %s\r\n", buff);
            HAL_UART_Transmit(&huart1, (uint8_t *)msg, strlen(msg), HAL_MAX_DELAY);

            if (strcmp(buff, "ACK") == 0) {  // Check if the received message is "ACK"
                HAL_UART_Transmit(&huart1, (uint8_t *)"Gateway available\r\n", 19, HAL_MAX_DELAY);
                gateway_available=1;
                return;  // Exit the function if ACK is received
            }
        }
    }
    // If the loop completes without receiving an "ACK", send the "gateway unavailable" message
    HAL_UART_Transmit(&huart1, (uint8_t *)"Gateway unavailable\r\n", 22, HAL_MAX_DELAY);
    gateway_available=0;
}

void store_message(const char* msg) {
    if (messageCount < MAX_MESSAGES) {
        strncpy(messageQueue[messageCount].message, msg, sizeof(messageQueue[messageCount].message) - 1);
        messageQueue[messageCount].message[sizeof(messageQueue[messageCount].message) - 1] = '\0'; // Ensure null termination
        messageCount++;
    } else {
        HAL_UART_Transmit(&huart1, (uint8_t *)"Message queue full, dropping message\r\n", 39, HAL_MAX_DELAY);
    }
}

void send_stored_messages(void) {
    for (uint16_t i = 0; i < messageCount; i++) {
        lora_begin_packet(&lora);
        lora_tx(&lora, (uint8_t *)messageQueue[i].message, strlen(messageQueue[i].message));
        lora_end_packet(&lora);
        HAL_UART_Transmit(&huart1, (uint8_t *)"Sending stored message\r\n", 25, HAL_MAX_DELAY);
        HAL_UART_Transmit(&huart1, (uint8_t *)messageQueue[i].message, strlen(messageQueue[i].message), HAL_MAX_DELAY);
        HAL_Delay(1000); // Slight delay between messages to avoid congestion
    }
    messageCount = 0; // Clear the message queue after sending
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
  __disable_irq();
  while (1)
  {
  }
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
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
