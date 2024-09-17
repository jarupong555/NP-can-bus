/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2023 STMicroelectronics.
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
#include "can.h"
#include "dma.h"
#include "i2c.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"
#include "wizchip_conf.h"
#include "string.h"
#include "led.h"
#include <stdio.h>
#include <stdlib.h>
#include "socket.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
// #include "eeprom_AT24Cxx.h"
#define MAX_LED 8
#define BRIGHTNESS 15
#define SERVER_IP "192.168.1.65"
#define SERVER_PORT 8086
#define DATABASE_NAME "STM32"
#define MEASUREMENT_NAME "datatest"
#define STRINGIFY(x) #x
#define TOSTRING(x) STRINGIFY(x)

#ifndef DATA_BUF_SIZE
#define DATA_BUF_SIZE			2048
#endif
#define UDP_SOCKET 1

#define TX_BUF_SIZE 2048
#define RX_BUF_SIZE 2048

extern uint8_t cnt_state_pressed;

uint8_t txBuf[TX_BUF_SIZE];
uint8_t rxBuf[RX_BUF_SIZE];

#define MSG_NURSE_PRESENCE "\"225\""
#define MSG_PENDANT "\"PENDANT\""
#define MSG_EMERGENCY "\"EMERGENCY\""
#define MSG_PULLCORD "PULLCORD"

const char M_CALL_SETACTIVE[] = ":1/A+1/1 \"ALARMCALL\" \"WARD-1\" \""; // activate message to server
const char M_CALL_DEACTIVE[] = ":1/A-1/1 \"ALARMCALL\" \"WARD-1\" \"";  // deactivate message to server
// length = 5
// #define M_LEN_FLAG 5
const char M_CALLGREEN[] = "-001\""; // Call Green flag to server
const char M_CALLEMER[] = "-004\"";  // Call Emer flag to server
const char M_PENDANT[] = "-007\"";   // Pendant flag to server
const char M_PC_CALL[] = "-008\"";   // Pull cord flag to server
const char M_SALINE[] = "-010\"";    // saline flag to server
const char M_EXTERNAL[] = "-011\"";  // External flag to server
const char M_PC_LOSE[] = "-111\"";   // Pull cord lose flag to server
const char M_DOCTOR[] = "-100\"";   // Pull cord lose flag to server

uint8_t server_ip[] = { 192, 168, 1, 255 };
void sendUDP(const char *command, const char *flag, uint32_t sender_id){
	char buffer[128];
	char charID[16] = {0};
	itoa(sender_id, charID, 10);
	memset(buffer, 0, sizeof(buffer));
	strcat(buffer, command);
    strcat(buffer, "CAN");
    strcat(buffer, charID);
    strcat(buffer, flag);
	sendto(UDP_SOCKET, (uint8_t*) buffer, sizeof(buffer), server_ip,
				24949);
	return;
}

void sendNS(const uint32_t id, uint8_t data[])
{
	switch (data[0])
			{
				case NONE: //cancel
					switch (data[1])
					{
						case NURSE_PRESENCE:
							sendUDP(M_CALL_DEACTIVE, M_CALLGREEN, id);
							break;

						case DOCTOR_PRESENCE:
							sendUDP(M_CALL_DEACTIVE, M_DOCTOR, id);
							break;

						case PENDANT:
							sendUDP(M_CALL_DEACTIVE, M_PENDANT, id);
							break;

						case EMERGENCY:
							sendUDP(M_CALL_DEACTIVE, M_CALLEMER, id);
							break;

						case SOS:
							sendUDP(M_CALL_DEACTIVE, M_EXTERNAL, id);
							break;
						case PULLCORD:
							sendUDP(M_CALL_DEACTIVE, M_PC_CALL, id);
							break;
						default:
							break;
					}
					break;
				case NURSE_PRESENCE:
					sendUDP(M_CALL_SETACTIVE, M_CALLGREEN, id);
					break;

				case DOCTOR_PRESENCE:
					sendUDP(M_CALL_SETACTIVE, M_DOCTOR, id);
					break;

				case PENDANT:
					sendUDP(M_CALL_SETACTIVE, M_PENDANT, id);
					break;

				case EMERGENCY:
					sendUDP(M_CALL_SETACTIVE, M_CALLEMER, id);
					break;

				case SOS:
					sendUDP(M_CALL_SETACTIVE, M_EXTERNAL, id);
					break;
				case PULLCORD:
					sendUDP(M_CALL_SETACTIVE, M_PC_CALL, id);
					break;
				default:
					break;
			}
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan1)
{
	HAL_CAN_GetRxMessage(hcan1, CAN_RX_FIFO0, &rxHeader, canRX);
	sendNS(rxHeader.StdId, canRX);
}

const char* getButtonStateMessage(lamp_state_t state) {
    switch (state) {
        case NURSE_PRESENCE:
            return M_CALL_SETACTIVE;
        case PENDANT:
            return MSG_PENDANT;
        case EMERGENCY:
            return MSG_EMERGENCY;
        case PULLCORD:
            return MSG_PULLCORD;
        default:
            return "UNKNOWN";
    }
}

void W5500_Select(void)
{
    HAL_GPIO_WritePin(nCS_GPIO_Port, nCS_Pin, GPIO_PIN_RESET);
}

void W5500_Unselect(void)
{
    HAL_GPIO_WritePin(nCS_GPIO_Port, nCS_Pin, GPIO_PIN_SET);
}

void W5500_ReadBuff(uint8_t *buff, uint16_t len)
{
    HAL_SPI_Receive(&hspi1, buff, len, HAL_MAX_DELAY);
}

void W5500_WriteBuff(uint8_t *buff, uint16_t len)
{
    HAL_SPI_Transmit(&hspi1, buff, len, HAL_MAX_DELAY);
}

uint8_t W5500_ReadByte(void)
{
    uint8_t byte;
    W5500_ReadBuff(&byte, sizeof(byte));
    return byte;
}

void W5500_WriteByte(uint8_t byte)
{
    W5500_WriteBuff(&byte, sizeof(byte));
}



//u32 W5500_Send_Delay=0; //W5500ส่งล่าช้า ตัวแปรนับ (ms)
wiz_NetInfo gWIZNETINFO;

void Load_Net_Parameters(void)
{
	gWIZNETINFO.gw[0] = 192; //Gateway
	gWIZNETINFO.gw[1] = 168;
	gWIZNETINFO.gw[2] = 1;
	gWIZNETINFO.gw[3] = 1;

	gWIZNETINFO.sn[0]=255; //Mask
	gWIZNETINFO.sn[1]=255;
	gWIZNETINFO.sn[2]=255;
	gWIZNETINFO.sn[3]=0;

	gWIZNETINFO.mac[0]=0x0a; //MAC
	gWIZNETINFO.mac[1]=0x34;
	gWIZNETINFO.mac[2]=0xab;
	gWIZNETINFO.mac[3]=0x5c;
	gWIZNETINFO.mac[4]=0x3b;
	gWIZNETINFO.mac[5]=0x36;

	gWIZNETINFO.ip[0]=192; //IP
	gWIZNETINFO.ip[1]=168;
	gWIZNETINFO.ip[2]=1;
	gWIZNETINFO.ip[3]=225;

	gWIZNETINFO.dhcp = NETINFO_STATIC;
}
uint8_t destip[4] = {192, 168, 1, 255};
uint16_t destport = 24948;
uint8_t buf[DATA_BUF_SIZE];
uint8_t memsize[2][8] = { {2,2,2,2,2,2,2,2},{2,2,2,2,2,2,2,2}};
/* USER CODE END Includes */
void network_init(void)
{


    reg_wizchip_cs_cbfunc(W5500_Select, W5500_Unselect);
    reg_wizchip_spi_cbfunc(W5500_ReadByte, W5500_WriteByte);
    reg_wizchip_spiburst_cbfunc(W5500_ReadBuff, W5500_WriteBuff);


  wiz_NetTimeout gWIZNETTIME = {.retry_cnt = 3, .time_100us = 2000};
  ctlnetwork(CN_SET_TIMEOUT,(void*)&gWIZNETTIME);
  ctlnetwork(CN_GET_TIMEOUT, (void*)&gWIZNETTIME);
  printf("TIMEOUT: %d, %d\r\n", gWIZNETTIME.retry_cnt,gWIZNETTIME.time_100us);


  wizchip_init(memsize[0], memsize[1]);

	ctlnetwork(CN_SET_NETINFO, (void*)&gWIZNETINFO);
	HAL_Delay(100);
	ctlnetwork(CN_GET_NETINFO, (void*)&gWIZNETINFO);
	// Display Network Information
  uint8_t tmpstr[6];
	ctlwizchip(CW_GET_ID,(void*)tmpstr);
	printf("\r\n=== %s NET CONF ===\r\n",(char*)tmpstr);
	printf("MAC: %02X:%02X:%02X:%02X:%02X:%02X\r\n",
    gWIZNETINFO.mac[0],gWIZNETINFO.mac[1],gWIZNETINFO.mac[2],gWIZNETINFO.mac[3],gWIZNETINFO.mac[4],gWIZNETINFO.mac[5]);
	printf("SIP: %d.%d.%d.%d\r\n", gWIZNETINFO.ip[0],gWIZNETINFO.ip[1],gWIZNETINFO.ip[2],gWIZNETINFO.ip[3]);
	printf("GAR: %d.%d.%d.%d\r\n", gWIZNETINFO.gw[0],gWIZNETINFO.gw[1],gWIZNETINFO.gw[2],gWIZNETINFO.gw[3]);

	printf("SUB: %d.%d.%d.%d\r\n", gWIZNETINFO.sn[0],gWIZNETINFO.sn[1],gWIZNETINFO.sn[2],gWIZNETINFO.sn[3]);
	printf("DNS: %d.%d.%d.%d\r\n", gWIZNETINFO.dns[0],gWIZNETINFO.dns[1],gWIZNETINFO.dns[2],gWIZNETINFO.dns[3]);
	printf("======================\r\n");

	setSHAR(gWIZNETINFO.mac);


	uint8_t tmp = 0;
//	ctlnetwork(CN_SET_NETMODE, (void*)NM_FORCEARP);
//	printf("set status: %d", wizchip_setnetmode(NM_FORCEARP));
	ctlnetwork(CN_GET_NETMODE, (void*)&tmp);
	printf("net mode: %d\r\n", tmp);
}
void SendToInfluxDB(lamp_state_t state)
{
    // Define variables for each status field
    const char* status1 = "NONE";
    const char* status2 = "NONE";
    const char* status3 = "NONE";

    // Update the status variables based on the state
    switch (state)
    {
        case NURSE_PRESENCE: status1 = "NURSE_PRESENCE"; break;
        case EMERGENCY: status2 = "EMERGENCY"; break;
        case PENDANT: status3 = "PENDANT"; break;
        default: break; // All fields remain "NONE"
    }

    // Create the line protocol data with the status fields
    char lineProtocolData[128];
    snprintf(lineProtocolData, sizeof(lineProtocolData),
             "%s,location=office status1=\"%s\",status2=\"%s\",status3=\"%s\"",
             MEASUREMENT_NAME, status1, status2, status3);

    const char* httpRequestTemplate = "POST /write?db=" DATABASE_NAME " HTTP/1.1\r\n"
                                      "Host: " SERVER_IP ":" TOSTRING(SERVER_PORT) "\r\n"
                                      "Content-Length: %d\r\n"
                                      "Content-Type: application/x-www-form-urlencoded\r\n"
                                      "\r\n"
                                      "%s";
    char httpRequest[512];
    sprintf(httpRequest, httpRequestTemplate, strlen(lineProtocolData), lineProtocolData);

    uint8_t sock = 0;
    uint8_t ip[] = {192, 168, 1, 65};
    uint16_t port = 8086;

    if (socket(sock, Sn_MR_TCP, port, 0) == sock) {
        if (connect(sock, ip, port) == SOCK_OK) {
            // Send the status data
            send(sock, (uint8_t*)httpRequest, strlen(httpRequest));
            // Disconnect and close the socket
            disconnect(sock);
        }
        close(sock);
    }
}



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

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

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
  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_CAN_Init();
  MX_I2C1_Init();
  MX_SPI1_Init();
  MX_TIM1_Init();
  MX_USART1_UART_Init();
  MX_TIM4_Init();

  /* USER CODE BEGIN Init */
  init_state_buf();

  // Start timer
  HAL_TIM_Base_Start_IT(&htim4);
  uint32_t time = millis();

  // Start the LED timer
  start_led();

  // Initialize LEDs
  Set_LED(2, 255, 255, 255); // call button
  Set_LED(1, 255, 255, 255); // call button
  Set_Brightness(BRIGHTNESS);
  WS2812_Send();

  // Uncomment the following lines if needed
   Load_Net_Parameters();
   network_init();
   if(ctlwizchip(0, (void*)memsize) == -1){
       printf("WIZCHIP Initialized fail.\r\n");
//       while(1);
   }

  // Initialize network parameters
  uint8_t dest_ip[] = {192, 168, 1, 200};
  uint16_t dest_port = 24949;
  void init_network()
  {
    uint8_t stat = socket(UDP_SOCKET, Sn_MR_UDP, 5555, 0);
    printf("Creating socket ...\r\n");
    if (stat != UDP_SOCKET)
        printf("socket() failed, code = %d\r\n", stat);
    else
        printf("Socket created, connecting...\r\n");

    setSn_DIPR(UDP_SOCKET, dest_ip);
    setSn_DPORT(UDP_SOCKET, dest_port);
  }

  init_network();
  printf("Go!");
 // uint8_t ctmp[10];
  /* USER CODE END Init */

  /* Infinite loop */
  while (1)
  {
    /* USER CODE BEGIN WHILE */
    // Update status LEDs
    status_led();

    // Get button state
    lamp_state_t buttonState = getButtonState();

    // Get the corresponding message for the button state
    const char* message = getButtonStateMessage(buttonState);

    // Send heartbeat and button state message every second
    if (millis() - time >= 1000)
    {
      time = millis();
      sendHeartbeat();
//      sendto(UDP_SOCKET, (uint8_t*)message, strlen(message), dest_ip, dest_port);
    }

    // Call debounce and buzzer check functions
    call_debounce();
    buzzCheck();

    // Debugging
#ifdef debug
    // printf("CNT: %d\r\n", cnt_state_pressed);
    // printf("PD1: %d, PD2: %d\r\n", HAL_GPIO_ReadPin(Pendant1_GPIO_Port, Pendant1_Pin), HAL_GPIO_ReadPin(Pendant2_GPIO_Port, Pendant2_Pin));
#endif

    // Update LED states based on button state
    if (cnt_state_pressed == NONE)
    {
      Set_LED(0, 0, 0, 0); // cancel button
      Set_Brightness(BRIGHTNESS);
      WS2812_Send();

      Set_LED(3, 0, 0, 0); // cancel button
      Set_Brightness(BRIGHTNESS);
      WS2812_Send();

      Set_LED(4, 0, 0, 0); // cancel button
      Set_Brightness(BRIGHTNESS);
      WS2812_Send();

      Set_LED(5, 0, 0, 0); // cancel button
      Set_Brightness(BRIGHTNESS);
      WS2812_Send();
    }
    else
    {
      Set_LED(0, 255, 0, 0); // cancel button
      Set_Brightness(BRIGHTNESS);
      WS2812_Send();
    }
    /* USER CODE END WHILE */
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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV2;
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
}

/* USER CODE BEGIN 4 */

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
