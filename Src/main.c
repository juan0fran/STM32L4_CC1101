/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
  ******************************************************************************
  *
  * COPYRIGHT(c) 2017 STMicroelectronics
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
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32l4xx_hal.h"
#include "dma.h"
#include "rng.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* USER CODE BEGIN Includes */

#include "cc1101_routine.h"
#include "command_parser.h"
#include "utils.h"
#include "of_reed-solomon_gf_2_m.h"
#include "link_layer.h"
#include "simple_link.h"

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void Error_Handler(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

//#define _FDEF_SLEEP
//#define _FDEF_UART

extern circ_buff_t circular_cc1101_queue;

static radio_packet_t packet;

static of_rs_2_m_cb_t rs;
static of_rs_2_m_parameters_t parms;

extern circ_buff_t uart_queue;

/* USER CODE END 0 */

int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_RNG_Init();
  MX_SPI1_Init();
  MX_TIM2_Init();
  MX_USART1_UART_Init();

  /* USER CODE BEGIN 2 */

  spi_parms_t spi;
  radio_parms_t radio;

  llc_parms_t llc;

  void * enc_sym_tabs[OF_MAX_ENCODING_SYMBOLS];
  void * dec_sym_tabs[OF_MAX_ENCODING_SYMBOLS];

  uint8_t symb_reserved_space_tx[OF_MAX_ENCODING_SYMBOLS * OF_MAX_SYMBOL_SIZE];
  uint8_t symb_reserved_space_rx[OF_MAX_ENCODING_SYMBOLS * OF_MAX_SYMBOL_SIZE];

  uint8_t buffer[MAC_PAYLOAD_SIZE];

  int i, cnt;

  set_freq_parameters(434.92e6f, 384e3f, 36e3f, &radio);
  set_sync_parameters(PREAMBLE_4, SYNC_30_over_32, 500, &radio);
  set_packet_parameters(false, true, &radio);
  set_modulation_parameters(RADIO_MOD_GFSK, RATE_9600, 0.5f, &radio);

  init_radio_config(&spi, &radio);

  enable_isr_routine(&spi, &radio);
  //radio_calibrate(&spi);

  init_command_handler();

  uint8_t char_set[3] = {'\n', '\r', '\0'};

  /* print_uart_ln("Endianess: %s", endian_check() ? "Little" : "Big"); */

  uint8_t simple_buffer[1500];
  simple_link_packet_t s_packet;
  simple_link_control_t s_control;

  prepare_simple_link('J', 'F', &s_control);

  memset(buffer, 0xAA, 1500);
  //set_simple_link_packet(buffer, 1500, 0, 0, &s_control, &s_packet);

  uint8_t byte;
  bool not_sent;
  int ret;
  chunk_handler_t chunk_tx;
  memset(&chunk_tx, 0, sizeof(chunk_handler_t));

  while(1){
	  if (available_items(&uart_queue) > 0){
		  while (dequeue(&uart_queue, &byte)){
	          if( get_simple_link_packet(byte, &s_control, &s_packet) > 0){
	              //print_uart_ln("Packet received of length: %u!!", s_packet.fields.len);
	        	  not_sent = true;
	        	  while(not_sent){
	        		  ret = get_new_packet_from_chunk(&chunk_tx, s_packet.fields.payload, s_packet.fields.len, 2, &packet);
	        		  if (ret > 0){
	        			  radio_send_packet(&spi, &radio, &packet);
	        		  }else if (ret == 0){
	        			  radio_send_packet(&spi, &radio, &packet);
	        			  not_sent = false;
	        		  }else{
	        			  not_sent = false;
	        		  }
	        	  }
	              prepare_simple_link('J', 'F', &s_control);
	          }
		  }
	  }else{
		  if (s_control.byte_cnt > 0){
			  HAL_Delay(1);
			  if (! (available_items(&uart_queue) > 0) ){
				  prepare_simple_link('J', 'F', &s_control);
			  }
		  }
	  }
  }

  while(1){
	  uart_send(&s_packet, s_control.full_size);
	  HAL_Delay(500);
  }

#if 0
  while(1){
	  if ( ( cnt = command_input_until(char_set, 3, buffer, sizeof(buffer), 100)) > 0 ){
		  print_uart("%s", buffer);
	  }
	  /*delay_us(100);*/
  }

  chunk_handler_t chunk_tx, chunk_rx;
  char j = 'A';
  for (i = 0; i < sizeof(symb_reserved_space_tx); i++){
	  if (i % MAC_PAYLOAD_SIZE == 0)
		  j++;
	  symb_reserved_space_tx[i] = j;
  }
  volatile int size = MAC_PAYLOAD_SIZE + MAC_PAYLOAD_SIZE + MAC_PAYLOAD_SIZE;
  bool not_sent;
  int ret;
  int timer;
  init_chunk_handler(&chunk_tx);
  init_chunk_handler(&chunk_rx);

  not_sent = true;
  while(1){
	  while(not_sent){
		  ret = get_new_packet_from_chunk(&chunk_tx, symb_reserved_space_tx, size, 2, &packet);
		  if (ret > 0){
			  /* If something has been received... */
			  radio_send_packet(&spi, &radio, &packet);
			  timer = 0;
		  }else if (ret == 0){
			  radio_send_packet(&spi, &radio, &packet);
			  not_sent = false;
			  timer = 0;
		  }else{
			  not_sent = false;
			  timer = 0;
		  }
	  }
	  if (dequeue(&circular_cc1101_queue, &packet) == true){
		  print_uart_ln("-> RSSI: %d LQI: %d%%", (int)rssi_dbm(packet.fields.rssi), (int) lqi_status(packet.fields.lqi));
		  if (set_new_packet_to_chunk(&chunk_rx, &packet, symb_reserved_space_rx) > 0){
			  print_uart_ln("Chunk Received!!");
			  not_sent = true;
			  timer = 0;
		  }
	  }
	  if (++timer > 50){
		  /* If this timer is reached, meaning 5 seconds without activity */
		  not_sent = true;
	  }else{
		  delay_us(MS_TO_US(100));
	  }
  }

  while(1){

	  of_rs_2_m_set_fec_parameters(&rs, &parms);

	  llc.chunk_seq++;
	  llc.src_addr = 0;
	  llc.k = 4;
	  llc.r = 4;

	  for (i = 0; i < rs.nb_source_symbols; i++){
		  memset(&symb_reserved_space_tx[i * rs.encoding_symbol_length], i+1, rs.encoding_symbol_length);
		  enc_sym_tabs[i] = &symb_reserved_space_tx[i * rs.encoding_symbol_length];
		  llc.esi = i;
		  build_llc_packet(enc_sym_tabs[i], rs.encoding_symbol_length, &llc, &packet);
		  radio_send_packet(&spi, &radio, &packet);
	  }

	  for (i = rs.nb_source_symbols; i < rs.nb_encoding_symbols; i++){
		  memset(&symb_reserved_space_tx[i * rs.encoding_symbol_length], 0, rs.encoding_symbol_length);
		  enc_sym_tabs[i] = &symb_reserved_space_tx[i * rs.encoding_symbol_length];
		  of_rs_2_m_build_repair_symbol(&rs, enc_sym_tabs, i);
		  llc.esi = i;
		  build_llc_packet(enc_sym_tabs[i], rs.encoding_symbol_length, &llc, &packet);
		  radio_send_packet(&spi, &radio, &packet);
	  }
	  /*
	  of_rs_2_m_set_fec_parameters(&rs, &parms);
	  for (i = 0; i < rs.nb_source_symbols; i++){
		  memcpy(&symb_reserved_space_rx[i * rs.encoding_symbol_length], &symb_reserved_space_tx[(i + 2)* rs.encoding_symbol_length], rs.encoding_symbol_length);
		  of_rs_2_m_decode_with_new_symbol(&rs, &symb_reserved_space_rx[i * rs.encoding_symbol_length], i+2);
	  }
	  of_rs_2_m_get_source_symbols_tab(&rs, dec_sym_tabs);
	  int equal = 0;
	  for (i = 0; i < rs.nb_source_symbols; i++){
		  if (memcmp(dec_sym_tabs[i], enc_sym_tabs[i], rs.encoding_symbol_length) == 0){
			  equal++;
		  }
	  }
	  if (equal == rs.nb_source_symbols){
		  print_uart_ln("Correct");
	  }else{
		  print_uart_ln("Incorrect");
	  }*/
  }
  cnt = 0;
  while(1){
	  if (dequeue(&circular_cc1101_queue, &packet) == true){
		  memset(buffer, 0, sizeof(buffer));
		  sscanf((char *) &packet.raw[1], "%[^\n]", (char *) buffer);
		  print_uart("Cnt: %d received -> RSSI: %d / %d dBm/Dec, LQI: %d%%\r\n%s\r\n", cnt++, (int)rssi_dbm(packet.fields.rssi), packet.fields.rssi, (int) lqi_status(packet.fields.lqi), buffer);
	  }
  }
#endif

#if 0
  HAL_DBGMCU_EnableDBGSleepMode();
  HAL_DBGMCU_EnableDBGStopMode();
  HAL_DBGMCU_EnableDBGStandbyMode();

  int i;
  uint8_t buffer[MAC_PAYLOAD_SIZE];
  uint32_t cnt;

  /* Enable this */
  spi_parms_t spi;
  radio_parms_t radio;

  set_freq_parameters(434.92e6f, 384e3f, 36e3f, &radio);
  set_sync_parameters(PREAMBLE_4, SYNC_30_over_32, 500, &radio);
  set_packet_parameters(false, true, &radio);
  set_modulation_parameters(RADIO_MOD_FSK2, RATE_9600, 0.5f, &radio);

  init_radio_config(&spi, &radio);

  enable_isr_routine(&spi, &radio);
  //radio_calibrate(&spi);

  init_command_handler();
  /* This timer is in charge of monitoring CC1101 state */
  /* It has some internal timeouts */
  //HAL_TIM_Base_Start_IT(&htim2);

  /* Test! */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  i = 0;
  cnt = 0;
  uint8_t char_set[3] = {'\n', '\r', '\0'};

  while (1)
  {
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */
	  HAL_PWR_EnterSLEEPMode(PWR_MAINREGULATOR_ON, PWR_STOPENTRY_WFI);

	  if (command_input_until(char_set, 3, buffer, sizeof(buffer), S_TO_MS(5)) > 0){
		  remove_endlines((char *) buffer);
		  strcat((char*) buffer, "\r\n");
		  radio_send_packet(&spi, &radio, buffer, strlen((const char *) buffer));
	  }

	  if (dequeue(&circular_cc1101_queue, &packet) == true){
		  memset(buffer, 0, sizeof(buffer));
		  sscanf((char *) packet.fields.data, "%[^\n]", (char *) buffer);
		  print_uart("Cnt: %d received -> RSSI: %d / %d dBm/Dec, LQI: %d%%\r\n%s\r\n", cnt++, (int)rssi_dbm(packet.fields.rssi), packet.fields.rssi, (int) lqi_status(packet.fields.lqi), buffer);
	  }
	  //delay_us(MS_TO_US(rand()%1000 + 500));
	  /*if (command_input_until(char_set, 3, buffer, sizeof(buffer), S_TO_MS(5)) > 0){
		  remove_endlines((char *) buffer);
		  print_uart("%s\r\n", buffer);
	  }*/
  }
#endif
  /* USER CODE END 3 */

}

/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInit;

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 20;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1|RCC_PERIPHCLK_RNG;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK2;
  PeriphClkInit.RngClockSelection = RCC_RNGCLKSOURCE_PLLSAI1;
  PeriphClkInit.PLLSAI1.PLLSAI1Source = RCC_PLLSOURCE_MSI;
  PeriphClkInit.PLLSAI1.PLLSAI1M = 1;
  PeriphClkInit.PLLSAI1.PLLSAI1N = 16;
  PeriphClkInit.PLLSAI1.PLLSAI1P = RCC_PLLP_DIV7;
  PeriphClkInit.PLLSAI1.PLLSAI1Q = RCC_PLLQ_DIV2;
  PeriphClkInit.PLLSAI1.PLLSAI1R = RCC_PLLR_DIV2;
  PeriphClkInit.PLLSAI1.PLLSAI1ClockOut = RCC_PLLSAI1_48M2CLK;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }

    /**Configure the main internal regulator output voltage 
    */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler */
  /* User can add his own implementation to report the HAL error return state */
  while(1) 
  {
  }
  /* USER CODE END Error_Handler */ 
}

#ifdef USE_FULL_ASSERT

/**
   * @brief Reports the name of the source file and the source line number
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source number
   * @retval None
   */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */

}

#endif

/**
  * @}
  */ 

/**
  * @}
*/ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
