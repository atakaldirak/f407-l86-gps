/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#include "lwgps/lwgps.h"
#include <stdio.h> // printf kullanmak için
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
UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */
lwgps_t gps; // Global GPS yapı değişkeni

// UART alım buffer'ı ve index'i
#define GPS_RX_BUFFER_SIZE 256 // NMEA cümleleri için yeterli büyüklükte bir buffer
uint8_t gps_rx_buffer[GPS_RX_BUFFER_SIZE];
uint16_t gps_rx_buffer_idx = 0; // uint8_t yerine uint16_t kullanmak daha güvenli
uint8_t gps_single_byte_rx; // Tek bayt okumak için
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART3_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/**
  * @brief UART Receive Complete Callback.
  * @param huart: UART handle
  * @retval None
  */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    // GPS verisinin geldiği UART'ın huart3 olduğundan emin olun
    if (huart == &huart3) {
        // Gelen baytı geçici buffera ekle
        if (gps_rx_buffer_idx < (GPS_RX_BUFFER_SIZE - 1)) { // Buffer taşmasını engelle, null sonlandırıcı için yer bırak
            gps_rx_buffer[gps_rx_buffer_idx++] = gps_single_byte_rx;
        }

        // Eğer gelen karakter '\n' ise (NMEA cümle sonu) veya buffer dolduysa, cümleyi işle
        if (gps_single_byte_rx == '\n') {
            // lwgps'e veriyi işlemesi için gönder
            // NMEA cümleleri CR-LF (\r\n) ile biter, lwgps kütüphanesi bunu içeride ayrıştırır.
            // Bu yüzden buffer'ın tamamını gönderiyoruz.
            lwgps_process(&gps, gps_rx_buffer, gps_rx_buffer_idx);

            // Buffer'ı ve index'i sıfırla, bir sonraki cümle için hazırla
            gps_rx_buffer_idx = 0;
        } else if (gps_rx_buffer_idx == (GPS_RX_BUFFER_SIZE - 1)) { // Buffer dolarsa ve '\n' gelmezse
            // Buffer doldu, ancak NMEA cümlesi bitmedi. Bu durumda buffer'ı sıfırla ve yeni cümleye başla.
            // Bu durum genellikle bir hata göstergesidir (örneğin çok uzun NMEA cümlesi veya veri akışında sorun).
            gps_rx_buffer_idx = 0;
        }

        // Sonraki bayt için kesmeyi tekrar başlat
        HAL_UART_Receive_IT(&huart3, &gps_single_byte_rx, 1);
    }
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
  MX_USART3_UART_Init();
  /* USER CODE BEGIN 2 */
  // lwgps kütüphanesini başlat
  lwgps_init(&gps);

  // UART kesmesini başlatarak ilk baytı al
  HAL_UART_Receive_IT(&huart3, &gps_single_byte_rx, 1);

  // SWV (Serial Wire Viewer) veya başka bir UART üzerinden printf çıktısı için
  // Eğer henüz yapılandırılmadıysa, STM32CubeIDE'de "System Core -> SYS -> Debug -> Serial Wire"
  // seçeneğini etkinleştirip, "SWV ITM Console"ı kullanmak üzere ayarlamanız gerekebilir.
  // Ya da başka bir UART (örneğin USART2) üzerinden printf yönlendirmesi yapmalısınız.
  // Bu kod printf'in nasıl çalıştığına dair bir örnek sağlar.
  setvbuf(stdout, NULL, _IONBF, 0); // printf buffer'ını kapat (isteğe bağlı, anlık çıktı için)
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    // GPS verisinin geçerli olup olmadığını kontrol et (GPRMC cümlesine göre)
    if (lwgps_is_valid(&gps)) {
        // Geçerli konum bilgisi varsa yazdır
        // gps.latitude ve gps.longitude doğrudan lwgps_t yapısı içinde bulunur
        printf("Latitude: %.6f\r\n", gps.latitude);
        printf("Longitude: %.6f\r\n", gps.longitude);

        // Yükseklik bilgisi varsa yazdır
        // lwgps_t yapısındaki altitude alanı
        printf("Altitude: %.2f m\r\n", gps.altitude);

        // Kullanımdaki uydu sayısı varsa yazdır
        // lwgps_t yapısındaki sats_in_use alanı
        printf("Satellites in use: %d\r\n", gps.sats_in_use);

        // Hız bilgisi varsa (GPRMC'den gelir)
        // gps.speed'i kullanarak hız verisini lwgps_to_speed ile farklı birime çevirebilirsiniz
        printf("Speed: %.2f km/h\r\n", lwgps_to_speed(gps.speed, LWGPS_SPEED_KPH));

        // Tarih bilgisi varsa (GPRMC'den gelir)
        // gps.date, gps.month, gps.year alanları
        if (gps.date != 0 && gps.month != 0 && gps.year != 0) { // Basit bir kontrol
            printf("Date: %02d/%02d/%04d\r\n", (int)gps.date, (int)gps.month, (int)gps.year + 2000); // Yılı 20XX olarak varsayalım
        }

        // Zaman bilgisi varsa (GPGGA'dan gelir)
        // gps.hours, gps.minutes, gps.seconds alanları
        if (gps.time_valid) { // time_valid bayrağı GPGGA'dan gelir
            printf("Time: %02d:%02d:%02d\r\n", (int)gps.hours, (int)gps.minutes, (int)gps.seconds);
        }
    } else {
        printf("Waiting for GPS fix or valid data...\r\n");
    }

    // Her 1 saniyede bir GPS verilerini kontrol et ve yazdır
    HAL_Delay(1000);
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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 9600; // L86 varsayılanı
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX; // Hem TX hem RX kullanılacak
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */
  // UART3 için kesme etkinleştirildiğinden emin olun (CubeMX'te NVIC kısmından).
  // Eğer değilse, burada HAL_NVIC_EnableIRQ(USART3_IRQn); eklemeniz gerekebilir.
  /* USER CODE END USART3_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
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
  * where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * \retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
