#include "math.h"
#include "main.h"

I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c2;

void SystemClock_Config(void);
static void GPIO_Init(void);
static void I2C1_Init(void);
static void I2C2_Init(void);

#define MPU6050_ADDR 0xD0


#define SMPLRT_DIV_REG 0x19
#define GYRO_CONFIG_REG 0x1B
#define ACCEL_CONFIG_REG 0x1C
#define ACCEL_XOUT_H_REG 0x3B
#define TEMP_OUT_H_REG 0x41
#define GYRO_XOUT_H_REG 0x43
#define PWR_MGMT_1_REG 0x6B
#define WHO_AM_I_REG 0x75

#define WINDOW_SIZE 2    // Kích thuoc cua so luu tru du lieu gia toc
#define THRESHOLD 1.25

float acc_window[WINDOW_SIZE][3];
int window_index = 0;
float current = 0;

int detectStep(float ax, float ay, float az) {
  float acc_magnitude = sqrt(ax*ax + ay*ay + az*az);

  // Thêm d? li?u gia t?c m?i vào c?a s? luu tr?
  acc_window[window_index][0] = ax;
  acc_window[window_index][1] = ay;
  acc_window[window_index][2] = az;
  window_index = (window_index + 1) % WINDOW_SIZE;

  // Tính toán pic c?a gia t?c
  float peak_value = 0.0;
  for (int i = 0; i < WINDOW_SIZE; i++) {
    float acc_magnitude_i = sqrt(acc_window[i][0]*acc_window[i][0] + acc_window[i][1]*acc_window[i][1] + acc_window[i][2]*acc_window[i][2]);
    if (acc_magnitude_i > peak_value) {
      peak_value = acc_magnitude_i;
    }
  }
	current = peak_value;

  // Ki?m tra n?u pic vu?t ngu?ng và d? s? bu?c t?i thi?u
  if (peak_value > THRESHOLD) {
    return 1;  // Tr? v? 1 d? ch? d?m bu?c chân
  }

  return 0;  // Không d?m bu?c chân
}

int16_t Accel_X_RAW = 0;
int16_t Accel_Y_RAW = 0;
int16_t Accel_Z_RAW = 0;

int16_t Gyro_X_RAW = 0;
int16_t Gyro_Y_RAW = 0;
int16_t Gyro_Z_RAW = 0;

float Ax, Ay, Az, Gx, Gy, Gz;

#define SLAVE_ADDRESS_LCD 0x4E // change this according to ur setup

void lcd_send_cmd (char cmd)
{
  char data_u, data_l;
	uint8_t data_t[4];
	data_u = (cmd&0xf0);
	data_l = ((cmd<<4)&0xf0);
	data_t[0] = data_u|0x0C;  //en=1, rs=0
	data_t[1] = data_u|0x08;  //en=0, rs=0
	data_t[2] = data_l|0x0C;  //en=1, rs=0
	data_t[3] = data_l|0x08;  //en=0, rs=0
	HAL_I2C_Master_Transmit (&hi2c1, SLAVE_ADDRESS_LCD,(uint8_t *) data_t, 4, 100);
}

void lcd_send_data (char data)
{
	char data_u, data_l;
	uint8_t data_t[4];
	data_u = (data&0xf0);
	data_l = ((data<<4)&0xf0);
	data_t[0] = data_u|0x0D;  //en=1, rs=0
	data_t[1] = data_u|0x09;  //en=0, rs=0
	data_t[2] = data_l|0x0D;  //en=1, rs=0
	data_t[3] = data_l|0x09;  //en=0, rs=0
	HAL_I2C_Master_Transmit (&hi2c1, SLAVE_ADDRESS_LCD,(uint8_t *) data_t, 4, 100);
}

void lcd_clear (void)
{
	lcd_send_cmd (0x80);
	for (int i=0; i<70; i++)
	{
		lcd_send_data (' ');
	}
}

void lcd_put_cur(int row, int col)
{
    switch (row)
    {
        case 0:
            col |= 0x80;
            break;
        case 1:
            col |= 0xC0;
            break;
    }

    lcd_send_cmd (col);
}


void lcd_init (void)
{
	// 4 bit initialisation
	HAL_Delay(50);  // wait for >40ms
	lcd_send_cmd (0x30);
	HAL_Delay(5);  // wait for >4.1ms
	lcd_send_cmd (0x30);
	HAL_Delay(1);  // wait for >100us
	lcd_send_cmd (0x30);
	HAL_Delay(10);
	lcd_send_cmd (0x20);  // 4bit mode
	HAL_Delay(10);

  // dislay initialisation
	lcd_send_cmd (0x28); // Function set --> DL=0 (4 bit mode), N = 1 (2 line display) F = 0 (5x8 characters)
	HAL_Delay(1);
	lcd_send_cmd (0x08); //Display on/off control --> D=0,C=0, B=0  ---> display off
	HAL_Delay(1);
	lcd_send_cmd (0x01);  // clear display
	HAL_Delay(1);
	HAL_Delay(1);
	lcd_send_cmd (0x06); //Entry mode set --> I/D = 1 (increment cursor) & S = 0 (no shift)
	HAL_Delay(1);
	lcd_send_cmd (0x0C); //Display on/off control --> D = 1, C and B = 0. (Cursor and blink, last two bits)
}

void lcd_send_string (char *str)
{
	while (*str) lcd_send_data (*str++);
}


void MPU6050_Init (void)
{
	uint8_t check;
	uint8_t Data;

	// check device ID WHO_AM_I

	HAL_I2C_Mem_Read (&hi2c1, MPU6050_ADDR,WHO_AM_I_REG,1, &check, 1, 1000);

	if (check == 104)  // 0x68 will be returned by the sensor if everything goes well
	{
		// power management register 0X6B we should write all 0's to wake the sensor up
		Data = 0;
		HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, PWR_MGMT_1_REG, 1,&Data, 1, 1000);

		// Set DATA RATE of 1KHz by writing SMPLRT_DIV register
		Data = 0x07;
		HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, SMPLRT_DIV_REG, 1, &Data, 1, 1000);

		// Set accelerometer configuration in ACCEL_CONFIG Register
		// XA_ST=0,YA_ST=0,ZA_ST=0, FS_SEL=0 -> ? 2g
		Data = 0x00;
		HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, ACCEL_CONFIG_REG, 1, &Data, 1, 1000);

		// Set Gyroscopic configuration in GYRO_CONFIG Register
		// XG_ST=0,YG_ST=0,ZG_ST=0, FS_SEL=0 -> ? 250 ?/s
		Data = 0x00;
		HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, GYRO_CONFIG_REG, 1, &Data, 1, 1000);
	}

}


void MPU6050_Read_Accel (void)
{
	uint8_t Rec_Data[6];

	// Read 6 BYTES of data starting from ACCEL_XOUT_H register

	HAL_I2C_Mem_Read (&hi2c1, MPU6050_ADDR, ACCEL_XOUT_H_REG, 1, Rec_Data, 6, 1000);

	Accel_X_RAW = (int16_t)(Rec_Data[0] << 8 | Rec_Data [1]);
	Accel_Y_RAW = (int16_t)(Rec_Data[2] << 8 | Rec_Data [3]);
	Accel_Z_RAW = (int16_t)(Rec_Data[4] << 8 | Rec_Data [5]);

	/*** convert the RAW values into acceleration in 'g'
	     we have to divide according to the Full scale value set in FS_SEL
	     I have configured FS_SEL = 0. So I am dividing by 16384.0
	     for more details check ACCEL_CONFIG Register              ****/

	Ax = Accel_X_RAW/16384.0;
	Ay = Accel_Y_RAW/16384.0;
	Az = Accel_Z_RAW/16384.0;
}


void MPU6050_Read_Gyro (void)
{
	uint8_t Rec_Data[6];

	// Read 6 BYTES of data starting from GYRO_XOUT_H register

	HAL_I2C_Mem_Read (&hi2c1, MPU6050_ADDR, GYRO_XOUT_H_REG, 1, Rec_Data, 6, 1000);

	Gyro_X_RAW = (int16_t)(Rec_Data[0] << 8 | Rec_Data [1]);
	Gyro_Y_RAW = (int16_t)(Rec_Data[2] << 8 | Rec_Data [3]);
	Gyro_Z_RAW = (int16_t)(Rec_Data[4] << 8 | Rec_Data [5]);


	Gx = Gyro_X_RAW/131.0;
	Gy = Gyro_Y_RAW/131.0;
	Gz = Gyro_Z_RAW/131.0;
}


int main(void)
{
	char buf[4];

	HAL_Init();
	SystemClock_Config();
	GPIO_Init();
	I2C1_Init();
	I2C2_Init();

	lcd_init();
	MPU6050_Init();
	lcd_send_string("initialized");
	HAL_Delay(1000);  // wait for 1 sec
	lcd_clear();

	// Khoi tao bien dem buoc chan
	int stepCount = 0;
	int pauseRequest = 0;
	int stepDetected = 0;
	float currentVector, previousVector = 0, diffVector = 0;
	float thresholdVector = 0.19;
	float delayTime = 100;
	
	// Khoi tao cau hinh GPIO
	GPIO_InitTypeDef GPIO_InitStruct;
	
	// Bat Clock cho GPIOA
	__HAL_RCC_GPIOA_CLK_ENABLE();

	// Cau hình chân PA0 và PA1 là input
	GPIO_InitStruct.Pin = GPIO_PIN_0 | GPIO_PIN_1;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_PULLUP; // S? d?ng pull-up resistor
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
	
	// Bat Clock cho GPIOC
  __HAL_RCC_GPIOC_CLK_ENABLE();
  
  // Cau hình chân PC13 là output (green led)
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
	
	// Bat Clock cho GPIOB
  __HAL_RCC_GPIOB_CLK_ENABLE();
  
  // Cau hình chân PB4 là output (red led)
  GPIO_InitStruct.Pin = GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
	
	// Cau hình SysTick timer
	SysTick_Config(SystemCoreClock / 2); // T?n s? SysTick là 1Hz (1 giây)
	
	// cau hinh chan PA1 có muc do ngat uu tien cao hon PA0
	HAL_NVIC_SetPriority(EXTI0_IRQn, 3, 0); //sw0
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

  HAL_NVIC_SetPriority(EXTI1_IRQn, 4, 0); //sw1
  HAL_NVIC_EnableIRQ(EXTI1_IRQn);

	while (1)
	{
		// Ðoc trang thái tu switch 1
		uint8_t switch1_state = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0);

		// Ðoc trang thái tu switch 2
		uint8_t switch2_state = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_1);

		if (switch1_state == GPIO_PIN_RESET) //sw1 là pause|resume
		{
			// Tam dung hoc tiep tuc dem buoc chân
			HAL_Delay(50); // delay tranh nhieu
			if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0) == GPIO_PIN_RESET)
			{
				pauseRequest = !pauseRequest;
				lcd_clear();
			}
		}

		if (switch2_state == GPIO_PIN_RESET) //sw2 là reset stepCount
		{
			// Reset bien stepCount bang 0
			stepCount = 0;
			lcd_clear();
		}
		
		if (pauseRequest) {
			//tat nhay den xanh chan PC13
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);
			//bat den do PB4 (red led)
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_RESET);
			
			//in ra man hinh thong bao tam dung
			lcd_send_cmd(0x80 | 0x00);  // goto 1,1
			lcd_send_string("Please press SW1");
			
			lcd_send_cmd(0x80 | 0x40);  // goto 2,1
			lcd_send_string("to continue!");
		} else {
			// Nháy LED chân PC13 voi tan so 1Hz
			static uint32_t previousTick = 0;
			uint32_t currentTick = HAL_GetTick();

			if (currentTick - previousTick >= 500)
			{
				previousTick = currentTick;
				HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
			}
			
			//tat red led PB4
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_SET);
			
			// read the Accelerometer and Gyro values
			MPU6050_Read_Accel();
			MPU6050_Read_Gyro();

			//Code dem buoc chân

			/*currentVector = sqrt(Ax * Ax + Ay * Ay + Az * Az);
			diffVector = fabs(currentVector - previousVector);

			// Kiem tra neu góc theta vuot quá nguong threshold
			// Neu tong bình phuong gia toc vuot quá nguong và chua phát hien buuc chân truoc dó
			if (diffVector > thresholdVector && previousVector && !stepDetected)
			{
				stepCount++; 
			}
			else if (diffVector <= thresholdVector)
			{
				stepDetected = 0;
			} */
			
			//code dem buoc chan
			if (detectStep(Ax, Ay, Az)) {
					if (!stepDetected) {
						stepCount++;
						stepDetected = 1;
					}
			} else {
					stepDetected = 0;
			}
			
			lcd_send_cmd(0x80 | 0x00);  // goto 1,1
			lcd_send_string("StepCount: ");
			sprintf(buf, "%d", stepCount);
			lcd_send_string(buf);
			lcd_send_string("");

			lcd_send_cmd(0x80 | 0x40);  // goto 2,1

			sprintf(buf, "%.3f", current);
			lcd_send_string(buf);

			previousVector = currentVector;
			HAL_Delay(delayTime);  // wait for a while
		}
	}
}


/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the CPU, AHB and APB busses clocks
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks
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

static void I2C1_Init(void)
{
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }

}
static void I2C2_Init(void)
{
  hi2c2.Instance = I2C2;
  hi2c2.Init.ClockSpeed = 100000;
  hi2c2.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }
}

static void GPIO_Init(void)
{

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

}

void Error_Handler(void)
{
}

#ifdef  USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{

}
#endif
