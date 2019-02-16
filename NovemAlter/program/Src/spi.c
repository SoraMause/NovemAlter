/**
  ******************************************************************************
  * File Name          : SPI.c
  * Description        : This file provides code for the configuration
  *                      of the SPI instances.
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

/* Includes ------------------------------------------------------------------*/
#include "spi.h"

/* USER CODE BEGIN 0 */
#include "variable.h"

static int16_t gyro_offset_cnt = 0; 
static int8_t  gyro_calc_flag = 1;
static float gyro_z_offset = 0.0f;
/* USER CODE END 0 */

SPI_HandleTypeDef hspi3;

/* SPI3 init function */
void MX_SPI3_Init(void)
{

  hspi3.Instance = SPI3;
  hspi3.Init.Mode = SPI_MODE_MASTER;
  hspi3.Init.Direction = SPI_DIRECTION_2LINES;
  hspi3.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi3.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi3.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi3.Init.NSS = SPI_NSS_SOFT;
  hspi3.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_32;
  hspi3.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi3.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi3.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi3.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi3) != HAL_OK)
  {
    Error_Handler();
  }

}

void HAL_SPI_MspInit(SPI_HandleTypeDef* spiHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(spiHandle->Instance==SPI3)
  {
  /* USER CODE BEGIN SPI3_MspInit 0 */

  /* USER CODE END SPI3_MspInit 0 */
    /* SPI3 clock enable */
    __HAL_RCC_SPI3_CLK_ENABLE();
  
    __HAL_RCC_GPIOC_CLK_ENABLE();
    /**SPI3 GPIO Configuration    
    PC10     ------> SPI3_SCK
    PC11     ------> SPI3_MISO
    PC12     ------> SPI3_MOSI 
    */
    GPIO_InitStruct.Pin = GPIO_PIN_10|GPIO_PIN_11|GPIO_PIN_12;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF6_SPI3;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /* USER CODE BEGIN SPI3_MspInit 1 */

  /* USER CODE END SPI3_MspInit 1 */
  }
}

void HAL_SPI_MspDeInit(SPI_HandleTypeDef* spiHandle)
{

  if(spiHandle->Instance==SPI3)
  {
  /* USER CODE BEGIN SPI3_MspDeInit 0 */

  /* USER CODE END SPI3_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_SPI3_CLK_DISABLE();
  
    /**SPI3 GPIO Configuration    
    PC10     ------> SPI3_SCK
    PC11     ------> SPI3_MISO
    PC12     ------> SPI3_MOSI 
    */
    HAL_GPIO_DeInit(GPIOC, GPIO_PIN_10|GPIO_PIN_11|GPIO_PIN_12);

  /* USER CODE BEGIN SPI3_MspDeInit 1 */

  /* USER CODE END SPI3_MspDeInit 1 */
  }
} 

/* USER CODE BEGIN 1 */
///////////////////////////////////////////////////////////////////////
// spi read 1 byte
// [argument] Register
// [Substitutiong] nothing
// [return] 1byte data
///////////////////////////////////////////////////////////////////////
uint8_t read_byte( uint8_t reg )
{
  uint8_t ret,val;
  HAL_GPIO_WritePin( gyro_cs_GPIO_Port, gyro_cs_Pin, GPIO_PIN_RESET ); //cs = 0;
  ret = reg | 0x80;   // 読み込みのとき�???��?��??��?��?上位ビ??��?��?トをHighにする
  HAL_SPI_Transmit( &hspi3, &ret,1,100 );
  HAL_SPI_Receive( &hspi3,&val,1,100 ); 
  HAL_GPIO_WritePin( gyro_cs_GPIO_Port,gyro_cs_Pin, GPIO_PIN_SET );  //cs = 1;
  return val;
}

///////////////////////////////////////////////////////////////////////
// spi read 1 byte upper bit ( signed )
// [argument] Register
// [Substitutiong] nothing
// [return] 1byte upper bit data 
///////////////////////////////////////////////////////////////////////
int16_t read_shift_byte( uint8_t reg )
{
  uint8_t address,val;
  int16_t ret;
  HAL_GPIO_WritePin( gyro_cs_GPIO_Port, gyro_cs_Pin, GPIO_PIN_RESET );
  address = reg | 0x80 ;
  HAL_SPI_Transmit( &hspi3, &address,1,100 );
  HAL_SPI_Receive( &hspi3,&val,1,100 );
  ret = (int16_t)(val<< 8);
  HAL_GPIO_WritePin( gyro_cs_GPIO_Port,gyro_cs_Pin, GPIO_PIN_SET );
  return ret;
}

///////////////////////////////////////////////////////////////////////
// spi write 1 byte
// [argument] Register
// [Substitutiong] write data
// [return] nothong
///////////////////////////////////////////////////////////////////////
void write_byte( uint8_t reg, uint8_t val )
{
  uint8_t ret;
  ret = reg & 0x7F ;
  HAL_GPIO_WritePin( gyro_cs_GPIO_Port, gyro_cs_Pin, GPIO_PIN_RESET );
  HAL_SPI_Transmit( &hspi3, &ret,1,100 ); // 書き込みアドレス
  HAL_SPI_Transmit( &hspi3, &val,1,100 ); // 書き込み
  HAL_GPIO_WritePin( gyro_cs_GPIO_Port,gyro_cs_Pin, GPIO_PIN_SET );
}

///////////////////////////////////////////////////////////////////////
// set up mpu-6500
// [argument] nothing
// [Substitutiong] nothing
// [return] nothong
///////////////////////////////////////////////////////////////////////
void MPU6500_init( void )
{
  uint8_t who_am_i;
  uint8_t recheck_who_am_i;
  
  HAL_Delay( 100 );
  who_am_i = read_byte( MPU6500_RA_WHO_AM_I );
  printf( "\r\n0x%x\r\n",who_am_i );

  if ( who_am_i != 0x70 ){
    HAL_Delay( 300 );
    recheck_who_am_i = read_byte( MPU6500_RA_WHO_AM_I );
    if ( recheck_who_am_i != 0x70 ){
      while(1){
        printf( "gyro_error\r");
      }
    } else {
      printf( "recheck_who_am_i = 0x%x\r\n", recheck_who_am_i );
    }
  }

  HAL_Delay( 50 );

  // solve sleep 
  write_byte( MPU6500_RA_PWR_MGMT_1, 0x00 );

  HAL_Delay( 50 );

	// DLPF_CFG = 0 : Gyro's LPF is not avaiable
	// not avaiable FIFO
  write_byte( MPU6500_RA_CONFIG, 0x00 );

  HAL_Delay( 50 );

  // Gyro scope is full scale
	write_byte(MPU6500_RA_GYRO_CONFIG, 0x18 );
  HAL_Delay( 50 );

  // Accel scope is full scale
  write_byte( MPU6500_RA_ACCEL_CONFIG, 0x18 );

  gyro_calc_flag = 1;

}


///////////////////////////////////////////////////////////////////////
// get mpu-6500 read z axis data
// [argument] nothing
// [Substitutiong] nothing
// [return] read z ( rad / sec)
///////////////////////////////////////////////////////////////////////
float MPU6500_read_gyro_z( void )
{
  int16_t gyro_z;
  float omega;

  gyro_z = (int16_t)( read_shift_byte(MPU6500_RA_GYRO_ZOUT_H) | read_byte(MPU6500_RA_GYRO_ZOUT_L) );

  if ( rotation_ideal.velocity == 0.0f && gyro_z == -256 ) gyro_z = 0;

  // GYRO FACTOR 
  // 180 * PI ( rad/sec )

  omega = (float)( ( gyro_z - gyro_z_offset ) / GYRO_FACTOR);

  return omega;
}

///////////////////////////////////////////////////////////////////////
// get mpu-6500 read z axis accel
// [argument] nothing
// [Substitutiong] nothing
// [return] read z ( rad / sec)
///////////////////////////////////////////////////////////////////////
float MPU6500_read_accel_x( void )
{
  int16_t accel_x;
  float accel;

  accel_x = (int16_t)( read_shift_byte(MPU6500_RA_ACCEL_XOUT_H) | read_byte(MPU6500_RA_ACCEL_XOUT_L) );

  // GYRO FACTOR 
  // 180 * PI ( rad/sec )
  
  accel = (float)( accel_x / ACCEL_FACTOR);

  return accel;
}

///////////////////////////////////////////////////////////////////////
// get mpu-6500 read z axis offset start
// [argument] nothing
// [Substitutiong] nothing
// [return] read z ( rad / sec)
///////////////////////////////////////////////////////////////////////
void MPU6500_z_axis_offset_calc_start( void )
{
  gyro_z_offset = 0.0f;
  gyro_offset_cnt = 0; 
  gyro_calc_flag = 0;
}

///////////////////////////////////////////////////////////////////////
// get mpu-6500 read z axis offset calc
// [argument] nothing
// [Substitutiong] nothing
// [return] read z ( rad / sec)
///////////////////////////////////////////////////////////////////////
void MPU6500_z_axis_offset_calc( void )
{
  int16_t gyro_z;

  gyro_z = (int16_t)( read_shift_byte(MPU6500_RA_GYRO_ZOUT_H) | read_byte(MPU6500_RA_GYRO_ZOUT_L) );

  // GYRO FACTOR ( deg / sec )

  if ( gyro_z == -256 ) gyro_z = 0;

  if ( gyro_offset_cnt < 256 ){
    gyro_z_offset += (float)gyro_z;
    gyro_offset_cnt++;
  } else {
    gyro_z_offset /= 256.0f;
    gyro_calc_flag = 1;
  }
}

int8_t MPU6500_calc_check( void )
{
  return gyro_calc_flag;
}

// debug ON
float checkGyroOffset()
{
  return gyro_z_offset;
}
/* USER CODE END 1 */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
