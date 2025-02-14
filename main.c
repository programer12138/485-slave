#include "main.h"          // 引入主配置头文件
#include "fatfs.h"         // 引入文件系统相关的头文件
#include "SDdriver.h"      // 引入SD卡驱动的头文件
#include "sys.h"           // 引入系统级配置和函数的头文件
#include "delay.h"         // 引入延时函数的头文件
#include "rs485.h"         // 引入RS485通信相关的头文件
#include "wodead.h"        // 引入额外的头文件（具体内容未知，可能是自定义的）
#include <string.h>

SPI_HandleTypeDef hspi1;   // 定义SPI接口的句柄
UART_HandleTypeDef huart1; // 定义UART接口的句柄

void SystemClock_Config(void);    // 声明系统时钟配置函数
static void MX_GPIO_Init(void);   // 声明GPIO初始化函数
static void MX_SPI1_Init(void);   // 声明SPI1接口初始化函数
static void MX_USART1_UART_Init(void); // 声明UART1接口初始化函数
void WritetoSD(BYTE write_buff[], uint8_t bufSize); // 声明写入SD卡的函数
void myCMDReadData(void);
// 实现fputc函数，用于UART传输
int fputc(int ch, FILE *f){
    HAL_UART_Transmit(&huart1, (unsigned char *)&ch, 1, 0xFFFF); 
    return ch;
}

void Get_SDCard_Capacity(void){
    //FRESULT result; // 定义一个FRESULT类型的变量用于存储文件系统操作的结果
    FATFS FS;       // 定义一个FATFS结构体，用于存储文件系统的相关信息
    FATFS *fs;      // 定义一个指向FATFS结构体的指针
    DWORD fre_clust, AvailableSize;//, UsedSize; // 定义变量用于存储空闲簇数、可用大小和已用大小
    uint16_t TotalSpace; // 定义一个变量用于存储SD卡的总空间
    uint8_t res;         // 定义一个变量用于存储操作的结果
    //printf("尝试获取SD卡容量信息！ \r\n"); // 打印信息表示已进入此函数
    res = SD_init(); // 初始化SD卡
    if(res == 1)    // 如果初始化成功
    {
        //printf("SD初始化成功 \r\n"); // 打印初始化成功的消息
    }
    else
    {
        //printf("SD初始化失败\r\n"); // 否则，打印初始化失败的消息
    }

    res = f_mount(&FS, "0:", 1); // 挂载文件系统，"0:"是逻辑驱动号
    if (res != FR_OK) // 如果挂载不成功
    {
        //printf("挂载失败(%d)\r\n", result); // 打印挂载失败的消息
    }

    res = f_getfree("0:", &fre_clust, &fs); // 获取SD卡的空闲簇信息
    if (res == FR_OK) // 如果成功获取
    {
        // 计算总空间、可用空间和已用空间
        TotalSpace = (uint16_t)(((fs->n_fatent - 2) * fs->csize) / 2 / 1024);
        AvailableSize = (uint16_t)((fre_clust * fs->csize) / 2 / 1024);
        DWORD UsedSize = TotalSpace - AvailableSize;              
        // 打印总空间、可用空间和已用空间信息
        printf("\r\n%d MB 总驱动空间。\r\n%lu MB 可用。\r\n%lu MB 已使用。\r\n", TotalSpace, AvailableSize, UsedSize);
    }
    else 
    {
        //printf("获取SD卡容量失败 (%d)\r\n", result); // 如果获取空间信息失败，打印错误信息
    }		
} 

int main(void){
    HAL_Init();                                 // 初始化HAL库
    Stm32_Clock_Init(RCC_PLL_MUL9);             // 初始化时钟，这里使用了9倍频
    delay_init(72);                             // 初始化延时函数，参数72根据系统时钟频率来定
    MX_GPIO_Init();                             // 初始化GPIO
    MX_SPI1_Init();                             // 初始化SPI1
    MX_FATFS_Init();                            // 初始化FAT文件系统
    MX_USART1_UART_Init();                      // 初始化USART1
    uint8_t aRxBuffer1;                         // 定义一个接收缓冲区变量
    HAL_UART_Receive_IT(&huart1, &aRxBuffer1, 1); // UART接收中断初始化
    //printf("初始化中...\r\n");
    //Get_SDCard_Capacity();                      // 获取SD卡容量信息
    FATFS fs;                                   // 定义文件系统对象
    FIL file;                                   // 定义文件对象
    uint8_t res;                                // 定义结果变量
    RS485_Init(115200);                         // 初始化RS485通信，波特率115200
    uint8_t AdcRegData[ADS126x_NUM_REG];        // 定义数组存储ADC寄存器读值
    uint8_t AdcRegData2[ADS126x_NUM_REG];        // 定义数组存储ADC寄存器读值
    uint8_t m;
    ADC_SPI_Init();                             // 初始化ADC SPI
    set_delay_table();
    //printf("初始化结束\r\n"); 
    //printf("ADC SPI初始化完成。\r\n");
    ADS_REST(GPIO_PIN_SET);                     // ADC复位
    ADS_REST(GPIO_PIN_RESET);
    ADS_REST(GPIO_PIN_SET);
	uint8_t WriteRegData[ADS126x_NUM_REG];						
	ADS1262ReadRegister(ID, ADS126x_NUM_REG, AdcRegData);	
	uint8_t WriteRegValues[] = {   //0x0A     //05                                       // 0000 1001  0x09           F0 03
    0x03,0x11,0x05,0x00,0x80,0x04,0x0A,0x00,0x05,0x00,0x00,0x00,0x40,0xBB,0x00,0x00,0x09,0x00,0x00,0x00,0x00};
    ADS1262WriteRegister(0x00, sizeof(WriteRegValues), WriteRegValues); 
    ADS1262ReadRegister(0x00, sizeof(WriteRegValues), AdcRegData); 
//    for(int m = 0; m < sizeof(WriteRegValues); m++){
//        printf("%x  ", AdcRegData[m]); // 打印寄存器值
//    }
    // 0x01 D1 + D2 -  0x0A D1 + AGND -  0x1A D2 + AGND -  0x2A D3 + AGND - 
    //自校正程序
    #if 0
    00000000h
		F
		1111
		32位
		00000 12s
		
		
		0000 0000h  0000 000F  -> 0.00000005V  
		#endif 
    #if 0
//    printf("自校正程序\r\n");
	uint8_t WriteRegData[ADS126x_NUM_REG];						
	ADS1262ReadRegister(ID, ADS126x_NUM_REG, AdcRegData);	
	uint8_t WriteRegValues[] = {
    0x03,0x11,0x05,0x00,0x80,0x04,0xFF,0x00,0x00,0x00,0x00,0x00,0x40,0xBB,0x00,0x00,0x00,0x00,0x00,0x00,0x00};
    ADS1262WriteRegister(0x00, sizeof(WriteRegValues), WriteRegValues); 
    ADS1262ReadRegister(0x00, sizeof(WriteRegValues), AdcRegData); 
//    for(int m = 0; m < sizeof(WriteRegValues); m++){
//        printf("%x  ", AdcRegData[m]); // 打印寄存器值
//    }
//    printf("\r\n");
    ADS_STAR(HIG);
    ADS_NCSS(LOW); 
    ADC_SPI_ReadWriteByte(0x19);
    while (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_1) == GPIO_PIN_SET);
    ADS_NCSS(HIG);
    ADS_STAR(LOW);
    ADS1262ReadRegister(ID, ADS126x_NUM_REG, AdcRegData2);	
	uint8_t WriteRegValues2[] = {
    0x03,0x11,0x05,0x00,0x80,0x04,0x0A ,0x00,0x00,0x00,0x00,0x00,0x40,0xBB,0x00,0x00,0x00,0x00,0x00,0x00,0x00};
    uint8_t OFF[] ={0x0A,0xE6,0x00,0x00};
    //ADS1262ReadRegister(0x07, sizeof(OFF), &OFF[1]); 
    ADS1262WriteRegister(0x06, sizeof(OFF), &OFF[0]); 
    ADS1262ReadRegister(0x00, sizeof(WriteRegValues2), AdcRegData2); 
//    for(int m = 0; m < sizeof(WriteRegValues2); m++){
//        printf("%x  ", AdcRegData2[m]); // 打印寄存器值
//    }
//    printf("自校已完成\r\n");
    #endif
    
    ADS_STAR(HIG);
		uint8_t count=1;
		uint8_t count2=0;
		
		

    while (1) 
    {
			if(count2==0)
				//说明没有进到核心位置，没有发生数据传输，不用重新配置，也不用加一
				{
					count=count;
					if(count==1)
						{
								uint8_t WriteRegValues[] = {
						0x03,0x11,0x05,0x00,0x40,0x04,0x10,0xF3,0x01,0x00,0x0f,0x87,0x40,0xBB,0x00,0x00,0x09,0x00,0x00,0x00,0x00};
						ADS1262WriteRegister(0x00, sizeof(WriteRegValues), WriteRegValues); 
							}
				}
			else{
					count++;
					switch(count)		
					{					
						case 1:
								{
										uint8_t WriteRegValues[] = {
								0x03,0x11,0x05,0x00,0x40,0x04,0x10,0xF3,0x01,0x00,0x0f,0x87,0x40,0xBB,0x00,0x00,0x09,0x00,0x00,0x00,0x00};
								ADS1262WriteRegister(0x00, sizeof(WriteRegValues), WriteRegValues); 
									}
						case 2:
									{
										uint8_t WriteRegValues[] = {
								0x03,0x11,0x05,0x00,0x40,0x04,0x20,0xF3,0x01,0x00,0x0f,0x87,0x40,0xBB,0x00,0x00,0x09,0x00,0x00,0x00,0x00};
								ADS1262WriteRegister(0x00, sizeof(WriteRegValues), WriteRegValues); 
									}
						case 3:
									{
										uint8_t WriteRegValues[] = {
								0x03,0x11,0x05,0x00,0x40,0x04,0x30,0xF3,0x01,0x00,0x0f,0x87,0x40,0xBB,0x00,0x00,0x09,0x00,0x00,0x00,0x00};
								ADS1262WriteRegister(0x00, sizeof(WriteRegValues), WriteRegValues); 
											count=1;
									}
					}
					count2=0;
				}		
			
        while (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_1) == GPIO_PIN_SET);
        uint8_t status = 0;
        uint32_t adcValue = 0;
        uint8_t dataBytes[4] = {0};
        ADS_NCSS(LOW); 
        ADC_SPI_ReadWriteByte(0x12); 
        status = ADC_SPI_ReadWriteByte(0x00);//状态位
        for (int i = 0; i < 4; ++i) {
            dataBytes[i] = ADC_SPI_ReadWriteByte(0x00);
            adcValue = (adcValue << 8) | dataBytes[i];
        }//数据位
        uint8_t crc = ADC_SPI_ReadWriteByte(0x00);
        ADS_NCSS(HIG); 
        int32_t intval = 0;
        long double value = 0;
        if (adcValue >= 0x80000000) intval = -(~adcValue + 0x00000001);            // 转换为有符号数
        else    intval = adcValue;
        value = ((long double)intval/0x80000000 ); // 计算实际值
        value *= 2.49628; //4.99787
//				 printf("hello");
				//地址，功能码，长度，数据，校验校验
		
				
/////////////////////调用通信///////////////////////////////////////////
				unsigned char rec_buf[8]={0};
				unsigned char add=0x01;
				
				
				unsigned char sendbuf[16]={0};
				unsigned char func=0x03;
				

				
if(RS485_RX_CNT>4)//??还是不明白为什么是4
{
	
		unsigned int crc;
	unsigned int rccrc;
		crc= crc16(&RS485_RX_BUF[0], RS485_RX_CNT-2);                             //计算校验码
		rccrc=RS485_RX_BUF[RS485_RX_CNT-2]*256 + RS485_RX_BUF[RS485_RX_CNT-1];  //收到的校验码
			if(crc ==  rccrc)                                                           //数据包符合CRC校验规则
		{ 
			if(RS485_RX_BUF[0] == add)         //确认数据包是否是发给本设备的 
			{
			count2=1;


				unsigned int i=0;
				sendbuf[i]=add;i++;
				sendbuf[i]=func;i++;
			////长度sendbuf[i]=12;i++  原来这里起到了换行的作用，原因不明。
				sendbuf[i]=12;i++;//这里起到了换行的作用，原因不明。
			
				sprintf(sendbuf+i,"%+.8Lf",value);i+=11;//以字符格式在串口助手中显示的时候，0不会显示，数字转字符要加‘0’
				//这里必须是.8不然显示不全。但是看来好像不是以字符串形式发送的，因为那个系统的数据在16进制显示下才能对上，否则是乱码
					crc=crc16(sendbuf,i);                //CRC校验
					sendbuf[i++]=crc/256;                //发送CRC的值高位
					sendbuf[i++]=crc%256;                //发送CRC的值低位
				
				i=0;
				RS485_Send_Data(sendbuf,16);
//				
				
			}
		}
		RS485_RX_CNT=0;
	}
//unsigned char sendbuf[18]={0};
//unsigned char a=0x01+'0';
//sendbuf[3]=a;
//RS485_Send_Data(sendbuf,18);
///////////////////////调用通信///////////////////////////////////////////
				
				
				
				
				
//        //2.5097     13e 
//        printf("状态字节: 0x%X ,电压读数:%x ,转换值:%.8Lf\r\n",status ,adcValue, value);
//       // printf("测得电压读数:%x ,转换值:%.8Lf\r\n",adcValue, value);
//        //value *= 5;//                       // 转换为电压值
//        //printf("%10x    %10x    %10.8Lf\r\n", status, adcValue, value);
//        // RS485发送数据
//        char buffer[100];
//        int len = sprintf(buffer, "485:状态字节: 0x%X ,读数:%x ,转换值:%.8Lf\r\n", status, adcValue, value);
//        RS485_Send_Data((u8 *)buffer, len);  // 通过RS485发送数据

}

	///f_close(&file);		
	///f_mount(NULL,"0:",1);
	
}


static void MX_SPI1_Init(void){
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
}

static void MX_USART1_UART_Init(void){
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
}

static void MX_GPIO_Init(void){
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  __HAL_RCC_GPIOA_CLK_ENABLE();
  HAL_GPIO_WritePin(SD_CS_GPIO_Port, SD_CS_Pin, GPIO_PIN_RESET);
  GPIO_InitStruct.Pin = SD_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(SD_CS_GPIO_Port, &GPIO_InitStruct);
}

void Error_Handler(void){
	
}

#ifdef  USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{ 

}
#endif
