#include "main.h"          // ����������ͷ�ļ�
#include "fatfs.h"         // �����ļ�ϵͳ��ص�ͷ�ļ�
#include "SDdriver.h"      // ����SD��������ͷ�ļ�
#include "sys.h"           // ����ϵͳ�����úͺ�����ͷ�ļ�
#include "delay.h"         // ������ʱ������ͷ�ļ�
#include "rs485.h"         // ����RS485ͨ����ص�ͷ�ļ�
#include "wodead.h"        // ��������ͷ�ļ�����������δ֪���������Զ���ģ�
#include <string.h>

SPI_HandleTypeDef hspi1;   // ����SPI�ӿڵľ��
UART_HandleTypeDef huart1; // ����UART�ӿڵľ��

void SystemClock_Config(void);    // ����ϵͳʱ�����ú���
static void MX_GPIO_Init(void);   // ����GPIO��ʼ������
static void MX_SPI1_Init(void);   // ����SPI1�ӿڳ�ʼ������
static void MX_USART1_UART_Init(void); // ����UART1�ӿڳ�ʼ������
void WritetoSD(BYTE write_buff[], uint8_t bufSize); // ����д��SD���ĺ���
void myCMDReadData(void);
// ʵ��fputc����������UART����
int fputc(int ch, FILE *f){
    HAL_UART_Transmit(&huart1, (unsigned char *)&ch, 1, 0xFFFF); 
    return ch;
}

void Get_SDCard_Capacity(void){
    //FRESULT result; // ����һ��FRESULT���͵ı������ڴ洢�ļ�ϵͳ�����Ľ��
    FATFS FS;       // ����һ��FATFS�ṹ�壬���ڴ洢�ļ�ϵͳ�������Ϣ
    FATFS *fs;      // ����һ��ָ��FATFS�ṹ���ָ��
    DWORD fre_clust, AvailableSize;//, UsedSize; // ����������ڴ洢���д��������ô�С�����ô�С
    uint16_t TotalSpace; // ����һ���������ڴ洢SD�����ܿռ�
    uint8_t res;         // ����һ���������ڴ洢�����Ľ��
    //printf("���Ի�ȡSD��������Ϣ�� \r\n"); // ��ӡ��Ϣ��ʾ�ѽ���˺���
    res = SD_init(); // ��ʼ��SD��
    if(res == 1)    // �����ʼ���ɹ�
    {
        //printf("SD��ʼ���ɹ� \r\n"); // ��ӡ��ʼ���ɹ�����Ϣ
    }
    else
    {
        //printf("SD��ʼ��ʧ��\r\n"); // ���򣬴�ӡ��ʼ��ʧ�ܵ���Ϣ
    }

    res = f_mount(&FS, "0:", 1); // �����ļ�ϵͳ��"0:"���߼�������
    if (res != FR_OK) // ������ز��ɹ�
    {
        //printf("����ʧ��(%d)\r\n", result); // ��ӡ����ʧ�ܵ���Ϣ
    }

    res = f_getfree("0:", &fre_clust, &fs); // ��ȡSD���Ŀ��д���Ϣ
    if (res == FR_OK) // ����ɹ���ȡ
    {
        // �����ܿռ䡢���ÿռ�����ÿռ�
        TotalSpace = (uint16_t)(((fs->n_fatent - 2) * fs->csize) / 2 / 1024);
        AvailableSize = (uint16_t)((fre_clust * fs->csize) / 2 / 1024);
        DWORD UsedSize = TotalSpace - AvailableSize;              
        // ��ӡ�ܿռ䡢���ÿռ�����ÿռ���Ϣ
        printf("\r\n%d MB �������ռ䡣\r\n%lu MB ���á�\r\n%lu MB ��ʹ�á�\r\n", TotalSpace, AvailableSize, UsedSize);
    }
    else 
    {
        //printf("��ȡSD������ʧ�� (%d)\r\n", result); // �����ȡ�ռ���Ϣʧ�ܣ���ӡ������Ϣ
    }		
} 

int main(void){
    HAL_Init();                                 // ��ʼ��HAL��
    Stm32_Clock_Init(RCC_PLL_MUL9);             // ��ʼ��ʱ�ӣ�����ʹ����9��Ƶ
    delay_init(72);                             // ��ʼ����ʱ����������72����ϵͳʱ��Ƶ������
    MX_GPIO_Init();                             // ��ʼ��GPIO
    MX_SPI1_Init();                             // ��ʼ��SPI1
    MX_FATFS_Init();                            // ��ʼ��FAT�ļ�ϵͳ
    MX_USART1_UART_Init();                      // ��ʼ��USART1
    uint8_t aRxBuffer1;                         // ����һ�����ջ���������
    HAL_UART_Receive_IT(&huart1, &aRxBuffer1, 1); // UART�����жϳ�ʼ��
    //printf("��ʼ����...\r\n");
    //Get_SDCard_Capacity();                      // ��ȡSD��������Ϣ
    FATFS fs;                                   // �����ļ�ϵͳ����
    FIL file;                                   // �����ļ�����
    uint8_t res;                                // ����������
    RS485_Init(115200);                         // ��ʼ��RS485ͨ�ţ�������115200
    uint8_t AdcRegData[ADS126x_NUM_REG];        // ��������洢ADC�Ĵ�����ֵ
    uint8_t AdcRegData2[ADS126x_NUM_REG];        // ��������洢ADC�Ĵ�����ֵ
    uint8_t m;
    ADC_SPI_Init();                             // ��ʼ��ADC SPI
    set_delay_table();
    //printf("��ʼ������\r\n"); 
    //printf("ADC SPI��ʼ����ɡ�\r\n");
    ADS_REST(GPIO_PIN_SET);                     // ADC��λ
    ADS_REST(GPIO_PIN_RESET);
    ADS_REST(GPIO_PIN_SET);
	uint8_t WriteRegData[ADS126x_NUM_REG];						
	ADS1262ReadRegister(ID, ADS126x_NUM_REG, AdcRegData);	
	uint8_t WriteRegValues[] = {   //0x0A     //05                                       // 0000 1001  0x09           F0 03
    0x03,0x11,0x05,0x00,0x80,0x04,0x0A,0x00,0x05,0x00,0x00,0x00,0x40,0xBB,0x00,0x00,0x09,0x00,0x00,0x00,0x00};
    ADS1262WriteRegister(0x00, sizeof(WriteRegValues), WriteRegValues); 
    ADS1262ReadRegister(0x00, sizeof(WriteRegValues), AdcRegData); 
//    for(int m = 0; m < sizeof(WriteRegValues); m++){
//        printf("%x  ", AdcRegData[m]); // ��ӡ�Ĵ���ֵ
//    }
    // 0x01 D1 + D2 -  0x0A D1 + AGND -  0x1A D2 + AGND -  0x2A D3 + AGND - 
    //��У������
    #if 0
    00000000h
		F
		1111
		32λ
		00000 12s
		
		
		0000 0000h  0000 000F  -> 0.00000005V  
		#endif 
    #if 0
//    printf("��У������\r\n");
	uint8_t WriteRegData[ADS126x_NUM_REG];						
	ADS1262ReadRegister(ID, ADS126x_NUM_REG, AdcRegData);	
	uint8_t WriteRegValues[] = {
    0x03,0x11,0x05,0x00,0x80,0x04,0xFF,0x00,0x00,0x00,0x00,0x00,0x40,0xBB,0x00,0x00,0x00,0x00,0x00,0x00,0x00};
    ADS1262WriteRegister(0x00, sizeof(WriteRegValues), WriteRegValues); 
    ADS1262ReadRegister(0x00, sizeof(WriteRegValues), AdcRegData); 
//    for(int m = 0; m < sizeof(WriteRegValues); m++){
//        printf("%x  ", AdcRegData[m]); // ��ӡ�Ĵ���ֵ
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
//        printf("%x  ", AdcRegData2[m]); // ��ӡ�Ĵ���ֵ
//    }
//    printf("��У�����\r\n");
    #endif
    
    ADS_STAR(HIG);
		uint8_t count=1;
		uint8_t count2=0;
		
		

    while (1) 
    {
			if(count2==0)
				//˵��û�н�������λ�ã�û�з������ݴ��䣬�����������ã�Ҳ���ü�һ
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
        status = ADC_SPI_ReadWriteByte(0x00);//״̬λ
        for (int i = 0; i < 4; ++i) {
            dataBytes[i] = ADC_SPI_ReadWriteByte(0x00);
            adcValue = (adcValue << 8) | dataBytes[i];
        }//����λ
        uint8_t crc = ADC_SPI_ReadWriteByte(0x00);
        ADS_NCSS(HIG); 
        int32_t intval = 0;
        long double value = 0;
        if (adcValue >= 0x80000000) intval = -(~adcValue + 0x00000001);            // ת��Ϊ�з�����
        else    intval = adcValue;
        value = ((long double)intval/0x80000000 ); // ����ʵ��ֵ
        value *= 2.49628; //4.99787
//				 printf("hello");
				//��ַ�������룬���ȣ����ݣ�У��У��
		
				
/////////////////////����ͨ��///////////////////////////////////////////
				unsigned char rec_buf[8]={0};
				unsigned char add=0x01;
				
				
				unsigned char sendbuf[16]={0};
				unsigned char func=0x03;
				

				
if(RS485_RX_CNT>4)//??���ǲ�����Ϊʲô��4
{
	
		unsigned int crc;
	unsigned int rccrc;
		crc= crc16(&RS485_RX_BUF[0], RS485_RX_CNT-2);                             //����У����
		rccrc=RS485_RX_BUF[RS485_RX_CNT-2]*256 + RS485_RX_BUF[RS485_RX_CNT-1];  //�յ���У����
			if(crc ==  rccrc)                                                           //���ݰ�����CRCУ�����
		{ 
			if(RS485_RX_BUF[0] == add)         //ȷ�����ݰ��Ƿ��Ƿ������豸�� 
			{
			count2=1;


				unsigned int i=0;
				sendbuf[i]=add;i++;
				sendbuf[i]=func;i++;
			////����sendbuf[i]=12;i++  ԭ���������˻��е����ã�ԭ������
				sendbuf[i]=12;i++;//�������˻��е����ã�ԭ������
			
				sprintf(sendbuf+i,"%+.8Lf",value);i+=11;//���ַ���ʽ�ڴ�����������ʾ��ʱ��0������ʾ������ת�ַ�Ҫ�ӡ�0��
				//���������.8��Ȼ��ʾ��ȫ�����ǿ������������ַ�����ʽ���͵ģ���Ϊ�Ǹ�ϵͳ��������16������ʾ�²��ܶ��ϣ�����������
					crc=crc16(sendbuf,i);                //CRCУ��
					sendbuf[i++]=crc/256;                //����CRC��ֵ��λ
					sendbuf[i++]=crc%256;                //����CRC��ֵ��λ
				
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
///////////////////////����ͨ��///////////////////////////////////////////
				
				
				
				
				
//        //2.5097     13e 
//        printf("״̬�ֽ�: 0x%X ,��ѹ����:%x ,ת��ֵ:%.8Lf\r\n",status ,adcValue, value);
//       // printf("��õ�ѹ����:%x ,ת��ֵ:%.8Lf\r\n",adcValue, value);
//        //value *= 5;//                       // ת��Ϊ��ѹֵ
//        //printf("%10x    %10x    %10.8Lf\r\n", status, adcValue, value);
//        // RS485��������
//        char buffer[100];
//        int len = sprintf(buffer, "485:״̬�ֽ�: 0x%X ,����:%x ,ת��ֵ:%.8Lf\r\n", status, adcValue, value);
//        RS485_Send_Data((u8 *)buffer, len);  // ͨ��RS485��������

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
