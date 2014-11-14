/******************** (C) COPYRIGHT 2008 STMicroelectronics ********************
* File Name          : main.c
* Author             : MCD Application Team
* Version            : Vx.x.x
* Date               :
* Description        : Main program body
********************************************************************************
* THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
* WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE TIME.
* AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY DIRECT,
* INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING FROM THE
* CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE CODING
* INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
*******************************************************************************/

/* Includes ------------------------------------------------------------------*/
#include "stm32f10x.h"
#include "v_includes.h"

/* Private variables ---------------------------------------------------------*/
#define LOOP_DLY_100US    2000
#define LOOP_DLY_250ns    2
#define NUMBER_OF_SAMPLES 100
#define FOUR_MILLISECONDS_TIMEOUT 4300
#define TWENTY_MILLISECONDS_TIMEOUT 20300

Int32U   CriticalSecCntr;

bool     configured_true= FALSE;
uint8_t  Rx_data1;
uint8_t  Rx_data2;
uint8_t  Rx_data3;


float  *sram_memory     = ((float *)(SRAM_BASE + 0xB00));
uint32_t  sram_mem_offset  = 0x0;

uint32_t  Dummy_var   = 0;


uint32_t  Res0_content;
uint32_t  Res1_content;
float     Res4_content;
float     Capacitance_ratio;
uint32_t  Status_content;

// For mathematical calculations
int         i;
int         j;
double      sum;
float      mean = 0; 
float       diff;
float       square;
float       numerator = 0;
float       variance;


/* Private functions ---------------------------------------------------------*/
void Dly100us(void *arg);
void Simple_delay_750ns(void *arg);
void GPIOs_LED_Test(void);
void SPI_GPIOs_Init(void);
void Embed_SRAM_Init(void);
void SPI_Interface_Init(void);

void Write_Emb_SRAM(float ESRAM_24bit_data);

float find_variance(uint16_t sample_count);


void pcap01_send_1byte (uint8_t pcap01_opcode_byte);
uint32_t pcap01_read_3bytes(uint8_t read_opcode_addr);
void pcap01_wr_config_reg (uint8_t opcode_address, uint32_t config_reg_data);



/*******************************************************************************
* Function Name  : main
* Description    : Main program.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/


void main(void)
{

  ENTR_CRT_SECTION();
  /* Setup STM32 system (clock, PLL and Flash configuration) */
  SystemInit();

  EXT_CRT_SECTION();

   
/* Infinite loop */
 while (Dummy_var!=1000)// Dummy_var!=15 // To Control the loop 
  {

    if(configured_true==FALSE)
    {  
      configured_true = TRUE;
      GPIOs_LED_Test();
      SPI_GPIOs_Init();
      SPI_Interface_Init();
      Embed_SRAM_Init();
      
      SPI_SSOutputCmd (SPI2, ENABLE); // Enabling the NSS Output during transmission
 
      GPIO_WriteBit(GPIOA, GPIO_Pin_4, Bit_SET);     // SSN TO Pcap01 - Set to High for reset

      pcap01_wr_config_reg(0xC0, 0x4200FF);   // Argument 1 = opcode + addr = 'b11 + {addr 000000}
      pcap01_wr_config_reg(0xC1, 0x201022);   // Argument 1 = opcode + addr = 'b11 + {addr 000001}
      pcap01_wr_config_reg(0xC2, 0x0F460B);   // Argument 1 = opcode + addr = 'b11 + {addr 000010}
      pcap01_wr_config_reg(0xC3, 0x0D0001);   // Argument 1 = opcode + addr = 'b11 + {addr 000011}
      pcap01_wr_config_reg(0xC4, 0x040011);   // Argument 1 = opcode + addr = 'b11 + {addr 000100}
                                              // CONTINUOUS MODE 
      pcap01_wr_config_reg(0xC5, 0x000000);   // Argument 1 = opcode + addr = 'b11 + {addr 000101}
      pcap01_wr_config_reg(0xC6, 0x004340);   // Argument 1 = opcode + addr = 'b11 + {addr 000110}
      pcap01_wr_config_reg(0xC7, 0x1F0000);   // Argument 1 = opcode + addr = 'b11 + {addr 000111}
      pcap01_wr_config_reg(0xC8, 0x800010);   // Argument 1 = opcode + addr = 'b11 + {addr 001000}
      pcap01_wr_config_reg(0xC9, 0xFF000F);   // Argument 1 = opcode + addr = 'b11 + {addr 001001}
      pcap01_wr_config_reg(0xCA, 0x180047);   // Argument 1 = opcode + addr = 'b11 + {addr 001010}
      pcap01_wr_config_reg(0xCB, 0x000000);   // Argument 1 = opcode + addr = 'b11 + {addr 001011}
      pcap01_wr_config_reg(0xCC, 0x000000);   // Argument 1 = opcode + addr = 'b11 + {addr 001100}
      pcap01_wr_config_reg(0xCD, 0x000000);   // Argument 1 = opcode + addr = 'b11 + {addr 001101}
      pcap01_wr_config_reg(0xCE, 0x000000);   // Argument 1 = opcode + addr = 'b11 + {addr 001110}
      pcap01_wr_config_reg(0xCF, 0x000000);   // Argument 1 = opcode + addr = 'b11 + {addr 001111}
      pcap01_wr_config_reg(0xD0, 0x000000);   // Argument 1 = opcode + addr = 'b11 + {addr 010000}
      pcap01_wr_config_reg(0xD1, 0x000000);   // Argument 1 = opcode + addr = 'b11 + {addr 010001}
      pcap01_wr_config_reg(0xD2, 0x000000);   // Argument 1 = opcode + addr = 'b11 + {addr 010010}
      pcap01_wr_config_reg(0xD3, 0x200000);   // Argument 1 = opcode + addr = 'b11 + {addr 010011}
      pcap01_wr_config_reg(0xD4, 0x000001);   // Argument 1 = opcode + addr = 'b11 + {addr 010100}
      
      pcap01_send_1byte(0x8A);          // Partial reset
    }

    
    while (GPIO_ReadInputDataBit(GPIOD, GPIO_Pin_4)==1)
    {} // wait for INTERRUPT

    Res0_content     = pcap01_read_3bytes(0x40); // Argument = opcode + addr = 'b01 + {addr 000000}
    Res1_content     = pcap01_read_3bytes(0x41); // Argument = opcode + addr = 'b01 + {addr 000001}
    Res4_content     = pcap01_read_3bytes(0x44); // Argument = opcode + addr = 'b01 + {addr 000100}
    //Status_content   = pcap01_read_3bytes(0x48); // Argument = opcode + addr = 'b01 + {addr 001000}
    

    float res1_float = Res1_content;
    Capacitance_ratio = res1_float/(2097152); //2097152 = 2^21
    
    printf("\n RES1(Cap)   Cap_ratio= %d, %f", Res1_content, Capacitance_ratio);
 //   Write_Emb_SRAM(Capacitance_ratio); // Storing the Capacitance ratio RES4 from PCap01
                   
    Dummy_var++;
    
  }	// while with Dummy_var
 
       find_variance(1000); //Dummy_var-1
                            // Taking 2000 meas values, using values 1000-2000 for variance for best results

} //main







/*************************************************************************
 * Function Name: SPI_Interface_Init
 * Parameters: Int32U Clk, Int32U Width
 * Return: none
 *
 * Description: Init SPI1 Interface
 *
 *************************************************************************/

void SPI_Interface_Init(void)
{
    // Initialising the SPI1 interface
  SPI_InitTypeDef SPI_InitStructure;
  
  // Clock Enable and Reset release
   RCC_APB2PeriphResetCmd (RCC_APB2Periph_SPI1, DISABLE);
   RCC_APB2PeriphClockCmd (RCC_APB2Periph_SPI1, ENABLE);
  
    
  SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex; // All are defined in stm32f10x_spi.h
  SPI_InitStructure.SPI_Mode = SPI_Mode_Master;
  SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;
  SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;
  SPI_InitStructure.SPI_CPHA = SPI_CPHA_2Edge;    
  SPI_InitStructure.SPI_NSS  = SPI_NSS_Soft;
  SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_16;
  SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
  SPI_Init(SPI1, &SPI_InitStructure);
  
  SPI_Cmd(SPI1, ENABLE); // Enabling the SPI1 Interface
}
/*************************************************************************
 * Function Name: SPI_GPIOs_Init
 * Parameters: Int32U Clk, Int32U Width
 * Return: none
 *
 * Description: Init GPIOs used in SPI interface
 *
 *************************************************************************/

void SPI_GPIOs_Init(void)
{
GPIO_InitTypeDef GPIO_InitStructure; // GPIO_InitTypeDef defined in library

  // Enable GPIO clock and release reset

  RCC_APB2PeriphResetCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB | RCC_APB2Periph_GPIOC | RCC_APB2Periph_GPIOD | RCC_APB2Periph_GPIOE |
                         RCC_APB2Periph_AFIO,  DISABLE);
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB | RCC_APB2Periph_GPIOC | RCC_APB2Periph_GPIOD | RCC_APB2Periph_GPIOE |
                         RCC_APB2Periph_AFIO,  ENABLE);

  // Configure SPI1_CLK  - PA5
  //           SPI1_MOSI - PA7
  //           SPI1_MISO - PA6
  // Chip select SPI1_NSS - PA4
  
  // External Interrupt Input line on PD4
  
  
// SPI1_NSS - PA4
  
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4; 
  GPIO_Init(GPIOA, &GPIO_InitStructure);

//  SPI1_CLK  - PA5
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5;
  GPIO_Init(GPIOA, &GPIO_InitStructure);


//   SPI1_MISO - PA6
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;
  GPIO_Init(GPIOA, &GPIO_InitStructure);

  
//   SPI1_MOSI - PA7
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7;
  GPIO_Init(GPIOA, &GPIO_InitStructure);
 
  SPI_I2S_DeInit(SPI1);

 
  
// External Interrupt Input  - PD4
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;
  GPIO_Init(GPIOD, &GPIO_InitStructure);

  
// SPI ENABLE Output for the evaluation kit  - PD3
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
  GPIO_Init(GPIOD, &GPIO_InitStructure);
  GPIO_WriteBit(GPIOD, GPIO_Pin_3, Bit_RESET); // SPI Enable = 0 for SPI Mode
  
}


/*************************************************************************
 * Function Name: GPIOs_LED_Test
 * Return: none
 *
 * Description: Init GPIOs to turn on LEDs on PC10, PC11 and PC12 on the test platform
 *
 *************************************************************************/
void GPIOs_LED_Test(void)
{

  GPIO_InitTypeDef GPIO_InitStructure; // GPIO_InitTypeDef defined in library


// Test GPIO for driving LEDs
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_OD;
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10; 
  GPIO_Init(GPIOC, &GPIO_InitStructure);
  GPIO_WriteBit(GPIOC, GPIO_Pin_10, Bit_RESET);
  
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_OD;
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11; 
  GPIO_Init(GPIOC, &GPIO_InitStructure);
  GPIO_WriteBit(GPIOC, GPIO_Pin_11, Bit_RESET);

  
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_OD;
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12; 
  GPIO_Init(GPIOC, &GPIO_InitStructure);
  GPIO_WriteBit(GPIOC, GPIO_Pin_12, Bit_RESET);

}

/*******************************************************************************
* Function Name  : Embed_SRAM_Init
* Description    : Embedded SRAM initialization
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void Embed_SRAM_Init (void)
{
 // Releasing reset for Embedded SRAM
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_SRAM, ENABLE);
}

/*******************************************************************************
* Function Name  : Write_Emb_SRAM
* Description    : Write to embedded SRAM
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/

void Write_Emb_SRAM(float ESRAM_24bit_data)
{
  // NOTE: While checking the SRAM contents, please note that the address of
  // consecutively stored results differs by 4. So, for 100 values, address range
  // is from 20000B00 to 20000B00+'d(100*4) = 20000C90
  sram_memory[sram_mem_offset] = ESRAM_24bit_data;
  sram_mem_offset++; // offset increments by 4 every time 
}

//*************************************************************************


/*************************************************************************
 * Function Name: Dly100us
 * Parameters: u32 Dly
 *
 * Return: none
 *
 * Description: Delay Dly * 100us
 *
 *************************************************************************/
void Dly100us(void *arg)
{
u32 Dely = (u32)arg;
 while(Dely--)
 {
   for(int i = LOOP_DLY_100US; i; i--);
 }
}


void Simple_delay_750ns(void *arg) // With arg 1, gives 750ns delay
{
u32 Dely = (u32)arg;
   for(int i = Dely; (i!=0); i--);

}
/*************************************************************************
 * Function Name: find_variance
 * Parameters: Calculates variance (S.D^2)
 *
 * Return: none
 *
 * Description: 
 *
 *************************************************************************/

float find_variance(uint16_t sample_count)
{
  sram_mem_offset = 1000;  // Using values 1000-2000 for Variance calculation
  sum=0;
  float data;
  
  for(i=0;i<sample_count;i++)
  {
    data = sram_memory[sram_mem_offset]; 
    sum = sum + data;
    sram_mem_offset ++; // For the sram_memory, the address increments by 4 everytime
  }
  
  mean = sum/sample_count;
  sram_mem_offset =1000; // Using values 1000-2000 for Variance calculation
  
  for(j=0;j<sample_count;j++)
  {
    data = sram_memory[sram_mem_offset];
    diff      = data - mean;
    square    = diff * diff;
    numerator += square;
    sram_mem_offset ++;
  }
  
  //variance = numerator/(sample_count-1); // Sample variance
  variance = numerator/(sample_count); // Population variance variance
  return variance;
}
//*********************************************************************









/*************************************************************************
 * Function Name: pcap01_send_1byte
 * Parameters: Opcode byte
 *
 * Return: none
 *
 * Description: Writes the Opcode to GP21
 *
 *************************************************************************/

void pcap01_send_1byte (uint8_t pcap01_opcode_byte)
{
     GPIO_WriteBit(GPIOA, GPIO_Pin_4, Bit_RESET); // Deactivating Reset
     SPI_I2S_SendData(SPI2, pcap01_opcode_byte);     // OPCODE TO PCAP01 
     while (SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_TXE)==0) {} 
 
     GPIO_WriteBit(GPIOA, GPIO_Pin_4, Bit_SET);   // Reset to PCAP01
}
//*********************************************************************
/*************************************************************************
 * Function Name: pcap01_wr_config_reg
 * Parameters: Address byte, 4 bytes of Configuration
 *
 * Return: none
 *
 * Description: Writes the config.reg. specified in GP21 with the data 
 *
 *************************************************************************/

void pcap01_wr_config_reg (uint8_t opcode_address, uint32_t config_reg_data)
{

   uint8_t Data_Byte_Lo   = config_reg_data;
   uint8_t Data_Byte_Mid  = config_reg_data>>8;
   uint8_t Data_Byte_Hi   = config_reg_data>>16;


GPIO_WriteBit(GPIOA, GPIO_Pin_4, Bit_RESET); // Deactivating Reset
      while (SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_TXE)==0) {} 

      SPI_I2S_SendData(SPI2, opcode_address);  // CFG WR OPCODE+ADDRESS

      while (SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_TXE)==0) {}
      SPI_I2S_SendData(SPI2, Data_Byte_Hi);  // DATA BYTE HIGH 

      while (SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_TXE)==0) {}
       SPI_I2S_SendData(SPI2, Data_Byte_Mid);  // DATA MID 

      while (SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_TXE)==0) {}
       SPI_I2S_SendData(SPI2, Data_Byte_Lo);  // DATA LOW

      while (SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_TXE)==0) {}

      GPIO_WriteBit(GPIOA, GPIO_Pin_4, Bit_SET);
      //Simple_delay_750ns((void*)1);

}
//*********************************************************************

/*************************************************************************
 * Function Name: pcap01_read_3bytes
* Parameters: Opcode = b'01 + <addr5.....0> = 8 bit opcode_addr
 *
 * Return: 3 bytes from the specified read address
 *
 * Description: Reads 3 bytes from an address in PCAP01
 *
 *************************************************************************/

uint32_t pcap01_read_3bytes(uint8_t read_opcode_addr)
{

    uint32_t Result_read=0;

     //.............. Result  word = 4 Byte = 32  bits......................
      GPIO_WriteBit(GPIOA, GPIO_Pin_4, Bit_RESET);
      
      SPI_I2S_SendData(SPI2, read_opcode_addr);  // READ OPCODE + Address 1
            
      while (SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_TXE)==RESET);
             
       //Compulsory reads to DR and SR to clear OVR, so that next incoming data is saved
       SPI_I2S_ReceiveData(SPI2);                     // To clear OVR
       SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_TXE); // To clear OVR
                 
       //Reading byte1
       SPI_I2S_SendData(SPI2, 0x00FF);  // DUMMY WRITE
                   
       // Wait until the RX buffer is not empty, then read the received data
       while (SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_RXNE)==0){}
       Result_read |= SPI_I2S_ReceiveData(SPI2); //  Read
       Result_read = Result_read<<8;
             
       //Reading byte2
       SPI_I2S_SendData(SPI2, 0x00FF);  // DUMMY WRITE
            
       // Wait until RX buffer is not empty, then read the received data
       while (SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_RXNE)==0) {}
       Result_read |= SPI_I2S_ReceiveData(SPI2); //  Read
       Result_read = Result_read<<8;
                 
       //Reading byte3
       SPI_I2S_SendData(SPI2, 0x00FF);  // DUMMY WRITE
                   
       // Wait until the RX buffer is not empty, then read the received data
       while (SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_RXNE)==0){}
       Result_read |= SPI_I2S_ReceiveData(SPI2); //  Read
/*       Result_read = Result_read<<8;
                  
       //Reading byte4
       SPI_I2S_SendData(SPI2, 0x00FF);  // DUMMY WRITE
                   
       // Wait until the RX buffer is not empty, then read the received data
       while (SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_RXNE)==0){}
       Result_read |= SPI_I2S_ReceiveData(SPI2); //  Read
*/            
  
       GPIO_WriteBit(GPIOA, GPIO_Pin_4, Bit_SET);     // Reset to PCAP01
       //Simple_delay_750ns((void*)1);
 
       return Result_read;
}                 
//*********************************************************************



/******************* (C) COPYRIGHT 2008 STMicroelectronics *****END OF FILE****/
