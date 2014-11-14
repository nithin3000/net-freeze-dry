/* PCAP intialising & interfacing to nRF 51422 using SPI bus
 * 
 * 
 * Example extracted from Nordic Semiconductor's SPI master example. All Rights Reserved.
 *
 */

/** @file
* @brief SPI interface to PCAP.
* @defgroup spi_master_example SPI master loopback usage example
* @{
*
* @brief SPI master example.
*
* This example needs that the slave is configured to transmit the received bytes. That is the slave
* behaves as a loopback device for the master. The loopback can also be achieved without using a slave device at all by wiring MOSI and
* MISO pins of the spi master together. @ref TX_RX_MSG_LENGTH number of bytes are transmitted through the master and the received bytes are
* verified to be the same as transmitted. IF there is an error, gpio pin for relevant spi module is set high to show the error @sa ERROR_PIN_SPI0,
* @sa ERROR_PIN_SPI1. If there is error from both modules that is if both pins are set high, then this application loops for ever
*
*/

#include "spi_master.h"
#include "nrf_delay.h"
#include "common.h"
#include "spi_master_config.h"
#include "limits.h"
#include "nRF6350.h"
#include "nrf_gpio.h"
#include "boards.h"
#include <string.h>
#include <stdio.h>

/* PCAP config register address definitions ** ask matthew if this is the right intialisation ***/
#define conf_reg0      (0)   
#define conf_reg1      (1)
#define conf_reg2      (2)
#define conf_reg3      (3)
#define conf_reg4      (4)
#define conf_reg5      (5)
#define conf_reg6      (6)
#define conf_reg7      (7)
#define conf_reg8      (8)
#define conf_reg9      (9)
#define conf_reg10      (10)

/*
#define conf_reg11      (11)
#define conf_reg12      (12)
#define conf_reg13      (13)
#define conf_reg14      (14)
#define conf_reg15      (15)
#define conf_reg16	    (16)
#define conf_reg17	    (17)
#define conf_reg18	    (18)
#define conf_reg19	    (19)
#define conf_reg20	    (20)
*/

/* PCAP  read register address definitions ** ask matthew if this is the right intialisation ** */
#define read_reg0      (0)   // C0 LSB
#define read_reg1      (1)	// C1/C0
#define read_reg2      (2)	// C2/C0
#define read_reg3      (3)	// C3/C0
#define read_reg4      (4)	// C4/C0
#define read_reg5      (5)	// C5/C0
#define read_reg6      (6)	// C6/C0
#define read_reg7      (7)	// C7/C0
#define read_stat      (8)	// Status register
#define read_reg8      (11)	// unused
#define read_reg9      (12)	// unused
#define read_reg10      (13) // R0/Rref
#define read_reg11      (14) // R0/Rref

/* Config registration parameters*/
// reconfig all the uint to be of 8 16 32 there is no 24 or 4 or 2

//Register 0 
#define MEMCOMP ((uint8_t) 0) // 0 = disable , 1 = 5 byte , 2 = 33 byte 3 = byte
#define ECC_MODE ((uint8_t) 0x00) // OTP internal error detection and repair 0x00 disable , 0x0F Double, 0xF0 Quad
#define AUTOBOOT_DIS ((uint8_t) 0x0F) // 0xF slave operation, 0x0 stand alone operation
#define MEM_LOCK_DIS ((uint8_t) 0x0F) // OxO activatin the memory read-out blocker 0xF Readoutremain un-blocled 

// Register 2
#define CMEAS_PORT_EN ((uint8_t) 0x0F) // Each bit activates individual PCx ports
#define CMEAS_BITS ((uint8_t) 4) // 1 = grounded cap , 4 =floating single capacitance, 8 = floating differential capacitances
#define RDCHG_INT_SEL ((uint8_t) 6) // 4 = 180 kohm , 5 = 90kohm , 6 = 30 kohm, 7 = 10 kohm

// Register 3
#define CY_CLK_SEL ((uint8_t) 0) // 0 = 20us 2 = 1us 3 = 0.25us 
#define SEQ_TIME ((uint8_t) 0x0D) // 0 = off otherwise s = seq_time  then trig perious will 20us*2^(s+1) 
#define CMEAS_FAKE ((uint8_t) 0) // no of fake block measurements 
#define C_AVRG  ((uint16_t) 1) // Averaging the CDC results
 
// Register 4
#define CMEAS_STARTPIN ((uint8_t) 0) // 0 = PG0, 1 = PG1 , .... 
#define CMEAS_TRIG_SEL ((uint8_t) 2) // 0 = softwaretrigger only , 1 = continuous mode, 2 = timer-triggered mode , 3 = pulse-triggered mode
#define CMEAS_CYTIME ((uint16_t) 0) // CDC cycle time = (CMEAS_CYTIME+1)*clock_period
#define TMEAS_CYTIME ((uint8_t) 0 ) // 0 = 140 us , 1 = 280us
#define TMEAS_STARTPIN ((uint8_t) 0) // 0 = PG0, 1 = PG1 ... pin for pulse triggered temperature measurement.
#define TMEAS_TRIG_SEL ((uint8_t) 0) // trigger source for the temperature measurement 0 = off/opcode triggered , 1 = cmeas-triggered 2 = timer-triggered mode 3= pulse-triggered mode. 

//Register 5
#define T_AVRG ((uint8_t) 0) // 0( 1 = no overaging, 1 (4 fold averaging) , 2 (8-fold averaging) , 3(16-fold averaging) 
#define TMEAS_TRIG_PREDIV ((uint32_t) 0) //zero counts as one, set zero for hygrometers etc 

//Register 6 
#define TMEAS_FAKE ((uint8_t) 0) // 0 = 2 dummy measuremtns,  1 = 8 dummy measurement
#define TMEAS_7BITS ((uint8_t) 0) 

//Register 8 
#define DSP_SRAM_SEL ((uint8_t) 1) // 0 = OTP , 1 = SRAM 
#define DSP_START ((uint8_t) 0) // start command
#define DSP_STARTONOVL ((uint8_t) 0) // 0 = default is mandatory 
#define DSP_STARTONTEMP ((uint8_t) 0) // 0 = default, mandatory with standard fimware 03.01.xx
#define DSP_STARTPIN ((uint8_t) 0) // 0 = PG0, 1 = PG1, 2 = PG2, 3 = PG3
#define DSP_FF_IN ((uint8_t) 0x00) // Bit 12 = PG0 , Bit 13 = PG1, ....
#define DSP_WATCHDOG_LENGTH ((uint8_t) 0) //
#define DSP_MOFLO_EN ((uint8_t) 0) //bit 9 for PG1 bit8 for PG0
#define DSP_SPEED ((uint8_t) 1) // 1 = standard (fast), 3 = low-current (slow)
#define INT2PG2 ((uint8_t) 0) /// interrupt pin reroute to PG2
#define PG1_X_G3 ((uint8_t) 0) //pulse codes reroute from PG1 to PG3
#define PG0_X_G2 ((uint8_t) 0) //pulse codes reroute from PG0 to PG2

//Register 9 
#define PG_DIR_IN ((uint8_t) 0x0F) // toggles outputs to input(PG3/bit23 to PG0/bit 20)
#define PG_PULL_UP ((uint8_t) 0x0F) // Activates pull-up resistor in PG0 to PG3 lines
#define PI_EN ((uint8_t) 0x00) // enables pulse-density or pulse0width mode code generation. PWM0/PDM0 can be output at ports PG0 or PG2. PWM1/PDM1 can be output at ports PG1 or PG3.
#define PI1_CLK_SEL ((uint8_t) 0x00) //Base frequency for the pulse code interfaces based on low-freq or external high-freq oscillator
#define PI0_CLK_SEL ((uint8_t) 0x00)
#define PI1_RES ((uint8_t) 3) // Resolution of pulse code interfaces: 0 = 7 bit, 1 = 8 bit , 2 = 9 bit, 3 = 10 bit.
#define PI0_RES ((uint8_t) 3) // Resolution of pulse code interfaces (see above)

//Register 10 
#define V_CORE_CTL ((uint8_t) 0x47) // 0x87 = Low-current , 0x47 standard 
 



static uint8_t tx_data[48]; /*!< SPI TX buffer */
static uint8_t rx_data[48]; /*!< SPI RX buffer */
static uint8_t MSG_LEN;
#define DELAY_MS               100        /*!< Timer Delay in milli-seconds */




/***** Bit Packing function to generate 32 bit function ****
 * @param a, b, c, d, e : Variables containing the bits to be combined
 * @param nb, nc, nd, ne : Position of the bits to be combined
 * @retval p 32bit integer to be returned
 */

uint32_t pack(uint32_t a, uint32_t b, uint8_t nb, uint32_t c, uint8_t nc, uint32_t d, uint8_t nd, uint32_t e, uint8_t ne)
	{
		uint32_t p = a;
		//suggestion move by size of 
		p = (p << nb)| b ;
		p = (p << nc)| c ;
		p = (p << nd)| d ;
		p = (p << ne)| e ;
		return p;
	}

/** Sets SPI interface.
 * @param lsb_first If true, least significant bits are transferred first
 * @param mod_num spi module to be used, either SPI0 or SPI1 from enumeration SPIModuleNumber
 * @retval Pointer to SPIaddress 
 * @retval false Error occurred
 * PCAP settings : 
 * 				@SPImode 1 , @BITsequence , MSB first 
 */
 uint32_t* pcap_spi_set(SPIModuleNumber mod_num)
{
  uint32_t *PCAP_spi_address = spi_master_init(mod_num, SPI_MODE1, (bool) 0);

  return PCAP_spi_address;
	
/** @def debug_init
 * If debug is enabled @ref DEBUG, then this function will configure @ref DEBUG_EVENT_READY_PIN to toggle (using GPIOTE) everytime
 * READY_EVENTS are generated in the SPI
 * @note This flag will configure GPIOTE CONFIG0 and PPI channel 0, do not enable DEBUG while using two spi modules in parallel
 */
// #ifdef DEBUG
//     *(uint32_t *)0x40000504 = 0xC007FFDF; // Workaround for PAN_028 rev1.1 anomaly 23 - System: Manual setup is required to enable use of peripherals


//     if ( NRF_SPI0_BASE == (uint32_t)spi_base_address )
//     {
//         nrf_gpio_cfg_output(DEBUG_EVENT_READY_PIN0);

//         /*lint -e{845} // A zero has been given as right argument to operator '|'" */
//         NRF_GPIOTE->CONFIG[0] = (GPIOTE_CONFIG_POLARITY_Toggle << GPIOTE_CONFIG_POLARITY_Pos) |
//                                 (DEBUG_EVENT_READY_PIN0 << GPIOTE_CONFIG_PSEL_Pos) |
//                                 (GPIOTE_CONFIG_MODE_Task << GPIOTE_CONFIG_MODE_Pos);

//         NRF_PPI->CH[0].EEP = (uint32_t)&(((NRF_SPI_Type *)spi_base_address)->EVENTS_READY);
//         NRF_PPI->CH[0].TEP = (uint32_t)&NRF_GPIOTE->TASKS_OUT[0];
//         NRF_PPI->CHEN |= (PPI_CHEN_CH0_Enabled << PPI_CHEN_CH0_Pos);
//     }

//     if ( NRF_SPI1_BASE == (uint32_t)spi_base_address )
//     {
//         nrf_gpio_cfg_output(DEBUG_EVENT_READY_PIN1);

//         /*lint -e{845} // A zero has been given as right argument to operator '|'" */
//         NRF_GPIOTE->CONFIG[1] = (GPIOTE_CONFIG_POLARITY_Toggle << GPIOTE_CONFIG_POLARITY_Pos) |
//                                 (DEBUG_EVENT_READY_PIN1 << GPIOTE_CONFIG_PSEL_Pos) |
//                                 (GPIOTE_CONFIG_MODE_Task << GPIOTE_CONFIG_MODE_Pos);

//         NRF_PPI->CH[1].EEP = (uint32_t)&(((NRF_SPI_Type *)spi_base_address)->EVENTS_READY);
//         NRF_PPI->CH[1].TEP = (uint32_t)&NRF_GPIOTE->TASKS_OUT[1];
//         NRF_PPI->CHEN |= (PPI_CHEN_CH1_Enabled << PPI_CHEN_CH1_Pos);
//     }
// #endif /* DEBUG */
}

/***** Transmits/Recieves data using SPI. ****
 * @param PCAP_spi_address Pointer containing the set spi address 
 * @param MSG_LEN length of the message to be transmitted
 * @param tx_data pointer containing the transmitted data
 * @param rx_data pointer containing the recieved data
 * @retval True 
 * @retval false Error occurred
 */
bool pcap_spi_tx_rx(uint32_t *PCAP_spi_address, uint8_t MSG_LEN, uint8_t *tx_data)
{
	bool PCAP_spi_chk = spi_master_tx_rx(PCAP_spi_address, MSG_LEN, (const uint8_t *)tx_data, (uint8_t *)rx_data);
  
  if (PCAP_spi_chk)
    return true;
	else 
		return false;
}

/**** PCAP config register SPI write function ***
* @param PCAP_spi_address:  SPI address set by the SPI Set function 
* @param regdata: Array of 24 bit data containing the register values in sequential order from (0-9)
* @retval false for unsucessful set
*/
	
bool pcap_config_write(uint32_t *PCAP_spi_address, uint32_t *regdata) // why does the pointer work here
	{
		// intialisation of counter and starting address. 
		uint8_t n = 0;
		uint8_t regadd = 0;
	  uint8_t x, y;
		uint32_t p;
		bool b;
		
		MSG_LEN = 48;
		// Address write loop
		for (x = 0; x < 11; x++)
			{
				regadd = x;		
				
				// Value to be written per address determined by P
				p = 0x0C;      // Add write code
				p = (p << 4)|regadd;   // add Address 
				p = (p << 24)|regdata[x];  // add data
				
				// segmentation of into 8 bit variables and add to data packet
				for (y = n; y < (n+4); y++)
					{
						tx_data[y] = (p >> ((3+n)-y)*CHAR_BIT)&(0xFF) ; // matthew : Check logic 
					}
				n = n + 4; // next segmentation incremetn
			}
			
			//run bit confiuration i.e register 20
			p = 0x03; // Add write code
			p = (p << 6)|20; // add Address 
			p = (p << 24)|(1); // add data
			for (y = n; y < (n+4); y++)
				{
					tx_data[y] = (p >> ((3+n)-y)*CHAR_BIT) & (0xFF) ;
				}
		
		nrf_delay_ms(DELAY_MS);	
		b = pcap_spi_tx_rx(PCAP_spi_address, MSG_LEN, tx_data); // Final transmisson of data.
		return b;
	}
	
/**** PCAP config register set function ***
	* @PCAP_spi_address:  SPI address set by the SPI Set function 
  * Sets the indivual configuration registers and send it to the PCAP_config_write function for SPI write.
	* Return false for unsucessful set
*/
bool config_reg_set(uint32_t *PCAP_spi_address) 
	{ 
		uint32_t config_reg_d[11];
		uint8_t DSP_PRESET, PG_PRESET;
		bool w; 
		
		/* register 0 */
		config_reg_d[0] = pack((0x04), (MEMCOMP << 2)|2, 4, ECC_MODE, 8, AUTOBOOT_DIS, 4, MEM_LOCK_DIS,4);
				
		/* register 1 */
		config_reg_d[1] = 0x201022;
		
		/* register 2 */
		config_reg_d[2] = pack(CMEAS_PORT_EN, CMEAS_BITS, 4, RDCHG_INT_SEL, 4, 0x0B, 8, 0, 0);
		
		/* register 3 */
		config_reg_d[3] = pack(CY_CLK_SEL, SEQ_TIME, 6, CMEAS_FAKE, 3, C_AVRG, 13 , 0, 0);
		
		/* register 4 */
		config_reg_d[4] = pack(CMEAS_STARTPIN, CMEAS_TRIG_SEL, 2, CMEAS_CYTIME, 10, TMEAS_CYTIME, 4, ((TMEAS_STARTPIN << 2)|TMEAS_TRIG_SEL), 4);
		
		/* register 5 */
		config_reg_d[5] = pack(T_AVRG, TMEAS_TRIG_PREDIV, 22, 0, 0, 0, 0, 0 ,0);
		//config_reg_d[5] = 0x000000
			
		/* register 6 */
		config_reg_d[6] = pack(0, TMEAS_FAKE, 1, TMEAS_7BITS, 7, 0x40, 8, 0, 0);
		//config_reg_d[6] = 0x000040 ;
		
		/* register 7 */
		config_reg_d[7] = 0x1F0000 ;
		
		/* register 8 */
		DSP_PRESET = pack(DSP_SRAM_SEL, DSP_START, 1, DSP_STARTONOVL, 1, DSP_STARTONTEMP, 1, DSP_STARTPIN, 4);
		PG_PRESET = pack(INT2PG2, PG1_X_G3, 1, PG0_X_G2, 1, 0, 0, 0, 0);
		config_reg_d[8] = pack(DSP_PRESET, DSP_FF_IN, 4, (DSP_WATCHDOG_LENGTH << 2)|DSP_MOFLO_EN, 4, DSP_SPEED, 4, PG_PRESET, 4);
		//config_reg_d[8] = 0x800030 ;
		
		/* register 9 */
		config_reg_d[9] = pack(PG_DIR_IN, PG_PULL_UP, 4, PI_EN, 4, (PI1_CLK_SEL << 4)| PI0_CLK_SEL, 8, (PI1_RES << 2) | PI0_RES, 4); 
		//config_reg_d[9] = 0xFF000F ; 
		
		/* register 10 */
		config_reg_d[10] = pack(0x18, 00, 8, V_CORE_CTL, 8,0,0,0,0);
		//config_reg_d[10] = 0x180087;
		
		w = pcap_config_write(PCAP_spi_address, config_reg_d);
		return w;
	}

	
/**** PCAP read register set function ***
	* Speifies the register to be read and send the command to the PCAP sensor
	* @param PCAP_spi_address:  SPI address set by the SPI Set function 
  * @param read register address see definition list fore register address
	* @retval Return false for unsucessful set or read
	* @retval Return register data for succesful set
*/

uint32_t read_reg(uint32_t *PCAP_spi_address, uint8_t wr_reg_add) 
	{ 
		bool meas;
		uint32_t rx_reg_data;
		uint8_t p;
		
		MSG_LEN = 4; //Length of message to be transmitted
	
		// Read Register address 
		p = 0x04;      // Read code
		p = (p << 4)|wr_reg_add;   // Address 
		tx_data[0] = p; 
		
		nrf_delay_ms(DELAY_MS);
		meas = pcap_spi_tx_rx(PCAP_spi_address, MSG_LEN, tx_data); //Intiate the read of capacitance data
	  
		//rx_reg_data = pack(rx_data[0], rx_data[1], 8, rx_data[2], 8, rx_data[3], 8, 0, 0 ); //Combine recieved data check for bit order
		
		rx_reg_data = pack(rx_data[0], rx_data[1], 8, rx_data[2], 8, 0, 0, 0, 0 );
		
		//rx_reg_data = pack(rx_data[2], rx_data[1], 8, rx_data[0], 8, 0, 0, 0, 0 );
		
		//Return capacitance data
		if (meas == 0)
			return meas;
		else
			return rx_reg_data;
	}

/**** PCAP data extraction function ***
	* Extract the recieved read register data and converts it to decimal nature
	* @param data: data obtained from the read register 
	* @retval ext capacitance data for succesful set
*/
	
float data_extract(uint32_t data)
{
	int p = (data >> 21) & 7;
	uint32_t d = data & 0x1FFFFF;
	float ext = (float) p + (float)(d/1000000);
	return ext;
}
		
	
	//old config write function
	/*bool pcap_config_write(uint32_t PCAP_spi_address, uint24_t regdata, uint_8t regadd);
	{
		uint32_t  p = regadd;
		//suggestion move by size of 
		p = (p << sizeof(regdata))| b ;
		for (uint8_t i = 0; i < 4; i++)
			{
				tx_data[i] = (p << (3-i)*CHAR_BIT) & (0xFF) 
			}
		pcap_spi_tx_rx(PCAP_spi_address, tx_data)
	}*/
/*


  // Fill tx_data with some simple pattern, rx is filled with zero's so that after receiving from
  // slave we verify rx_Data is same as tx_data
  for(uint8_t i = 0; i < TX_RX_MSG_LENGTH; i++)
  {
    tx_data[i] = i;
    rx_data[i] = 0;
  }

  // Transmit TX_RX_MSG_LENGTH bytes from tx_data and receive same number of bytes and data into rx_data
  if(!spi_master_tx_rx(spi_base_address, TX_RX_MSG_LENGTH, (const uint8_t *)tx_data, rx_data) )
    return false;

  // Validate that we got all transmitted bytes back in the exact order
  for(uint8_t i = 0; i < TX_RX_MSG_LENGTH; i++)
  {
    if( tx_data[i] != rx_data[i] )
      return false;
  }
  return true;
}
*/

/**
 * main() function
 * @return 0. int return type required by ANSI/ISO standard. 
 */
int main(void)
{
  // Variable declarations 
	bool ret0, ret1, ret2, ret3; // error checkers
	uint32_t sw = 1, sw2 = 1, stat; 
	uint8_t CYC_ACTIVE, T_END_FLAG, RUNBIT, COMBI_ERR, CAP_ERR, CAP_ERR_PC, TEMP_ERR; // Status register error checkers
	uint32_t  capref_t, cap1_t;
	
	//uint32_t* PCAP_spi_address;
	char cap1_s[16], capref_s[16]; // String displays for LCD
	float cap1, capref;
		
	// Clearing Error pins
	NRF_GPIO->DIRSET = (1UL << ERROR_PIN_SPI0);
	NRF_GPIO->DIRSET = (1UL << ERROR_PIN_SPI1);
	nrf_gpio_range_cfg_output(LED_START, LED_STOP);
	// Intialise and check and LCD initalisation
	if(!nrf6350_lcd_init())
	{
		// LED0 for LCD intialisation error
		NRF_GPIO->OUTSET = (1UL << ERROR_PIN_SPI0);
	}
	nrf6350_lcd_clear();
	nrf6350_lcd_write_string("press but0", 16, LCD_UPPER_LINE, 0);
	
	/* Main Loop */
	while(true)
	{
		
		sw = 1;
		sw2 = 1;
		/* Configuring Button 0 as input */
		nrf_gpio_cfg_input(BUTTON0, NRF_GPIO_PIN_NOPULL);
		sw = nrf_gpio_pin_read(BUTTON0); // 0 indicates button pressed & 1 indicates button releas
		
		/* Configuring Button 1 as input */
		nrf_gpio_cfg_input(BUTTON1, NRF_GPIO_PIN_NOPULL);
		sw2 = nrf_gpio_pin_read(BUTTON1);
		
		/*Setting Button 1 as LCD Clear button.*/
		if(sw2 == 0)
		{
			nrf6350_lcd_clear();
		}
		
		
		while(sw == 0)
		{
			
			/* SPI Intialisation */
			uint32_t *PCAP_spi_address = pcap_spi_set(SPI1);
			//sprintf(cap1_s, "%d ", PCAP_spi_address);
			//nrf6350_lcd_write_string(cap1_s, 16, LCD_UPPER_LINE, 0);
			if (PCAP_spi_address == 0)
			{
				// Error LED = 1
				NRF_GPIO->OUTSET = (1UL << ERROR_PIN_SPI1);
				
			}
			
			/* Set configuration registers */
			memset(tx_data, 0, 48);
			memset(rx_data, 0, 48);
			ret0 = config_reg_set(PCAP_spi_address);
			if (ret0 == 0)
			{
				//Error LED = 2 for config check
				NRF_GPIO->OUTSET = (1UL << 10UL);
			}
			
			/* Send a partial reset */
			MSG_LEN = 4;
			memset(tx_data, 0, 48);
			memset(rx_data, 0, 48);
			tx_data[0] = 0x8A; // Partial Reset 
			
			nrf_delay_ms(DELAY_MS);
			ret1 = pcap_spi_tx_rx(PCAP_spi_address, MSG_LEN, tx_data);
			if (ret1 == 0)
			{
				//Error LED = 3 for Reset Check
				NRF_GPIO->OUTSET = (1UL << 11UL);
			}
			
			/* Start Capacitance Measurement */
			MSG_LEN = 4;
			memset(tx_data, 0, 48);
			memset(rx_data, 0, 48);
			tx_data[0] = 0x8C; // Start Command 
			
			nrf_delay_ms(DELAY_MS);
			ret2 = pcap_spi_tx_rx(PCAP_spi_address, MSG_LEN, tx_data);
			if (ret2 == 0)
			{
				//Error LED = 4 for Measurement Start
				NRF_GPIO->OUTSET = (1UL << 12UL);
			}
			
			// Measurement Delay 
			nrf_delay_ms(1000);
			memset(tx_data, 0, 48);
			memset(rx_data, 0, 48);
			/* Read Status register: */
			stat = read_reg(PCAP_spi_address, read_stat);
			sprintf(cap1_s, "%d ", stat);
			nrf6350_lcd_write_string(cap1_s, 16, LCD_LOWER_LINE, 0);
			
			/* Read CO & C1/C0 values: */
			memset(tx_data, 0, 48);
			memset(rx_data, 0, 48);
			capref_t = read_reg(PCAP_spi_address, read_reg0);
			
			memset(tx_data, 0, 48);
			memset(rx_data, 0, 48);
			cap1_t = read_reg(PCAP_spi_address, read_reg1);
			
			ret3 = (stat == 0 && capref_t == 0 && cap1_t == 0); 
			if (stat == 0) // if (ret3 == 0)
			{
				// Error LED = 5 for Status & Measurement Check 
				NRF_GPIO->OUTSET = (1UL << 13UL);
			}
			
			if (cap1_t == 0)
			{
				// Error LED = 6 for Measurement Check
				NRF_GPIO->OUTSET = (1UL << 14UL);
			}
			
			/* Complete error checker */
			if (!ret0 && !ret1 && !ret2 && !ret3 && !PCAP_spi_address)
			{
				while(true)
				{
						// Loop forever
					nrf6350_lcd_write_string("Error", 16, LCD_UPPER_LINE, 0);
				}
			}
			
// 			if (!ret0 || !ret1 || !ret2 || !ret3 || !PCAP_spi_address)
// 			{
// 				while(true)
// 				{
// 						// Loop forever
// 					nrf6350_lcd_write_string("Error", 16, LCD_UPPER_LINE, 0);
// 				}
// 			}
			
			/* Status bits */ 
			//configure later for further analysis 
			CYC_ACTIVE = (stat >> 23) & 1;
			T_END_FLAG = (stat >> 22) & 1;
			RUNBIT = (stat >> 19) & 1;
			COMBI_ERR = (stat >> 15) & 1;
			CAP_ERR = (stat >> 12) & 1;
			CAP_ERR_PC = (stat >> 5) & 0x0F;
			TEMP_ERR = (stat >> 3) & 1;
			
			/*Data extraction:*/
			cap1 = data_extract(cap1_t)*47;
			//capref = data_extract(capref_t)*47 ;
			capref = 5;
			
			/*Data display on LCD */
			sprintf(cap1_s, "Cap1 = %f ", cap1);
			sprintf(capref_s, "Cref = %f", capref);
			//nrf6350_lcd_write_string(cap1_s, 16, LCD_UPPER_LINE, 0);
			//nrf6350_lcd_write_string(capref_s, 16, LCD_LOWER_LINE, 0);
			sw = nrf_gpio_pin_read(BUTTON0);
		}

	}	
}

/**
 Change Log 
 28/2/2013
 Included the start measurement command
 3/1/2013
 Changed the data extract function to extract the right bits.
 Changed the no of the recieved bit in the read_reg funtion  (i.e from 32 to 24) 
 Changed the decimal place division. 
 3/2/2013
 Fixed the tx_data function in the config reg
 Cleared tx_data using memset.
 3/3/2013
 Cleared rx_data using memset.
 Delay Function before transmssion.
 MSG_LEN is common for both transmit and recieve , the message lengths have been adjusted accordingly
 Debug portion has been commented out.
 **/
