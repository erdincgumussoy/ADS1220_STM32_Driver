/*
 * ADS1220.h
 *
 *  Created on: July 21, 2023
 *      Author: erdincgmssy
 */

#ifndef INC_ADS1220_H_
#define INC_ADS1220_H_

#ifdef __cplusplus
extern "C" {
#endif

/***********************************Includes***********************************/
//#include "main.h"
#include "stm32f4xx_hal.h"

/*Command Table*/
#define ADS1220_PWR_DWN			0x02
#define ADS1220_RESET			0x06
#define ADS1220_STRT_SYNC		0x08
#define ADS1220_R_DATA			0x10
#define ADS1220_R_REG			0x20
#define ADS1220_W_REG			0x40

/*******************************************************************************/


/*Configuration Registers*/


/*Configuration Register Addresses*/
/*******************************************************************************/

#define CONFIG_REG0_ADRESS 0x00
#define CONFIG_REG1_ADRESS 0x01
#define CONFIG_REG2_ADRESS 0x02
#define CONFIG_REG3_ADRESS 0x03
/*******************************************************************************/

/*Configuration Register 0 Field*/
/*******************************************************************************/
/*Input multiplexer configuration*/

/*These bits configure the input multiplexer
 *For settings where AINN = AVSS, the PGA
 *must be disabled (PGA_BYPASS = 1)and only
 *gains 1, 2, and 4 can be used.
 */
typedef enum
{
//		  AIN_P - AIN_N
ADS1220_MUX_AIN0_AIN1,		//AINP = AIN0, AINN = AIN1 (default)
ADS1220_MUX_AIN0_AIN2,		//AINP = AIN0, AINN = AIN2
ADS1220_MUX_AIN0_AIN3,		//AINP = AIN0, AINN = AIN3
ADS1220_MUX_AIN1_AIN2,		//AINP = AIN1, AINN = AIN2
ADS1220_MUX_AIN1_AIN3,		//AINP = AIN1, AINN = AIN3
ADS1220_MUX_AIN2_AIN3,		//AINP = AIN2, AINN = AIN3
ADS1220_MUX_AIN1_AIN0,		//AINP = AIN1, AINN = AIN0
ADS1220_MUX_AIN3_AIN2,		//AINP = AIN3, AINN = AIN2
ADS1220_MUX_AIN0_AVSS,		//AINP = AIN0, AINN = AVSS
ADS1220_MUX_AIN1_AVSS,		//AINP = AIN1, AINN = AVSS
ADS1220_MUX_AIN2_AVSS,		//AINP = AIN2, AINN = AVSS
ADS1220_MUX_AIN3_AVSS,		//AINP = AIN3, AINN = AVSS
ADS1220_MUX_VRFP_VRFN,		//(V(REFPx) – V(REFNx)) / 4 monitor (PGA bypassed)
ADS1220_MUX_AVDD_AVSS,		//(AVDD – AVSS) / 4 monitor (PGA bypassed)
ADS1220_MUX_SPLY_DIV2,		//AINP and AINN shorted to (AVDD + AVSS) / 2
} input_mux_select_t;
/*******************************************************************************/
/*Gain configuration*/

/*These bits configure the device gain. Gains 1, 2, and 4 can be used without the PGA.
 * In this case, gain is obtained by a switched-capacitor structure.
 */
typedef enum
{
ADS1220_PGA_GAIN_1,			//Gain = 1 (default)
ADS1220_PGA_GAIN_2,			//Gain = 2
ADS1220_PGA_GAIN_4,			//Gain = 4
ADS1220_PGA_GAIN_8,			//Gain = 8
ADS1220_PGA_GAIN_16,		//Gain = 16
ADS1220_PGA_GAIN_32,		//Gain = 32
ADS1220_PGA_GAIN_64,		//Gain = 64
ADS1220_PGA_GAIN_128		//Gain = 128
} pga_gain_select_t;
/*******************************************************************************/
/*Disables and bypasses the internal low-noise PGA*/

/*Disabling the PGA reduces overall power consumption and allows the common-mode voltage
 *range (VCM) to span from AVSS – 0.1 V to AVDD + 0.1 V. The PGA can only be disabled for
 *gains 1, 2, and 4. The PGA is always enabled for gain settings 8 to 128, regardless of the
 *PGA_BYPASS setting.
*/
typedef enum
{
ADS1220_PGA_ENABLE,			//PGA enabled (default)
ADS1220_PGA_DISBLE			//PGA disabled and bypassed
}pga_bypass_config_t;
/*******************************************************************************/

/*Configuration Register 1 Field*/
/*******************************************************************************/
/*Data rate*/

/*These bits control the data rate setting depending on the selected operating mode.
*/
typedef enum
{
ADS1220_DATA_RATE_20SPS,	//NORMAL MODE=20SPS, DUTY-CYCLE MODE=5SPS, TURBO MODE=40
ADS1220_DATA_RATE_45SPS,	//NORMAL MODE=45SPS, DUTY-CYCLE MODE=11.25SPS, TURBO MODE=90
ADS1220_DATA_RATE_90SPS,	//NORMAL MODE=90SPS, DUTY-CYCLE MODE=22.5SPS, TURBO MODE=180
ADS1220_DATA_RATE_175SPS,	//NORMAL MODE=175SPS, DUTY-CYCLE MODE=44SPS, TURBO MODE=350
ADS1220_DATA_RATE_330SPS,	//NORMAL MODE=330SPS, DUTY-CYCLE MODE=82.5SPS, TURBO MODE=660
ADS1220_DATA_RATE_600SPS,	//NORMAL MODE=600SPS, DUTY-CYCLE MODE=150SPS, TURBO MODE=1200
ADS1220_DATA_RATE_1000SPS	//NORMAL MODE=1000SPS, DUTY-CYCLE MODE=250SPS, TURBO MODE=2000
} sample_rate_select_t;
/*******************************************************************************/
/*Operating mode*/

/*These bits control the operating mode the device operates in.
*/
typedef enum
{
ADS1220_NORMAL_MODE,		//00 : Normal mode (256-kHz modulator clock, default
ADS1220_DUTY_CYCLE_MODE,	//01 : Duty-cycle mode (internal duty cycle of 1:4
ADS1220_TURBO_MODE			//10 : Turbo mode (512-kHz modulator clock)
} operation_mode_select_t;
/*******************************************************************************/
/*Conversion mode*/

/*This bit sets the conversion mode for the device.
*/
typedef enum
{
ADS1220_SNGL_CONV_MODE,		//0 : Single-shot mode (default)
ADS1220_CONT_CONV_MODE		//1 : Continuous conversion mode
} conversion_mode_select_t;
/*******************************************************************************/
/*Temperature sensor mode*/

/*This bit enables the internal temperature sensor and puts the device in
 * temperature sensor mode. The settings of configuration register 0 have
 * no effect and the device uses the internal reference for measurement
 * when temperature sensor mode is enabled.
*/

typedef enum
{
ADS1220_TEMP_MEAS_DSBL,				//0 : Disables temperature sensor (default)
ADS1220_TEMP_MEAS_ENBL				//1 : Enables temperature sensor
} tempereture_mode_activate_t;

/*******************************************************************************/
/*Burn-out current sources*/

/*This bit controls the 10-µA, burn-out current sources.
 *The burn-out current sources can be used to detect sensor
 *faults such as wire breaks and shorted sensors.
*/

typedef enum
{
ADS1220_BCS_DSB,					//0 : Current sources off (default)
ADS1220_BCS_ENBL,					//1 : Current sources on
} current_source_activate_t;

/*******************************************************************************/

/*Configuration Register 2 Field*/
/*******************************************************************************/
/*Voltage reference selection*/

/*These bits select the voltage reference source that is
 * used for the conversion.
 */

typedef enum
{
ADS1220_VREF_INTRNAL_2_048,			//00 : Internal 2.048-V reference selected (default)
ADS1220_VREF_EXT_RFP0_RFN0,			//01 : External reference selected using dedicated REFP0 and REFN0 inputs
ADS1220_VREF_EXT_AIN0_AIN3,			//10 : External reference selected using AIN0/REFP1 and AIN3/REFN1 inputs
ADS1220_VREF_EXT_AVDD_AVSS,			//11 : Analog supply (AVDD – AVSS) used as reference
} vref_select_t;

/*******************************************************************************/
/*FIR filter configuration*/

/*These bits configure the filter coefficients for the internal
 * FIR filter. Only use these bits together with the 20-SPS
 * setting in normal mode and the 5-SPS setting in duty-cycle
 * mode. Set to 00 for all other data rates.
 */

typedef enum
{
ADS1220_FIR_FILTER_DISABLE,				//00 : No 50-Hz or 60-Hz rejection (default)
ADS1220_FIR_FILTER_50_60HZ,				//01 : Simultaneous 50-Hz and 60-Hz rejection
ADS1220_FIR_FILTER_50HZ,				//10 : 50-Hz rejection only
ADS1220_FIR_FILTER_60HZ					//11 : 60-Hz rejection only
} fir_filter_select_t;

/*******************************************************************************/
/*Low-side power switch configuration*/

/*This bit configures the behavior of the low-side switch connected
 * between AIN3/REFN1 and AVSS.
 */

typedef enum
{
ADS1220_PWR_SWITCH_OFF,					//0 : Switch is always open (default)
ADS1220_PWR_SWITCH_ON					//1 : Switch automatically closes when the START/SYNC
										//command is sent and opens when the POWERDOWN command
										//is issued
} pwr_switch_config_t;

/*******************************************************************************/
/*IDAC current setting*/

/*These bits set the current for both IDAC1 and IDAC2 excitation
 * current sources
 */

typedef enum
{
ADS1220_IDAC_DISABLE,					//000 : Off (default)
ADS1220_IDAC_10_UA,						//001 : 10 µA
ADS1220_IDAC_50_UA,						//010 : 50 µA
ADS1220_IDAC_100_UA,					//011 : 100 µA
ADS1220_IDAC_250_UA,					//100 : 250 µA
ADS1220_IDAC_500_UA,					//101 : 500 µA
ADS1220_IDAC_1000_UA,					//110 : 1000 µA
ADS1220_IDAC_1500_UA					//111 : 1500 µA
} idac_current_config_t;

/*******************************************************************************/

/*Configuration Register 3 Field*/
/*******************************************************************************/
/*IDAC1 routing configuration*/

/*These bits select the channel where IDAC1 is routed to.
 */

typedef enum
{
ADS1220_IDAC1_DISABLE,					//000 : IDAC1 disabled (default)
ADS1220_IDAC1_AIN0,						//001 : IDAC1 connected to AIN0/REFP1
ADS1220_IDAC1_AIN1,						//010 : IDAC1 connected to AIN1
ADS1220_IDAC1_AIN2,						//011 : IDAC1 connected to AIN2
ADS1220_IDAC1_AIN3,						//100 : IDAC1 connected to AIN3/REFN1
ADS1220_IDAC1_REFP0,					//101 : IDAC1 connected to REFP0
ADS1220_IDAC1_REFN0,					//110 : IDAC1 connected to REFN0
} idac1_mux_config_t;

/*******************************************************************************/
/*IDAC2 routing configuration*/

/*These bits select the channel where IDAC2 is routed to.
 */

typedef enum
{
ADS1220_IDAC2_DISABLE,					//000 : IDAC1 disabled (default)
ADS1220_IDAC2_AIN0,						//001 : IDAC1 connected to AIN0/REFP1
ADS1220_IDAC2_AIN1,						//010 : IDAC1 connected to AIN1
ADS1220_IDAC2_AIN2,						//011 : IDAC1 connected to AIN2
ADS1220_IDAC2_AIN3,						//100 : IDAC1 connected to AIN3/REFN1
ADS1220_IDAC2_REFP0,					//101 : IDAC1 connected to REFP0
ADS1220_IDAC2_REFN0,					//110 : IDAC1 connected to REFN0
} idac2_mux_config_t;

/*******************************************************************************/
/*DRDY mode*/

/*This bit controls the behavior of the DOUT/DRDY pin when new data are ready.
 */

typedef enum
{
ADS1220_DRDYn_READY,					//0 : Only the dedicated DRDY pin is used to indicate
										//when data are ready (default)
ADS1220_DOUT_READY,						//1 :  Data ready is indicated simultaneously on DOUT/DRDY and DRDY

} drdyn_config_t;

typedef struct __ADS1220_handle_config_t{
	input_mux_select_t 				input_mux;
	pga_gain_select_t 				pga_gain;
	pga_bypass_config_t				pga_actvt;
	sample_rate_select_t			sample_rate;
	operation_mode_select_t			oprtn_mode;
	conversion_mode_select_t   	 	convrtn_mode;
	tempereture_mode_activate_t		temprtr_mode;
	current_source_activate_t		curr_src_act;
	vref_select_t					ref_volt;
	fir_filter_select_t				fir_filter;
	pwr_switch_config_t				pwr_swtch;
	idac_current_config_t			idac_curr;
	idac1_mux_config_t				idac1_mux;
	idac2_mux_config_t				idac2_mux;
	drdyn_config_t					data_rdy;
	GPIO_TypeDef*					cs_port;
	uint16_t						cs_pin;
	//SPI_HandleTypeDef				*hspi;

}ADS1220_handle_config_t;

/*******************************************************************************/

void mux_config(input_mux_select_t data);
void gain_config(pga_gain_select_t data);
void pga_bypass_config(pga_bypass_config_t data);
void data_rate_config(sample_rate_select_t data);
void operation_mode_config(operation_mode_select_t data);
void conversion_mode_config(conversion_mode_select_t data);
void tempereture_mode_config(tempereture_mode_activate_t data);
void current_source_config(current_source_activate_t data);
void ref_voltage_config(vref_select_t data);
void fir_filter_config(fir_filter_select_t data);
void power_switch_config(pwr_switch_config_t data);
void idac_current_config(idac_current_config_t data);
void idac1_mux_config(idac1_mux_config_t data);
void idac2_mux_config(idac2_mux_config_t data);
void drdym_config(drdyn_config_t data);


/*******************************************************************************/

HAL_StatusTypeDef ADS1220_reset(SPI_HandleTypeDef *hspi);
HAL_StatusTypeDef ADS1220_power_down(SPI_HandleTypeDef *hspi);
HAL_StatusTypeDef ADS1220_start_conversion(SPI_HandleTypeDef *hspi);


uint32_t ADS1220_default_config(void);
uint32_t ADS1220_config(ADS1220_handle_config_t *config_data);
HAL_StatusTypeDef ADS1220_read_data(SPI_HandleTypeDef *hspi, uint8_t data[]);
uint8_t ADS1220_read_register(SPI_HandleTypeDef *hspi, uint8_t address);
void ADS1220_write_register(SPI_HandleTypeDef *hspi, uint8_t address, uint8_t value);


#ifdef __cplusplus
}
#endif

#endif /* INC_ADS1220_H_ */
