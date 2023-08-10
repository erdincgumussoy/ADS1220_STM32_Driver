/*
 * ADS1220.c
 *
 *  Created on: May 21, 2023
 *      Author: erdin
 */
#include "ADS1220.h"
//#include "stm32f4xx_hal.h"

uint8_t config_reg[4];
ADS1220_handle_config_t default_config;



/*******************************************************************************/
void mux_config(input_mux_select_t data)
{
	if(data!=ADS1220_MUX_AIN0_AIN1){
		config_reg[0]&=0x0F;							//clear old data
		config_reg[0]|=data<<4;							//set new data
	}
	else{
		config_reg[0]&=0x0F;							//clear old data
		config_reg[0]|=ADS1220_MUX_AIN0_AIN1<<4;		//set default
	}
}

void gain_config(pga_gain_select_t data)
{
	if(data!=ADS1220_PGA_GAIN_1){
		config_reg[0]&=0xF1;							//clear old data
		config_reg[0]|=data<<1;							//set new data
	}
	else{
		config_reg[0]&=0xF1;							//clear old data
		config_reg[0]|=ADS1220_PGA_GAIN_1<<1;			//set default
	}
}

void pga_bypass_config(pga_bypass_config_t data)
{
	if(data!=ADS1220_PGA_ENABLE){
		config_reg[0]&=0xFE;							//clear old data
		config_reg[0]|=data;							//set new data
	}
	else{
		config_reg[0]&=0xFE;							//clear old data
		config_reg[0]|=ADS1220_PGA_ENABLE;				//set default
	}
}

/*******************************************************************************/

void data_rate_config(sample_rate_select_t data)
{
	if(data!=ADS1220_DATA_RATE_20SPS){
		config_reg[1]&=0x1F;							//clear old data
		config_reg[1]|=data<<5;							//set new data
	}
	else{
		config_reg[1]&=0x1F;							//clear old data
		config_reg[1]|=ADS1220_DATA_RATE_20SPS<<5;		//set default
	}
}

void operation_mode_config(operation_mode_select_t data)
{
	if(data!=ADS1220_NORMAL_MODE){
		config_reg[1]&=0xE7;							//clear old data
		config_reg[1]|=data<<3;							//set new data
	}
	else{
		config_reg[1]&=0xE7;							//clear old data
		config_reg[1]|=ADS1220_NORMAL_MODE<<3;			//set default
	}
}

void conversion_mode_config(conversion_mode_select_t data)
{
	if(data!=ADS1220_SNGL_CONV_MODE){
		config_reg[1]&=0xFB;							//clear old data
		config_reg[1]|=data<<2;							//set new data
	}
	else{
		config_reg[1]&=0xFB;							//clear old data
		config_reg[1]|=ADS1220_SNGL_CONV_MODE<<2;		//set default
	}
}

void tempereture_mode_config(tempereture_mode_activate_t data)
{
	if(data!=ADS1220_TEMP_MEAS_DSBL){
		config_reg[1]&=0xFD;							//clear old data
		config_reg[1]|=data<<1;							//set new data
	}
	else{
		config_reg[1]&=0xFD;							//clear old data
		config_reg[1]|=ADS1220_TEMP_MEAS_DSBL<<1;		//set default
	}
}

void current_source_config(current_source_activate_t data)
{
	if(data!=ADS1220_BCS_DSB){
		config_reg[1]&=0xFE;							//clear old data
		config_reg[1]|=data;							//set new data
	}
	else{
		config_reg[1]&=0xFE;							//clear old data
		config_reg[1]|=ADS1220_BCS_DSB;					//set default
	}
}

/*******************************************************************************/

void ref_voltage_config(vref_select_t data)
{
	if(data!=ADS1220_VREF_INTRNAL_2_048){
		config_reg[2]&=0x3F;								//clear old data
		config_reg[2]|=data<<6;								//set new data
	}
	else{
		config_reg[2]&=0x3F;								//clear old data
		config_reg[2]|=ADS1220_VREF_INTRNAL_2_048<<6;		//set default
	}
}

void fir_filter_config(fir_filter_select_t data)
{
	if(data!=ADS1220_FIR_FILTER_DISABLE){
		config_reg[2]&=0xCF;								//clear old data
		config_reg[2]|=data<<4;								//set new data
	}
	else{
		config_reg[2]&=0xCF;								//clear old data
		config_reg[2]|=ADS1220_FIR_FILTER_DISABLE<<4;		//set default
	}
}

void power_switch_config(pwr_switch_config_t data)
{
	if(data!=ADS1220_PWR_SWITCH_ON){
		config_reg[2]&=0xF7;								//clear old data
		config_reg[2]|=data<<3;								//set new data
	}
	else{
		config_reg[2]&=0xF7;								//clear old data
		config_reg[2]|=ADS1220_PWR_SWITCH_ON<<3;			//set default
	}
}

void idac_current_config(idac_current_config_t data)
{
	if(data!=ADS1220_IDAC_DISABLE){
		config_reg[2]&=0xF8;								//clear old data
		config_reg[2]|=data;								//set new data
	}
	else{
		config_reg[2]&=0xF8;								//clear old data
		config_reg[2]|=ADS1220_IDAC_DISABLE;			//set default
	}
}

/*******************************************************************************/

void idac1_mux_config(idac1_mux_config_t data)
{
	if(data!=ADS1220_IDAC1_DISABLE){
		config_reg[3]&=0x1F;								//clear old data
		config_reg[3]|=data<<5;								//set new data
	}
	else{
		config_reg[3]&=0x1F;								//clear old data
		config_reg[3]|=ADS1220_IDAC1_DISABLE<<5;			//set default
	}
}

void idac2_mux_config(idac2_mux_config_t data)
{
	if(data!=ADS1220_IDAC2_DISABLE){
		config_reg[3]&=0xE3;								//clear old data
		config_reg[3]|=data<<1;								//set new data
	}
	else{
		config_reg[3]&=0xE3;								//clear old data
		config_reg[3]|=ADS1220_IDAC2_DISABLE<<2;			//set default
	}
}

void drdym_config(drdyn_config_t data)
{
	if(data!=ADS1220_DRDYn_READY){
		config_reg[3]&=0xFD;								//clear old data
		config_reg[3]|=data<<1;								//set new data
	}
	else{
		config_reg[3]&=0xFD;								//clear old data
		config_reg[3]|=ADS1220_DRDYn_READY<<1;				//set default
	}
}

/*******************************************************************************/

HAL_StatusTypeDef ADS1220_reset( SPI_HandleTypeDef *hspi )
{
	uint8_t data = ADS1220_RESET;
	HAL_StatusTypeDef ret;
	ret = HAL_SPI_Transmit( hspi, &data, 1, HAL_MAX_DELAY );
	HAL_Delay(1);
	return ret;
}

HAL_StatusTypeDef ADS1220_power_down( SPI_HandleTypeDef *hspi )
{
	uint8_t data = ADS1220_PWR_DWN;
	HAL_StatusTypeDef ret;
	ret = HAL_SPI_Transmit(hspi, &data, 1, HAL_MAX_DELAY);
	HAL_Delay(1);
	return ret;
}

HAL_StatusTypeDef ADS1220_start_conversion( SPI_HandleTypeDef *hspi )
{
	uint8_t data = ADS1220_STRT_SYNC;
	HAL_StatusTypeDef ret;
	ret = HAL_SPI_Transmit(hspi, &data, 1, HAL_MAX_DELAY);
	HAL_Delay(1);
	return ret;
}

uint32_t ADS1220_default_config(void)
{
	default_config.input_mux	=	ADS1220_MUX_AIN0_AIN1;
	default_config.pga_gain		=	ADS1220_PGA_GAIN_1;
	default_config.pga_actvt	=	ADS1220_PGA_ENABLE;
	default_config.sample_rate	=	ADS1220_DATA_RATE_20SPS;
	default_config.oprtn_mode	=	ADS1220_NORMAL_MODE;
	default_config.convrtn_mode	=	ADS1220_SNGL_CONV_MODE;
	default_config.temprtr_mode	=	ADS1220_TEMP_MEAS_DSBL;
	default_config.curr_src_act	=	ADS1220_BCS_DSB;
	default_config.ref_volt		=	ADS1220_VREF_INTRNAL_2_048;
	default_config.fir_filter	=	ADS1220_FIR_FILTER_DISABLE;
	default_config.pwr_swtch	=	ADS1220_PWR_SWITCH_ON;
	default_config.idac_curr	=	ADS1220_IDAC_DISABLE;
	default_config.idac1_mux	=	ADS1220_IDAC1_DISABLE;
	default_config.idac2_mux	=	ADS1220_IDAC2_DISABLE;
	default_config.data_rdy		=	ADS1220_DRDYn_READY;

	return ADS1220_config(&default_config);
}

uint32_t ADS1220_config(ADS1220_handle_config_t *config_data)
{
	mux_config(config_data->input_mux);
	gain_config(config_data->pga_gain);
	pga_bypass_config(config_data->pga_actvt);
	data_rate_config(config_data->sample_rate);
	operation_mode_config(config_data->oprtn_mode);
	conversion_mode_config(config_data->convrtn_mode);
	tempereture_mode_config(config_data->temprtr_mode);
	current_source_config(config_data->curr_src_act);
	ref_voltage_config(config_data->ref_volt);
	fir_filter_config(config_data->fir_filter);
	power_switch_config(config_data->pwr_swtch);
	idac_current_config(config_data->idac_curr);
	idac1_mux_config(config_data->idac1_mux);
	idac2_mux_config(config_data->idac2_mux);
	drdym_config(config_data->data_rdy);

	return ( (config_reg[3]<<24)|(config_reg[2]<<16)|(config_reg[1]<<8)|config_reg[0] );
}

uint8_t ADS1220_read_register(SPI_HandleTypeDef *hspi, uint8_t address)
{
	uint8_t rx_data[2]={0};
	uint8_t tx_data[2]={ADS1220_R_REG|(address<<2), 0x00};

	//uint8_t rx_data[4]={0};
	//uint8_t tx_data[4]={0x00,0x00,0x00, 0x03};

	HAL_SPI_Transmit(hspi, tx_data, 1, HAL_MAX_DELAY);
	HAL_SPI_Receive(hspi, rx_data, 1, HAL_MAX_DELAY);
	//HAL_SPI_TransmitReceive(hspi, &tx_data[0], &rx_data[0], 1, HAL_MAX_DELAY);
	return rx_data[0];
}

void ADS1220_write_register(SPI_HandleTypeDef *hspi, uint8_t address, uint8_t value)
{
	uint8_t tx_data[2]={0};
	tx_data[1]=value;
	tx_data[0]=ADS1220_W_REG|(address<<2);
	HAL_SPI_Transmit(hspi, tx_data, sizeof(tx_data), HAL_MAX_DELAY);
}

HAL_StatusTypeDef ADS1220_read_data(SPI_HandleTypeDef *hspi, uint8_t data[])
{
	uint8_t reg_data = ADS1220_R_DATA;
	HAL_StatusTypeDef ret;

	HAL_SPI_Transmit(hspi, &reg_data, 1, HAL_MAX_DELAY);
	ret = HAL_SPI_Receive(hspi, data, 3, HAL_MAX_DELAY);
	//ret = HAL_SPI_TransmitReceive(hspi, dummy, data, 3, HAL_MAX_DELAY);
	//ret = HAL_SPI_Receive(hspi, data, 3, HAL_MAX_DELAY);
	HAL_Delay(1);
	return ret;
}
