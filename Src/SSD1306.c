/*
 * SSD1306.c
 *
 *  Created on: 2019. 9. 12.
 *      Author: rs
 */

#include<stdint.h>

#include "SSD1306.h"
#include "fonts.h"

void ssd1306_W_Command(uint8_t cmd)
{
	uint8_t buffer[2]={0};		//Control Byte + Command Byte
	buffer[0]=(0<<7)|(0<<6);	//Co=0 , D/C=0
	buffer[1]=cmd;

	if(HAL_I2C_Master_Transmit_DMA(&ssd1306_I2C_PORT,(uint16_t)(ssd1306_Address)<<1,(uint8_t*)buffer,2)!= HAL_OK)
	{
		Error_Handler();
	}
	while (HAL_I2C_GetState(&ssd1306_I2C_PORT) != HAL_I2C_STATE_READY)
	{
	}
}

void ssd1306_W_Data(uint8_t* data_buffer, uint16_t buffer_size)
{
	if(HAL_I2C_Mem_Write_DMA(&ssd1306_I2C_PORT,(uint16_t)(ssd1306_Address<<1),0x40,1,data_buffer,buffer_size)!= HAL_OK)
	{
		Error_Handler();
	}
	while(HAL_I2C_GetState(&ssd1306_I2C_PORT) != HAL_I2C_STATE_READY)
	{
	}
}

void ssd1306_Init(void)
{
	ssd1306_W_Command(0xA8);	//Set Mux Ratio
	ssd1306_W_Command(0x3F);	//64MUX

	ssd1306_W_Command(0xD3);	//Set Display Offset
	ssd1306_W_Command(0x00);	//COM0

	ssd1306_W_Command(0x40);	//Set Display Start Line

	ssd1306_W_Command(0xA1);	//Set Segment re-map, Default 0xA0
								//column address 127 is mapped to SEG0 (좌우 반전)

	ssd1306_W_Command(0xC8);	//Set COM Output Scan Direction, default 0xC0
								//remapped mode. Scan from COM[N-1] to COM0 (상하 반전)

	ssd1306_W_Command(0xDA);	//Set COM Pins hardware configuration
	ssd1306_W_Command(0x12);

	ssd1306_W_Command(0x20);	//Set Memory Addressing Mode
	ssd1306_W_Command(0x02);	//Page Addressing Mode

	ssd1306_W_Command(0x81);	//Set Contrast Control
	ssd1306_W_Command(0x7F);	//1~256

	ssd1306_W_Command(0xA4);	//Disable Entire Display On

	ssd1306_W_Command(0xA6);	//Set Normal Display

	ssd1306_W_Command(0xD5);	//Set Osc Frequency
	ssd1306_W_Command(0x80);

	ssd1306_W_Command(0x8D);	//Enable charge pump regulator
	ssd1306_W_Command(0x14);

	ssd1306_W_Command(0xAF);	//Display ON
}


void ssd1306_Clear(void)
{
	uint8_t buffer[128]={0};

	ssd1306_W_Command(0x00);
	ssd1306_W_Command(0x10);

	for(uint8_t i=0;i<8;i++)
	{
		ssd1306_W_Command(0xB0+i);
		ssd1306_W_Data(buffer,128);
	}
}


void ssd1306_Fill_Screen(uint8_t data)
{
	uint8_t buffer[128]={0};

	if(data!=0)
	{
		for(uint8_t i=0;i<128;i++)
		{
			buffer[i]=data;
		}
	}

	ssd1306_W_Command(0x00);
	ssd1306_W_Command(0x10);

	for(uint8_t i=0;i<8;i++)
	{
		ssd1306_W_Command(0xB0+i);
		ssd1306_W_Data(buffer,128);
	}
}

void ssd1306_Set_Coord(uint8_t page, uint8_t col)
{
	uint8_t col_low=0x0F,col_high=0x1F;
	col_low=(col&0x0F);
	col_high=0x10|((col>>4)&0x0F);
	ssd1306_W_Command(0xB0+page);
	ssd1306_W_Command(col_low);
	ssd1306_W_Command(col_high);
}

void ssd1306_W_Char(uint8_t character_Code, uint8_t page, uint16_t column)
{
	uint8_t char_Buffer[font_width*2]={0};

	for(uint8_t i=0;i<font_width*2;i++)
	{
		char_Buffer[i]=ssd1306_Fonts[(character_Code-32)*(font_width*2)+i];
	}

	for(uint8_t i=0;i<2;i++)
	{
		ssd1306_Set_Coord(page+i,column);
		ssd1306_W_Data(&char_Buffer[i*font_width],font_width);
	}
}

void ssd1306_W_String(char *str, uint8_t page, uint8_t col)
{
	while(*str)
	{
		if((127<col+font_width))
		{
			if(page==6)
			{
				break;
			}
			page+=2;
			col=0;
		}
		ssd1306_W_Char(*str,page,col);

		col+=font_width;
		str++;
	}
}
