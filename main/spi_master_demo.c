/* SPI Master example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include <time.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "soc/gpio_struct.h"
#include "driver/gpio.h"
#include "spi_master.h"
#include "tftfunc.h"


/*
 This code tests accessing ILI9341 based display in various modes
 and prints some timings

 Some fancy graphics is displayed on the ILI9341-based 320x240 LCD
 In DMA/Transactions mode this example demonstrate the use of both spi_device_transmit
 as well as spi_device_queue_trans/spi_device_get_trans_result as well as pre-transmit callbacks.

 The usage of NonDMA/NoTransaction mode is also demonstrated and the speed is compared.
 In this mode, sending individual pixels is more than 10 times faster than when using DMA/Transactions!
 
 Reading the display content is demonstrated.
 
 If Touch screen is available, reading the touch coordinates (non calibrated) is also demonstrated.
 
 Some info about the ILI9341: It has an C/D line, which is connected to a GPIO here. It expects this
 line to be low for a command and high for data. We use a pre-transmit callback here to control that
 line: every transaction has as the user-definable argument the needed state of the D/C line and just
 before the transaction is sent, the callback will set this line to the correct state.
*/

// comment this if your display does not have touch screen
#define USE_TOUCH

// define which spi bus to use VSPI_HOST or HSPI_HOST
#define SPI_BUS VSPI_HOST

#define DELAY 0x80

//Transaction descriptors
static spi_transaction_t trans[6];

// Init for ILI7341
// ----------------
static const uint8_t ILI9341_init[] = {
  23,                   					        // 23 commands in list
  ILI9341_SWRESET, DELAY,   						//  1: Software reset, no args, w/delay
  200,												//     200 ms delay
  ILI9341_POWERA, 5, 0x39, 0x2C, 0x00, 0x34, 0x02,
  ILI9341_POWERB, 3, 0x00, 0XC1, 0X30,
  0xEF, 3, 0x03, 0x80, 0x02,
  ILI9341_DTCA, 3, 0x85, 0x00, 0x78,
  ILI9341_DTCB, 2, 0x00, 0x00,
  ILI9341_POWER_SEQ, 4, 0x64, 0x03, 0X12, 0X81,
  ILI9341_PRC, 1, 0x20,
  ILI9341_PWCTR1, 1,  								//Power control
  0x23,               								//VRH[5:0]
  ILI9341_PWCTR2, 1,   								//Power control
  0x10,                 							//SAP[2:0];BT[3:0]
  ILI9341_VMCTR1, 2,    							//VCM control
  0x3e,                 							//Contrast
  0x28,
  ILI9341_VMCTR2, 1,  								//VCM control2
  0x86,
  TFT_MADCTL, 1,									// Memory Access Control (orientation)
  (MADCTL_MV | MADCTL_BGR),
  ILI9341_PIXFMT, 1,
  0x55,
  ILI9341_FRMCTR1, 2,
  0x00,
  0x18,
  ILI9341_DFUNCTR, 3,   							// Display Function Control
  0x08,
  0x82,
  0x27,
  TFT_PTLAR, 4, 0x00, 0x00, 0x01, 0x3F,
  ILI9341_3GAMMA_EN, 1,								// 3Gamma Function Disable
  0x00, // 0x02
  ILI9341_GAMMASET, 1, 								//Gamma curve selected
  0x01,
  ILI9341_GMCTRP1, 15,   							//Positive Gamma Correction
  0x0F, 0x31, 0x2B, 0x0C, 0x0E, 0x08, 0x4E, 0xF1,
  0x37, 0x07, 0x10, 0x03, 0x0E, 0x09, 0x00,
  ILI9341_GMCTRN1, 15,   							//Negative Gamma Correction
  0x00, 0x0E, 0x14, 0x03, 0x11, 0x07, 0x31, 0xC1,
  0x48, 0x08, 0x0F, 0x0C, 0x31, 0x36, 0x0F,
  ILI9341_SLPOUT, DELAY, 							//  Sleep out
  120,			 									//  120 ms delay
  TFT_DISPON, 0,
};

//Send a command to the ILI9341. Uses spi_device_transmit, which waits until the transfer is complete.
//-------------------------------------------------------------
static void ili_cmd(spi_device_handle_t spi, const uint8_t cmd) 
{
    esp_err_t ret;
    spi_transaction_t t;
    memset(&t, 0, sizeof(t));          //Zero out the transaction
    t.length=8;                        //Command is 8 bits
    t.tx_buffer=&cmd;                  //The data is the cmd itself
    t.user=(void*)0;                   //D/C needs to be set to 0
    ret=spi_device_transmit(spi, &t);  //Transmit!
    assert(ret==ESP_OK);               //Should have had no issues.
}

//Send data to the ILI9341. Uses spi_device_transmit, which waits until the transfer is complete.
//-------------------------------------------------------------------------
static void ili_data(spi_device_handle_t spi, const uint8_t *data, int len) 
{
    if (len==0) return;             //no need to send anything

    esp_err_t ret;
    spi_transaction_t t;
    memset(&t, 0, sizeof(t));          //Zero out the transaction
    t.length=len*8;                    //Len is in bytes, transaction length is in bits.
    t.tx_buffer=data;                  //Data
    t.user=(void*)1;                   //D/C needs to be set to 1
    ret=spi_device_transmit(spi, &t);  //Transmit!
    assert(ret==ESP_OK);               //Should have had no issues.
}

//------------------------------------------------------
// Companion code to the above tables.  Reads and issues
// a series of LCD commands stored in byte array
//---------------------------------------------------------------------
static void commandList(spi_device_handle_t spi, const uint8_t *addr) {
  uint8_t  numCommands, numArgs, cmd;
  uint16_t ms;

  numCommands = *addr++;         // Number of commands to follow
  while(numCommands--) {         // For each command...
    cmd = *addr++;               // save command
    numArgs  = *addr++;          //   Number of args to follow
    ms       = numArgs & DELAY;  //   If high bit set, delay follows args
    numArgs &= ~DELAY;           //   Mask out delay bit

    // Use fast direct spi function to send command & data
	disp_spi_transfer_cmd_data(spi, cmd, (uint8_t *)addr, numArgs);
	/*
	// Or use DMA/transactions to send command and data
    ili_cmd(spi, cmd);
    ili_data(spi, addr, numArgs);
	*/
    addr += numArgs;

    if(ms) {
      ms = *addr++;              // Read post-command delay time (ms)
      if(ms == 255) ms = 500;    // If 255, delay for 500 ms
	  vTaskDelay(ms / portTICK_RATE_MS);
    }
  }
}

//This function is called (in irq context!) just before a transmission starts. It will
//set the D/C line to the value indicated in the user field.
//-----------------------------------------------------------------------
static void IRAM_ATTR ili_spi_pre_transfer_callback(spi_transaction_t *t) 
{
    int dc=(int)t->user;
    gpio_set_level(PIN_NUM_DC, dc);
}

//Initialize the display
//--------------------------------------------
static void ili_init(spi_device_handle_t spi) 
{
    esp_err_t ret;
    //Initialize non-SPI GPIOs
    gpio_set_direction(PIN_NUM_DC, GPIO_MODE_OUTPUT);
	/* Reset and backlit pins are not used
    gpio_set_direction(PIN_NUM_RST, GPIO_MODE_OUTPUT);
    gpio_set_direction(PIN_NUM_BCKL, GPIO_MODE_OUTPUT);

    //Reset the display
    gpio_set_level(PIN_NUM_RST, 0);
    vTaskDelay(100 / portTICK_RATE_MS);
    gpio_set_level(PIN_NUM_RST, 1);
    vTaskDelay(100 / portTICK_RATE_MS);
	*/

	//Send all the commands
	ret = spi_device_select(spi, 0);
	assert(ret==ESP_OK);

    commandList(spi, ILI9341_init);

	ret = spi_device_deselect(spi);
	assert(ret==ESP_OK);

    ///Enable backlight
    //gpio_set_level(PIN_NUM_BCKL, 0);
}


//To send a line we have to send a command, 2 data bytes, another command, 2 more data bytes and another command
//before sending the line data itself; a total of 6 transactions. (We can't put all of this in just one transaction
//because the D/C line needs to be toggled in the middle.)
//This routine queues these commands up so they get sent as quickly as possible.
//----------------------------------------------------------------------
static void send_line(spi_device_handle_t spi, int ypos, uint16_t *line) 
{
    esp_err_t ret;
    int x;

	for (x=0; x<6; x++) {
        memset(&trans[x], 0, sizeof(spi_transaction_t));
        if ((x&1)==0) {
            //Even transfers are commands
            trans[x].length=8;
            trans[x].user=(void*)0;
        } else {
            //Odd transfers are data
            trans[x].length=8*4;
            trans[x].user=(void*)1;
        }
        trans[x].flags=SPI_TRANS_USE_TXDATA;
    }
    trans[0].tx_data[0]=0x2A;           //Column Address Set
    trans[1].tx_data[0]=0;              //Start Col High
    trans[1].tx_data[1]=0;              //Start Col Low
    trans[1].tx_data[2]=(319)>>8;       //End Col High
    trans[1].tx_data[3]=(319)&0xff;     //End Col Low
    trans[2].tx_data[0]=0x2B;           //Page address set
    trans[3].tx_data[0]=ypos>>8;        //Start page high
    trans[3].tx_data[1]=ypos&0xff;      //start page low
    trans[3].tx_data[2]=(ypos)>>8;      //end page high
    trans[3].tx_data[3]=(ypos)&0xff;    //end page low
    trans[4].tx_data[0]=TFT_RAMWR;      //memory write
    trans[5].tx_buffer=line;            //finally send the line data
    trans[5].length=320*2*8;            //Data length, in bits
    trans[5].flags=0;                   //undo SPI_TRANS_USE_TXDATA flag

    //Queue all transactions.
    for (x=0; x<6; x++) {
        ret=spi_device_queue_trans(spi, &trans[x], portMAX_DELAY);
        assert(ret==ESP_OK);
    }

    //When we are here, the SPI driver is busy (in the background) getting the transactions sent. That happens
    //mostly using DMA, so the CPU doesn't have much to do here. We're not going to wait for the transaction to
    //finish because we may as well spend the time calculating the next line. When that is done, we can call
    //send_line_finish, which will wait for the transfers to be done and check their status.
}

// Send pixel using DMA transfer & transactions
//-------------------------------------------------------------------------------------
static void send_pixel(spi_device_handle_t spi, uint16_t x, uint16_t y, uint16_t color) 
{
    esp_err_t ret;
    int xx;

	for (xx=0; xx<6; xx++) {
        memset(&trans[xx], 0, sizeof(spi_transaction_t));
        if ((xx&1)==0) {
            //Even transfers are commands
            trans[xx].length=8;
            trans[xx].user=(void*)0;
        } else {
            //Odd transfers are data
            trans[xx].length=8*4;
            trans[xx].user=(void*)1;
        }
        trans[xx].flags=SPI_TRANS_USE_TXDATA;
    }
    trans[0].tx_data[0]=0x2A;        //Column Address Set
    trans[1].tx_data[0]=x>>8;        //Start Col High
    trans[1].tx_data[1]=x&0xff;      //Start Col Low
    trans[1].tx_data[2]=(x+1)>>8;    //End Col High
    trans[1].tx_data[3]=(x+1)&0xff;  //End Col Low
    trans[2].tx_data[0]=0x2B;        //Page address set
    trans[3].tx_data[0]=y>>8;        //Start page high
    trans[3].tx_data[1]=y&0xff;      //start page low
    trans[3].tx_data[2]=(y+1)>>8;    //end page high
    trans[3].tx_data[3]=(y+1)&0xff;  //end page low
    trans[4].tx_data[0]=0x2C;        //memory write
    trans[5].tx_data[0]=color>>8;    //set color lo byte
    trans[5].tx_data[1]=color&0xff;  //het color hi byte
    trans[5].length=16;              //Data length, in bits

    //Queue all transactions.
    for (xx=0; xx<6; xx++) {
        ret=spi_device_queue_trans(spi, &trans[xx], portMAX_DELAY);
        assert(ret==ESP_OK);
    }

    //When we are here, the SPI driver is busy (in the background) getting the transactions sent. That happens
    //mostly using DMA, so the CPU doesn't have much to do here.

    //We don't have much to do after sending the pixel
    //so wait for the transfers to be done right here
    spi_transaction_t *rtrans;
    //Wait for all 6 transactions to be done and get back the results.
    for (int xx=0; xx<6; xx++) {
        ret=spi_device_get_trans_result(spi, &rtrans, portMAX_DELAY);
        assert(ret==ESP_OK);
        //We could inspect rtrans now if we received any info back. The LCD is treated as write-only, though.
    }
}

//Wait for all 6 transactions to be done and get back the results.
//---------------------------------------------------
static void send_line_finish(spi_device_handle_t spi) 
{
    spi_transaction_t *rtrans;
    esp_err_t ret;
    //Wait for all 6 transactions to be done and get back the results.
    for (int x=0; x<6; x++) {
        ret=spi_device_get_trans_result(spi, &rtrans, portMAX_DELAY);
        assert(ret==ESP_OK);
        //We could inspect rtrans now if we received any info back. The LCD is treated as write-only, though.
    }
}

#ifdef USE_TOUCH
//Send a command to the Touch screen
//---------------------------------------------------------
uint16_t ts_cmd(spi_device_handle_t spi, const uint8_t cmd) 
{
    esp_err_t ret;
    spi_transaction_t t;
	uint8_t txdata[4] = {0};
	uint8_t rxdata[4] = {0};
	txdata[0] = cmd;
    memset(&t, 0, sizeof(t));            //Zero out the transaction
    t.length=8*3;                        //Command is 8 bits
    t.rxlength=8*3;
    t.tx_buffer=&txdata;
    t.rx_buffer=&rxdata;
    //ret=spi_device_transmit(spi, &t);  //Transmit!
    ret = spi_transfer_data(spi, &t);    // Transmit using direct mode
    assert(ret==ESP_OK);                 //Should have had no issues.
	//printf("TS: %02x,%02x,%02x,%02x\r\n",rxdata[0],rxdata[1],rxdata[2],rxdata[3]);
    return (((uint16_t)(rxdata[1] << 8) | (uint16_t)(rxdata[2])) >> 4);
}
#endif

/**
 * Converts the components of a color, as specified by the HSB
 * model, to an equivalent set of values for the default RGB model.
 * The _sat and _brightness components
 * should be floating-point values between zero and one (numbers in the range 0.0-1.0)
 * The _hue component can be any floating-point number.  The floor of this number is
 * subtracted from it to create a fraction between 0 and 1.
 * This fractional number is then multiplied by 360 to produce the hue
 * angle in the HSB color model.
 * The integer that is returned by HSBtoRGB encodes the
 * value of a color in bits 0-15 of an integer value
*/
//-------------------------------------------------------------------
static uint16_t HSBtoRGB(float _hue, float _sat, float _brightness) {
 float red = 0.0;
 float green = 0.0;
 float blue = 0.0;

 if (_sat == 0.0) {
   red = _brightness;
   green = _brightness;
   blue = _brightness;
 } else {
   if (_hue == 360.0) {
     _hue = 0;
   }

   int slice = (int)(_hue / 60.0);
   float hue_frac = (_hue / 60.0) - slice;

   float aa = _brightness * (1.0 - _sat);
   float bb = _brightness * (1.0 - _sat * hue_frac);
   float cc = _brightness * (1.0 - _sat * (1.0 - hue_frac));

   switch(slice) {
     case 0:
         red = _brightness;
         green = cc;
         blue = aa;
         break;
     case 1:
         red = bb;
         green = _brightness;
         blue = aa;
         break;
     case 2:
         red = aa;
         green = _brightness;
         blue = cc;
         break;
     case 3:
         red = aa;
         green = bb;
         blue = _brightness;
         break;
     case 4:
         red = cc;
         green = aa;
         blue = _brightness;
         break;
     case 5:
         red = _brightness;
         green = aa;
         blue = bb;
         break;
     default:
         red = 0.0;
         green = 0.0;
         blue = 0.0;
         break;
   }
 }

 uint8_t ired = (uint8_t)(red * 31.0);
 uint8_t igreen = (uint8_t)(green * 63.0);
 uint8_t iblue = (uint8_t)(blue * 31.0);

 return (uint16_t)((ired << 11) | (igreen << 5) | (iblue & 0x001F));
}

//Simple routine to test display functions in DMA/transactions & direct mode
//--------------------------------------------------------------------------
static void display_test(spi_device_handle_t spi, spi_device_handle_t tsspi) 
{
    uint32_t speeds[6] = {5000000,8000000,16000000,20000000,30000000,40000000};
    int speed_idx = 0;
	esp_err_t ret;
    uint16_t line[3][320];
    int x, y;
    //Indexes of the line currently being sent to the LCD and the line we're calculating.
    int sending_line=-1;
    int calc_line=0;
	int line_check=0, line_check1=0;
	uint16_t color;
    uint32_t t1=0,t2=0,t3=0,t4=0,t5=0, tstart;
	float hue_inc;
#ifdef USE_TOUCH
	int tx, ty, tz=0;
#endif
    
    while(1) {
		// *** Clear screen
		color = 0;
		ret = spi_device_select(spi, 0);
		assert(ret==ESP_OK);
		tstart = clock();
		disp_spi_transfer_addrwin(spi, 0, 319, 0, 239);
		disp_spi_transfer_cmd(spi, TFT_RAMWR);
		disp_spi_transfer_color_rep(spi, (uint8_t *)&color,  320*240, 1);
		t5 = clock() - tstart;
		color = 0xFFFF;
		disp_spi_transfer_addrwin(spi, 0, 319, 120, 120);
		disp_spi_transfer_color_rep(spi, (uint8_t *)&color,  320, 1);
		disp_spi_transfer_addrwin(spi, 160, 160, 0, 239);
		disp_spi_transfer_color_rep(spi, (uint8_t *)&color,  240, 1);
		vTaskDelay(1000 / portTICK_RATE_MS);
		ret = spi_device_deselect(spi);
		assert(ret==ESP_OK);

		// *** display lines in DMA mode
		sending_line=-1;
		calc_line=0;
		line_check=-9999;
		tstart = clock();
		ret = spi_device_select(spi, 0);
		assert(ret==ESP_OK);
        for (y=0; y<240; y++) {
            //Calculate a line.
			hue_inc = (float)(((float)y / 239.0) * 360.0);
            for (x=0; x<320; x++) {
				color = HSBtoRGB(hue_inc, 1.0, 1.0 - (float)(x/640.0));
				line[calc_line][x]=color; //(uint16_t)((color>>8) | (color & 0xFF));
            }
            if (y == 120) memcpy(line[2], line[calc_line], 320*2);
            //Finish up the sending process of the previous line, if any
            if (sending_line!=-1) send_line_finish(spi);
            //Swap sending_line and calc_line
            sending_line=calc_line;
            calc_line=(calc_line==1)?0:1;
            //Send the line we currently calculated.
            send_line(spi, y, line[sending_line]);
            //The line is queued up for sending now; the actual sending happens in the
            //background. We can go on to calculate the next line as long as we do not
            //touch line[sending_line]; the SPI sending process is still reading from that.
        }
        if (sending_line!=-1) send_line_finish(spi);
		// Check line
		ret = disp_spi_read_data(spi, 0, 120, 319, 120, 320, (uint8_t *)(line[0]));
		if (ret == ESP_OK) line_check = memcmp((uint8_t *)(line[0]), (uint8_t *)(line[2]), 320*2);

		ret =spi_device_deselect(spi);
		assert(ret==ESP_OK);
		t1 = clock() - tstart;
		vTaskDelay(1000 / portTICK_RATE_MS);

		// *** Display pixels using DMA/transactions mode
		//     It is slow, only write 1/4 of the screen
		tstart = clock();
		ret = spi_device_select(spi, 0);
		assert(ret==ESP_OK);
		for (y=0; y<120; y++) {
			for (x=0; x<160; x++) {
				//disp_spi_transfer_addrwin(spi,x,x+1,y,y+1);
				send_pixel(spi, x, y, 0x07E0);
			}
		}
		ret =spi_device_deselect(spi);
		assert(ret==ESP_OK);
		t3 = clock() - tstart;
		vTaskDelay(1000 / portTICK_RATE_MS);

		// *** Send lines in direct mode
		tstart = clock();
		ret = spi_device_select(spi, 0);
		assert(ret==ESP_OK);
		line_check1=-9999;
		for (y=0; y<240; y++) {
			hue_inc = (float)(((float)y / 239.0) * 360.0);
            for (x=0; x<320; x++) {
				color = HSBtoRGB(hue_inc, 1.0, (float)(x/640.0) + 0.5);
				line[0][x] = color; //(uint16_t)((color>>8) | (color & 0xFF));
			}
			disp_spi_transfer_addrwin(spi, 0, 319, y, y);
			disp_spi_transfer_color_rep(spi, (uint8_t *)(line[0]),  320, 0);
            if (y == 120) memcpy(line[2], line[0], 320*2);
		}
		// Check line
		ret = disp_spi_read_data(spi, 0, 120, 319, 120, 320, (uint8_t *)(line[0]));
		if (ret == ESP_OK) line_check1 = memcmp((uint8_t *)(line[0]), (uint8_t *)(line[2]), 320*2);

		ret =spi_device_deselect(spi);
		assert(ret==ESP_OK);
		t2 = clock() - tstart;
		vTaskDelay(1000 / portTICK_RATE_MS);

		// *** Display pixels using direct mode
		tstart = clock();
		ret = spi_device_select(spi, 0);
		assert(ret==ESP_OK);
		for (y=120; y<240; y++) {
			for (x=160; x<320; x++) {
				//disp_spi_transfer_addrwin(spi,x,x+1,y,y+1);
				disp_spi_set_pixel(spi, x, y, 0x001f);
			}
		}
		ret = spi_device_deselect(spi);
		assert(ret==ESP_OK);
		t4 = clock() - tstart;
		vTaskDelay(1000 / portTICK_RATE_MS);
		
		// *** Clear screen using direct mode
		color = 0xF800;
		ret = spi_device_select(spi, 0);
		assert(ret==ESP_OK);
		disp_spi_transfer_addrwin(spi, 10, 309, 10, 229);
		disp_spi_transfer_cmd(spi, TFT_RAMWR);
		disp_spi_transfer_color_rep(spi, (uint8_t *)&color,  310*230, 1);
		ret = spi_device_deselect(spi);
		assert(ret==ESP_OK);
		vTaskDelay(1000 / portTICK_RATE_MS);

#ifdef USE_TOUCH
		// Get toush status
		ret = spi_device_select(tsspi, 0);
		assert(ret==ESP_OK);
		tz = ts_cmd(tsspi, 0xB0);
		if (tz > 100) {
			// Touched, get coordinates
			tx = ts_cmd(tsspi, 0xD0);
			ty = ts_cmd(tsspi, 0x90);
		}
		ret = spi_device_deselect(tsspi);
		assert(ret==ESP_OK);
#endif

		// *** Print info
		printf("-------------\r\n");
		printf(" Disp clock = %u Hz (%u)\r\n", get_speed(spi), speeds[speed_idx]);
		printf(" Lines(DMA) = %d ms\r\n",t1);
		printf(" Read check   ");
		if (line_check == 0) printf("OK\r\n");
		else printf("Err\r\n");
		printf("      Lines = %d ms\r\n",t2);
		printf(" Read check   ");
		if (line_check1 == 0) printf("OK\r\n");
		else printf("Err\r\n");
		printf("Pixels(DMA) = %d ms\r\n",t3*4);
		printf("     Pixels = %d ms\r\n",t4*4);
		printf("        Cls = %d ms\r\n",t5);
#ifdef USE_TOUCH
		if (tz > 100) {
			printf(" Touched at   (%d,%d)\r\n",tx,ty);
		}
#endif
		printf("-------------\r\n");

		// Change SPI speed
		speed_idx++;
		if (speed_idx > 5) speed_idx = 0;
		set_speed(spi, speeds[speed_idx]);
    }
}

//-------------
void app_main()
{
    esp_err_t ret;
    spi_device_handle_t spi;
    spi_device_handle_t tsspi = NULL;
    spi_bus_config_t buscfg={
        .miso_io_num=PIN_NUM_MISO,
        .mosi_io_num=PIN_NUM_MOSI,
        .sclk_io_num=PIN_NUM_CLK,
        .quadwp_io_num=-1,
        .quadhd_io_num=-1
    };
    spi_device_interface_config_t devcfg={
        .clock_speed_hz=5000000,                //Initial clock out at 5 MHz
        .mode=0,                                //SPI mode 0
        .spics_io_num=-1,                       //we will use external CS pin
		.spics_ext_io_num=PIN_NUM_CS,           //external CS pin
        .queue_size=7,                          //We want to be able to queue 7 transactions at a time
        .pre_cb=ili_spi_pre_transfer_callback,  //Specify pre-transfer callback to handle D/C line
    };

#ifdef USE_TOUCH
	spi_device_interface_config_t tsdevcfg={
        .clock_speed_hz=2500000,                //Clock out at 2.5 MHz
        .mode=0,                                //SPI mode 2
        .spics_io_num=PIN_NUM_TCS,              //Touch CS pin
		.spics_ext_io_num=-1,                   //Not using the external CS
        .queue_size=3,                          //We want to be able to queue 3 transactions at a time
        .pre_cb=NULL,                           //No need for pre-transfer callback
    };
#endif

	//Initialize the SPI bus
    ret=spi_bus_initialize(SPI_BUS, &buscfg, 1);
    assert(ret==ESP_OK);
    //Attach the LCD to the SPI bus
    ret=spi_bus_add_device(SPI_BUS, &devcfg, &buscfg, &spi);
    assert(ret==ESP_OK);
	printf("SPI: bus initialized\r\n");

	// Test select/deselect
	ret = spi_device_select(spi, 1);
    assert(ret==ESP_OK);
	ret = spi_device_deselect(spi);
    assert(ret==ESP_OK);

	printf("SPI: attached display device, speed=%u\r\n", get_speed(spi));
	printf("SPI: bus uses native pins: %s\r\n", spi_uses_native_pins(spi) ? "true" : "false");

#ifdef USE_TOUCH
    //Attach the TS to the SPI bus
    ret=spi_bus_add_device(SPI_BUS, &tsdevcfg, &buscfg, &tsspi);
    assert(ret==ESP_OK);

	// Test select/deselect
	ret = spi_device_select(tsspi, 1);
    assert(ret==ESP_OK);
	ret = spi_device_deselect(tsspi);
    assert(ret==ESP_OK);

	printf("SPI: attached TS device, speed=%u\r\n", get_speed(tsspi));
#endif

	//Initialize the LCD
	printf("SPI: display init...\r\n");
    ili_init(spi);
	printf("OK\r\n");
	vTaskDelay(500 / portTICK_RATE_MS);

	//Start display test
    display_test(spi, tsspi);
}
