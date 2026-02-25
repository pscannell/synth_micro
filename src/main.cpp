//
// This file is part of the GNU ARM Eclipse distribution.
// Copyright (c) 2014 Liviu Ionescu.
//

// ----------------------------------------------------------------------------

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "diag/Trace.h"

extern "C" {
#include "uart.h"
#include "stm32f0xx.h"
#include "Timer.h"
#include "BlinkLed.h"
#include "ssd1306.h"
#include "rtt_io.h"
}

#include "OSC/OSCMessage.h"
#include "SLIPEncodedSerial.h"
#include "OSC/SimpleWriter.h"

// ADC DMA stuff
#define ADC1_DR_Address    0x40012440
__IO uint16_t RegularConvData_Tab[9];

// keys and knobs
uint8_t keyValuesRaw[28];
uint8_t keyValues[4][28];
uint8_t keyValuesLast[28];
uint32_t knobValues[6];

// key mux
#define MUX_SEL_A_1 GPIO_SetBits(GPIOC, GPIO_Pin_6)
#define MUX_SEL_A_0 GPIO_ResetBits(GPIOC, GPIO_Pin_6)
#define MUX_SEL_B_1 GPIO_SetBits(GPIOC, GPIO_Pin_7)
#define MUX_SEL_B_0 GPIO_ResetBits(GPIOC, GPIO_Pin_7)
#define MUX_SEL_C_1 GPIO_SetBits(GPIOC, GPIO_Pin_8)
#define MUX_SEL_C_0 GPIO_ResetBits(GPIOC, GPIO_Pin_8)

// key mux states
#define MUX_0 MUX_SEL_A_0;MUX_SEL_B_0;MUX_SEL_C_0;
#define MUX_1 MUX_SEL_A_1;MUX_SEL_B_0;MUX_SEL_C_0;
#define MUX_2 MUX_SEL_A_0;MUX_SEL_B_1;MUX_SEL_C_0;
#define MUX_3 MUX_SEL_A_1;MUX_SEL_B_1;MUX_SEL_C_0;
#define MUX_4 MUX_SEL_A_0;MUX_SEL_B_0;MUX_SEL_C_1;
#define MUX_5 MUX_SEL_A_1;MUX_SEL_B_0;MUX_SEL_C_1;
#define MUX_6 MUX_SEL_A_0;MUX_SEL_B_1;MUX_SEL_C_1;
#define MUX_7 MUX_SEL_A_1;MUX_SEL_B_1;MUX_SEL_C_1;

// OSC stuff
SLIPEncodedSerial slip;
SimpleWriter oscBuf;

// for outputting to screen
uint8_t spi_out_buf[130];
uint8_t spi_out_buf_remaining = 0;
uint8_t spi_out_buf_index = 0;

// OLED frame
extern uint8_t pix_buf[];

//// hardware init
static void ADC_Config(void);
static void DMA_Config(void);
void hardwareInit(void);


// OSC callbacks
void oledControl(OSCMessage &msg);
void ledControl(OSCMessage &msg);
void getKnobs(OSCMessage &msg);
void shutdown(OSCMessage &msg);
// end OSC callbacks

/// scan keys
void keyMuxSel(uint32_t sel);
uint32_t scanKeys();
void remapKeys();
void checkForKeyEvent();
void updateKeyStateNoOsc();
void updateKnobs() ;
void checkEncoder(void) ;
void setLedState(uint8_t stat);
void rttHandleCommandLine(const char *line);
void rttService(void);
void rttEmitTelemetry(void);
uint32_t packKeyBits(void);

//foot
void checkFootSwitch (void) ;

// RTT state
uint8_t g_led_state = 0;
int32_t g_encoder_accum = 0;
uint8_t g_encoder_button = 0;
uint8_t g_foot_state = 0;
uint32_t g_next_rtt_tick = 0;
uint32_t g_next_encoder_tick = 0;
const int8_t ENC_STEPS_PER_DETENT = 2;
const uint8_t ENCODER_INVERT = 1;

int main(int argc, char* argv[]) {
	// SystemInit();
    // SystemCoreClockUpdate();

	OSCMessage msgIn;

	blink_led_init();
	blink_led_off();


	timer_start();

	// flash leds while power stabilizes
	// before initializing ADC
	stopwatchStart();
	while (stopwatchReport() < 500){ AUX_LED_RED_ON; }
	stopwatchStart();
	while (stopwatchReport() < 500){ AUX_LED_GREEN_ON; }
	stopwatchStart();
	while (stopwatchReport() < 500){ AUX_LED_BLUE_ON; }
	stopwatchStart();

	AUX_LED_RED_OFF;
	AUX_LED_GREEN_OFF;
	AUX_LED_BLUE_OFF;

	uart2_init();

	hardwareInit();
	rtt_init();
	g_next_rtt_tick = timer_now() + 1000;
	g_next_encoder_tick = timer_now() + 10;
	rtt_printf("RTT ready. Commands: help, led <0-7>, dump, oled <text>\n");

	// Infinite loop

	// oled init
	ssd1306_init(0);

	// Print Intro  Header
	println_8_spacy(" Open Pd Synth ", 15, 4, 4);

	println_8("open_synth.local", 16, 8, 28);
	println_8("for storage", 11, 8, 38);

	char progressStr[20];
	int len = 0;
	int progress = 0;
	len = sprintf(progressStr, "starting: %d %%", progress);
	println_8(progressStr, len, 8, 52);
	ssd1306_refresh();

	stopwatchStart();

	while (1) {
		rttService();
		if (scanKeys() == 0) {
			updateKeyStateNoOsc();
			checkFootSwitch();
		}
		updateKnobs();
		if ((int32_t) (timer_now() - g_next_encoder_tick) >= 0) {
			g_next_encoder_tick = timer_now() + 10; // 1 ms at 10 kHz tick rate.
			checkEncoder();
		}

		if (slip.recvMessage()) {
			// fill the message and dispatch it

			msgIn.fill(slip.decodedBuf, slip.decodedLength);

			// dispatch it
			if (!msgIn.hasError()) {
				// wait for start message so we aren't sending stuff during boot
				if (msgIn.fullMatch("/ready", 0)) {
					msgIn.empty(); // free space occupied by message
					break;
				}
				msgIn.empty();
			} else {   // just empty it if there was an error
				msgIn.empty(); // free space occupied by message
			}
		}
		if (stopwatchReport() > 1800) { // Increased boot time for Pi (PS)
			stopwatchStart();
			len = sprintf(progressStr, "starting: %d %%", progress);
			if (progress < 99)
				progress++;
			println_8(progressStr, len, 8, 52);
			ssd1306_refresh();
		}

	} // waiting for /ready command

	stopwatchStart();   // used to check encoder only 1 per 5 ms

	while (1) {
		rttService();
		if (slip.recvMessage()) {
			// fill the message and dispatch it

			msgIn.fill(slip.decodedBuf, slip.decodedLength);

			// dispatch it
			if (!msgIn.hasError()) {
				msgIn.dispatch("/led", ledControl, 0);
				msgIn.dispatch("/oled", oledControl, 0);
				msgIn.dispatch("/getknobs", getKnobs, 0);
				msgIn.dispatch("/shutdown", shutdown, 0);
				msgIn.empty(); // free space occupied by message
			} else {   // just empty it if there was an error
				msgIn.empty(); // free space occupied by message
			}
		}

		// every time mux gets back to 0 key scan is complete
		if (scanKeys() == 0) {
			checkForKeyEvent(); // and send em out if we got em
			// also check about the foot switch
			checkFootSwitch();
		}
		updateKnobs();

		// check encoder
		// only check every 2 ms cause the button needs a lot of debounce time
		if (stopwatchReport() > 5){
			stopwatchStart();
			checkEncoder();
		}

	} // Infinite loop, never return.
}

void hardwareInit(void){


	/* DMA configuration */
	DMA_Config();

	/* ADC1 configuration */
	ADC_Config();

	// MUX SEL Lines
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_StructInit(&GPIO_InitStructure);
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOC, ENABLE);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7 | GPIO_Pin_8;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;

	GPIO_Init(GPIOC, &GPIO_InitStructure);

	GPIO_ResetBits(GPIOC, GPIO_Pin_6); //
	GPIO_SetBits(GPIOC, GPIO_Pin_7); //
	GPIO_SetBits(GPIOC, GPIO_Pin_8); //
	// end mux select lines

	// Enc lines
	// config as input
	// GPIO_StructInit(&GPIO_InitStructure);
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOB, ENABLE);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_14 | GPIO_Pin_13 | GPIO_Pin_15;
	// GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	// end enc lines

	// key lines
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOB, ENABLE);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2
			| GPIO_Pin_3 | GPIO_Pin_9 | GPIO_Pin_10;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP; // Changed to pull-up for buttons connected to GND (PS)
	GPIO_Init(GPIOC, &GPIO_InitStructure);

	// foot switch
	/*RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;
	GPIO_Init(GPIOA, &GPIO_InitStructure);*/

}

//// ADC DMA stuff
static void ADC_Config(void) {
	ADC_InitTypeDef ADC_InitStructure;
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_StructInit(&GPIO_InitStructure);
	/* ADC1 DeInit */
	ADC_DeInit(ADC1);

	/* GPIOC Periph clock enable */
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOB, ENABLE);
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOC, ENABLE);

	/* ADC1 Periph clock enable */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);

	/* Configure  as analog input */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4 | GPIO_Pin_5;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOC, &GPIO_InitStructure);

	/* Configure as analog input */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	/* Configure as analog input */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4 | GPIO_Pin_1;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	/* Initialize ADC structure */
	ADC_StructInit(&ADC_InitStructure);

	/* Configure the ADC1 in continuous mode withe a resolution equal to 12 bits  */
	ADC_InitStructure.ADC_Resolution = ADC_Resolution_10b;
	ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;
	ADC_InitStructure.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_None;
	ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
	ADC_InitStructure.ADC_ScanDirection = ADC_ScanDirection_Backward;
	ADC_Init(ADC1, &ADC_InitStructure);

	/* Convert the ADC1 Channel11 and channel10 with 55.5 Cycles as sampling time */
	ADC_ChannelConfig(ADC1, ADC_Channel_14, ADC_SampleTime_55_5Cycles);
	ADC_ChannelConfig(ADC1, ADC_Channel_15, ADC_SampleTime_55_5Cycles);
	ADC_ChannelConfig(ADC1, ADC_Channel_4, ADC_SampleTime_55_5Cycles);
	ADC_ChannelConfig(ADC1, ADC_Channel_8, ADC_SampleTime_55_5Cycles);
	ADC_ChannelConfig(ADC1, ADC_Channel_9, ADC_SampleTime_55_5Cycles);
	ADC_ChannelConfig(ADC1, ADC_Channel_1, ADC_SampleTime_55_5Cycles);

	/* ADC Calibration */
	ADC_GetCalibrationFactor(ADC1);

	/* ADC DMA request in circular mode */
	ADC_DMARequestModeConfig(ADC1, ADC_DMAMode_Circular);

	/* Enable ADC_DMA */
	ADC_DMACmd(ADC1, ENABLE);

	/* Enable the ADC peripheral */
	ADC_Cmd(ADC1, ENABLE);

	/* Wait the ADRDY flag */
	while (!ADC_GetFlagStatus(ADC1, ADC_FLAG_ADRDY))
		;

	/* ADC1 regular Software Start Conv */
	ADC_StartOfConversion(ADC1);
}

/**
 * @brief  DMA channel1 configuration
 * @param  None
 * @retval None
 */
static void DMA_Config(void) {
	DMA_InitTypeDef DMA_InitStructure;
	/* DMA1 clock enable */
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);

	/* DMA1 Channel1 Config */
	DMA_DeInit(DMA1_Channel1);
	DMA_StructInit(&DMA_InitStructure);
	DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t) ADC1_DR_Address;
	DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t) RegularConvData_Tab;
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;
	DMA_InitStructure.DMA_BufferSize = 6;
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
	DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
	DMA_InitStructure.DMA_Priority = DMA_Priority_High;
	DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
	DMA_Init(DMA1_Channel1, &DMA_InitStructure);

	/* DMA1 Channel1 enable */
	DMA_Cmd(DMA1_Channel1, ENABLE);

}

//// end ADC DMA

// OSC callbacks
void oledControl(OSCMessage &msg) {

	uint8_t tmp[132];
	uint8_t i;
	uint8_t line = 0;

	if (msg.isInt(0)) {
		line = msg.getInt(0) & 0x7;
	}
	if (msg.isBlob(1)) {
		msg.getBlob(1, tmp, 132);
	}

	// shift array 4 spaces cause first 4 bytes are the length of blob
	for (i = 0; i < 128; i++)
		pix_buf[i + (line * 128)] = tmp[i + 4];

	ssd1306_refresh_line(line);
}

void setLedState(uint8_t stat) {
	g_led_state = stat & 0x7;

	if (g_led_state == 0) {
		AUX_LED_RED_OFF;
		AUX_LED_GREEN_OFF;
		AUX_LED_BLUE_OFF;
	}
	if (g_led_state == 1) {
		AUX_LED_RED_OFF;
		AUX_LED_GREEN_OFF;
		AUX_LED_BLUE_ON;
	}
	if (g_led_state == 2) {
		AUX_LED_RED_OFF;
		AUX_LED_GREEN_ON;
		AUX_LED_BLUE_OFF;
	}
	if (g_led_state == 3) {
		AUX_LED_RED_OFF;
		AUX_LED_GREEN_ON;
		AUX_LED_BLUE_ON;
	}
	if (g_led_state == 4) {
		AUX_LED_RED_ON;
		AUX_LED_GREEN_OFF;
		AUX_LED_BLUE_OFF;
	}
	if (g_led_state == 5) {
		AUX_LED_RED_ON;
		AUX_LED_GREEN_OFF;
		AUX_LED_BLUE_ON;
	}
	if (g_led_state == 6) {
		AUX_LED_RED_ON;
		AUX_LED_GREEN_ON;
		AUX_LED_BLUE_OFF;
	}
	if (g_led_state == 7) {
		AUX_LED_RED_ON;
		AUX_LED_GREEN_ON;
		AUX_LED_BLUE_ON;
	}
}

uint32_t packKeyBits(void) {
	uint32_t bits = 0;
	uint32_t i;
	for (i = 0; i < 28; i++) {
		if (keyValuesLast[i]) {
			bits |= (1u << i);
		}
	}
	return bits;
}

void rttEmitTelemetry(void) {
	rtt_printf(
			"STAT knobs=%lu,%lu,%lu,%lu,%lu,%lu keys=%08lx enc=%ld encbtn=%u foot=%u led=%u\n",
			(unsigned long) knobValues[0],
			(unsigned long) knobValues[1],
			(unsigned long) knobValues[2],
			(unsigned long) knobValues[3],
			(unsigned long) knobValues[4],
			(unsigned long) knobValues[5],
			(unsigned long) packKeyBits(),
			(long) g_encoder_accum,
			(unsigned) g_encoder_button,
			(unsigned) g_foot_state,
			(unsigned) g_led_state);
}

void rttHandleCommandLine(const char *line) {
	if ((line == 0) || (line[0] == '\0')) {
		return;
	}
	if (strcmp(line, "help") == 0) {
		rtt_printf("CMD help | dump | led <0-7> | oled <text>\n");
		return;
	}
	if (strcmp(line, "dump") == 0) {
		rttEmitTelemetry();
		return;
	}
	if (strncmp(line, "led ", 4) == 0) {
		int stat = atoi(line + 4);
		if (stat >= 0 && stat <= 7) {
			setLedState((uint8_t) stat);
			rtt_printf("ACK led %d\n", stat);
		} else {
			rtt_printf("ERR led range 0..7\n");
		}
		return;
	}
	if (strncmp(line, "oled ", 5) == 0) {
		char text[22];
		uint32_t i;
		const char *msg = line + 5;
		for (i = 0; i < 1024; i++) {
			pix_buf[i] = 0;
		}
		snprintf(text, sizeof(text), "%s", msg);
		println_8(text, (int) strlen(text), 1, 18);
		ssd1306_refresh();
		rtt_printf("ACK oled\n");
		return;
	}
	rtt_printf("ERR unknown command: %s\n", line);
}

void rttService(void) {
	char line[128];
	uint32_t now = timer_now();

	while (rtt_read_line(line, sizeof(line))) {
		rttHandleCommandLine(line);
	}

	if ((int32_t) (now - g_next_rtt_tick) >= 0) {
		g_next_rtt_tick = now + 1000; // 100 ms at 10 kHz tick rate.
		rttEmitTelemetry();
	}
}

void ledControl(OSCMessage &msg) {

	blink_led_on();

	int stat = 0;

	// digitalWrite(ledPin, LOW);
	if (msg.isInt(0)) {
		stat = msg.getInt(0);
		if (stat < 0) {
			stat = 0;
		}
		setLedState((uint8_t) (stat % 8));
	}
}

void getKnobs(OSCMessage &msg) {

	OSCMessage msgKnobs("/knobs");

	uint32_t i;
	for (i = 0; i < 6; i++) {
		msgKnobs.add((int32_t) knobValues[i]);
	}

	msgKnobs.send(oscBuf);
	slip.sendMessage(oscBuf.buffer, oscBuf.length);
	msgKnobs.empty();
}

void shutdown(OSCMessage &msg) {

	int i;
	char progressStr[20];
	int len = 0;
	int progress = 0;

	// clear screen
	for (i = 0; i < 1024; i++) {
		pix_buf[i] = 0;
	}

	/*
	 len = sprintf(progressStr, "starting: %d %%", progress);
	 println_8(progressStr, len, 8, 52);
	 ssd1306_refresh();
	 */

	stopwatchStart();
	while (progress < 99) {
		if (stopwatchReport() > 1200) {	// Increased time for shutdown on Pi
			stopwatchStart();
			len = sprintf(progressStr, "Shutting down: %d %%", progress++);
			println_8(progressStr, len, 1, 21);
			ssd1306_refresh();
		}
	}
	// clear screen
	for (i = 0; i < 1024; i++) {
		pix_buf[i] = 0;
	}
	len = sprintf(progressStr, "Shutdown complete.");
	println_8(progressStr, len, 1, 21);
	len = sprintf(progressStr, "Safe to remove power.");
	println_8(progressStr, len, 1, 43);
	ssd1306_refresh();

	for (;;)
		;  // endless loop here
}

// end OSC callbacks

/// scan keys
void keyMuxSel(uint32_t sel) {
	if (sel == 0) {
		MUX_0
		;
	} else if (sel == 1) {
		MUX_1
		;
	} else if (sel == 2) {
		MUX_2
		;
	} else if (sel == 3) {
		MUX_3
		;
	} else if (sel == 4) {
		MUX_4
		;
	} else if (sel == 5) {
		MUX_5
		;
	} else if (sel == 6) {
		MUX_6
		;
	} else if (sel == 7) {
		MUX_7
		;
	} else {
		MUX_0
		;
	}
}

uint32_t scanKeys() {
	static uint32_t seqCount = 0;
	static uint32_t muxSelCount = 0;

	if (seqCount == 0) {
		keyMuxSel(muxSelCount);  // select the mux
		muxSelCount++;        	// and increment for next time through
		muxSelCount %= 8;
	}
	if (seqCount == 1) {
		// do nothing, wait for next conversion sequence since the muxes were just changed
	}
	if (seqCount == 2) {
		/* Changed Aux key pin (PS) */
		keyValuesRaw[0] = (GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_3)) ? 0 : 100; // the aux key
		keyValuesRaw[26] = (GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_9)) ? 0 : 100; // oct up
		keyValuesRaw[27] = (GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_10)) ? 0 : 100; // oct down
		keyValuesRaw[1 + muxSelCount] =
				(GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_2)) ? 0 : 100; //RegularConvData_Tab[3];
		keyValuesRaw[9 + muxSelCount] =
				(GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_1)) ? 0 : 100; //RegularConvData_Tab[4];
		keyValuesRaw[17 + muxSelCount] =
				(GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_0)) ? 0 : 100; //RegularConvData_Tab[5];
		keyValuesRaw[25] = 0;//(GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_1)) ? 0 : 100; // the foot switch can be implemented later
	}
	seqCount++;
	seqCount %= 3;
	return muxSelCount;
}

void remapKeys() {
	static uint32_t cycleCount = 0;

	// Key values all updated for PCB (PS)
	keyValues[cycleCount][0] = keyValuesRaw[0];
	keyValues[cycleCount][25] = keyValuesRaw[25];
	keyValues[cycleCount][26] = keyValuesRaw[26];
	keyValues[cycleCount][27] = keyValuesRaw[27];

	keyValues[cycleCount][1] = keyValuesRaw[18];
	keyValues[cycleCount][2] = keyValuesRaw[19];
	keyValues[cycleCount][3] = keyValuesRaw[20];
	keyValues[cycleCount][4] = keyValuesRaw[21];
	keyValues[cycleCount][5] = keyValuesRaw[22];
	keyValues[cycleCount][6] = keyValuesRaw[23];
	keyValues[cycleCount][7] = keyValuesRaw[24];
	keyValues[cycleCount][8] = keyValuesRaw[17];

	keyValues[cycleCount][9] = keyValuesRaw[10];
	keyValues[cycleCount][10] = keyValuesRaw[11];
	keyValues[cycleCount][11] = keyValuesRaw[12];
	keyValues[cycleCount][12] = keyValuesRaw[13];
	keyValues[cycleCount][13] = keyValuesRaw[14];
	keyValues[cycleCount][14] = keyValuesRaw[15];
	keyValues[cycleCount][15] = keyValuesRaw[16];
	keyValues[cycleCount][16] = keyValuesRaw[9];

	keyValues[cycleCount][17] = keyValuesRaw[2];
	keyValues[cycleCount][18] = keyValuesRaw[3];
	keyValues[cycleCount][19] = keyValuesRaw[4];
	keyValues[cycleCount][20] = keyValuesRaw[5];
	keyValues[cycleCount][21] = keyValuesRaw[6];
	keyValues[cycleCount][22] = keyValuesRaw[7];
	keyValues[cycleCount][23] = keyValuesRaw[8];
	keyValues[cycleCount][24] = keyValuesRaw[1];

	cycleCount++;
	cycleCount &= 0x3;  // i between 0-3
}



void checkForKeyEvent() {
	remapKeys();
	uint32_t i;

	for (i = 0; i < 28; i++) {
		if ((keyValues[0][i]) && (keyValues[1][i]) && (keyValues[2][i])
				&& (keyValues[3][i])) {

			if (!keyValuesLast[i]) {
				OSCMessage msgKey("/key");

				msgKey.add((int32_t) i);
				msgKey.add((int32_t) 100);

				msgKey.send(oscBuf);
				slip.sendMessage(oscBuf.buffer, oscBuf.length);

				msgKey.empty(); // free space occupied by message
				keyValuesLast[i] = 100;
			}
		}
		if ((!keyValues[0][i]) && (!keyValues[1][i]) && (!keyValues[2][i])
				&& (!keyValues[3][i])) {
			if (keyValuesLast[i]) {
				OSCMessage msgKey("/key");

				msgKey.add((int32_t) i);
				msgKey.add((int32_t) 0);

				msgKey.send(oscBuf);
				slip.sendMessage(oscBuf.buffer, oscBuf.length);

				msgKey.empty();
				keyValuesLast[i] = 0;
			}
		}
	}
}

void updateKeyStateNoOsc() {
	uint32_t i;
	remapKeys();
	for (i = 0; i < 28; i++) {
		if ((keyValues[0][i]) && (keyValues[1][i]) && (keyValues[2][i]) && (keyValues[3][i])) {
			keyValuesLast[i] = 100;
		}
		if ((!keyValues[0][i]) && (!keyValues[1][i]) && (!keyValues[2][i]) && (!keyValues[3][i])) {
			keyValuesLast[i] = 0;
		}
	}
}

/// end keys

void updateKnobs() {
	// Per-channel filtering + endpoint hysteresis to reduce jitter and make
	// full-scale positions "stick" at 0/1023.
	static uint16_t filt[6] = {0, 0, 0, 0, 0, 0};
	static uint8_t latch_low[6] = {0, 0, 0, 0, 0, 0};
	static uint8_t latch_high[6] = {0, 0, 0, 0, 0, 0};
	uint16_t raw[6];
	uint32_t i;

	// Hysteresis thresholds (10-bit ADC scale).
	const uint16_t HIGH_LOCK = 1016;
	const uint16_t HIGH_RELEASE = 1000;
	const uint16_t LOW_LOCK = 8;
	const uint16_t LOW_RELEASE = 20;

	// see if a new conversion is ready
	if ((DMA_GetFlagStatus(DMA1_FLAG_TC1)) == SET) {
		DMA_ClearFlag(DMA1_FLAG_TC1);

		raw[4] = (uint16_t) RegularConvData_Tab[0];
		raw[0] = (uint16_t) RegularConvData_Tab[1];
		raw[1] = (uint16_t) RegularConvData_Tab[2];
		raw[3] = (uint16_t) RegularConvData_Tab[3];
		raw[2] = (uint16_t) RegularConvData_Tab[4];
		raw[5] = (uint16_t) RegularConvData_Tab[5];

		for (i = 0; i < 6; i++) {
			uint16_t v = raw[i];

			// 1st-order low-pass filter: 75% old + 25% new.
			filt[i] = (uint16_t) (((uint32_t) filt[i] * 3u + (uint32_t) v) >> 2);
			v = filt[i];

			// Rail hysteresis for stable endpoints.
			if (latch_high[i]) {
				if (v <= HIGH_RELEASE) {
					latch_high[i] = 0;
				}
			} else if (v >= HIGH_LOCK) {
				latch_high[i] = 1;
				latch_low[i] = 0;
			}

			if (latch_low[i]) {
				if (v >= LOW_RELEASE) {
					latch_low[i] = 0;
				}
			} else if (v <= LOW_LOCK) {
				latch_low[i] = 1;
				latch_high[i] = 0;
			}

			if (latch_high[i]) {
				knobValues[i] = 1023;
			} else if (latch_low[i]) {
				knobValues[i] = 0;
			} else {
				knobValues[i] = v;
			}
		}
	}
}

// check for foot switch change
void checkFootSwitch (void) {
	static uint8_t foot_last = 0;
	static uint8_t foot_debounce[2] = {0, 0};
	static uint8_t foot_debounce_count = 0;

	if (knobValues[5] < 100) foot_debounce[foot_debounce_count] = 0;
	if (knobValues[5] > 900) foot_debounce[foot_debounce_count] = 1;

	foot_debounce_count++;
	foot_debounce_count &= 1;
	//only proceed if debounced (same for 2 times)
	if (foot_debounce[0] == foot_debounce[1]){
		if ((knobValues[5] < 100) && foot_last){
			foot_last = 0;
			g_foot_state = 0;
			// send press
			OSCMessage msgEncoder("/fs");
			msgEncoder.add(1);
			msgEncoder.send(oscBuf);
			slip.sendMessage(oscBuf.buffer, oscBuf.length);
		}
		if ((knobValues[5] > 900) && !foot_last){
			foot_last = 1;
			g_foot_state = 1;
			// send press
			OSCMessage msgEncoder("/fs");
			msgEncoder.add(0);
			msgEncoder.send(oscBuf);
			slip.sendMessage(oscBuf.buffer, oscBuf.length);
		}
	}
}

void checkEncoder(void) {
	// Robust quadrature decode:
	// - Uses Gray-code transition table to reject illegal/bouncy edges.
	// - Requires 4 quarter-steps before reporting one detent move.
	static uint8_t prev_ab = 0;
	static int8_t substep = 0;

	// because the encoder has a crappy switch, we need  a
	// different debounce time for press and release.
	// checkEncoder is called every ~5 ms in normal run loop.
	// Use shorter windows to avoid UI "hang" feeling on click.
	#define PRESS 0
	#define RELEASE 1
	const uint8_t ENC_BTN_PRESS_COUNTS = 6;    // ~30 ms
	const uint8_t ENC_BTN_RELEASE_COUNTS = 12; // ~60 ms
	// const uint8_t PRESS = 0;
	// const uint8_t RELEASE = 1;
	uint8_t button;
	static uint8_t button_last = RELEASE;
	static uint8_t press_count = 0;
	static uint8_t release_count = 0;
	uint8_t ab = 0;

	button = (GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_13));

	if (button == PRESS) {
		press_count++;
		release_count = 0;
	}
	if ((press_count > ENC_BTN_PRESS_COUNTS) && (button_last == RELEASE)){	// press
			button_last = PRESS;
			release_count = 0;
			g_encoder_button = 1;

			// send press
			OSCMessage msgEncoder("/encbut");
			msgEncoder.add(1);
			msgEncoder.send(oscBuf);
			slip.sendMessage(oscBuf.buffer, oscBuf.length);
	}

	if (button == RELEASE) {
		release_count++;
		press_count = 0;
	}
	if ((release_count > ENC_BTN_RELEASE_COUNTS) && (button_last == PRESS)){	// release
			button_last = RELEASE;
			press_count = 0;
			g_encoder_button = 0;

			// send release
			OSCMessage msgEncoder("/encbut");
			msgEncoder.add(0);
			msgEncoder.send(oscBuf);
			slip.sendMessage(oscBuf.buffer, oscBuf.length);
	}




	// turning
	ab = 0;
	if (GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_14))
		ab |= 0x1;
	if (GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_15))
		ab |= 0x2;

	{
		static const int8_t qdec[16] = {
			0, -1,  1,  0,
			1,  0,  0, -1,
			-1, 0,  0,  1,
			0,  1, -1,  0
		};
		uint8_t idx = (uint8_t) ((prev_ab << 2) | ab);
		int8_t step = qdec[idx];
		prev_ab = ab;

		if (step != 0) {
			substep += step;
			if (substep >= ENC_STEPS_PER_DETENT) {
				// Direction mapping: 1 = CW, 0 = CCW (or inverted by flag).
				OSCMessage msgEncoder("/enc");
				msgEncoder.add(ENCODER_INVERT ? 0 : 1);
				msgEncoder.send(oscBuf);
				slip.sendMessage(oscBuf.buffer, oscBuf.length);
				g_encoder_accum += ENCODER_INVERT ? -1 : 1;
				substep = 0;
			} else if (substep <= -ENC_STEPS_PER_DETENT) {
				OSCMessage msgEncoder("/enc");
				msgEncoder.add(ENCODER_INVERT ? 1 : 0);
				msgEncoder.send(oscBuf);
				slip.sendMessage(oscBuf.buffer, oscBuf.length);
				g_encoder_accum += ENCODER_INVERT ? 1 : -1;
				substep = 0;
			}
		}
	}
}


#pragma GCC diagnostic pop

// ----------------------------------------------------------------------------
