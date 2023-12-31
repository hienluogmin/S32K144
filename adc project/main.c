/*
 * Copyright (c) 2014 - 2016, Freescale Semiconductor, Inc.
 * Copyright (c) 2016 - 2018, NXP.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY NXP "AS IS" AND ANY EXPRESSED OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
 * IN NO EVENT SHALL NXP OR ITS CONTRIBUTORS BE LIABLE FOR ANY DIRECT,
 * INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING
 * IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF
 * THE POSSIBILITY OF SUCH DAMAGE.
 */

/*!
 * Description:	
 * ======================================================================
 * The ADC is initialized to convert two channels using software triggers 
 * that are configured for one-shot conversions. Each conversion requires 
 * its own software trigger. One channel (AD12) connects to a potentiometer 
 * on the EVB the other to VREFSH. The results are scaled 0 to 5000 mV.
 */

#include "device_registers.h" 
#include "clocks_and_modes.h"
#include "ADC.h"

#define PTD0 (0)   	/* Port D0: FRDM EVB output to blue LED 	*/
#define PTD15 (15)	/* Port D15: FRDM EVB output to red LED 	*/
#define PTD16 (16) 	/* Port D16: FRDM EVB output to green LED 	*/

void PORT_init (void)
{
	/*!
	 * Pins definitions
	 * ===================================================
	 *
	 * Pin number        | Function
	 * ----------------- |------------------
	 * PTD0              | GPIO [BLUE LED]
	 * PTD15             | GPIO [RED LED]
	 * PTD16			 | GPIO [GREEN LED]
	 */
  PCC->PCCn[PCC_PORTD_INDEX ]|=PCC_PCCn_CGC_MASK;   /* Enable clock for PORTD */
  PORTD->PCR[PTD0]  = PORT_PCR_MUX(1);	/* Port D0: MUX = GPIO  */
  PORTD->PCR[PTD15] = PORT_PCR_MUX(1);  /* Port D15: MUX = GPIO */
  PORTD->PCR[PTD16] = PORT_PCR_MUX(1);  /* Port D16: MUX = GPIO */

  PTD->PDDR |= 1<<PTD0		/* Port D0:  Data Direction = output */
  	  	  	  |1<<PTD15	    /* Port D15: Data Direction = output */
  	  	  	  |1<<PTD16;    /* Port D16: Data Direction = output */
}

void WDOG_disable (void)
{
  WDOG->CNT=0xD928C520;     /* Unlock watchdog 		*/
  WDOG->TOVAL=0x0000FFFF;   /* Maximum timeout value 	*/
  WDOG->CS = 0x00002100;    /* Disable watchdog 		*/
}

int main(void)  
{
  uint32_t adcResultInMv=0;	/*< ADC0 Result in miliVolts */

	/*!
	 * Initialization:
	 * =======================
	 */
  WDOG_disable();        /* Disable WDOG												*/
  SOSC_init_8MHz();      /* Initialize system oscillator for 8 MHz xtal 				*/
  SPLL_init_160MHz();    /* Initialize SPLL to 160 MHz with 8 MHz SOSC 				*/
  NormalRUNmode_80MHz(); /* Init clocks: 80 MHz sysclk & core, 40 MHz bus, 20 MHz flash*/
  PORT_init();		     /* Init  port clocks and gpio outputs 						*/
  ADC_init();            /* Init ADC resolution 12 bit									*/

	/*!
	 * Infinite for:
	 * ========================
	 */
	  for(;;)
	  {
		convertAdcChan(12);                   /* Convert Channel AD12 to pot on EVB 	*/
		while(adc_complete()==0){}            /* Wait for conversion complete flag 	*/
		adcResultInMv = read_adc_chx();       /* Get channel's conversion results in mv */

		if (adcResultInMv > 5000) {           /* If result > 5V 		*/
		  PTD->PTOR |= 1<<PTD0 | 1<<PTD15 | 1<<PTD16;    /* turn white LED */
		}
		else if (adcResultInMv >= 2500 && adcResultInMv < 5000) {      /* If result > 2.50V and result < 5V 		*/
		  PTD->PSOR |= 1<<PTD0 | 1<<PTD15;    /* turn off blue, red LEDs 	*/
		  PTD->PTOR |= 1<<PTD16;     	      /* turn on green LED 		*/
		}
		else if (adcResultInMv > 0 && adcResultInMv < 2500) {       /* If result > 0V and result < 2.5V 		*/
		  PTD->PSOR |= 1<<PTD15 | 1<<PTD16;   /* turn off red, green LEDs  */
		  PTD->PTOR |= 1<<PTD0;     	      /* turn on blue LED 			*/
		}
		else {
		  PTD->PSOR |= 1<<PTD0 | 1<<PTD16;   /* turn off red, green LEDs  */
		  PTD->PTOR |= 1<<PTD15;     	      /* turn on red LED*/
		}

		convertAdcChan(29);                   /* Convert chan 29, Vrefsh 			*/
		while(adc_complete()==0){}            /* Wait for conversion complete flag */
		adcResultInMv = read_adc_chx();       /* Get channel's conversion results in mv */
	  }
}
