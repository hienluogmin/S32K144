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
 * ============================================================================================================
 * A FlexCAN module is initialized for 500 KHz (2 usec period) bit time based on an 8 MHz crystal. 
 * Message buffer 0 transmits 8 byte messages and message buffer 4 can receive 8 byte messages.
 * To enable signals to the CAN bus, the SBC must be powered with external 12 V. EVBs with SBC
 * MC33903 require CAN transceiver configuration with SPI. EVBs with SBC UJA1169 do not require configuration.
 * 
 * By default, the example configures for SBC MC33903 and “Node A” IDs. If a second EVB or CAN tool
 * is available, “Node B” IDs can be used for the tool or second EVB. FlexCAN.h file contains controls for
 * initializing Node A vs B and selecting SBC MC33903.
 */

#include "device_registers.h" /* include peripheral declarations S32K144 */
#include "FlexCAN.h"
#include "clocks_and_modes.h"
#include "ADC.h"

#define PTD0 (0)
#define PTD15 (15)
#define PTD16 (16)
#define PTE4 (4)
#define PTE5 (5)

int idle_counter = 0;           /*< main loop idle counter */
int lpit0_ch0_flag_counter = 0; /*< LPIT0 chan 0 timeout counter */ 
uint32_t global_data1;
uint32_t prescale;
extern uint32_t global_data;
extern uint32_t global_id;
void WDOG_disable (void)
{
  WDOG->CNT=0xD928C520;     /* Unlock watchdog 		*/
  WDOG->TOVAL=0x0000FFFF;   /* Maximum timeout value 	*/
  WDOG->CS = 0x00002100;    /* Disable watchdog 		*/
}

void NVIC_init_IRQs (void)
{
	S32_NVIC->ICPR[1] = 1 << (48 % 32);  /* IRQ48-LPIT0 ch0: clr any pending IRQ*/
	S32_NVIC->ISER[1] = 1 << (48 % 32);  /* IRQ48-LPIT0 ch0: enable IRQ */
	S32_NVIC->IP[48] = 0xA;              /* IRQ48-LPIT0 ch0: priority 10 of 0-15*/
}

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
   * PTD16             | GPIO [GREEN LED]
   * PTE4              | CAN0_RX
   * PTE5              | CAN0_TX
   */
  /* SETTING LED */
  PCC->PCCn[PCC_PORTD_INDEX]|=PCC_PCCn_CGC_MASK;   /* Enable clock for PORTD */
  PORTD->PCR[PTD0]  = PORT_PCR_MUX(1);  /* Port D0: MUX = GPIO  */
  PORTD->PCR[PTD15] = PORT_PCR_MUX(1);  /* Port D15: MUX = GPIO */
  PORTD->PCR[PTD16] = PORT_PCR_MUX(1);  /* Port D16: MUX = GPIO */
  PTD->PDDR |= 1<<PTD0  /* Port D0:  Data Direction = output */
            |1<<PTD15 /* Port D15: Data Direction = output */
            |1<<PTD16;  /* Port D16: Data Direction = output */
  /* SETTING CAN0 */
  PCC->PCCn[PCC_PORTE_INDEX]|=PCC_PCCn_CGC_MASK; /* Enable clock for PORTE */
  PORTE->PCR[PTE4] = PORT_PCR_MUX(5); /* Port E4: MUX = ALT5, CAN0_RX */
  PORTE->PCR[PTE5] = PORT_PCR_MUX(5); /* Port E5: MUX = ALT5, CAN0_TX */
}

void LPIT0_init(void)
{
  PCC->PCCn[PCC_LPIT_INDEX] = PCC_PCCn_PCS(6);    /* Clock Src = 6 (SPLL2_DIV2_CLK)*/
  PCC->PCCn[PCC_LPIT_INDEX] |= PCC_PCCn_CGC_MASK; /* Enable clk to LPIT0 regs       */

  LPIT0->MCR |= LPIT_MCR_M_CEN_MASK;  /* DBG_EN-0: Timer chans stop in Debug mode */
  //LPIT0->MCR |= (1<<0);                 
  LPIT0->MIER = LPIT_MIER_TIE0_MASK;  /* TIE0=1: Timer Interrupt Enabled fot Chan 0 */
  //LPIT0->MIER = 1<<0;
  LPIT0->TMR[0].TVAL = 40000000;      /* Chan 0 Timeout period: 40M clocks */
  LPIT0->TMR[0].TCTRL |= LPIT_TMR_TCTRL_T_EN_MASK;
  //LPIT0->TMR[0].TCTRL = 1<<0;       /* T_EN=1: Timer channel is enabled */
}

void delay(int seconds)
{
    lpit0_ch0_flag_counter = 0;               /* Reset flag counter */
    uint32_t tval = seconds * 40000000;       /* Adjust for 40 MHz clocks */
    LPIT0->TMR[0].TVAL = tval;
    while (lpit0_ch0_flag_counter == 0)
    {
        // waiting for flag
    }

    lpit0_ch0_flag_counter = 0; // Reset flag counter
}

int main(void)  
{
  uint32_t adcResultInMv=0; /*< ADC0 Result in miliVolts */

  /*!
   * Initialization:
   * =======================
   */
  WDOG_disable();        /* Disable WDOG                        */
  SOSC_init_8MHz();      /* Initialize system oscillator for 8 MHz xtal         */
  SPLL_init_160MHz();    /* Initialize SPLL to 160 MHz with 8 MHz SOSC        */
  NormalRUNmode_80MHz(); /* Init clocks: 80 MHz sysclk & core, 40 MHz bus, 20 MHz flash*/
  PORT_init();         /* Init  port clocks and gpio outputs            */
  ADC_init();            /* Init ADC resolution 12 bit                  */
  NVIC_init_IRQs();       /* Enable desired interrupts and priorities */
  LPIT0_init();           /* Initialize PIT0 for 1 second timeout  */
  FLEXCAN0_init();         /* Init FlexCAN0 */
#ifdef NODE_A              /* Node A transmits first; Node B transmits after reception */
  send_string(0x00,0xA000); /* Transmit initial message from Node A to Node B */
#endif
  /*!
   * Infinite for:
   * ========================
   */
      for (;;)
{
    convertAdcChan(12);
    while (adc_complete() == 0) {}
    adcResultInMv = read_adc_chx();
    if (global_data1 == 0x05000000)
     {
      PTD->PSOR |= (1 << PTD15) | (1 << PTD16) | (1 << PTD0);
      send_string(0x05000000, 0x02C00000);
      FLEXCAN0_receive_msg();
     }
    else
     {                    
      if (prescale != adcResultInMv)
     {

    if (adcResultInMv >= 5000) {
        PTD->PTOR |= 1 << PTD0 | 1 << PTD15 | 1 << PTD16; // Turn on white LED
        send_string(0x04, 0xA000);
        FLEXCAN0_receive_msg();
    }
    else if (adcResultInMv >= 2500) {
        PTD->PSOR |= 1 << PTD0 | 1 << PTD15; // Turn off blue, red LEDs
        PTD->PTOR |= 1 << PTD16; // Turn on green LED
        delay(1);
        send_string(0x03, 0xA000);
        FLEXCAN0_receive_msg();
    }
    else if (adcResultInMv > 0) {
        PTD->PSOR |= 1 << PTD15 | 1 << PTD16; // Turn off red, green LEDs
        PTD->PTOR |= 1 << PTD0; // Turn on blue LED
        delay(1);
        send_string(0x02, 0xA000);
        FLEXCAN0_receive_msg();
    }
    else {
        PTD->PSOR |= 1 << PTD0 | 1 << PTD16; // Turn off red, green LEDs
        PTD->PTOR |= 1 << PTD15; // Turn on RED LED
        send_string(0x01, 0xA000);
        FLEXCAN0_receive_msg();
    }

    convertAdcChan(29);
    while (adc_complete() == 0) {}
    adcResultInMv = read_adc_chx();
      }
    }
  }
}
void LPIT0_Ch0_IRQHandler(void)
{
  lpit0_ch0_flag_counter++;                /* Increment LPIT0 timeout counter */
  LPIT0->MSR |= LPIT_MSR_TIF0_MASK;         /* Clear LPIT0 timer flag 0 */
  //__DSB();
}

