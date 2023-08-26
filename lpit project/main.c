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
 * ==================================================================================================
 * This project provides common initialization for clocks and an LPIT channel counter function.
 * Core clock is set to 80 MHz. LPIT0 channel 0 is configured to count one second of SPLL clocks.
 * Software polls the channel’s timeout flag and toggles the GPIO output to the LED when the flag sets.
 */
<<<<<<< HEAD

=======
<<<<<<< HEAD

=======
#include <intrinsics.h>
>>>>>>> 5103828 (lpit project)
>>>>>>> 9fbe046 (LPIT project)
#include "device_registers.h"            /* include peripheral declarations S32K144 */
#include "clocks_and_modes.h"

int lpit0_ch0_flag_counter = 0; /*< LPIT0 timeout counter */
<<<<<<< HEAD
=======
<<<<<<< HEAD
>>>>>>> 9fbe046 (LPIT project)
int counter = 0;
void ConfigureClockPTD14 (int div)
{
  SIM -> CHIPCTL &= ~(1<<11);//Disable Clock
  SIM -> CHIPCTL &= ~(0x7<<8);//Clear CLKOUTSEL bits 
  SIM -> CHIPCTL |= SIM_CHIPCTL_CLKOUTSEL(2);//Select SOSC DIV2 as CLKOUT source
  SIM -> CHIPCTL |= SIM_CHIPCTL_CLKOUTDIV(div);//Set CLKOUT driver
  SIM -> CHIPCTL |= (1<<11);//Enable Clock
<<<<<<< HEAD
=======
=======
int idle_counter = 0;
void LPIT_Ch0_IRQHandler (void)
{
  LPIT0 -> MSR |= LPIT_MSR_TIF0_MASK;
  idle_counter++;
}

void delay(int ms)
{
  lpit0_ch0_flag_counter = 0;
  uint32_t thoi_gian_ban_dau = lpit0_ch0_flag_counter;
  uint32_t thoi_gian_cuoi_cung = thoi_gian_ban_dau + (ms*40000000);
  while(lpit0_ch0_flag_counter < thoi_gian_cuoi_cung)
  {}
>>>>>>> 5103828 (lpit project)
>>>>>>> 9fbe046 (LPIT project)
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
	 */
  PCC-> PCCn[PCC_PORTD_INDEX] = PCC_PCCn_CGC_MASK; /* Enable clock for PORT D */
  PTD->PDDR |= 1<<0;            		/* Port D0:  Data Direction= output */
  PORTD->PCR[0] |=  PORT_PCR_MUX(1);  	/* Port D0:  MUX = ALT1, GPIO (to blue LED on EVB) */
<<<<<<< HEAD
=======
<<<<<<< HEAD
>>>>>>> 9fbe046 (LPIT project)
  PTD->PDDR |= 1<<15;            		/* Port D15:  Data Direction= output */
  PORTD->PCR[15] |=  PORT_PCR_MUX(1);  	/* Port D0:  MUX = ALT1, GPIO (to red LED on EVB) */
  PTD->PDDR |= 1<<16;            		/* Port D16:  Data Direction= output */
  PORTD->PCR[16] |=  PORT_PCR_MUX(1);  	/* Port D0:  MUX = ALT1, GPIO (to green LED on EVB) */
          /*!---Button---!*/
  PCC-> PCCn[PCC_PORTC_INDEX] = PCC_PCCn_CGC_MASK; /* Enable clock for PORT D */
  PTC->PDDR &= ~ (1<<12);            		/* Port C12:  Data Direction= input */
  PORTC->PCR[12] |= PORT_PCR_MUX(1);/* Port C12:  MUX = ALT1, GPIO (to switch button on EVB) */
  PTC->PDDR &= ~ (1<<13);            		/* Port C13:  Data Direction= input */
  PORTC->PCR[13] |= PORT_PCR_MUX(1);/* Port C13:  MUX = ALT1, GPIO (to switch button on EVB) */
          /*!---CLK_OUT---!*/
  PTD->PDDR |= 1<<14;            		/* Port D14:  Data Direction= output */
  PORTD->PCR[14] |= PORT_PCR_MUX(7);/* Port D14:  MUX = ALT7, (CLK_OUT) */
}

void BUTTON_init_IRQs(void)
{
  /*!---Configure Interrupt for button pin---!*/
  PORTC->PCR[12] |= PORT_PCR_IRQC(9);/*SF flag and Interrupt on rising edge */
  PORTC->PCR[13] |= PORT_PCR_IRQC(12);/*SF flag and Interrupt when logic 1 */
  S32_NVIC->ICPR[PORTC_IRQn/32] |= (1<<(PORTC_IRQn%32));/*Clear any pending interrupt */
  S32_NVIC->ISER[PORTC_IRQn/32] |= (1<<(PORTC_IRQn%32));/*Enable the interrupt */
  S32_NVIC->IP[PORTC_IRQn] = 1;/*Set the interrupt priority*/
}


<<<<<<< HEAD
=======
=======
}

>>>>>>> 5103828 (lpit project)
>>>>>>> 9fbe046 (LPIT project)
void LPIT0_init (void)
{
	/*!
	 * LPIT Clocking:
	 * ==============================
	 */
  PCC->PCCn[PCC_LPIT_INDEX] = PCC_PCCn_PCS(6);    /* Clock Src = 6 (SPLL2_DIV2_CLK)*/
  PCC->PCCn[PCC_LPIT_INDEX] |= PCC_PCCn_CGC_MASK; /* Enable clk to LPIT0 regs 		*/

  /*!
   * LPIT Initialization:
   */
  LPIT0->MCR |= LPIT_MCR_M_CEN_MASK;  /* DBG_EN-0: Timer chans stop in Debug mode */
                              	  	  /* DOZE_EN=0: Timer chans are stopped in DOZE mode */
                              	  	  /* SW_RST=0: SW reset does not reset timer chans, regs */
                              	  	  /* M_CEN=1: enable module clk (allows writing other LPIT0 regs) */
  LPIT0->TMR[0].TVAL = 40000000;      /* Chan 0 Timeout period: 40M clocks */

  LPIT0->TMR[0].TCTRL |= LPIT_TMR_TCTRL_T_EN_MASK;
  	  	  	  	  	  	  	  /* T_EN=1: Timer channel is enabled */
                              /* CHAIN=0: channel chaining is disabled */
                              /* MODE=0: 32 periodic counter mode */
                              /* TSOT=0: Timer decrements immediately based on restart */
                              /* TSOI=0: Timer does not stop after timeout */
                              /* TROT=0 Timer will not reload on trigger */
                              /* TRG_SRC=0: External trigger soruce */
                              /* TRG_SEL=0: Timer chan 0 trigger source is selected*/
}

void WDOG_disable (void)
{
  WDOG->CNT=0xD928C520;     /* Unlock watchdog 		*/
  WDOG->TOVAL=0x0000FFFF;   /* Maximum timeout value 	*/
  WDOG->CS = 0x00002100;    /* Disable watchdog 		*/
}

<<<<<<< HEAD
=======
<<<<<<< HEAD
>>>>>>> 9fbe046 (LPIT project)
void PORTC_IRQHandler(void)
{
    // Check if interrupts are triggered from pins 12 or 13 of PORTC
    if (PORTC->ISFR & ((1 << 12) | (1 << 13)))
    {
        // Increment the value of the counter
        counter++;

        // Limit the counter value within the range of 1 to 3
        if (counter > 3)
        {
            counter = 1;
        }

        // Configure Clock for PTD14 based on the value of the counter
        ConfigureClockPTD14(counter == 1 ? 3 : (counter == 2 ? 7 : 1));

        // Perform actions on PTD pins depending on the counter value
        if (counter == 1)
        {
            PTD->PTOR |= 1 << 0; // Toggle PTD0 pin
        }
        else if (counter == 2)
        {
            PTD->PTOR |= 1 << 15; // Toggle PTD15 pin
        }
        else if (counter == 3)
        {
            PTD->PTOR |= 1 << 16; // Toggle PTD16 pin
        }

        // Clear interrupt flags from pins 12 and 13 of PORTC
        PORTC->ISFR |= (1 << 12) | (1 << 13);
    }
}

<<<<<<< HEAD
=======
=======
>>>>>>> 5103828 (lpit project)
>>>>>>> 9fbe046 (LPIT project)
int main(void)
{
	/*!
	 * Initialization:
	 * =======================
	 */
  WDOG_disable();		  /* Disable WDOG */
  PORT_init();            /* Configure ports */
  SOSC_init_8MHz();       /* Initialize system oscilator for 8 MHz xtal */
  SPLL_init_160MHz();     /* Initialize SPLL to 160 MHz with 8 MHz SOSC */
  NormalRUNmode_80MHz();  /* Init clocks: 80 MHz sysclk & core, 40 MHz bus, 20 MHz flash */
<<<<<<< HEAD
  ConfigureClockPTD14(7);
  BUTTON_init_IRQs();
=======
<<<<<<< HEAD
  ConfigureClockPTD14(7);
  BUTTON_init_IRQs();
=======
  LPIT0_init();           /* Initialize PIT0 for 1 second timeout  */
>>>>>>> 5103828 (lpit project)
>>>>>>> 9fbe046 (LPIT project)

	/*!
	 * Infinite for:
	 * ========================
	 */
	  for (;;)
<<<<<<< HEAD
	  {

=======
<<<<<<< HEAD
	  {

=======
	  {     
                idle_counter = 0;
		  /* Toggle output to LED every LPIT0 timeout */
		while (idle_counter < 2) {} /* Wait for LPIT0 CH0 Flag */
		lpit0_ch0_flag_counter++;         /* Increment LPIT0 timeout counter */
		PTD->PTOR |= 1<<0;                /* Toggle output on port D0 (blue LED) */
		LPIT0->MSR |= LPIT_MSR_TIF0_MASK; /* Clear LPIT0 timer flag 0 */
>>>>>>> 5103828 (lpit project)
>>>>>>> 9fbe046 (LPIT project)
	  }
}
