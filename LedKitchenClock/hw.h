/******************************************************************************
  Copyright (C) 2012 Jakub Chalupnik <jakubchalupnik@gmail.com>.

  This program is free software; you can redistribute it and/or
  modify it under the terms of the GNU General Public License
  as published by the Free Software Foundation; either version 2
  of the License, or (at your option) any later version.

  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with this program; if not, write to the Free Software Foundation,
  Inc., 59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.
******************************************************************************/

#ifndef _HW_H
#define _HW_H

//-------------------------------------------------------------------------
// Pin defines
//

#define LED_SDI_BIT     0
#define LED_SDI_PORT    PORTB
#define LED_SDI_DDR     DDRB

#define LED_CLK_BIT     7
#define LED_CLK_PORT    PORTD
#define LED_CLK_DDR     DDRD

#define LED_LE_BIT      6
#define LED_LE_PORT     PORTD
#define LED_LE_DDR      DDRD

#define LED_OE_BIT      5
#define LED_OE_PORT     PORTD
#define LED_OE_DDR      DDRD

#define LED_D1_BIT      3
#define LED_D1_PORT     PORTC
#define LED_D1_DDR      DDRC

#define LED_D2_BIT      2
#define LED_D2_PORT     PORTD
#define LED_D2_DDR      DDRD

#define LED_D3_BIT      3
#define LED_D3_PORT     PORTD
#define LED_D3_DDR      DDRD

#define LED_D4_BIT      0
#define LED_D4_PORT     PORTD
#define LED_D4_DDR      DDRD

#define LedBitSet(Bit)      LED_##Bit##_PORT |= (1 << LED_##Bit##_BIT)
#define LedBitClear(Bit)    LED_##Bit##_PORT &= ~(1 << LED_##Bit##_BIT)

#define LedBitOutput(Bit)   LED_##Bit##_DDR |= (1 << LED_##Bit##_BIT)

//-------------------------------------------------------------------------
// Frequencies, Baud rates...
#define F_CPU	16000000UL

//-------------------------------------------------------------------------
// HW UART defines


//-------------------------------------------------------------------------
// Macros

#endif // _HW_H
