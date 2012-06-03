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

#ifndef _LED_SEG_H
#define _LED_SEG_H

#define _A_SEG 7
#define _B_SEG 4
#define _C_SEG 1
#define _D_SEG 2
#define _E_SEG 0
#define _F_SEG 6
#define _G_SEG 5
#define _P_SEG 3

#define seg_mac(a,b,c,d,e,f,g,p) (((a << _A_SEG) | (b << _B_SEG) | (c << _C_SEG) | (d << _D_SEG) | (e << _E_SEG) | (f << _F_SEG) | (g << _G_SEG) | (p << _P_SEG)))

//----------------------------------------------------------------------------
// 7seg definitions

unsigned char seg_hex_table[] = {
    seg_mac(1, 1, 1, 1, 1, 1, 0, 0),             // 0
    seg_mac(0, 1, 1, 0, 0, 0, 0, 0),             // 1
    seg_mac(1, 1, 0, 1, 1, 0, 1, 0),             // 2
    seg_mac(1, 1, 1, 1, 0, 0, 1, 0),             // 3
    seg_mac(0, 1, 1, 0, 0, 1, 1, 0),             // 4
    seg_mac(1, 0, 1, 1, 0, 1, 1, 0),             // 5
    seg_mac(1, 0, 1, 1, 1, 1, 1, 0),             // 6
    seg_mac(1, 1, 1, 0, 0, 0, 0, 0),             // 7
    seg_mac(1, 1, 1, 1, 1, 1, 1, 0),             // 8
    seg_mac(1, 1, 1, 1, 0, 1, 1, 0),             // 9
    seg_mac(1, 1, 1, 0, 1, 1, 1, 0),             // A
    seg_mac(0, 0, 1, 1, 1, 1, 1, 0),             // B
    seg_mac(1, 0, 0, 1, 1, 1, 0, 0),             // C
    seg_mac(0, 1, 1, 1, 1, 0, 1, 0),             // D
    seg_mac(1, 0, 0, 1, 1, 1, 1, 0),             // E
    seg_mac(1, 0, 0, 0, 1, 1, 1, 0),             // F
};

#endif // _LED_SEG_H
