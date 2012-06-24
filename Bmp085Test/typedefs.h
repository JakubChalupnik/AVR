//---------------------------------------------------------------------------
//
// FILENAME.
//      Typedef.h - standard type definitions.
//
// FUNCTIONAL DESCRIPTION.
//      Standard type definitions
//
// PROCESSOR.
//      Atmel AVR
//
// LICENSE.
//      Copyright 2012 Jakub Chalupnik (JakubChalupnik@gmail.com). All rights reserved.
//      (see license.txt)
//
//---------------------------------------------------------------------------

#ifndef _TYPEDEFS_H
#define _TYPEDEFS_H

typedef unsigned char uint8_t;
typedef unsigned int uint16_t;
typedef unsigned long int uint32_t;

#define byte uint8_t
#define word uint16_t
#define dword uint32_t

#define bool_t 	unsigned char
#define false 	(1 == 0)
#define true 	(1 == 1)

#endif
