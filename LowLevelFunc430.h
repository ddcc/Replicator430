/*
 * Copyright (C) 2016 Texas Instruments Incorporated - http://www.ti.com/ 
 * 
 *  Redistribution and use in source and binary forms, with or without 
 *  modification, are permitted provided that the following conditions 
 *  are met:
 *
 *    Redistributions of source code must retain the above copyright 
 *    notice, this list of conditions and the following disclaimer.
 *
 *    Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the 
 *    documentation and/or other materials provided with the   
 *    distribution.
 *
 *    Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS 
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT 
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 *  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT 
 *  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, 
 *  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT 
 *  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 *  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 *  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT 
 *  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE 
 *  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
*/
/*==========================================================================*\
|                                                                            |
| LowLevelFunc430.h                                                          |
|                                                                            |
| Low Level function prototypes, macros, and pin-to-signal assignments       |
| regarding to user's hardware                                               |
|----------------------------------------------------------------------------|
| Project:              MSP430 Replicator                                    |
| Developed using:      IAR Embedded Workbench 6.20                          |
|             and:      Code Composer Studio 6.0                             |
|----------------------------------------------------------------------------|
| Version history:                                                           |
| 1.0 04/02 FRGR        Initial version.                                     |
| 1.1 04/02 FRGR        Included SPI mode to speed up shifting function by 2.|
|                       (JTAG control now on Port5)                          |
| 1.2 06/02 ALB2        Formatting changes, added comments. Removed code used|
|                       for debug purposes during development.               |
| 1.3 08/02 ALB2        Initial code release with Lit# SLAA149.              |
| 1.4 09/05 SUN1        Software delays redesigned to use TimerA harware;    |
|                       see MsDelay() routine. Added TA constant.            |
| 1.5 12/05 STO         Added RESET pin definition                           |
| 1.6 08/08 WLUT        Added DrvSignals and RlsSignals macros for clean     |
|                       JTAG init sequence.                                  |
| 1.7 05/09 GC (Elprotronic)  Added support for the new hardware - REP430F   |
| 1.8 07/09 FB          Added macros for Spy-Bi-Wire support                 |
| 1.9 05/12 RL          Updated commentaries                                 |
|----------------------------------------------------------------------------|
| Designed 2002 by Texas Instruments Germany                                 |
\*==========================================================================*/
//! \file LowLevelFunc430.h
//! \brief Low Level function prototypes, macros, and pin-to-signal assignments regarding to user's hardware
/****************************************************************************/
/* INCLUDES                                                                 */
/****************************************************************************/

#include "Config430.h"        // High-level user input
#include <msp430fr5739.h>

/****************************************************************************/
/* DEFINES & CONSTANTS                                                      */
/****************************************************************************/

//! \brief JTAG interface
#define JTAG_IF             1
//! \brief JTAG interface on a device that supports JTAG and SBW
#define SPYBIWIREJTAG_IF    2
//! \brief Spy-Bi-Wire interface
#define SPYBIWIRE_IF        3

#if( INTERFACE == SPYBIWIRE_IF )
#define SPYBIWIRE_MODE 
#endif

#ifndef __DATAFORMATS__
#define __DATAFORMATS__
#define F_BYTE                     8
#define F_WORD                     16
#define F_ADDR                     20
#define F_LONG                     32
#endif

// Constants for runoff status
//! \brief return 0 = error
#define STATUS_ERROR     0      // false
//! \brief return 1 = no error
#define STATUS_OK        1      // true
//! \brief GetDevice returns this if the security fuse is blown
#define STATUS_FUSEBLOWN 2

//! \brief Replicator is active
#define STATUS_ACTIVE    2
//! \brief Replicator is idling
#define STATUS_IDLE      3

/****************************************************************************/
/* Macros and Pin-to-Signal assignments which have to be programmed         */
/* by the user. This implementation assumes use of an MSP430F5437 as the    */
/* host controller and the corresponding hardware given in the MSP430       */
/* Programming Via the JTAG Interface User's Guide (SLAU320).               */
/*                                                                          */
/* The following MSP430 example acts as a hint of how to generally          */
/* implement a micro-controller programmer solution for the MSP430 flash-   */
/* based devices.                                                           */
/****************************************************************************/

#define SW_OUT          P4OUT
#define SW_IN           P4IN
#define SW_DIR          P4DIR
#define SW_PULLUP       P4REN
//! \brief LED output register
#define LED_OUT         P3OUT
//! \brief LED direction register     
#define LED_DIR         P3DIR

//! \brief Mode-0 switch, P4.0
#define SW_MODE0        0x01
//! \brief Mode-1 switch, P4.1
#define SW_MODE1        0x02
//! \brief Switch-Vpp Enable - test and set/clr
//#define SW_VPPEN        0x04
//! \brief YELLOW LED   P3.6, LED6
#define LED_YELLOW      0x40
//! \brief GREEN LED    P3.5, LED5
#define LED_GREEN       0x20
//! \brief RED LED      P3.4, LED4
#define LED_RED         0x10
//! \brief SW-1 TEST
//#define SW_1            0x40

//#define spareP17        0x80

//! \brief VCC output register
//#define TVCC_OUT        P6OUT
//! \brief VCC direction register
//#define TVCC_DIR        P6DIR
//! \brief Minimum VCC value
//#define TVCC_MASK       0xF0
//! \brief Value to shift up voltage level
//#define TVCC_SHIFT      4

//#define TVCC_EN_DIR     P5DIR
//#define TVCC_EN_OUT     P5OUT
//#define TVCC_DIS_BIT    1

//! \brief Switch on yellow LED
#define LED_yellow_on()     ( LED_OUT |= LED_YELLOW )
//! \brief Switch off yellow LED
#define LED_yellow_off()    ( LED_OUT &= ~LED_YELLOW )
//! \brief Switch on red LED
#define LED_red_on()        ( LED_OUT |= LED_RED )
//! \brief Switch off red LED
#define LED_red_off()       ( LED_OUT &= ~LED_RED )
//! \brief Switch on green LED
#define LED_green_on()      ( LED_OUT |= LED_GREEN )
//! \brief Switch off green LED
#define LED_green_off()     ( LED_OUT &= ~LED_GREEN )
//! \brief Switch off all LEDs
#define All_LEDs_off()      ( LED_OUT &= ~(LED_GREEN | LED_RED | LED_YELLOW ))

// JTAG ports are P1.x
//! \brief JTAG output register
#define JTAGOUT         P1OUT
//! \brief JTAG input register
#define JTAGIN          P1IN
//! \brief JTAG direction register
#define JTAGDIR         P1DIR
//! \brief JTAG select register
#define JTAGSEL0         P1SEL0
#define JTAGSEL1         P1SEL1
//! \brief P1.3 JTAG TMS input pin
#define TMS             0x08
//! \brief P1.5 JTAG TDI input pin
#define TDI             0x20
//! \brief P1.6 JTAG TDO output pin
#define TDO             0x40
//! \brief P1.2 JTAG TCK input pin
#define TCK             0x04
//! \brief P1.0 JTAG Test input pin
#define TEST            0x01
//! \brief P1.1 Hardware RESET input pin
#define RST             0x02
//! \brief P1.5 TDI (former XOUT) receives TCLK
#define TCLK            TDI

//! \brief BSL output register
//! \details Not used in this project
#define BSLOUT          P3OUT
//! \brief BSL input register
//! \details Not used in this project
#define BSLIN           P3IN
//! \brief BSL direction register
//! \details Not used in this project
#define BSLDIR          P3DIR
//! \brief BSL select register
//! \details Not used in this project
#define BSLSEL          P3SEL
//! \brief P3.5 BSL-TX output pin
//! \details Not used in this project
#define BSLTX           0x20
//! \brief P3.4 BSL-RX input pin
//! \details Not used in this project
#define BSLRX           0x10

// VPP ports are P8.x
//! \brief Fuse blow voltage (Vpp) output register
//#define VPPOUT          P8OUT
//! \brief Fuse blow voltage (Vpp) direction register   
//#define VPPDIR          P8DIR
//! \brief Fuse blow voltage (Vpp) select register
//#define VPPSEL          P8SEL

//! \brief P8.2 Fuse blow voltage switched to TEST
#define VPPONTEST       0x04
//! \brief P8.1 Fuse blow voltage switched to TDI
#define VPPONTDI        0x02

// Constants for Setting up Timer A
//! \brief CCR0 delay for 1ms with a 1.5 MHz TA clock
#define ONEMS           0x05DC

/****************************************************************************/
/* Macros for processing the JTAG port                                      */
/****************************************************************************/

//! \brief JTAG macro: clear TMS signal
#define ClrTMS()    ((JTAGOUT) &= (~TMS))
//! \brief JTAG macro: set TMS signal
#define SetTMS()    ((JTAGOUT) |= (TMS))
//! \brief JTAG macro: clear TDI signal
#define ClrTDI()    ((JTAGOUT) &= (~TDI))
//! \brief JTAG macro: set TDI signal
#define SetTDI()    ((JTAGOUT) |= (TDI))
//! \brief JTAG macro: clear TCK signal
#define ClrTCK()    ((JTAGOUT) &= (~TCK))
//! \brief JTAG macro: set TCK signal
#define SetTCK()    ((JTAGOUT) |= (TCK))
//! \brief JTAG macro: return current TCLK signal (on TDI pin)
#define StoreTCLK() ((JTAGOUT  &   TCLK))
//! \brief JTAG macro: restore TCLK signal on TDI pin (based on input: x)
#define RestoreTCLK(x)  (x == 0 ? (JTAGOUT &= ~TCLK) : (JTAGOUT |= TCLK))
//! \brief JTAG macro: return TDO value (result 0 or TDO (0x40))
#define ScanTDO()   ((JTAGIN   &   TDO))
//! \brief Supply fuse blow voltage based on TEST or TDI pin (based on input: x(
#define VPPon(x)         (x == VPP_ON_TEST ? (SetVpp(VPPONTEST)) : (SetVpp(VPPONTDI)))
//! \brief Switch off fuse blow voltage
#define VPPoff()         (SetVpp(0))
//! \brief JTAG macro: set RST signal
#define SetRST()    ((JTAGOUT) |= (RST))
//! \brief JTAG macro: clear RST signal
#define ClrRST()    ((JTAGOUT) &= (~RST))
//! \brief JTAG macro: release RST pin
#define ReleaseRST() ( RST_dir( 0 )) 
//! \brief JTAG macro: set TST signal
#define SetTST()    ((JTAGOUT) |= (TEST))
//! \brief JTAG macro: clear TST signal
#define ClrTST()    ((JTAGOUT) &= (~TEST))

/****************************************************************************/
/* Macros to control Spy-Bi-Wire-IF                                         */
/****************************************************************************/
#define SBW_DELAY   { _NOP();_NOP();_NOP();_NOP();_NOP();}
   
//! \brief JTAG data_out pin in SBW mode -separate pin in MSP430F5437 - common IO translator
#define   SBWDATO   TDI
//! \brief JTAG data in pin in SBW mode - separate pin in MSP430F5437 - common IO translator
#define   SBWDATI   TDO
//! \brief JTAG clock pin in SBW mode
#define   SBWCLK    TCK

//! \brief SBW macro: set TMS signal
#define   TMSH    JTAGOUT |= SBWDATO;  SBW_DELAY; JTAGOUT &= ~SBWCLK; SBW_DELAY; JTAGOUT |= SBWCLK;
//! \brief SBW macro: clear TMS signal
#define   TMSL    JTAGOUT &= ~SBWDATO; SBW_DELAY; JTAGOUT &= ~SBWCLK; SBW_DELAY; JTAGOUT |= SBWCLK;
//! \brief SBW macro: clear TMS signal and immediately set it high again in
//! the SBWTCK low phase to enter the TDI slot with a high signal 
//! \details Only used to clock TCLK (=TDI for SBW) in Run-Test/IDLE mode of
//! the JTAG FSM 
#define   TMSLDH  JTAGOUT &= ~SBWDATO;  SBW_DELAY; JTAGOUT &= ~SBWCLK; SBW_DELAY; JTAGOUT |= SBWDATO; JTAGOUT |= SBWCLK;
//! \brief SBW macro: Set TDI = 1
#define   TDIH    JTAGOUT |= SBWDATO;   SBW_DELAY; JTAGOUT &= ~SBWCLK; SBW_DELAY; JTAGOUT |= SBWCLK;
//! \brief SBW macro: clear TDI signal
#define   TDIL    JTAGOUT &= ~SBWDATO;  SBW_DELAY; JTAGOUT &= ~SBWCLK; SBW_DELAY; JTAGOUT |= SBWCLK;
//! \brief SBW macro: TDO cycle without reading TDO
#define   TDOsbw  TRSLDIR |= TDOI_DIR;  SBW_DELAY; JTAGOUT &= ~SBWCLK; SBW_DELAY; JTAGOUT |= SBWCLK; TRSLDIR &= ~TDOI_DIR;
//! \brief SBW macro: TDO cycle with TDO read
#define   TDO_RD  TRSLDIR |= TDOI_DIR;  SBW_DELAY; JTAGOUT &= ~SBWCLK; SBW_DELAY; tdo_bit = JTAGIN; SBW_DELAY; JTAGOUT |= SBWCLK; TRSLDIR &= ~TDOI_DIR;

//! \brief SBW macro: set TCK signal
#define   SetSBWTCK()     (JTAGOUT |=   SBWCLK)
//! \brief SBW macro: clear TCK signal
#define   ClrSBWTCK()     (JTAGOUT &=  ~SBWCLK)
//! \brief SBW macro: set TDIO signal
#define   SetSBWTDIO()    (JTAGOUT |=   SBWDATO)
//! \brief SBW macro: clear TDIO signal
#define   ClrSBWTDIO()    (JTAGOUT &=  ~SBWDATO)

#if ( INTERFACE == SPYBIWIRE_IF )

void ClrTCLK_sbw(void);
void SetTCLK_sbw(void);
//! \brief SBW macro: clear TCLK signal
#define ClrTCLK()  ClrTCLK_sbw()
//! \brief SBW macro: set TCLK signal
#define SetTCLK()  SetTCLK_sbw()

#else

//! \brief clear the TCLK signal
#define ClrTCLK()  ((JTAGOUT) &= ~(TCLK))
//! \brief set the TCLK signal
#define SetTCLK()  ((JTAGOUT) |=  (TCLK))
 
#endif

/*----------------------------------------------------------------------------
   Definition of global variables
*/
extern byte TCLK_saved;      // holds the last value of TCLK before entering a JTAG sequence

/****************************************************************************/
/* TYPEDEFS                                                                 */
/****************************************************************************/

#ifndef __BYTEWORD__
#define __BYTEWORD__
typedef unsigned short word;
typedef unsigned char byte;
#endif

/****************************************************************************/
/* FUNCTION PROTOTYPES                                                      */
/****************************************************************************/

void    TMSL_TDIL(void);
void    TMSH_TDIL(void);
void    TMSL_TDIH(void);
void    TMSH_TDIH(void);
void    TMSL_TDIH_TDOrd(void);
void    TMSL_TDIL_TDOrd(void);
void    TMSH_TDIH_TDOrd(void);
void    TMSH_TDIL_TDOrd(void);
void    IR_Ex_Blow_SBW_Shift(void);
void    MsDelay(word milliseconds);      // millisecond delay loop, uses Timer_A
void    usDelay(word microeconds);       // microsecond delay loop, uses nops
void    InitController(void);
void    InitSerial(void);
void    InitTarget(void);
void    ReleaseTarget(void);
word    Shift(word Format, word Data);   // used for IR- as well as DR-shift
void    TDOisInput(void);
void    TCLKstrobes(word Amount);
void    ShowStatus(word Status, word Index);
void    TriggerPulse(word Mode);         // optional for test
void    DrvSignals( void );
void    RlsSignals( void );
void    SetTargetVcc (word level);
word    Get_target_Vcc( void);
word    Get_Ext_Vcc( void );
void    TDI_dir( word dir );
void    TDOI_dir( word dir );
void    TEST_dir( word dir );
void    TMS_dir( word dir );
void    RST_dir( word dir );
void    TCK_dir( word dir );
void    SetVpp( word source );

void    SetVCoreUp( word level);
word    Get_Vx( word index );
void    Enable_Vpp( void );
void    Disable_Vpp( void );
void    configure_IO_JTAG( void );
void    configure_IO_BSL( void );
void    IO_3state( void );
void    configure_IO_SBW( void );
