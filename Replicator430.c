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
| Replicator430.c                                                            |
|                                                                            |
| JTAG Replicator for the MSP430 Flash-based family devices                  |
|                                                                            |
| Key features:                                                              |
| � Supports JTAG communication to all MSP430 Flash Devices                  |
| � Max. code size of target program: 57kB                                   |
| � Programming speed: ~60kB/10sec (~6kB/sec)                                |
| � Fast Verification and Erase Check: ~60kB/350ms                           |
| � Supports Programming of JTAG Access Protection Fuse                      |
|                                                                            |
|----------------------------------------------------------------------------|
| Project:              MSP430 Replicator                                    |
| Developed using:      IAR Embedded Workbench 6.20                          |
|             and:      Code Composer Studio 6.0                             |
|----------------------------------------------------------------------------|
| Version history:                                                           |
| 1.0 04/02 FRGR        Initial version.                                     |
| 1.1 04/02 FRGR        Included SPI mode to speed up shifting function by 2.|
| 1.2 06/02 ALB2        Formatting changes, added comments.                  |
| 1.3 06/02 ALB2        DEVICE ID info added.                                |
| 1.4 08/02 ALB2        Initial code release with Lit# SLAA149.              |
| 1.5 01/03 ALB2        Unnecessary lines have been commented out(see below).|
| 1.6 09/05 JDI         converted to IAR EW 3.30. F2xxx support added        |
|           SUN1        Software delays redesigned to use TimerA harware;    |
|                       see MsDelay() routine.                               |
|           ALB2        Added 2xx Info memory B, C, & D erase & check        |
| 1.7 12/05 STO         Adapted for 2xx devices with SpyBiWire               |
| 1.8 02/06 STO         Minor cosmetic changes                               |
| 1.9 05/06 STO         Minor cosmetic changes                               |
| 2.0 04/07 WLUT        Minor changes at declarations and calls              |
|                       according to use srec_cat.exe                        |
| 2.1 07/09 FB          Added loop for polling P1.6 / added support for      |
|                       replicator                                           |
| 2.2 07/12 RL          Updated commentaries                                 |
| 2.3 03/13 RL/MD       Added InfoA unlock                                   |
| 2.4 06/15 RL          Commented out optional RAM operations                |
|----------------------------------------------------------------------------|
| Designed 2002 by Texas Instruments Germany                                 |
\*==========================================================================*/
//! \file Replicator430.c
//! \brief JTAG Replicator for the MSP430 Flash-based family.
/****************************************************************************/
/* Main section of Replicator program: User can modify/insert code as needed*/
/****************************************************************************/
/* Main function:
   Upon execution completion, LED blinks for F1xx & F4xx device target socket
   boards when used as the target system (P1.0 and P5.1 drive LEDs).

   Note: All High Level JTAG Functions are applied here.

   A number of lines have been commented out which are not strictly required
   to successfully program a flash-based MSP430 device using the Replicator
   concept. Please note that these lines have been commented out in order
   to provide reference examples for calling such functions.
   Uncommented lines below represent the minimum requirements to successfully
   program an MSP430 flash device using the Replicator.
   
    The basic routine consists of the following steps:
   
   1.  | Initialize the host MSP430 on the Replicator board
   2.  | Connect to the target device
   3.  | Control the target peripherals directly (optional)
   4.  | Perform a write + read + verify in the target RAM (optional)
   5.  | Operations in the device's main memory
   6.  | Blow JTAG access fuse (optional)
   7.  | Release the device from JTAG control and wait for the user to press button "S1"
*/

/****************************************************************************/
/* Includes                                                                 */
/****************************************************************************/

#include "JTAGfunc430.h"           // JTAG functions
#include "LowLevelFunc430.h"       // low level functions
#include "Devices430.h"            // holds Device specific information
#include "Target_Code.h"           // holds declarations and/or data of target program code

/****************************************************************************/
/* FUNCTIONS                                                                */
/****************************************************************************/
void runProgramm(void);

//! Main function
void main(void)
{
    runProgramm();
}

//! \brief The basic Replicator routine
//! \details This function is executed once at startup and can be restarted 
//! by pressing button "S1" on the REP430F board.
void runProgramm(void)
{
    //! \brief Data pointer
//    word p;
    //! \brief Buffer, used for memory read with ReadMemQuick()
//    word ReadArray[0x40];

/*------------------------------------------------------------------------------------------------------*/
/*  1. | Initialize host MSP430 (on Replicator board) & target board                                    */
/*------------------------------------------------------------------------------------------------------*/    
    
    InitController();                     // Initialize the host MSP430F5437
    
    InitSerial();

    ShowStatus(STATUS_ACTIVE, 0);         // Switch both LEDs on to indicate operation.

    InitTarget();                         // Initialize target board

    UCA0TXBUF = '0';

/*------------------------------------------------------------------------------------------------------*/
/*  2. | Connect to the target device                                                                   */
/*------------------------------------------------------------------------------------------------------*/    
    
    word status;
    if ((status = GetDevice()) != STATUS_OK)         // Set DeviceId
    {
        if (status == STATUS_FUSEBLOWN) {
            UCA0TXBUF = '2';
            ShowStatus(STATUS_ACTIVE, 0);
        } else {
            UCA0TXBUF = '1';
            ShowStatus(STATUS_ERROR, 1);      // stop here if invalid JTAG ID or
        }                                     // time-out. (error: red LED is ON)
    }
    
/*------------------------------------------------------------------------------------------------------*/
/*  3. | Control the target peripherals directly (OPTIONAL)                                             */
/*------------------------------------------------------------------------------------------------------*/

    // Remove the following comments to toggle Pin 1.0 (i.e flash the LED for MSP430 target socket boards)
    /*{
        word k;
        
        WriteMem(F_BYTE, 0x21, 0x01);         // P1.0 for F1xx,2xx devices
        WriteMem(F_BYTE, 0x31, 0x02);         // P5.1 for F4xx devices
    
        for(k = 0; k < 3; k++)
        {
            WriteMem(F_BYTE, 0x22, 0x01);
            WriteMem(F_BYTE, 0x32, 0x02);
            MsDelay(500);                     // LED on for 0.5s
            WriteMem(F_BYTE, 0x21, 0x00);
            WriteMem(F_BYTE, 0x31, 0x00);
            MsDelay(500);                     // LED off for 0.5s    
        }
    }*/

/*------------------------------------------------------------------------------------------------------*/
/*  4. | Perform a write + read + verify in the target RAM (OPTIONAL)                                   */
/*------------------------------------------------------------------------------------------------------*/
    
	// The following section is not required and included only as a reference on
    // how to access the target's RAM and alter its content

	/*
	// Communication test: Write 2 RAM bytes
    WriteMem(F_BYTE, 0x0200, 0x34);
    WriteMem(F_BYTE, 0x0201, 0x12);
    // Read back word
    if (ReadMem(F_WORD, 0x0200) != 0x1234)
    {
        ShowStatus(STATUS_ERROR, 5);
    }
    
    // Write RAM word
    WriteMem(F_WORD, 0x0202, 0x5678);
    // Read back 2 bytes
    if (ReadMem(F_BYTE, 0x0202) != 0x78)
    {
        ShowStatus(STATUS_ERROR, 6);
    }
    if (ReadMem(F_BYTE, 0x0203) != 0x56)
    {
        ShowStatus(STATUS_ERROR, 7);
    }

    // Is the Read/WriteQuick for current device possible?
    if (DeviceHas_DataQuick())
    {
        // Write RAM block
        WriteMemQuick (0x0200, 0x0010, (word*)&eprom[0]);
        // Verify(PSA) RAM of target
        if (!VerifyMem(0x0200, 0x0010, (word*)&eprom[0]))
        {
            ShowStatus(STATUS_ERROR, 8);
        }
        // Read RAM block
        ReadMemQuick  (0x0200, 0x0010, &ReadArray[0]);
        // Verify(word-for-word) RAM of target
        for (p = 0; p < 0x0010; p++)
        {
            if (ReadArray[p] != eprom[p])
            {
                ShowStatus(STATUS_ERROR, 9);
            }
        }
    }
	*/
    
/*------------------------------------------------------------------------------------------------------*/
/*  5. | Operations in the device's main memory                                                         */
/*------------------------------------------------------------------------------------------------------*/
    
    // The following section is not required and included only for reference as to the
    // manner in which a single flash memory segment can be erased. The more common
    // "mass erase" is used to prepare the target device for replicator programming.
    /*{
        // Segment 0 erase Flash (all types)
        EraseFLASH(ERASE_SGMT, 0xFE00);
        // Check segment 0 memory erasure
        if (!EraseCheck(0xFE00, 0x0100))
        {
            ShowStatus(STATUS_ERROR, 2);
        }
    }*/

    // Perform a mass erase  
    //EraseFLASH(ERASE_MASS, 0xFE00);     // Mass-Erase Flash (all types)
    // NOTE: the INFO memory in F2xx device will be not erased
    // if the memory is locked. For more info See EraseFLASH() and
    // UnlockInfoA_430() in JTAGfunc430.c
    
    //if (!EraseCheck(0x1000, 0x0040))      // Check info memory erasure (Fxx2..9)
    //{
    //    ShowStatus(STATUS_ERROR, 3);
    //}

    /*
    if (!EraseCheck(0xF800, 0x0400))        // Check main memory erasure (Fxx2..9)
    {
        ShowStatus(STATUS_ERROR, 3);
    }*/

    /* For all 1xx & 4xx & 2xx devices, where ALL Flash has been erased*/
    /*
    EraseFLASH(ERASE_SGMT, 0xFE00);
    if (!EraseCheck(0xfe00, 0x0100))      // Check part of main memory erasure (Fxx2..9)
    {
        ShowStatus(STATUS_ERROR, 4);
    }*/

    // For all 2xx devices, where Info A was not erased (Info A keeps calibration data)
    /*
    // Check info memory erasure (Fxx2..9)
    if (!EraseCheck(0x1000, 0x0080))
    {  
        ShowStatus(STATUS_ERROR, 4);
    }
    */

    // Comment in the following section to erase Info-Segments on 2xx Devices selectively
    // Please adjust addresses for devices with special memory segment sizes (i.e. MSP430i2040)
    /*{
        // Comment-in the following code to unlock Info memory segment A
        // WARNING: Info A may include calibration data!
        {
            // Info A unlock
            UnlockInfoA();

            // Info A erase
            EraseFLASH(ERASE_SGMT, 0x10C0);
            // Check Info A memory erasure (2xx)
            if (!EraseCheck(0x10C0, 0x0020))
            {
                ShowStatus(STATUS_ERROR, 2);
            }
        }
            
        // Info B erase (2xx)
        EraseFLASH(ERASE_SGMT, 0x1080);
        // Check Info B memory erasure (2xx)
        if (!EraseCheck(0x1080, 0x0020))
        {
            ShowStatus(STATUS_ERROR, 2);
        }
        
        // Info C erase (2xx)
        EraseFLASH(ERASE_SGMT, 0x1040);
        // Check Info C memory erasure (2xx)
        if (!EraseCheck(0x1040, 0x0020))
        {
            ShowStatus(STATUS_ERROR, 2);
        }
        
        // Info D erase (2xx)
        EraseFLASH(ERASE_SGMT, 0x1000);
        // Check Info D memory erasure (2xx)
        if (!EraseCheck(0x1000, 0x0020))
        {
            ShowStatus(STATUS_ERROR, 2);
        }
    }*/

    // Program target code
    /*
    if (!WriteFLASHallSections(&eprom[0], &eprom_address[0], &eprom_length_of_sections[0], eprom_sections))
    {
        ShowStatus(STATUS_ERROR, 10);
    }*/

/*------------------------------------------------------------------------------------------------------*/
/*  6. | Blow the JTAG access protection fuse (OPTIONAL)                                                */
/*------------------------------------------------------------------------------------------------------*/
     
    // Remove following comments to enable the JTAG fuse blow routine.
    // This makes the MSP430 device permanently inaccessible via JTAG

    /*if (!BlowFuse())              // ***Action is permanent***
    {
        ShowStatus(STATUS_ERROR, 15);
    }*/

/*------------------------------------------------------------------------------------------------------*/
/*  7. | Release the target device from JTAG control and wait for the user to press button "S1"         */
/*------------------------------------------------------------------------------------------------------*/
    
    ReleaseDevice(V_RESET);         // Perform Reset, release CPU from JTAG control
                                    // Target board LED should start blinking
    ShowStatus(STATUS_OK, 0);       // OK: green LED is ON
    UCA0TXBUF = '3';
    
    _EINT();                        // Enable Interrupts
    P4IE |= 0x01;                   // P4.0 interrupt enabled
    
    while(1);                       // Idle, wait for keypress (handled in ISR)
}

/*------------------------------------------------------------------------------------------------------*/

//! This interrupt service routine calls runProgramm() if button "S1" is pressed
#pragma vector=PORT1_VECTOR
__interrupt void Port_1(void)
{   
        ReleaseTarget();                // Release target board from power
        ShowStatus(STATUS_ACTIVE, 0);   // Switch both LEDs on to indicate operation.
        P4IE &= ~0x01;                  // P4.0 interrupt disabled
        MsDelay(1000);                  // Wait 1s before restarting the Controller and Target
        P4IFG = 0;
        runProgramm();                  // Restart Controller and Target
}

/****************************************************************************/
/*                         END OF SOURCE FILE                               */
/****************************************************************************/
