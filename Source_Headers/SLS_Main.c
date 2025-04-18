//#############################################################################
//
// FILE:    gpio_ex2_toggle.c
//
// TITLE:   Device GPIO Toggle
//
//! \addtogroup driver_example_list
//! <h1> Device GPIO Toggle </h1>
//!
//! Configures the device GPIO through the sysconfig file. The GPIO pin is 
//! toggled in the infinite loop.
//! In order to migrate the project within syscfg to any device, 
//! click the swtich button under the device view and select your   
//! corresponding device to migrate, saving the project will 
//! auto-migrate your project settings.
//!
//!  \note: This example project has support for migration across our C2000 
//!  device families. If you are wanting to build this project from launchpad
//!  or controlCARD, please specify in the .syscfg file the board you're using.
//!  At any time you can select another device to migrate this example.
//
//#############################################################################
//
//
// $Copyright:
// Copyright (C) 2022 Texas Instruments Incorporated - http://www.ti.com
//
// Redistribution and use in source and binary forms, with or without 
// modification, are permitted provided that the following conditions 
// are met:
// 
//   Redistributions of source code must retain the above copyright 
//   notice, this list of conditions and the following disclaimer.
// 
//   Redistributions in binary form must reproduce the above copyright
//   notice, this list of conditions and the following disclaimer in the 
//   documentation and/or other materials provided with the   
//   distribution.
// 
//   Neither the name of Texas Instruments Incorporated nor the names of
//   its contributors may be used to endorse or promote products derived
//   from this software without specific prior written permission.
// 
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS 
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT 
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
// A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT 
// OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, 
// SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT 
// LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
// DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
// THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT 
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE 
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
// $
//#############################################################################

//
// Included Files
//
#include "driverlib.h"
#include "device.h"
#include "board.h"
#include <SLS_I2C.h>
#include <SLS_SPI.h>
#include <SLS_Timers.h>
#include <SLS_UART.h>
#include <SLS_THERMAL.h>
#include <SLS_CANBUS.h>
#include <SLS_States.h>

//
// Main
//
void main(void)
{
    //
    // Initializes system control, device clock, and peripherals
    //
    Device_init();

    //
    // Initializes PIE and clear PIE registers. Disables CPU interrupts.
    // and clear all CPU interrupt flags.
    //
    Interrupt_initModule();

    //
    // Initialize the PIE vector table with pointers to the shell interrupt
    // Service Routines (ISR).
    //
    Interrupt_initVectorTable();

    //
    // Board Initialization
    //
    Board_init();

    ConfigureUART();

    //
    // Enables CPU interrupts
    //
    Interrupt_enableGlobal();

    I2CA_Init();
    I2CB_Init();
    FANinit();
    SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_TBCLKSYNC);
    IncrementPowerCycles();
    Init_CANA();

    //
    // Loop.
    //
    while(1)
        {

            switch(GetCurrentState())              // Case structure to support movement between state machines
            {
                case Standby:
                SetFan2DutyCycle(50.0f);
                break;

                case Running:
                SetFan2DutyCycle(0.0f);
                SetFan1DutyCycle(100.0f);
                break;

                case Fault:
                SetFan2DutyCycle(20.0f);
                SetFan1DutyCycle(20.0f);
                break;

                case Stop:
                SetFan1DutyCycle(0.0f);
                SetFan2DutyCycle(0.0f);
                break;

            }
       //Run the UART messenger
        CheckTwoHz();

    }
}



//
// End of File
//

