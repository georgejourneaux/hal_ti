/******************************************************************************
*  Filename:       group_aux_doc.h
*
*  Copyright (c) 2015 - 2022, Texas Instruments Incorporated
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions are met:
*
*  1) Redistributions of source code must retain the above copyright notice,
*     this list of conditions and the following disclaimer.
*
*  2) Redistributions in binary form must reproduce the above copyright notice,
*     this list of conditions and the following disclaimer in the documentation
*     and/or other materials provided with the distribution.
*
*  3) Neither the name of the ORGANIZATION nor the names of its contributors may
*     be used to endorse or promote products derived from this software without
*     specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
*  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
*  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
*  ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
*  LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
*  CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
*  SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
*  INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
*  CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
*  ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*
******************************************************************************/
//! \addtogroup aux_group
//! @{
//! \section sec_aux Introduction
//!
//! The AUX is a collective description of all the analog peripherals (ADC, comparators, and current source) and
//! the digital modules in the AUX power domain (AUX_PD) such as the sensor controller, timers, time-to-digital
//! converter, etc. AUX_PD is located within the AON voltage domain of the device.
//!
//! The sensor controller has the ability to
//! do its own power and clock management of AUX_PD, independently of the MCU domain. The sensor
//! controller can also continue doing tasks while the MCU subsystem is powered down, but with limited
//! resources compared to the larger MCU domain.
//!
//! The AUX power domain is connected to the MCU system through an asynchronous interface, ensuring
//! that all modules connected to the AUX bus are accessible from the system CPU.
//! Accessing the analog peripherals from the system CPU must be done by using TI-provided
//! drivers to ensure proper control of power management.
//!
//! \note To ease development of program code running on the sensor controller, TI provides a tool
//! chain for writing software for the controller, Sensor Controller Studio (SCS), which is a fully
//! integrated tool consisting of an IDE, compiler, assembler, and linker.
//!
//! @}
