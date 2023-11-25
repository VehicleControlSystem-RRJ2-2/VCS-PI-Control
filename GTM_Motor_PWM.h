/**********************************************************************************************************************
 * \file GTM_TOM_PWM.h
 * \copyright Copyright (C) Infineon Technologies AG 2019
 *
 * Use of this file is subject to the terms of use agreed between (i) you or the company in which ordinary course of
 * business you are acting and (ii) Infineon Technologies AG or its licensees. If and as long as no such terms of use
 * are agreed, use of this file is subject to following:
 *
 * Boost Software License - Version 1.0 - August 17th, 2003
 *
 * Permission is hereby granted, free of charge, to any person or organization obtaining a copy of the software and
 * accompanying documentation covered by this license (the "Software") to use, reproduce, display, distribute, execute,
 * and transmit the Software, and to prepare derivative works of the Software, and to permit third-parties to whom the
 * Software is furnished to do so, all subject to the following:
 *
 * The copyright notices in the Software and this entire statement, including the above license grant, this restriction
 * and the following disclaimer, must be included in all copies of the Software, in whole or in part, and all
 * derivative works of the Software, unless such copies or derivative works are solely in the form of
 * machine-executable object code generated by a source language processor.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE
 * WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE, TITLE AND NON-INFRINGEMENT. IN NO EVENT SHALL THE
 * COPYRIGHT HOLDERS OR ANYONE DISTRIBUTING THE SOFTWARE BE LIABLE FOR ANY DAMAGES OR OTHER LIABILITY, WHETHER IN
 * CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS
 * IN THE SOFTWARE.
 *********************************************************************************************************************/

#ifndef GTM_MOTOR_PWM_H_
#define GTM_MOTOR_PWM_H_

#include "Ifx_Types.h"

#define ISR_PRIORITY_TOM    20                                      /* Interrupt priority number                    */
#define LED                 IfxGtm_TOM0_2_TOUT104_P10_2_OUT         /* LED which will be driven by the PWM          */
#define PWM_PERIOD          50000                                   /* PWM period for the TOM                       */
#define FADE_STEP           PWM_PERIOD / 100                        /* PWM duty cycle for the TOM                   */

#define DIR_A       &MODULE_P10,1
#define PWM_A       &MODULE_P02,1
#define BREAK_A     &MODULE_P02,7
#define MOTOR_A_TOM     IfxGtm_TOM0_9_TOUT1_P02_1_OUT

#define DIR_B       &MODULE_P10,2
#define PWM_B       &MODULE_P10,3
#define BREAK_B     &MODULE_P02,6
#define MOTOR_B_TOM     IfxGtm_TOM0_3_TOUT105_P10_3_OUT

#define REAR_ENCODER_A      &MODULE_P02, 4
#define REAR_ENCODER_B      &MODULE_P02, 3

#define FRONT_ENCODER_A     &MODULE_P10, 5
#define FRONT_ENCODER_B     &MODULE_P02, 5


/*********************************************************************************************************************/
/*-----------------------------------------------Function Prototypes-------------------------------------------------*/
/*********************************************************************************************************************/
void initGtmTomPwm(void);

void fadeMotor(void);

void setMotorDutyCycle_A(uint32 dutyCycle);

void setMotorDutyCycle_B(uint32 dutyCycle);

void initMotor();


#endif /* GTM_MOTOR_PWM_H_ */
