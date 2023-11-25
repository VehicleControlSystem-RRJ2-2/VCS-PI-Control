/***********************************************************************/
/*Include*/
/***********************************************************************/
#include "GTM_Motor_PWM.h"
#include "Driver_Stm.h"
#include "IfxPort.h"
#include "IfxPort_PinMap.h"
#include "IfxStm.h"

#include "Bsp.h"

#include "Blinky_LED.h"

#include "math.h"

#include <stdio.h>

/***********************************************************************/
/*Define*/
/***********************************************************************/

/***********************************************************************/
/*Typedef*/
/***********************************************************************/
typedef struct
{
    Ifx_STM *stmSfr; /**< \brief Pointer to Stm register base */
    IfxStm_CompareConfig stmConfig; /**< \brief Stm Configuration structure */
    volatile uint32 counter; /**< \brief interrupt counter */
} App_Stm;

/***********************************************************************/
/*Static Function Prototype*/
/***********************************************************************/

/***********************************************************************/
/*Variable*/
/***********************************************************************/
App_Stm g_Stm; /**< \brief Stm global data */
uint32 u32nuCounter1ms = 0u;

/***********************************************************************/
/*Function*/
/***********************************************************************/
//uint32 ULSraw;
////TestCnt stTestCnt;
//boolean ENCA;
//boolean ENCB;
//boolean ENCA_old;
//boolean ENCB_old;
uint32 trig_A;
uint32 trig_B;
boolean intr_A;
boolean intr_B;
float32 duty[2];
float32 Vin;
//int S;
//int S_old;
//int PosCnt = 0;
//int PosCntd = 0;
//float32 Direction;
//float32 Pos_rad;
//float32 ome_d;
//float32 Pos_rad_d;
//int Pos_deg;
//float32 theta;
//float32 theta_old;
//float32 theta_d_old;
float32 w = 0;
float32 Wd;
float32 w_old = 0;
float32 error_w = 0;
uint32 W_RPM;
float32 w_ref;
uint32 ACnt;
float32 error_w_int_old;
float32 error_w_int;
//float32 du_w;
//float32 kp = 0.017;
//float32 ki = 0.46;
float32 kp = 0.017;
float32 ki = 0.46;
float32 t;
float32 per;
float32 per_old;
float32 W_st;
float32 V_st;

float32 front_theta;
float32 front_theta_old;
float32 rear_theta;
float32 rear_theta_old;
float32 front_error_w;
float32 rear_error_w;

float32 front_w=0;
float32 front_w_old=0;
float32 front_error_w_int=0;
float32 front_error_w_int_old=0;
float32 rear_w=0;
float32 rear_w_old=0;
float32 rear_error_w_int=0;
float32 rear_error_w_int_old=0;

Ifx_TickTime g_ticksFor1ms;

IFX_INTERRUPT(STM_Int0Handler, 0, 100);

float32 LPF (float32 y_old, float32 x, float32 Ts, float32 band)
{
    double A1 = Ts / (Ts + 1 / band);

    float32 y = y_old + A1 * (x - y_old);

    return y;

}

void Driver_Stm_Init (void)
{
    /* disable interrupts */
    boolean interruptState = IfxCpu_disableInterrupts();

    IfxStm_enableOcdsSuspend(&MODULE_STM0);

    g_ticksFor1ms = IfxStm_getTicksFromMilliseconds(BSP_DEFAULT_TIMER, 1);

    g_Stm.stmSfr = &MODULE_STM0;
    IfxStm_initCompareConfig(&g_Stm.stmConfig);

    g_Stm.stmConfig.triggerPriority = 100u;
    g_Stm.stmConfig.typeOfService = IfxSrc_Tos_cpu0;
    g_Stm.stmConfig.ticks = g_ticksFor1ms;

    IfxStm_initCompare(g_Stm.stmSfr, &g_Stm.stmConfig);

    /* enable interrupts again */
    IfxCpu_restoreInterrupts(interruptState);

}

void STM_Int0Handler (void)
{
    IfxCpu_enableInterrupts();
    
    IfxStm_clearCompareFlag(g_Stm.stmSfr, g_Stm.stmConfig.comparator);
    IfxStm_increaseCompare(g_Stm.stmSfr, g_Stm.stmConfig.comparator, 100000u);

    u32nuCounter1ms++;

    if (u32nuCounter1ms % 10 == 0)
    {
        float32 Ts = 0.01;
        w_ref = 128;
        ACnt++;
        Wd = 5;
        t = ACnt * Ts;

        front_w = (front_theta - front_theta_old) / Ts;
        front_w = LPF(w_old, front_w, Ts, 500);
        front_w_old = front_w;
        W_RPM = (uint32) (60 * w / (2 * 3.141592));
        front_theta_old = front_theta;

        front_error_w = w_ref - front_w;
        front_error_w_int = front_error_w_int_old + (front_error_w) * Ts;
        front_error_w_int_old = front_error_w_int;

        if (front_error_w_int > 12)
        {
            front_error_w_int = 12;
        }

        Vin = (kp * front_error_w + ki * front_error_w_int);

        if (Vin > 12)
        {
            Vin = 12;
        }
        else if (Vin < 0)
        {
            Vin = 0;
        }

        duty[0] = Vin / 12;

        rear_w = (rear_theta - rear_theta_old) / Ts;
        rear_w = LPF(rear_w_old, rear_w, Ts, 500);
        rear_w_old = rear_w;
//        W_RPM = (uint32) (60 * w / (2 * 3.141592));
        rear_theta_old = rear_theta;

        rear_error_w = w_ref - rear_w;
        rear_error_w_int = rear_error_w_int_old + (rear_error_w) * Ts;
        rear_error_w_int_old = rear_error_w_int;

        if (rear_error_w_int > 12)
        {
            rear_error_w_int = 12;
        }

        Vin = (kp * error_w + ki * error_w_int);

        if (Vin > 12)
        {
            Vin = 12;
        }
        else if (Vin < 0)
        {
            Vin = 0;
        }

        duty[1] = Vin / 12;

//        setMotorDutyCycle_A(duty[0] * PWM_PERIOD);
//        setMotorDutyCycle_B(duty[1] * PWM_PERIOD);


        setMotorDutyCycle_A(0 * PWM_PERIOD);
        setMotorDutyCycle_B(0 * PWM_PERIOD);
    }

    if (u32nuCounter1ms % 1000 == 0)
    {
//        printf("front_w: %f\n", front_w);
//        printf("rear_w: %f\n", rear_w);
//        printf("front_duty : %f\n", duty[0]);
//        printf("rear_duty: %f\n", duty[1]);
//        printf("front_ERW :%f\n", front_error_w);
//        printf("rear_ERwW: %f\n", rear_error_w);
//        printf("front_theta: %f\n", front_theta);
//        printf("rear_theta: %f\n", rear_theta);
//
//        printf("\n");

        u32nuCounter1ms = 0;
    }
}

