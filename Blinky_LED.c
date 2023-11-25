#include "Blinky_LED.h"
#include "ADC_Background_Scan.h"

#include "IfxPort_PinMap.h"


void blinkLED1(){
    IfxPort_togglePin(LED1);
}

void blinkLED2(){
    IfxPort_togglePin(LED2);
}

void initLED(){
    IfxPort_setPinModeOutput(LED1, IfxPort_OutputMode_pushPull, IfxPort_OutputIdx_general);
    IfxPort_setPinModeOutput(LED2, IfxPort_OutputMode_pushPull, IfxPort_OutputIdx_general);

    IfxPort_setPinHigh(LED1);
    IfxPort_setPinLow(LED2);
}
