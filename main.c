#include "lib/terasic_includes.h"

#include "lib/led.h"
#include "lib/seg7.h"
#include "lib/key.h"
#include "lib/timer.h"

#include "lib/UART.h"
#include "lib/SPI.h"

#include "lib/myfunc.h"

#include <math.h>

char str[100];
char str_temp[100];

void DeviceInit()
{
    TimerIrqInit();
    KeyIrqInit();
    SEG7_Clear();

    UartIrqInit(1, 921600);
}

void PeripheralInit()
{
    usleep(1000000); // 1s

    // ctrl_error = -1;
    // while (ctrl_error != 0x88)
    // {
    //     sprintf(str, "rest");
    //     UartFrameTX(str);
    //     usleep(300000); // 0.3s
    // }

    // printf("Inited\n");
}

int main()
{
    // printf("Hello Nios\n");

    DeviceInit();
    PeripheralInit();

    while (1)
    {
        LED_Display(IORD(SW_BASE, 0));

        if (key_value != -1)
        {
            switch (key_value)
            {
            case 3:
                break;
            case 2:
                break;
            case 1:
                break;
            case 0:
                PeripheralInit();
                break;
            default:
                break;
            }

            key_value = -1;
        }

        if (ctrl[0x30] != -1)
        {
            ctrl[0x30] = -1;
        }
    }
}
