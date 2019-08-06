/*----------------------------------------------------------------------------
LAB 1 TASK 1: DOUBLING
--------------------------------------
Write an assembly code subroutine to double a number.
 *----------------------------------------------------------------------------*/

#include "mbed.h"

//#include "ble/BLE.h"
//BLE &ble = BLE::Instance();

#include "SimpleBLE/SimpleBLE.h"

SimpleBLE ble("DEVICE_NAME");

SimpleChar<uint8_t> heartrate = ble.readOnly_u8(0x180d, 0x2a37, true, 100);

void updateHR() {
    heartrate = heartrate + 1;
    if (heartrate > 180) {
        heartrate = 100;
    }
}

Ticker t;

int main(void)
{
    t.attach(updateHR, 1.0f);
    ble.start();
    while (1) {
        ble.waitForEvent();
    }
}
// *******************************ARM University Program Copyright © ARM Ltd 2013*************************************   
