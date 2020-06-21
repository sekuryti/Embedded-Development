#include <sodaq_wdt.h>

void setup()
{
    sodaq_wdt_enable(WDT_PERIOD_8X); // set wdt at 8 second interval
    sodaq_wdt_reset();
    SCB->SCR |= SCB_SCR_SLEEPDEEP_Msk; // set sleep mode to deep sleep	
}

void loop()
{
    sodaq_wdt_reset(); // Resets the wdt to prevent device resetting
    __WFI(); // Wait For Interrupt - sleeps according to sleep mode
}
