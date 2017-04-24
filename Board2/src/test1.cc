/** 
 *   The simplest test - LED blinker 
 */

#include "os.hh"
#include "pins.hh"

DigitalOutput<GPIOA, GPIO12> pinLED;

void setup()
{
    pinLED.enable();    
}

void loop()
{
    pinLED.set();
    delay_ms(200);
    pinLED.clear();
    delay_ms(800);    
}
