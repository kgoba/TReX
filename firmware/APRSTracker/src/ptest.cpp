#include <ptlib/periph.h>

USART<USART2, PA_2, PA_3>   nmea;
USART<USART1, PA_9, PA_10>  debug;

DigitalOut<PB_7>    gpsEnable;

void setup()
{
    nmea.begin(9600);
    debug.begin(9600);
    
    debug.putc('X');
    debug.putc('\n');

    gpsEnable.begin();
    gpsEnable = 0;
}

void loop()
{
    int c = nmea.getc();
    if (c != -1) {
        debug.putc(c);
    }
}

int main()
{
    rcc_clock_setup_in_hsi_out_48mhz();

    setup();    
    while (true) {
        loop();
    }    
    
    return 0;
}
