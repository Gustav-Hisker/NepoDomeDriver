#include "pigpio-master/pigpio.h"
#include <iostream>


#define PIN_R 22
#define PIN_L 23
#define PIN_O 24
#define PIN_S 25

using namespace std;


int main(int argc, char *argv[])
{
   if (gpioInitialise() < 0) {
      cout << "Initialization of GPIO failed" << endl;
      return -1;
   }

   int err = gpioSetMode(PIN_R, PI_INPUT);
   if (err) {
      cout << "Setting the Mode of" << R_PIN << " failed. Error code: " << err << endl;
      return -1;
   }

   gpioWrite(PIN_R, PI_ON)
   time_sleep(1);
   gpioWrite(PIN_R, PI_OFF)
   time_sleep(1);
   gpioWrite(PIN_R, PI_ON)

   gpioTerminate();
}