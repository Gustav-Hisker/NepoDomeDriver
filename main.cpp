#include <pigpio/pigpio.h>
#include <iostream>


#define PIN_R 22
#define PIN_L 23
#define PIN_O 24
#define PIN_C 25

#define PIN_ISO 26
#define PIN_ISC 16
#define PIN_ISN 13
#define PIN_ROT 6

using namespace std;

void right()
{
    gpioWrite(PIN_L, PI_ON);
    gpioWrite(PIN_R, PI_OFF);
}

void left()
{
    gpioWrite(PIN_R, PI_ON);
    gpioWrite(PIN_L, PI_OFF);
}

void stopRot()
{
    gpioWrite(PIN_R, PI_ON);
    gpioWrite(PIN_L, PI_ON);
}

void open() {
    gpioWrite(PIN_C, PI_ON);
    gpioWrite(PIN_O, PI_OFF);
}

void close() {
    gpioWrite(PIN_O, PI_ON);
    gpioWrite(PIN_C, PI_OFF);
}

void stopShutter() {
    gpioWrite(PIN_O, PI_ON);
    gpioWrite(PIN_C, PI_ON);
}

// check funktions for sensors
bool isOpen() {
    return 0 == gpioRead(PIN_ISO);
}

bool isClosed() {
    return 0 == gpioRead(PIN_ISC);
}

bool isNorthed() {
    return 1 == gpioRead(PIN_ISN);
}

bool isRotImp() {
    return 1 == gpioRead(PIN_ROT);
}

bool initPiGPIO() {
    // Initialization of PIGPIO
    if (gpioInitialise() < 0) {
        LOG_ERROR("Initialization of PIGPIO failed");
        return false;
    }

    // Setting up relays
    int relay_pins[] = {PIN_R, PIN_L, PIN_O, PIN_C};
    std::string relay_names[] = {"right", "left", "open", "close"};
    for (int i = 0; i < 4; i++) {
        int err = gpioSetMode(relay_pins[i], PI_OUTPUT);
        if (err) {
            cout << ((std::string("Setting the mode of GPIO ") + relay_names[i] + std::string(" (" + std::to_string(relay_pins[i]) + ") failed. Error code: " + std::to_string(err))).c_str()) << endl;
            return false;
        }
        gpioWrite(relay_pins[i], PI_ON);
    }

    // Setting up sensors
    int sensor_pins[] = {PIN_ISO, PIN_ISC, PIN_ISN, PIN_ROT};
    std::string sensor_names[] = {"is open", "is closed", "is northed", "rotation meassuring impuls"};
    for (int i = 0; i < 2; i++) {
        int err = gpioSetMode(sensor_pins[i], PI_INPUT);
        if (err) {
            cout << ((std::string("Setting the mode of GPIO ") + sensor_names[i] + std::string(" (" + std::to_string(sensor_pins[i]) + ") failed. Error code: " + std::to_string(err))).c_str()) << endl;
            return false;
        }
        err = gpioSetPullUpDown(sensor_pins[i], PI_PUD_DOWN);
        if (err) {
            cout << ((std::string("Setting the pull down of GPIO ") + sensor_names[i] + std::string(" (" + std::to_string(sensor_pins[i]) + ") failed. Error code: " + std::to_string(err))).c_str()) << endl;
            return false;
        }
    }

    return true;
}

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