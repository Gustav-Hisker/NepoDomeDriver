#include <pigpio/pigpio.h>
#include <iostream>

#include "nepo_dome.h"

#include "indicom.h"

#include <cmath>
#include <cstring>
#include <ctime>
#include <memory>

#define PIN_R 22
#define PIN_L 23
#define PIN_O 24
#define PIN_C 25

#define PIN_ISO 26
#define PIN_ISC 16



using namespace std;

static std::unique_ptr<NepoDomeDriver> nepoDomeDriver(new NepoDomeDriver());

NepoDomeDriver::NepoDomeDriver() {
    SetDomeCapability(DOME_CAN_ABORT | DOME_CAN_ABS_MOVE | DOME_CAN_REL_MOVE | DOME_CAN_PARK | DOME_HAS_SHUTTER);
}

// Control funktions for motors
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

void stopShutter(){
    gpioWrite(PIN_O, PI_ON);
    gpioWrite(PIN_C, PI_ON);
}

// check funktion for sensors
bool isOpen(){
    return 0 == gpioRead(PIN_ISO);
}

bool isClosed(){
    return 0 == gpioRead(PIN_ISC);
}

const char *NepoDomeDriver::getDefaultName()
{
    return "Nepo Dome Driver";
}

bool NepoDomeDriver::Connect()
{
    right();
    time_sleep(2);
    left();
    time_sleep(2);
    stopRot();
    LOG_INFO("Dome connected successfully!");
    return true;
}

bool NepoDomeDriver::Disconnect()
{
    LOG_INFO("Dome disconnected successfully!");
    return true;
}

bool NepoDomeDriver::initPiGPIO() {
    // Initialization of PIGPIO
    if (gpioInitialise() < 0) {
        LOG_INFO("Initialization of PIGPIO failed");
        return false;
    }

    // Setting up relays
    int relay_pins[] = {PIN_R, PIN_L, PIN_O, PIN_C};
    string relay_names[] = {"right", "left", "open", "close"};
    for (int i = 0; i < 4; i++) {
        int err = gpioSetMode(relay_pins[i], PI_OUTPUT);
        if (err) {
            LOG_INFO((std::string("Setting the mode of GPIO ") + relay_names[i] + std::string(" (" + std::to_string(relay_pins[i]) + ") failed. Error code: " + std::to_string(err))).c_str());
            return false;
        }
        gpioWrite(relay_pins[i], PI_ON);
    }

    // Setting up sensors
    int sensor_pins[] = {PIN_ISO, PIN_ISC};
    string sensor_names[] = {"is open", "is closed"};
    for (int i = 0; i < 2; i++) {
        int err = gpioSetMode(sensor_pins[i], PI_INPUT);
        if (err) {
            LOG_INFO((std::string("Setting the mode of GPIO ") + sensor_names[i] + std::string(" (" + std::to_string(sensor_pins[i]) + ") failed. Error code: " + std::to_string(err))).c_str());
            return false;
        }
        err = gpioSetPullUpDown(sensor_pins[i], PI_PUD_DOWN);
        if (err) {
            LOG_INFO((std::string("Setting the pull down of GPIO ") + sensor_names[i] + std::string(" (" + std::to_string(sensor_pins[i]) + ") failed. Error code: " + std::to_string(err))).c_str());
            return false;
        }
    }

    return true;
}

bool NepoDomeDriver::initProperties()
{
    INDI::Dome::initProperties();

    SetParkDataType(PARK_AZ);

    bool ok = initPiGPIO();
    if (!ok) {
        return false;
    }

    right();
    time_sleep(2);
    left();
    time_sleep(2);
    stopRot();

    SetTimer(10);

    return true;
}

void NepoDomeDriver::TimerHit() {
    // hanle shutter movement
    if (currentShutterAction == ShutterAction::OPENING){
        if (isOpen()){
            stopShutter();
            currentShutterAction = ShutterAction::OPEN;
        } else {
            open();
        }
    } else if (currentShutterAction == ShutterAction::CLOSING){
        if (isClosed()){
            stopShutter();
            currentShutterAction = ShutterAction::CLOSED;
        } else {
            close();
        }
    }

    SetTimer(10);
}

IPState NepoDomeDriver::ControlShutter(ShutterOperation operation)
{
    if (operation == ShutterOperation::SHUTTER_OPEN) {
        if (currentShutterAction == ShutterAction::OPEN)
            return IPS_OK;
        currentShutterAction = ShutterAction::OPENING;
        return IPS_BUSY;
    } else {
        if (currentShutterAction == ShutterAction::CLOSED)
            return IPS_OK;
        currentShutterAction = ShutterAction::CLOSING;
        return IPS_BUSY;
    }
}

IPState NepoDomeDriver::Move(DomeDirection dir, DomeMotionCommand operation) {

}

IPState NepoDomeDriver::MoveRel(double azDiff){

}

IPState NepoDomeDriver::MoveAbs(double az){

}

IPState NepoDomeDriver::Park(){

}

IPState NepoDomeDriver::UnPark(){

}

bool NepoDomeDriver::Abort(){

}

bool NepoDomeDriver::SetCurrentPark(){

}

bool NepoDomeDriver::SetDefaultPark(){

}
