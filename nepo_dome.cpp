#include <pigpio/pigpio.h>
#include <iostream>

#include "nepo_dome.h"

#include "libindi/indicom.h"

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
#define PIN_ISN 13
#define PIN_ROT 12


//using namespace std;

static std::unique_ptr<NepoDomeDriver> nepoDomeDriver(new NepoDomeDriver());

NepoDomeDriver::NepoDomeDriver() {
    SetDomeCapability(DOME_CAN_ABORT | DOME_CAN_ABS_MOVE | DOME_CAN_REL_MOVE | DOME_CAN_PARK | DOME_HAS_SHUTTER);
}

// Control funktions for motors
void right() {
    gpioWrite(PIN_L, PI_ON);
    gpioWrite(PIN_R, PI_OFF);
}

void left() {
    gpioWrite(PIN_R, PI_ON);
    gpioWrite(PIN_L, PI_OFF);
}

void stopRot() {
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
    return 0 == gpioRead(PIN_ISN);
}

bool isRotImp() {
    return 0 == gpioRead(PIN_ROT);
}

const char* NepoDomeDriver::getDefaultName()
{
    return "Nepo Dome Driver";
}

bool NepoDomeDriver::Connect()
{
    LOGF_INFO("%i impulses", rotImps);

    if (isOpen()) {
        currentShutterAction = ShutterAction::OPEN;
        DomeShutterSP[0].setState(ISS_ON);
        DomeShutterSP[1].setState(ISS_OFF);
    } else if (isClosed()) {
        currentShutterAction = ShutterAction::CLOSED;
        DomeShutterSP[0].setState(ISS_ON);
        DomeShutterSP[1].setState(ISS_OFF);
    } else {
        //Dome isn't open or closed
        currentShutterAction = ShutterAction::STOPPED;
        DomeShutterSP.setState(IPS_ALERT);
    }
    DomeShutterSP.apply();

    LOG_INFO("Dome connected successfully!");
    return true;
}

bool NepoDomeDriver::Disconnect() {
    LOG_INFO("Dome disconnected successfully!");
    return true;
}

bool NepoDomeDriver::initPiGPIO() {
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
            LOG_ERROR((std::string("Setting the mode of GPIO ") + relay_names[i] + std::string(" (" + std::to_string(relay_pins[i]) + ") failed. Error code: " + std::to_string(err))).c_str());
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
            LOG_ERROR((std::string("Setting the mode of GPIO ") + sensor_names[i] + std::string(" (" + std::to_string(sensor_pins[i]) + ") failed. Error code: " + std::to_string(err))).c_str());
            return false;
        }
        err = gpioSetPullUpDown(sensor_pins[i], PI_PUD_DOWN);
        if (err) {
            LOG_ERROR((std::string("Setting the pull down of GPIO ") + sensor_names[i] + std::string(" (" + std::to_string(sensor_pins[i]) + ") failed. Error code: " + std::to_string(err))).c_str());
            return false;
        }
    }

    return true;
}

bool NepoDomeDriver::initProperties()
{
    INDI::Dome::initProperties();

    SetParkDataType(PARK_AZ);

    addAuxControls();

    // initialization of PiGPIO
    bool ok = initPiGPIO();
    if (!ok) {
        return false;
    }

    // calibrating rotational meassurements
    // moving to the leftmost point that's still north
    right();
    while (!isNorthed()) {};
    stopRot();
    while (isNorthed()) {};
    // counting switches of rotation impuls sensor while turning around 360°
    bool prevState = isRotImp();
    right();
    while (!isNorthed()) {
        bool currState = isRotImp();
        rotImps += prevState != currState;
        prevState = currState;
    }
    stopRot();
    // the Count of impulses has to be half of the amount of switches because every impuls is counted twice: rising edge and falling edge
    rotImps /= 2;

    // starting Timer loop
    SetTimer(10);

    return true;
}

void NepoDomeDriver::TimerHit() {
    // handle shutter movement
    if (currentShutterAction == ShutterAction::OPENING) {
        if (isOpen()) {
            stopShutter();
            currentShutterAction = ShutterAction::OPEN;
            DomeShutterSP.setState(IPS_OK);
            DomeShutterSP.apply();
        } else {
            open();
        }
    } else if (currentShutterAction == ShutterAction::CLOSING) {
        if (isClosed()) {
            stopShutter();
            currentShutterAction = ShutterAction::CLOSED;
            DomeShutterSP.setState(IPS_OK);
            DomeShutterSP.apply();
        } else {
            close();
        }
    } else {
        stopShutter();
    }

    // handle dome rotation
    // meassurements
    if (isNorthed()) {
        DomeAbsPosNP[0].setValue(range360(0));
    }
    /*/
    if (isNorthed()) {
        stopRot();
    } else {
        right();
    }
    /**/

    // call setTimer to continue the loop
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
    if (operation == DomeMotionCommand::MOTION_STOP) {
        stopRot();
        DomeAbsPosNP.setState(IPS_OK);
        DomeAbsPosNP.apply();
        return IPS_OK;
    }

    if (dir == DomeDirection::DOME_CW) {
        right();
        DomeAbsPosNP.setState(IPS_BUSY);
    } else if (dir == DomeDirection::DOME_CCW) {
        left();
        DomeAbsPosNP.setState(IPS_BUSY);
    }
    DomeAbsPosNP.apply();
    return IPS_BUSY;
}

IPState NepoDomeDriver::MoveRel(double azDiff) {
    targetedAz = range360(DomeAbsPosNP[0].getValue() + azDiff);

    return IPS_BUSY;
}

IPState NepoDomeDriver::MoveAbs(double az) {
    targetedAz = az;

    return IPS_BUSY;
}

IPState NepoDomeDriver::Park()
{
    IPState s = NepoDomeDriver::ControlShutter(SHUTTER_CLOSE);
    IPState d = IPS_OK; //NepoDomeDriver::MoveAbs(GetAxis1Park());

    SetParked(true);

    if (s == IPS_OK && d == IPS_OK)
        return IPS_OK;
    if (s == IPS_ALERT || d == IPS_ALERT)
        return IPS_ALERT;
    return IPS_OK;
}

IPState NepoDomeDriver::UnPark() {
    LOG_INFO("UnPark was called");
    DomeAbsPosNP.setState(IPS_OK);
    DomeAbsPosNP.apply();
    NepoDomeDriver::ControlShutter(SHUTTER_OPEN);

    SetParked(false);
    return IPS_OK;
}

bool NepoDomeDriver::Abort()
{
    if (DomeShutterSP.getState() == IPS_BUSY) {
        currentShutterAction = ShutterAction::STOPPED;
        DomeShutterSP.setState(IPS_ALERT);
        DomeShutterSP.apply();
    }
    Move(DOME_CW, MOTION_STOP);
    return true;
}

bool NepoDomeDriver::SetCurrentPark() {
    SetAxis1Park(DomeAbsPosNP[0].getValue());
    return true;
}

bool NepoDomeDriver::SetDefaultPark() {
    SetAxis1Park(90);
    return true;
}

