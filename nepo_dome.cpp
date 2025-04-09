#include <pigpio/pigpio.h>
#include <iostream>
#include <chrono>

#include "nepo_dome.h"
#include "config.h"

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


static std::unique_ptr<NepoDomeDriver> nepoDomeDriver(new NepoDomeDriver());

NepoDomeDriver::NepoDomeDriver() {
    SetDomeCapability(DOME_CAN_ABORT | DOME_CAN_ABS_MOVE | DOME_CAN_REL_MOVE | DOME_CAN_PARK | DOME_HAS_SHUTTER);
}

inline long getMillis() {
    return std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
}

// Control funktions for motors
enum RotDirection {
    RIGHT,
    LEFT,
    NONE
};
RotDirection curRot = RotDirection::NONE;

void right() {
    gpioWrite(PIN_L, PI_ON);
    gpioWrite(PIN_R, PI_OFF);
    curRot = RotDirection::RIGHT;
}

void left() {
    gpioWrite(PIN_R, PI_ON);
    gpioWrite(PIN_L, PI_OFF);
    curRot = RotDirection::LEFT;
}

void stopRot() {
    gpioWrite(PIN_R, PI_ON);
    gpioWrite(PIN_L, PI_ON);
    curRot = RotDirection::NONE;
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

    calibrate();

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

void NepoDomeDriver::calibrate() {
    // calibrating rotational meassurements
    // moving to the leftmost point that's north
    LOG_INFO("Started calibration");
    DomeAbsPosNP.setState(IPS_BUSY);
    DomeAbsPosNP.apply();
    right();
    while (!isNorthed()) {};
    // start of time meassurement of a full right/clockwise rotation (leftmost north to leftmost north)
    long time_started = getMillis();
    // counting edges (both kinds) of rotation impuls sensor while turning around 360°
    // time meassurement and counting of rotation impulses happens at the same time to save time
    int edges = 0;
    bool prevState = isRotImp();
    while (isNorthed()) {
        bool currState = isRotImp();
        edges += prevState != currState;
        prevState = currState;
    }
    while (!isNorthed()) {
        bool currState = isRotImp();
        edges += prevState != currState;
        prevState = currState;
    }
    long time_finished = getMillis();
    // the Count of impulses has to be half of the amount of edges because every impuls is counted twice: rising edge and falling edge
    impCount[0].setValue(edges / 2);
    impCount.apply();
    // the speed is meassured in °/ms
    speed[SPEED_R].setValue(360.0/(time_finished - time_started));

    // meassuring the time a full left/counterclockwise rotation (leftmost north to leftmost north)
    // the modell overshoots sometimes after rotating clockwise and is therefore rotated slightly right of the north that's why it's rotating counterclockwise back to north
    // to still be reliable it rotates 1 second to the right to guarantee an overshoot
    sleep(1);
    left();
    while (!isNorthed()) {};
    while (isNorthed()) {};
    time_started = getMillis();
    while (!isNorthed()) {};
    while (isNorthed()) {};
    time_finished = getMillis();
    speed[SPEED_L].setValue(360.0/(time_finished - time_started));
    speed.apply();

    // again creating an overshoot/offset (this time to the left)
    sleep(1);
    right();
    //meassuring the positions of the imps
    while (!isNorthed()) {};
    if (isRotImp()) { // avoid meassuring uncertainty in the case at north is also a imp
        stopRot();
        impToNorthOffset[0].setValue(0);
        nextRightImpAz = 360.0 / impCount[0].getValue();
        nextLeftImpAz = -360.0 / impCount[0].getValue();
    } else {
        time_started = getMillis();
        while (!isRotImp()) {};
        time_finished = getMillis();
        impToNorthOffset[0].setValue(speed[SPEED_R].getValue()*(time_finished-time_started));
        nextRightImpAz = impToNorthOffset[0].getValue();
        nextLeftImpAz = nextRightImpAz - 360.0 / impCount[0].getValue();
        // again creating an overshoot/offset (to the right)
        sleep(1);
        //returning to North
        left();
        while (!isNorthed()) {};
        stopRot();
    }
    impToNorthOffset.apply();

    if (impToNorthOffset[0].getValue() > (360.0 / impCount[0].getValue())){
        LOG_WARN("Calibration failed: retrying");
        calibrate();
        return;
    }

    DomeAbsPosNP[0].setValue(0);
    DomeAbsPosNP.setState(IPS_OK);
    DomeAbsPosNP.apply();

    LOGF_INFO("Offset between north and its right impuls: %f°", impToNorthOffset[0].getValue());
    LOGF_INFO("Counterclockwise speed: %f°/ms", speed[SPEED_L].getValue());
    LOGF_INFO("Clockwise speed: %f°/ms", speed[SPEED_R].getValue());
    LOGF_INFO("Rotation impulses per complete rotation: %f", impCount[0].getValue());
    LOG_INFO("Finished calibration");
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
    double nextPos = DomeAbsPosNP[0].getValue();
    long now = getMillis();
    bool currImpState = isRotImp();

    if (curRot == RotDirection::RIGHT) {
        nextPos += speed[SPEED_R].getValue()*(now - lastMeassurements);
        if (currImpState && !prevImpState) {
            nextPos = nextRightImpAz;
            nextRightImpAz += 360.0 / impCount[0].getValue();
            nextLeftImpAz = nextRightImpAz - 720.0 / impCount[0].getValue();
        }
    } else if (curRot == RotDirection::LEFT) {
        nextPos -= speed[SPEED_L].getValue()*(now - lastMeassurements);
        if (currImpState && !prevImpState) {
            nextPos = nextLeftImpAz;
            nextLeftImpAz -= 360.0 / impCount[0].getValue();
            nextRightImpAz = nextLeftImpAz + 720.0 / impCount[0].getValue();
        }
    }

    // 540° is 1.5*360° this is to check if nextLeftImpAz and nextRightImpAz have a difference of 1 or 2 times 360.0 / impCount[0].getValue() and avoids rounding errors
    if (prevImpState && !currImpState) {
        if (curRot == RotDirection::RIGHT) {
            nextLeftImpAz += 360.0 / impCount[0].getValue();
        } else if (curRot == RotDirection::LEFT) {
            nextRightImpAz -= 360.0 / impCount[0].getValue();
        }
    }

    if (isNorthed()) {
        nextPos = 0;
        if (impToNorthOffset[0].getValue()==0) {
            nextRightImpAz = 360.0 / impCount[0].getValue();
            nextLeftImpAz = -360.0 / impCount[0].getValue();
        } else {
            nextRightImpAz = impToNorthOffset[0].getValue();
            nextLeftImpAz = nextRightImpAz - 360.0 / impCount[0].getValue();
        }
    }

    lastMeassurements = now;
    prevImpState = currImpState;
    // check if targetedAz lies between current and next position
    if (moveToTarget && ((range360(DomeAbsPosNP[0].getValue() - targetedAz)<180) != (range360(nextPos - targetedAz)<180))) {
        moveToTarget = false;
        stopRot();
        DomeAbsPosNP.setState(IPS_OK);
        DomeRelPosNP.setState(IPS_OK);
        DomeRelPosNP.apply();
    }
    DomeAbsPosNP[0].setValue(range360(nextPos));
    DomeAbsPosNP.apply();

    if (moveToTarget) {
        if (range360(DomeAbsPosNP[0].getValue() - targetedAz)<180) {
            left();
        } else {
            right();
        }
    }


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
    moveToTarget = false;
    if (operation == DomeMotionCommand::MOTION_STOP) {
        stopRot();
        DomeAbsPosNP.setState(IPS_OK);
        DomeAbsPosNP.apply();
        DomeRelPosNP.setState(IPS_OK);
        DomeRelPosNP.apply();
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
    moveToTarget = true;

    return IPS_BUSY;
}

IPState NepoDomeDriver::MoveAbs(double az) {
    targetedAz = range360(az);
    moveToTarget = true;

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

bool NepoDomeDriver::saveConfigItems(FILE *fp) {
    Dome::saveConfigItems(fp);
    impCount.save(fp);
    speed.save(fp);
    impToNorthOffset.save(fp);
    return true;
}