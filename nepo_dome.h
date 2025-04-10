#pragma once

#include "libindi/indidome.h"

class NepoDomeDriver : public INDI::Dome
{
public:
    NepoDomeDriver();
    virtual ~NepoDomeDriver() = default;

    virtual bool initProperties() override;
    virtual bool updateProperties() override;
    virtual const char *getDefaultName() override;

protected:
    bool Connect() override;
    bool Disconnect() override;

    void TimerHit() override;

    virtual IPState Move(DomeDirection dir, DomeMotionCommand operation) override;
    virtual IPState MoveRel(double azDiff) override;
    virtual IPState MoveAbs(double az) override;
    virtual IPState Park() override;
    virtual IPState UnPark() override;
    virtual IPState ControlShutter(ShutterOperation operation) override;
    virtual bool Abort() override;

private:
    bool initPiGPIO();
    enum ShutterAction {
        OPEN,
        OPENING,
        STOPPED,
        CLOSING,
        CLOSED
    };
    ShutterAction currentShutterAction;
    void calibrate();
    INDI::PropertySwitch CalibrateSP {1};
    INDI::PropertyNumber impCount {1};
    INDI::PropertyNumber speed {2};
    INDI::PropertyNumber impToNorthOffset {1};
    enum {
        SPEED_R,
        SPEED_L
    };
    double nextRightImpAz;
    double nextLeftImpAz;
    long lastMeassurements;
    bool prevImpState;
    double targetedAz;
    bool moveToTarget;
    bool shallPark;
};
