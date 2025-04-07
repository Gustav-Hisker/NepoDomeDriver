#pragma once

#include "libindi/indidome.h"

class NepoDomeDriver : public INDI::Dome
{
public:
    NepoDomeDriver();
    virtual ~NepoDomeDriver() = default;

    virtual bool initProperties() override;
    virtual const char *getDefaultName() override;
    //virtual bool saveConfigItems(FILE *fp) override;

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

    // Parking
    virtual bool SetCurrentPark() override;
    virtual bool SetDefaultPark() override;

private:
    bool initPiGPIO();
    double targetedAz;
    enum ShutterAction {
        OPEN,
        OPENING,
        STOPPED,
        CLOSING,
        CLOSED
    };
    ShutterAction currentShutterAction;
};
