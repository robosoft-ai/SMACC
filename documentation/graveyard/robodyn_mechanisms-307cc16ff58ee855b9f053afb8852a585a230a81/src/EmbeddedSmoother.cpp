#include "nasa_robodyn_mechanisms_core/EmbeddedSmoother.h"
#include <cmath>

EmbeddedSmoother::EmbeddedSmoother(const Settings& settings_in)
    : settings(settings_in)
    , prevDesiredPos(0.)
    , smoothedPos(0.)
    , smoothedVel(0.)
    , smoothedAcc(0.)
    , desiredDir(0.)
    , velDir(0.)
    , commandVel(0.)
    , velError(0.)
{
}

EmbeddedSmoother::~EmbeddedSmoother()
{}

double EmbeddedSmoother::update(double desiredPosition, double desiredVelocity)
{
    if (prevDesiredPos != desiredPosition || desiredDir == 0.)
    {
        desiredDir = ((desiredPosition - smoothedPos) < 0. ? -1. : 1.);
        prevDesiredPos = desiredPosition;
    }

    commandVel = fabs(desiredVelocity);
    if (commandVel < settings.minVel)
    {
        commandVel = settings.minVel;
    }

    velDir = (desiredPosition - smoothedPos) < 0. ? -1. : 1.;
    if (velDir != desiredDir)
    {
        commandVel = 0.;
    }

    velError = commandVel * velDir - smoothedVel;
    if (fabs(velError) < 1E-9)
    {
        velError = 0.;
    }

    smoothedAcc = settings.accGain * velError;
    if (smoothedAcc < -settings.maxAcc)
    {
        smoothedAcc = -settings.maxAcc;
    }
    else if (smoothedAcc > settings.maxAcc)
    {
        smoothedAcc = settings.maxAcc;
    }

    smoothedVel += smoothedAcc * settings.timestep;
    if (fabs(smoothedVel) < 1E-9)
    {
        smoothedVel = 0.;
    }

    smoothedPos += smoothedVel * settings.timestep;

    return smoothedPos;
}

double EmbeddedSmoother::reset(double desiredPosition, double desiredVelocity)
{
    desiredDir = 0.;
    prevDesiredPos = desiredPosition;
    smoothedPos = desiredPosition;
    smoothedVel = desiredVelocity;

    return smoothedPos;
}
