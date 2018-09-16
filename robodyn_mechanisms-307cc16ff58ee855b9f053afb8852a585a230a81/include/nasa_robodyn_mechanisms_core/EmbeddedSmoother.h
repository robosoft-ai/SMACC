/*!
 * @file  EmbeddedSmoother.h
 * @brief Defines the EmbeddedSmoother class which smooths out slower rate breadcrumbs.
 * @author Ross C. Taylor
 * @date 2013-08-27
 */

#ifndef EMBEDDEDSMOOTHER_H
#define EMBEDDEDSMOOTHER_H

/***************************************************************************//**
 *
 * @brief This class provides a smoother for slower rate breadcrumbs
 *
 ******************************************************************************/

class EmbeddedSmoother
{
public:
    struct Settings
    {
        Settings() : timestep(0.), minVel(0.), maxAcc(0.), accGain(0.) {}

        double timestep;
        double minVel;
        double maxAcc;
        double accGain;
    };

    EmbeddedSmoother(const Settings& settings_in);
    ~EmbeddedSmoother();

    double update(double desiredPosition, double desiredVelocity);
    double reset(double desiredPosition, double desiredVelocity);

private:
    Settings settings;

    double prevDesiredPos;
    double smoothedPos;
    double smoothedVel;
    double smoothedAcc;
    double desiredDir;
    double velDir;
    double commandVel;
    double velError;
};

#endif
