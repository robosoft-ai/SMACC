/**
  * @brief Converts Halls sensor data to an angle
  **/

#ifndef HALLS_TO_ANGLE_H
#define HALLS_TO_ANGLE_H

#include <math.h>

class HallsToAngle
{
public:
    inline HallsToAngle();
    inline ~HallsToAngle();

    inline void useCoeffs(float coeff0, float coeff1, float coeff2, float coeff3, float scaleFactor);
    inline void useOffsetGain(float offset, float multiplier);

    inline float getAngleFromHalls(float halls);
    static inline float getAngleFromHalls(float halls, float coeff0, float coeff1, float coeff2, float coeff3, float scaleFactor);
    static inline float getAngleFromHalls(float halls, float offset, float multiplier);
private:
    float coeff0, coeff1, coeff2, coeff3, scaleFactor;
    float offset, multiplier;
    bool use_coeff;

};


HallsToAngle::HallsToAngle()
{

}
HallsToAngle::~HallsToAngle()
{

}

void HallsToAngle::useCoeffs(float coeff0, float coeff1, float coeff2, float coeff3, float scaleFactor)
{
    use_coeff = true;
    this->coeff0 = coeff0;
    this->coeff1 = coeff1;
    this->coeff2 = coeff2;
    this->coeff3 = coeff3;
    this->scaleFactor = scaleFactor;
}

void HallsToAngle::useOffsetGain(float offset, float multiplier)
{
    use_coeff = false;
    this->offset = offset;
    this->multiplier = multiplier;
}

float HallsToAngle::getAngleFromHalls(float halls)
{
    if(use_coeff)
    {
        return getAngleFromHalls(halls, coeff0, coeff1, coeff2, coeff3, scaleFactor);
    }
    else
    {
        return getAngleFromHalls(halls, offset, multiplier);
    }
}

float HallsToAngle::getAngleFromHalls(float halls, float coeff0, float coeff1, float coeff2, float coeff3, float scaleFactor)
{
    halls *= scaleFactor;
    float hallSq = halls*halls;
    return coeff0 + coeff1*halls + coeff2*hallSq + coeff3*hallSq*halls;
}


float HallsToAngle::getAngleFromHalls(float halls, float offset, float multiplier)
{
    return ((halls-offset)*multiplier);
}

#endif
