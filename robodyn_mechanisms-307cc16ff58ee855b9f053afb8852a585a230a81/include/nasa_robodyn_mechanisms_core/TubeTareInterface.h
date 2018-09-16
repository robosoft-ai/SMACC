#ifndef TUBETAREINTERFACE_H
#define TUBETAREINTERFACE_H

#include <vector>
#include <cstring>
#include <boost/function.hpp>
#include "tinyxml.h"
#include "nasa_common_utilities/Logger.h"
#include <cstring>
#include <boost/shared_ptr.hpp>
#include <ros/builtin_message_traits.h>
#include "nasa_r2_config_core/ApiMap.h"

class TubeTareInterface
{
public:
    class IoFunctions
    {
    public:
        boost::function<uint16_t(const std::string&)>                 getUInt16;
        boost::function<void(const std::string&, uint16_t)>           setUInt16;
        boost::function<uint32_t(const std::string&)>                 getUInt32;
        boost::function<void(const std::string&, uint32_t)>           setUInt32;
        boost::function<int16_t(const std::string&)>                  getInt16;
        boost::function<void(const std::string&, int16_t)>            setInt16;
        boost::function<int32_t(const std::string&)>                  getInt32;
        boost::function<void(const std::string&, int32_t)>            setInt32;
        boost::function<float(const std::string&)>                    getFloat;
        boost::function<void(const std::string&, float)>              setFloat;
        boost::function<float(const std::string&)>                    getMotorCoeff;
        boost::function<std::vector<std::string>(const std::string&)> getJointNames;
        boost::function<std::vector<std::string>(const std::string&)> getActuatorNames;
        boost::function<std::string(const std::string&)>              getCommandFile;
        boost::function<bool(const std::string&)>                     hasLiveCoeff;
        boost::function<float(const std::string&)>                    getLiveCoeff;
        boost::function<void(const std::string&, float)>              setLiveCoeff;
    };

    virtual void setStop(const std::vector<std::string>& actuators = std::vector<std::string>()) = 0; //the motcom command for stop (0)
    virtual bool isMoving() = 0; //checks if the velocity is 0
    virtual void setRelease(const std::vector<std::string>& actuators = std::vector<std::string>()) = 0; //the motcom command for release (limited)
    virtual void getEncoderTarePos() = 0; //perform the tare (TareEncoderAbsolute)
    virtual void setTighten(const std::vector<std::string>& actuators = std::vector<std::string>()) = 0; //set motcom to tighten position (limited)
    virtual int  goodPosition() = 0; //checks if near end of travel
    virtual void getSliderTarePos() = 0;//SliderVectorType &encoderOffsets, SliderVectorType &sliderOffsets);
    virtual void getTensionTareValue() = 0;
    virtual void resetTensionCounter() = 0;

    std::string mechanism;
    std::vector<std::string> roboDynJoints;
    std::vector<std::string> roboDynActuators;
protected:
    TubeTareInterface(const std::string& mechanism, IoFunctions io)
        : mechanism(mechanism), io(io)
    {}
    virtual ~TubeTareInterface() {}

    IoFunctions io;
};

typedef boost::shared_ptr<TubeTareInterface> TubeTareInterfacePtr;


#endif // TUBETAREINTERFACE_H
