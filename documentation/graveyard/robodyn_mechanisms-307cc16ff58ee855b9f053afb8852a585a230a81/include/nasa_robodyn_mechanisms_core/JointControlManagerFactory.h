#ifndef JOINTCONTROLMANAGERFACTORY_H_
#define JOINTCONTROLMANAGERFACTORY_H_

#include <boost/make_shared.hpp>
#include "nasa_r2_config_core/NodeRegisterManager.h"
#include "nasa_robodyn_mechanisms_core/JointControlManagerSeriesElastic.h"
#include "nasa_robodyn_mechanisms_core/JointControlManagerGripper.h"
#include "nasa_robodyn_mechanisms_core/JointControlManagerWrist.h"
#include "nasa_robodyn_mechanisms_core/JointControlManagerFinger.h"

namespace JointControlManagerFactory
{
    JointControlManagerPtr generate(const std::string& mechanism, const JointControlCommonInterface::IoFunctions& io, double timeLimit, NodeRegisterManagerPtr nodeRegisterManager);

namespace Private
{
    std::string getPropertyFromFile(const std::string& parameterFile, const std::string& property);
};
};

#endif /* JOINTCOMMANDFACTORY_H_ */
