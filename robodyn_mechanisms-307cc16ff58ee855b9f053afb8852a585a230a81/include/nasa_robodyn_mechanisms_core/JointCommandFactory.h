#ifndef JOINTCOMMANDFACTORY_H_
#define JOINTCOMMANDFACTORY_H_

#include <boost/make_shared.hpp>
#include "nasa_robodyn_mechanisms_core/JointCommandSeriesElastic.h"
#include "nasa_robodyn_mechanisms_core/JointCommandRigid.h"
#include "nasa_robodyn_mechanisms_core/JointCommandGripper.h"
#include "nasa_robodyn_mechanisms_core/JointCommandWrist.h"
#include "nasa_robodyn_mechanisms_core/JointCommandFinger.h"

namespace JointCommandFactory
{
    JointCommandInterfacePtr generate(const std::string& mechanism, const JointCommandInterface::IoFunctions& io);

namespace Private
{
    std::string getJointTypeFromParameterFile(const std::string& parameterFile);
};
};

#endif /* JOINTCOMMANDFACTORY_H_ */
