/***************************************************************************//**
 *
 * @file JointControlManagerSeriesElastic.h
 *
 * @brief This file contains the declaration of the JointControlManagerSeriesElastic class.
 *
 ******************************************************************************/
#ifndef JOINTCONTROLMANAGERSERIESELASTIC_H_
#define JOINTCONTROLMANAGERSERIESELASTIC_H_

#include "nasa_r2_config_core/NodeRegisterManager.h"
#include "nasa_robodyn_mechanisms_core/JointControlManagerInterface.h"
#include "nasa_robodyn_mechanisms_core/JointControlActualFsmSeriesElastic.h"
#include "nasa_robodyn_mechanisms_core/JointControlCommandFsmSeriesElastic.h"

class JointControlManagerSeriesElastic : public JointControlManagerInterface
{
public:
    JointControlManagerSeriesElastic(const std::string&, IoFunctions, double, const std::string&, NodeRegisterManagerPtr);
    virtual ~JointControlManagerSeriesElastic();

    bool                                  verifyStates(void);
    bool                                  verifyControlModeState(void);
    bool                                  verifyCommandModeState(void);
    bool                                  verifyCalibrationModeState(void);
    bool                                  verifyClearFaultModeState(void);
    nasa_r2_common_msgs::JointControlData getActualStates(void);
    nasa_r2_common_msgs::JointControlData getCommandStates(void);
    void                                  setCommandStates(nasa_r2_common_msgs::JointControlData);
    std::string                           buildFaultString();

protected:
    NodeRegisterManagerPtr nodeRegisterManager;

    void setParameters();

    // Status bit names
    std::string ProcAliveStatusName;
    std::string CommAliveStatusName;
    std::string BridgeFaultStatusName;
    std::string JointFaultStatusName;
    std::string BusVoltageFaultStatusName;
    std::string ApsFaultStatusName;
    std::string Aps1TolFaultStatusName;
    std::string Aps2TolFaultStatusName;
    std::string EncDriftFaultStatusName;
    std::string VelocityFaultStatusName;
    std::string LimitFaultStatusName;
    std::string CoeffsLoadedStatusName;
    std::string CurrentFaultStatusName;

    // Control bit names
    std::string InitAPS2toAPS1ControlName;
};

#endif
