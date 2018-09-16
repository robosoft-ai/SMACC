/***************************************************************************//**
 *
 * @file JointControlActualFsmSeriesElastic.h
 *
 * @brief This file contains the declaration of the JointControlActualFsmSeriesElastic class.
 *
 ******************************************************************************/
#ifndef JOINTCONTROLACTUALFSMSERIESELASTIC_H_
#define JOINTCONTROLACTUALFSMSERIESELASTIC_H_

#include "nasa_robodyn_mechanisms_core/JointControlActualFsmInterface.h"
#include "nasa_r2_config_core/NodeRegisterManager.h"

class JointControlActualFsmSeriesElastic : public JointControlActualFsmInterface
{
public:
    JointControlActualFsmSeriesElastic(const std::string&, IoFunctions, NodeRegisterManagerPtr);
    virtual ~JointControlActualFsmSeriesElastic();

    nasa_r2_common_msgs::JointControlData getStates(void);
    void                                  updateControlModeState(void);
    void                                  updateCommandModeState(void);
    void                                  updateCalibrationModeState(void);
    void                                  updateClearFaultModeState(void);
    void                                  updateCoeffState(void);

protected:
    void setParameters();

    // Status bit names
    std::string ProcAliveStatusName;
    std::string CommAliveStatusName;
    std::string JointFaultStatusName;
    std::string CoeffsLoadedStatusName;
    std::string MotorEnableStatusName;
    std::string BrakeReleaseStatusName;
    std::string BridgeEnableStatusName;
    std::string MotComSourceStatusName;
    std::string PosComVelocityStatusName;
    std::string ControlModeStatusName;
    std::string CalibrationModeStatusName;
    std::string ClearFaultStatusName;

    // Control bit names
    std::string BootEnableControlName;

protected:
    NodeRegisterManagerPtr nodeRegisterManager;
};

#endif
