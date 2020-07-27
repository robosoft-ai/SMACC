
#pragma once
#include <smacc/smacc.h>
#include <smacc/client_base_components/cp_topic_subscriber.h>
#include <boost/optional/optional_io.hpp>

#include <microstrain_mips/SetGyroBiasModel.h>
#include <microstrain_mips/SetSensorVehicleFrameTrans.h>
#include <microstrain_mips/SetTareOrientation.h>
#include <microstrain_mips/SetComplementaryFilter.h>
#include <microstrain_mips/SetFilterHeading.h>
#include <microstrain_mips/SetAccelBias.h>
#include <microstrain_mips/SetAccelNoise.h>
#include <microstrain_mips/SetEstimationControlFlags.h>
#include <microstrain_mips/SetAccelBiasModel.h>
#include <microstrain_mips/SetConingScullingComp.h>
#include <microstrain_mips/SetSensorVehicleFrameOffset.h>
#include <microstrain_mips/SetGyroNoise.h>
#include <microstrain_mips/SetFilterEuler.h>
#include <microstrain_mips/SetReferencePosition.h>
#include <microstrain_mips/SetMagDipAdaptiveVals.h>
#include <microstrain_mips/SetZeroAngleUpdateThreshold.h>
#include <microstrain_mips/SetHardIronValues.h>
#include <microstrain_mips/SetMagNoise.h>
#include <microstrain_mips/SetMagAdaptiveVals.h>
#include <microstrain_mips/SetSoftIronMatrix.h>
#include <microstrain_mips/SetDynamicsMode.h>
#include <microstrain_mips/SetAccelAdaptiveVals.h>
#include <microstrain_mips/SetGyroBias.h>

#include <std_msgs/Empty.h>
#include <std_srvs/Empty.h>
#include <std_srvs/Trigger.h>

#include <sensor_msgs/Imu.h>
#include <microstrain_mips/status_msg.h>

namespace cl_microstrain_mips
{
using namespace microstrain_mips;

class ClMicrostainMips : public smacc::ISmaccClient
{
public:
    boost::optional<std::string> nodeName_;

    smacc::components::CpTopicSubscriber<sensor_msgs::Imu> *imuSubscriber;
    smacc::components::CpTopicSubscriber<sensor_msgs::Imu> *imuFilteredSubscriber;
    smacc::components::CpTopicSubscriber<microstrain_mips::status_msg> *statusSubscriber;

    ClMicrostainMips()
    {
        initialized_ = false;
    }

    virtual void initialize() override
    {
        if (!initialized_)
        {
            if (!nodeName_)
            {
                ROS_ERROR("service client with no service name set. Skipping.");
            }
            else
            {
                ROS_INFO_STREAM("[" << this->getName() << "] Client Service: " << *nodeName_);
                this->initialized_ = true;

                std::string namebase = *nodeName_ + "/";

                this->resetFilterSrv = nh_.serviceClient<std_srvs::Empty>(namebase + "reset_kf");
                this->deviceReportSrv = nh_.serviceClient<std_srvs::Trigger>(namebase + "device_report");
                this->gyroBiasCaptureSrv = nh_.serviceClient<std_srvs::Trigger>(namebase + "gyro_bias_capture");
                this->setSoftIronMatrixSrv = nh_.serviceClient<microstrain_mips::SetSoftIronMatrix>(namebase + "set_soft_iron_matrix");
                this->setComplementaryFilterSrv = nh_.serviceClient<microstrain_mips::SetComplementaryFilter>(namebase + "set_complementary_filter");
                this->setFilterEulerSrv = nh_.serviceClient<microstrain_mips::SetFilterEuler>(namebase + "set_filter_euler");
                this->setFilterHeadingSrv = nh_.serviceClient<microstrain_mips::SetFilterHeading>(namebase + "set_filter_heading");
                this->setAccelBiasModelSrv = nh_.serviceClient<microstrain_mips::SetAccelBiasModel>(namebase + "set_accel_bias_model");
                this->setAccelAdaptiveValsSrv = nh_.serviceClient<microstrain_mips::SetAccelAdaptiveVals>(namebase + "set_accel_adaptive_vals");
                this->setSensorVehicleFrameTransSrv = nh_.serviceClient<microstrain_mips::SetSensorVehicleFrameTrans>(namebase + "set_sensor_vehicle_frame_trans");
                this->setSensorVehicleFrameOffsetSrv = nh_.serviceClient<microstrain_mips::SetSensorVehicleFrameOffset>(namebase + "set_sensor_vehicle_frame_offset");
                this->setAccelBiasSrv = nh_.serviceClient<microstrain_mips::SetAccelBiasModel>(namebase + "set_accel_bias");
                this->setGyroBiasSrv = nh_.serviceClient<microstrain_mips::SetGyroBias>(namebase + "set_gyro_bias");
                this->setHardIronValuesSrv = nh_.serviceClient<microstrain_mips::SetHardIronValues>(namebase + "set_hard_iron_values");

                this->getAccelBiasSrv = nh_.serviceClient<std_srvs::Trigger>(namebase + "get_accel_bias");
                this->getGyroBiasSrv = nh_.serviceClient<std_srvs::Trigger>(namebase + "get_gyro_bias");
                this->getHardIronValuesSrv = nh_.serviceClient<std_srvs::Trigger>(namebase + "get_hard_iron_values");
                this->getSoftIronMatrixSrv = nh_.serviceClient<std_srvs::Trigger>(namebase + "get_soft_iron_matrix");
                this->getSensorVehicleFrameTransSrv = nh_.serviceClient<std_srvs::Trigger>(namebase + "get_sensor_vehicle_frame_trans");
                this->getComplementaryFilterSrv = nh_.serviceClient<std_srvs::Trigger>(namebase + "get_complementary_filter");

                this->setReferencePositionSrv = nh_.serviceClient<microstrain_mips::SetReferencePosition>(namebase + "set_reference_position");
                this->getReferencePositionSrv = nh_.serviceClient<std_srvs::Trigger>(namebase + "get_reference_position");
                this->setConingScullingCompSrv = nh_.serviceClient<microstrain_mips::SetConingScullingComp>(namebase + "set_coning_sculling_comp");
                this->getConingScullingCompSrv = nh_.serviceClient<std_srvs::Trigger>(namebase + "get_coning_sculling_comp");

                this->setEstimationControlFlagsSrv = nh_.serviceClient<microstrain_mips::SetEstimationControlFlags>(namebase + "set_estimation_control_flags");
                this->getEstimationControlFlagsSrv = nh_.serviceClient<std_srvs::Trigger>(namebase + "get_estimation_control_flags");

                this->setDynamicsModeSrv = nh_.serviceClient<microstrain_mips::SetDynamicsMode>(namebase + "set_dynamics_mode");
                this->getBasicStatusSrv = nh_.serviceClient<std_srvs::Trigger>(namebase + "get_basic_status");
                this->getDiagnosticReportSrv = nh_.serviceClient<std_srvs::Trigger>(namebase + "get_diagnostic_report");

                this->setZeroAngleUpdateThresholdSrv = nh_.serviceClient<microstrain_mips::SetZeroAngleUpdateThreshold>(namebase + "set_zero_angle_update_threshold"); //  -
                this->getZeroAngleUpdateThresholdSrv = nh_.serviceClient<std_srvs::Trigger>(namebase + "get_zero_angle_update_threshold");                             //  -

                this->setTareOrientationSrv = nh_.serviceClient<microstrain_mips::SetTareOrientation>(namebase + "set_tare_orientation");

                this->setAccelNoiseSrv = nh_.serviceClient<microstrain_mips::SetAccelNoise>(namebase + "set_accel_noise");
                this->getAccelNoiseSrv = nh_.serviceClient<std_srvs::Trigger>(namebase + "get_accel_noise");

                this->setGyroNoiseSrv = nh_.serviceClient<microstrain_mips::SetGyroNoise>(namebase + "set_gyro_noise");
                this->getGyroNoiseSrv = nh_.serviceClient<std_srvs::Trigger>(namebase + "get_gyro_noise");

                this->setMagNoiseSrv = nh_.serviceClient<microstrain_mips::SetMagNoise>(namebase + "set_mag_noise");
                this->getMagNoiseSrv = nh_.serviceClient<std_srvs::Trigger>(namebase + "get_mag_noise");

                this->setGyroBiasModelSrv = nh_.serviceClient<microstrain_mips::SetGyroBiasModel>(namebase + "set_gyro_bias_model");
                this->getGyroBiasModelSrv = nh_.serviceClient<std_srvs::Trigger>(namebase + "get_gyro_bias_model");

                this->getAccelAdaptiveValsSrv = nh_.serviceClient<std_srvs::Trigger>(namebase + "get_accel_adaptive_vals");

                this->setMagAdaptiveValsSrv = nh_.serviceClient<microstrain_mips::SetMagAdaptiveVals>(namebase + "set_mag_adaptive_vals");
                this->getMagAdaptiveValsSrv = nh_.serviceClient<std_srvs::Trigger>(namebase + "get_mag_adaptive_vals");

                this->setMagDipAdaptiveValsSrv = nh_.serviceClient<microstrain_mips::SetMagDipAdaptiveVals>(namebase + "set_mag_dip_adaptive_vals");
                this->getAccelBiasModelSrv = nh_.serviceClient<std_srvs::Trigger>(namebase + "get_accel_bias_model");

                this->getMagDipAdaptiveValsSrv = nh_.serviceClient<std_srvs::Trigger>(namebase + "get_mag_dip_adaptive_vals");
                this->getSensorVehicleFrameOffsetSrv = nh_.serviceClient<std_srvs::Trigger>(namebase + "get_sensor_vehicle_frame_offset");
                this->getGynamicsModeSrv = nh_.serviceClient<std_srvs::Trigger>(namebase + "get_dynamics_mode");
            }
        }
    }

    template <typename TOrthogonal, typename TSourceObject>
    void onOrthogonalAllocation()
    {
        this->imuSubscriber = this->createComponent<TSourceObject, TOrthogonal, smacc::components::CpTopicSubscriber<sensor_msgs::Imu>>("imu/data");
        this->imuFilteredSubscriber = this->createComponent<TSourceObject, TOrthogonal, smacc::components::CpTopicSubscriber<sensor_msgs::Imu>>("filtered/imu/data");
        this->statusSubscriber = this->createComponent<TSourceObject, TOrthogonal, smacc::components::CpTopicSubscriber<microstrain_mips::status_msg>>("imu/data");
    }

    void resetFilter()
    {
        std_srvs::Empty::Request req;
        std_srvs::Empty::Response res;

        resetFilterSrv.call(req, res);
    }

    bool deviceReport()
    {
        std_srvs::Trigger::Request req;
        std_srvs::Trigger::Response res;

        deviceReportSrv.call(req, res);
        return res.success;
    }

    bool gyroBiasCapture()
    {
        std_srvs::Trigger::Request req;
        std_srvs::Trigger::Response res;

        gyroBiasCaptureSrv.call(req, res);
        return res.success;
    }

    bool setSoftIronMatrix(const geometry_msgs::Vector3 &soft_iron_1, const geometry_msgs::Vector3 &soft_iron_2, const geometry_msgs::Vector3 &soft_iron_3)
    {
        SetSoftIronMatrix::Request req;
        SetSoftIronMatrix::Response res;
        req.soft_iron_1 = soft_iron_1;
        req.soft_iron_2 = soft_iron_2;
        req.soft_iron_3 = soft_iron_3;

        setSoftIronMatrixSrv.call(req, res);
        return res.success;
    }

    bool setComplementaryFilter(int8_t north_comp_enable, int8_t up_comp_enable, float north_comp_time_const, float up_comp_time_const)
    {
        SetComplementaryFilter::Request req;
        SetComplementaryFilter::Response res;

        req.north_comp_enable = north_comp_enable;
        req.up_comp_enable = up_comp_enable;
        req.north_comp_time_const = north_comp_time_const;
        req.up_comp_time_const = up_comp_time_const;

        setComplementaryFilterSrv.call(req, res);
        return res.success;
    }

    bool setFilterEulerService(const geometry_msgs::Vector3 &angle)
    {
        SetFilterEuler::Request req;
        SetFilterEuler::Response res;
        req.angle = angle;

        setFilterEulerSrv.call(req, res);
        return res.success;
    }

    bool setFilterHeading(float angle)
    {
        SetFilterHeading::Request req;
        SetFilterHeading::Response res;
        req.angle = angle;

        setFilterHeadingSrv.call(req, res);
        return res.success;
    }

    bool setAccelBiasModel(const geometry_msgs::Vector3 &noise_vector, const geometry_msgs::Vector3 &beta_vector)
    {
        SetAccelBiasModel::Request req;
        SetAccelBiasModel::Response res;
        req.noise_vector = noise_vector;
        req.beta_vector = beta_vector;

        setAccelBiasModelSrv.call(req, res);
        return res.success;
    }

    bool setAccelAdaptiveVals(float enable, float low_pass_cutoff, float min_1sigma, float low_limit, float high_limit, float low_limit_1sigma, float high_limit_1sigma)
    {
        SetAccelAdaptiveVals::Request req;
        SetAccelAdaptiveVals::Response res;

        req.enable = enable;
        req.low_pass_cutoff = low_pass_cutoff;
        req.min_1sigma = min_1sigma;
        req.low_limit = low_limit;
        req.high_limit = high_limit;
        req.low_limit_1sigma = low_limit_1sigma;
        req.high_limit_1sigma = high_limit_1sigma;

        setAccelAdaptiveValsSrv.call(req, res);
        return res.success;
    }

    bool setSensorVehicleFrameTrans(const geometry_msgs::Vector3 &angle)
    {
        SetSensorVehicleFrameTrans::Request req;
        SetSensorVehicleFrameTrans::Response res;
        req.angle = angle;

        setSensorVehicleFrameTransSrv.call(req, res);
        return res.success;
    }

    bool setSensorVehicleFrameOffset(const geometry_msgs::Vector3 &offset)
    {
        SetSensorVehicleFrameOffset::Request req;
        SetSensorVehicleFrameOffset::Response res;
        req.offset = offset;

        setSensorVehicleFrameOffsetSrv.call(req, res);
        return res.success;
    }

    bool setAccelBias(const geometry_msgs::Vector3 &bias)
    {
        SetAccelBias::Request req;
        SetAccelBias::Response res;
        req.bias = bias;

        setAccelBiasSrv.call(req, res);
        return res.success;
    }

    bool setGyroBias(const geometry_msgs::Vector3 &bias)
    {
        SetGyroBias::Request req;
        SetGyroBias::Response res;
        req.bias = bias;

        setGyroBiasSrv.call(req, res);
        return res.success;
    }

    bool setHardIronValues(const geometry_msgs::Vector3 &bias)
    {
        SetHardIronValues::Request req;
        SetHardIronValues::Response res;
        req.bias = bias;

        setHardIronValuesSrv.call(req, res);
        return res.success;
    }

    bool getAccelBias()
    {
        std_srvs::Trigger::Request req;
        std_srvs::Trigger::Response res;

        getAccelBiasSrv.call(req, res);
        return res.success;
    }

    bool getGyroBias()
    {
        std_srvs::Trigger::Request req;
        std_srvs::Trigger::Response res;

        getGyroBiasSrv.call(req, res);
        return res.success;
    }

    bool getHardIronValues()
    {
        std_srvs::Trigger::Request req;
        std_srvs::Trigger::Response res;

        getHardIronValuesSrv.call(req, res);
        return res.success;
    }

    bool getSoftIronMatrix()
    {
        std_srvs::Trigger::Request req;
        std_srvs::Trigger::Response res;

        getSoftIronMatrixSrv.call(req, res);
        return res.success;
    }

    bool getSensorVehicleFrameTrans()
    {
        std_srvs::Trigger::Request req;
        std_srvs::Trigger::Response res;

        getSensorVehicleFrameTransSrv.call(req, res);
        return res.success;
    }

    bool getComplementaryFilter()
    {
        std_srvs::Trigger::Request req;
        std_srvs::Trigger::Response res;

        getComplementaryFilterSrv.call(req, res);
        return res.success;
    }

    bool setReferencePosition(const geometry_msgs::Vector3 &position)
    {
        SetReferencePosition::Request req;
        SetReferencePosition::Response res;
        req.position = position;

        setReferencePositionSrv.call(req, res);
        return res.success;
    }

    bool getReferencePosition()
    {
        std_srvs::Trigger::Request req;
        std_srvs::Trigger::Response res;

        getReferencePositionSrv.call(req, res);
        return res.success;
    }

    bool setConingScullingComp(int8_t enable)
    {
        SetConingScullingComp::Request req;
        SetConingScullingComp::Response res;
        req.enable = enable;

        setConingScullingCompSrv.call(req, res);
        return res.success;
    }

    bool getConingScullingComp()
    {
        std_srvs::Trigger::Request req;
        std_srvs::Trigger::Response res;

        getConingScullingCompSrv.call(req, res);
        return res.success;
    }

    bool setEstimationControlFlags(int8_t flag)
    {
        SetEstimationControlFlags::Request req;
        SetEstimationControlFlags::Response res;
        req.flag = flag;

        setEstimationControlFlagsSrv.call(req, res);
        return res.success;
    }

    bool getEstimationControlFlags()
    {
        std_srvs::Trigger::Request req;
        std_srvs::Trigger::Response res;

        getEstimationControlFlagsSrv.call(req, res);
        return res.success;
    }

    bool setDynamicsMode(int8_t mode)
    {
        SetDynamicsMode::Request req;
        SetDynamicsMode::Response res;
        req.mode = mode;

        setDynamicsModeSrv.call(req, res);
        return res.success;
    }

    bool getBasicStatus()
    {
        std_srvs::Trigger::Request req;
        std_srvs::Trigger::Response res;

        getBasicStatusSrv.call(req, res);
        return res.success;
    }

    bool getDiagnosticReport()
    {
        std_srvs::Trigger::Request req;
        std_srvs::Trigger::Response res;

        getDiagnosticReportSrv.call(req, res);
        return res.success;
    }

    bool setZeroAngleUpdateThreshold(int8_t enable, float threshold)
    {
        SetZeroAngleUpdateThreshold::Request req;
        SetZeroAngleUpdateThreshold::Response res;
        req.enable = enable;
        req.threshold = threshold;

        setZeroAngleUpdateThresholdSrv.call(req, res);
        return res.success;
    }

    bool getZeroAngleUpdateThreshold()
    {
        std_srvs::Trigger::Request req;
        std_srvs::Trigger::Response res;

        getZeroAngleUpdateThresholdSrv.call(req, res);
        return res.success;
    }

    bool setTareOrientation(int8_t axis)
    {
        SetTareOrientation::Request req;
        SetTareOrientation::Response res;
        req.axis = axis;

        setTareOrientationSrv.call(req, res);
        return res.success;
    }

    bool setAccelNoise(const geometry_msgs::Vector3 &noise)
    {
        SetAccelNoise::Request req;
        SetAccelNoise::Response res;
        req.noise = noise;

        setAccelNoiseSrv.call(req, res);
        return res.success;
    }

    bool getAccelNoise()
    {
        std_srvs::Trigger::Request req;
        std_srvs::Trigger::Response res;

        getAccelNoiseSrv.call(req, res);
        return res.success;
    }

    bool setGyroNoise(const geometry_msgs::Vector3 &noise)
    {
        SetGyroNoise::Request req;
        SetGyroNoise::Response res;
        req.noise = noise;

        setGyroNoiseSrv.call(req, res);
        return res.success;
    }

    bool getGyroNoise()
    {
        std_srvs::Trigger::Request req;
        std_srvs::Trigger::Response res;

        getGyroNoiseSrv.call(req, res);
        return res.success;
    }

    bool setMagNoise(const geometry_msgs::Vector3 &noise)
    {
        SetMagNoise::Request req;
        SetMagNoise::Response res;

        req.noise = noise;

        setMagNoiseSrv.call(req, res);
        return res.success;
    }

    bool getMagNoise()
    {
        std_srvs::Trigger::Request req;
        std_srvs::Trigger::Response res;

        getMagNoiseSrv.call(req, res);
        return res.success;
    }

    bool setGyroBiasModel(const geometry_msgs::Vector3 &noise_vector, const geometry_msgs::Vector3 &beta_vector)
    {
        SetGyroBiasModel::Request req;
        SetGyroBiasModel::Response res;
        req.noise_vector = noise_vector;
        req.beta_vector = beta_vector;

        setGyroBiasModelSrv.call(req, res);
        return res.success;
    }

    bool getGyroBiasModel()
    {
        std_srvs::Trigger::Request req;
        std_srvs::Trigger::Response res;

        getGyroBiasModelSrv.call(req, res);
        return res.success;
    }

    bool getAccelAdaptiveVals()
    {
        std_srvs::Trigger::Request req;
        std_srvs::Trigger::Response res;

        getAccelAdaptiveValsSrv.call(req, res);
        return res.success;
    }

    bool setMagAdaptiveVals(float enable, float low_pass_cutoff, float min_1sigma, float low_limit, float high_limit, float low_limit_1sigma, float high_limit_1sigma)
    {
        SetMagAdaptiveVals::Request req;
        SetMagAdaptiveVals::Response res;

        req.enable = enable;
        req.low_pass_cutoff = low_pass_cutoff;
        req.min_1sigma = min_1sigma;
        req.low_limit = low_limit;
        req.high_limit = high_limit;
        req.low_limit_1sigma = low_limit_1sigma;
        req.high_limit_1sigma = high_limit_1sigma;

        setMagAdaptiveValsSrv.call(req, res);
        return res.success;
    }

    bool getMagAdaptiveVals()
    {
        std_srvs::Trigger::Request req;
        std_srvs::Trigger::Response res;

        getMagAdaptiveValsSrv.call(req, res);
        return res.success;
    }

    bool setMagDipAdaptiveVals(float enable, float low_pass_cutoff, float min_1sigma, float high_limit, float high_limit_1sigma)
    {
        SetMagDipAdaptiveVals::Request req;
        SetMagDipAdaptiveVals::Response res;

        req.enable = enable;
        req.low_pass_cutoff = low_pass_cutoff;
        req.min_1sigma = min_1sigma;
        req.high_limit = high_limit;
        req.high_limit_1sigma = high_limit_1sigma;

        setMagDipAdaptiveValsSrv.call(req, res);
        return res.success;
    }

    bool getAccelBiasModel()
    {
        std_srvs::Trigger::Request req;
        std_srvs::Trigger::Response res;

        getAccelBiasModelSrv.call(req, res);
        return res.success;
    }

    bool getMagDipAdaptiveVals()
    {
        std_srvs::Trigger::Request req;
        std_srvs::Trigger::Response res;

        getMagDipAdaptiveValsSrv.call(req, res);
        return res.success;
    }

    bool getSensorVehicleFrameOffset()
    {
        std_srvs::Trigger::Request req;
        std_srvs::Trigger::Response res;

        getSensorVehicleFrameOffsetSrv.call(req, res);
        return res.success;
    }

    bool getGynamicsMode()
    {
        std_srvs::Trigger::Request req;
        std_srvs::Trigger::Response res;

        getGynamicsModeSrv.call(req, res);
        return res.success;
    }

protected:
    ros::NodeHandle nh_;
    bool initialized_;

    ros::ServiceClient resetFilterSrv;                 // reset_kf - std_srvs::Empty
    ros::ServiceClient deviceReportSrv;                // device_report - std_srvs::Trigger
    ros::ServiceClient gyroBiasCaptureSrv;             // gyro_bias_capture - std_srvs::Trigger
    ros::ServiceClient setSoftIronMatrixSrv;           // set_soft_iron_matrix - microstrain_mips::SetSoftIronMatrix
    ros::ServiceClient setComplementaryFilterSrv;      //set_complementary_filter - microstrain_mips::SetComplementaryFilter
    ros::ServiceClient setFilterEulerSrv;              // set_filter_euler - microstrain_mips::SetFilterEuler
    ros::ServiceClient setFilterHeadingSrv;            // set_filter_heading - microstrain_mips::SetFilterHeading
    ros::ServiceClient setAccelBiasModelSrv;           // set_accel_bias_model - microstrain_mips::SetAccelBiasModel
    ros::ServiceClient setAccelAdaptiveValsSrv;        // set_accel_adaptive_vals - microstrain_mips::SetAccelAdaptiveVals
    ros::ServiceClient setSensorVehicleFrameTransSrv;  // set_sensor_vehicle_frame_trans - microstrain_mips::SetSensorVehicleFrameTrans
    ros::ServiceClient setSensorVehicleFrameOffsetSrv; // set_sensor_vehicle_frame_offset - microstrain_mips::SetSensorVehicleFrameOffset
    ros::ServiceClient setAccelBiasSrv;                // set_accel_bias - microstrain_mips::SetAccelBiasModel
    ros::ServiceClient setGyroBiasSrv;                 // set_gyro_bias - microstrain_mips::SetGyroBias
    ros::ServiceClient setHardIronValuesSrv;           // set_hard_iron_values - microstrain_mips::SetHardIronValues
    ros::ServiceClient getAccelBiasSrv;                // get_accel_bias - std_srvs::Trigger
    ros::ServiceClient getGyroBiasSrv;                 // get_gyro_bias - std_srvs::Trigger
    ros::ServiceClient getHardIronValuesSrv;           // get_hard_iron_values - std_srvs::Trigger
    ros::ServiceClient getSoftIronMatrixSrv;           // get_soft_iron_matrix - std_srvs::Trigger
    ros::ServiceClient getSensorVehicleFrameTransSrv;  // get_sensor_vehicle_frame_trans - std_srvs::Trigger
    ros::ServiceClient getComplementaryFilterSrv;      // get_complementary_filter - std_srvs::Trigger
    ros::ServiceClient setReferencePositionSrv;        // set_reference_position - microstrain_mips::SetReferencePosition
    ros::ServiceClient getReferencePositionSrv;        // get_reference_position - std_srvs::Trigger
    ros::ServiceClient setConingScullingCompSrv;       // set_coning_sculling_comp - microstrain_mips::SetConingScullingComp
    ros::ServiceClient getConingScullingCompSrv;       // get_coning_sculling_comp - std_srvs::Trigger
    ros::ServiceClient setEstimationControlFlagsSrv;   // set_estimation_control_flags - microstrain_mips::SetEstimationControlFlags
    ros::ServiceClient getEstimationControlFlagsSrv;   // get_estimation_control_flags - std_srvs::Trigger
    ros::ServiceClient setDynamicsModeSrv;             // set_dynamics_mode - microstrain_mips::SetDynamicsMode
    ros::ServiceClient getBasicStatusSrv;              // get_basic_status - std_srvs::Trigger
    ros::ServiceClient getDiagnosticReportSrv;         // get_diagnostic_report - std_srvs::Trigger
    ros::ServiceClient setZeroAngleUpdateThresholdSrv; // set_zero_angle_update_threshold - microstrain_mips::SetZeroAngleUpdateThreshold
    ros::ServiceClient getZeroAngleUpdateThresholdSrv; // get_zero_angle_update_threshold - std_srvs::Trigger
    ros::ServiceClient setTareOrientationSrv;          // set_tare_orientation - microstrain_mips::SetTareOrientation
    ros::ServiceClient setAccelNoiseSrv;               // set_accel_noise - microstrain_mips::SetAccelNoise
    ros::ServiceClient getAccelNoiseSrv;               // get_accel_noise - std_srvs::Trigger
    ros::ServiceClient setGyroNoiseSrv;                // set_gyro_noise - microstrain_mips::SetGyroNoise
    ros::ServiceClient getGyroNoiseSrv;                // get_gyro_noise - std_srvs::Trigger
    ros::ServiceClient setMagNoiseSrv;                 // set_mag_noise - microstrain_mips::SetMagNoise
    ros::ServiceClient getMagNoiseSrv;                 // get_mag_noise - std_srvs::Trigger
    ros::ServiceClient setGyroBiasModelSrv;            // set_gyro_bias_model - microstrain_mips::SetGyroBiasModel
    ros::ServiceClient getGyroBiasModelSrv;            // get_gyro_bias_model - std_srvs::Trigger
    ros::ServiceClient getAccelAdaptiveValsSrv;        // get_accel_adaptive_vals - std_srvs::Trigger
    ros::ServiceClient setMagAdaptiveValsSrv;          // set_mag_adaptive_vals - microstrain_mips::SetMagAdaptiveVals
    ros::ServiceClient getMagAdaptiveValsSrv;          // get_mag_adaptive_vals - std_srvs::Trigger
    ros::ServiceClient setMagDipAdaptiveValsSrv;       // set_mag_dip_adaptive_vals - microstrain_mips::SetMagDipAdaptiveVals
    ros::ServiceClient getAccelBiasModelSrv;           // get_accel_bias_model - std_srvs::Trigger
    ros::ServiceClient getMagDipAdaptiveValsSrv;       // get_mag_dip_adaptive_vals - std_srvs::Trigger
    ros::ServiceClient getSensorVehicleFrameOffsetSrv; // get_sensor_vehicle_frame_offset - std_srvs::Trigger
    ros::ServiceClient getGynamicsModeSrv;             // get_dynamics_mode - std_srvs::Trigger
};
}