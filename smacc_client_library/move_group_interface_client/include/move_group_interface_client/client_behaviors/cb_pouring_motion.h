/*****************************************************************************************************************
 * ReelRobotix Inc. - Software License Agreement      Copyright (c) 2018-2020
 * 	 Authors: Pablo Inigo Blasco, Brett Aldrich
 *
 ******************************************************************************************************************/
#pragma once 

#include "cb_move_end_effector_trajectory.h"
#include "tf/transform_datatypes.h"
#include <tf/transform_listener.h>

namespace cl_move_group_interface
{
    class CbCircularPouringMotion : public CbMoveEndEffectorTrajectory
    {
    public:
        boost::optional<double> angularSpeed_rad_s_;

        boost::optional<double> linearSpeed_m_s_;

        CbCircularPouringMotion(const geometry_msgs::PointStamped &pivotPoint, double deltaHeight, std::string tipLink = "");

        virtual void generateTrajectory() override;

        virtual void createMarkers() override;
        
        void getCurrentEndEffectorPose(tf::StampedTransform& currentEndEffectorTransform);

    protected:
        geometry_msgs::PointStamped pivotPoint_;

        // distance in meters from the initial pose to the bottom/top direction in z axe
        double deltaHeight_;

    private:
        void computeCurrentEndEffectorPoseRelativeToPivot();
    };

} // namespace cl_move_group_interface