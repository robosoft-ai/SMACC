#include "cb_move_end_effector_trajectory.h"
#include "tf/transform_datatypes.h"
#include <tf/transform_listener.h>

namespace cl_move_group_interface
{
    class CbCircularPivotMotion : public CbMoveEndEffectorTrajectory
    {

    public:
        boost::optional<double> angularSpeed_rad_s_;
        boost::optional<double> linearSpeed_m_s_;

        // if not specified it would be used the current robot pose
        boost::optional<geometry_msgs::Pose> relativeInitialPose_;

        CbCircularPivotMotion()
        {
        }

        CbCircularPivotMotion(const geometry_msgs::PoseStamped &planePivotPose, double deltaRadians)
            : planePivotPose_(planePivotPose)
             , deltaRadians_(deltaRadians)
        {

        }

        CbCircularPivotMotion(const geometry_msgs::PoseStamped &planePivotPose, const geometry_msgs::Pose &relativeInitialPose, double deltaRadians)
            : planePivotPose_(planePivotPose), relativeInitialPose_(relativeInitialPose), deltaRadians_(deltaRadians)
        {
        }

        virtual void generateTrajectory() override
        {
            if(!relativeInitialPose_)
            {
                auto currentRobotEndEffectorPose = this->movegroupClient_->moveGroupClientInterface.getCurrentPose();

                tf::TransformListener tfListener;
                tf::StampedTransform globalBaseLink;

                try
                {
                    // we define here the global frame as the pivot frame id
                    tfListener.waitForTransform(currentRobotEndEffectorPose.header.frame_id, planePivotPose_.header.frame_id,ros::Time(0), ros::Duration(10));
                    tfListener.lookupTransform(currentRobotEndEffectorPose.header.frame_id, planePivotPose_.header.frame_id,ros::Time(0),globalBaseLink);
                }
                catch(const std::exception& e)
                {
                    std::cerr << e.what() << '\n';
                }

                tf::Transform endEffectorInBaseLinkFrame;
                tf::poseMsgToTF(currentRobotEndEffectorPose.pose, endEffectorInBaseLinkFrame);

                tf::Transform endEffectorInPivotFrame =  globalBaseLink * endEffectorInBaseLinkFrame; // pose composition

                // now pivot and EndEffector share a common reference frame (let say map)
                // now get the current pose from the pivot reference frame with inverse composition
                tf::Transform pivotTransform;
                tf::poseMsgToTF(planePivotPose_.pose,pivotTransform);
                tf::Transform invertedNewReferenceFrame = pivotTransform.inverse();

                tf::Transform currentPoseRelativeToPivot = invertedNewReferenceFrame * endEffectorInPivotFrame;

                geometry_msgs::Pose finalEndEffectorRelativePose;
                tf::poseTFToMsg(currentPoseRelativeToPivot, finalEndEffectorRelativePose);
                relativeInitialPose_ = finalEndEffectorRelativePose;
            }

            // project offset into the xy-plane
            // get the radius
            double radius = sqrt(relativeInitialPose_->position.z * relativeInitialPose_->position.z + relativeInitialPose_->position.y * relativeInitialPose_->position.y);
            double initialAngle = atan2(relativeInitialPose_->position.z, relativeInitialPose_->position.y);

            double totallineardist = radius * deltaRadians_;
            double totalangulardist = deltaRadians_;

            // at least 1 sample per centimeter (average)
            // at least 1 sample per ~1.1 degrees (average)

            const double RADS_PER_SAMPLE = 0.02;
            const double METERS_PER_SAMPLE = 0.01;

            int totalSamplesCount = std::max(totallineardist / METERS_PER_SAMPLE, totalangulardist / RADS_PER_SAMPLE);

            double linearSecondsPerSample;
            double angularSecondsPerSamples;
            double secondsPerSample;

            if (linearSpeed_m_s_)
            {
                linearSecondsPerSample = METERS_PER_SAMPLE / (*linearSpeed_m_s_);
            }
            else
            {
                linearSecondsPerSample = std::numeric_limits<double>::max();
            }

            if (angularSpeed_rad_s_)
            {
                angularSecondsPerSamples = RADS_PER_SAMPLE / (*angularSpeed_rad_s_);
            }
            else
            {
                angularSecondsPerSamples = std::numeric_limits<double>::max();
            }

            if (!linearSpeed_m_s_ && !angularSpeed_rad_s_)
            {
                secondsPerSample = 0.5;
            }
            else
            {
                secondsPerSample = std::min(linearSecondsPerSample, angularSecondsPerSamples);
            }

            double currentAngle = initialAngle;

            double angleStep = deltaRadians_ / (double)totalSamplesCount;

            tf::Transform tfBasePose;
            tf::poseMsgToTF(planePivotPose_.pose, tfBasePose);

            for (int i = 0; i < totalSamplesCount; i++)
            {
                // relativePose i
                currentAngle += angleStep;
                double y = radius * cos(currentAngle);
                double z = radius * sin(currentAngle);

                geometry_msgs::Pose relativeCurrentPose;

                relativeCurrentPose.position.x = relativeInitialPose_->position.x;
                relativeCurrentPose.position.y = y;
                relativeCurrentPose.position.z = z;

                auto localquat = tf::createQuaternionFromRPY(currentAngle, 0, 0);
                //relativeCurrentPose.orientation = relativeInitialPose_.orientation;
                //tf::quaternionTFToMsg(localquat, relativeCurrentPose.orientation);
                relativeCurrentPose.orientation.w = 1;

                tf::Transform tfRelativeCurrentPose;
                tf::poseMsgToTF(relativeCurrentPose, tfRelativeCurrentPose);

                tf::Transform tfGlobalPose = tfRelativeCurrentPose * tfBasePose;

                tfGlobalPose.setRotation(tfGlobalPose.getRotation() * localquat);

                geometry_msgs::PoseStamped globalPose;
                tf::poseTFToMsg(tfGlobalPose, globalPose.pose);
                globalPose.header.frame_id = planePivotPose_.header.frame_id;
                globalPose.header.stamp = planePivotPose_.header.stamp + ros::Duration(i * secondsPerSample);

                this->endEffectorTrajectory_.push_back(globalPose);
            }
        }

    private:
        geometry_msgs::PoseStamped planePivotPose_;
        double deltaRadians_;

        ros::Publisher markersPub_;
    };

} // namespace cl_move_group_interface