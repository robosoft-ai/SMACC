#include <angles/angles.h>
#include <pluginlib/class_list_macros.h>
#include <backward_local_planner/backward_local_planner.h>
#include <visualization_msgs/MarkerArray.h>
#include <boost/intrusive_ptr.hpp>

//register this planner as a BaseLocalPlanner plugin
PLUGINLIB_EXPORT_CLASS(cl_move_base_z::backward_local_planner::BackwardLocalPlanner, nav_core::BaseLocalPlanner)

namespace cl_move_base_z
{
    namespace backward_local_planner
    {

        /**
******************************************************************************************************************
* BackwardLocalPlanner()
******************************************************************************************************************
*/
        BackwardLocalPlanner::BackwardLocalPlanner()
            : paramServer_(ros::NodeHandle("~BackwardLocalPlanner"))
        {
        }

        /**
******************************************************************************************************************
* ~BackwardLocalPlanner()
******************************************************************************************************************
*/
        BackwardLocalPlanner::~BackwardLocalPlanner()
        {
        }

        void BackwardLocalPlanner::initialize(std::string name, tf2_ros::Buffer *tf, costmap_2d::Costmap2DROS *costmap_ros)
        {
            this->costmapRos_ = costmap_ros;
            this->initialize();
        }

        void BackwardLocalPlanner::initialize()
        {
            k_rho_ = -1.0;
            k_alpha_ = 0.5;
            k_betta_ = -1.0; // set to zero means that orientation is not important
            carrot_angular_distance_ = 0.4;
            linear_mode_rho_error_threshold_ = 0.02;

            f = boost::bind(&BackwardLocalPlanner::reconfigCB, this, _1, _2);
            paramServer_.setCallback(f);
            this->currentCarrotPoseIndex_ = 0;

            ros::NodeHandle nh("~/BackwardLocalPlanner");
            nh.param("pure_spinning_straight_line_mode", pureSpinningMode_, true);
            nh.param("yaw_goal_tolerance", yaw_goal_tolerance_, 0.05);
            nh.param("xy_goal_tolerance", xy_goal_tolerance_, 0.10);
            nh.param("k_rho", k_rho_, k_rho_);
            nh.param("k_betta", k_betta_, k_betta_);
            nh.param("linear_mode_rho_error_threshold", linear_mode_rho_error_threshold_, linear_mode_rho_error_threshold_);

            nh.param("carrot_distance", carrot_distance_, carrot_distance_);
            nh.param("carrot_angular_distance", carrot_angular_distance_, carrot_angular_distance_);
            nh.param("enable_obstacle_checking", enable_obstacle_checking_, enable_obstacle_checking_);

            nh.param("max_linear_x_speed", max_linear_x_speed_, 1.0);
            nh.param("max_angular_z_speed", max_angular_z_speed_, 2.0);

            // we have to do this, for example for the case we are refining the final orientation.
            // se check at some point if the carrot is reached in "goal linear distance", then we go into
            // some automatic pure-spinning mode where we only update the orientation
            // This means that if we reach the carrot with precission we go into pure spinning mode but we cannot
            // leave that point (maybe this could be improved)
            if (carrot_angular_distance_ < yaw_goal_tolerance_)
            {
                ROS_WARN_STREAM("[BackwardLocalPlanner] carrot_angular_distance (" << carrot_angular_distance_ << ") cannot be lower than yaw_goal_tolerance (" << yaw_goal_tolerance_ << ") setting carrot_angular_distance = " << yaw_goal_tolerance_);
                carrot_angular_distance_ = yaw_goal_tolerance_;
            }

            if (carrot_distance_ < xy_goal_tolerance_)
            {
                ROS_WARN_STREAM("[BackwardLocalPlanner] carrot_linear_distance (" << carrot_distance_ << ") cannot be lower than xy_goal_tolerance_ (" << yaw_goal_tolerance_ << ") setting carrot_angular_distance = " << xy_goal_tolerance_);
                carrot_distance_ = xy_goal_tolerance_;
            }

            goalMarkerPublisher_ = nh.advertise<visualization_msgs::MarkerArray>("goal_marker", 1);
            waitingTimeout_ = ros::Duration(10);
        }

        /**
******************************************************************************************************************
* initialize()
******************************************************************************************************************
*/
        void BackwardLocalPlanner::initialize(std::string name, tf::TransformListener *tf, costmap_2d::Costmap2DROS *costmap_ros)
        {
            this->costmapRos_ = costmap_ros;
            this->initialize();
        }

        void BackwardLocalPlanner::computeCurrentEuclideanAndAngularErrorsToCarrotGoal(const tf::Stamped<tf::Pose> &tfpose, double &dist, double &angular_error)
        {
            double angle = tf::getYaw(tfpose.getRotation());
            auto &carrot_pose = backwardsPlanPath_[currentCarrotPoseIndex_];
            const geometry_msgs::Point &carrot_point = carrot_pose.pose.position;

            tf::Quaternion carrot_orientation;
            tf::quaternionMsgToTF(carrot_pose.pose.orientation, carrot_orientation);

            geometry_msgs::Pose currentPoseDebugMsg;
            tf::poseTFToMsg(tfpose, currentPoseDebugMsg);

            // take error from the current position to the path point
            double dx = carrot_point.x - tfpose.getOrigin().x();
            double dy = carrot_point.y - tfpose.getOrigin().y();
            dist = sqrt(dx * dx + dy * dy);

            double pangle = tf::getYaw(carrot_orientation);
            angular_error = fabs(angles::shortest_angular_distance(pangle, angle));

            ROS_DEBUG_STREAM("[BackwardLocalPlanner] Compute errors. (linear " << dist << ")(angular " << angular_error << ")" << std::endl
                                                                               << "Current carrot pose: " << carrot_pose << std::endl
                                                                               << "Current actual pose:" << currentPoseDebugMsg);
        }

        /**
******************************************************************************************************************
* createCarrotGoal()
******************************************************************************************************************
*/
        bool BackwardLocalPlanner::createCarrotGoal(const tf::Stamped<tf::Pose> &tfpose)
        {
            double disterr = 0, angleerr = 0;
            // iterate the point from the current position and backward until reaching a new goal point in the path
            // this algorithm among other advantages has that skip the looping with an eager global planner
            // that recalls the same plan (the already performed part of the plan in the current pose is skipped)
            while (currentCarrotPoseIndex_ < backwardsPlanPath_.size())
            {
                computeCurrentEuclideanAndAngularErrorsToCarrotGoal(tfpose, disterr, angleerr);

                ROS_DEBUG_STREAM("[BackwardsLocalPlanner] create carrot goal: Current index: " << currentCarrotPoseIndex_ << "/" << backwardsPlanPath_.size());
                ROS_DEBUG("[BackwardsLocalPlanner] create carrot goal: linear error %lf, angular error: %lf", disterr, angleerr);

                // target pose found, goal carrot tries to escape!
                if (disterr < carrot_distance_ && angleerr < carrot_angular_distance_)
                {
                    currentCarrotPoseIndex_++;
                    ROS_DEBUG_STREAM("[BackwardsLocalPlanner] fw " << currentCarrotPoseIndex_ << "/" << backwardsPlanPath_.size());
                }
                else
                {
                    break;
                }
            }
            //ROS_DEBUG("[BackwardsLocalPlanner] computing angular error");
            if (currentCarrotPoseIndex_ >= backwardsPlanPath_.size() && backwardsPlanPath_.size() > 0)
            {
                currentCarrotPoseIndex_ = backwardsPlanPath_.size() - 1;
                computeCurrentEuclideanAndAngularErrorsToCarrotGoal(tfpose, disterr, angleerr);
            }

            ROS_DEBUG("[BackwardsLocalPlanner] Current index carrot goal: %d", currentCarrotPoseIndex_);
            ROS_DEBUG("[BackwardsLocalPlanner] create carrot goal: linear error  %lf, angular error: %lf", disterr, angleerr);

            return disterr < xy_goal_tolerance_;
        }

        bool BackwardLocalPlanner::checkGoalReached(const tf::Stamped<tf::Pose> &tfpose, double vetta, double gamma, double angle_error, geometry_msgs::Twist &cmd_vel)
        {
            auto &finalgoal = backwardsPlanPath_.back();
            double gdx = finalgoal.pose.position.x - tfpose.getOrigin().x();
            double gdy = finalgoal.pose.position.y - tfpose.getOrigin().y();
            double goaldist = sqrt(gdx * gdx + gdy * gdy);

            auto abs_angle_error = fabs(angle_error);
            // final_alpha_error =
            ROS_DEBUG_STREAM("[BackwardLocalPlanner] goal check. linear dist: " << goaldist << "(" << this->xy_goal_tolerance_ << ")"
                                                                                << ", angular dist: " << abs_angle_error << "(" << this->yaw_goal_tolerance_ << ")");

            if (goaldist < this->xy_goal_tolerance_ && abs_angle_error < this->yaw_goal_tolerance_) // 5cm
            {
                ROS_INFO("[BackwardLocalPlanner] goal reached, clearing backwards Plan path");
                goalReached_ = true;
                backwardsPlanPath_.clear();
                return true;
            }

            return false;
        }
        /**
******************************************************************************************************************
* defaultBackwardCmd()
******************************************************************************************************************
*/
        void BackwardLocalPlanner::defaultBackwardCmd(const tf::Stamped<tf::Pose> &tfpose, double vetta, double gamma, double alpha_error, double betta_error, geometry_msgs::Twist &cmd_vel)
        {
            cmd_vel.linear.x = vetta;
            cmd_vel.angular.z = gamma;

            checkGoalReached(tfpose, vetta, gamma, betta_error, cmd_vel);
        }
        /**
******************************************************************************************************************
* pureSpinningCmd()
******************************************************************************************************************
*/
        void BackwardLocalPlanner::pureSpinningCmd(const tf::Stamped<tf::Pose> &tfpose, double vetta, double gamma, double alpha_error, double betta_error, double rho_error, geometry_msgs::Twist &cmd_vel)
        {
            if (rho_error > linear_mode_rho_error_threshold_) // works in straight motion mode
            {
                vetta = k_rho_ * rho_error;
                gamma = k_alpha_ * alpha_error;
            }
            else if (fabs(betta_error) >= this->yaw_goal_tolerance_) // works in pure spinning mode
            {
                vetta = 0;
                gamma = k_betta_ * betta_error;
            }
            /*
            // THIS IS NOT TRUE, IF THE CARROT REACHES THE GOAL DOES NOT MEAN THE ROBOT IS
            // IN THE CORRECT POSITION
            else if (currentCarrotPoseIndex_ >= backwardsPlanPath_.size() - 1)
            {
                vetta = 0;
                gamma = 0;
                goalReached_ = true;
                backwardsPlanPath_.clear();

                ROS_INFO_STREAM("BACKWARD LOCAL PLANNER END: Goal Reached. Stop action [rhoerror: " << rho_error<<"]");
            }*/

            if (this->checkGoalReached(tfpose, vetta, gamma, betta_error, cmd_vel))
            {
                vetta = 0;
                gamma = 0;
            }

            cmd_vel.linear.x = vetta;
            cmd_vel.angular.z = gamma;
        }

// MELODIC
#if ROS_VERSION_MINIMUM(1, 13, 0)
        tf::Stamped<tf::Pose> optionalRobotPose(costmap_2d::Costmap2DROS *costmapRos)
        {
            geometry_msgs::PoseStamped paux;
            costmapRos->getRobotPose(paux);
            tf::Stamped<tf::Pose> tfpose;
            tf::poseStampedMsgToTF(paux, tfpose);
            return tfpose;
        }
#else
        // INDIGO AND PREVIOUS
        tf::Stamped<tf::Pose> optionalRobotPose(costmap_2d::Costmap2DROS *costmapRos)
        {
            tf::Stamped<tf::Pose> tfpose;
            costmapRos->getRobotPose(tfpose);
            return tfpose;
        }

#endif

        /**
******************************************************************************************************************
* computeVelocityCommands()
******************************************************************************************************************
*/
        bool BackwardLocalPlanner::computeVelocityCommands(geometry_msgs::Twist &cmd_vel)
        {
            ROS_DEBUG("[BackwardLocalPlanner] LOCAL PLANNER LOOP");
            geometry_msgs::PoseStamped paux;
            tf::Stamped<tf::Pose> tfpose = optionalRobotPose(costmapRos_);

            bool carrotGoalInRange = createCarrotGoal(tfpose);

            ROS_DEBUG_STREAM("[BackwardLocalPlanner] carrot goal created");

            if (currentCarrotPoseIndex_ < backwardsPlanPath_.size())
            {
                // getting carrot goal information
                tf::Quaternion q = tfpose.getRotation();
                ROS_DEBUG_STREAM("[BackwardLocalPlanner] carrot goal: " << currentCarrotPoseIndex_ << "/" << backwardsPlanPath_.size());
                const geometry_msgs::PoseStamped &carrotgoalpose = backwardsPlanPath_[currentCarrotPoseIndex_];
                ROS_DEBUG_STREAM("[BackwardLocalPlanner] carrot goal pose current index: " << currentCarrotPoseIndex_ << "/" << backwardsPlanPath_.size() << ": " << carrotgoalpose);
                const geometry_msgs::Point &carrotGoalPosition = carrotgoalpose.pose.position;

                tf::Quaternion goalQ;
                tf::quaternionMsgToTF(carrotgoalpose.pose.orientation, goalQ);
                ROS_DEBUG_STREAM("[BackwardLocalPlanner] goal orientation: " << goalQ);

                // ------- COMMON CONTROL COMPUTATION -------------
                //goal orientation (global frame)
                double betta = tf::getYaw(goalQ);
                betta = betta + betta_offset_;

                double dx = carrotGoalPosition.x - tfpose.getOrigin().x();
                double dy = carrotGoalPosition.y - tfpose.getOrigin().y();

                //distance error to the targetpoint
                double rho_error = sqrt(dx * dx + dy * dy);

                //heading to goal angle
                double theta = tf::getYaw(q);
                double alpha = atan2(dy, dx);
                alpha = alpha + alpha_offset_;

                double alpha_error = angles::shortest_angular_distance(alpha, theta);
                double betta_error = angles::shortest_angular_distance(betta, theta);

                double vetta = k_rho_ * rho_error;
                double gamma = k_alpha_ * alpha_error + k_betta_ * betta_error;
                // --------------------

                if (pureSpinningMode_)
                {
                    this->pureSpinningCmd(tfpose, vetta, gamma, alpha_error, betta_error, rho_error, cmd_vel);
                }
                else // default curved backward-free motion mode
                {
                    // case B: goal position reached but orientation not yet reached. deactivate linear motion.
                    if (carrotGoalInRange)
                    {
                        ROS_DEBUG("[BackwardLocalPlanner] pure spinning even in not pure-spining mode, carrotDistanceGoalReached: %d", carrotGoalInRange);
                        gamma = k_betta_ * betta_error;
                        vetta = 0;
                    }

                    //clasical control to reach a goal backwards
                    this->defaultBackwardCmd(tfpose, vetta, gamma, alpha_error, betta_error, cmd_vel);
                }

                if (cmd_vel.linear.x > max_linear_x_speed_)
                {
                    cmd_vel.linear.x = max_linear_x_speed_;
                }
                else if (cmd_vel.linear.x < -max_linear_x_speed_)
                {
                    cmd_vel.linear.x = -max_linear_x_speed_;
                }

                if (cmd_vel.angular.z > max_angular_z_speed_)
                {
                    cmd_vel.angular.z = max_angular_z_speed_;
                }
                else if (cmd_vel.angular.z < -max_angular_z_speed_)
                {
                    cmd_vel.angular.z = -max_angular_z_speed_;
                }

                publishGoalMarker(carrotGoalPosition.x, carrotGoalPosition.y, betta);

                ROS_DEBUG_STREAM("[BackwardLocalPlanner] local planner," << std::endl
                                                                         << " pureSpiningMode: " << pureSpinningMode_ << std::endl
                                                                         << " theta: " << theta << std::endl
                                                                         << " betta: " << theta << std::endl
                                                                         << " err_x: " << dx << std::endl
                                                                         << " err_y:" << dy << std::endl
                                                                         << " rho_error:" << rho_error << std::endl
                                                                         << " alpha_error:" << alpha_error << std::endl
                                                                         << " betta_error:" << betta_error << std::endl
                                                                         << " vetta:" << vetta << std::endl
                                                                         << " gamma:" << gamma);
            }
            else
            {
                ROS_INFO_STREAM(" [BackwardLocalPlanner] all the points in the trajectory are reached, we consider goal reached");
                goalReached_ = true;
            }

            if (goalReached_)
            {
                ROS_INFO_STREAM(" [BackwardLocalPlanner] goal reached, stop command and skipping trajectory collision: " << cmd_vel);
                return true;
            }

            // ---------------------- TRAJECTORY PREDICTION AND COLLISION AVOIDANCE ---------------------
            //cmd_vel.linear.x=0;
            //cmd_vel.angular.z = 0;
            tf::Stamped<tf::Pose> global_pose = optionalRobotPose(costmapRos_);

            auto *costmap2d = costmapRos_->getCostmap();
            auto yaw = tf::getYaw(global_pose.getRotation());

            auto &pos = global_pose.getOrigin();

            Eigen::Vector3f currentpose(pos.x(), pos.y(), yaw);
            Eigen::Vector3f currentvel(cmd_vel.linear.x, cmd_vel.linear.y, cmd_vel.angular.z);
            std::vector<Eigen::Vector3f> trajectory;
            this->generateTrajectory(currentpose, currentvel, 0.8 /*meters*/, M_PI / 8 /*rads*/, 3.0 /*seconds*/, 0.05 /*seconds*/, trajectory);

            // check plan rejection
            bool aceptedplan = true;

            unsigned int mx, my;

            if (this->enable_obstacle_checking_)
            {
                if (backwardsPlanPath_.size() > 0)
                {
                    auto &finalgoalpose = backwardsPlanPath_.back();

                    int i = 0;
                    // ROS_INFO_STREAM("lplanner goal: " << finalgoalpose.pose.position);
                    for (auto &p : trajectory)
                    {
                        float dx = p[0] - finalgoalpose.pose.position.x;
                        float dy = p[1] - finalgoalpose.pose.position.y;

                        float dst = sqrt(dx * dx + dy * dy);
                        if (dst < xy_goal_tolerance_)
                        {
                            ROS_DEBUG("[BackwardLocalPlanner] trajectory simulation for collision checking: goal reached with no collision");
                            break;
                        }

                        costmap2d->worldToMap(p[0], p[1], mx, my);
                        unsigned int cost = costmap2d->getCost(mx, my);

                        // ROS_DEBUG("[BackwardLocalPlanner] checking cost pt %d [%lf, %lf] cell[%d,%d] = %d", i, p[0], p[1], mx, my, cost);
                        // ROS_DEBUG_STREAM("[BackwardLocalPlanner] cost: " << cost);

                        // static const unsigned char NO_INFORMATION = 255;
                        // static const unsigned char LETHAL_OBSTACLE = 254;
                        // static const unsigned char INSCRIBED_INFLATED_OBSTACLE = 253;
                        // static const unsigned char FREE_SPACE = 0;

                        if (costmap2d->getCost(mx, my) >= costmap_2d::INSCRIBED_INFLATED_OBSTACLE)
                        {
                            aceptedplan = false;
                            ROS_WARN_STREAM("[BackwardLocalPlanner] ABORTED LOCAL PLAN BECAUSE OBSTACLE DETEDTED at point " << i << "/" << trajectory.size() << std::endl
                                                                                                                            << p[0] << ", " << p[1]);
                            break;
                        }
                        i++;
                    }
                }
                else
                {
                    ROS_WARN("[BackwardLocalPlanner] Abort local - Backwards global plan size: %ld", backwardsPlanPath_.size());
                    return false;
                }
            }

            if (aceptedplan)
            {
                waiting_ = false;
                return true;
            }
            else // that is not appceted because existence of obstacles
            {
                // emergency stop for collision: waiting a while before sending error
                cmd_vel.linear.x = 0;
                cmd_vel.angular.z = 0;

                if (waiting_ == false)
                {
                    waiting_ = true;
                    waitingStamp_ = ros::Time::now();
                    ROS_WARN("[BackwardLocalPlanner][Not accepted local plan] starting countdown");
                }
                else
                {
                    auto waitingduration = ros::Time::now() - waitingStamp_;

                    if (waitingduration > this->waitingTimeout_)
                    {
                        ROS_WARN("[BackwardLocalPlanner][Abort local] timeout! duration %lf/%f", waitingduration.toSec(), waitingTimeout_.toSec());
                        return false;
                    }
                }

                return true;
            }
        }

        /**
******************************************************************************************************************
* reconfigCB()
******************************************************************************************************************
*/
        void BackwardLocalPlanner::reconfigCB(::backward_local_planner::BackwardLocalPlannerConfig &config, uint32_t level)
        {
            ROS_INFO("[BackwardLocalPlanner] reconfigure Request");
            k_alpha_ = config.k_alpha;
            k_betta_ = config.k_betta;
            k_rho_ = config.k_rho;
            enable_obstacle_checking_ = config.enable_obstacle_checking;

            //alpha_offset_ = config.alpha_offset;
            //betta_offset_ = config.betta_offset;

            carrot_distance_ = config.carrot_distance;
            carrot_angular_distance_ = config.carrot_angular_distance;
            xy_goal_tolerance_ = config.xy_goal_tolerance;
            yaw_goal_tolerance_ = config.yaw_goal_tolerance;
            pureSpinningMode_ = config.pure_spinning_straight_line_mode;

            if (carrot_angular_distance_ < yaw_goal_tolerance_)
            {
                ROS_WARN_STREAM("[BackwardLocalPlanner] carrot_angular_distance (" << carrot_angular_distance_ << ") cannot be lower than yaw_goal_tolerance (" << yaw_goal_tolerance_ << ") setting carrot_angular_distance = " << yaw_goal_tolerance_);
                carrot_angular_distance_ = yaw_goal_tolerance_;
                config.carrot_angular_distance = yaw_goal_tolerance_;
            }

            if (carrot_distance_ < xy_goal_tolerance_)
            {
                ROS_WARN_STREAM("[BackwardLocalPlanner] carrot_linear_distance (" << carrot_distance_ << ") cannot be lower than xy_goal_tolerance_ (" << yaw_goal_tolerance_ << ") setting carrot_angular_distance = " << xy_goal_tolerance_);
                carrot_distance_ = xy_goal_tolerance_;
                config.carrot_distance = xy_goal_tolerance_;
            }
        }

        /**
******************************************************************************************************************
* isGoalReached()
******************************************************************************************************************
*/
        bool BackwardLocalPlanner::isGoalReached()
        {
            return goalReached_;
        }

        /**
******************************************************************************************************************
* setPlan()
******************************************************************************************************************
*/
        bool BackwardLocalPlanner::setPlan(const std::vector<geometry_msgs::PoseStamped> &plan)
        {
            ROS_INFO_STREAM("[BackwardLocalPlanner] setPlan: new global plan received (" << plan.size() << ")");
            initialPureSpinningStage_ = true;
            goalReached_ = false;
            backwardsPlanPath_ = plan;
            currentCarrotPoseIndex_ = 0;

            // find again the new carrot goal, from the destiny direction

            tf::Stamped<tf::Pose> tfpose = optionalRobotPose(costmapRos_);
            double lineardisterr, angleerr;
            bool found = false;

            if (plan.size() == 0)
            {
                return true;
            }

            // initial state check
            computeCurrentEuclideanAndAngularErrorsToCarrotGoal(tfpose, lineardisterr, angleerr);

            // initial path handling for the case the initial point is too much far away
            // lets crop a bit of the initial path, assuming it will be easer to converge to it
            // allowing some carrot distance
            if (lineardisterr > carrot_distance_ || angleerr > carrot_angular_distance_)
            {
                tf::poseStampedMsgToTF(plan[0], tfpose);
            }

            int closestIndex = -1;
            double minpointdist = std::numeric_limits<double>::max();

            // lets set the carrot-goal in the corret place with this loop
            while (currentCarrotPoseIndex_ < backwardsPlanPath_.size())
            {
                computeCurrentEuclideanAndAngularErrorsToCarrotGoal(tfpose, lineardisterr, angleerr);

                //ROS_DEBUG("[BackwardLocalPlanner] Set plan - Current index: %d", currentCarrotPoseIndex_);
                //ROS_DEBUG("[BackwardLocalPlanner] Set plan - linear error to goal %lf, angular error to goal: %lf", disterr, angleerr);

                if (!found) // STAGE 1
                {
                    // current path point is inside the carrot distance range, goal carrot tries to escape!
                    if (lineardisterr < carrot_distance_ && angleerr < carrot_angular_distance_)
                    {
                        found = true;
                        // we are inside the goal range
                    }
                }
                else // STAGE 2
                {
                    // we were inside the carrot range, but we want to continue forward (currentCarrotPoseIndex_++)
                    // unless we go out of the carrot range

                    if (lineardisterr > carrot_distance_ || angleerr > carrot_angular_distance_)
                    {
                        // but we rollback last index increment (to go back inside the carrot goal scope) and start motion with that carrot goal we found
                        currentCarrotPoseIndex_--;
                        break;
                    }
                }

                currentCarrotPoseIndex_++;
                ROS_DEBUG_STREAM("[BackwardLocalPlanner] setPlan: fw" << currentCarrotPoseIndex_);
            }

            ROS_INFO_STREAM("[BackwardLocalPlanner] setPlan: (found first carrot:" << found << ") initial carrot point index: " << currentCarrotPoseIndex_ << "/" << backwardsPlanPath_.size());

            if (!found)
            {
                ROS_WARN("[BackwardLocalPlanner] new plan rejected");
                return false; // in this case, the new plan broke the current execution
            }
            else
            {
                // SANDARD AND PREFERED CASE ON NEW PLAN
                return true;
            }
        }

        void BackwardLocalPlanner::generateTrajectory(const Eigen::Vector3f &pos, const Eigen::Vector3f &vel, float maxdist, float maxanglediff, float maxtime, float dt, std::vector<Eigen::Vector3f> &outtraj)
        {
            //simulate the trajectory and check for collisions, updating costs along the way
            bool end = false;
            float time = 0;
            Eigen::Vector3f currentpos = pos;
            int i = 0;
            while (!end)
            {
                //add the point to the trajectory so we can draw it later if we want
                //traj.addPoint(pos[0], pos[1], pos[2]);

                // if (continued_acceleration_) {
                //   //calculate velocities
                //   loop_vel = computeNewVelocities(sample_target_vel, loop_vel, limits_->getAccLimits(), dt);
                //   //ROS_WARN_NAMED("Generator", "Flag: %d, Loop_Vel %f, %f, %f", continued_acceleration_, loop_vel[0], loop_vel[1], loop_vel[2]);
                // }

                auto loop_vel = vel;
                //update the position of the robot using the velocities passed in
                auto newpos = computeNewPositions(currentpos, loop_vel, dt);

                auto dx = newpos[0] - currentpos[0];
                auto dy = newpos[1] - currentpos[1];
                float dist, angledist;

                //ROS_INFO("traj point %d", i);
                dist = sqrt(dx * dx + dy * dy);
                if (dist > maxdist)
                {
                    end = true;
                    //ROS_INFO("dist break: %f", dist);
                }
                else
                {
                    // ouble from, double to
                    angledist = angles::shortest_angular_distance(currentpos[2], newpos[2]);
                    if (angledist > maxanglediff)
                    {
                        end = true;
                        //ROS_INFO("angle dist break: %f", angledist);
                    }
                    else
                    {
                        outtraj.push_back(newpos);

                        time += dt;
                        if (time > maxtime)
                        {
                            end = true;
                            //ROS_INFO("time break: %f", time);
                        }

                        //ROS_INFO("dist: %f, angledist: %f, time: %f", dist, angledist, time);
                    }
                }

                currentpos = newpos;
                i++;
            } // end for simulation steps
        }

        Eigen::Vector3f BackwardLocalPlanner::computeNewPositions(const Eigen::Vector3f &pos, const Eigen::Vector3f &vel, double dt)
        {
            Eigen::Vector3f new_pos = Eigen::Vector3f::Zero();
            new_pos[0] = pos[0] + (vel[0] * cos(pos[2]) + vel[1] * cos(M_PI_2 + pos[2])) * dt;
            new_pos[1] = pos[1] + (vel[0] * sin(pos[2]) + vel[1] * sin(M_PI_2 + pos[2])) * dt;
            new_pos[2] = pos[2] + vel[2] * dt;
            return new_pos;
        }

        /**
******************************************************************************************************************
* publishGoalMarker()
******************************************************************************************************************
*/
        void BackwardLocalPlanner::publishGoalMarker(double x, double y, double phi)
        {
            visualization_msgs::Marker marker;
            marker.header.frame_id = this->costmapRos_->getGlobalFrameID();
            marker.header.stamp = ros::Time::now();

            marker.ns = "my_namespace2";
            marker.id = 0;
            marker.type = visualization_msgs::Marker::ARROW;
            marker.action = visualization_msgs::Marker::ADD;
            marker.pose.orientation.w = 1;

            marker.scale.x = 0.05;
            marker.scale.y = 0.15;
            marker.scale.z = 0.05;
            marker.color.a = 1.0;

            marker.color.r = 1;
            marker.color.g = 0;
            marker.color.b = 0;

            geometry_msgs::Point start, end;
            start.x = x;
            start.y = y;

            end.x = x + 0.5 * cos(phi);
            end.y = y + 0.5 * sin(phi);

            marker.points.push_back(start);
            marker.points.push_back(end);

            visualization_msgs::MarkerArray ma;
            ma.markers.push_back(marker);

            goalMarkerPublisher_.publish(ma);
        }
    } // namespace backward_local_planner
} // namespace cl_move_base_z