/**
 * @ Author: zauberflote1
 * @ Create Time: 2025-01-25 13:14:16
 * @ Modified by: zauberflote1
 * @ Modified time: 2025-02-12 09:11:26
 * @ Description: FOLLOWER CODE FOR CAROLUS
 */

#include <ros/ros.h>
#include <ff_msgs/CommandStamped.h>
#include <ff_msgs/MotionActionFeedback.h>
#include <ff_msgs/MobilityState.h>
#include <ff_msgs/AgentStateStamped.h>

#include <ff_msgs/AckStamped.h>
#include <ff_msgs/AckStatus.h>
#include <ff_msgs/AckCompletedStatus.h>

#include <ff_msgs/CommandArg.h>
#include <ff_msgs/CommandConstants.h>
#include <ff_msgs/VisualLandmarks.h>

#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h> 
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>

#include <Eigen/Geometry>
#include <deque>
#include <algorithm>
#include <vector>
#include <string>
#include <sstream>
#include <iterator>
#include <mutex>
#include <condition_variable>


class CarolusFollower {
public:
    CarolusFollower() : tf_listener_(tf_buffer_), command_in_progress_(false) {
        ros::NodeHandle nh("~");  // Use private namespace for parameters

        // Load desired distances from parameters
        nh.param("desired_distance_x", desired_distance_x_, 0.15);
        nh.param("desired_distance_y", desired_distance_y_, 0.15);
        nh.param("desired_distance_z", desired_distance_z_, 0.15);
        nh.param("duration", duration_, 5.0);

        // Log loaded parameters
        ROS_INFO("Loaded desired distances: x=%.2f, y=%.2f, z=%.2f",
                 desired_distance_x_, desired_distance_y_, desired_distance_z_);

        // Timer for periodic pose processing
        timer_tf_ = nh.createTimer(ros::Duration(duration_), &CarolusFollower::processClosestPose, this);

        // Subscriber for acknowledgments
        sub_ = nh.subscribe("/mgt/ack", 100, &CarolusFollower::AckCallback, this);
        pose_sub_ = nh.subscribe("/loc/ar/features", 100, &CarolusFollower::poseCallback, this);

        // Command publisher
        cmd_pub_ = nh.advertise<ff_msgs::CommandStamped>("/command", 10);
    }

private:
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;
    ros::Timer timer_tf_;
    ros::Subscriber sub_;
    ros::Subscriber pose_sub_;
    ros::Publisher cmd_pub_;
    geometry_msgs::PoseStamped closest_pose_;  // Directly updated in `poseCallback`
    bool command_in_progress_;
    std::string unique_cmd_id;

    // Distance thresholds loaded from ROS parameters
    double desired_distance_x_;
    double desired_distance_y_;
    double desired_distance_z_;
    double duration_;

    void poseCallback(const ff_msgs::VisualLandmarks::ConstPtr& msg) {
        // Update the closest pose (assume the latest message is the closest)
        closest_pose_.header = msg->header;
        closest_pose_.pose = msg->pose;

        ROS_INFO("Pose received from VisualLandmarks callback.");
    }

    void processClosestPose(const ros::TimerEvent&) {
        if (command_in_progress_) {
            ROS_INFO("Command in progress, skipping pose processing.");
            return;
        }

        try {
            // Get transform from world to dock/body
            auto local_transform = tf_buffer_.lookupTransform("world", "dock/body", ros::Time(0));

            // Transform the closest pose into the world frame
            geometry_msgs::PoseStamped transformed_pose;
            tf2::doTransform(closest_pose_, transformed_pose, local_transform);

            // Get the current robot pose
            auto curr_pose = tf_buffer_.lookupTransform("world", "body", ros::Time(0));

            // Build and send the movement command
            buildCommand(transformed_pose, curr_pose);

        } catch (const tf2::TransformException& ex) {
            ROS_WARN_STREAM("Transform lookup failed: " << ex.what());
        }
    }

    void buildCommand(const geometry_msgs::PoseStamped& target_pose, const geometry_msgs::TransformStamped& bot_pose) {
        if (command_in_progress_) {
            return;
        }

        // Create a new command
        ff_msgs::CommandStamped cmd;
        cmd.header.stamp = ros::Time::now();
        cmd.cmd_name = ff_msgs::CommandConstants::CMD_NAME_SIMPLE_MOVE6DOF;

        // Generate unique command ID
        unique_cmd_id = "carolus" + std::to_string(cmd.header.stamp.toSec());
        cmd.cmd_id = unique_cmd_id;
        cmd.subsys_name = "Astrobee";

        // Prepare command arguments
        std::vector<ff_msgs::CommandArg> args(4);
        args[0].data_type = ff_msgs::CommandArg::DATA_TYPE_STRING;
        args[0].s = "world";  // Reference frame

        // Calculate deltas for maintaining desired distance
        args[1].data_type = ff_msgs::CommandArg::DATA_TYPE_VEC3d;
        
        auto posex = adjustDistance(target_pose.pose.position.x, bot_pose.transform.translation.x, desired_distance_x_);
        auto posey = adjustDistance(target_pose.pose.position.y, bot_pose.transform.translation.y, desired_distance_y_);
        auto posez = adjustDistance(target_pose.pose.position.z, bot_pose.transform.translation.z, desired_distance_z_);
        args[1].vec3d[0] = posex;
        args[1].vec3d[1] = posey;
        args[1].vec3d[2] = posez;
            //TOLERANCES NOT USED
            args[2].data_type = ff_msgs::CommandArg::DATA_TYPE_VEC3d;
            args[2].vec3d[0] = 0.0;
            args[2].vec3d[1] = 0.0;
            args[2].vec3d[2] = 0.0;



        // Orientation remains the same as the bot's current orientation
        args[3].data_type = ff_msgs::CommandArg::DATA_TYPE_MAT33f;
        args[3].mat33f[0] = bot_pose.transform.rotation.x;
        args[3].mat33f[1] = bot_pose.transform.rotation.y;
        args[3].mat33f[2] = bot_pose.transform.rotation.z;
        args[3].mat33f[3] = bot_pose.transform.rotation.w;

        cmd.args = args;

        // Publish the command
        cmd_pub_.publish(cmd);
        command_in_progress_ = true;

        ROS_INFO("Published command with ID: %s", cmd.cmd_id.c_str());
        ROS_INFO("Target pose: x=%.2f, y=%.2f, z=%.2f",
                 posex, posey, posez);
    }

    double adjustDistance(double target, double current, double threshold) {
        double delta = target - current;
        if (std::abs(delta) > threshold) {
            return current + (delta > 0 ? threshold : -threshold);
        }
        return current;
    }

    void AckCallback(const ff_msgs::AckStamped::ConstPtr& ack) {
        if (ack->cmd_id == unique_cmd_id) {
            if (ack->completed_status.status == ff_msgs::AckCompletedStatus::OK) {
                ROS_INFO("Command %s completed successfully.", ack->cmd_id.c_str());
                command_in_progress_ = false;
            } else if (ack->completed_status.status == ff_msgs::AckCompletedStatus::NOT) {
                ROS_INFO("Command %s is still in progress.", ack->cmd_id.c_str());
            } else {
                ROS_WARN("Command %s failed: %s", ack->cmd_id.c_str(), ack->message.c_str());
                command_in_progress_ = false;
            }
        }
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "carolus_follower");
    CarolusFollower follower;
    ros::spin();
    return 0;
}
