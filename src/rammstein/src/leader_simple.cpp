/**
 * @ Author: zauberflote1
 * @ Create Time: 2025-01-25 19:35:51
 * @ Modified by: zauberflote1
 * @ Modified time: 2025-02-12 09:05:59
 * @ Description: Leader code for Carolus
 */

#include <ros/ros.h>
#include <ff_msgs/CommandStamped.h>
#include <ff_msgs/AckStamped.h>
#include <ff_msgs/AckStatus.h>
#include <ff_msgs/AckCompletedStatus.h>
#include <ff_msgs/CommandArg.h>
#include <ff_msgs/CommandConstants.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <Eigen/Geometry>
#include <vector>
#include <string>
#include <sstream>

class CarolusManeuver {
public:
    CarolusManeuver() : tf_buffer_(), tf_listener_(tf_buffer_), command_in_progress_(false), line_executed_(false) {
        ros::NodeHandle nh("~");

        // Load maneuver parameters
        nh.param<std::string>("maneuver_type", maneuver_type_, "square");
        nh.param<std::string>("maneuver_axis", maneuver_axis_, "x");
        nh.param("maneuver_length", maneuver_length_, 0.6);
        nh.param("duration", duration_, 5.0);

        // Log parameters
        ROS_INFO("Loaded maneuver parameters: type=%s, axis=%s, length=%.2f",
                 maneuver_type_.c_str(), maneuver_axis_.c_str(), maneuver_length_);

        // Timer to execute maneuver
        maneuver_timer_ = nh.createTimer(ros::Duration(duration_), &CarolusManeuver::executeManeuver, this);

        // Subscriber for acknowledgments
        ack_sub_ = nh.subscribe("/mgt/ack", 100, &CarolusManeuver::AckCallback, this);

        // Command publisher
        cmd_pub_ = nh.advertise<ff_msgs::CommandStamped>("/command", 10);
    }

private:
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;
    ros::Timer maneuver_timer_;
    ros::Subscriber ack_sub_;
    ros::Publisher cmd_pub_;
    bool command_in_progress_;
    bool line_executed_;  // Flag to check if the line maneuver has been executed
    std::string unique_cmd_id;
    double duration_;

    std::string maneuver_type_;
    std::string maneuver_axis_;
    double maneuver_length_;
    Eigen::Vector3d initial_position_;  
    bool initial_position_set_ = false;

    int current_step_ = 0;  // Current step in the maneuver
    const int steps_in_square_ = 4;  // Number of steps in a square

    void executeManeuver(const ros::TimerEvent&) {
        if (command_in_progress_) {
            ROS_INFO("Command in progress, waiting for completion.");
            return;
        }

        if (maneuver_type_ == "square" && current_step_ > steps_in_square_) {
            ROS_INFO("Square maneuver completed. Shutting down node.");
            ros::shutdown();
            return;
        }

        if (maneuver_type_ == "line" && line_executed_) {
            ROS_INFO("Line maneuver completed. Shutting down node.");
            ros::shutdown();
            return;
        }

        try {
            // Get the robot's current pose
            geometry_msgs::TransformStamped curr_pose = tf_buffer_.lookupTransform("world", "body", ros::Time(0));

            // Calculate the target pose relative to the current pose
            geometry_msgs::PoseStamped target_pose;
            target_pose.header.frame_id = "world";
            target_pose.header.stamp = ros::Time::now();
            target_pose.pose = calculateTargetPose(curr_pose);

            // Build and send the movement command
            buildCommand(target_pose, curr_pose);

        } catch (const tf2::TransformException& ex) {
            ROS_WARN_STREAM("Could not get robot pose: " << ex.what());
        }
    }

    geometry_msgs::Pose calculateTargetPose(const geometry_msgs::TransformStamped& curr_pose) {
        geometry_msgs::Pose pose;

        // Convert orientation
        tf2::Quaternion tf_quat(
            curr_pose.transform.rotation.x,
            curr_pose.transform.rotation.y,
            curr_pose.transform.rotation.z,
            curr_pose.transform.rotation.w);

        pose.orientation.x = tf_quat.x();
        pose.orientation.y = tf_quat.y();
        pose.orientation.z = tf_quat.z();
        pose.orientation.w = tf_quat.w();

        // Determine target position based on maneuver type
        if (maneuver_type_ == "line") {
            pose = calculateLinearTarget(curr_pose);
        } else if (maneuver_type_ == "square") {
            pose = calculateSquareTarget(curr_pose);
        } else {
            ROS_WARN("Invalid maneuver type: %s. Skipping maneuver.", maneuver_type_.c_str());
        }

        return pose;
    }

    geometry_msgs::Pose calculateLinearTarget(const geometry_msgs::TransformStamped& curr_pose) {
        geometry_msgs::Pose pose;

        // Retain current orientation
        tf2::Quaternion tf_quat(
            curr_pose.transform.rotation.x,
            curr_pose.transform.rotation.y,
            curr_pose.transform.rotation.z,
            curr_pose.transform.rotation.w);

        pose.orientation.x = tf_quat.x();
        pose.orientation.y = tf_quat.y();
        pose.orientation.z = tf_quat.z();
        pose.orientation.w = tf_quat.w();

        // Adjust position along the specified axis
        pose.position.x = curr_pose.transform.translation.x + (maneuver_axis_ == "x" ? maneuver_length_ : 0);
        pose.position.y = curr_pose.transform.translation.y + (maneuver_axis_ == "y" ? maneuver_length_ : 0);
        pose.position.z = curr_pose.transform.translation.z + (maneuver_axis_ == "z" ? maneuver_length_ : 0);

        // Mark the line as executed (only one step for the line maneuver)
        line_executed_ = true;

        return pose;
    }

geometry_msgs::Pose calculateSquareTarget(const geometry_msgs::TransformStamped& curr_pose) {
    geometry_msgs::Pose pose;

    // Retain initial orientation
    tf2::Quaternion tf_quat(
        curr_pose.transform.rotation.x,
        curr_pose.transform.rotation.y,
        curr_pose.transform.rotation.z,
        curr_pose.transform.rotation.w);

    pose.orientation.x = tf_quat.x();
    pose.orientation.y = tf_quat.y();
    pose.orientation.z = tf_quat.z();
    pose.orientation.w = tf_quat.w();

    // Store initial position only once and always use it for calculations
    if (!initial_position_set_) {
        initial_position_ = Eigen::Vector3d(curr_pose.transform.translation.x, 
                                            curr_pose.transform.translation.y, 
                                            curr_pose.transform.translation.z);
        initial_position_set_ = true;
    }

    Eigen::Vector3d new_position = initial_position_;  // Always reference the initial position

    // Execute exactly four moves in a square, relative to initial_position_
    switch (current_step_) {
        case 0:
            new_position.x() += maneuver_length_;  // Move forward
            break;
        case 1:
            new_position.x() += maneuver_length_;  // Move right
            new_position.y() -= maneuver_length_;  // Move right
            break;
        case 2:
            new_position.y() -= maneuver_length_; // Move backward
            break;
        case 3:
            ROS_INFO("Returning to initial position.");
            break;
    }

    pose.position.x = new_position.x();
    pose.position.y = new_position.y();
    pose.position.z = initial_position_.z();  // Keep z constant (do not modify)

    current_step_++;

    // if (current_step_ >= steps_in_square_) {
    //     ROS_INFO("Square maneuver completed. Shutting down node.");
    //     ros::shutdown();
    // }

    return pose;
}



    void buildCommand(const geometry_msgs::PoseStamped& target_pose, const geometry_msgs::TransformStamped& bot_pose) {
        if (command_in_progress_) {
            return;
        }

        ff_msgs::CommandStamped cmd;
        cmd.header.stamp = ros::Time::now();
        cmd.cmd_name = ff_msgs::CommandConstants::CMD_NAME_SIMPLE_MOVE6DOF;

        unique_cmd_id = "carolus" + std::to_string(cmd.header.stamp.toSec());
        cmd.cmd_id = unique_cmd_id;
        cmd.subsys_name = "Astrobee";

        std::vector<ff_msgs::CommandArg> args(4);
        args[0].data_type = ff_msgs::CommandArg::DATA_TYPE_STRING;
        args[0].s = "world";  // Reference frame

        args[1].data_type = ff_msgs::CommandArg::DATA_TYPE_VEC3d;
        args[1].vec3d[0] = target_pose.pose.position.x;
        args[1].vec3d[1] = target_pose.pose.position.y;
        args[1].vec3d[2] = target_pose.pose.position.z;
            //TOLERANCES NOT USED
            args[2].data_type = ff_msgs::CommandArg::DATA_TYPE_VEC3d;
            args[2].vec3d[0] = 0.0;
            args[2].vec3d[1] = 0.0;
            args[2].vec3d[2] = 0.0;


        args[3].data_type = ff_msgs::CommandArg::DATA_TYPE_MAT33f;
        args[3].mat33f[0] = bot_pose.transform.rotation.x;
        args[3].mat33f[1] = bot_pose.transform.rotation.y;
        args[3].mat33f[2] = bot_pose.transform.rotation.z;
        args[3].mat33f[3] = bot_pose.transform.rotation.w;

        cmd.args = args;

        cmd_pub_.publish(cmd);
        command_in_progress_ = true;

        ROS_INFO("Published maneuver command with ID: %s", cmd.cmd_id.c_str());
        ROS_INFO("Target position: %.2f, %.2f, %.2f", target_pose.pose.position.x, target_pose.pose.position.y, target_pose.pose.position.z);
    }

    void AckCallback(const ff_msgs::AckStamped::ConstPtr& ack) {
        if (ack->cmd_id == unique_cmd_id) {
            if (ack->completed_status.status == ff_msgs::AckCompletedStatus::OK) {
                ROS_INFO("Command %s completed successfully.", ack->cmd_id.c_str());
                command_in_progress_ = false;

                // Shutdown if maneuver is complete
                if (maneuver_type_ == "square" && current_step_ >= steps_in_square_) {
                    ROS_INFO("Square maneuver completed. Shutting down node.");
                    ros::shutdown();
                } else if (maneuver_type_ == "line" && line_executed_) {
                    ROS_INFO("Line maneuver completed. Shutting down node.");
                    ros::shutdown();
                }
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
    ros::init(argc, argv, "carolus_maneuver");
    CarolusManeuver maneuver;
    ros::spin();
    return 0;
}
