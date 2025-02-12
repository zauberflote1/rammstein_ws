#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <geometry_msgs/PoseStamped.h>
#include <ff_msgs/EkfState.h>
#include <ff_msgs/VisualLandmarks.h>
#include <ff_msgs/VisualLandmark.h>
#include <boost/filesystem.hpp>
#include <fstream>

void writeEkfStateToCSV(const std::string &filename, const std::vector<ff_msgs::EkfState> &ekf_data) {
    std::ofstream file(filename);
    file << "timestamp,child_frame_id,x,y,z,vx,vy,vz,wx,wy,wz,ax,ay,az,qx,qy,qz,qw,cov_diag,confidence,aug_state_enum,status,of_count,ml_count,hr_x,hr_y,hr_z,hr_qx,hr_qy,hr_qz,hr_qw,estimating_bias" << std::endl;
    for (const auto &msg : ekf_data) {
        file << msg.header.stamp.toSec() << ","
             << msg.child_frame_id << ","
             << msg.pose.position.x << "," << msg.pose.position.y << "," << msg.pose.position.z << ","
             << msg.velocity.x << "," << msg.velocity.y << "," << msg.velocity.z << ","
             << msg.omega.x << "," << msg.omega.y << "," << msg.omega.z << ","
             << msg.accel.x << "," << msg.accel.y << "," << msg.accel.z << ","
             << msg.pose.orientation.x << "," << msg.pose.orientation.y << "," << msg.pose.orientation.z << "," << msg.pose.orientation.w << ",";
        
        for (size_t i = 0; i < 15; ++i) {
            file << msg.cov_diag[i] << (i < 14 ? "," : "");
        }
        
        file << "," << static_cast<int>(msg.confidence) << ","
             << static_cast<int>(msg.aug_state_enum) << ","
             << static_cast<int>(msg.status) << ","
             << static_cast<int>(msg.of_count) << ","
             << static_cast<int>(msg.ml_count) << ","
             << msg.hr_global_pose.position.x << "," << msg.hr_global_pose.position.y << "," << msg.hr_global_pose.position.z << ","
             << msg.hr_global_pose.orientation.x << "," << msg.hr_global_pose.orientation.y << "," << msg.hr_global_pose.orientation.z << "," << msg.hr_global_pose.orientation.w << ","
             << msg.estimating_bias << std::endl;
    }
    file.close();
}

void writePoseStampedToCSV(const std::string &filename, const std::vector<geometry_msgs::PoseStamped> &pose_data) {
    std::ofstream file(filename);
    file << "timestamp,camera_id,x,y,z,qx,qy,qz,qw" << std::endl;
    for (const auto &msg : pose_data) {
        file << msg.header.stamp.toSec() << ","
             << msg.pose.position.x << "," << msg.pose.position.y << "," << msg.pose.position.z << ","
             << msg.pose.orientation.x << "," << msg.pose.orientation.y << "," << msg.pose.orientation.z << "," << msg.pose.orientation.w << std::endl;
    }
    file.close();
}

void writeVisualLandmarksToCSV(const std::string &filename, const std::vector<ff_msgs::VisualLandmarks> &landmark_data) {
    std::ofstream file(filename);
    file << "timestamp,camera_id,pose_x,pose_y,pose_z,pose_qx,pose_qy,pose_qz,pose_qw,runtime,landmark_x,landmark_y,landmark_z,landmark_u,landmark_v" << std::endl;
    
    for (const auto &msg : landmark_data)  {
        for (const auto &landmark : msg.landmarks) {
            file << msg.header.stamp.toSec() << ","
                 << msg.camera_id << ","
                 << msg.pose.position.x << "," << msg.pose.position.y << "," << msg.pose.position.z << ","
                 << msg.pose.orientation.x << "," << msg.pose.orientation.y << "," << msg.pose.orientation.z << "," << msg.pose.orientation.w << ","
                 << msg.runtime << ","
                 << landmark.x << "," << landmark.y << "," << landmark.z << ","
                 << landmark.u << "," << landmark.v << std::endl;
        }
    }
    file.close();
}


int main(int argc, char **argv) {
    if (argc < 2) {
        std::cerr << "Usage: rosrun package_name rosbag_to_csv <bagfile>" << std::endl;
        return 1;
    }
    
    std::string bag_file = argv[1];
    std::string bag_name = bag_file.substr(0, bag_file.find_last_of("."));
    
    rosbag::Bag bag;
    try {
        bag.open(bag_file, rosbag::bagmode::Read);
    } catch (const rosbag::BagException &e) {
        std::cerr << "Error opening bag file: " << e.what() << std::endl;
        return 1;
    }
    
    std::vector<ff_msgs::EkfState> ekf_data;
    std::vector<geometry_msgs::PoseStamped> pose_data;
    std::vector<ff_msgs::VisualLandmarks> landmark_data;
    
    rosbag::View view(bag);
    for (const rosbag::MessageInstance &msg : view) {
        if (msg.getTopic() == "/gnc/ekf") {
            ff_msgs::EkfState::ConstPtr ekf_msg = msg.instantiate<ff_msgs::EkfState>();
            if (ekf_msg) ekf_data.push_back(*ekf_msg);
        } else if (msg.getTopic() == "/loc/pose") {
            geometry_msgs::PoseStamped::ConstPtr pose_msg = msg.instantiate<geometry_msgs::PoseStamped>();
            if (pose_msg) pose_data.push_back(*pose_msg);
        } else if (msg.getTopic() == "/loc/ar/features") {
            ff_msgs::VisualLandmarks::ConstPtr landmark_msg = msg.instantiate<ff_msgs::VisualLandmarks>();
            if (landmark_msg) landmark_data.push_back(*landmark_msg);
        }
    }
    
    bag.close();
    
    writeEkfStateToCSV("gnc_ekf_" + bag_name + ".csv", ekf_data);
    writePoseStampedToCSV("loc_pose_" + bag_name + ".csv", pose_data);
    writeVisualLandmarksToCSV("loc_landmarks_" + bag_name + ".csv", landmark_data);
    std::cout << "CSV files generated: gnc_ekf_" + bag_name + ".csv, loc_pose_" + bag_name + ".csv" << std::endl;
    return 0;
}