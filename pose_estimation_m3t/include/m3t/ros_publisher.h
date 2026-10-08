/*
 * ros_publisher.h
 *
 *  Created on: Dec 26, 2023
 *      Author: oussbzrt
 */

#ifndef POSE_ESTIMATION_M3T_INCLUDE_M3T_ROS_PUBLISHER_H_
#define POSE_ESTIMATION_M3T_INCLUDE_M3T_ROS_PUBLISHER_H_


#include <m3t/body.h>
#include <m3t/camera.h>
#include <m3t/publisher.h>
#include <m3t/ros_camera.h>

#include <mutex>

// ROS
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/header.hpp>
#include <dexterity_msgs/msg/estimated_pose.hpp>

namespace m3t
{

    class RosPublisher : public Publisher
    {
    public:
        // Constructor
        RosPublisher (const std::string &name);
        RosPublisher (const std::string &name,
                      const std::filesystem::path &metafile_path);

        // Setters
        void set_name (const std::string &name);
        void set_metafile_path (const std::filesystem::path &metafile_path);
        // Header of the image the next poses are calculated from (written by the image callback)
        void set_msg_header (const std_msgs::msg::Header &header);

        // Class pointers to get to all necessary data
        bool AddReferencedBody (const std::shared_ptr<Body> &referenced_body_ptr);
        bool DeleteReferencedBody (const std::string &name);
        void ClearReferencedBodies();
        void AddRosPublisher (rclcpp::Publisher<dexterity_msgs::msg::EstimatedPose>::SharedPtr pose_pub);
        void AddCamera (const std::shared_ptr<Camera> &camera_ptr);

        // Main methods
        bool SetUp() override;
        bool UpdatePublisher (int iteration) override;

        // Getters
        const std::string &name() const;
        const std::filesystem::path &metafile_path() const;
        bool set_up() const;

    protected:
        // Variables
        std::string name_{};
        std::filesystem::path metafile_path_{};
        bool set_up_ = false;

    private:
        std::vector<std::shared_ptr<Body>> referenced_body_ptrs_{};
        std::shared_ptr<Camera> camera_ptr_ = nullptr;
        rclcpp::Publisher<dexterity_msgs::msg::EstimatedPose>::SharedPtr posePub;
        std_msgs::msg::Header msg_header_;
        std::mutex header_mutex_;
    };

} // namespace m3t



#endif /* POSE_ESTIMATION_M3T_INCLUDE_M3T_ROS_PUBLISHER_H_ */
