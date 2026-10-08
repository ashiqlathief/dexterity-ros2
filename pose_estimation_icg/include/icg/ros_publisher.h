/*
 * ros_publisher.h
 *
 *  Created on: Dec 26, 2023
 *      Author: oussbzrt
 */

#ifndef POSE_ESTIMATION_ICG_INCLUDE_ICG_ROS_PUBLISHER_H_
#define POSE_ESTIMATION_ICG_INCLUDE_ICG_ROS_PUBLISHER_H_


#include <icg/body.h>
#include <icg/camera.h>
#include <icg/publisher.h>
#include <icg/ros_camera.h>

#include <mutex>

// ROS
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/header.hpp>
#include <dexterity_msgs/msg/estimated_pose.hpp>

namespace icg
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

} // namespace icg



#endif /* POSE_ESTIMATION_ICG_INCLUDE_ICG_ROS_PUBLISHER_H_ */
