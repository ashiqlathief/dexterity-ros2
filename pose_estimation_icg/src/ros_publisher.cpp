
/*
 * ros_publisher.cpp
 *
 *  Created on: Dec 26, 2023
 *      Author: oussbzrt
 */



#include <icg/ros_publisher.h>

namespace icg
{

    void RosPublisher::set_name (const std::string &name) { name_ = name; }

    void RosPublisher::set_metafile_path (const std::filesystem::path &metafile_path)
    {
        metafile_path_ = metafile_path;
        set_up_ = false;
    }

    const std::string &RosPublisher::name() const { return name_; }

    const std::filesystem::path &RosPublisher::metafile_path() const
    {
        return metafile_path_;
    }

    bool RosPublisher::set_up() const { return set_up_; }

    bool RosPublisher::SetUp()
    {
        if (!camera_ptr_)
        {
            std::cout << "Publisher class requires camera pointer!" << std::endl;
            return false;
        }
        if (!posePub)
        {
            std::cout << "Publisher class requires ros publisher pointer!" << std::endl;
            return false;
        }
        set_up_ = true;
        return true;
    }

    bool RosPublisher::AddReferencedBody (
        const std::shared_ptr<Body> &referenced_body_ptr)
    {
        set_up_ = false;
        if (!AddPtrIfNameNotExists (referenced_body_ptr, &referenced_body_ptrs_))
        {
            std::cerr << "Referenced body " << referenced_body_ptr->name()
                      << " already exists" << std::endl;
            return false;
        }
        return true;
    }

    bool RosPublisher::DeleteReferencedBody (const std::string &name)
    {
        set_up_ = false;
        if (!DeletePtrIfNameExists (name, &referenced_body_ptrs_))
        {
            std::cerr << "Referenced body " << name << " not found" << std::endl;
            return false;
        }
        return true;
    }

    void RosPublisher::ClearReferencedBodies()
    {
        set_up_ = false;
        referenced_body_ptrs_.clear();
    }

    void RosPublisher::AddRosPublisher (rclcpp::Publisher<dexterity_msgs::msg::EstimatedPose>::SharedPtr pose_pub)
    {
        posePub = pose_pub;
    }

    void RosPublisher::set_msg_header (const std_msgs::msg::Header &header)
    {
        const std::lock_guard<std::mutex> lock{header_mutex_};
        msg_header_ = header;
    }

    void RosPublisher::AddCamera (const std::shared_ptr<Camera> &camera_ptr)
    {
        camera_ptr_ = camera_ptr;
    }

    bool RosPublisher::UpdatePublisher (int iteration)
    {
        // For each tracked body, get its body2world pose and the cameras
        // world2camera pose to publish a body2camera pose for each object in
        // one message.
        std_msgs::msg::Header header;
        {
            const std::lock_guard<std::mutex> lock{header_mutex_};
            header = msg_header_;
        }
        for (auto &referenced_body_ptr : referenced_body_ptrs_)
        {
            auto body2camera_pose = camera_ptr_->world2camera_pose() *
                                    referenced_body_ptr->body2world_pose();

            auto translation = body2camera_pose.translation();
            Eigen::Quaternionf quaternionEigen (body2camera_pose.rotation());

            // Pose of the body in the optical frame of the camera image it was estimated from
            dexterity_msgs::msg::EstimatedPose estimatedPose;
            estimatedPose.pose_stamped.header = header; // when and where the image was taken
            estimatedPose.pose_stamped.pose.position.x = translation.x();
            estimatedPose.pose_stamped.pose.position.y = translation.y();
            estimatedPose.pose_stamped.pose.position.z = translation.z();
            estimatedPose.pose_stamped.pose.orientation.x = quaternionEigen.x();
            estimatedPose.pose_stamped.pose.orientation.y = quaternionEigen.y();
            estimatedPose.pose_stamped.pose.orientation.z = quaternionEigen.z();
            estimatedPose.pose_stamped.pose.orientation.w = quaternionEigen.w();
            estimatedPose.success = true;
            estimatedPose.tracked_object_name = referenced_body_ptr->name();
            posePub->publish (estimatedPose);
        }
        return true;
    }

    RosPublisher::RosPublisher (const std::string &name) : Publisher{name} {}

    RosPublisher::RosPublisher (const std::string &name,
                                const std::filesystem::path &metafile_path)
        : Publisher{name, metafile_path} {}

} // namespace icg

