/*
 * icg_tracker_node.cpp
 *
 * ROS 2 node around the ICG tracker (ROS 2 port of MainRos.cpp / main_ros.cpp, Object Handling group).
 *
 * The tracker is (re)initialized every time the GUI publishes the 6 manual detection points.
 * It then tracks the selected body in the color image and publishes its pose relative to the
 * camera as dexterity_msgs/EstimatedPose, which dexterity consumes with tracker "icg".
 */

#include <filesystem>
#include <memory>
#include <set>
#include <string>
#include <thread>
#include <vector>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <dexterity_msgs/msg/estimated_pose.hpp>
#include <dexterity_msgs/msg/pixel_position.hpp>

#include <icg/basic_depth_renderer.h>
#include <icg/body.h>
#include <icg/depth_modality.h>
#include <icg/depth_model.h>
#include <icg/manual_detector.h>
#include <icg/normal_viewer.h>
#include <icg/region_modality.h>
#include <icg/region_model.h>
#include <icg/renderer_geometry.h>
#include <icg/ros_camera.h>
#include <icg/ros_publisher.h>
#include <icg/tracker.h>

namespace
{
	constexpr size_t BOOTSTRAP_POINTS = 6; // Number of reference points in <body>_manual_detector.yaml
}

class ICGTrackerNode : public rclcpp::Node
{
	public:
		ICGTrackerNode();
		~ICGTrackerNode() override;

	private:
		bool initICG(const std::string& bodyName, const std::vector<cv::Point2f>& detectionPoints);
		void runICG();
		void stopICG();
		std::string resolveBodyName(const std::string& requested) const;

		// Callbacks
		void rgbCameraInfoCallback(const sensor_msgs::msg::CameraInfo::ConstSharedPtr msg);
		void depthCameraInfoCallback(const sensor_msgs::msg::CameraInfo::ConstSharedPtr msg);
		void rgbImageCallback(const sensor_msgs::msg::Image::ConstSharedPtr msg);
		void depthImageCallback(const sensor_msgs::msg::Image::ConstSharedPtr msg);
		void manualDetectionCallback(const dexterity_msgs::msg::PixelPosition::ConstSharedPtr msg);

		// Parameters
		std::string rgbImageTopic;
		std::string rgbCameraInfoTopic;
		std::string depthImageTopic;
		std::string depthCameraInfoTopic;
		std::filesystem::path modelDirectory;
		std::string defaultBody;
		bool useDepthCamera = false;
		bool displayImages = false;

		// Subscribers
		rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr rgbCameraInfoSub;
		rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr depthCameraInfoSub;
		rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr rgbImageSub;
		rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depthImageSub;
		rclcpp::Subscription<dexterity_msgs::msg::PixelPosition>::SharedPtr manualDetectionSub;

		// Publishers
		rclcpp::Publisher<dexterity_msgs::msg::EstimatedPose>::SharedPtr posePub;
		rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr mixedImagePub;

		// ICG objects
		std::shared_ptr<icg::RosCameraColorCamera> colorCamera;
		std::shared_ptr<icg::RosCameraDepthCamera> depthCamera;
		std::shared_ptr<icg::Tracker> tracker;
		std::shared_ptr<icg::RosPublisher> publisher;
		std::shared_ptr<icg::NormalColorViewer> colorViewer;
		std::thread icgThread;

		// Last images, used to wake up the tracker thread when it has to stop
		cv::Mat lastRgbFrame;
		cv::Mat lastDepthFrame;

		// Internal state
		bool initDone = false;
		bool runICGDone = false;
		bool firstImageRgb = false;
		bool firstImageDepth = false;
};

ICGTrackerNode::ICGTrackerNode() : Node("pose_estimation_icg")
{
	rgbImageTopic = declare_parameter<std::string>("rgb_image_topic", "/wrist_mounted_camera/image");
	rgbCameraInfoTopic = declare_parameter<std::string>("rgb_camera_info_topic", "/wrist_mounted_camera/camera_info");
	useDepthCamera = declare_parameter<bool>("use_depth", false);
	depthImageTopic = declare_parameter<std::string>("depth_image_topic", "/wrist_mounted_camera/depth_image");
	depthCameraInfoTopic = declare_parameter<std::string>("depth_camera_info_topic", "/wrist_mounted_camera/camera_info");
	displayImages = declare_parameter<bool>("display_images", false);
	defaultBody = declare_parameter<std::string>("default_body", "asymmetric_pipestar_without_cap");
	std::string modelDir = declare_parameter<std::string>("model_directory", "");
	modelDirectory = modelDir.empty()
		? std::filesystem::path(ament_index_cpp::get_package_share_directory("pose_estimation_icg")) / "models"
		: std::filesystem::path(modelDir);
	const std::string poseTopic = declare_parameter<std::string>("pose_topic", "/tracked_object_poses");
	const std::string manualDetectionTopic = declare_parameter<std::string>("manual_detection_topic", "GUI_MANUAL_DETECTION_POINTS");
	const std::string debugImageTopic = declare_parameter<std::string>("debug_image_topic", "ICG_Tracker_Streaming/image_raw");

	// Initialization is done once the camera parameters and the manual detection points from the GUI arrived
	rgbCameraInfoSub = create_subscription<sensor_msgs::msg::CameraInfo>(rgbCameraInfoTopic, 10,
		std::bind(&ICGTrackerNode::rgbCameraInfoCallback, this, std::placeholders::_1));
	RCLCPP_INFO(get_logger(), "Waiting for RGB camera info on %s", rgbCameraInfoTopic.c_str());
	if (useDepthCamera)
	{
		depthCameraInfoSub = create_subscription<sensor_msgs::msg::CameraInfo>(depthCameraInfoTopic, 10,
			std::bind(&ICGTrackerNode::depthCameraInfoCallback, this, std::placeholders::_1));
		RCLCPP_INFO(get_logger(), "Waiting for depth camera info on %s", depthCameraInfoTopic.c_str());
	}
	manualDetectionSub = create_subscription<dexterity_msgs::msg::PixelPosition>(manualDetectionTopic, 1,
		std::bind(&ICGTrackerNode::manualDetectionCallback, this, std::placeholders::_1));

	posePub = create_publisher<dexterity_msgs::msg::EstimatedPose>(poseTopic, 100);
	// Camera image with the rendered body on top, shown by the GUI
	mixedImagePub = create_publisher<sensor_msgs::msg::Image>(debugImageTopic, 1);

	RCLCPP_INFO(get_logger(), "Models are loaded from %s", modelDirectory.c_str());
}

ICGTrackerNode::~ICGTrackerNode()
{
	stopICG();
}

std::string ICGTrackerNode::resolveBodyName(const std::string& requested) const
{
	// The GUI may send a file name (e.g. "pipestar.cao" from the ViSP models), ICG uses the folder name
	std::string name = std::filesystem::path(requested).stem().string();
	if (!name.empty() && std::filesystem::exists(modelDirectory / name / (name + ".yaml")))
	{
		return name;
	}
	if (!name.empty())
	{
		RCLCPP_WARN(get_logger(), "No ICG model '%s' in %s, using default body '%s'",
			name.c_str(), modelDirectory.c_str(), defaultBody.c_str());
	}
	return defaultBody;
}

bool ICGTrackerNode::initICG(const std::string& bodyName, const std::vector<cv::Point2f>& detectionPoints)
{
	const std::filesystem::path directory = modelDirectory / bodyName;
	if (!std::filesystem::exists(directory / (bodyName + ".yaml")))
	{
		RCLCPP_ERROR(get_logger(), "Model %s not found", (directory / (bodyName + ".yaml")).c_str());
		return false;
	}
	RCLCPP_INFO(get_logger(), "Initializing ICG for body '%s'", bodyName.c_str());

	constexpr bool kMeasureOcclusions = true;
	constexpr bool kModelOcclusions = false;

	// Set up tracker and renderer geometry
	tracker = std::make_shared<icg::Tracker>("tracker");
	auto rendererGeometry = std::make_shared<icg::RendererGeometry>("renderer geometry");

	// Set up publisher; pose data gets calculated relative to the publisher camera
	publisher = std::make_shared<icg::RosPublisher>("pose");
	publisher->AddRosPublisher(posePub);
	publisher->AddCamera(colorCamera);
	tracker->AddPublisher(publisher);

	// Set up viewers (the color viewer also produces the image published on the debug topic)
	colorViewer = std::make_shared<icg::NormalColorViewer>("color_viewer_ICG", colorCamera, rendererGeometry);
	colorViewer->set_display_images(displayImages);
	tracker->AddViewer(colorViewer);
	if (useDepthCamera)
	{
		auto depthViewer = std::make_shared<icg::NormalDepthViewer>("depth_viewer", depthCamera, rendererGeometry, 0.3f, 1.0f);
		depthViewer->set_display_images(displayImages);
		tracker->AddViewer(depthViewer);
	}

	// Set up renderers
	auto colorDepthRenderer = std::make_shared<icg::FocusedBasicDepthRenderer>("color_depth_renderer", rendererGeometry, colorCamera);
	std::shared_ptr<icg::FocusedBasicDepthRenderer> depthDepthRenderer;
	if (useDepthCamera)
	{
		depthDepthRenderer = std::make_shared<icg::FocusedBasicDepthRenderer>("depth_depth_renderer", rendererGeometry, depthCamera);
	}

	// Set up body
	auto body = std::make_shared<icg::Body>(bodyName, directory / (bodyName + ".yaml"));
	rendererGeometry->AddBody(body);
	colorDepthRenderer->AddReferencedBody(body);
	if (useDepthCamera)
	{
		depthDepthRenderer->AddReferencedBody(body);
	}
	publisher->AddReferencedBody(body);

	// Set up models; they are generated and saved next to the body on first use, which takes a while
	auto regionModel = std::make_shared<icg::RegionModel>(bodyName + "_region_model", body, directory / (bodyName + "_region_model.bin"));
	std::shared_ptr<icg::DepthModel> depthModel;
	if (useDepthCamera)
	{
		depthModel = std::make_shared<icg::DepthModel>(bodyName + "_depth_model", body, directory / (bodyName + "_depth_model.bin"));
	}

	// Set up modalities
	auto regionModality = std::make_shared<icg::RegionModality>(bodyName + "_region_modality", body, colorCamera, regionModel);
	std::shared_ptr<icg::DepthModality> depthModality;
	if (useDepthCamera)
	{
		depthModality = std::make_shared<icg::DepthModality>(bodyName + "_depth_modality", body, depthCamera, depthModel);
		if (kMeasureOcclusions)
		{
			regionModality->MeasureOcclusions(depthCamera);
			depthModality->MeasureOcclusions();
		}
		if (kModelOcclusions)
		{
			regionModality->ModelOcclusions(colorDepthRenderer);
			depthModality->ModelOcclusions(depthDepthRenderer);
		}
	}

	// Set up optimizer
	auto optimizer = std::make_shared<icg::Optimizer>(bodyName + "_optimizer");
	optimizer->AddModality(regionModality);
	if (useDepthCamera)
	{
		optimizer->AddModality(depthModality);
	}
	tracker->AddOptimizer(optimizer);

	// Set up detector with the pixel positions clicked in the GUI
	auto detector = std::make_shared<icg::ManualDetector>(bodyName + "_detector",
		directory / (bodyName + "_manual_detector.yaml"), body, colorCamera, detectionPoints);
	tracker->AddDetector(detector);

	// Subscribe to the camera images; the tracker starts with the first image
	rgbImageSub = create_subscription<sensor_msgs::msg::Image>(rgbImageTopic, rclcpp::SensorDataQoS().keep_last(1),
		std::bind(&ICGTrackerNode::rgbImageCallback, this, std::placeholders::_1));
	if (useDepthCamera)
	{
		depthImageSub = create_subscription<sensor_msgs::msg::Image>(depthImageTopic, rclcpp::SensorDataQoS().keep_last(1),
			std::bind(&ICGTrackerNode::depthImageCallback, this, std::placeholders::_1));
	}

	initDone = true;
	runICGDone = false;
	RCLCPP_INFO(get_logger(), "Initialization complete, waiting for images on %s", rgbImageTopic.c_str());
	return true;
}

void ICGTrackerNode::runICG()
{
	runICGDone = true;
	if (!tracker->SetUp())
	{
		RCLCPP_ERROR(get_logger(), "Setting up the ICG tracker failed");
		return;
	}
	// Detection with the GUI points and the start of tracking are triggered by the tracker itself (see Tracker::UpdateViewers)
	icgThread = std::thread([tracker = tracker]()
	{
		tracker->RunTrackerProcess(false, false);
	});
	RCLCPP_INFO(get_logger(), "Tracking started");
}

void ICGTrackerNode::stopICG()
{
	if (icgThread.joinable())
	{
		tracker->QuitTrackerProcess();
		// The tracker thread may wait for a new image, which will not come while this callback runs
		if (colorCamera && !lastRgbFrame.empty())
		{
			colorCamera->UpdateCapture(lastRgbFrame);
		}
		if (depthCamera && !lastDepthFrame.empty())
		{
			depthCamera->UpdateCapture(lastDepthFrame);
		}
		RCLCPP_INFO(get_logger(), "Waiting for the tracker to stop ...");
		icgThread.join();
		RCLCPP_INFO(get_logger(), "... done");
	}
	rgbImageSub.reset();
	depthImageSub.reset();
	tracker.reset();
	initDone = false;
	runICGDone = false;
	firstImageRgb = false;
	firstImageDepth = false;
}

void ICGTrackerNode::rgbCameraInfoCallback(const sensor_msgs::msg::CameraInfo::ConstSharedPtr msg)
{
	if (colorCamera)
	{
		return;
	}
	colorCamera = std::make_shared<icg::RosCameraColorCamera>("ros_camera_color", msg);
	RCLCPP_INFO(get_logger(), "RGB camera parameters arrived (%ux%u)", msg->width, msg->height);
	rgbCameraInfoSub.reset();
}

void ICGTrackerNode::depthCameraInfoCallback(const sensor_msgs::msg::CameraInfo::ConstSharedPtr msg)
{
	if (depthCamera)
	{
		return;
	}
	depthCamera = std::make_shared<icg::RosCameraDepthCamera>("ros_camera_depth", msg);
	RCLCPP_INFO(get_logger(), "Depth camera parameters arrived (%ux%u)", msg->width, msg->height);
	depthCameraInfoSub.reset();
}

void ICGTrackerNode::manualDetectionCallback(const dexterity_msgs::msg::PixelPosition::ConstSharedPtr msg)
{
	if (!colorCamera || (useDepthCamera && !depthCamera))
	{
		RCLCPP_WARN(get_logger(), "Manual detection points arrived before the camera parameters, ignoring them");
		return;
	}
	if (msg->pixel_coordinates.size() < BOOTSTRAP_POINTS)
	{
		RCLCPP_WARN(get_logger(), "Need %zu manual detection points, got %zu", BOOTSTRAP_POINTS, msg->pixel_coordinates.size());
		return;
	}

	std::vector<cv::Point2f> detectionPoints;
	for (size_t i = 0; i < BOOTSTRAP_POINTS; ++i)
	{
		detectionPoints.emplace_back(float(msg->pixel_coordinates[i].x), float(msg->pixel_coordinates[i].y));
	}
	const std::string bodyName = resolveBodyName(msg->object_model);

	// A new selection ends the current tracking process before a new tracker is set up
	if (initDone)
	{
		RCLCPP_INFO(get_logger(), "New detection points received, restarting the tracker");
		stopICG();
	}
	initICG(bodyName, detectionPoints);
}

void ICGTrackerNode::rgbImageCallback(const sensor_msgs::msg::Image::ConstSharedPtr msg)
{
	if (!initDone)
	{
		return;
	}
	firstImageRgb = true;

	// ROS uses callbacks for new images while ICG uses active polling, so the camera object buffers the latest image
	cv::Mat frame;
	try
	{
		frame = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8)->image;
	}
	catch (const cv_bridge::Exception& e)
	{
		RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 5000, "cv_bridge: %s", e.what());
		return;
	}
	lastRgbFrame = frame;
	publisher->set_msg_header(msg->header);
	colorCamera->UpdateCapture(frame);

	// Run ICG once the first RGB (and depth) images arrived
	if (!runICGDone && (firstImageDepth || !useDepthCamera))
	{
		runICG();
	}

	cv::Mat mixedImage = colorViewer->Image_renderer_mix();
	if (runICGDone && !mixedImage.empty() && mixedImagePub->get_subscription_count() > 0)
	{
		mixedImagePub->publish(*cv_bridge::CvImage(msg->header, sensor_msgs::image_encodings::BGR8, mixedImage).toImageMsg());
	}
}

void ICGTrackerNode::depthImageCallback(const sensor_msgs::msg::Image::ConstSharedPtr msg)
{
	if (!initDone)
	{
		return;
	}
	firstImageDepth = true;

	// ICG expects depth in millimeters as 16UC1; Gazebo publishes meters as 32FC1
	cv::Mat frame;
	try
	{
		if (msg->encoding == sensor_msgs::image_encodings::TYPE_32FC1)
		{
			cv::Mat meters = cv_bridge::toCvCopy(msg)->image;
			// NaN and out of range values mean no measurement, which is 0 for ICG
			cv::patchNaNs(meters, 0.0);
			meters.setTo(0.0f, meters > 65.0f);
			meters.convertTo(frame, CV_16UC1, 1000.0);
		}
		else
		{
			frame = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_16UC1)->image;
		}
	}
	catch (const cv_bridge::Exception& e)
	{
		RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 5000, "cv_bridge: %s", e.what());
		return;
	}
	lastDepthFrame = frame;
	depthCamera->UpdateCapture(frame);

	if (!runICGDone && firstImageRgb)
	{
		runICG();
	}
}

int main(int argc, char* argv[])
{
	rclcpp::init(argc, argv);
	auto node = std::make_shared<ICGTrackerNode>();
	rclcpp::spin(node);
	node.reset();
	rclcpp::shutdown();
	return 0;
}
