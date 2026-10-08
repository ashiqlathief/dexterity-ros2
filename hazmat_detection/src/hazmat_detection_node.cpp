/*
 * hazmat_detection_node.cpp
 *
 * ROS 2 port of the GETbot hazmat detection (HazmatDetection.cpp, M. Weber 2018; matching by
 * D. Gaspers, RoboCup 2017). Every template image in template_path is matched against the camera
 * image with SIFT features; the template pose is found by Hough voting over position, rotation and
 * scale of the matched features.
 *
 * Instead of the GETbot image markers, detections are published as vision_msgs/Detection2DArray
 * (class_id = template file name without extension) together with an annotated image.
 */

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <string>
#include <vector>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <vision_msgs/msg/detection2_d_array.hpp>

class HazmatDetectionNode : public rclcpp::Node
{
	public:
		HazmatDetectionNode();

	private:
		struct HazmatTemplate
		{
			std::string label;
			cv::Size size;
			std::vector<cv::KeyPoint> keypoints;
			cv::Mat descriptors;
		};

		void loadTemplates(const std::filesystem::path& path);
		cv::Rect detectObject(const HazmatTemplate& hazmat, const cv::Size& imageSize,
			const std::vector<cv::KeyPoint>& inputKeypoints, const cv::Mat& inputDescriptors, int& votes) const;
		void cameraImageCallback(const sensor_msgs::msg::Image::ConstSharedPtr msg);

		cv::Ptr<cv::SIFT> sift = cv::SIFT::create();
		std::vector<HazmatTemplate> templates;
		double snnRatio;
		int minVotes;

		rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr imageSub;
		rclcpp::Publisher<vision_msgs::msg::Detection2DArray>::SharedPtr detectionPub;
		rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr imagePub;
};

HazmatDetectionNode::HazmatDetectionNode() : Node("hazmat_detection")
{
	const std::string imageTopic = declare_parameter<std::string>("image_topic", "/wrist_mounted_camera/image");
	std::string templatePath = declare_parameter<std::string>("template_path", "");
	if (templatePath.empty())
	{
		templatePath = ament_index_cpp::get_package_share_directory("hazmat_detection") + "/templates";
	}
	// Ratio test of the nearest to the second nearest descriptor match
	snnRatio = declare_parameter<double>("snn_ratio", 0.9);
	// Minimum number of matched features voting for the same template pose
	minVotes = declare_parameter<int>("min_votes", 10);

	loadTemplates(templatePath);

	detectionPub = create_publisher<vision_msgs::msg::Detection2DArray>("hazmat_detections", 10);
	imagePub = create_publisher<sensor_msgs::msg::Image>("hazmat_detection/image", 1);
	imageSub = create_subscription<sensor_msgs::msg::Image>(imageTopic, rclcpp::SensorDataQoS().keep_last(1),
		std::bind(&HazmatDetectionNode::cameraImageCallback, this, std::placeholders::_1));
	RCLCPP_INFO(get_logger(), "Detecting hazmats in %s", imageTopic.c_str());
}

void HazmatDetectionNode::loadTemplates(const std::filesystem::path& path)
{
	std::vector<std::filesystem::path> files;
	if (std::filesystem::is_directory(path))
	{
		for (const auto& entry : std::filesystem::directory_iterator(path))
		{
			if (entry.is_regular_file())
			{
				files.push_back(entry.path());
			}
		}
	}
	std::sort(files.begin(), files.end());

	for (const auto& file : files)
	{
		cv::Mat image = cv::imread(file.string(), cv::IMREAD_GRAYSCALE);
		if (image.empty())
		{
			RCLCPP_WARN(get_logger(), "Skipping %s, not an image", file.c_str());
			continue;
		}
		HazmatTemplate hazmat;
		hazmat.label = file.stem().string();
		hazmat.size = image.size();
		sift->detectAndCompute(image, cv::noArray(), hazmat.keypoints, hazmat.descriptors);
		templates.push_back(hazmat);
	}
	if (templates.empty())
	{
		RCLCPP_ERROR(get_logger(), "No hazmat templates found in %s", path.c_str());
	}
	else
	{
		RCLCPP_INFO(get_logger(), "%zu hazmat templates loaded from %s", templates.size(), path.c_str());
	}
}

cv::Rect HazmatDetectionNode::detectObject(const HazmatTemplate& hazmat, const cv::Size& imageSize,
	const std::vector<cv::KeyPoint>& inputKeypoints, const cv::Mat& inputDescriptors, int& votes) const
{
	votes = 0;
	if (hazmat.descriptors.rows == 0 || inputDescriptors.rows < 2)
	{
		return {};
	}

	// Match template features to image features
	cv::BFMatcher matcher(cv::NORM_L2, false);
	std::vector<std::vector<cv::DMatch>> knnMatches;
	matcher.knnMatch(hazmat.descriptors, inputDescriptors, knnMatches, 2);
	std::vector<cv::DMatch> matches;
	for (const auto& knn : knnMatches)
	{
		if (knn.size() > 1 && knn[0].distance < snnRatio * knn[1].distance)
		{
			matches.push_back(knn[0]);
		}
	}
	if (matches.empty())
	{
		return {};
	}

	// Hough space over x, y, rotation and scale of the template in the image
	const cv::Point2f templateCenter(hazmat.size.width * 0.5f, hazmat.size.height * 0.5f);
	const int rSize = 12;
	const float sMax = 2.0f;
	const int sSize = 8;
	const float translationFactor = 0.1f;
	const int xSize = std::max(1, int(imageSize.width * translationFactor));
	const int ySize = std::max(1, int(imageSize.height * translationFactor));
	const int dims = 4;
	const int size[] = {xSize, ySize, rSize, sSize};
	cv::SparseMat houghSpace(dims, size, CV_32F);

	for (const auto& match : matches)
	{
		const cv::KeyPoint& templateKeypoint = hazmat.keypoints[match.queryIdx];
		const cv::KeyPoint& inputKeypoint = inputKeypoints[match.trainIdx];

		float angle = inputKeypoint.angle - templateKeypoint.angle;
		if (angle < 0.0f)
		{
			angle += 360.0f;
		}
		const float s = inputKeypoint.size / templateKeypoint.size;
		const float d1x = templateKeypoint.pt.x - templateCenter.x;
		const float d1y = templateKeypoint.pt.y - templateCenter.y;
		const float a = angle / 180.0f * float(M_PI);

		int idx[dims];
		idx[0] = int((inputKeypoint.pt.x - templateKeypoint.pt.x - (std::cos(a) * d1x - std::sin(a) * d1y) * s + d1x + templateCenter.x) * translationFactor);
		idx[1] = int((inputKeypoint.pt.y - templateKeypoint.pt.y - (std::sin(a) * d1x + std::cos(a) * d1y) * s + d1y + templateCenter.y) * translationFactor);
		idx[2] = std::min(rSize - 1, int(angle / (360.0f / rSize)));
		idx[3] = s > sMax ? sSize - 1 : std::min(sSize - 1, int(s / (sMax / sSize)));

		// Votes for a center outside of the image can not be a detection
		if (idx[0] < 0 || idx[0] >= xSize || idx[1] < 0 || idx[1] >= ySize)
		{
			continue;
		}
		houghSpace.ref<float>(idx) += 1.0f;
	}
	if (houghSpace.nzcount() == 0)
	{
		return {};
	}

	int maxIdx[dims];
	double maxVal = 0.0;
	cv::minMaxLoc(houghSpace, nullptr, &maxVal, nullptr, maxIdx);
	votes = int(maxVal);
	if (votes < minVotes)
	{
		return {};
	}

	// Bounding box of the template with the voted pose
	cv::RotatedRect rect(
		cv::Point2f(maxIdx[0] / translationFactor, maxIdx[1] / translationFactor),
		cv::Size2f(hazmat.size) * ((maxIdx[3] + 0.5f) * (sMax / sSize)),
		maxIdx[2] * 360.0f / rSize + 360.0f / rSize / 2.0f);
	return rect.boundingRect() & cv::Rect(cv::Point(0, 0), imageSize);
}

void HazmatDetectionNode::cameraImageCallback(const sensor_msgs::msg::Image::ConstSharedPtr msg)
{
	cv_bridge::CvImageConstPtr cvPtr;
	try
	{
		cvPtr = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::BGR8);
	}
	catch (const cv_bridge::Exception& e)
	{
		RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 5000, "cv_bridge exception: %s", e.what());
		return;
	}

	cv::Mat gray;
	cv::cvtColor(cvPtr->image, gray, cv::COLOR_BGR2GRAY);
	std::vector<cv::KeyPoint> inputKeypoints;
	cv::Mat inputDescriptors;
	sift->detectAndCompute(gray, cv::noArray(), inputKeypoints, inputDescriptors);

	vision_msgs::msg::Detection2DArray detections;
	detections.header = msg->header;
	const bool publishImage = imagePub->get_subscription_count() > 0;
	cv::Mat annotated = publishImage ? cvPtr->image.clone() : cv::Mat();

	for (const auto& hazmat : templates)
	{
		int votes = 0;
		const cv::Rect bb = detectObject(hazmat, gray.size(), inputKeypoints, inputDescriptors, votes);
		if (bb.area() == 0)
		{
			continue;
		}

		vision_msgs::msg::Detection2D detection;
		detection.header = msg->header;
		detection.id = hazmat.label;
		detection.bbox.center.position.x = bb.x + bb.width * 0.5;
		detection.bbox.center.position.y = bb.y + bb.height * 0.5;
		detection.bbox.size_x = bb.width;
		detection.bbox.size_y = bb.height;
		vision_msgs::msg::ObjectHypothesisWithPose hypothesis;
		hypothesis.hypothesis.class_id = hazmat.label;
		hypothesis.hypothesis.score = votes; // Number of features voting for this pose, not a probability
		detection.results.push_back(hypothesis);
		detections.detections.push_back(detection);

		if (publishImage)
		{
			cv::rectangle(annotated, bb, cv::Scalar(0, 0, 255), 2);
			cv::putText(annotated, hazmat.label, bb.tl() + cv::Point(5, 20), cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 0, 255), 2);
		}
	}

	detectionPub->publish(detections);
	if (publishImage)
	{
		imagePub->publish(*cv_bridge::CvImage(msg->header, sensor_msgs::image_encodings::BGR8, annotated).toImageMsg());
	}
}

int main(int argc, char* argv[])
{
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<HazmatDetectionNode>());
	rclcpp::shutdown();
	return 0;
}
