/*
 * ArmControl.hpp
 *
 * rqt plugin to operate the dexterity node. ROS 2 port of the ArmControl widget (object_handling_gui),
 * which ran inside the GET Lab operator interface.
 *
 *   rqt --standalone dexterity_gui/ArmControl
 */

#ifndef DEXTERITY_GUI_ARMCONTROL_HPP_
#define DEXTERITY_GUI_ARMCONTROL_HPP_

#include <memory>
#include <string>

#include <QImage>
#include <QObject>
#include <QString>
#include <QWidget>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <rqt_gui_cpp/plugin.h>
#include <sensor_msgs/msg/image.hpp>

#include <dexterity_msgs/action/dexterity.hpp>
#include <dexterity_msgs/msg/pixel_position.hpp>

namespace Ui
{
class ArmControlForm;
}

namespace dexterity_gui
{

class TrackerGraphicsScene;

class ArmControl : public rqt_gui_cpp::Plugin
{
	Q_OBJECT

	public:
		using DexterityAction = dexterity_msgs::action::Dexterity;
		using GoalHandle = rclcpp_action::ClientGoalHandle<DexterityAction>;

		ArmControl();
		~ArmControl() override;
		void initPlugin(qt_gui_cpp::PluginContext& context) override;
		void shutdownPlugin() override;
		void saveSettings(qt_gui_cpp::Settings& pluginSettings, qt_gui_cpp::Settings& instanceSettings) const override;
		void restoreSettings(const qt_gui_cpp::Settings& pluginSettings, const qt_gui_cpp::Settings& instanceSettings) override;

	Q_SIGNALS:
		// Emitted from ROS callbacks, handled in the Qt thread
		void imageReceived(QImage image);
		void statusChanged(QString text);
		void feedbackReceived(double distance);

	private Q_SLOTS:
		void updateImage(QImage image);
		void setStatus(QString text);
		void showFeedback(double distance);
		void onStartBootstrap();
		void onStopBootstrap();
		void onStartTracker();
		void onShowReference();
		void onImageTopicsChanged();

	private:
		QWidget* widget = nullptr;
		std::unique_ptr<Ui::ArmControlForm> ui;
		TrackerGraphicsScene* scene = nullptr;
		bool freezeImage = false;
		bool trackingInProgress = false;

		rclcpp_action::Client<DexterityAction>::SharedPtr actionClient;
		rclcpp::Publisher<dexterity_msgs::msg::PixelPosition>::SharedPtr manualDetectionPublisher;
		rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr cameraSubscriber;
		rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr trackerImageSubscriber;

		void setupActions();
		void setupPerception();
		void setupTracker();
		void updateVispFields();

		void runAction(const std::string& command, const QString& statusText);
		void imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr& msg, bool fromTracker);
};

} // namespace dexterity_gui

#endif /* DEXTERITY_GUI_ARMCONTROL_HPP_ */
