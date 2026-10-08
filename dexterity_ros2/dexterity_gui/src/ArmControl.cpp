/*
 * ArmControl.cpp
 */

#include <dexterity_gui/ArmControl.hpp>
#include <dexterity_gui/TrackerGraphicsScene.hpp>
#include <ui_ArmControl.h>

#include <filesystem>

#include <QComboBox>
#include <QDialog>
#include <QHBoxLayout>
#include <QLabel>
#include <QLineEdit>
#include <QMessageBox>
#include <QPixmap>
#include <QPushButton>
#include <QGraphicsView>
#include <QCheckBox>
#include <QSpinBox>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <pluginlib/class_list_macros.hpp>

namespace dexterity_gui
{

namespace
{
const char* MANUAL_DETECTION_TOPIC = "GUI_MANUAL_DETECTION_POINTS";

// .cao files shipped with the dexterity package, plus the ICG / M3T body names of the old GUI
QStringList objectModels()
{
	QStringList models;
	try
	{
		std::filesystem::path dir = std::filesystem::path(ament_index_cpp::get_package_share_directory("dexterity")) / "config" / "cad_models";
		for (const auto& entry : std::filesystem::directory_iterator(dir))
		{
			if (entry.path().extension() == ".cao")
			{
				models << QString::fromStdString(entry.path().filename().string());
			}
		}
	}
	catch (const std::exception&)
	{
	}
	models.sort();
	if (models.contains("cube_at_rl2.cao"))
	{
		models.move(models.indexOf("cube_at_rl2.cao"), 0);
	}
	models << "asymmetric_pipestar_without_cap" << "asymmetric_pipestar_with_cap" << "cube";
	return models;
}
}

ArmControl::ArmControl()
{
	setObjectName("DexterityArmControl");
}

ArmControl::~ArmControl() = default;

void ArmControl::initPlugin(qt_gui_cpp::PluginContext& context)
{
	// The layout lives in ui/ArmControl.ui (edit it with Qt Designer)
	widget = new QWidget();
	ui = std::make_unique<Ui::ArmControlForm>();
	ui->setupUi(widget);
	if (context.serialNumber() > 1)
	{
		widget->setWindowTitle(widget->windowTitle() + " (" + QString::number(context.serialNumber()) + ")");
	}

	setupActions();
	setupPerception();
	setupTracker();
	context.addWidget(widget);

	connect(this, &ArmControl::imageReceived, this, &ArmControl::updateImage, Qt::QueuedConnection);
	connect(this, &ArmControl::statusChanged, this, &ArmControl::setStatus, Qt::QueuedConnection);
	connect(this, &ArmControl::feedbackReceived, this, &ArmControl::showFeedback, Qt::QueuedConnection);

	actionClient = rclcpp_action::create_client<DexterityAction>(node_, "dexterity_action");
	manualDetectionPublisher = node_->create_publisher<dexterity_msgs::msg::PixelPosition>(MANUAL_DETECTION_TOPIC, 1);
	onImageTopicsChanged();
}

void ArmControl::shutdownPlugin()
{
	cameraSubscriber.reset();
	trackerImageSubscriber.reset();
	manualDetectionPublisher.reset();
	actionClient.reset();
}

void ArmControl::setupActions()
{
	struct ButtonDef { QPushButton* button; const char* command; const char* status; };
	const ButtonDef buttons[] = {
		{ui->startTouchButton, "touch", "TOUCH OPERATION"},
		{ui->startFollowButton, "inspect_non_stationary", "FOLLOW OPERATION"},
		{ui->startInspectButton, "inspect_stationary", "INSPECT OPERATION"},
		{ui->startGraspButton, "grasp", "GRASP OPERATION"},
		{ui->stopButton, "stop", "STOPPED"},
		{ui->resetArmButton, "reset", "ARM HOME POSITION"}};
	for (const auto& def : buttons)
	{
		const std::string command = def.command;
		const QString status = def.status;
		connect(def.button, &QPushButton::clicked, this, [this, command, status]() { runAction(command, status); });
	}
}

void ArmControl::setupPerception()
{
	ui->objectModel->addItems(objectModels());
	updateVispFields();

	connect(ui->trackerName, &QComboBox::currentTextChanged, this, [this](const QString& name)
	{
		updateVispFields();
		// The tracker image stream of the old GUI depends on the tracker
		if (name == "icg")
		{
			ui->trackerImageTopic->setText("/ICG_Tracker_Streaming/image_raw");
		}
		else if (name == "m3t")
		{
			ui->trackerImageTopic->setText("/M3T_Tracker_Streaming/image_raw");
		}
		onImageTopicsChanged();
	});
}

// Tracking mode and the AprilTag bootstrapping only configure the ViSP tracker; icg / m3t ignore them
void ArmControl::updateVispFields()
{
	const bool visp = ui->trackerName->currentText() == "visp";
	for (QWidget* field : {
		static_cast<QWidget*>(ui->trackerMode), static_cast<QWidget*>(ui->trackerModeLabel),
		static_cast<QWidget*>(ui->vispLabel),
		static_cast<QWidget*>(ui->tagSize), static_cast<QWidget*>(ui->tagSizeLabel),
		static_cast<QWidget*>(ui->tagFamily), static_cast<QWidget*>(ui->tagFamilyLabel),
		static_cast<QWidget*>(ui->poseEstimationMethod), static_cast<QWidget*>(ui->poseEstimationMethodLabel)})
	{
		field->setEnabled(visp);
	}
}

void ArmControl::setupTracker()
{
	connect(ui->cameraTopic, &QLineEdit::editingFinished, this, &ArmControl::onImageTopicsChanged);
	connect(ui->trackerImageTopic, &QLineEdit::editingFinished, this, &ArmControl::onImageTopicsChanged);
	connect(ui->startBootstrapButton, &QPushButton::clicked, this, &ArmControl::onStartBootstrap);
	connect(ui->stopBootstrapButton, &QPushButton::clicked, this, &ArmControl::onStopBootstrap);
	connect(ui->startTrackerButton, &QPushButton::clicked, this, &ArmControl::onStartTracker);
	connect(ui->showReferenceButton, &QPushButton::clicked, this, &ArmControl::onShowReference);

	scene = new TrackerGraphicsScene(this);
	ui->trackerView->setScene(scene);
}

/* -------------------------------------------------------------
 * Actions
 * ------------------------------------------------------------- */

void ArmControl::runAction(const std::string& command, const QString& statusText)
{
	if (!actionClient->action_server_is_ready())
	{
		if (command != "update_parameters")
		{
			setStatus("DEXTERITY NODE NOT AVAILABLE");
		}
		return;
	}

	DexterityAction::Goal goal;
	goal.command = command;
	goal.accuracy = ui->minAccuracy->value();
	goal.tracker_mode = ui->trackerMode->currentText().toStdString();
	goal.tracker_name = ui->trackerName->currentText().toStdString();
	goal.object_model = ui->objectModel->currentText().toStdString();
	goal.object_side = ui->objectSide->currentText().toStdString();
	goal.tag_size = ui->tagSize->value() / 1000.0;
	goal.tag_family = ui->tagFamily->text().toStdString();
	goal.pose_estimation_method = ui->poseEstimationMethod->text().toStdString();
	goal.inspection_pose_error_margin = ui->poseMargin->value();
	goal.inspection_orientation_error_margin = ui->orientationMargin->value();
	goal.open_loop_mode = ui->openLoopMode->isChecked();
	goal.orientation_lock = ui->orientationLock->isChecked();
	goal.inspection_distance = ui->inspectionDistance->value() / 100.0;
	goal.gripper_frame_id = ui->gripperFrame->currentText().toStdString();
	goal.truncation_type = ui->truncationType->currentText().toStdString();
	goal.planning_timeout = ui->planningTimeout->value();

	rclcpp_action::Client<DexterityAction>::SendGoalOptions options;
	options.goal_response_callback = [this](const GoalHandle::SharedPtr& handle)
	{
		if (!handle)
		{
			Q_EMIT statusChanged("GOAL REJECTED");
		}
	};
	options.feedback_callback = [this](GoalHandle::SharedPtr, const std::shared_ptr<const DexterityAction::Feedback> feedback)
	{
		Q_EMIT feedbackReceived(feedback->percentage);
	};
	if (command != "update_parameters")
	{
		options.result_callback = [this, statusText](const GoalHandle::WrappedResult& result)
		{
			switch (result.code)
			{
				case rclcpp_action::ResultCode::SUCCEEDED:
					Q_EMIT statusChanged(statusText + " - DONE");
					break;
				case rclcpp_action::ResultCode::CANCELED:
					Q_EMIT statusChanged(statusText + " - CANCELLED");
					break;
				default:
					// A task is aborted when it is stopped or replaced by a new one; the new status is already shown
					break;
			}
		};
		setStatus(statusText);
		ui->feedbackLabel->clear();
	}
	actionClient->async_send_goal(goal, options);
}

void ArmControl::setStatus(QString text)
{
	ui->statusLabel->setText(text);
}

void ArmControl::showFeedback(double distance)
{
	ui->feedbackLabel->setText(distance >= 0 ? QString("Distance to object: %1 m").arg(distance, 0, 'f', 3) : "Object not found yet");
}

/* -------------------------------------------------------------
 * Tracker tab (manual detection points for ICG / M3T)
 * ------------------------------------------------------------- */

void ArmControl::onImageTopicsChanged()
{
	if (!node_)
	{
		return;
	}
	cameraSubscriber = node_->create_subscription<sensor_msgs::msg::Image>(ui->cameraTopic->text().toStdString(), rclcpp::SensorDataQoS().keep_last(1),
		[this](sensor_msgs::msg::Image::ConstSharedPtr msg) { imageCallback(msg, false); });
	trackerImageSubscriber = node_->create_subscription<sensor_msgs::msg::Image>(ui->trackerImageTopic->text().toStdString(), rclcpp::SensorDataQoS().keep_last(1),
		[this](sensor_msgs::msg::Image::ConstSharedPtr msg) { imageCallback(msg, true); });
}

void ArmControl::imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr& msg, bool fromTracker)
{
	// Camera image while not tracking, tracker image (with the rendered model) while tracking
	if (fromTracker != trackingInProgress)
	{
		return;
	}
	try
	{
		cv_bridge::CvImageConstPtr cvImage = cv_bridge::toCvShare(msg, "rgb8");
		QImage image(cvImage->image.data, cvImage->image.cols, cvImage->image.rows, static_cast<int>(cvImage->image.step), QImage::Format_RGB888);
		Q_EMIT imageReceived(image.copy());
	}
	catch (const cv_bridge::Exception& e)
	{
		RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 5000, "Cannot convert image: %s", e.what());
	}
}

void ArmControl::updateImage(QImage image)
{
	if (freezeImage)
	{
		return;
	}
	scene->clear();
	scene->addPixmap(QPixmap::fromImage(image));
	scene->setSceneRect(0, 0, image.width(), image.height());
	ui->trackerView->fitInView(scene->sceneRect(), Qt::KeepAspectRatio);
}

void ArmControl::onStartBootstrap()
{
	freezeImage = true;
	scene->drawPoints = true;
	scene->bootstrapPoints.clear();
	trackingInProgress = false;
	setStatus("SELECT 6 DETECTION POINTS");
}

void ArmControl::onStopBootstrap()
{
	freezeImage = false;
	scene->drawPoints = false;
	scene->bootstrapPoints.clear();
	trackingInProgress = false;
}

void ArmControl::onStartTracker()
{
	if (!scene->verifyBootstrapPoints())
	{
		QMessageBox::warning(widget, "Dexterity", QString("Select exactly %1 points for the tracker (selected: %2).")
			.arg(TrackerGraphicsScene::BOOTSTRAP_POINTS).arg(scene->bootstrapPoints.size()));
		return;
	}
	dexterity_msgs::msg::PixelPosition msg;
	msg.pixel_coordinates = scene->bootstrapPoints;
	msg.object_model = ui->objectModel->currentText().toStdString();
	manualDetectionPublisher->publish(msg);
	scene->drawPoints = false;
	freezeImage = false;
	trackingInProgress = true;
	setStatus("TRACKER STARTED");
}

void ArmControl::onShowReference()
{
	const QString model = ui->objectModel->currentText();
	QString file;
	if (model.contains("asymmetric_pipestar_with_cap"))
	{
		file = "asymmetric_pipestar_with_cap_manual_detector.png";
	}
	else if (model.contains("asymmetric_pipestar_without_cap"))
	{
		file = "asymmetric_pipestar_without_cap_manual_detector.png";
	}
	else if (model.contains("pipestar"))
	{
		file = "pipestar_unsym_manual_detector.png";
	}
	else if (model.contains("cube"))
	{
		file = "cube.png";
	}
	if (file.isEmpty())
	{
		QMessageBox::information(widget, "Dexterity", "No reference image for " + model);
		return;
	}
	const QString path = QString::fromStdString(ament_index_cpp::get_package_share_directory("dexterity_gui")) + "/images/" + file;
	QDialog dialog(widget);
	dialog.setWindowTitle("Reference: " + model);
	auto* layout = new QHBoxLayout(&dialog);
	auto* label = new QLabel;
	label->setPixmap(QPixmap(path));
	layout->addWidget(label);
	dialog.exec();
}

/* -------------------------------------------------------------
 * Settings (stored by rqt between sessions)
 * ------------------------------------------------------------- */

void ArmControl::saveSettings(qt_gui_cpp::Settings&, qt_gui_cpp::Settings& s) const
{
	s.setValue("tracker_name", ui->trackerName->currentText());
	s.setValue("tracker_mode", ui->trackerMode->currentText());
	s.setValue("object_model", ui->objectModel->currentText());
	s.setValue("object_side", ui->objectSide->currentText());
	s.setValue("min_accuracy", ui->minAccuracy->value());
	s.setValue("tag_size", ui->tagSize->value());
	s.setValue("tag_family", ui->tagFamily->text());
	s.setValue("pose_estimation_method", ui->poseEstimationMethod->text());
	s.setValue("open_loop_mode", ui->openLoopMode->isChecked());
	s.setValue("orientation_lock", ui->orientationLock->isChecked());
	s.setValue("inspection_distance", ui->inspectionDistance->value());
	s.setValue("gripper_frame", ui->gripperFrame->currentText());
	s.setValue("truncation_type", ui->truncationType->currentText());
	s.setValue("pose_margin", ui->poseMargin->value());
	s.setValue("orientation_margin", ui->orientationMargin->value());
	s.setValue("planning_timeout", ui->planningTimeout->value());
	s.setValue("camera_topic", ui->cameraTopic->text());
	s.setValue("tracker_image_topic", ui->trackerImageTopic->text());
}

void ArmControl::restoreSettings(const qt_gui_cpp::Settings&, const qt_gui_cpp::Settings& s)
{
	auto restoreText = [&s](QComboBox* combo, const char* key)
	{
		if (s.contains(key)) combo->setCurrentText(s.value(key).toString());
	};
	auto restoreInt = [&s](QSpinBox* spin, const char* key)
	{
		if (s.contains(key)) spin->setValue(s.value(key).toInt());
	};
	auto restoreBool = [&s](QCheckBox* box, const char* key)
	{
		if (s.contains(key)) box->setChecked(s.value(key).toBool());
	};
	restoreText(ui->trackerName, "tracker_name");
	restoreText(ui->trackerMode, "tracker_mode");
	restoreText(ui->objectModel, "object_model");
	restoreText(ui->objectSide, "object_side");
	restoreInt(ui->minAccuracy, "min_accuracy");
	restoreInt(ui->tagSize, "tag_size");
	if (s.contains("tag_family")) ui->tagFamily->setText(s.value("tag_family").toString());
	if (s.contains("pose_estimation_method")) ui->poseEstimationMethod->setText(s.value("pose_estimation_method").toString());
	restoreBool(ui->openLoopMode, "open_loop_mode");
	restoreBool(ui->orientationLock, "orientation_lock");
	restoreInt(ui->inspectionDistance, "inspection_distance");
	restoreText(ui->gripperFrame, "gripper_frame");
	restoreText(ui->truncationType, "truncation_type");
	restoreInt(ui->poseMargin, "pose_margin");
	restoreInt(ui->orientationMargin, "orientation_margin");
	restoreInt(ui->planningTimeout, "planning_timeout");
	if (s.contains("camera_topic")) ui->cameraTopic->setText(s.value("camera_topic").toString());
	if (s.contains("tracker_image_topic")) ui->trackerImageTopic->setText(s.value("tracker_image_topic").toString());
	onImageTopicsChanged();
}

} // namespace dexterity_gui

PLUGINLIB_EXPORT_CLASS(dexterity_gui::ArmControl, rqt_gui_cpp::Plugin)
