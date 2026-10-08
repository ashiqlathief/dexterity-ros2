/*
 * TrackerGraphicsScene.hpp
 *
 * Camera image in which the operator clicks the manual detection points for the ICG / M3T trackers.
 * The image is added unscaled at the scene origin, so scene coordinates are image pixel coordinates.
 */

#ifndef DEXTERITY_GUI_TRACKERGRAPHICSSCENE_HPP_
#define DEXTERITY_GUI_TRACKERGRAPHICSSCENE_HPP_

#include <vector>

#include <QBrush>
#include <QGraphicsScene>
#include <QGraphicsSceneMouseEvent>
#include <QPen>

#include <geometry_msgs/msg/point.hpp>

namespace dexterity_gui
{

class TrackerGraphicsScene : public QGraphicsScene
{
	public:
		static constexpr size_t BOOTSTRAP_POINTS = 6; // Number of points sent to the tracker

		explicit TrackerGraphicsScene(QObject* parent = nullptr) : QGraphicsScene(parent) {}

		bool drawPoints = false;
		std::vector<geometry_msgs::msg::Point> bootstrapPoints;

		bool verifyBootstrapPoints() const { return bootstrapPoints.size() == BOOTSTRAP_POINTS; }

	protected:
		void mousePressEvent(QGraphicsSceneMouseEvent* mouseEvent) override
		{
			if (drawPoints)
			{
				const QPointF p = mouseEvent->scenePos();
				addEllipse(p.x() - 4, p.y() - 4, 8, 8, QPen(Qt::red), QBrush(Qt::red));
				geometry_msgs::msg::Point point;
				point.x = p.x();
				point.y = p.y();
				bootstrapPoints.push_back(point);
			}
			QGraphicsScene::mousePressEvent(mouseEvent);
		}
};

} // namespace dexterity_gui

#endif /* DEXTERITY_GUI_TRACKERGRAPHICSSCENE_HPP_ */
