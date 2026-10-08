"""
Ground truth evaluation of dexterity in the Gazebo simulation (ROS 2 port of the simulator mode of the ROS 1
closed_loop_touch, which recorded distance and planning time statistics against the true pose of the cube).

The true pose of the object comes from Gazebo (/world/<world>/pose/info, read with gz-transport because the
ROS bridge drops the model names) and is published as TF frame "ground_truth_object". For every dexterity task
(started / finished events on /dexterity/events) the node writes to output_dir:

  <run>_samples.csv    periodic samples: tracking error (tracked object_origin vs ground truth), true distance
                       gripper -> point of interest and its error to the inspection distance, approach angle error
  <run>_planning.csv   every MoveIt plan of the controller: state, success, planning time
  summary.csv          one line per task with the final errors and statistics
"""

import csv
import math
import os
import threading
import time
from datetime import datetime

import numpy as np
import rclpy
from geometry_msgs.msg import TransformStamped
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time
from tf2_ros import Buffer, TransformBroadcaster, TransformListener

from dexterity_msgs.msg import DexterityEvent

from gz.msgs10.pose_v_pb2 import Pose_V
from gz.transport13 import Node as GzNode

# Targets of dexterity_ur/launch/ur_sim.launch.py, used when object_model is not set
KNOWN_TARGETS = ["apriltag_target", "asymmetric_pipestar_with_cap", "asymmetric_pipestar_without_cap", "pipestar_without_pipes",
                 "object_robocup_pipestar", "cube", "april_tag_cube"]


def quaternion_to_matrix(x, y, z, w):
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def matrix_to_quaternion(r):
    w = math.sqrt(max(0.0, 1 + r[0, 0] + r[1, 1] + r[2, 2])) / 2
    x = math.copysign(math.sqrt(max(0.0, 1 + r[0, 0] - r[1, 1] - r[2, 2])) / 2, r[2, 1] - r[1, 2])
    y = math.copysign(math.sqrt(max(0.0, 1 - r[0, 0] + r[1, 1] - r[2, 2])) / 2, r[0, 2] - r[2, 0])
    z = math.copysign(math.sqrt(max(0.0, 1 - r[0, 0] - r[1, 1] + r[2, 2])) / 2, r[1, 0] - r[0, 1])
    return x, y, z, w


def rpy_to_matrix(roll, pitch, yaw):
    cr, sr, cp, sp, cy, sy = math.cos(roll), math.sin(roll), math.cos(pitch), math.sin(pitch), math.cos(yaw), math.sin(yaw)
    return np.array([[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
                     [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
                     [-sp, cp * sr, cp * cr]])


def homogeneous(rotation, translation):
    t = np.eye(4)
    t[:3, :3] = rotation
    t[:3, 3] = translation
    return t


def from_transform(transform):
    q, p = transform.rotation, transform.translation
    return homogeneous(quaternion_to_matrix(q.x, q.y, q.z, q.w), [p.x, p.y, p.z])


def rotation_angle_deg(rotation):
    return math.degrees(math.acos(max(-1.0, min(1.0, (np.trace(rotation) - 1) / 2))))


class Run:
    """Files and statistics of one dexterity task."""

    SAMPLE_FIELDS = ["time_s", "tracked", "tracked_pose_age_s", "tracking_position_error_mm", "tracking_rotation_error_deg",
                     "true_distance_m", "distance_error_mm", "approach_angle_error_deg", "gripper_x", "gripper_y", "gripper_z"]
    PLANNING_FIELDS = ["time_s", "controller_state", "success", "planning_time_s", "distance_to_object_m"]

    def __init__(self, directory, event):
        self.start = time.time()
        self.event = event
        self.name = f"{datetime.now():%Y%m%d_%H%M%S}_{event.command}_{event.tracker_name}"
        self.samples_file = open(os.path.join(directory, self.name + "_samples.csv"), "w", newline="")
        self.planning_file = open(os.path.join(directory, self.name + "_planning.csv"), "w", newline="")
        self.samples = csv.DictWriter(self.samples_file, self.SAMPLE_FIELDS)
        self.planning = csv.DictWriter(self.planning_file, self.PLANNING_FIELDS)
        self.samples.writeheader()
        self.planning.writeheader()
        self.tracking_position_errors = []
        self.tracking_rotation_errors = []
        self.planning_times = []
        self.failed_plans = 0
        self.last_sample = None

    def elapsed(self):
        return round(time.time() - self.start, 3)

    def close(self):
        self.samples_file.close()
        self.planning_file.close()


class EvaluationNode(Node):
    def __init__(self):
        super().__init__("dexterity_evaluation")
        self.world = self.declare_parameter("world_name", "empty").value
        self.robot_model = self.declare_parameter("robot_model", "ur").value
        self.object_model = self.declare_parameter("object_model", "").value      # empty: the only other model of the world
        offset = self.declare_parameter("object_offset", [0.0] * 6).value          # x y z roll pitch yaw of the tracked frame in the model
        self.world_frame = self.declare_parameter("world_frame", "world").value    # ROS frame of the robot model's origin
        self.tracked_frame = self.declare_parameter("tracked_frame", "object_origin").value
        self.poi_frame = self.declare_parameter("poi_frame", "object_frame").value
        self.output_dir = os.path.expanduser(self.declare_parameter("output_dir", "~/dexterity_evaluation").value)
        rate = self.declare_parameter("rate", 10.0).value
        os.makedirs(self.output_dir, exist_ok=True)

        self.offset = homogeneous(rpy_to_matrix(*offset[3:6]), offset[:3])
        self.lock = threading.Lock()
        self.ground_truth = None      # tracked object frame in world_frame (4x4)
        self.model_name = None
        self.run = None
        self.last_tracked_stamp = None

        self.tf_buffer = Buffer(cache_time=Duration(seconds=30))
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.create_subscription(DexterityEvent, "/dexterity/events", self.on_event, 50)
        self.create_timer(1.0 / rate, self.on_timer)

        # Gazebo poses (separate gz-transport thread)
        self.gz_node = GzNode()
        topic = f"/world/{self.world}/pose/info"
        if not self.gz_node.subscribe(Pose_V, topic, self.on_gazebo_poses):
            self.get_logger().error(f"Cannot subscribe to the Gazebo topic {topic}")
        self.get_logger().info(f"Evaluation running, results in {self.output_dir}")

    # ---------------------------------------------------------------- ground truth

    def on_gazebo_poses(self, message):
        poses = {}
        for pose in message.pose:
            if pose.name and pose.name not in poses:   # models come first, link names may repeat
                p, q = pose.position, pose.orientation
                poses[pose.name] = homogeneous(quaternion_to_matrix(q.x, q.y, q.z, q.w), [p.x, p.y, p.z])
        if self.robot_model not in poses:
            return
        name = self.object_model or next((t for t in KNOWN_TARGETS if t in poses), None)
        if name not in poses:
            if self.model_name is None:
                self.get_logger().warn(f"Object model '{self.object_model}' not found in Gazebo, set object_model",
                                       throttle_duration_sec=10.0)
            return
        with self.lock:
            self.model_name = name
            self.ground_truth = np.linalg.inv(poses[self.robot_model]) @ poses[name] @ self.offset

    def publish_ground_truth(self):
        with self.lock:
            truth = None if self.ground_truth is None else self.ground_truth.copy()
        if truth is None:
            return
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = self.world_frame
        t.child_frame_id = "ground_truth_object"
        t.transform.translation.x, t.transform.translation.y, t.transform.translation.z = truth[:3, 3]
        q = matrix_to_quaternion(truth[:3, :3])
        t.transform.rotation.x, t.transform.rotation.y, t.transform.rotation.z, t.transform.rotation.w = q
        self.tf_broadcaster.sendTransform(t)

    # ---------------------------------------------------------------- dexterity events

    def on_event(self, event):
        if event.event == DexterityEvent.TASK_STARTED:
            if self.run:
                self.finish_run("replaced")
            self.run = Run(self.output_dir, event)
            self.last_tracked_stamp = None
            self.get_logger().info(f"Recording {self.run.name} (object model in Gazebo: {self.model_name})")
        elif event.event == DexterityEvent.PLANNING and self.run:
            self.run.planning.writerow({
                "time_s": self.run.elapsed(), "controller_state": event.controller_state, "success": int(event.success),
                "planning_time_s": round(event.planning_time, 4), "distance_to_object_m": round(event.distance_to_object, 4)})
            self.run.planning_times.append(event.planning_time)
            self.run.failed_plans += 0 if event.success else 1
        elif event.event == DexterityEvent.TASK_FINISHED and self.run:
            self.on_timer()   # final sample
            self.finish_run(event.result)

    def finish_run(self, result):
        run = self.run
        self.run = None
        run.close()
        last = run.last_sample or {}
        summary = {
            "run": run.name, "command": run.event.command, "tracker": run.event.tracker_name,
            "object_model": run.event.object_model, "gazebo_model": self.model_name, "object_side": run.event.object_side,
            "result": result, "duration_s": run.elapsed(),
            "plans": len(run.planning_times), "failed_plans": run.failed_plans,
            "mean_planning_time_s": round(float(np.mean(run.planning_times)), 4) if run.planning_times else "",
            "max_planning_time_s": round(float(np.max(run.planning_times)), 4) if run.planning_times else "",
            "final_true_distance_m": last.get("true_distance_m", ""),
            "final_distance_error_mm": last.get("distance_error_mm", ""),
            "final_approach_angle_error_deg": last.get("approach_angle_error_deg", ""),
            "mean_tracking_position_error_mm": round(float(np.mean(run.tracking_position_errors)), 2) if run.tracking_position_errors else "",
            "max_tracking_position_error_mm": round(float(np.max(run.tracking_position_errors)), 2) if run.tracking_position_errors else "",
            "mean_tracking_rotation_error_deg": round(float(np.mean(run.tracking_rotation_errors)), 2) if run.tracking_rotation_errors else "",
            "max_tracking_rotation_error_deg": round(float(np.max(run.tracking_rotation_errors)), 2) if run.tracking_rotation_errors else "",
        }
        path = os.path.join(self.output_dir, "summary.csv")
        new_file = not os.path.exists(path)
        with open(path, "a", newline="") as f:
            writer = csv.DictWriter(f, list(summary))
            if new_file:
                writer.writeheader()
            writer.writerow(summary)
        self.get_logger().info(
            f"{run.name}: {result}, {summary['duration_s']} s, {summary['plans']} plans ({summary['failed_plans']} failed), "
            f"final distance error {summary['final_distance_error_mm']} mm, approach error {summary['final_approach_angle_error_deg']} deg, "
            f"tracking error mean {summary['mean_tracking_position_error_mm']} mm / {summary['mean_tracking_rotation_error_deg']} deg")

    # ---------------------------------------------------------------- periodic samples

    def lookup(self, target, source):
        try:
            return self.tf_buffer.lookup_transform(target, source, Time())
        except Exception:
            return None

    def on_timer(self):
        self.publish_ground_truth()
        run = self.run
        with self.lock:
            truth = None if self.ground_truth is None else self.ground_truth.copy()
        if run is None or truth is None:
            return
        sample = {"time_s": run.elapsed(), "tracked": 0}

        # Tracking error: latest pose of the tracker (TF of dexterity at the image time) vs ground truth
        tracked = self.lookup(self.world_frame, self.tracked_frame)
        if tracked is not None:
            stamp = Time.from_msg(tracked.header.stamp)
            age = (self.get_clock().now() - stamp).nanoseconds * 1e-9
            if age < 1.0:
                estimate = from_transform(tracked.transform)
                position_error = float(np.linalg.norm(estimate[:3, 3] - truth[:3, 3]) * 1000)
                rotation_error = rotation_angle_deg(estimate[:3, :3].T @ truth[:3, :3])
                sample.update(tracked=1, tracked_pose_age_s=round(age, 3),
                              tracking_position_error_mm=round(position_error, 2), tracking_rotation_error_deg=round(rotation_error, 2))
                if self.last_tracked_stamp != stamp:   # count every tracker update once
                    run.tracking_position_errors.append(position_error)
                    run.tracking_rotation_errors.append(rotation_error)
                    self.last_tracked_stamp = stamp

        # True distance gripper -> point of interest (POI offset of the selected side applied to the true object)
        gripper = self.lookup(self.world_frame, run.event.gripper_frame_id)
        if gripper is not None:
            g = from_transform(gripper.transform)
            poi_offset = self.lookup(self.tracked_frame, self.poi_frame)
            true_poi = truth @ (from_transform(poi_offset.transform) if poi_offset is not None else np.eye(4))
            distance = float(np.linalg.norm(g[:3, 3] - true_poi[:3, 3]))
            # Dexterity's orientation criterion: gripper +X against the approach direction -Z of the point of interest
            cosine = float(np.dot(g[:3, 0], -true_poi[:3, 2]))
            sample.update(true_distance_m=round(distance, 4),
                          distance_error_mm=round((distance - run.event.inspection_distance) * 1000, 2),
                          approach_angle_error_deg=round(math.degrees(math.acos(max(-1.0, min(1.0, cosine)))), 2),
                          gripper_x=round(g[0, 3], 4), gripper_y=round(g[1, 3], 4), gripper_z=round(g[2, 3], 4))
        run.samples.writerow(sample)
        run.last_sample = sample


def main():
    rclpy.init()
    node = EvaluationNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node.run:
            node.finish_run("evaluation stopped")
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
