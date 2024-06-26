// Copyright 2023 Chen Tingxu
#include "outpost_tracker/tracker_node.hpp"

// STD
#include <memory>
#include <vector>
#include <angles/angles.h>

namespace outpost_auto_aim
{
ArmorTrackerNode::ArmorTrackerNode(const rclcpp::NodeOptions & options)
: Node("outpost_tracker", options)
{
  RCLCPP_INFO(this->get_logger(), "Starting TrackerNode!");

  tracker_debug_ = this->declare_parameter("debug", false);

  // Maximum allowable armor distance in the XOY plane
  max_armor_distance_ = this->declare_parameter("max_armor_distance", 15.0);

  // Tracker
  double max_match_distance = this->declare_parameter("tracker.max_match_distance", 0.15);
  double max_match_yaw_diff = this->declare_parameter("tracker.max_match_yaw_diff", 1.0);
  tracker_ = std::make_unique<Tracker>(max_match_distance, max_match_yaw_diff);
  tracker_->tracking_thres = this->declare_parameter("tracker.tracking_thres", 5);
  lost_time_thres_ = this->declare_parameter("tracker.lost_time_thres", 0.3);

  // outpost EKF
  // state: xc, yc, zc, yaw, v_yaw
  // measurement: xa, ya, za, yaw

  auto f = [this](const Eigen::VectorXd & x) {
      Eigen::VectorXd x_new = x;
      x_new(3) += x(4) * dt_; // yaw += v_yaw * dt
      // if (x_new(3) > M_PI) {
      //   x_new(3) -= 2 * M_PI;
      // } else if (x_new(3) < -M_PI) {
      //   x_new(3) += 2 * M_PI;
      // }
      return x_new;
    };
  // J_f - Jacobian of process function
  auto j_f = [this](const Eigen::VectorXd &) {
      Eigen::MatrixXd f(5, 5);
      // clang-format off
      f << 1, 0, 0, 0, 0,
        0, 1, 0, 0, 0,
        0, 0, 1, 0, 0,
        0, 0, 0, 1, dt_,
        0, 0, 0, 0, 1;
      // clang-format on
      return f;
    };
  // h - Observation function
  auto h = [](const Eigen::VectorXd & x) {
      Eigen::VectorXd z(4);
      double xc = x(0), yc = x(1), yaw = x(3), r = 0.2765;
      z(0) = xc - r * cos(yaw); // xa
      z(1) = yc - r * sin(yaw); // ya
      z(2) = x(2);             // za
      z(3) = x(3);             // yaw
      return z;
    };
  // J_h - Jacobian of observation function
  auto j_h = [](const Eigen::VectorXd & x) {
      Eigen::MatrixXd h(4, 5);
      double yaw = x(3), r = 0.2765;
      // clang-format off
      //    xc   v_xc yc   v_yc za   v_za yaw         v_yaw r
      h << 1, 0, 0, r * sin(yaw), 0,
        0, 1, 0, -r * cos(yaw), 0,
        0, 0, 1, 0, 0,
        0, 0, 0, 1, 0;
      // clang-format on
      return h;
    };
  // update_Q - process noise covariance matrix
  s2qxyz_ = declare_parameter("ekf.sigma2_q_xyz", 20.0);
  s2qyaw_ = declare_parameter("ekf.sigma2_q_yaw", 100.0);
  s2qvyaw_ = declare_parameter("ekf.sigma2_q_v_yaw", 800.0);
  auto u_q = [this]() {
      Eigen::MatrixXd q(5, 5);

      double t = dt_, x = s2qxyz_, y = s2qyaw_, z = s2qvyaw_;
      double q_x_x = pow(t, 4) / 4 * x;
      double q_y_y = pow(t, 4) / 4 * y;
      double q_vy_vy = pow(t, 2) * z;
      q << q_x_x, 0, 0, 0, 0,
        0, q_x_x, 0, 0, 0,
        0, 0, q_x_x, 0, 0,
        0, 0, 0, q_y_y, 0,
        0, 0, 0, 0, q_vy_vy;
      // clang-format on
      return q;
    };
  // update_R - measurement noise covariance matrix
  r_xyz_factor = declare_parameter("ekf.r_xyz_factor", 0.05);
  r_yaw = declare_parameter("ekf.r_yaw", 0.02);
  auto u_r = [this](const Eigen::VectorXd & z) {
      Eigen::DiagonalMatrix<double, 4> r; // 创建对角矩阵
      double x = r_xyz_factor;
      r.diagonal() << abs(x * z[0]), abs(x * z[1]), abs(x * z[2]), r_yaw;
      return r;
    };
  // P - error estimate covariance matrix
  Eigen::DiagonalMatrix<double, 5> p0;
  p0.setIdentity();
  tracker_->ekf = ExtendedKalmanFilter{f, h, j_f, j_h, u_q, u_r, p0};

  // Subscriber with tf2 message_filter
  // tf2 relevant
  tf2_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  // Create the timer interface before call to waitForTransform,
  // to avoid a tf2_ros::CreateTimerInterfaceException exception
  auto timer_interface = std::make_shared<tf2_ros::CreateTimerROS>(
    this->get_node_base_interface(), this->get_node_timers_interface());
  tf2_buffer_->setCreateTimerInterface(timer_interface);
  tf2_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf2_buffer_);
  // subscriber and filter
  armors_sub_.subscribe(this, "/detector/armors", rmw_qos_profile_sensor_data);
  target_frame_ = this->declare_parameter("target_frame", "odom");
  tf2_filter_ = std::make_shared<tf2_filter>(
    armors_sub_, *tf2_buffer_, target_frame_, 10, this->get_node_logging_interface(),
    this->get_node_clock_interface(), std::chrono::duration<int>(1));
  // Register a callback with tf2_ros::MessageFilter to be called when transforms are available
  tf2_filter_->registerCallback(&ArmorTrackerNode::armorsCallback, this);

  // Measurement publisher (for debug usage)
  info_pub_ = this->create_publisher<auto_aim_interfaces::msg::TrackerInfo>("/tracker/info", 10);

  // // Debug publisher
  debug_msg_.tracker_state = "Empty";
  debug_msg_.detect_pos.x = 0;
  debug_msg_.detect_pos.y = 0;
  debug_msg_.detect_pos.z = 0;
  debug_msg_.detect_yaw = 0;
  debug_msg_.detect_v_yaw = 0;
  debug_msg_.predict_pos.x = 0;
  debug_msg_.predict_pos.y = 0;
  debug_msg_.predict_pos.z = 0;
  debug_msg_.predict_yaw = 0;
  debug_msg_.predict_v_yaw = 0;
  debug_pub_ = this->create_publisher<auto_aim_interfaces::msg::DebugTrackerInfo>(
    "/tracker/debug", 10);

  // Publisher
  target_pub_ = this->create_publisher<auto_aim_interfaces::msg::TargetOutpost>(
    "/tracker/target", rclcpp::SensorDataQoS());

  // Visualization Marker Publisher
  // See http://wiki.ros.org/rviz/DisplayTypes/Marker
  position_marker_.ns = "position";
  position_marker_.type = visualization_msgs::msg::Marker::SPHERE;
  position_marker_.scale.x = position_marker_.scale.y = position_marker_.scale.z = 0.1;
  position_marker_.color.a = 1.0;
  position_marker_.color.g = 1.0;
  angular_v_marker_.type = visualization_msgs::msg::Marker::ARROW;
  angular_v_marker_.ns = "angular_v";
  angular_v_marker_.scale.x = 0.03;
  angular_v_marker_.scale.y = 0.05;
  angular_v_marker_.color.a = 1.0;
  angular_v_marker_.color.b = 1.0;
  angular_v_marker_.color.g = 1.0;
  armor_marker_.ns = "armors";
  armor_marker_.type = visualization_msgs::msg::Marker::CUBE;
  armor_marker_.scale.x = 0.03;
  armor_marker_.scale.z = 0.125;
  armor_marker_.color.a = 1.0;
  armor_marker_.color.r = 1.0;
  trajectory_marker_.ns = "trajectory";
  trajectory_marker_.type = visualization_msgs::msg::Marker::LINE_STRIP;
  trajectory_marker_.scale.x = 0.01;
  trajectory_marker_.color.a = 1.0;
  trajectory_marker_.color.r = 1.0;
  marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/tracker/marker", 10);

  bullet_marker_.ns = "bullet";
  bullet_marker_.type = visualization_msgs::msg::Marker::SPHERE;
  bullet_marker_.scale.x = bullet_marker_.scale.y = bullet_marker_.scale.z = 0.1;
  bullet_marker_.color.a = 1.0;
  bullet_marker_.color.r = 1.0;
  bullet_marker_.color.g = 1.0;
  bullet_marker_.color.b = 1.0;
  trajectory_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
    "/tracker/trajectory", 10);

  //trajectory slover param
  int max_iter = this->declare_parameter("trajectory.max_iter", 10);
  double stop_error = this->declare_parameter("trajectory.stop_error", 0.001);
  time_step = this->declare_parameter("trajectory.time_step", 0.01);
  double init_speed = this->declare_parameter("trajectory.init_bullet_speed", 26.1);
  bool is_hero = this->declare_parameter("trajectory.is_hero", false);
  static_offset_yaw_ = this->declare_parameter("trajectory.static_offset.yaw", 0.0);
  static_offset_pitch_ = this->declare_parameter("trajectory.static_offset.pitch", 0.0);

  //Get fire angle thres
  yaw_angle_thres = this->declare_parameter("yaw_angle_thres", 5.0);
  fire_permit_thres = this->declare_parameter("fire_permit_thres", 1.5);
  fire_latency = this->declare_parameter("fire_latency", 0.02);

  trajectory_slover_ =
    std::make_shared<TrajectorySlover>(
    max_iter, stop_error, time_step, init_speed,
    max_armor_distance_);

  bullet_speed_sub_ = this->create_subscription<std_msgs::msg::Float64>(
    "/bullet_speed", 10,
    std::bind(&ArmorTrackerNode::setBulletSpeed, this, std::placeholders::_1));

  latency_sub_ = this->create_subscription<std_msgs::msg::Float64>(
    "/latency", 10, std::bind(&ArmorTrackerNode::setLatancy, this, std::placeholders::_1));
}

void ArmorTrackerNode::setBulletSpeed(const std_msgs::msg::Float64::SharedPtr bulletspeed)
{
  if (bulletspeed->data != 0) {
    auto diff = bulletspeed->data - trajectory_slover_->getBulletSpeed();
    if (diff > 0.2 || diff < -0.2) {
      trajectory_slover_->setBulletSpeed(bulletspeed->data);
      RCLCPP_INFO(
        this->get_logger(), "set bullet speed: %.3f", trajectory_slover_->getBulletSpeed());
    }
  }
}

void ArmorTrackerNode::setLatancy(const std_msgs::msg::Float64::SharedPtr latency)
{
  if (latency->data >= 0) {
    latency_ = latency->data;
  }
}

void ArmorTrackerNode::armorsCallback(const auto_aim_interfaces::msg::Armors::SharedPtr armors_msg)
{
  debug_msg_.tracker_state = "armor_callback";
  debug_msg_.detect_pos.x = 0;
  debug_msg_.detect_pos.y = 0;
  debug_msg_.detect_pos.z = 0;
  debug_msg_.detect_yaw = 0;
  debug_msg_.detect_v_yaw = 0;
  debug_msg_.predict_pos.x = 0;
  debug_msg_.predict_pos.y = 0;
  debug_msg_.predict_pos.z = 0;
  debug_msg_.predict_yaw = 0;
  debug_msg_.predict_v_yaw = 0;
  debug_msg_.armors_number = 0;
  // Print armors
  if (tracker_debug_) {
    RCLCPP_INFO(
      this->get_logger(), "Armor number: %d",
      static_cast<int>(armors_msg->armors.size()));
    for (auto & armor : armors_msg->armors) {
      RCLCPP_INFO(
        this->get_logger(), "armor x: %f, y: %f, z: %f", armor.pose.position.x,
        armor.pose.position.y, armor.pose.position.z);
    }
  }
  // Tranform armor position from image frame to world coordinate
  for (auto & armor : armors_msg->armors) {
    geometry_msgs::msg::PoseStamped ps;
    ps.header = armors_msg->header;
    ps.pose = armor.pose;
    try {
      armor.pose = tf2_buffer_->transform(ps, target_frame_).pose;
    } catch (const tf2::ExtrapolationException & ex) {
      RCLCPP_ERROR(get_logger(), "Error while transforming %s", ex.what());
      return;
    }
  }

  // Filter abnormal armors
  armors_msg->armors.erase(
    std::remove_if(
      armors_msg->armors.begin(), armors_msg->armors.end(),
      [this](const auto_aim_interfaces::msg::Armor & armor) {
        return abs(armor.pose.position.z) > 1.2 ||
        Eigen::Vector2d(armor.pose.position.x, armor.pose.position.y).norm() >
        max_armor_distance_;
      }),
    armors_msg->armors.end());

  debug_msg_.armors_number = armors_msg->armors.size();

  //Get gimbal status
  //Tranform gimbal state from world coordinate
  geometry_msgs::msg::TransformStamped transform;
  try {
    transform = tf2_buffer_->lookupTransform("pitch_link", target_frame_, tf2::TimePointZero);
    auto orientation_gimbal = transform.transform.rotation;
    tf2::Quaternion q_;
    tf2::fromMsg(orientation_gimbal, q_);
    gimbal_yaw = -tf2::impl::getYaw(q_);
    //RCLCPP_INFO(get_logger(), "gimbal_yaw : %lf", gimbal_yaw);
  } catch (const tf2::ExtrapolationException & ex) {
    RCLCPP_ERROR(get_logger(), "Error while transforming %s", ex.what());
    return;
  }

  // Init message
  auto_aim_interfaces::msg::TrackerInfo info_msg;
  auto_aim_interfaces::msg::TargetOutpost target_msg; // output message
  rclcpp::Time time = armors_msg->header.stamp;
  target_msg.header.stamp = time;
  target_msg.header.frame_id = target_frame_;

  // Update tracker
  if (tracker_->tracker_state == Tracker::LOST) {
    tracker_->init(armors_msg);
    target_msg.tracking = false;
    debug_msg_.tracker_state = "lost";
    debug_msg_.tracker_state_num = 0;
  } else {
    dt_ = (time - last_time_).seconds();
    tracker_->lost_thres = static_cast<int>(lost_time_thres_ / dt_);
    tracker_->update(armors_msg);

    // Publish Info
    info_msg.position_diff = tracker_->info_position_diff;
    info_msg.yaw_diff = tracker_->info_yaw_diff;
    info_msg.position.x = tracker_->measurement(0);
    info_msg.position.y = tracker_->measurement(1);
    info_msg.position.z = tracker_->measurement(2);
    info_msg.yaw = tracker_->measurement(3);
    info_pub_->publish(info_msg);

    if (tracker_->tracker_state == Tracker::DETECTING) {
      target_msg.tracking = false;
      debug_msg_.tracker_state = "detecting";
      debug_msg_.tracker_state_num = 1;
    } else if (
      tracker_->tracker_state == Tracker::TRACKING ||
      tracker_->tracker_state == Tracker::TEMP_LOST)
    {
      target_msg.tracking = true;
      // Fill target message
      const auto & state = tracker_->target_state;
      target_msg.id = tracker_->tracked_id;
      target_msg.position.x = state(0);
      target_msg.position.y = state(1);
      target_msg.position.z = state(2);
      target_msg.yaw = state(3);
      target_msg.v_yaw = state(4);

      // Fill debug message
      if (tracker_->tracker_state == Tracker::TRACKING) {
        debug_msg_.tracker_state = "tracking";
        debug_msg_.tracker_state_num = 4;
      } else {
        debug_msg_.tracker_state = "temp_lost";
        debug_msg_.tracker_state_num = 3;
      }
      debug_msg_.predict_pos.x = state(0);
      debug_msg_.predict_pos.y = state(1);
      debug_msg_.predict_pos.z = state(2);
      debug_msg_.predict_yaw = state(3);
      debug_msg_.predict_v_yaw = state(4);
    }
  }
  debug_msg_.detect_pos.x = tracker_->measurement(0);
  debug_msg_.detect_pos.y = tracker_->measurement(1);
  debug_msg_.detect_pos.z = tracker_->measurement(2);
  debug_msg_.detect_yaw = tracker_->measurement(3);

  last_time_ = time;

  if (target_msg.tracking == true) {

    Eigen::Vector3d now_outpost_pos = Eigen::Vector3d(
      target_msg.position.x,
      target_msg.position.y,
      target_msg.position.z);
    auto outpost_center_diff = calcYawAndPitch(now_outpost_pos);

    Eigen::Vector3d armor_target = Eigen::Vector3d(
      now_outpost_pos[0] - outpost_radius_ * cos(outpost_center_diff[0]),
      now_outpost_pos[1] - outpost_radius_ * sin(outpost_center_diff[0]),
      now_outpost_pos[2]);

    geometry_msgs::msg::Vector3Stamped point_target, armor_target_tf;
    point_target.vector.x = armor_target(0);
    point_target.vector.y = armor_target(1);
    point_target.vector.z = armor_target(2);
    tf2::doTransform(point_target, armor_target_tf, transform);
    Eigen::Vector3d armor_target_pitch_link;
    armor_target_pitch_link(0) = armor_target_tf.vector.x;
    armor_target_pitch_link(1) = armor_target_tf.vector.y;
    armor_target_pitch_link(2) = armor_target_tf.vector.z;

    Eigen::Vector2d angel_diff = calcYawAndPitch(armor_target_pitch_link);
    auto trajectory_pitch = (-trajectory_slover_->solvePitch(armor_target));
    // RCLCPP_INFO(this->get_logger(), "armor_target: %f, %f, %f", armor_target(0), armor_target(1), armor_target(2));
    auto trajectory_view = trajectory_slover_->getTrajectoryWorld();

    target_msg.offset_yaw = rad2deg((double)angel_diff(0)) + static_offset_yaw_;
    target_msg.offset_pitch = rad2deg((double)angel_diff(1)) + trajectory_pitch +
      static_offset_pitch_;

    double outpost_yaw = target_msg.yaw;
    double outpost_w = target_msg.v_yaw;
    double flight_time = trajectory_slover_->getFlightTime();
    // double pred_dt =
    //   fire_latency + latency_ / 1000 + (now_outpost_pos.norm() - outpost_radius_) /
    //   trajectory_slover_->getBulletSpeed();
    double pred_dt = fire_latency + latency_ / 1000 + flight_time;
    double pred_yaw = outpost_yaw + outpost_w * pred_dt;
    if (pred_yaw > M_PI) {
      pred_yaw -= 2 * M_PI;
    } else if (pred_yaw < -M_PI) {
      pred_yaw += 2 * M_PI;
    }

    int8_t fire_permit = 0;
    double permit_yaw_angle = atan(0.135 / 2.0 / outpost_radius_);
    // RCLCPP_INFO(this->get_logger(), "permit_yaw_angle: %f", permit_yaw_angle / M_PI * 180);
    double permit_scale = 0.4;

    double armor_1_yaw = pred_yaw - outpost_center_diff[0];
    double armor_2_yaw = pred_yaw - outpost_center_diff[0] - 2 * M_PI / 3;
    if (armor_2_yaw > M_PI) {
      armor_2_yaw -= 2 * M_PI;
    } else if (armor_2_yaw < -M_PI) {
      armor_2_yaw += 2 * M_PI;
    }
    double armor_3_yaw = pred_yaw - outpost_center_diff[0] - 4 * M_PI / 3;
    if (armor_3_yaw > M_PI) {
      armor_3_yaw -= 2 * M_PI;
    } else if (armor_3_yaw < -M_PI) {
      armor_3_yaw += 2 * M_PI;
    }

    // RCLCPP_INFO(this->get_logger(), "armor_1_yaw: %f", armor_1_yaw);
    // RCLCPP_INFO(this->get_logger(), "armor_2_yaw: %f", armor_2_yaw);
    // RCLCPP_INFO(this->get_logger(), "armor_3_yaw: %f", armor_3_yaw);
    // double min_yaw = std::min(std::min(abs(armor_1_yaw), abs(armor_2_yaw)), abs(armor_3_yaw));
    // RCLCPP_INFO(this->get_logger(), "min_yaw: %f", min_yaw / M_PI * 180);

    fire_permit = (abs(armor_1_yaw) < permit_yaw_angle * permit_scale) ||
      (abs(armor_2_yaw) < permit_yaw_angle * permit_scale) ||
      (abs(armor_3_yaw) < permit_yaw_angle * permit_scale);

    if (fire_permit == 1) {
      // RCLCPP_INFO(this->get_logger(), "Fire permit!");
      if (is_fire_ == false) {
        // RCLCPP_INFO(this->get_logger(), "Fire!");
        is_fire_ = true;
        fire_time_ = time.seconds();
        now_trajectory_world_ = trajectory_slover_->getTrajectoryWorld();
      } else {
        // RCLCPP_INFO(this->get_logger(), "time - fire_time_: %f", time.seconds() - fire_time_);
        // RCLCPP_INFO(this->get_logger(), "flight_time: %f", flight_time);
        if (time.seconds() - fire_time_ > flight_time + 1) { // 1s is a safe time
          is_fire_ = false;
        }
      }
    }

    target_msg.fire_permit = fire_permit;

    if (!(isnan(target_msg.offset_yaw) || isnan(target_msg.offset_pitch))) {
      target_pub_->publish(target_msg);
    }

    publishMarkers(target_msg, trajectory_view);
    publishTrajectory(
      target_msg, trajectory_view,
      time.seconds() - fire_time_ - fire_latency - latency_ / 1000);
  }

  debug_pub_->publish(debug_msg_);
}

void ArmorTrackerNode::publishMarkers(
  const auto_aim_interfaces::msg::TargetOutpost & target_msg,
  const std::vector<Eigen::Vector3d> & trajectory_msg)
{
  position_marker_.header = target_msg.header;
  angular_v_marker_.header = target_msg.header;
  armor_marker_.header = target_msg.header;
  trajectory_marker_.header = target_msg.header;

  visualization_msgs::msg::MarkerArray marker_array;

  if (target_msg.tracking) {
    // Center of outpost
    position_marker_.action = visualization_msgs::msg::Marker::ADD;
    position_marker_.pose.position.x = target_msg.position.x;
    position_marker_.pose.position.y = target_msg.position.y;
    position_marker_.pose.position.z = target_msg.position.z;

    // Angular velocity arrow
    angular_v_marker_.action = visualization_msgs::msg::Marker::ADD;

    geometry_msgs::msg::Point arrow_end = position_marker_.pose.position;
    arrow_end.z += target_msg.v_yaw / M_PI;
    angular_v_marker_.points.clear();
    angular_v_marker_.points.emplace_back(position_marker_.pose.position);
    angular_v_marker_.points.emplace_back(arrow_end);

    // Armor visualization
    armor_marker_.action = visualization_msgs::msg::Marker::ADD;
    armor_marker_.scale.y = 0.135;

    for (int i = 0; i < 3; i++) {
      armor_marker_.id = i;
      double tmp_yaw = target_msg.yaw + i * (2 * M_PI / 3);
      armor_marker_.pose.position.x = target_msg.position.x - outpost_radius_ * cos(tmp_yaw);
      armor_marker_.pose.position.y = target_msg.position.y - outpost_radius_ * sin(tmp_yaw);
      armor_marker_.pose.position.z = target_msg.position.z;
      tf2::Quaternion q;
      q.setRPY(0, -0.26, tmp_yaw);
      armor_marker_.pose.orientation = tf2::toMsg(q);
      marker_array.markers.emplace_back(armor_marker_);
    }

    //trajectory visualization
    trajectory_marker_.action = visualization_msgs::msg::Marker::ADD;
    trajectory_marker_.points.clear();
    trajectory_marker_.points.reserve(trajectory_msg.size());
    for (const auto & point : trajectory_msg) {
      geometry_msgs::msg::Point p;
      p.x = point[0];
      p.y = point[1];
      p.z = point[2];
      trajectory_marker_.points.emplace_back(p);
    }
    marker_array.markers.emplace_back(trajectory_marker_);

  } else {
    position_marker_.action = visualization_msgs::msg::Marker::DELETE;
    angular_v_marker_.action = visualization_msgs::msg::Marker::DELETE;
    trajectory_marker_.action = visualization_msgs::msg::Marker::DELETE;
    armor_marker_.action = visualization_msgs::msg::Marker::DELETE;

    marker_array.markers.emplace_back(armor_marker_);
    marker_array.markers.emplace_back(trajectory_marker_);
  }

  marker_array.markers.emplace_back(position_marker_);
  marker_array.markers.emplace_back(angular_v_marker_);

  marker_pub_->publish(marker_array);
}

void ArmorTrackerNode::publishTrajectory(
  const auto_aim_interfaces::msg::TargetOutpost & target_msg,
  const std::vector<Eigen::Vector3d> & trajectory_msg,
  double now_time)
{
  int index = static_cast<int>(now_time / time_step);
  if (index < 0) {return;}

  visualization_msgs::msg::MarkerArray marker_array;

  if (is_fire_) {
    if (index >= static_cast<int>(trajectory_msg.size())) {
      is_fire_ = false;
      return;
    }
    bullet_marker_.header = target_msg.header;
    bullet_marker_.action = visualization_msgs::msg::Marker::ADD;
    bullet_marker_.pose.position.x = trajectory_msg[index][0];
    bullet_marker_.pose.position.y = trajectory_msg[index][1];
    bullet_marker_.pose.position.z = trajectory_msg[index][2];

    marker_array.markers.emplace_back(bullet_marker_);
  } else {
    bullet_marker_.action = visualization_msgs::msg::Marker::DELETE;
    marker_array.markers.emplace_back(bullet_marker_);
  }
  trajectory_pub_->publish(marker_array);
}

double ArmorTrackerNode::orientationToYaw(const geometry_msgs::msg::Quaternion & q)
{
  // Get armor yaw
  tf2::Quaternion tf_q;
  tf2::fromMsg(q, tf_q);
  double roll, pitch, yaw;
  tf2::Matrix3x3(tf_q).getRPY(roll, pitch, yaw);
  // Make yaw change continuous (-pi~pi to -inf~inf)
  yaw = last_yaw_ + angles::shortest_angular_distance(last_yaw_, yaw);
  last_yaw_ = yaw;
  return yaw;
}

}  // namespace outpost_auto_aim

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(outpost_auto_aim::ArmorTrackerNode)
