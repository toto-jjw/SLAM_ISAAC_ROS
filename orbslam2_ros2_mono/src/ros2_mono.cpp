#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <tf2_ros/transform_broadcaster.h>

// NEW: PointCloud2
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

#include <opencv2/opencv.hpp>
#include <Eigen/Core>
#include <Eigen/Geometry>

#include <string>
#include <memory>
#include <stdexcept>
#include <vector>
#include <cmath>
#include <iostream>

// ORB-SLAM2
#include "System.h"
#include "MapPoint.h"  // for publishing landmarks

// ===== helpers =====
static int cvTypeFromEncoding(const std::string& enc, int& channels) {
  if (enc == "mono8" || enc == "8UC1") { channels = 1; return CV_8UC1; }
  if (enc == "bgr8"  || enc == "rgb8") { channels = 3; return CV_8UC3; }
  if (enc == "bgra8" || enc == "rgba8"){ channels = 4; return CV_8UC4; }
  throw std::runtime_error("Unsupported encoding: " + enc);
}

static cv::Mat imageMsgToGray(const sensor_msgs::msg::Image& msg) {
  int ch = 0;
  const std::string enc = msg.encoding;
  const int cvtype = cvTypeFromEncoding(enc, ch);

  cv::Mat view(msg.height, msg.width, cvtype,
               const_cast<uint8_t*>(msg.data.data()), msg.step);
  cv::Mat img = view.clone();

  cv::Mat gray;
  if (ch == 1) {
    gray = img;
  } else if (enc == "rgb8") {
    cv::cvtColor(img, gray, cv::COLOR_RGB2GRAY);
  } else if (enc == "bgr8") {
    cv::cvtColor(img, gray, cv::COLOR_BGR2GRAY);
  } else if (enc == "rgba8") {
    cv::cvtColor(img, gray, cv::COLOR_RGBA2GRAY);
  } else if (enc == "bgra8") {
    cv::cvtColor(img, gray, cv::COLOR_BGRA2GRAY);
  } else {
    throw std::runtime_error("Unsupported color encoding: " + enc);
  }
  return gray;
}

// ----------------------- Node -----------------------
class OrbSlam2MonoNode : public rclcpp::Node {
public:
  explicit OrbSlam2MonoNode(ORB_SLAM2::System* slam)
  : rclcpp::Node("orbslam2_mono"), slam_(slam) {
    image_topic_   = declare_parameter<std::string>("image_topic", "/camera/color/image_raw");
    world_frame_   = declare_parameter<std::string>("world_frame", "map");
    child_frame_   = declare_parameter<std::string>("child_frame", "camera_link");
    max_path_len_  = declare_parameter<int>("max_path_length", 10000);

    pose_pub_      = create_publisher<geometry_msgs::msg::PoseStamped>("orb_slam2/pose", 10);
    path_pub_      = create_publisher<nav_msgs::msg::Path>("orb_slam2/trajectory", 10);
    odom_pub_      = create_publisher<nav_msgs::msg::Odometry>("orb_slam2/odometry", 10);
    status_pub_    = create_publisher<diagnostic_msgs::msg::DiagnosticArray>("orb_slam2/status", 10);

    // CHANGED: landmarks PointCloud2 + markers (debug)
    landmarks_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>("orb_slam2/landmarks", 1);
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    path_msg_.header.frame_id = world_frame_;

    using std::placeholders::_1;
    sub_ = create_subscription<sensor_msgs::msg::Image>(
      image_topic_, rclcpp::SensorDataQoS(),
      std::bind(&OrbSlam2MonoNode::imageCallback, this, _1));
  }

private:
  // 좌표계 변환 행렬 (optical -> ROS base(ENU))
  // optical: +x right, +y down, +z forward
  // ROS ENU(base): +x forward, +y left, +z up
  static Eigen::Matrix3f R_opt2base() {
    Eigen::Matrix3f R;
    R <<  0,  0,  1,
         -1,  0,  0,
          0, -1,  0;
    return R;
  }

  void publishTFAndPath(const geometry_msgs::msg::PoseStamped& ps) {
    geometry_msgs::msg::TransformStamped tf_msg;
    tf_msg.header = ps.header;
    tf_msg.header.frame_id = world_frame_;
    tf_msg.child_frame_id  = child_frame_;
    tf_msg.transform.translation.x = ps.pose.position.x;
    tf_msg.transform.translation.y = ps.pose.position.y;
    tf_msg.transform.translation.z = ps.pose.position.z;
    tf_msg.transform.rotation      = ps.pose.orientation;
    tf_broadcaster_->sendTransform(tf_msg);

    path_msg_.header.frame_id = world_frame_;
    path_msg_.header.stamp    = ps.header.stamp;
    path_msg_.poses.push_back(ps);
    if (max_path_len_ > 0 && static_cast<int>(path_msg_.poses.size()) > max_path_len_) {
      path_msg_.poses.erase(path_msg_.poses.begin(),
                            path_msg_.poses.begin() + (path_msg_.poses.size() - max_path_len_));
    }
    path_pub_->publish(path_msg_);
  }

  void publishOdometry(const geometry_msgs::msg::PoseStamped& ps,
                       const Eigen::Vector3f& twb,
                       const Eigen::Quaternionf& qwb)
  {
    rclcpp::Time curr(ps.header.stamp);
    double dt = 0.0;
    if (have_last_) {
      dt = (curr - last_stamp_).seconds();
    }

    nav_msgs::msg::Odometry odom;
    odom.header = ps.header;
    odom.child_frame_id = child_frame_;
    odom.pose.pose = ps.pose;

    if (have_last_ && dt > 0.0) {
      // linear velocity
      Eigen::Vector3f v = (twb - last_twb_) / static_cast<float>(dt);

      // angular velocity (body frame): dq = q_last^{-1} * q_curr
      Eigen::Quaternionf dq = last_qwb_.conjugate() * qwb;
      Eigen::AngleAxisf aa(dq);
      Eigen::Vector3f omega = aa.axis() * aa.angle() / static_cast<float>(dt);

      odom.twist.twist.linear.x = v.x();
      odom.twist.twist.linear.y = v.y();
      odom.twist.twist.linear.z = v.z();
      odom.twist.twist.angular.x = omega.x();
      odom.twist.twist.angular.y = omega.y();
      odom.twist.twist.angular.z = omega.z();
    } else {
      odom.twist.twist.linear.x = odom.twist.twist.linear.y = odom.twist.twist.linear.z = 0.0;
      odom.twist.twist.angular.x = odom.twist.twist.angular.y = odom.twist.twist.angular.z = 0.0;
    }

    odom_pub_->publish(odom);

    // update last
    last_stamp_ = curr;
    last_twb_   = twb;
    last_qwb_   = qwb;
    have_last_  = true;
  }

  void publishStatusOK(size_t tracked_pts, size_t total_pts) {
    diagnostic_msgs::msg::DiagnosticStatus st;
    st.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
    st.name = "orbslam2_mono";
    st.message = "Tracking";
    st.hardware_id = "ORB-SLAM2";
    st.values.reserve(2);
    diagnostic_msgs::msg::KeyValue kv1; kv1.key = "tracked_map_points"; kv1.value = std::to_string(tracked_pts);
    diagnostic_msgs::msg::KeyValue kv2; kv2.key = "total_map_points";   kv2.value = std::to_string(total_pts);
    st.values.push_back(kv1);
    st.values.push_back(kv2);

    diagnostic_msgs::msg::DiagnosticArray arr;
    arr.header.stamp = this->get_clock()->now();
    arr.status.push_back(st);
    status_pub_->publish(arr);
  }

  void publishStatusInit() {
    diagnostic_msgs::msg::DiagnosticStatus st;
    st.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
    st.name = "orbslam2_mono";
    st.message = "Initializing";
    st.hardware_id = "ORB-SLAM2";

    diagnostic_msgs::msg::DiagnosticArray arr;
    arr.header.stamp = this->get_clock()->now();
    arr.status.push_back(st);
    status_pub_->publish(arr);
  }

  // NEW: PointCloud2 퍼블리시
  void publishLandmarksPC2(const std::vector<ORB_SLAM2::MapPoint*>& mps,
                           const Eigen::Matrix3f& Rwopt_to_ros,
                           const rclcpp::Time& stamp)
  {
    sensor_msgs::msg::PointCloud2 cloud;
    cloud.header.frame_id = world_frame_;
    cloud.header.stamp    = stamp;

    sensor_msgs::PointCloud2Modifier mod(cloud);
    mod.setPointCloud2FieldsByString(1, "xyz");

    // count valid first to resize once
    size_t valid = 0;
    for (auto* p : mps) if (p && !p->isBad()) ++valid;
    mod.resize(valid);

    sensor_msgs::PointCloud2Iterator<float> it_x(cloud, "x");
    sensor_msgs::PointCloud2Iterator<float> it_y(cloud, "y");
    sensor_msgs::PointCloud2Iterator<float> it_z(cloud, "z");

    for (auto* p : mps) {
      if (!p || p->isBad()) continue;
      const cv::Mat Xw = p->GetWorldPos(); // 3x1 (world_optical)
      if (Xw.empty() || Xw.rows != 3) continue;
      Eigen::Vector3f wopt(Xw.at<float>(0), Xw.at<float>(1), Xw.at<float>(2));
      Eigen::Vector3f wros = Rwopt_to_ros * wopt;
      *it_x = wros.x(); *it_y = wros.y(); *it_z = wros.z();
      ++it_x; ++it_y; ++it_z;
    }
    landmarks_pub_->publish(cloud);
  }


  void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg) {
    // 1) gray
    cv::Mat imGray;
    try {
      imGray = imageMsgToGray(*msg);
    } catch (const std::exception& e) {
      RCLCPP_WARN(this->get_logger(), "Image convert failed: %s", e.what());
      return;
    }

    // 2) track
    const double t = rclcpp::Time(msg->header.stamp).seconds();
    cv::Mat Tcw = slam_->TrackMonocular(imGray, t);

    if (Tcw.empty()) {
      RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                           "No pose yet (initializing)...");
      publishStatusInit();
      return;
    }

    // 3) Tcw -> (Rwc, twc)
    cv::Mat Tcw_f;
    if (Tcw.type() != CV_32F) Tcw.convertTo(Tcw_f, CV_32F);
    else Tcw_f = Tcw;

    cv::Mat Rcw = Tcw_f(cv::Rect(0,0,3,3)).clone();
    cv::Mat tcw = Tcw_f(cv::Rect(3,0,1,3)).clone();

    Eigen::Matrix3f Rcw_eig =
        Eigen::Map<Eigen::Matrix<float,3,3,Eigen::RowMajor>>(Rcw.ptr<float>());
    Eigen::Vector3f tcw_eig = Eigen::Map<Eigen::Vector3f>(tcw.ptr<float>());

    Eigen::Matrix3f Rwc = Rcw_eig.transpose();
    Eigen::Vector3f twc = -Rwc * tcw_eig;

    // 4) optical(world) -> ROS(ENU) 변환
    const Eigen::Matrix3f Ropt2base = R_opt2base();
    Eigen::Matrix3f R_wopt_b = Rwc * Ropt2base.transpose(); // cam->base in world_opt
    Eigen::Matrix3f R_wros_b = Ropt2base * R_wopt_b;        // rotate world
    Eigen::Vector3f t_wros_b = Ropt2base * twc;

    Eigen::Quaternionf q_wros_b(R_wros_b);
    q_wros_b.normalize();

    // 5) Pose
    geometry_msgs::msg::PoseStamped ps;
    ps.header = msg->header;
    ps.header.frame_id = world_frame_;
    ps.pose.position.x = t_wros_b.x();
    ps.pose.position.y = t_wros_b.y();
    ps.pose.position.z = t_wros_b.z();
    ps.pose.orientation.x = q_wros_b.x();
    ps.pose.orientation.y = q_wros_b.y();
    ps.pose.orientation.z = q_wros_b.z();
    ps.pose.orientation.w = q_wros_b.w();

    pose_pub_->publish(ps);
    publishTFAndPath(ps);
    publishOdometry(ps, t_wros_b, q_wros_b);

    // 6) 상태 + 랜드마크 퍼블리시 (PointCloud2 + debug markers)
    std::vector<ORB_SLAM2::MapPoint*> tracked = slam_->GetTrackedMapPoints();
    publishStatusOK(tracked.size(), tracked.size()); // total 미상 시 동일 보고

    rclcpp::Time stamp(ps.header.stamp);
    publishLandmarksPC2(tracked, Ropt2base, stamp);     // <-- /orb_slam2/landmarks (PointCloud2)
  }

  // members
  ORB_SLAM2::System* slam_;
  std::string image_topic_, world_frame_, child_frame_;
  int max_path_len_{10000};

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr             path_pub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr         odom_pub_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr    status_pub_;

  // CHANGED
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr            landmarks_pub_;

  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  nav_msgs::msg::Path path_msg_;

  // odom 계산용
  bool have_last_{false};
  rclcpp::Time last_stamp_{0,0,RCL_ROS_TIME};
  Eigen::Vector3f last_twb_{Eigen::Vector3f::Zero()};
  Eigen::Quaternionf last_qwb_{Eigen::Quaternionf::Identity()};
};

// ----------------------- main -----------------------
int main(int argc, char** argv) {
  rclcpp::init(argc, argv);

  if (argc < 3) {
    std::cerr << "Usage: ros2 run <pkg> ros2_mono <path_to_ORBvoc.txt> <path_to_settings.yaml>\n";
    return 1;
  }
  const std::string voc_path = argv[1];
  const std::string settings_path = argv[2];

  // Pangolin Viewer 켜기: true
  ORB_SLAM2::System SLAM(voc_path, settings_path, ORB_SLAM2::System::MONOCULAR, true);

  auto node = std::make_shared<OrbSlam2MonoNode>(&SLAM);
  rclcpp::executors::MultiThreadedExecutor exec;
  exec.add_node(node);
  exec.spin();

  SLAM.Shutdown();
  SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");

  rclcpp::shutdown();
  return 0;
}
