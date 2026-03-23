#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/path.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

#include <queue>
#include <vector>
#include <limits>
#include <algorithm>
#include <cmath>
#include <string>
#include <memory>
#include <chrono>

// ---------------------------------------------------------------------------
// Grid: thin wrapper that translates between world (m) and cell coordinates
// ---------------------------------------------------------------------------
struct Grid {
  double res{0.05}; // metres per cell
  int    w{200};
  int    h{200};
  double ox{0.0};   // world x of cell (0,0) corner
  double oy{0.0};   // world y of cell (0,0) corner

  inline bool worldToCell(double x, double y, int& i, int& j) const {
    i = static_cast<int>(std::floor((x - ox) / res));
    j = static_cast<int>(std::floor((y - oy) / res));
    return (i >= 0 && i < w && j >= 0 && j < h);
  }
  inline void cellToWorld(int i, int j, double& x, double& y) const {
    x = ox + (i + 0.5) * res;
    y = oy + (j + 0.5) * res;
  }
  inline int idx(int i, int j) const { return j * w + i; }
};

// ---------------------------------------------------------------------------
// AStarPlannerNode
//   Subscribes to /orb_slam2/pose and /orb_slam2/landmarks (PointCloud2),
//   builds a 2-D occupancy grid, plans a direction-aware A* path to a
//   configurable goal, and publishes cmd_vel for a differential-drive robot.
// ---------------------------------------------------------------------------
class AStarPlannerNode : public rclcpp::Node {
public:
  AStarPlannerNode() : Node("hybrid_astar_planner")
  {
    world_frame_   = declare_parameter<std::string>("world_frame",  "map");
    pose_topic_    = declare_parameter<std::string>("pose_topic",   "/orb_slam2/pose");
    pc_topic_      = declare_parameter<std::string>("pc_topic",     "/orb_slam2/landmarks");
    grid_.res      = declare_parameter<double>("grid_resolution",   0.05);
    inflate_r_     = declare_parameter<double>("inflate_radius",    0.01);
    half_extent_m_ = declare_parameter<double>("half_extent_m",     10.0);

    auto goal = declare_parameter<std::vector<double>>("goal", std::vector<double>{1.0, -1.0, 0.0});
    if (goal.size() >= 2) { goal_x_ = goal[0]; goal_y_ = goal[1]; }
    goal_tol_ = declare_parameter<double>("goal_tolerance", 0.03);

    publish_cmd_ = declare_parameter<bool>("publish_cmd_vel", true);
    cmd_topic_   = declare_parameter<std::string>("cmd_topic", "/cmd_vel");
    v_max_       = declare_parameter<double>("v_max", 1.5);
    w_max_       = declare_parameter<double>("w_max", 0.8);

    range_max_keep_  = declare_parameter<double>("range_max_keep",  10.0);
    ground_z_thresh_ = declare_parameter<double>("ground_z_thresh",  0.03);

    lost_timeout_sec_    = declare_parameter<double>("lost_timeout_sec",    1.0);
    recover_pause_sec_   = declare_parameter<double>("recover_pause_sec",   0.2);
    recover_forward_sec_ = declare_parameter<double>("recover_forward_sec", 2.0);
    recover_v_           = declare_parameter<double>("recover_v",           0.5);

    lookahead_cells_         = declare_parameter<int>("lookahead_cells",    3);
    advance_dist_thresh_     = declare_parameter<double>("advance_dist_thresh", 0.1);
    forward_check_n_         = declare_parameter<int>("forward_check_n",    20);
    replan_min_interval_sec_ = declare_parameter<double>("replan_min_interval_sec", 1.0);
    replan_max_interval_sec_ = declare_parameter<double>("replan_max_interval_sec", 4.0);

    turn_weight_               = declare_parameter<double>("turn_weight",               2.0);
    replace_improvement_ratio_ = declare_parameter<double>("replace_improvement_ratio", 0.2);

    occ_pub_  = create_publisher<nav_msgs::msg::OccupancyGrid>("debug/occupancy", 1);
    path_pub_ = create_publisher<nav_msgs::msg::Path>("planned_path", 1);
    if (publish_cmd_) cmd_pub_ = create_publisher<geometry_msgs::msg::Twist>(cmd_topic_, 10);

    pose_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
        pose_topic_, rclcpp::SensorDataQoS(),
        std::bind(&AStarPlannerNode::onPose, this, std::placeholders::_1));

    pc_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
        pc_topic_, rclcpp::SystemDefaultsQoS(),
        std::bind(&AStarPlannerNode::onCloud, this, std::placeholders::_1));

    timer_ = create_wall_timer(std::chrono::milliseconds(200),
              std::bind(&AStarPlannerNode::planTick, this));

    RCLCPP_INFO(get_logger(),
      "A* planner ready. goal=(%.2f, %.2f) tol=%.2f "
      "ground_z<=%.2f turn_weight=%.2f",
      goal_x_, goal_y_, goal_tol_, ground_z_thresh_, turn_weight_);
  }

private:
  // ---------------------------------------------------------------------------
  // Callbacks
  // ---------------------------------------------------------------------------
  void onPose(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
    if (msg->header.frame_id != world_frame_) return;
    cur_pose_       = *msg;
    have_pose_      = true;
    last_pose_time_ = now();
    if (recover_state_ != State::NORMAL) {
      recover_state_ = State::NORMAL;
    }
  }

  void onCloud(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
    last_pc2_ = msg;
    have_obs_ = true;
  }

  // ---------------------------------------------------------------------------
  // Helpers
  // ---------------------------------------------------------------------------
  double pathCost(const std::vector<std::pair<int,int>>& cells) const {
    if (cells.size() < 2) return 0.0;
    double sum = 0.0;
    for (size_t k = 1; k < cells.size(); ++k) {
      int di = std::abs(cells[k].first  - cells[k-1].first);
      int dj = std::abs(cells[k].second - cells[k-1].second);
      sum += (di + dj == 2 ? std::sqrt(2.0) : 1.0);
    }
    return sum;
  }

  bool upcomingBlocked(const std::vector<uint8_t>& occ,
                       const std::vector<std::pair<int,int>>& cells,
                       size_t from_idx, int N) const
  {
    if (cells.empty()) return true;
    const size_t end = std::min(from_idx + static_cast<size_t>(N), cells.size() - 1);
    for (size_t k = from_idx; k <= end; ++k) {
      int i = cells[k].first, j = cells[k].second;
      if (i < 0 || j < 0 || i >= grid_.w || j >= grid_.h) return true;
      if (occ[grid_.idx(i, j)]) return true;
    }
    return false;
  }

  // ---------------------------------------------------------------------------
  // Main planning loop (called at 5 Hz)
  // ---------------------------------------------------------------------------
  void planTick() {
    // Check goal reached
    if (have_pose_) {
      double dx = cur_pose_.pose.position.x - goal_x_;
      double dy = cur_pose_.pose.position.y - goal_y_;
      if (std::hypot(dx, dy) <= goal_tol_) {
        stopRobot();
        if (!arrived_logged_) {
          RCLCPP_INFO(get_logger(), "Goal reached! (within %.2fm)", goal_tol_);
          arrived_logged_ = true;
        }
        return;
      }
    }

    // Pose-loss recovery state machine
    const bool lost = (!have_pose_) ||
                      ((now() - last_pose_time_).seconds() > lost_timeout_sec_);
    if (lost) {
      if (recover_state_ == State::NORMAL) {
        recover_state_    = State::PAUSE;
        state_start_time_ = now();
        stopRobot();
        RCLCPP_WARN(get_logger(), "Pose lost — pausing for %.2fs", recover_pause_sec_);
        return;
      } else if (recover_state_ == State::PAUSE) {
        if ((now() - state_start_time_).seconds() >= recover_pause_sec_) {
          recover_state_    = State::FORWARD;
          state_start_time_ = now();
          RCLCPP_WARN(get_logger(), "Pose still lost — driving forward %.2fs at %.2f m/s",
                      recover_forward_sec_, recover_v_);
        } else {
          stopRobot();
          return;
        }
      } else if (recover_state_ == State::FORWARD) {
        if ((now() - state_start_time_).seconds() < recover_forward_sec_) {
          geometry_msgs::msg::Twist cmd;
          cmd.linear.x = recover_v_;
          if (publish_cmd_) cmd_pub_->publish(cmd);
          return;
        } else {
          recover_state_    = State::PAUSE;
          state_start_time_ = now();
          stopRobot();
          return;
        }
      }
    }

    if (recover_state_ != State::NORMAL) return;
    if (!have_pose_ || !have_obs_) return;

    const double cx = cur_pose_.pose.position.x;
    const double cy = cur_pose_.pose.position.y;

    // Build occupancy grid centred on current pose
    grid_.w  = static_cast<int>(std::ceil((2.0 * half_extent_m_) / grid_.res));
    grid_.h  = grid_.w;
    grid_.ox = cx - half_extent_m_;
    grid_.oy = cy - half_extent_m_;

    std::vector<uint8_t> occ(grid_.w * grid_.h, 0);
    size_t kept = 0, dropped_ground = 0, dropped_range = 0, dropped_nan = 0;
    rasterizeSimple(occ, kept, dropped_ground, dropped_range, dropped_nan);
    inflate(occ, static_cast<int>(std::ceil(inflate_r_ / grid_.res)));

    int si, sj, gi, gj;
    if (!grid_.worldToCell(cx, cy, si, sj)) return;
    if (!grid_.worldToCell(goal_x_, goal_y_, gi, gj)) return;

    publishOccGridMsg(occ);

    // Decide whether to replan
    const auto now_t            = now();
    const double since_last     = (now_t - last_plan_time_).seconds();
    bool need_plan              = false;

    if (active_cells_.empty()) {
      need_plan = true;
    } else {
      advanceAlongActivePath(cx, cy);
      if (upcomingBlocked(occ, active_cells_, active_idx_, forward_check_n_)) {
        need_plan = true;
      }
      if (!need_plan && since_last >= replan_max_interval_sec_) {
        need_plan = true;
      }
      if (need_plan && since_last < replan_min_interval_sec_) {
        need_plan = false;
      }
    }

    if (need_plan) {
      if (occ[grid_.idx(si, sj)] || occ[grid_.idx(gi, gj)]) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
          "Start or goal cell is inside an obstacle.");
        stopRobot();
        return;
      }

      std::vector<int> parents;
      if (!astar_dir(occ, si, sj, gi, gj, parents)) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
          "A* found no path to goal.");
        stopRobot();
        return;
      }

      // Reconstruct candidate path from parent array
      std::vector<std::pair<int,int>> candidate;
      {
        int cur    = grid_.idx(gi, gj);
        const int sidx = grid_.idx(si, sj);
        while (cur != sidx && cur >= 0 && cur < static_cast<int>(parents.size())) {
          candidate.emplace_back(cur % grid_.w, cur / grid_.w);
          int p = parents[cur];
          if (p == cur) break;
          cur = p;
        }
        candidate.emplace_back(si, sj);
        std::reverse(candidate.begin(), candidate.end());
      }

      // Path replacement policy
      bool replace = false;
      if (active_cells_.empty()) {
        replace = true;
      } else if (upcomingBlocked(occ, active_cells_, active_idx_, forward_check_n_)) {
        replace = true;
      } else {
        double cand_cost = pathCost(candidate);
        std::vector<std::pair<int,int>> remain(
          active_cells_.begin() + static_cast<long>(active_idx_), active_cells_.end());
        double remain_cost = pathCost(remain);
        if (cand_cost < remain_cost * (1.0 - replace_improvement_ratio_)) {
          replace = true;
        } else {
          RCLCPP_INFO(get_logger(),
            "Keeping current path (cand=%.2f, remain=%.2f, required improvement=%.0f%%).",
            cand_cost, remain_cost, replace_improvement_ratio_ * 100.0);
        }
      }

      if (replace) {
        active_cells_ = std::move(candidate);
        active_idx_   = 0;
        RCLCPP_INFO(get_logger(), "Path updated (%zu cells).", active_cells_.size());
      }

      last_plan_time_ = now_t;
    }

    if (active_cells_.empty()) return;

    // Publish planned path
    nav_msgs::msg::Path path;
    path.header.frame_id = world_frame_;
    path.header.stamp    = now();
    path.poses.reserve(active_cells_.size());
    for (auto& c : active_cells_) {
      double x, y;
      grid_.cellToWorld(c.first, c.second, x, y);
      geometry_msgs::msg::PoseStamped ps;
      ps.header              = path.header;
      ps.pose.position.x     = x;
      ps.pose.position.y     = y;
      ps.pose.orientation.w  = 1.0;
      path.poses.push_back(ps);
    }
    path_pub_->publish(path);

    // Lookahead path following
    if (publish_cmd_) {
      size_t tgt = std::min(active_idx_ + static_cast<size_t>(lookahead_cells_),
                            active_cells_.size() - 1);
      double tx, ty;
      grid_.cellToWorld(active_cells_[tgt].first, active_cells_[tgt].second, tx, ty);
      const double dx       = tx - cur_pose_.pose.position.x;
      const double dy       = ty - cur_pose_.pose.position.y;
      const double dist     = std::hypot(dx, dy);
      const double tgt_yaw  = std::atan2(dy, dx);
      const double cur_yaw  = yawFromQuat(cur_pose_.pose.orientation);
      const double err_yaw  = normAngle(tgt_yaw - cur_yaw);

      geometry_msgs::msg::Twist cmd;
      cmd.linear.x  = std::clamp(dist,        0.0, v_max_);
      cmd.angular.z = std::clamp(1.5 * err_yaw, -w_max_, w_max_);
      cmd_pub_->publish(cmd);
    }

    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000,
      "Obstacles: kept=%zu ground=%zu range=%zu nan=%zu | path=%zu idx=%zu",
      kept, dropped_ground, dropped_range, dropped_nan,
      active_cells_.size(), active_idx_);
  }

  void advanceAlongActivePath(double cx, double cy) {
    while (active_idx_ < active_cells_.size()) {
      double tx, ty;
      grid_.cellToWorld(active_cells_[active_idx_].first,
                        active_cells_[active_idx_].second, tx, ty);
      if (std::hypot(tx - cx, ty - cy) <= advance_dist_thresh_) {
        if (active_idx_ + 1 < active_cells_.size()) ++active_idx_;
        else break;
      } else {
        break;
      }
    }
  }

  // ---------------------------------------------------------------------------
  // Obstacle rasterisation: PointCloud2 → occupancy grid
  // ---------------------------------------------------------------------------
  void rasterizeSimple(std::vector<uint8_t>& occ,
                       size_t& kept, size_t& drop_ground,
                       size_t& drop_range, size_t& drop_nan)
  {
    kept = drop_ground = drop_range = drop_nan = 0;
    if (!last_pc2_) return;

    try {
      sensor_msgs::PointCloud2ConstIterator<float> it_x(*last_pc2_, "x");
      sensor_msgs::PointCloud2ConstIterator<float> it_y(*last_pc2_, "y");
      sensor_msgs::PointCloud2ConstIterator<float> it_z(*last_pc2_, "z");

      for (; it_x != it_x.end(); ++it_x, ++it_y, ++it_z) {
        const float x = *it_x, y = *it_y, z = *it_z;
        if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) {
          ++drop_nan; continue;
        }
        if (z <= ground_z_thresh_) { ++drop_ground; continue; }

        const double dx = x - cur_pose_.pose.position.x;
        const double dy = y - cur_pose_.pose.position.y;
        if (std::hypot(dx, dy) > range_max_keep_) { ++drop_range; continue; }

        int i, j;
        if (grid_.worldToCell(x, y, i, j)) {
          occ[grid_.idx(i, j)] = 100;
          ++kept;
        }
      }
    } catch (const std::exception& e) {
      RCLCPP_WARN_ONCE(get_logger(),
        "PointCloud2 field access failed: %s. Check cloud format.", e.what());
    }
  }

  // Square-kernel morphological dilation
  void inflate(std::vector<uint8_t>& occ, int r) {
    if (r <= 0) return;
    std::vector<uint8_t> copy = occ;
    for (int j = 0; j < grid_.h; ++j) {
      for (int i = 0; i < grid_.w; ++i) {
        if (copy[grid_.idx(i, j)] == 0) continue;
        for (int dj = -r; dj <= r; ++dj) {
          for (int di = -r; di <= r; ++di) {
            int ni = i + di, nj = j + dj;
            if (ni < 0 || nj < 0 || ni >= grid_.w || nj >= grid_.h) continue;
            occ[grid_.idx(ni, nj)] = 100;
          }
        }
      }
    }
  }

  // ---------------------------------------------------------------------------
  // Direction-aware A*
  //   State space: (cell_i, cell_j, heading_direction[0..7])
  //   Heading:  E=0, NE=1, N=2, NW=3, W=4, SW=5, S=6, SE=7
  //   Turn penalty discourages sharp heading changes.
  // ---------------------------------------------------------------------------
  bool astar_dir(const std::vector<uint8_t>& occ,
                 int si, int sj, int gi, int gj,
                 std::vector<int>& parents)
  {
    const int W = grid_.w, H = grid_.h, D = 8;
    auto idx2d  = [&](int i, int j) { return j * W + i; };
    auto sidx3  = [&](int i, int j, int d) { return idx2d(i, j) * D + d; };
    const int gidx = idx2d(gi, gj);

    const int    di[D]        = { 1,  1,  0, -1, -1, -1,  0,  1 };
    const int    dj[D]        = { 0,  1,  1,  1,  0, -1, -1, -1 };
    const double step_cost[D] = { 1, std::sqrt(2), 1, std::sqrt(2),
                                   1, std::sqrt(2), 1, std::sqrt(2) };
    auto h = [&](int i, int j) { return std::hypot(i - gi, j - gj); };

    const int Nstate = W * H * D;
    std::vector<double> g(Nstate, std::numeric_limits<double>::infinity());
    std::vector<int>    parent_state(Nstate, -1);

    auto fcmp = [&](const std::pair<double,int>& a, const std::pair<double,int>& b) {
      return a.first > b.first;
    };
    std::priority_queue<std::pair<double,int>,
                        std::vector<std::pair<double,int>>,
                        decltype(fcmp)> open(fcmp);

    // Seed start with all headings at equal cost
    for (int d = 0; d < D; ++d) {
      int s = sidx3(si, sj, d);
      g[s]            = 0.0;
      parent_state[s] = s;
      open.emplace(h(si, sj), s);
    }

    int goal_state = -1;
    while (!open.empty()) {
      auto [fcost, u] = open.top(); open.pop();
      int cell = u / D;
      int dcur = u % D;
      int ui = cell % W, uj = cell / W;

      if (cell == gidx) { goal_state = u; break; }

      for (int k = 0; k < D; ++k) {
        int vi = ui + di[k], vj = uj + dj[k];
        if (vi < 0 || vj < 0 || vi >= W || vj >= H) continue;
        int vcell = idx2d(vi, vj);
        if (occ[vcell]) continue;

        int dd        = std::abs(k - dcur);
        int turn_diff = std::min(dd, D - dd);
        double turn_pen = turn_weight_ * static_cast<double>(turn_diff);

        int    v         = sidx3(vi, vj, k);
        double tentative = g[u] + step_cost[k] + turn_pen;
        if (tentative < g[v]) {
          g[v]            = tentative;
          parent_state[v] = u;
          open.emplace(g[v] + h(vi, vj), v);
        }
      }
    }

    if (goal_state < 0) return false;

    // Reconstruct directional state chain → unique 2-D cell chain
    std::vector<std::pair<int,int>> cells_rev;
    for (int s = goal_state; ; s = parent_state[s]) {
      int cell = s / D;
      int i = cell % W, j = cell / W;
      if (cells_rev.empty() ||
          cells_rev.back().first != i || cells_rev.back().second != j) {
        cells_rev.emplace_back(i, j);
      }
      if (parent_state[s] == s) break;
    }
    std::reverse(cells_rev.begin(), cells_rev.end());

    // Convert to 2-D parent array for the caller
    const int N = W * H;
    parents.assign(N, -1);
    parents[idx2d(si, sj)] = idx2d(si, sj);
    for (size_t k = 1; k < cells_rev.size(); ++k) {
      int uij = idx2d(cells_rev[k-1].first, cells_rev[k-1].second);
      int vij = idx2d(cells_rev[k].first,   cells_rev[k].second);
      parents[vij] = uij;
    }
    return true;
  }

  // ---------------------------------------------------------------------------
  // Publish occupancy grid for RViz debug
  // ---------------------------------------------------------------------------
  void publishOccGridMsg(const std::vector<uint8_t>& occ) {
    if (!occ_pub_) return;
    nav_msgs::msg::OccupancyGrid msg;
    msg.header.frame_id        = world_frame_;
    msg.header.stamp           = now();
    msg.info.resolution        = grid_.res;
    msg.info.width             = grid_.w;
    msg.info.height            = grid_.h;
    msg.info.origin.position.x = grid_.ox;
    msg.info.origin.position.y = grid_.oy;
    msg.info.origin.orientation.w = 1.0;
    msg.data.resize(grid_.w * grid_.h);
    for (int j = 0; j < grid_.h; ++j)
      for (int i = 0; i < grid_.w; ++i)
        msg.data[grid_.idx(i, j)] = occ[grid_.idx(i, j)] ? 100 : 0;
    occ_pub_->publish(msg);
  }

  // ---------------------------------------------------------------------------
  // Utilities
  // ---------------------------------------------------------------------------
  void stopRobot() {
    if (!publish_cmd_) return;
    cmd_pub_->publish(geometry_msgs::msg::Twist{});
  }
  static double yawFromQuat(const geometry_msgs::msg::Quaternion& q) {
    return std::atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z));
  }
  static double normAngle(double a) {
    while (a >  M_PI) a -= 2 * M_PI;
    while (a < -M_PI) a += 2 * M_PI;
    return a;
  }

  // ---------------------------------------------------------------------------
  // State machine
  // ---------------------------------------------------------------------------
  enum class State { NORMAL, PAUSE, FORWARD };
  State        recover_state_{State::NORMAL};
  rclcpp::Time state_start_time_{0, 0, RCL_ROS_TIME};

  // Parameters
  std::string world_frame_, pose_topic_, pc_topic_, cmd_topic_;
  bool   publish_cmd_{true};
  double inflate_r_{0.01}, half_extent_m_{10.0};
  double goal_x_{1.0}, goal_y_{-1.0}, goal_tol_{0.03};
  double v_max_{1.5}, w_max_{0.8};
  double range_max_keep_{10.0}, ground_z_thresh_{0.03};
  double lost_timeout_sec_{1.0}, recover_pause_sec_{0.2};
  double recover_forward_sec_{2.0}, recover_v_{0.5};
  int    lookahead_cells_{3}, forward_check_n_{20};
  double advance_dist_thresh_{0.1};
  double replan_min_interval_sec_{1.0}, replan_max_interval_sec_{4.0};
  double turn_weight_{2.0}, replace_improvement_ratio_{0.2};

  // ROS interfaces
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr occ_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr          path_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr    cmd_pub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr  pose_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr    pc_sub_;
  rclcpp::TimerBase::SharedPtr timer_;

  // Runtime state
  geometry_msgs::msg::PoseStamped cur_pose_;
  bool have_pose_{false}, have_obs_{false}, arrived_logged_{false};
  rclcpp::Time last_pose_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_plan_time_{0, 0, RCL_ROS_TIME};
  sensor_msgs::msg::PointCloud2::SharedPtr last_pc2_;
  Grid   grid_;
  std::vector<std::pair<int,int>> active_cells_;
  size_t active_idx_{0};
};

// ---------------------------------------------------------------------------
int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<AStarPlannerNode>());
  rclcpp::shutdown();
  return 0;
}
