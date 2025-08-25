#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/path.hpp>
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

#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>


struct Grid {
  double res{0.05}; // m/cell
  int    w{200};
  int    h{200};
  double ox{0.0};  // origin x of cell(0,0)
  double oy{0.0};  // origin y of cell(0,0)

  inline bool worldToCell(double x, double y, int& i, int& j) const {
    i = static_cast<int>(std::floor((x - ox) / res));
    j = static_cast<int>(std::floor((y - oy) / res));
    return (i>=0 && i<w && j>=0 && j<h);
  }
  inline void cellToWorld(int i, int j, double& x, double& y) const {
    x = ox + (i + 0.5) * res;
    y = oy + (j + 0.5) * res;
  }
  inline int idx(int i, int j) const { return j*w + i; }
};

class AStarPlannerNode : public rclcpp::Node {
public:
  AStarPlannerNode() : Node("hybrid_astar_planner")
  {
    // 기본 파라미터
    world_frame_   = declare_parameter<std::string>("world_frame", "map");
    pose_topic_    = declare_parameter<std::string>("pose_topic", "/orb_slam2/pose");
    pc_topic_      = declare_parameter<std::string>("pc_topic", "/orb_slam2/landmarks"); // PointCloud2
    grid_.res      = declare_parameter<double>("grid_resolution", 0.05);
    inflate_r_     = declare_parameter<double>("inflate_radius", 0.01);
    half_extent_m_ = declare_parameter<double>("half_extent_m", 10.0);

    // 목표
    auto goal = declare_parameter<std::vector<double>>("goal", std::vector<double>{1.0, -1.0, 0.0});
    if (goal.size() >= 2) { goal_x_ = goal[0]; goal_y_ = goal[1]; }
    goal_tol_ = declare_parameter<double>("goal_tolerance", 0.03);

    // 속도 제한 및 제어
    publish_cmd_ = declare_parameter<bool>("publish_cmd_vel", true);
    cmd_topic_   = declare_parameter<std::string>("cmd_topic", "/cmd_vel");
    v_max_       = declare_parameter<double>("v_max", 1.5);
    w_max_       = declare_parameter<double>("w_max", 0.8);

    // 단순 필터
    range_max_keep_   = declare_parameter<double>("range_max_keep", 10.0);
    ground_z_thresh_  = declare_parameter<double>("ground_z_thresh", 0.03); // z<= 이면 바닥으로 제거

    // 로스트/복구
    lost_timeout_sec_   = declare_parameter<double>("lost_timeout_sec", 1.0);
    recover_pause_sec_  = declare_parameter<double>("recover_pause_sec", 0.2);
    recover_forward_sec_= declare_parameter<double>("recover_forward_sec", 2.0);
    recover_v_          = declare_parameter<double>("recover_v", 0.5);

    // ★ 추종/재계획 관련(필요 최소만 유지)
    lookahead_cells_         = declare_parameter<int>("lookahead_cells", 3);
    advance_dist_thresh_     = declare_parameter<double>("advance_dist_thresh", 0.1);
    forward_check_n_         = declare_parameter<int>("forward_check_n", 20);
    replan_min_interval_sec_ = declare_parameter<double>("replan_min_interval_sec", 1.0);
    replan_max_interval_sec_ = declare_parameter<double>("replan_max_interval_sec", 4.0);

    // ★ 방향 A* 회전 가중치
    turn_weight_             = declare_parameter<double>("turn_weight", 2.0);

    // ★ 새 경로 교체 임계(기존 대비 몇 % 이상 개선 시 교체할지)
    replace_improvement_ratio_ = declare_parameter<double>("replace_improvement_ratio", 0.2); // 20%
    
    // RViz용 OccupancyGrid 퍼블리셔
    occ_pub_ = create_publisher<nav_msgs::msg::OccupancyGrid>("debug/occupancy", 1);


    // 퍼블리셔/서브스크라이버
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

    RCLCPP_INFO(get_logger(), "A* planner ready. pc_topic=%s pose_topic=%s (z<=%.2f treated as ground, turn_weight=%.2f)",
                pc_topic_.c_str(), pose_topic_.c_str(), ground_z_thresh_, turn_weight_);
  }

private:
  /* ===== 콜백 ===== */
  void onPose(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
    if (msg->header.frame_id != world_frame_) return;
    cur_pose_ = *msg;
    have_pose_ = true;
    last_pose_time_ = now();

    // 포즈가 다시 들어오면 복구 상태 해제
    if (recover_state_ != State::NORMAL) {
      recover_state_ = State::NORMAL;
    }
  }

  void onCloud(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
    last_pc2_ = msg;
    have_obs_ = true;
  }

  /* ===== 보조: 길이/비용 계산 ===== */
  double pathCost(const std::vector<std::pair<int,int>>& cells) const {
    if (cells.size() < 2) return 0.0;
    double sum = 0.0;
    for (size_t k=1; k<cells.size(); ++k) {
      int di = std::abs(cells[k].first  - cells[k-1].first);
      int dj = std::abs(cells[k].second - cells[k-1].second);
      sum += (di+dj==2 ? std::sqrt(2.0) : 1.0);
    }
    return sum;
  }

  bool upcomingBlocked(const std::vector<uint8_t>& occ,
                       const std::vector<std::pair<int,int>>& cells,
                       size_t from_idx, int N) const
  {
    if (cells.empty()) return true;
    const size_t end = std::min(from_idx + static_cast<size_t>(N), cells.size()-1);
    for (size_t k = from_idx; k <= end; ++k) {
      int i = cells[k].first;
      int j = cells[k].second;
      if (i<0||j<0||i>=grid_.w||j>=grid_.h) return true;
      if (occ[grid_.idx(i,j)]) return true;
    }
    return false;
  }

  /* ===== 주기적 계획 ===== */
  void planTick() {
    // 도착 체크(우선 멈춤/메시지)
    if (have_pose_) {
      double dx = cur_pose_.pose.position.x - goal_x_;
      double dy = cur_pose_.pose.position.y - goal_y_;
      double dist_goal = std::hypot(dx, dy);
      if (dist_goal <= goal_tol_) {
        stopRobot();
        if (!arrived_logged_) {
          RCLCPP_INFO(get_logger(), "Goal reached! (within %.2fm)", goal_tol_);
          arrived_logged_ = true;
        }
        return;
      }
    }

    // 포즈 끊김 감지 → 복구 상태머신
    const bool lost = (!have_pose_) ||
                      ((now() - last_pose_time_).seconds() > lost_timeout_sec_);
    if (lost) {
      if (recover_state_ == State::NORMAL) {
        recover_state_ = State::PAUSE;
        state_start_time_ = now();
        stopRobot();
        RCLCPP_WARN(get_logger(), "Pose lost → PAUSE for %.2fs", recover_pause_sec_);
        return;
      } else if (recover_state_ == State::PAUSE) {
        if ((now() - state_start_time_).seconds() >= recover_pause_sec_) {
          recover_state_ = State::FORWARD;
          state_start_time_ = now();
          RCLCPP_WARN(get_logger(), "Pose still lost → FORWARD for %.2fs (v=%.2f)",
                      recover_forward_sec_, recover_v_);
        } else {
          stopRobot();
          return;
        }
      } else if (recover_state_ == State::FORWARD) {
        if ((now() - state_start_time_).seconds() < recover_forward_sec_) {
          geometry_msgs::msg::Twist cmd;
          cmd.linear.x = recover_v_;
          cmd.angular.z = 0.0;
          if (publish_cmd_) cmd_pub_->publish(cmd);
          return; // 복구 중에는 플래닝 스킵
        } else {
          recover_state_ = State::PAUSE;
          state_start_time_ = now();
          stopRobot();
          return;
        }
      }
    }

    // 정상 상태가 아니면 스킵
    if (recover_state_ != State::NORMAL) return;
    if (!have_pose_ || !have_obs_) return;

    const double cx = cur_pose_.pose.position.x;
    const double cy = cur_pose_.pose.position.y;

    // 그리드 세팅
    grid_.w  = static_cast<int>(std::ceil((2.0*half_extent_m_) / grid_.res));
    grid_.h  = grid_.w;
    grid_.ox = cx - half_extent_m_;
    grid_.oy = cy - half_extent_m_;

    // 장애물 rasterize
    std::vector<uint8_t> occ(grid_.w * grid_.h, 0);
    size_t kept=0, dropped_ground=0, dropped_range=0, dropped_nan=0;
    rasterizeSimple(occ, kept, dropped_ground, dropped_range, dropped_nan);

    // inflate
    inflate(occ, static_cast<int>(std::ceil(inflate_r_ / grid_.res)));

    // 시작/목표
    int si,sj, gi,gj;
    if (!grid_.worldToCell(cx, cy, si, sj)) return;
    if (!grid_.worldToCell(goal_x_, goal_y_, gi, gj)) return;
    
    ///
    // 디버그 시각화/퍼블리시
    publishOccGridMsg(occ);            // RViz: /debug/occupancy
    ///


    // ====== 경로 존재/막힘/주기 기반 재계획 판단 ======
    const auto now_t = now();
    const double since_last_plan = (now_t - last_plan_time_).seconds();

    bool need_plan = false;

    if (active_cells_.empty()) {
      need_plan = true;
    } else {
      // 진행 인덱스 갱신
      advanceAlongActivePath(cx, cy);

      // 앞쪽 구간 막힘?
      if (upcomingBlocked(occ, active_cells_, active_idx_, forward_check_n_)) {
        need_plan = true;
      }

      // 최대 재계획 주기 초과?
      if (!need_plan && since_last_plan >= replan_max_interval_sec_) {
        need_plan = true;
      }

      // 최소 재계획 주기 이내면 보류
      if (need_plan && since_last_plan < replan_min_interval_sec_) {
        need_plan = false;
      }
    }

    // ====== 필요시 방향 A*로 새 경로 계산 ======
    if (need_plan) {
      std::vector<int> parents;
      if (occ[grid_.idx(si,sj)] || occ[grid_.idx(gi,gj)]) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "Start/Goal is in obstacle.");
        stopRobot();
        return;
      }
      if (!astar_dir(occ, si,sj, gi,gj, parents)) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "A* failed (no path).");
        stopRobot();
        return;
      }

      // 후보 경로 복원 (2D parents → cell 체인)
      std::vector<std::pair<int,int>> candidate;
      {
        int cur = grid_.idx(gi,gj);
        const int sidx = grid_.idx(si,sj);
        while (cur != sidx && cur>=0 && cur<(int)parents.size()) {
          int i = cur % grid_.w;
          int j = cur / grid_.w;
          candidate.emplace_back(i,j);
          int p = parents[cur];
          if (p == cur) break;
          cur = p;
        }
        candidate.emplace_back(si,sj);
        std::reverse(candidate.begin(), candidate.end());
      }

      // === 여기부터 교체 정책 ===
      bool replace = false;

      if (active_cells_.empty()) {
        // 경로 자체가 없으면 무조건 교체
        replace = true;
      } else {
        // 앞쪽 구간이 막혔으면 무조건 교체
        bool blocked = upcomingBlocked(occ, active_cells_, active_idx_, forward_check_n_);
        if (blocked) {
          replace = true;
        } else {
          // 비용 비교: 새 경로가 '남은 기존 경로'보다 30% 이상 더 짧아야 교체
          double cand_cost = pathCost(candidate);
          std::vector<std::pair<int,int>> remain(active_cells_.begin() + static_cast<long>(active_idx_), active_cells_.end());
          double remain_cost = pathCost(remain);

          if (cand_cost < remain_cost * (1.0 - replace_improvement_ratio_)) {
            replace = true;
          } else {
            RCLCPP_INFO(get_logger(),
              "Keep current path (cand=%.2f, remain=%.2f, thresh=%.0f%%).",
              cand_cost, remain_cost, replace_improvement_ratio_*100.0);
          }
        }
      }

      if (replace) {
        active_cells_ = std::move(candidate);
        active_idx_   = 0;
        RCLCPP_INFO(get_logger(), "Path replaced.");
      }

      // 재계획을 했다는 사실은 타이머로 기록해서 과도한 재계산 방지
      last_plan_time_ = now_t;
    }

    // ====== active path를 기준으로 Path 퍼블리시 + 추종 ======
    if (!active_cells_.empty()) {
      // Path publish
      nav_msgs::msg::Path path;
      path.header.frame_id = world_frame_;
      path.header.stamp    = now();
      path.poses.reserve(active_cells_.size());
      for (auto& c : active_cells_) {
        double x,y; grid_.cellToWorld(c.first, c.second, x,y);
        geometry_msgs::msg::PoseStamped ps;
        ps.header = path.header;
        ps.pose.position.x = x;
        ps.pose.position.y = y;
        ps.pose.orientation.w = 1.0;
        path.poses.push_back(ps);
      }
      path_pub_->publish(path);

      // lookahead 추종
      if (publish_cmd_) {
        geometry_msgs::msg::Twist cmd;
        size_t tgt = std::min(active_idx_ + static_cast<size_t>(lookahead_cells_),
                              active_cells_.size()-1);
        double tx,ty; grid_.cellToWorld(active_cells_[tgt].first, active_cells_[tgt].second, tx,ty);
        const double dx = tx - cur_pose_.pose.position.x;
        const double dy = ty - cur_pose_.pose.position.y;
        const double target_yaw = std::atan2(dy, dx);
        const double cyaw = yawFromQuat(cur_pose_.pose.orientation);
        const double eyaw = normAngle(target_yaw - cyaw);
        const double dist = std::hypot(dx,dy);



        cmd.linear.x  = std::clamp(1.0*dist, 0.0, v_max_);
        cmd.angular.z = std::clamp(1.5*eyaw, -w_max_, w_max_);
        cmd_pub_->publish(cmd);

      }

      RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000,
        "Obs(simple): kept=%zu, ground(z<=%.2f)=%zu, range=%zu, nan=%zu | path sz=%zu idx=%zu",
        kept, ground_z_thresh_, dropped_ground, dropped_range, dropped_nan,
        active_cells_.size(), active_idx_);
    }
  }

  // active path 따라 인덱스 진행
  void advanceAlongActivePath(double cx, double cy) {
    if (active_cells_.empty()) return;
    while (active_idx_ < active_cells_.size()) {
      double tx,ty;
      grid_.cellToWorld(active_cells_[active_idx_].first,
                        active_cells_[active_idx_].second, tx,ty);
      double d = std::hypot(tx - cx, ty - cy);
      if (d <= advance_dist_thresh_) {
        if (active_idx_ + 1 < active_cells_.size()) ++active_idx_;
        else break;
      } else {
        break;
      }
    }
  }

  /* ===== 단순 장애물 rasterize ===== */
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
        const float x=*it_x, y=*it_y, z=*it_z;
        if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) { ++drop_nan; continue; }

        // (1) 바닥 제거: z <= ground_z_thresh_ → DROP
        if (z <= ground_z_thresh_) { ++drop_ground; continue; }

        // (2) 범위 제한
        const double dx = x - cur_pose_.pose.position.x;
        const double dy = y - cur_pose_.pose.position.y;
        if (std::hypot(dx, dy) > range_max_keep_) { ++drop_range; continue; }

        int i,j; if (grid_.worldToCell(x,y,i,j)) {
          occ[grid_.idx(i,j)] = 100;
          ++kept;
        }
      }
    } catch (...) {
      // 필드가 없거나 타입이 다른 경우 조용히 무시
    }
  }

  /* ===== inflate ===== */
  void inflate(std::vector<uint8_t>& occ, int r) {
    if (r <= 0) return;
    std::vector<uint8_t> copy = occ;
    for (int j=0;j<grid_.h;++j) {
      for (int i=0;i<grid_.w;++i) {
        if (copy[grid_.idx(i,j)] == 0) continue;
        for (int dj=-r; dj<=r; ++dj) {
          for (int di=-r; di<=r; ++di) {
            int ni=i+di, nj=j+dj;
            if (ni<0||nj<0||ni>=grid_.w||nj>=grid_.h) continue;
            occ[grid_.idx(ni,nj)] = 100;
          }
        }
      }
    }
  }

  /* ===== 방향 확장 A* ===== */
  bool astar_dir(const std::vector<uint8_t>& occ, int si,int sj, int gi,int gj,
                 std::vector<int>& parents)
  {
    const int W = grid_.w, H = grid_.h, D = 8;
    auto idx2d = [&](int i,int j){ return j*W + i; };
    auto sidx  = idx2d(si,sj);
    auto gidx  = idx2d(gi,gj);
    auto sidx3 = [&](int i,int j,int d){ return (idx2d(i,j)*D) + d; };

    // 방향: E, NE, N, NW, W, SW, S, SE
    const int di[D] = {1, 1, 0,-1,-1,-1, 0, 1};
    const int dj[D] = {0, 1, 1, 1, 0,-1,-1,-1};
    const double step_cost[D] = {1, std::sqrt(2), 1, std::sqrt(2), 1, std::sqrt(2), 1, std::sqrt(2)};

    auto h = [&](int i,int j){ return std::hypot(i - gi, j - gj); };

    const int Nstate = W*H*D;
    std::vector<double> g(Nstate, std::numeric_limits<double>::infinity());
    std::vector<int>    parent_state(Nstate, -1);

    auto fcmp = [&](const std::pair<double,int>& a, const std::pair<double,int>& b){
      return a.first > b.first;
    };
    std::priority_queue<std::pair<double,int>, std::vector<std::pair<double,int>>, decltype(fcmp)> open(fcmp);

    // 시작: 모든 방향을 동일 가중치로 시드
    for (int d=0; d<D; ++d) {
      int s = sidx3(si,sj,d);
      g[s] = 0.0;
      parent_state[s] = s;
      open.emplace(h(si,sj), s);
    }

    int goal_state = -1;

    while (!open.empty()) {
      auto [fcost, u] = open.top(); open.pop();
      int cell = u / D;
      int dcur = u % D;
      int ui = cell % W;
      int uj = cell / W;

      if (cell == gidx) { goal_state = u; break; }

      for (int k=0; k<D; ++k) {
        int vi = ui + di[k];
        int vj = uj + dj[k];
        if (vi<0 || vj<0 || vi>=W || vj>=H) continue;
        int vcell = idx2d(vi,vj);
        if (occ[vcell]) continue;

        // 회전량(0~4): 방향 인덱스의 원형 거리
        int dd = std::abs(k - dcur);
        int turn_diff = std::min(dd, D - dd);
        double turn_pen = turn_weight_ * static_cast<double>(turn_diff);

        int v = sidx3(vi,vj,k);
        double tentative = g[u] + step_cost[k] + turn_pen;

        if (tentative < g[v]) {
          g[v] = tentative;
          parent_state[v] = u;
          open.emplace(g[v] + h(vi,vj), v);
        }
      }
    }

    if (goal_state < 0) return false;

    // 상태 체인 → 2D 셀 체인
    std::vector<std::pair<int,int>> cells_rev;
    for (int s = goal_state; ; s = parent_state[s]) {
      int cell = s / D;
      int i = cell % W;
      int j = cell / W;
      if (cells_rev.empty() || cells_rev.back().first != i || cells_rev.back().second != j) {
        cells_rev.emplace_back(i,j);
      }
      if (parent_state[s] == s) break; // start
    }
    std::reverse(cells_rev.begin(), cells_rev.end());

    // 2D parents 채우기
    const int N = W*H;
    parents.assign(N, -1);
    parents[sidx] = sidx;
    for (size_t k=1; k<cells_rev.size(); ++k) {
      int uij = idx2d(cells_rev[k-1].first, cells_rev[k-1].second);
      int vij = idx2d(cells_rev[k].first,   cells_rev[k].second);
      parents[vij] = uij;
    }
    return true;
  }





  //
  // RViz에서 볼 수 있도록 nav_msgs/OccupancyGrid 퍼블리시
  void publishOccGridMsg(const std::vector<uint8_t>& occ)
  {
    if (!occ_pub_) return;

    nav_msgs::msg::OccupancyGrid msg;
    msg.header.frame_id = world_frame_;
    msg.header.stamp    = now();

    msg.info.resolution = grid_.res;
    msg.info.width      = grid_.w;
    msg.info.height     = grid_.h;
    msg.info.origin.position.x = grid_.ox;
    msg.info.origin.position.y = grid_.oy;
    msg.info.origin.position.z = 0.0;
    msg.info.origin.orientation.w = 1.0;

    msg.data.resize(grid_.w * grid_.h);
    for (int j=0; j<grid_.h; ++j) {
      for (int i=0; i<grid_.w; ++i) {
        int id = grid_.idx(i,j);
        // 0: free, 100: occupied
        msg.data[id] = occ[id] ? 100 : 0;
      }
    }

    occ_pub_->publish(msg);
  }
  //

  /* ===== 유틸 ===== */
  void stopRobot() {
    if (!publish_cmd_) return;
    geometry_msgs::msg::Twist zero;
    cmd_pub_->publish(zero);
  }
  static double yawFromQuat(const geometry_msgs::msg::Quaternion& q) {
    double siny_cosp = 2.0*(q.w*q.z + q.x*q.y);
    double cosy_cosp = 1.0 - 2.0*(q.y*q.y + q.z*q.z);
    return std::atan2(siny_cosp, cosy_cosp);
  }
  static double normAngle(double a) { while(a>M_PI) a-=2*M_PI; while(a<-M_PI) a+=2*M_PI; return a; }

private:
  // 상태 머신
  enum class State { NORMAL, PAUSE, FORWARD };
  State        recover_state_{State::NORMAL};
  rclcpp::Time state_start_time_{0,0,RCL_ROS_TIME};

  // params
  std::string world_frame_, pose_topic_, pc_topic_, cmd_topic_;
  bool   publish_cmd_{true};
  double inflate_r_{0.01}, half_extent_m_{10.0};
  double goal_x_{1.0}, goal_y_{-1.0}, goal_tol_{0.03};
  double v_max_{1.5}, w_max_{0.8};

  // 단순 필터
  double range_max_keep_{10.0};
  double ground_z_thresh_{0.03}; // z<=이 값은 바닥으로 간주

  // 로스트/복구
  double       lost_timeout_sec_{1.0};
  double       recover_pause_sec_{0.2};
  double       recover_forward_sec_{2.0};
  double       recover_v_{0.5};
  rclcpp::Time last_pose_time_{0,0,RCL_ROS_TIME};

  // 추종/재계획(필요 최소)
  int     lookahead_cells_{3};
  double  advance_dist_thresh_{0.1};
  int     forward_check_n_{20};
  double  replan_min_interval_sec_{1.0};
  double  replan_max_interval_sec_{4.0};
  rclcpp::Time last_plan_time_{0,0,RCL_ROS_TIME};

  // 방향 A* 회전 가중치
  double  turn_weight_{2.0};

  // 새 경로 교체 임계(비용 개선율)
  double  replace_improvement_ratio_{0.2};

  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr occ_pub_;

  // subs/pubs
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr   pc_sub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  // state
  geometry_msgs::msg::PoseStamped cur_pose_;
  bool have_pose_{false}, have_obs_{false}, arrived_logged_{false};
  sensor_msgs::msg::PointCloud2::SharedPtr last_pc2_;
  Grid grid_;

  // Active path
  std::vector<std::pair<int,int>> active_cells_;
  size_t active_idx_{0};
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<AStarPlannerNode>());
  rclcpp::shutdown();
  return 0;
}
