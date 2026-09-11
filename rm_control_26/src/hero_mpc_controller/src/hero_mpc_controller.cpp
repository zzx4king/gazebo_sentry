// Copyright 2025 Jinbo Liu
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

/**
 * @file hero_mpc_controller.cpp
 * @brief 基于 MINCO 轨迹和 acados MPC 的 Nav2 Controller 插件实现
 *
 * 本文件实现了 HeroMpcController 类的所有成员函数。
 *
 * 核心控制流程:
 * ==========================================================================
 * 1. configure(): 初始化参数、MPC 求解器、订阅器
 * 2. mincoTrajectoryCallback(): 接收并解析 MINCO 多项式轨迹
 * 3. computeVelocityCommands(): 每个控制周期执行
 *    a. 获取当前机器人状态（位置、速度）
 *    b. 检查轨迹有效性
 *    c. 计算轨迹相对时间
 *    d. 在 MINCO 轨迹上采样 N+1 个参考点
 *    e. 设置 MPC 初始状态和参考轨迹
 *    f. 调用 MPC 求解
 *    g. 提取速度指令并转换坐标系
 *    h. 发布速度指令
 *
 * 关键坐标系转换:
 * ==========================================================================
 * - Nav2 输入的 velocity 是机器人坐标系 (base_link)
 * - MPC 使用世界坐标系状态
 * - 输出 cmd_vel 是机器人坐标系
 *
 * 作者: Jinbo Liu
 * 日期: 2025.12.27
 */

#include "hero_mpc_controller/hero_mpc_controller.hpp"

#include <algorithm>
#include <cmath>
#include <rclcpp/logging.hpp>
#include <rclcpp/qos.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include "nav2_core/controller_exceptions.hpp"
#include "pluginlib/class_list_macros.hpp"

using nav2_util::declare_parameter_if_not_declared;

namespace hero_mpc_controller {

// =============================================================================
// Nav2 Controller 接口实现
// =============================================================================

void HeroMpcController::configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr& parent, std::string name,
    std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) {
    auto node = parent.lock();
    node_ = parent;
    if (!node) {
        throw nav2_core::ControllerException("无法锁定节点!");
    }

    costmap_ros_ = costmap_ros;
    tf_ = tf;
    plugin_name_ = name;
    logger_ = node->get_logger();
    clock_ = node->get_clock();

    // -------------------------------------------------------------------------
    // 声明和获取参数
    // -------------------------------------------------------------------------
    RCLCPP_INFO(logger_, "[%s] 正在配置 MPC 控制器...", plugin_name_.c_str());

    // 轨迹超时参数
    declare_parameter_if_not_declared(node,
                                      plugin_name_ + ".trajectory_timeout",
                                      rclcpp::ParameterValue(1.0));
    node->get_parameter(plugin_name_ + ".trajectory_timeout", traj_timeout_);

    // 速度约束参数
    declare_parameter_if_not_declared(node, plugin_name_ + ".max_velocity",
                                      rclcpp::ParameterValue(2.0));
    declare_parameter_if_not_declared(node, plugin_name_ + ".max_omega",
                                      rclcpp::ParameterValue(1.5));
    declare_parameter_if_not_declared(node, plugin_name_ + ".max_acceleration",
                                      rclcpp::ParameterValue(4.0));
    declare_parameter_if_not_declared(node, plugin_name_ + ".max_alpha",
                                      rclcpp::ParameterValue(1.0));

    node->get_parameter(plugin_name_ + ".max_velocity", max_vel_);
    node->get_parameter(plugin_name_ + ".max_omega", max_omega_);
    node->get_parameter(plugin_name_ + ".max_acceleration", max_acc_);
    node->get_parameter(plugin_name_ + ".max_alpha", max_alpha_);

    // 航向跟踪参数
    declare_parameter_if_not_declared(node,
                                      plugin_name_ + ".enable_yaw_tracking",
                                      rclcpp::ParameterValue(false));
    declare_parameter_if_not_declared(node, plugin_name_ + ".reference_yaw",
                                      rclcpp::ParameterValue(0.0));

    node->get_parameter(plugin_name_ + ".enable_yaw_tracking",
                        enable_yaw_tracking_);
    node->get_parameter(plugin_name_ + ".reference_yaw", reference_yaw_);

    // TF 容差
    double transform_tolerance = 0.1;
    declare_parameter_if_not_declared(node,
                                      plugin_name_ + ".transform_tolerance",
                                      rclcpp::ParameterValue(0.1));
    node->get_parameter(plugin_name_ + ".transform_tolerance",
                        transform_tolerance);
    transform_tolerance_ = tf2::durationFromSec(transform_tolerance);

    // 控制频率
    declare_parameter_if_not_declared(node, "controller_frequency",
                                      rclcpp::ParameterValue(100.0));
    node->get_parameter("controller_frequency", control_frequency_);

    // 权重参数 Q (6 维状态)
    declare_parameter_if_not_declared(
        node, plugin_name_ + ".weight_q",
        rclcpp::ParameterValue(
            std::vector<double>{400.0, 100.0, 0.0, 10.0, 10.0, 0.0}));
    node->get_parameter(plugin_name_ + ".weight_q", weight_q_);

    // 权重参数 R (3 维控制)
    declare_parameter_if_not_declared(
        node, plugin_name_ + ".weight_r",
        rclcpp::ParameterValue(std::vector<double>{1.0, 1.0, 0.5}));
    node->get_parameter(plugin_name_ + ".weight_r", weight_r_);

    // Carrot Pose 发布器（用于发布预测点在 base_link 坐标系下的位置）
    declare_parameter_if_not_declared(
        node, plugin_name_ + ".carrot_point_index", rclcpp::ParameterValue(5));
    node->get_parameter(plugin_name_ + ".carrot_point_index",
                        carrot_point_index_);

    // -------------------------------------------------------------------------
    // 初始化 MPC 求解器
    // -------------------------------------------------------------------------
    mpc_wrapper_ = std::make_unique<MpcWrapper>();
    if (!mpc_wrapper_->init()) {
        throw nav2_core::ControllerException("MPC 求解器初始化失败!");
    }

    // 设置权重矩阵
    if (weight_q_.size() == kStateSize && weight_r_.size() == kInputSize) {
        StateWeightMatrix Q = StateWeightMatrix::Zero();
        InputWeightMatrix R = InputWeightMatrix::Zero();

        for (int i = 0; i < kStateSize; ++i) {
            Q(i, i) = weight_q_[i];
        }
        for (int i = 0; i < kInputSize; ++i) {
            R(i, i) = weight_r_[i];
        }

        mpc_wrapper_->set_weights(Q, R);
        RCLCPP_INFO(logger_, "[%s] 权重矩阵设置成功", plugin_name_.c_str());
    } else {
        RCLCPP_WARN(logger_, "[%s] 权重参数维度不匹配，使用默认值",
                    plugin_name_.c_str());
    }

    // 设置约束
    mpc_wrapper_->set_control_bounds(max_acc_, max_alpha_);
    mpc_wrapper_->set_velocity_bounds(max_vel_, max_omega_);

    // -------------------------------------------------------------------------
    // 创建订阅器和发布器
    // -------------------------------------------------------------------------

    // MINCO 轨迹订阅器
    // 话题名称: ~/minco_polynomial_trajectory
    minco_traj_sub_ =
        node->create_subscription<interfaces::msg::MincoTrajectory>(
            "/smoother_server/minco_polynomial_trajectory", rclcpp::QoS(10),
            std::bind(&HeroMpcController::mincoTrajectoryCallback, this,
                      std::placeholders::_1));

    // 可视化发布器
    predicted_path_pub_ =
        node->create_publisher<visualization_msgs::msg::MarkerArray>(
            plugin_name_ + "/predicted_path", 10);
    reference_path_pub_ =
        node->create_publisher<visualization_msgs::msg::MarkerArray>(
            plugin_name_ + "/reference_path", 10);
    carrot_pose_pub_ = node->create_publisher<interfaces::msg::NavOutput>(
        "/carrot_pose", rclcpp::SensorDataQoS());

    RCLCPP_INFO(logger_, "[%s] 配置完成: max_vel=%.2f, max_omega=%.2f, dt=%.4f",
                plugin_name_.c_str(), max_vel_, max_omega_,
                MpcWrapper::get_timestep());
}

void HeroMpcController::cleanup() {
    RCLCPP_INFO(logger_, "[%s] 正在清理...", plugin_name_.c_str());
    minco_traj_sub_.reset();
    predicted_path_pub_.reset();
    reference_path_pub_.reset();
    carrot_pose_pub_.reset();
    mpc_wrapper_.reset();
}

void HeroMpcController::activate() {
    RCLCPP_INFO(logger_, "[%s] 激活", plugin_name_.c_str());
    predicted_path_pub_->on_activate();
    reference_path_pub_->on_activate();
    carrot_pose_pub_->on_activate();
}

void HeroMpcController::deactivate() {
    RCLCPP_INFO(logger_, "[%s] 停用", plugin_name_.c_str());
    predicted_path_pub_->on_deactivate();
    reference_path_pub_->on_deactivate();
    carrot_pose_pub_->on_deactivate();
}

geometry_msgs::msg::TwistStamped HeroMpcController::computeVelocityCommands(
    const geometry_msgs::msg::PoseStamped& pose,
    const geometry_msgs::msg::Twist& velocity,
    nav2_core::GoalChecker* /*goal_checker*/) {
    // 初始化输出速度指令
    geometry_msgs::msg::TwistStamped cmd_vel;
    cmd_vel.header = pose.header;
    cmd_vel.header.stamp = clock_->now();
    cmd_vel.twist.linear.x = 0.0;
    cmd_vel.twist.linear.y = 0.0;
    cmd_vel.twist.angular.z = 0.0;

    // 步骤 1: 检查轨迹有效性
    {
        std::lock_guard<std::mutex> lock(traj_mutex_);

        if (!traj_valid_) {
            RCLCPP_WARN_THROTTLE(logger_, *clock_, 1000,
                                 "[%s] 轨迹无效，输出零速度",
                                 plugin_name_.c_str());
            return cmd_vel;
        }

        // 检查轨迹是否超时
        double traj_age = (clock_->now() - traj_start_time_).seconds();
        if (traj_age > traj_total_duration_ + traj_timeout_) {
            RCLCPP_WARN_THROTTLE(
                logger_, *clock_, 1000,
                "[%s] 轨迹已超时 (age=%.2fs, duration=%.2fs)，输出零速度",
                plugin_name_.c_str(), traj_age, traj_total_duration_);
            return cmd_vel;
        }
    }

    // 步骤 2: 获取当前机器人状态
    // [修订] 统一转换到 map 坐标系
    geometry_msgs::msg::PoseStamped robot_pose_map;

    // 如果输入的 pose 不是 map 系，则进行转换
    // 为了满足“不做任何等待延迟”，我们查询最新的 TF (TimePointZero)
    if (pose.header.frame_id != "map") {
        try {
            // lookupTransform(target_frame, source_frame, time)
            auto transform = tf_->lookupTransform("map", pose.header.frame_id,
                                                  tf2::TimePointZero);
            tf2::doTransform(pose, robot_pose_map, transform);
        } catch (const tf2::TransformException& ex) {
            // 降级处理：仅在无法转换时报错，并使用原始 pose
            // (防止此时没有任何输入给 MPC)
            RCLCPP_ERROR_THROTTLE(
                logger_, *clock_, 1000,
                "[%s] 无法将机器人姿态转换到 map 坐标系 (TF Error: %s)",
                plugin_name_.c_str(), ex.what());
            robot_pose_map = pose;
        }
    } else {
        robot_pose_map = pose;
    }

    // 从 robot_pose_map 获取位置和航向
    double x = robot_pose_map.pose.position.x;
    double y = robot_pose_map.pose.position.y;
    double yaw = tf2::getYaw(robot_pose_map.pose.orientation);

    // 坐标系转换: 将机器人坐标系速度转换为世界坐标系速度
    // Nav2 提供的 velocity 是机器人坐标系下的速度 (base_link)
    double vx_body = velocity.linear.x;
    double vy_body = velocity.linear.y;
    double omega = velocity.angular.z;

    // 步骤 3: 设置 MPC 初始状态
    // 【工业级改造】状态截断 (State Decoupling)
    // 速度不再使用带噪声且受迫干扰的真实里程计速度，而是使用 MPC
    // 上一次的最优预测指令。
    // 这能有效防串扰（屏蔽细微抖动）和防止死锁（通过积分积攒逃逸力矩）。
    static double last_cmd_vx_world = 0.0;
    static double last_cmd_vy_world = 0.0;
    static double last_cmd_omega = 0.0;
    static rclcpp::Time last_call_time = clock_->now();

    rclcpp::Time current_time = clock_->now();
    double dt_call = (current_time - last_call_time).seconds();
    last_call_time = current_time;

    // 如果控制中断时间过长（例如重规划、停机重新开跑等），将内部记忆库对齐回现实世界
    // if (dt_call > 0.5) {
    //     bodyToWorld(vx_body, vy_body, yaw, last_cmd_vx_world,
    //                 last_cmd_vy_world);
    //     last_cmd_omega = omega;
    //     RCLCPP_INFO(
    //         logger_,
    //         "[%s] 控制间隔过长(%.2fs)，已重置速度记忆锚点至当前真实里程计",
    //         plugin_name_.c_str(), dt_call);
    // }

    // 将混合状态灌入 MPC (位置 x, y, yaw
    // 严格以里程计为准闭环，速度以指令作为基准)
    mpc_wrapper_->set_state(x, y, yaw, last_cmd_vx_world, last_cmd_vy_world,
                            last_cmd_omega);

    // 步骤 4: 计算轨迹相对时间并采样参考轨迹
    double t_current;
    {
        std::lock_guard<std::mutex> lock(traj_mutex_);
        t_current = (clock_->now() - traj_start_time_).seconds();
    }

    // 采样参考轨迹并设置到 MPC
    const int N = MpcWrapper::get_horizon_steps();
    const double dt = MpcWrapper::get_timestep();

    for (int k = 0; k <= N; ++k) {
        double t_k = t_current + k * dt;
        TrajectoryState ref_state = sampleTrajectory(t_k);

        // 计算参考航向角
        // 如果启用航向跟踪，根据速度方向计算；否则使用固定值
        double psi_ref;
        if (enable_yaw_tracking_) {
            // 使用速度方向作为参考航向
            double vel_norm = std::hypot(ref_state.vx, ref_state.vy);
            if (vel_norm > 0.1) {
                psi_ref = std::atan2(ref_state.vy, ref_state.vx);
            } else {
                psi_ref = yaw;  // 速度太小时保持当前航向
            }
        } else {
            psi_ref = reference_yaw_;
        }

        // 计算参考角速度（航向变化率）
        double omega_ref = 0.0;

        // 计算参考角加速度
        double alpha_ref = 0.0;

        if (k < N) {
            // 路径代价参考 (9 维): [p_x, p_y, ψ, v_x, v_y, ω, a_x, a_y, α]
            RefVector yref;
            yref << ref_state.x, ref_state.y, psi_ref, ref_state.vx,
                ref_state.vy, omega_ref, ref_state.ax, ref_state.ay, alpha_ref;

            mpc_wrapper_->set_reference(k, yref);
        } else {
            // 终端代价参考 (6 维): [p_x, p_y, ψ, v_x, v_y, ω]
            EndRefVector yref_e;
            yref_e << ref_state.x, ref_state.y, psi_ref, ref_state.vx,
                ref_state.vy, omega_ref;

            mpc_wrapper_->set_terminal_reference(yref_e);
        }
    }

    // 步骤 5: 求解 MPC
    int status = mpc_wrapper_->solve();

    if (status != 0) {
        RCLCPP_WARN(logger_, "[%s] MPC 求解失败 (status=%d)，输出零速度",
                    plugin_name_.c_str(), status);
        return cmd_vel;
    }

    // 步骤 6: 获取最优速度并转换坐标系
    // 获取 MPC 预测的下一个状态 x_1
    // 从中提取速度分量 [v_x, v_y, ω] 作为速度指令
    // 这比直接积分控制量更稳健
    StateVector x_next = mpc_wrapper_->get_next_state();

    double vx_cmd_world = x_next(kVelX);
    double vy_cmd_world = x_next(kVelY);
    double omega_cmd = x_next(kOmega);

    // 【工业级改造】更新内部指令记忆，供下一帧 MPC
    // 的起推起点使用（蓄力机制的核心）
    last_cmd_vx_world = vx_cmd_world;
    last_cmd_vy_world = vy_cmd_world;
    last_cmd_omega = omega_cmd;

    // 关键坐标系转换: 将世界坐标系速度转换为机器人坐标系速度
    // Nav2 的 cmd_vel 需要机器人坐标系下的速度
    double vx_cmd_body, vy_cmd_body;
    worldToBody(vx_cmd_world, vy_cmd_world, yaw, vx_cmd_body, vy_cmd_body);

    // 限幅
    double vel_norm = std::hypot(vx_cmd_body, vy_cmd_body);
    if (vel_norm > max_vel_) {
        double scale = max_vel_ / vel_norm;
        vx_cmd_body *= scale;
        vy_cmd_body *= scale;
    }

    omega_cmd = std::clamp(omega_cmd, -max_omega_, max_omega_);

    // 设置输出
    cmd_vel.twist.linear.x = vx_cmd_body;
    cmd_vel.twist.linear.y = vy_cmd_body;
    cmd_vel.twist.angular.z = omega_cmd;

    // 步骤 7: 发布可视化信息
    publishPredictedPath(mpc_wrapper_->get_predicted_states());
    publishReferencePath(t_current);

    // RCLCPP_INFO(logger_, "[%s] 输出速度指令: vx=%.2f, vy=%.2f, omega=%.2f",
    //             plugin_name_.c_str(), cmd_vel.twist.linear.x,
    //             cmd_vel.twist.linear.y, cmd_vel.twist.angular.z);
    return cmd_vel;
}

void HeroMpcController::setPlan(const nav_msgs::msg::Path& path) {
    // 缓存全局路径（兼容 Nav2 接口）
    global_plan_ = path;

    // 注意: 本控制器不使用离散路径，而是通过话题订阅 MINCO 多项式轨迹
    RCLCPP_DEBUG(logger_,
                 "[%s] 收到全局路径 (%zu 个点)，但本控制器使用 MINCO 轨迹",
                 plugin_name_.c_str(), path.poses.size());
}

void HeroMpcController::setSpeedLimit(const double& /*speed_limit*/,
                                      const bool& /*percentage*/) {
    // TODO(jinbo): 实现速度限制功能
    RCLCPP_WARN_ONCE(logger_, "[%s] setSpeedLimit 功能尚未实现",
                     plugin_name_.c_str());
}

// MINCO 轨迹处理函数
void HeroMpcController::mincoTrajectoryCallback(
    const interfaces::msg::MincoTrajectory::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(traj_mutex_);

    // 解析轨迹段数
    size_t num_segments = msg->durations.size();

    // 验证数据完整性
    // 每段有 2 维 x 6 系数 = 12 个数据 (S=3 五次多项式)
    size_t expected_coeffs = num_segments * 2 * 6;
    if (msg->coefficients.size() != expected_coeffs) {
        RCLCPP_ERROR(logger_, "[%s] 轨迹系数数量不匹配: 期望 %zu，实际 %zu",
                     plugin_name_.c_str(), expected_coeffs,
                     msg->coefficients.size());
        traj_valid_ = false;
        return;
    }

    // 清空旧数据
    minco_segments_.clear();
    minco_segments_.reserve(num_segments);

    // 解析多项式系数
    // 系数顺序: [段0_x_c5, 段0_x_c4, 段0_x_c3, 段0_x_c2, 段0_x_c1, 段0_x_c0,
    //           段0_y_c5, 段0_y_c4, 段0_y_c3, 段0_y_c2, 段0_y_c1, 段0_y_c0,
    //           段1_x_c5, ...]
    traj_total_duration_ = 0.0;
    for (size_t i = 0; i < num_segments; ++i) {
        MincoSegment seg;
        seg.duration = msg->durations[i];

        // X 维度系数 [c5, c4, c3, c2, c1, c0]
        size_t base_idx = i * 12;
        seg.x_coeffs[0] = msg->coefficients[base_idx + 0];  // c5
        seg.x_coeffs[1] = msg->coefficients[base_idx + 1];  // c4
        seg.x_coeffs[2] = msg->coefficients[base_idx + 2];  // c3
        seg.x_coeffs[3] = msg->coefficients[base_idx + 3];  // c2
        seg.x_coeffs[4] = msg->coefficients[base_idx + 4];  // c1
        seg.x_coeffs[5] = msg->coefficients[base_idx + 5];  // c0

        // Y 维度系数 [c5, c4, c3, c2, c1, c0]
        seg.y_coeffs[0] = msg->coefficients[base_idx + 6];   // c5
        seg.y_coeffs[1] = msg->coefficients[base_idx + 7];   // c4
        seg.y_coeffs[2] = msg->coefficients[base_idx + 8];   // c3
        seg.y_coeffs[3] = msg->coefficients[base_idx + 9];   // c2
        seg.y_coeffs[4] = msg->coefficients[base_idx + 10];  // c1
        seg.y_coeffs[5] = msg->coefficients[base_idx + 11];  // c0

        minco_segments_.push_back(seg);
        traj_total_duration_ += seg.duration;
    }

    // 更新轨迹元数据
    traj_start_time_ = rclcpp::Time(msg->start_time);
    traj_id_ = msg->trajectory_id;
    traj_valid_ = true;
}

TrajectoryState HeroMpcController::sampleTrajectory(double t) const {
    TrajectoryState state{0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

    std::lock_guard<std::mutex> lock(traj_mutex_);

    if (minco_segments_.empty()) {
        return state;
    }

    // 查找所在段
    size_t seg_idx;
    double local_t_;
    if (!findSegment(t, seg_idx, local_t_)) {
        // t 超出轨迹范围，返回终点状态
        const auto& last_seg = minco_segments_.back();
        double t_end = last_seg.duration;

        state.x = evaluatePolynomial(last_seg.x_coeffs, t_end);
        state.y = evaluatePolynomial(last_seg.y_coeffs, t_end);
        state.vx = evaluatePolynomialDerivative(last_seg.x_coeffs, t_end);
        state.vy = evaluatePolynomialDerivative(last_seg.y_coeffs, t_end);
        state.ax = evaluatePolynomialSecondDerivative(last_seg.x_coeffs, t_end);
        state.ay = evaluatePolynomialSecondDerivative(last_seg.y_coeffs, t_end);

        return state;
    }

    // 在当前段采样
    const auto& seg = minco_segments_[seg_idx];

    state.x = evaluatePolynomial(seg.x_coeffs, local_t_);
    state.y = evaluatePolynomial(seg.y_coeffs, local_t_);
    state.vx = evaluatePolynomialDerivative(seg.x_coeffs, local_t_);
    state.vy = evaluatePolynomialDerivative(seg.y_coeffs, local_t_);
    state.ax = evaluatePolynomialSecondDerivative(seg.x_coeffs, local_t_);
    state.ay = evaluatePolynomialSecondDerivative(seg.y_coeffs, local_t_);
    // RCLCPP_INFO(logger_,
    //             "sampleTrajectory: local_t_ =%.2f, "
    //             "x=%.2f, y=%.2f, vx=%.2f, vy=%.2f",
    //             local_t_, state.x, state.y, state.vx, state.vy);
    return state;
}

double HeroMpcController::evaluatePolynomial(
    const std::array<double, 6>& coeffs, double t) {
    // p(t) = c5*t^5 + c4*t^4 + c3*t^3 + c2*t^2 + c1*t + c0
    // 使用 Horner 方法提高数值稳定性和效率
    // p(t) = ((((c5*t + c4)*t + c3)*t + c2)*t + c1)*t + c0
    return ((((coeffs[0] * t + coeffs[1]) * t + coeffs[2]) * t + coeffs[3]) *
                t +
            coeffs[4]) *
               t +
           coeffs[5];
}

double HeroMpcController::evaluatePolynomialDerivative(
    const std::array<double, 6>& coeffs, double t) {
    // p'(t) = 5*c5*t^4 + 4*c4*t^3 + 3*c3*t^2 + 2*c2*t + c1
    // 使用 Horner: (((5*c5*t + 4*c4)*t + 3*c3)*t + 2*c2)*t + c1
    return (((5.0 * coeffs[0] * t + 4.0 * coeffs[1]) * t + 3.0 * coeffs[2]) *
                t +
            2.0 * coeffs[3]) *
               t +
           coeffs[4];
}

double HeroMpcController::evaluatePolynomialSecondDerivative(
    const std::array<double, 6>& coeffs, double t) {
    // p''(t) = 20*c5*t^3 + 12*c4*t^2 + 6*c3*t + 2*c2
    // 使用 Horner: ((20*c5*t + 12*c4)*t + 6*c3)*t + 2*c2
    return ((20.0 * coeffs[0] * t + 12.0 * coeffs[1]) * t + 6.0 * coeffs[2]) *
               t +
           2.0 * coeffs[3];
}

bool HeroMpcController::findSegment(double t, size_t& segment_index,
                                    double& local_t) const {
    if (t < 0.0) {
        segment_index = 0;
        local_t = 0.0;
        return true;
    }

    double accumulated_time = 0.0;
    for (size_t i = 0; i < minco_segments_.size(); ++i) {
        if (t <= accumulated_time + minco_segments_[i].duration) {
            segment_index = i;
            local_t = t - accumulated_time;
            return true;
        }
        accumulated_time += minco_segments_[i].duration;
    }

    // t 超出轨迹范围
    return false;
}

// =============================================================================
// 坐标系转换函数
// =============================================================================

void HeroMpcController::bodyToWorld(double vx_body, double vy_body, double yaw,
                                    double& vx_world, double& vy_world) {
    // 旋转矩阵 R(yaw):
    // [ cos(yaw)  -sin(yaw) ]
    // [ sin(yaw)   cos(yaw) ]
    //
    // v_world = R * v_body
    double c = std::cos(yaw);
    double s = std::sin(yaw);

    vx_world = vx_body * c - vy_body * s;
    vy_world = vx_body * s + vy_body * c;
}

void HeroMpcController::worldToBody(double vx_world, double vy_world,
                                    double yaw, double& vx_body,
                                    double& vy_body) {
    // 逆旋转矩阵 R^T(yaw) = R(-yaw):
    // [  cos(yaw)  sin(yaw) ]
    // [ -sin(yaw)  cos(yaw) ]
    //
    // v_body = R^T * v_world
    double c = std::cos(yaw);
    double s = std::sin(yaw);

    vx_body = vx_world * c + vy_world * s;
    vy_body = -vx_world * s + vy_world * c;
}

double HeroMpcController::normalizeAngle(double angle) {
    while (angle > M_PI) {
        angle -= 2.0 * M_PI;
    }
    while (angle < -M_PI) {
        angle += 2.0 * M_PI;
    }
    return angle;
}

// =============================================================================
// 可视化函数
// =============================================================================

void HeroMpcController::publishPredictedPath(
    const std::vector<StateVector>& predicted_states) {
    rclcpp::Time current_time = clock_->now();

    // -------------------------------------------------------------------------
    // 发布 Carrot Pose（指定预测点在 base_link 坐标系下的位置）
    // -------------------------------------------------------------------------
    if (carrot_point_index_ >= 0 &&
        static_cast<size_t>(carrot_point_index_) < predicted_states.size()) {
        const auto& carrot_state = predicted_states[carrot_point_index_];

        // 获取 map -> base_link 变换（使用 TimePointZero，不等待）
        try {
            auto transform =
                tf_->lookupTransform("base_link", "map", tf2::TimePointZero);

            // 构造 map 坐标系下的点
            geometry_msgs::msg::PoseStamped carrot_in_map;
            carrot_in_map.header.frame_id = "map";
            carrot_in_map.header.stamp = current_time;
            carrot_in_map.pose.position.x = carrot_state(kPosX);
            carrot_in_map.pose.position.y = carrot_state(kPosY);
            carrot_in_map.pose.position.z = 0.0;
            carrot_in_map.pose.orientation.w = 1.0;

            // 转换到 base_link 坐标系
            geometry_msgs::msg::PoseStamped carrot_in_base;
            tf2::doTransform(carrot_in_map, carrot_in_base, transform);
            carrot_in_base.header.frame_id = "base_link";
            carrot_in_base.header.stamp = current_time;

            // 如果 carrot_pose 距离 base_link 原点大于 15cm (0.15m)
            // 才发布，避免导航过冲时产生原地掉头
            double dist_to_base = std::hypot(carrot_in_base.pose.position.x,
                                             carrot_in_base.pose.position.y);
            // if (dist_to_base > 0.15) {
            interfaces::msg::NavOutput nav_output_msg;
            nav_output_msg.header = carrot_in_base.header;
            nav_output_msg.mode = 0; // 0: normal navigation
            nav_output_msg.point = carrot_in_base;
            carrot_pose_pub_->publish(nav_output_msg);
            // }
        } catch (const tf2::TransformException& ex) {
            RCLCPP_WARN_THROTTLE(logger_, *clock_, 1000,
                                 "[%s] 无法获取 map->base_link 变换: %s",
                                 plugin_name_.c_str(), ex.what());
        }
    }

    // -------------------------------------------------------------------------
    // 发布预测轨迹可视化 Marker
    // -------------------------------------------------------------------------
    if (predicted_path_pub_->get_subscription_count() == 0) {
        return;
    }

    visualization_msgs::msg::MarkerArray marker_array;

    // 1. 定义删除标记（用于清理可能存在的旧标记）
    visualization_msgs::msg::Marker delete_marker;
    delete_marker.action = visualization_msgs::msg::Marker::DELETEALL;
    // marker_array.markers.push_back(delete_marker); // Nav2 Lifecycle
    // Publisher 可能不支持混合 Action，通常直接覆盖即可

    // 2. 遍历预测状态生成球体 Marker
    for (size_t i = 0; i < predicted_states.size(); ++i) {
        const auto& state = predicted_states[i];

        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = "map";  // 或 use costmap frame
        marker.header.stamp = current_time;
        marker.ns = "predicted_trajectory";
        marker.id = static_cast<int>(i);
        marker.type = visualization_msgs::msg::Marker::SPHERE;
        marker.action = visualization_msgs::msg::Marker::ADD;

        // 设置位置
        marker.pose.position.x = state(kPosX);
        marker.pose.position.y = state(kPosY);
        marker.pose.position.z = 0.0;

        // 方向默认即可，球体不敏感
        marker.pose.orientation.w = 1.0;

        // 设置尺寸 (直径 m)
        // 可以根据时间 i 渐变大小，或者固定大小
        double scale = 0.1;
        marker.scale.x = scale;
        marker.scale.y = scale;
        marker.scale.z = scale;

        // 设置颜色 (RGBA)
        // 例如：绿色，透明度随时间递减
        marker.color.r = 0.0;
        marker.color.g = 1.0;
        marker.color.b = 0.0;
        marker.color.a = 1.0;

        marker.lifetime = rclcpp::Duration::from_seconds(
            0.1);  // 生命周期短一点，自动消失防止重影

        marker_array.markers.push_back(marker);
    }

    predicted_path_pub_->publish(marker_array);
}

void HeroMpcController::publishReferencePath(double t_start) {
    if (reference_path_pub_->get_subscription_count() == 0) {
        return;
    }

    visualization_msgs::msg::MarkerArray marker_array;
    rclcpp::Time current_time = clock_->now();

    const int N = MpcWrapper::get_horizon_steps();
    const double dt = MpcWrapper::get_timestep();

    for (int k = 0; k <= N; ++k) {
        double t_k = t_start + k * dt;
        TrajectoryState ref = sampleTrajectory(t_k);

        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = "map";
        marker.header.stamp = current_time;
        marker.ns = "reference_trajectory";
        marker.id = static_cast<int>(k);
        marker.type = visualization_msgs::msg::Marker::SPHERE;
        marker.action = visualization_msgs::msg::Marker::ADD;

        // 设置位置
        marker.pose.position.x = ref.x;
        marker.pose.position.y = ref.y;
        marker.pose.position.z = 0.0;

        // 方向默认即可
        marker.pose.orientation.w = 1.0;

        // 设置尺寸 (直径 m)
        double scale = 0.08;
        marker.scale.x = scale;
        marker.scale.y = scale;
        marker.scale.z = scale;

        // 设置颜色 (RGBA) - 红色
        marker.color.r = 1.0;
        marker.color.g = 0.0;
        marker.color.b = 0.0;
        marker.color.a = 1.0;

        marker.lifetime = rclcpp::Duration::from_seconds(0.1);

        marker_array.markers.push_back(marker);
    }

    reference_path_pub_->publish(marker_array);
}

}  // namespace hero_mpc_controller

// 注册 Nav2 插件
PLUGINLIB_EXPORT_CLASS(hero_mpc_controller::HeroMpcController,
                       nav2_core::Controller)
