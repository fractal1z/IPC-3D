#include "planner.h"
#include "../include/polytope/emvp.hpp"

#define BACKWARD_HAS_DW 1
#include "backward.hpp"
namespace backward{
    backward::SignalHandling sh;
}

void PlannerClass::TimerCallback(const ros::TimerEvent &)
{
    if (!has_odom_flag_) return;// 如果没有获取到里程计数据，则退出回调函数
    if (mode_ == Manual) {// 如果处于手动模式，则发布全零姿态控制指令，以保持当前姿态
        BodyrateCtrlPub(Eigen::Vector3d(0,0,0), 0.05, ros::Time::now());
        return;
    }

    timer_mutex_.lock();// 加锁以防止其他线程同时访问定时器回调函数中的变量
    
    ros::Time t_start = ros::Time::now();// 记录回调函数开始执行的时间
    ros::Time t_0 = odom_time_;

    if (mode_ == Command) {// 如果处于指令模式，则进行路径规划和控制
        if (new_goal_flag_) {// 如果有新目标点标志被设置，则重新规划路径
            new_goal_flag_ = false;
            replan_flag_ = false;
            PathReplan(true);
        }// 如果没有新目标点标志被设置，但需要重新规划路径，则进行路径重新规划
        if (new_goal_flag_ == false && replan_flag_) {
            replan_flag_ = false;
            PathReplan(false);
        }
        log_times_[1] = (ros::Time::now() - t_start).toSec() * 1000.0; // 记录路径规划时间

        ros::Time sfc_start = ros::Time::now();// 开始跟踪路径
        if (follow_path_.size() > 0) { // follow A* path
            double min_dis = 10000.0;// 寻找最近的路径点
            for (int i = 0; i < follow_path_.size(); i++) { // find the nearest point
                double dis = (odom_p_ - follow_path_[i]).norm();
                if (dis < min_dis) {
                    min_dis = dis;
                    astar_index_ = i;
                }
            }

            int goal_in_sfc = astar_index_;
            // 根据当前位置与路径点的关系进行不同的处理
            // 此处代码用于计算最优路径以及设置MPC控制器的目标状态
            // 以下是复杂的路径跟随和MPC设置逻辑，包括生成安全飞行走廊（Safe Flight Corridor, SFC）、设置MPC的目标点等
            // 一些与MPC控制相关的复杂逻辑，包括优化控制命令的计算


            // 记录路径跟踪时间
            if (follow_path_.size() - astar_index_ <= ref_dis_) { // near the end, find the sfc at current point
            // 如果接近终点，则将目标点设置为路径的末端
                goal_in_sfc = follow_path_.size();
                Eigen::Matrix<double, Eigen::Dynamic, 4> planes;
                GenerateAPolytope(odom_p_, odom_p_, planes, 0);// 为当前位置生成一个多边形（polytope），代表动态空间约束
                // 将这个多边形应用到MPC（Model Predictive Control）控制器的预测范围内的每一个时间点
                for (int i = 0; i < mpc_->MPC_HORIZON; i++) {
                    mpc_->SetFSC(planes, i);
                }
            } else { // find the biggest and the longest sfc
            // 如果不接近终点，寻找最大且最长的安全飞行走廊（SFC）
                Eigen::Matrix<double, Eigen::Dynamic, 4> planes, last_planes;
                GenerateAPolytope(follow_path_[astar_index_], follow_path_[astar_index_], planes, 0); // 为A*路径上当前索引点生成一个多边形，表示当前的动态空间约束
                last_planes = planes;
                int init_num = 0;
                // 检查当前位置是否在初始SFC中
                if (mpc_->IsInFSC(odom_p_, planes) == false) { // if odom is not in initial sfc
                // 如果不在，则计算需要初始化的SFC数量
                    init_num = (odom_p_ - follow_path_[astar_index_]).norm() / path_dis_ / ref_dis_;
                    ROS_INFO("\033[35m UAV is out sfc! init num is %d \033[0m", init_num);
                }
                int first_id = astar_index_, mpc_goal_index = 0;
                  // 寻找第一个SFC内的最远点
                for (int i = first_id+1; i < follow_path_.size(); i++) { // find last pos in first sfc
                    if (mpc_->IsInFSC(follow_path_[i], planes)) {
                        first_id = i;
                        goal_in_sfc = i;
                    } else {
                        first_id -= 0.2 / path_dis_;
                        if (first_id < astar_index_) first_id = astar_index_;
                        break;
                    }
                }
                mpc_goal_index = (first_id - astar_index_) / ref_dis_ + init_num + 1;
                assert(first_id >= astar_index_);
                // 为找到的SFC设置MPC目标
                for (int i = init_num; i <= mpc_goal_index && i < mpc_->MPC_HORIZON; i++) {
                    mpc_->SetFSC(planes, i);
                }
                int end_index = first_id + (mpc_->MPC_HORIZON-mpc_goal_index) * ref_dis_;
                if (end_index >= follow_path_.size()) end_index = follow_path_.size() - 1;
                // 扩展SFC以包含更远的路径点
                for (int i = end_index; i >= first_id && mpc_goal_index < mpc_->MPC_HORIZON - 1; i--) {
                    if (local_astar_->CheckPoint(follow_path_[i]) == false) continue;
                    if (local_astar_->CheckLineObstacleFree(follow_path_[first_id], follow_path_[i]) == false) continue;
                    i = i - 0.2 / path_dis_;
                    if (i <= first_id) break;
                    Eigen::Matrix<double, Eigen::Dynamic, 4> long_planes;
                    GenerateAPolytope(follow_path_[first_id], follow_path_[i], long_planes, 1);
                    if (long_planes.rows() > 0) {
                        for (int j = mpc_goal_index+1; j < mpc_->MPC_HORIZON; j++) {
                            mpc_->SetFSC(long_planes, j);
                        }
                        mpc_goal_index = mpc_->MPC_HORIZON;
                        for (int k = first_id; k < follow_path_.size(); k++) {
                            if (mpc_->IsInFSC(follow_path_[k], long_planes)) goal_in_sfc = k;
                            else break;
                        }
                        // ROS_INFO("\033[43;37m long sfc : %d \033[0m", goal_in_sfc);
                        break;
                    }
                }
                for (int i = mpc_goal_index+1; i < mpc_->MPC_HORIZON; i++) {
                    if (last_planes.rows() > 0) mpc_->SetFSC(last_planes, i);
                }
            }

            // MPC set goal
            mpc_goals_.clear();
            Eigen::Vector3d last_p_ref;
            for (int i = 0; i < mpc_->MPC_HORIZON; i++) {
                int index = astar_index_ + i * ref_dis_;
                if (index >= goal_in_sfc) index = goal_in_sfc;
                if (index >= follow_path_.size()) index = follow_path_.size() - 1;
                Eigen::Vector3d v_r(0, 0, 0);
                if (i == 0) v_r = (follow_path_[index] - odom_p_) / mpc_->MPC_STEP;
                else if (i == mpc_->MPC_HORIZON) v_r.setZero();
                else v_r = (follow_path_[index] - last_p_ref) / mpc_->MPC_STEP;
                last_p_ref = follow_path_[index];
                
                mpc_->SetGoal(follow_path_[index], v_r, Eigen::Vector3d::Zero(), i);
                // std::cout << "mpc goal " << i << ": " << follow_path_[index].transpose() << " v: " << v_r.transpose() << std::endl;
                mpc_goals_.push_back(follow_path_[index]);
            }
            // 发布A*规划的路径，用于可视化或调试
            AstarPublish(mpc_goals_, 3, 0.1);

            // yaw control// // 控制yaw角，使无人机朝向路径的最终方向
            if (astar_index_ < follow_path_.size() - 0.3 / path_dis_) {
                yaw_r_ = std::atan2(follow_path_.back().y()-odom_p_.y(), follow_path_.back().x()-odom_p_.x());
                // yaw_r_ = std::atan2(follow_path_[astar_index_+ref_dis_].y()-odom_p_.y(), follow_path_[astar_index_+ref_dis_].x()-odom_p_.x());
            }
        } else { // stay at initial position
        // 如果没有有效的路径，则保持当前位置不变
            yaw_r_ = 0.0;
            for (int i = 0; i < mpc_->MPC_HORIZON; i++) {
                mpc_->SetGoal(goal_p_, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(), i);
            }
        }// 记录完成路径跟随计算所需的时间
        log_times_[2] = (ros::Time::now() - sfc_start).toSec() * 1000.0;
    }

    if (mode_ == Hover) { // 如果模式设置为悬停，设置MPC的目标保持当前位置
        for (int i = 0; i < mpc_->MPC_HORIZON; i++) {
            mpc_->SetGoal(goal_p_, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(), i);
        }
    }
    
    // calculate model predict control algorithm
    // 开始计算模型预测控制算法的时间
    ros::Time mpc_start = ros::Time::now();
    // 设置当前无人机的状态（位置、速度、加速度）
    mpc_->SetStatus(odom_p_, odom_v_, odom_a_);
    bool success_flag = mpc_->Run();// 运行MPC算法，计算最优控制指令
    log_times_[3] = (ros::Time::now() - mpc_start).toSec() * 1000.0;// 记录MPC计算所花费的时间

    // 定义用于存储最优解的变量
    Eigen::Vector3d u_optimal, p_optimal, v_optimal, a_optimal, u_predict;
    Eigen::MatrixXd A1, B1;
    Eigen::VectorXd x_optimal = mpc_->X_0_;
    if (success_flag) {// 如果MPC成功找到最优解
        last_mpc_time_ = ros::Time::now();
        // 根据控制延迟和MPC步长计算需要应用的控制指令
        for (int i = 0; i <= ctrl_delay_/mpc_->MPC_STEP; i++) {
            mpc_->GetOptimCmd(u_optimal, i);// 获取最优控制指令
            mpc_->SystemModel(A1, B1, mpc_->MPC_STEP);// 获取系统模型
            x_optimal = A1 * x_optimal + B1 * u_optimal;// 更新状态
        }
        mpc_ctrl_index_ = ctrl_delay_/mpc_->MPC_STEP;
        // 从状态向量中提取最优位置、速度和加速度
        p_optimal << x_optimal(0,0), x_optimal(1,0), x_optimal(2,0);
        v_optimal << x_optimal(3,0), x_optimal(4,0), x_optimal(5,0);
        a_optimal << x_optimal(6,0), x_optimal(7,0), x_optimal(8,0);
        // 根据仿真标志，发布相应的控制命令
        if (!perfect_simu_flag_) CmdPublish(odom_p_, v_optimal, a_optimal, u_optimal);
        else CmdPublish(p_optimal, v_optimal, a_optimal, u_optimal);

        // 生成并发布MPC路径
        std::vector<Eigen::Vector3d> path;
        x_optimal = mpc_->X_0_;
        for (int i = 0; i < mpc_->MPC_HORIZON; i++) {
            mpc_->GetOptimCmd(u_predict, i);
            mpc_->SystemModel(A1, B1, mpc_->MPC_STEP);
            x_optimal = A1 * x_optimal + B1 * u_predict;
            path.push_back(Eigen::Vector3d(x_optimal(0,0), x_optimal(1,0), x_optimal(2,0)));
        }
        MPCPathPublish(path);
    } else { // 如果MPC未成功，处理失败后的情况
        double delta_t = (ros::Time::now()-last_mpc_time_).toSec();
        if (delta_t >= mpc_->MPC_STEP) {
            mpc_ctrl_index_ += delta_t / mpc_->MPC_STEP;
            last_mpc_time_ = ros::Time::now();
        }
        mpc_->GetOptimCmd(u_optimal, mpc_ctrl_index_);
        // std::cout << "index: " << mpc_ctrl_index_ << " u_optimal:" << u_optimal.transpose() << std::endl;
        // std::cout << "x_0: " << mpc_->X_0_.transpose() << std::endl << std::endl;
        mpc_->SystemModel(A1, B1, mpc_->MPC_STEP);
        x_optimal = A1 * mpc_->X_0_ + B1 * u_optimal;
        p_optimal << x_optimal(0,0), x_optimal(1,0), x_optimal(2,0);
        v_optimal << x_optimal(3,0), x_optimal(4,0), x_optimal(5,0);
        a_optimal << x_optimal(6,0), x_optimal(7,0), x_optimal(8,0);
        if (!perfect_simu_flag_) CmdPublish(odom_p_, v_optimal, a_optimal, u_optimal);
        else CmdPublish(p_optimal, v_optimal, a_optimal, u_optimal);
    }
    
    ros::Time df_start = ros::Time::now();
    estimateThrustModel(imu_a_);
    a_optimal = a_optimal + Gravity_;
    ComputeThrust(a_optimal);
    ConvertCommand(a_optimal, u_optimal);
    BodyrateCtrlPub(rate_, thrust_, ros::Time::now());
    std::cout<<"\n"<<"rate"<<rate_<<"\n"<<"thrust"<<thrust_;
    timed_thrust_.push(std::pair<ros::Time, double>(ros::Time::now(), thrust_));
    while (timed_thrust_.size() > 100) {
        timed_thrust_.pop();
    }
    log_times_[4] = (ros::Time::now() - df_start).toSec() * 1000.0;

    // log_times_[0] = (ros::Time::now() - t_start).toSec() * 1000.0;
    WriteLogTime();

    timer_mutex_.unlock();
}
//这段代码是无人机路径规划类中的一个方法，用于生成一个多面体（Polytope），
//这个多面体定义了无人机可以安全飞行的空间区域。这个多面体是通过考虑环境中的障碍物来生成的，以确保无人机在飞行过程中避开这些障碍物
void PlannerClass::GenerateAPolytope(Eigen::Vector3d p1, Eigen::Vector3d p2, Eigen::Matrix<double, Eigen::Dynamic, 4>& planes, uint8_t index)
{
    planes.resize(0, 4);// 初始化planes矩阵，用于存储多面体的平面方程
    ros::Time start_t = ros::Time::now();
    // 定义包围盒的最大和最小点
    Eigen::Vector3d box_max(10, 10, 10), box_min(-10, -10, -10);
    // 初始化用于定义多面体边界的矩阵
    Eigen::Matrix<double, 6, 4> bd;// 设置边界平面的法向量和常数项，构建一个围绕点p1的立方体
    bd.setZero();
    bd(0, 0) = 1.0;
    bd(1, 0) = -1.0;
    bd(2, 1) = 1.0;
    bd(3, 1) = -1.0;
    bd(4, 2) = 1.0;
    bd(5, 2) = -1.0;
    bd(0, 3) = -p1.x()-box_max.x();
    bd(1, 3) =  p1.x()+box_min.x();
    bd(2, 3) = -p1.y()-box_max.y();
    bd(3, 3) =  p1.y()+box_min.y();
    bd(4, 3) = -p1.z()-box_max.z();
    bd(5, 3) =  p1.z()+box_min.z();

    Polytope p;// 初始化多面体对象
    if (local_pc_.empty()) { // 障碍物点云为空，直接返回一个方块
        planes.resize(6, 4); // Ax + By + Cz + D = 0
        planes.row(0) <<  1,  0,  0, -p1.x()-box_max.x();
        planes.row(1) <<  0,  1,  0, -p1.y()-box_max.y();
        planes.row(2) <<  0,  0,  1, -p1.z()-box_max.z();
        planes.row(3) << -1,  0,  0,  p1.x()+box_min.x();
        planes.row(4) <<  0, -1,  0,  p1.y()+box_min.y();
        planes.row(5) <<  0,  0, -1,  p1.z()+box_min.z();
        return ; 
    }
     // 将本地点云数据转换为Eigen矩阵格式
    Eigen::Map<const Eigen::Matrix<double, 3, -1, Eigen::ColMajor>> pp(local_pc_[0].data(), 3, local_pc_.size());

    bool success = emvp::emvp(bd, pp, p1, p2, p, sfc_dis_, false, 1);// 使用EMVP算法生成多面体，其中包括障碍物的考虑
    if (success) {// 如果成功生成了多面体
        if (index == 0) p.Visualize(sfc_pub_, "emvp", true);
        p.Visualize(sfc_pub_, "emvp", false);
        planes = p.GetPlanes();
    } else {
        p.Reset();
    }
}

void PlannerClass::PathReplan(bool extend)
{
    // 清除之前的路径数据
    astar_path_.clear();
    waypoints_.clear();
    follow_path_.clear();

    Eigen::Vector3d start_p, end_p;
    start_p = odom_p_;// 设定起始点为当前位置（里程计提供的位置）
    // start_p = odom_p_ + odom_v_ * 0.1;
    if (extend) {// 根据extend标志扩展搜索区域中心
        local_astar_->SetCenter(Eigen::Vector3d(odom_p_.x(), odom_p_.y(), odom_p_.z()));
    } else {
        // ROS_WARN("[MPC FSM]: Replan!");
    }
    // 设置本地A*算法的障碍物信息
    local_astar_->setObsVector(local_pc_, expand_dyn_);// 设置本地A*算法的障碍物信息

   // 检查目标点是否超出地图范围，并相应调整
    bool add_goal_flag = false; // 标记是否需要在路径末尾添加原始目标点
    end_p = goal_p_;// 设定终点为目标点
    double delta_x = goal_p_.x() - odom_p_.x();
    double delta_y = goal_p_.y() - odom_p_.y();
    double delta_z = goal_p_.z() - odom_p_.z();

    if (std::fabs(delta_x) > map_upp_.x() || std::fabs(delta_y) > map_upp_.y()) {
        add_goal_flag = true;
        // 调整终点使其不超出地图界限
        // 计算终点调整后的坐标，保持其在地图范围内
        if (std::fabs(delta_x) > std::fabs(delta_y)) {
            end_p.x() = odom_p_.x() + (delta_x/std::fabs(delta_x)) * (map_upp_.x() - resolution_);
            end_p.y() = odom_p_.y() + ((map_upp_.x() - resolution_)/std::fabs(delta_x)) * delta_y;
        } else {
            end_p.x() = odom_p_.x() + ((map_upp_.y() - resolution_)/std::fabs(delta_y)) * delta_x;
            end_p.y() = odom_p_.y() + (delta_y/std::fabs(delta_y)) * (map_upp_.y() - resolution_);
        }
    }
    if (std::fabs(delta_z) > map_upp_.z()) {
        add_goal_flag = true;
        // 调整终点使其不超出地图界限
        // 计算终点调整后的坐标，保持其在地图范围内
    
            end_p.z() = odom_p_.z() + (delta_z/std::fabs(delta_z)) * (map_upp_.z() - resolution_);
         
    }

    // start_p.z() = end_p.z(); // only for 2d path searching
    // std::cout << "odom: " << odom_p_.transpose() << " end_p: " << end_p.transpose() << std::endl;
    // 检查起点和终点是否在障碍物内，若是则进行调整
    int point_num = 8;
    double r = expand_fix_ / 2;
    if (local_astar_->CheckStartEnd(start_p) == false) {// 检查起点，若在障碍物内则进行微调
        ROS_INFO("\033[41;37m start point in obstacle \033[0m");
        while (true) {
            bool flag = false;
            for(int i = 0; i < point_num; i++) {
                Eigen::Vector3d pt;
                pt << start_p.x() + r*sin(M_PI*2*i/point_num), start_p.y() + r*cos(M_PI*2*i/point_num), start_p.z();
                double dis_min = 10000.0;
                double dis = (odom_p_ - pt).norm();
                if(local_astar_->CheckStartEnd(pt) == true && dis < dis_min) {
                    dis_min = dis;
                    start_p = pt;
                    flag = true;
                }
            }
            if (flag) {
                ROS_INFO("\033[1;32m Change start goal! %f %f %f\033[0m", start_p.x(), start_p.y(), start_p.z());
                break;
            }
            r += expand_fix_ / 2;
        }
    }
    r = expand_fix_ / 2;
    if (local_astar_->CheckStartEnd(end_p) == false) {// 检查终点，若在障碍物内则进行微调
        ROS_INFO("\033[41;37m end point in obstacle \033[0m");
        while (true) {
            bool flag = false;
            for(int i = 0; i < point_num; i++) {
                Eigen::Vector3d pt;
                pt << end_p.x() + r*sin(M_PI*2*i/point_num), end_p.y() + r*cos(M_PI*2*i/point_num), end_p.z();
                double dis_min = 10000.0;
                double dis = (odom_p_ - pt).norm();
                if(local_astar_->CheckStartEnd(pt) == true && dis < dis_min) {
                    dis_min = dis;
                    end_p = pt;
                    flag = true;
                }
            }
            if (flag) {
                ROS_INFO("\033[1;32m Change end goal! %f %f %f\033[0m", end_p.x(), end_p.y(), end_p.z());
                break;
            }
            r += expand_fix_ / 2;
        }
    }
    // 使用A*算法搜索从起点到终点的路径
    bool search_flag = local_astar_->SearchPath(start_p, end_p);
    if (search_flag) {
        local_astar_->GetPath(astar_path_);
        AstarPublish(astar_path_, 0, 0.1);// 若找到路径，则获取并发布该路径

        local_astar_->FloydHandle(astar_path_, waypoints_);   // 对A*路径进行弗洛伊德路径平滑处理，并获取路径关键点
        // 若需要，将原始目标点加入到路径末尾
        // waypoints_.insert(waypoints_.begin(), odom_p_);
        if (add_goal_flag && local_astar_->CheckPoint(goal_p_)) waypoints_.push_back(goal_p_);
        AstarPublish(waypoints_, 1, 0.1);
        // 根据关键点生成最终的跟随路径
        for (int i = 0; i < waypoints_.size()-1; i++) {
            Eigen::Vector3d vector = waypoints_[i+1] - waypoints_[i];
            int num = vector.norm() / path_dis_; // Insert a point every 0.1m (adjust)
            for (int j = 0; j < num; j++) {
                Eigen::Vector3d pt = waypoints_[i] + vector * j / num;
                follow_path_.push_back(pt);
            }
        }
        follow_path_.push_back(waypoints_.back());
        AstarPublish(follow_path_, 2, path_dis_);
    } else {
        ROS_INFO("\033[41;37m No path! Stay at current point! \033[0m");
        follow_path_.push_back(odom_p_);
    }

    geometry_msgs::PoseStamped msg;
    msg.header.frame_id = "world";
    msg.header.stamp = ros::Time::now();
    msg.pose.position.x = follow_path_.back().x();
    msg.pose.position.y = follow_path_.back().y();
    msg.pose.position.z = follow_path_.back().z();    
    goal_pub_.publish(msg);

    ros::Time now = ros::Time::now();
    local_astar_->Reset(); // reset astar data at the end
    // std::cout << "astar reset time is: " << (ros::Time::now() - now).toSec()*1000 << " ms. " << std::endl;
}

void PlannerClass::GoalCallback(const geometry_msgs::PoseStampedConstPtr& msg)
{
    goal_mutex_.lock();
    // 创建静态变量，用于存储上一个目标点的位置
    static Eigen::Vector3d last_goal;
    // 从收到的消息中提取新的目标点位置
    Eigen::Vector3d new_goal(msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);
    if (last_goal != new_goal && mode_ == Command) {
        // goal_p_ << msg->pose.position.x, msg->pose.position.y, goal_p_.z(); // 2d path searching
        // 更新目标点位置
        goal_p_ << msg->pose.position.x, msg->pose.position.y, msg->pose.position.z;
        // 对目标点的 z 坐标进行限制，确保不超出地图的上限和下限
        // if (goal_p_.z() > map_upp_.z() - 0.5) goal_p_.z() = map_upp_.z() - 0.5;
        // if (goal_p_.z() < 1.5) goal_p_.z() = 1.5;
        new_goal_flag_ = true;
    }
    // std::cout <<goal_p_.x() << " "<< goal_p_.y() << " "<<goal_p_.z();
    // 更新上一个目标点为当前目标点
    last_goal = new_goal;

    goal_mutex_.unlock();
}
// 处理里程计消息的回调函数
void PlannerClass::OdomCallback(const nav_msgs::OdometryConstPtr& msg)
{   
  
    odom_mutex_.lock();// 锁定互斥锁，确保线程安全
    // 更新里程计时间戳
    odom_time_ = msg->header.stamp;
    // 更新无人机的位置
    odom_p_ << msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z;
    // 更新无人机的速度
    odom_v_ << msg->twist.twist.linear.x, msg->twist.twist.linear.y, msg->twist.twist.linear.z; 
    // 更新无人机的姿态四元数
    odom_q_ = Eigen::Quaterniond(msg->pose.pose.orientation.w, msg->pose.pose.orientation.x, 
                                 msg->pose.pose.orientation.y, msg->pose.pose.orientation.z);
    // 根据当前姿态、推力和推力转加速度系数计算加速度，并减去重力影响
    odom_a_ = odom_q_ * Eigen::Vector3d(0, 0, 1) * (thrust_ * thr2acc_) - Gravity_;
    // 如果是完美仿真模式，则将速度置零（可选：也可以将加速度置零）
    if (perfect_simu_flag_) {
        odom_v_.setZero();
        // odom_a_.setZero();
    }
    // 从四元数姿态中计算出偏航角
    yaw_ = tf::getYaw(msg->pose.pose.orientation);
    // 设置标志位，表示已接收到里程计数据
    has_odom_flag_ = true;
    odom_mutex_.unlock();// 解锁互斥锁
}

void PlannerClass::IMUCallback(const sensor_msgs::ImuConstPtr& msg)
{
    imu_mutex_.lock();
    // 从收到的 IMU 消息中提取线性加速度信息，并存储到 imu_a_ 中（在机体坐标系中）
    imu_a_ << msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z; // body frame
    // 将 IMU 传感器测得的加速度数据转换到世界坐标系中
    Eigen::Matrix3d Rotate = odom_q_.toRotationMatrix().inverse();
    imu_a_ = Rotate * imu_a_;
    imu_mutex_.unlock();
}

void PlannerClass::LocalPcCallback(const sensor_msgs::PointCloud2ConstPtr& msg)
{
    local_pc_mutex_.lock();// 锁定互斥量，以确保线程安全

    ros::Time now = ros::Time::now();// 获取当前时间戳
    // 清空本地点云数据
    local_pc_.clear();
    // 将接收到的 ROS 消息转换为 PCL 格式的点云
    pcl::PointCloud<pcl::PointXYZ> cloud;
    pcl::fromROSMsg(*msg, cloud);

    // 将当前点云数据加入队列中，并限制队列大小
    vec_cloud_.push_back(cloud);
    if (vec_cloud_.size() > 10) vec_cloud_.pop_front();
    // 将所有点云数据合并为一个点云
    static_cloud_.reset(new pcl::PointCloud<pcl::PointXYZ>());
    for (int i = 0; i < vec_cloud_.size(); i++) *static_cloud_ += vec_cloud_[i];
    
    // 使用 CropBox 过滤器删除不需要的点云
    pcl::CropBox<pcl::PointXYZ> cb; // CropBox filter (delete unuseful points)
    cb.setMin(Eigen::Vector4f(odom_p_.x() - (map_upp_.x()-0.5), odom_p_.y() - (map_upp_.y()-0.5), odom_p_.z() - (map_upp_.z()-0.5), 1.0));
    cb.setMax(Eigen::Vector4f(odom_p_.x() + (map_upp_.x()-0.5), odom_p_.y() + (map_upp_.y()-0.5), odom_p_.z() + (map_upp_.z()-0.5), 1.0));
    cb.setInputCloud(static_cloud_);
    cb.filter(*static_cloud_);
    // 使用 VoxelGrid 滤波器对静态地图进行下采样
    pcl::VoxelGrid<pcl::PointXYZ> vf;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cur_cloud_ds(new pcl::PointCloud<pcl::PointXYZ>());//存储下采样后的点云数据。
    Eigen::Vector3f pos = odom_p_.cast<float>();
    vf.setLeafSize(0.2, 0.2, 0.2);
    vf.setInputCloud(static_cloud_);
    vf.filter(static_map_);
    // 将静态地图中的点转换为 Eigen::Vector3d 类型并存储到 local_pc_ 中
    for(auto &point: static_map_.points) {
        local_pc_.push_back(Eigen::Vector3d(point.x, point.y, point.z));
    }

    // update astar map
     // 更新 A* 算法的地图信息，并检查路径是否被障碍物占据，从而确定是否需要重新规划路径
    static int obs_count = 0;
    local_astar_->SetCenter(Eigen::Vector3d(odom_p_.x(), odom_p_.y(), odom_p_.z()));
    local_astar_->setObsVector(local_pc_, expand_fix_);
    std::vector<Eigen::Vector3d> remain_path;
    remain_path.insert(remain_path.begin(), follow_path_.begin()+astar_index_, follow_path_.end());
    if (local_astar_->CheckPathFree(remain_path) == false) replan_flag_ = true;
    // bool flag = local_astar_->CheckPathFree(follow_path_);     // check path is free or not
    // if (flag == false) obs_count++;
    // else obs_count = 0;
    // if (obs_count >= 2) {
    //     obs_count = 0;
    //     replan_flag_ = true;
    // }
    
    // local_astar_->GetOccupyPcl(cloud);
    // cloud.width = cloud.points.size();
    // cloud.height = 1;
    // cloud.is_dense = true;
    // sensor_msgs::PointCloud2 map_msg;
    // pcl::toROSMsg(cloud, map_msg);
    // map_msg.header.frame_id = "world";
    // gird_map_pub_.publish(map_msg);
    
    // 计算回调函数执行的时间，并记录到日志中
    log_times_[0] = (ros::Time::now() - now).toSec() * 1000.0;

    local_pc_mutex_.unlock();
}

void PlannerClass::RCInCallback(const mavros_msgs::RCInConstPtr& msg)
{
    rc_mutex_.lock();

    if (simu_flag_) {
        mode_ = Command;
    } else {
        static UAVMode_e mode_last = Manual;
        static uint16_t takeoff_ch_last = 0;
        if (msg->channels[4] > 1650 && msg->channels[4] < 1800 && mode_last != Command) {
            mode_ = Command;
            ROS_INFO("\033[1;32m[MPC FSM]: Manual or Hover --> Command.\033[0m");
        }
        if (mode_ == Command && msg->channels[4] > 1850) {
            mode_ = Hover;
            goal_p_ = odom_p_;
            ROS_INFO("\033[32m[MPC FSM]: Command --> Hover. Pos is: %f %f %f\033[32m", goal_p_.x(), goal_p_.y(), goal_p_.z());
        }
        // 如果当前模式为 Hover，且上一时刻的起飞通道值小于 1500，当前时刻的起飞通道值大于 1500，那么将飞行器的目标高度设置为 0，表示降落。
        if (mode_ == Hover && takeoff_ch_last < 1500 && msg->channels[10] > 1500) {
            goal_p_.z() = 0.0;
            ROS_WARN("[MPC FSM]: Land! Pos is: %f %f %f", goal_p_.x(), goal_p_.y(), goal_p_.z());
        }
        
        takeoff_ch_last = msg->channels[10];//更新起飞通道值为当前值。
        mode_last = mode_;//更新上一时刻的模式为当前模式。
    }

    rc_mutex_.unlock();
}


// nav_msgs::Odometry NedToEnu(const nav_msgs::Odometry& ned_odom) {
//     nav_msgs::Odometry enu_odom = ned_odom;

//     // Swap X and Y
//     enu_odom.pose.pose.position.x = ned_odom.pose.pose.position.y;
//     enu_odom.pose.pose.position.y = ned_odom.pose.pose.position.x;

//     // Invert Z
//     enu_odom.pose.pose.position.z = -ned_odom.pose.pose.position.z;

//     // Swap linear velocities
//     enu_odom.twist.twist.linear.x = ned_odom.twist.twist.linear.y;
//     enu_odom.twist.twist.linear.y = ned_odom.twist.twist.linear.x;

//     // Invert Z velocity
//     enu_odom.twist.twist.linear.z = -ned_odom.twist.twist.linear.z;

//     return enu_odom;
// }