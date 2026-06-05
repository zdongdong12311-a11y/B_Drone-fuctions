#!/usr/bin/env python3
"""
navigation.py - 无人机自主导航控制器 (适配 EGO-Planner)

基于 Livox Mid-360 + FAST-LIO2 + EGO-Planner
实现多航点自主导航，支持：
- 自动起飞 / 航点导航 / 安全降落
- 订阅 EGO-Planner 输出的三维轨迹并下发飞控
- 电池电压监测与低电量保护
- 连接丢失 / 位姿超时保护
- 航点文件支持注释行与环境变量配置
- ROS 参数服务器动态配置
"""
import os
import sys
import signal
import rospy
import math
from geometry_msgs.msg import PoseStamped, Twist
from mavros_msgs.msg import PositionTarget, State, BatteryStatus
from mavros_msgs.srv import CommandBool, SetMode, CommandTOL
from tf import transformations

try:
    from quadrotor_msgs.msg import PositionCommand
except ImportError:
    rospy.logwarn("未检测到 quadrotor_msgs，请确保 EGO-Planner 工作空间已 source！")
    class PositionCommand:
        pass


class NavigationController:
    """无人机自主导航控制器"""

    def __init__(self):
        # ---- ROS 参数 ----
        self.target_z = rospy.get_param('~target_z', 0.0)
        self.max_z_speed = rospy.get_param('~max_z_speed', 0.8)
        self.waypoint_xy_tol = rospy.get_param('~waypoint_xy_tol', 0.3)
        self.waypoint_z_tol = rospy.get_param('~waypoint_z_tol', 0.2)
        self.waypoint_timeout = rospy.get_param('~waypoint_timeout', 120.0)
        self.takeoff_timeout = rospy.get_param('~takeoff_timeout', 30.0)
        self.land_timeout = rospy.get_param('~land_timeout', 60.0)
        self.low_battery_threshold = rospy.get_param('~low_battery_threshold', 20.0)
        self.takeoff_height = rospy.get_param('~takeoff_height', 0.8)

        # ---- 内部状态 ----
        self.current_state = State()
        self.current_position = PoseStamped()
        self.battery = BatteryStatus()
        self.pose_received = False
        self.last_pose_time = rospy.Time(0)
        self.now_yaw = 0.0
        self.rate = rospy.Rate(20)  # 20 Hz
        self._emergency_land_triggered = False
        self._home_position = None
        self.nav_state = "IDLE"  # IDLE, TAKEOFF, NAVIGATING, HOVER, LANDING

        # EGO-Planner 轨迹状态
        self.ego_cmd = None
        self.last_ego_cmd_time = rospy.Time(0)

        # ---- 发布者 ----
        self.goal_pub = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=10)
        self.setpoint_pub = rospy.Publisher('/mavros/setpoint_raw/local', PositionTarget, queue_size=10)

        # ---- 订阅者 ----
        rospy.Subscriber('/mavros/state', State, self.state_callback)
        rospy.Subscriber('/mavros/local_position/pose', PoseStamped, self.current_position_callback)
        rospy.Subscriber('/mavros/battery', BatteryStatus, self.battery_callback)
        # 订阅 EGO-Planner 输出的轨迹指令
        rospy.Subscriber('/position_cmd', PositionCommand, self.position_cmd_callback)
        rospy.Subscriber('/drone_0_traj_server/position_cmd', PositionCommand, self.position_cmd_callback) # 兼容带有 namespace 的情况

        # ---- 服务客户端 ----
        rospy.wait_for_service('/mavros/set_mode', timeout=30)
        self.set_mode_client = rospy.ServiceProxy('/mavros/set_mode', SetMode)
        rospy.wait_for_service('/mavros/cmd/arming', timeout=30)
        self.arm_client = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)
        try:
            rospy.wait_for_service('/mavros/cmd/land', timeout=5)
            self.land_client = rospy.ServiceProxy('/mavros/cmd/land', CommandTOL)
        except rospy.ROSException:
            rospy.logwarn("LAND 服务不可用，将使用模式切换降落")
            self.land_client = None

        # ---- 航点文件路径 ----
        self.waypoint_file = self._get_waypoint_path()

        # ---- 信号处理 ----
        signal.signal(signal.SIGINT, self._sigint_handler)

    # ===================== 回调函数 =====================

    def state_callback(self, msg):
        self.current_state = msg
        if not msg.connected and self.pose_received:
            rospy.logerr("MAVROS 连接断开! 尝试紧急降落...")
            self._trigger_emergency_land()

    def battery_callback(self, msg):
        self.battery = msg
        if msg.percentage >= 0 and msg.percentage < self.low_battery_threshold and self.pose_received:
            rospy.logerr("低电量警告! 剩余: %.1f%%, 阈值: %.1f%%", msg.percentage, self.low_battery_threshold)
            self._trigger_emergency_land()

    def current_position_callback(self, msg):
        self.current_position = msg
        self.pose_received = True
        self.last_pose_time = rospy.Time.now()
        q = [msg.pose.orientation.x, msg.pose.orientation.y,
             msg.pose.orientation.z, msg.pose.orientation.w]
        self.now_yaw = transformations.euler_from_quaternion(q)[2]

    def position_cmd_callback(self, msg):
        """接收 EGO-Planner 的轨迹点指令并在此直接转发 (若处于导航状态)"""
        self.ego_cmd = msg
        self.last_ego_cmd_time = rospy.Time.now()

        if self.nav_state == "NAVIGATING":
            # 转发给 MAVROS
            setpoint = PositionTarget()
            setpoint.header.stamp = rospy.Time.now()
            setpoint.coordinate_frame = PositionTarget.FRAME_LOCAL_NED
            
            # 使用 位置、速度 和 偏航角 (忽略加速度和偏航角速率)
            # IGNORE_AFX(64) | IGNORE_AFY(128) | IGNORE_AFZ(256) | IGNORE_YAW_RATE(2048) = 2496
            setpoint.type_mask = 2496
            
            setpoint.position.x = msg.position.x
            setpoint.position.y = msg.position.y
            setpoint.position.z = msg.position.z
            
            setpoint.velocity.x = msg.velocity.x
            setpoint.velocity.y = msg.velocity.y
            setpoint.velocity.z = msg.velocity.z
            
            setpoint.yaw = msg.yaw
            self.setpoint_pub.publish(setpoint)

    # ===================== 工具函数 =====================

    def _get_waypoint_path(self):
        ros_param_path = rospy.get_param('~waypoint_file', '')
        if ros_param_path and os.path.isfile(ros_param_path): return ros_param_path
        env_path = os.environ.get('DRONE_WAYPOINT_FILE', '')
        if env_path and os.path.isfile(env_path): return env_path

        script_dir = os.path.dirname(os.path.abspath(__file__))
        candidates = [os.path.join(script_dir, 'point.txt'), os.path.expanduser('~/point.txt')]
        for path in candidates:
            if os.path.isfile(path): return path

        fallback = os.path.join(script_dir, 'point.txt')
        rospy.logwarn("未找到航点文件，使用默认路径: %s", fallback)
        return fallback

    def _clamp(self, value, lower, upper):
        return max(lower, min(upper, value))

    def _pose_is_fresh(self, max_age=1.0):
        if not self.pose_received: return False
        return (rospy.Time.now() - self.last_pose_time).to_sec() <= max_age

    def _check_timeout(self, start_time, timeout):
        return (rospy.Time.now() - start_time).to_sec() > timeout

    def get_distance_to_target_3d(self, target_x, target_y, target_z):
        dx = self.current_position.pose.position.x - target_x
        dy = self.current_position.pose.position.y - target_y
        dz = self.current_position.pose.position.z - target_z
        return math.sqrt(dx**2 + dy**2 + dz**2)

    @staticmethod
    def load_waypoints(filepath):
        waypoints = []
        with open(filepath, 'r') as f:
            for line_num, line in enumerate(f, 1):
                stripped = line.strip()
                if not stripped or stripped.startswith('#'): continue
                parts = stripped.split()
                if len(parts) >= 4:
                    try:
                        x, y, z, t = float(parts[0]), float(parts[1]), float(parts[2]), float(parts[3])
                        waypoints.append((x, y, z, t))
                    except ValueError:
                        rospy.logwarn("航点文件第 %d 行解析失败: %s", line_num, stripped)
                else:
                    rospy.logwarn("航点文件第 %d 行格式无效: %s", line_num, stripped)
        return waypoints

    # ===================== 安全保护 =====================

    def _trigger_emergency_land(self):
        if self._emergency_land_triggered: return
        self._emergency_land_triggered = True
        self.nav_state = "LANDING"
        rospy.logerr("=" * 40)
        rospy.logerr("!!! 紧急降落已触发 !!!")
        rospy.logerr("=" * 40)
        self.land_at_current_position()

    def _check_emergency(self):
        if self._emergency_land_triggered: return True
        if self.pose_received and not self._pose_is_fresh(max_age=3.0):
            rospy.logerr("位姿数据超时 (>3s)，触发紧急降落")
            self._trigger_emergency_land()
            return True
        if not self.current_state.connected and self.pose_received:
            rospy.logerr("MAVROS 连接断开，触发紧急降落")
            self._trigger_emergency_land()
            return True
        return False

    def _sigint_handler(self, signum, frame):
        rospy.loginfo("收到中断信号，尝试安全降落...")
        if self.pose_received and self.current_position.pose.position.z > 0.15:
            self.land_at_current_position()
        sys.exit(0)

    # ===================== 基础飞行控制 =====================

    def send_position_setpoint(self, x, y, z, yaw=None):
        """发送纯位置设定点 (用于起飞、悬停、降落，不依赖 EGO-Planner)"""
        setpoint = PositionTarget()
        setpoint.header.stamp = rospy.Time.now()
        setpoint.coordinate_frame = PositionTarget.FRAME_LOCAL_NED
        setpoint.type_mask = 2552  # 忽略速度和加速度，使用位置和偏航角
        setpoint.position.x = x
        setpoint.position.y = y
        setpoint.position.z = z
        setpoint.yaw = yaw if yaw is not None else self.now_yaw
        self.setpoint_pub.publish(setpoint)

    def wait_for_fcu_ready(self, timeout=30.0):
        rospy.loginfo("等待 MAVROS 连接和本地位姿...")
        start_time = rospy.Time.now()
        while not rospy.is_shutdown():
            if self.current_state.connected and self._pose_is_fresh():
                rospy.loginfo("MAVROS 已连接，本地位姿可用。")
                return True
            if self._check_timeout(start_time, timeout):
                rospy.logerr("等待 MAVROS/本地位姿超时。")
                return False
            self.rate.sleep()
        return False

    def set_offboard_and_arm(self):
        if not self.wait_for_fcu_ready(): return False

        lock_x = self.current_position.pose.position.x
        lock_y = self.current_position.pose.position.y
        lock_yaw = self.now_yaw

        rospy.loginfo("预发布设定点 (5s)...")
        for _ in range(100):
            self.send_position_setpoint(lock_x, lock_y, self.target_z, yaw=lock_yaw)
            self.rate.sleep()

        try:
            if not self.set_mode_client(custom_mode='OFFBOARD').mode_sent: return False
        except rospy.ServiceException: return False

        try:
            if not self.arm_client(True).success: return False
        except rospy.ServiceException: return False

        return True

    # ===================== 任务逻辑 =====================

    def takeoff(self, height=None):
        self.nav_state = "TAKEOFF"
        self.target_z = height if height is not None else self.takeoff_height
        rospy.loginfo("起飞至 %.2f 米...", self.target_z)

        if not self.set_offboard_and_arm(): return False

        self._home_position = (self.current_position.pose.position.x, self.current_position.pose.position.y)
        lock_x = self.current_position.pose.position.x
        lock_y = self.current_position.pose.position.y
        lock_yaw = self.now_yaw

        start_time = rospy.Time.now()
        while not rospy.is_shutdown():
            self.send_position_setpoint(lock_x, lock_y, self.target_z, yaw=lock_yaw)
            if self._check_emergency(): return False

            current_z = self.current_position.pose.position.z
            if abs(current_z - self.target_z) < self.waypoint_z_tol:
                rospy.loginfo("已到达目标高度: %.2f 米", current_z)
                self.nav_state = "HOVER"
                return True

            if self._check_timeout(start_time, self.takeoff_timeout):
                rospy.logerr("起飞超时。")
                return False
            self.rate.sleep()
        return False

    def send_ego_goal(self, x, y, z):
        """向 EGO-Planner 发布全局 3D 目标点"""
        goal = PoseStamped()
        goal.header.stamp = rospy.Time.now()
        goal.header.frame_id = "camera_init"  # FAST-LIO2 父坐标系
        goal.pose.position.x = x
        goal.pose.position.y = y
        goal.pose.position.z = z
        
        # 设定偏航角指向目标
        dx = x - self.current_position.pose.position.x
        dy = y - self.current_position.pose.position.y
        yaw = math.atan2(dy, dx)
        q = transformations.quaternion_from_euler(0, 0, yaw)
        goal.pose.orientation.x = q[0]
        goal.pose.orientation.y = q[1]
        goal.pose.orientation.z = q[2]
        goal.pose.orientation.w = q[3]
        
        self.goal_pub.publish(goal)

    def navigation_target(self, x, y, z, hover_time=2.0):
        self.nav_state = "NAVIGATING"
        self.target_z = z
        
        # 发送目标给 EGO-Planner
        rospy.sleep(1.0)
        self.send_ego_goal(x, y, z)
        rospy.loginfo("规划目标下发至 x=%.2f, y=%.2f, z=%.2f (悬停 %.1fs)", x, y, z, hover_time)

        start_time = rospy.Time.now()
        while not rospy.is_shutdown():
            if self._check_emergency(): return False

            # 若 EGO-Planner 无输出超过 0.5s，则强制悬停以保证安全
            if (rospy.Time.now() - self.last_ego_cmd_time).to_sec() > 0.5:
                rospy.logwarn_throttle(2.0, "未收到 EGO-Planner 轨迹指令，正在原地悬停...")
                self.send_position_setpoint(
                    self.current_position.pose.position.x,
                    self.current_position.pose.position.y,
                    self.current_position.pose.position.z
                )

            dist = self.get_distance_to_target_3d(x, y, z)
            if dist < self.waypoint_xy_tol:
                rospy.loginfo("到达航点! 误差: %.2f m", dist)
                break

            if self._check_timeout(start_time, self.waypoint_timeout):
                rospy.logwarn("航点导航超时，跳转下一航点。")
                break

            self.rate.sleep()

        self.nav_state = "HOVER"
        rospy.loginfo("悬停 %.1f 秒...", hover_time)
        hover_start = rospy.Time.now()
        while not rospy.is_shutdown() and (rospy.Time.now() - hover_start).to_sec() < hover_time:
            self.send_position_setpoint(x, y, z)
            self.rate.sleep()

        rospy.loginfo("航点任务完成。")

    def land_at_current_position(self):
        self.nav_state = "LANDING"
        rospy.loginfo("启动安全降落...")

        if not self.pose_received:
            try: self.set_mode_client(custom_mode='AUTO.LAND')
            except: pass
            return

        lock_x = self.current_position.pose.position.x
        lock_y = self.current_position.pose.position.y
        lock_yaw = self.now_yaw
        current_z = self.current_position.pose.position.z
        target_z = current_z

        if current_z > 0.15:
            rospy.loginfo("缓慢垂直降落 (锁定 XY+Yaw)...")
            start_time = rospy.Time.now()
            while not rospy.is_shutdown() and current_z > 0.2:
                target_z -= 0.02
                target_z = max(target_z, 0.15)
                self.send_position_setpoint(lock_x, lock_y, target_z, yaw=lock_yaw)
                current_z = self.current_position.pose.position.z
                self.rate.sleep()
                if self._check_timeout(start_time, self.land_timeout): break

        try:
            self.set_mode_client(custom_mode='AUTO.LAND')
            rospy.loginfo("AUTO.LAND 已请求，等待落地...")
        except: pass

        start_time = rospy.Time.now()
        while not rospy.is_shutdown():
            if self.current_position.pose.position.z < 0.05:
                rospy.loginfo("已落地。")
                break
            if self._check_timeout(start_time, 30.0): break
            self.rate.sleep()


def main():
    rospy.init_node('navigation_controller', anonymous=True)
    nav = NavigationController()
    rospy.sleep(2.0)

    if not nav.takeoff():
        rospy.logerr("起飞失败，跳过导航。")
        return

    wp_file = nav.waypoint_file
    try: waypoints = NavigationController.load_waypoints(wp_file)
    except IOError:
        rospy.logerr("打开航点文件失败: %s", wp_file)
        nav.land_at_current_position()
        return

    if not waypoints:
        rospy.logwarn("航点为空，悬停后降落。")
        rospy.sleep(3.0)
        nav.land_at_current_position()
        return

    rospy.loginfo("共 %d 个航点，开始执行...", len(waypoints))
    for i, (x, y, z, t) in enumerate(waypoints):
        if nav._emergency_land_triggered: break
        rospy.loginfo("=" * 15 + " 航点 #%d/%d " + "=" * 15, i + 1, len(waypoints))
        nav.navigation_target(x, y, z, t)

    if not nav._emergency_land_triggered:
        rospy.loginfo("所有航点完毕，3秒后降落...")
        rospy.sleep(3.0)
        nav.land_at_current_position()

if __name__ == "__main__":
    main()
