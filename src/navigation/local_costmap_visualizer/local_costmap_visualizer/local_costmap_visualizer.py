import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseStamped, Point
from visualization_msgs.msg import Marker, MarkerArray
from tf2_ros import TransformListener, Buffer
from rclpy.duration import Duration
import numpy as np
from tf_transformations import quaternion_multiply, quaternion_from_euler
from scipy.spatial.transform import Rotation as R


class GradientPathPlanner(Node):
    def __init__(self):
        super().__init__('gradient_path_planner')

        self.declare_parameter('radius', 0.5)
        self.radius_m = self.get_parameter('radius').get_parameter_value().double_value

        self.costmap = None

        # TF
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Subscribers and Publishers
        self.create_subscription(OccupancyGrid, '/local_costmap/costmap', self.costmap_callback, 10)
        self.marker_pub = self.create_publisher(MarkerArray, '/local_costmap_cells', 10)
        self.direction_pub = self.create_publisher(PoseStamped, '/gradient_escape_direction', 10)

        self.timer = self.create_timer(1.0, self.timer_callback)

    def costmap_callback(self, msg: OccupancyGrid):
        self.costmap = msg

    def timer_callback(self):
        if self.costmap is None:
            return

        try:
            transform = self.tf_buffer.lookup_transform(
                self.costmap.header.frame_id,
                'base_link_fake',
                rclpy.time.Time(),
                timeout=Duration(seconds=0.5)
            )
            x_robot = transform.transform.translation.x
            y_robot = transform.transform.translation.y

            # costmap 大小
            width = self.costmap.info.width
            height = self.costmap.info.height
            res = self.costmap.info.resolution

            # costmap 代价地图左下角（0,0格子）在世界坐标系中的位置
            origin_x = self.costmap.info.origin.position.x
            origin_y = self.costmap.info.origin.position.y

            data = np.array(self.costmap.data, dtype=np.int8).reshape((height, width))

            # robot在costmap中距离原点的像素
            mx = int((x_robot - origin_x) / res)
            my = int((y_robot - origin_y) / res)
            radius_cells = int(self.radius_m / res)

            # submap的起始与结束
            x_start = max(0, mx - radius_cells)
            x_end = min(width, mx + radius_cells + 1)
            y_start = max(0, my - radius_cells)
            y_end = min(height, my + radius_cells + 1)
            submap = data[y_start:y_end, x_start:x_end]

            # 可视化子图区域
            self.visualize_submap(submap, x_start, y_start, res, origin_x, origin_y)

            # 计算路径并发布方向箭头
            self.compute_gradient_path(submap, mx, my, x_start, y_start, transform)

        except Exception as e:
            self.get_logger().warn(f"TF 获取失败: {str(e)}")

    def visualize_submap(self, submap, x_start, y_start, res, origin_x, origin_y):
        marker_array = MarkerArray()
        marker = Marker()
        marker.header.frame_id = self.costmap.header.frame_id
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "local_costmap"
        marker.id = 0
        marker.type = Marker.CUBE_LIST
        marker.action = Marker.ADD
        marker.scale.x = res
        marker.scale.y = res
        marker.scale.z = 0.01
        marker.color.a = 0.8
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0

        for iy in range(submap.shape[0]):
            for ix in range(submap.shape[1]):
                cost = submap[iy][ix]
                if cost <= 0:
                    continue
                world_x = (x_start + ix) * res + origin_x + res / 2
                world_y = (y_start + iy) * res + origin_y + res / 2
                pt = Point(x=world_x, y=world_y, z=0.01)
                marker.points.append(pt)

        marker_array.markers.append(marker)
        self.marker_pub.publish(marker_array)

    def compute_gradient_path(self, submap, mx, my, x_start, y_start, transform):
        # 机器人在submap内的坐标
        px = mx - x_start
        py = my - y_start
        print(submap)
        # 获取高代价点（障碍物区域）
        high_cost_points = []
        h, w = submap.shape
        for iy in range(h):
            for ix in range(w):
                cost = submap[iy][ix]
                x = x_start + ix
                y = y_start + iy

                if cost == -1:
                    continue  # 忽略未知区域

                weight = cost
                high_cost_points.append((x, y, weight))

        if not high_cost_points:
            self.get_logger().warn("无高权重点，路径无法生成")
            return []

        # 计算力并得到合成力
        total_force = np.array([0.0, 0.0])
        epsilon = 1e-6
        for x, y, weight in high_cost_points:
            dx = px-(x- x_start)
            dy = py-(y- y_start)
            dist_sq = dx**2 + dy**2 + epsilon
            # print(dx,dy)
            dist = np.sqrt(dist_sq)
            force = (weight / dist_sq) * np.array([dx, dy]) / dist
            total_force += force

            
        # 计算合力的方向
        norm = np.linalg.norm(total_force)
        if norm == 0:
            return
        direction = total_force / norm

        # 计算角度
        theta = np.arctan2(direction[1], direction[0])
        print(f"方向角度: {theta / np.pi * 180}°")
        print(f"力大小：{np.sqrt(total_force[0]**2+total_force[0]**2)}")
        # 原始四元数（来自 TF）
        q1 = R.from_quat([
            transform.transform.rotation.x,
            transform.transform.rotation.y,
            transform.transform.rotation.z,
            transform.transform.rotation.w,
        ])

        # 相对旋转的四元数（绕 Z 轴旋转 theta）
        q2 =  R.from_quat(quaternion_from_euler(0, 0, theta))  # Roll, pitch, yaw

        q_result = q1.inv() * q2
        q_result =q_result.as_quat()

        # 发布机器人方向
        msg = PoseStamped()
        msg.header.frame_id = "base_link_fake"
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.position.x = 0.0
        msg.pose.position.y = 0.0
        msg.pose.position.z = 0.0
        # 填充结果
        msg.pose.orientation.x = q_result[0]
        msg.pose.orientation.y = q_result[1]
        msg.pose.orientation.z = q_result[2]
        msg.pose.orientation.w = q_result[3]


        self.direction_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = GradientPathPlanner()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
