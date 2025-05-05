import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from tf2_ros import TransformListener, Buffer
import numpy as np
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
from rclpy.duration import Duration

class LocalCostmapVisualizer(Node):
    def __init__(self):
        super().__init__('local_costmap_visualizer')
        self.radius_m = 0.25  # 提取的半径，单位：米
        self.costmap = None

        # TF Buffer & Listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Costmap 订阅
        self.subscription = self.create_subscription(
            OccupancyGrid,
            '/local_costmap/costmap',
            self.costmap_callback,
            10
        )

        # Marker 发布器
        self.marker_pub = self.create_publisher(MarkerArray, '/local_costmap_cells', 10)

        # 定时器周期性执行
        self.timer = self.create_timer(1.0, self.timer_callback)

    def costmap_callback(self, msg: OccupancyGrid):
        self.costmap = msg

    def timer_callback(self):
        if self.costmap is None:
            return

        try:
            # 获取 base_link 在 costmap frame 下的位置
            transform = self.tf_buffer.lookup_transform(
                self.costmap.header.frame_id,
                'base_link',
                rclpy.time.Time(),
                timeout=Duration(seconds=0.5)
            )

            x_robot = transform.transform.translation.x
            y_robot = transform.transform.translation.y

            # 地图参数
            width = self.costmap.info.width
            height = self.costmap.info.height
            res = self.costmap.info.resolution
            origin_x = self.costmap.info.origin.position.x
            origin_y = self.costmap.info.origin.position.y

            data = np.array(self.costmap.data, dtype=np.int8).reshape((height, width))
            mx = int((x_robot - origin_x) / res)
            my = int((y_robot - origin_y) / res)
            radius_cells = int(self.radius_m / res)

            # 提取局部区域
            y_start = max(0, my - radius_cells)
            y_end = min(height, my + radius_cells + 1)
            x_start = max(0, mx - radius_cells)
            x_end = min(width, mx + radius_cells + 1)
            submap = data[y_start:y_end, x_start:x_end]

            # 可视化 Marker 构建
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
                        continue  # 忽略空白或未知

                    world_x = (x_start + ix) * res + origin_x + res / 2
                    world_y = (y_start + iy) * res + origin_y + res / 2

                    pt = Point()
                    pt.x = world_x
                    pt.y = world_y
                    pt.z = 0.01
                    marker.points.append(pt)

            marker_array.markers.append(marker)
            self.marker_pub.publish(marker_array)

        except Exception as e:
            self.get_logger().warn(f"TF 获取失败: {str(e)}")

def main(args=None):
    rclpy.init(args=args)
    node = LocalCostmapVisualizer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
