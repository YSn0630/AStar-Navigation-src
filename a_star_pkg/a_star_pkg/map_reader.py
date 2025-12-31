import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid # 맵(/map) 토픽의 타입
import numpy as np

class MapReader(Node):
    def __init__(self):
        super().__init__('step1_map_reader')
        
        # 실제 맵을 받아서 초기화해야 하는 속성들 
        self.map_data = None
        self.map_resolution = 0.05
        self.map_width = 0
        self.map_height = 0
        self.map_origin = [0.0, 0.0]

        self.subscription = self.create_subscription(
            OccupancyGrid,
            '/map',
            self.map_callback,
            10
        )

    def map_callback(self, msg):
        self.map_resolution = msg.info.resolution
        self.map_width = msg.info.width
        self.map_height = msg.info.height
        self.map_origin = [msg.info.origin.position.x, msg.info.origin.position.y]

        # 맵 데이터는 1차원 리스트 형태이므로, 다루기 쉬운 2차원 형태로 reshape 하기
        # 이렇게 하면 특정 좌표가 벽인지 길인지 판단 가능. 예를 들면... self.map_data[y][x]의 값을 확인하는 식으로
        self.map_data = np.array(msg.data).reshape((self.map_height, self.map_width))
        
        self.get_logger().info(f"Map Received: {self.map_width} x {self.map_height}")
        self.get_logger().info(f"Map info: {self.map_resolution}\n  {self.map_origin}")

def main(args=None):
    rclpy.init(args=args)
    node = MapReader()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
