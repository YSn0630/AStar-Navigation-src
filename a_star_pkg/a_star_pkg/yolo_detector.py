#!/usr/bin/env python3
# Image data preprocessing + Timer-based publish

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Bool
import numpy as np
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
import cv2
from cv_bridge import CvBridge
from ultralytics import YOLO
import os
from rcl_interfaces.msg import SetParametersResult

MODEL_PATH = '/home/dev/__pwd__/a_star_pkg/best.pt'
IMGSZ_SIZE = 640
CONFIDENCE_THRESHOLD = 0.5

image_raw_qos = QoSProfile(
    depth=10,
    reliability=ReliabilityPolicy.BEST_EFFORT,
    history=HistoryPolicy.KEEP_LAST
)
stop_msg_qos = QoSProfile(
    depth=5,
    reliability=ReliabilityPolicy.RELIABLE,
    history=HistoryPolicy.KEEP_LAST
)

class YoloImageDetector(Node):
    def __init__(self):
        super().__init__('waffle_v4l2')

        # yaml Parameters
        self.declare_parameter('model_path', MODEL_PATH)

        self.model_path = self.get_parameter(
            'model_path').value

        if not os.path.exists(self.model_path):
            self.get_logger().error(f"ERROR: Can't find {self.model_path}")
            raise FileNotFoundError(f"Model file not found at {self.model_path}")

        self.add_on_set_parameters_callback(self.param_callback)

        self.model = YOLO(self.model_path)
        self.bridge = CvBridge()
        self.stop_by_yolo = False
        self.prev_state = None

        # Subscriber
        self.image_subscriber = self.create_subscription(
            Image,
            '/image_compressed',
            self.image_callback,
            image_raw_qos
        )

        # Publisher
        self.stop_pub = self.create_publisher(
            Bool,
            '/stop_signal',
            stop_msg_qos
        )

        # Timer
        self.timer = self.create_timer(0.1, self.publish_msg)

    def image_callback(self, data):
        try:
            cv_image = self.bridge.compressed_imgmsg_to_cv2(data, "bgr8")
        except Exception as e:
            self.get_logger().warn(f"CV Bridge error: {e}")
            return
        
        results = self.model(cv_image, imgsz=IMGSZ_SIZE, conf=CONFIDENCE_THRESHOLD, verbose=False)
        
        if len(results[0].boxes) > 0:
            self.stop_by_yolo = True
        else:
            self.stop_by_yolo = False

        annotated_frame = results[0].plot()
        
        cv2.imshow("YOLO Detection Viewer", annotated_frame)
        cv2.waitKey(100)

    def publish_msg(self):
        msg = Bool()
        msg.data = self.stop_by_yolo

        if msg.data != self.prev_state:
            self.stop_pub.publish(msg)
            self.get_logger().info(f"stop_signal changed -> {msg.data}")

        self.prev_state = msg.data

    def param_callback(self, params):
        for param in params:
            if param.name == 'MODEL_PATH':
                if not os.path.exists(param.value):
                    self.get_logger().error("Invalid MODEL_PATH")
                    return SetParametersResult(successful=False)
                self.model_path = param.value
                self.model = YOLO(self.model_path)
                self.get_logger().info(f"MODEL_PATH updated: {self.model_path}")

        return SetParametersResult(successful=True)


def main(args=None):
    rclpy.init(args=args)
    node = None
    
    try:
        node = YoloImageDetector() 
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except FileNotFoundError as e:
        print(f"\n\nCan't find model file. Check again.\n{e}")
    except Exception as e:
        if node:
            node.get_logger().error(f"Unknown error: {e}")
        else:
             print(f"Unknown error before creating Node: {e}")
    finally:
        if node: 
            node.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main()