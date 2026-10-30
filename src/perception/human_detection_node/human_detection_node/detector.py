"""YOLO person detector for recorded RGB images."""

from pathlib import Path

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from ultralytics import YOLO
from vision_msgs.msg import Detection2D, Detection2DArray, ObjectHypothesisWithPose


class Detector(Node):
    """Turn each RGB bag image into person boxes and an annotated image."""

    def __init__(self):
        super().__init__('human_detector')
        model_path = Path(self.declare_parameter('model_path', '/opt/models/yolo26n.pt').value)
        self.confidence = float(self.declare_parameter('confidence', 0.35).value)
        self.image_size = int(self.declare_parameter('image_size', 640).value)
        if not model_path.is_file():
            raise FileNotFoundError(f'model weights missing: {model_path}; see perception setup guide')
        # The default weights are bundled with the image; model_path can override them.
        self.model = YOLO(str(model_path))
        self.annotated_pub = self.create_publisher(Image, '/m20/camera/front/human/image_annotated', qos_profile_sensor_data)
        self.detection_pub = self.create_publisher(Detection2DArray, '/m20/camera/front/human/detections', qos_profile_sensor_data)
        self.image_sub = self.create_subscription(Image, '/m20/camera/front/image_raw', self.on_image, qos_profile_sensor_data)
        self.get_logger().info(f'loaded person model {model_path}')

    def on_image(self, msg):
        """Run CPU inference and publish both required outputs for this frame."""
        if msg.encoding != 'rgb8' or msg.step != msg.width * 3 or len(msg.data) != msg.step * msg.height:
            self.get_logger().error('expected packed rgb8 image; skipping frame')
            return
        rgb = np.frombuffer(msg.data, np.uint8).reshape(msg.height, msg.width, 3)
        # The bag is rgb8; OpenCV drawing and Ultralytics' NumPy input use BGR.
        bgr = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
        # COCO class 0 is person. Filtering here keeps the output contract
        # focused on humans rather than every object YOLO can recognize.
        result = self.model.predict(bgr, classes=[0], conf=self.confidence,
                                    imgsz=self.image_size, device='cpu', verbose=False)[0]
        detections = Detection2DArray()
        # An empty array is still published when this frame has no people.
        detections.header = msg.header
        for box in result.boxes:
            x1, y1, x2, y2 = [float(v) for v in box.xyxy[0].tolist()]
            score = float(box.conf[0])
            detection = Detection2D()
            detection.header = msg.header
            # vision_msgs stores a 2D box as its centre plus width and height.
            detection.bbox.center.position.x = (x1 + x2) / 2
            detection.bbox.center.position.y = (y1 + y2) / 2
            detection.bbox.size_x = x2 - x1
            detection.bbox.size_y = y2 - y1
            hypothesis = ObjectHypothesisWithPose()
            hypothesis.hypothesis.class_id = 'person'
            hypothesis.hypothesis.score = score
            detection.results.append(hypothesis)
            detections.detections.append(detection)
            cv2.rectangle(bgr, (round(x1), round(y1)), (round(x2), round(y2)), (0, 255, 0), 2)
            cv2.putText(bgr, f'person {score:.2f}', (round(x1), max(20, round(y1) - 8)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
        annotated = Image()
        annotated.header = msg.header
        annotated.height = msg.height
        annotated.width = msg.width
        annotated.encoding = 'rgb8'
        annotated.step = msg.step
        # Convert back to rgb8 so the output encoding matches its bytes.
        annotated.data = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB).tobytes()
        self.annotated_pub.publish(annotated)
        self.detection_pub.publish(detections)


def main():
    """Start the ROS detector node."""
    rclpy.init()
    node = Detector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        # Ctrl+C is the normal way to stop a node during bag review.
        pass
    finally:
        node.destroy_node()
        # ROS may have already shut down its context after SIGINT.
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
