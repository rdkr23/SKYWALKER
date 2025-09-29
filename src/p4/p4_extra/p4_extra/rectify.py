import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge
import cv2

class JpegCompressor(Node):
    def __init__(self):
        super().__init__('jpeg_compressor_node')
        self.bridge = CvBridge()

      
      
        self.jpeg_quality = self.declare_parameter('jpeg_quality', 70).value
        self.encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), self.jpeg_quality]

        # Publish every N-th frame (≥1 ⇒ no skipping)
        self.frame_skip = max(1, self.declare_parameter('frame_skip', 3).value)
        self.counter = 0

        # Resize: target width (px). If <=0 ➜ keep full resolution
        self.output_width = self.declare_parameter('output_width', 640).value
        # ───────────────────────────────────────────────────────────────────────

        # Subscribers / publishers
        self.subscription = self.create_subscription(
            Image,
            '/camera/camera/color/image_raw',
            self.image_callback,
            10)

        self.publisher = self.create_publisher(
            CompressedImage,
            '/camera/camera/color/image_compressed_jpeg',
            10)

        self.get_logger().info(
            f"JPEG compressor ‣ quality={self.jpeg_quality} "
            f"| frame_skip={self.frame_skip} "
            f"| output_width={self.output_width if self.output_width>0 else 'native'}"
        )

    # ───────────────────────────────────────────────────────────────────────────
    def image_callback(self, msg: Image):
        # Throttle FPS by skipping frames
        self.counter += 1
        if (self.counter % self.frame_skip) != 0:
            return

        try:
            # Convert to OpenCV BGR image
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

            # Resize if requested
            if self.output_width > 0 and cv_image.shape[1] > self.output_width:
                scale = self.output_width / float(cv_image.shape[1])
                new_height = int(cv_image.shape[0] * scale)
                cv_image = cv2.resize(cv_image, (self.output_width, new_height),
                                      interpolation=cv2.INTER_AREA)

            # JPEG encode
            success, encoded_img = cv2.imencode('.jpg', cv_image, self.encode_param)
            if not success:
                self.get_logger().warn("JPEG encoding failed")
                return

            # Publish compressed message
            comp_msg = CompressedImage()
            comp_msg.header = msg.header          # keep original time stamp
            comp_msg.format = 'jpeg'
            comp_msg.data = encoded_img.tobytes()
            self.publisher.publish(comp_msg)

            # Optional debug
            self.get_logger().debug(
                f"Pub {len(comp_msg.data)/1024:.1f} KB "
                f"({1.0/self.get_clock().now().nanoseconds*1e9:.1f} Hz)"
            )

        except Exception as e:
            self.get_logger().error(f"Compression error: {e}")
    # ───────────────────────────────────────────────────────────────────────────

def main(args=None):
    rclpy.init(args=args)
    node = JpegCompressor()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
