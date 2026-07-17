import cv2
import torch
import numpy as np
import segmentation_models_pytorch as smp

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from cv_bridge import CvBridge


class UNetNode(Node):

    def __init__(self):

        super().__init__("unet_node")

        self.bridge = CvBridge()

        self.device = torch.device("cpu")

        self.get_logger().info("Loading U-Net...")

        self.model = smp.Unet(
            encoder_name="resnet34",
            encoder_weights=None,
            in_channels=3,
            classes=3
        )

        self.model.load_state_dict(
            torch.load(
                "/home/mariam/parking_ws/models/UNet_best.pth",
                map_location=self.device
            )
        )

        self.model.to(self.device)
        self.model.eval()

        self.get_logger().info("U-Net Loaded Successfully")

        self.subscription = self.create_subscription(
            Image,
            "/camera/image_raw",
            self.image_callback,
            10
        )

        self.mask_pub = self.create_publisher(
            Image,
            "/lane_mask",
            10
        )

        self.class_pub = self.create_publisher(
            Image,
            "/lane_classes",
            10
        )

    def image_callback(self, msg):

        frame = self.bridge.imgmsg_to_cv2(
            msg,
            desired_encoding="bgr8"
        )

        rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

        rgb = cv2.resize(rgb, (512, 512))

        rgb = rgb.astype(np.float32) / 255.0

        tensor = torch.from_numpy(rgb)

        tensor = tensor.permute(2, 0, 1)

        tensor = tensor.unsqueeze(0)

        tensor = tensor.to(self.device)

        with torch.no_grad():
            output = self.model(tensor)

        prediction = torch.argmax(output, dim=1)
        prediction = prediction.squeeze().cpu().numpy().astype(np.uint8)

        color = np.zeros((512, 512, 3), dtype=np.uint8)

        color[prediction == 1] = (61, 61, 245)
        color[prediction == 2] = (250, 69, 50)

        mask_msg = self.bridge.cv2_to_imgmsg(
            color,
            encoding="rgb8"
        )

        class_msg = self.bridge.cv2_to_imgmsg(
            prediction,
            encoding="mono8"
        )

        self.mask_pub.publish(mask_msg)
        self.class_pub.publish(class_msg)


def main():

    rclpy.init()

    node = UNetNode()

    rclpy.spin(node)

    node.destroy_node()

    rclpy.shutdown()


if __name__ == "__main__":
    main()
