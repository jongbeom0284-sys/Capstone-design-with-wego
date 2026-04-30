import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

import cv2
import numpy as np
from cv_bridge import CvBridge

from sensor_msgs.msg import Image
from std_msgs.msg import Int32


class LabAdaptiveLane(Node):
    def __init__(self):
        super().__init__('lab_adaptive_lane')

        self.br = CvBridge()
        self.sub_flag = False

        self.subscription = self.create_subscription(
            Image,
            self.declare_parameter('image_topic', '/camera/color/image_raw').value,
            self.image_callback,
            qos_profile_sensor_data,
        )
        self.subscription  # prevent unused warning

        self.dis_publisher = self.create_publisher(
            Int32,
            self.declare_parameter('distance_topic', 'distance_y').value,
            10,
        )
        self.debug_publisher = self.create_publisher(
            Image,
            self.declare_parameter('debug_topic', 'debug_image').value,
            10,
        )
        self.timer_ = self.create_timer(
            float(self.declare_parameter('debug_publish_period_s', 0.1).value),
            self.timer_callback,
        )

        # ROI parameters
        self.roi_x_l = int(self.declare_parameter('roi_x_l', 0).value)
        self.roi_x_h = int(self.declare_parameter('roi_x_h', 320).value)
        self.roi_y_l = int(self.declare_parameter('roi_y_l', 400).value)
        self.roi_y_h = int(self.declare_parameter('roi_y_h', 480).value)

        # Reference distance (x pixel in ROI coordinates)
        self.reference_distance = int(self.declare_parameter('reference_distance', 170).value)

        # Adaptive threshold parameters
        self.blur_ksize = int(self.declare_parameter('blur_ksize', 5).value)
        self.adapt_block_size = int(self.declare_parameter('adapt_block_size', 31).value)
        self.adapt_c = int(self.declare_parameter('adapt_c', -5).value)

        # Morphology parameters (opening then closing)
        self.open_ksize = int(self.declare_parameter('open_ksize', 3).value)
        self.open_iters = int(self.declare_parameter('open_iters', 1).value)
        self.close_ksize = int(self.declare_parameter('close_ksize', 7).value)
        self.close_iters = int(self.declare_parameter('close_iters', 2).value)

        # Contour filtering
        self.min_area = int(self.declare_parameter('min_area', 120).value)
        self.min_aspect_ratio = float(self.declare_parameter('min_aspect_ratio', 3.0).value)
        self.aspect_mode = str(self.declare_parameter('aspect_mode', 'wh').value)  # 'wh' or 'max'

        # Debug image selector
        # 0: ROI, 1: B-channel, 2: binary mask, 3: morph filtered, 4: contour overlay
        self.debug_image_num = int(self.declare_parameter('debug_image_num', 4).value)
        # If true, publish debug image in full-frame size (ROI pasted back).
        self.debug_full_frame = bool(self.declare_parameter('debug_full_frame', True).value)

        self.last_debug = None

    def timer_callback(self):
        if not self.sub_flag or self.last_debug is None:
            return

        encoding = 'bgr8'
        if self.last_debug.ndim == 2:
            encoding = 'mono8'
        self.debug_publisher.publish(self.br.cv2_to_imgmsg(self.last_debug, encoding))

    @staticmethod
    def _odd(v: int, default: int) -> int:
        v = int(v)
        if v <= 1:
            return int(default)
        return v if v % 2 == 1 else v + 1

    @staticmethod
    def _kernel(ksize: int) -> np.ndarray:
        k = max(1, int(ksize))
        return cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (k, k))

    def image_callback(self, msg: Image):
        image = self.br.imgmsg_to_cv2(msg, 'bgr8')

        h, w = image.shape[:2]
        x1 = int(np.clip(self.roi_x_l, 0, w - 1))
        x2 = int(np.clip(self.roi_x_h, x1 + 1, w))
        y1 = int(np.clip(self.roi_y_l, 0, h - 1))
        y2 = int(np.clip(self.roi_y_h, y1 + 1, h))

        roi = image[y1:y2, x1:x2]

        # 1) Color space conversion: BGR -> LAB, use B-channel (yellow contrast)
        lab = cv2.cvtColor(roi, cv2.COLOR_BGR2LAB)
        b_ch = lab[:, :, 2]

        # 2) Dynamic binarization: adaptive threshold for shadows/highlights
        blur_k = self._odd(self.blur_ksize, 5)
        b_blur = cv2.GaussianBlur(b_ch, (blur_k, blur_k), 0)

        block = self._odd(self.adapt_block_size, 31)
        mask = cv2.adaptiveThreshold(
            b_blur,
            255,
            cv2.ADAPTIVE_THRESH_GAUSSIAN_C,
            cv2.THRESH_BINARY,
            block,
            self.adapt_c,
        )

        # 3) Noise filtering: morphological opening + closing
        opened = cv2.morphologyEx(
            mask,
            cv2.MORPH_OPEN,
            self._kernel(self.open_ksize),
            iterations=max(1, self.open_iters),
        )
        filtered = cv2.morphologyEx(
            opened,
            cv2.MORPH_CLOSE,
            self._kernel(self.close_ksize),
            iterations=max(1, self.close_iters),
        )

        # 4) Blob analysis: contour detection + aspect ratio filter
        contours, _ = cv2.findContours(filtered, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        lane_contours = []
        for c in contours:
            area = cv2.contourArea(c)
            if area < self.min_area:
                continue
            x, y, ww, hh = cv2.boundingRect(c)
            if hh <= 0 or ww <= 0:
                continue

            ar_wh = float(ww) / float(hh)
            ar_max = max(ar_wh, 1.0 / max(1e-6, ar_wh))
            ar = ar_wh if self.aspect_mode == 'wh' else ar_max
            if ar < self.min_aspect_ratio:
                continue

            lane_contours.append((c, area, x, y, ww, hh))

        # Merge selected blobs and compute centroid for distance output
        distance_to_ref = 0
        overlay = roi.copy()
        if lane_contours:
            total_area = 0.0
            sum_cx = 0.0
            for c, area, x, y, ww, hh in lane_contours:
                M = cv2.moments(c)
                if M['m00'] <= 1e-6:
                    continue
                cx = float(M['m10'] / M['m00'])
                total_area += area
                sum_cx += cx * area

                cv2.rectangle(overlay, (x, y), (x + ww, y + hh), (0, 255, 0), 2)
                cv2.drawContours(overlay, [c], -1, (255, 0, 0), 2)

            if total_area > 0:
                cx_roi = int(sum_cx / total_area)
                cx_img = x1 + cx_roi

                cv2.circle(overlay, (cx_roi, int((y2 - y1) * 0.5)), 8, (0, 0, 255), -1)
                cv2.line(overlay, (self.reference_distance, 0), (self.reference_distance, y2 - y1), (0, 255, 255), 3)

                distance_to_ref = int(self.reference_distance - cx_roi)

        # Publish distance (same interface as existing detect_line)
        dis = Int32()
        dis.data = int(distance_to_ref)
        self.dis_publisher.publish(dis)

        # Debug image selection
        if self.debug_image_num == 0:
            debug_roi = roi
        elif self.debug_image_num == 1:
            debug_roi = b_ch
        elif self.debug_image_num == 2:
            debug_roi = mask
        elif self.debug_image_num == 3:
            debug_roi = filtered
        else:
            debug_roi = overlay

        if not self.debug_full_frame:
            self.last_debug = debug_roi
        else:
            if debug_roi.ndim == 2:
                full = np.zeros((h, w), dtype=debug_roi.dtype)
                full[y1:y2, x1:x2] = debug_roi
            else:
                full = image.copy()
                full[y1:y2, x1:x2] = debug_roi
                cv2.rectangle(full, (x1, y1), (x2 - 1, y2 - 1), (0, 255, 255), 2)
            self.last_debug = full

        self.sub_flag = True


def main(args=None):
    rclpy.init(args=args)
    node = LabAdaptiveLane()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
