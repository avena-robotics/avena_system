import rclpy
from rclpy.node import Node
import cv2
from cv_bridge import CvBridge
from custom_interfaces.msg import CamerasData
import numpy as np
from .yolo_detector.detect import YoloInference
from .yolo_detector.utils.plots import colors, plot_one_box
from pathlib import Path
from std_msgs.msg import Bool


class Rgb_Diff(Node):

    def __init__(self):
        super().__init__('rgb_diff')
        self.set_masks()
        self.cameras_data_subscription = self.create_subscription(
            CamerasData,
            'cameras_data',
            self.cameras_data_callback,
            10)

        self.sec_trigger_publisher = self.create_publisher(Bool, 'security_trigger', 10)


        self.buffered_cameras_data_msg = None
        self.background_cam1 = None
        self.background_cam2 = None
        self.cam1_mask = None
        self.cam2_mask = None
        self.yolo_model = YoloInference()
        self.cv_bridge = CvBridge()
        self.security_trigger_msg = Bool()
        self.visualize = True

    def check_masks_set(self):
        if self.cam1_mask is None or self.cam2_mask is None:
            return False
        return True

    def sec_area_breached(self, cam1_bbox_mask, cam2_bbox_mask):

        assert cam1_bbox_mask.shape == self.cam1_mask.shape, "Shapes of input image from cam 1 and security area mask bo not match"
        assert cam2_bbox_mask.shape == self.cam2_mask.shape, "Shapes of input image from cam 2 and security area mask bo not match"

        res_cam1 = cv2.bitwise_and(cam1_bbox_mask, self.cam1_mask)
        res_cam2 = cv2.bitwise_and(cam2_bbox_mask, self.cam2_mask)

        sec_area_breached = np.any(res_cam1) or np.any(res_cam2)
        return sec_area_breached


    def get_bboxes_mask(self, image, bboxes):
        mask = np.zeros(image.shape[:2], dtype="uint8")
        for bbox in bboxes:
            cv2.rectangle(mask, (bbox[0], bbox[1]), (bbox[2], bbox[3]), 255, -1)
        return mask

    def cameras_data_callback(self, cameras_data_msg):

        if not self.check_masks_set():
            print('Not performing diff operation - background masks are not set')
            return

        ros_cam1 = cameras_data_msg.cam1_rgb
        ros_cam2 = cameras_data_msg.cam2_rgb
        ros_cam1 = self.cv_bridge.imgmsg_to_cv2(ros_cam1, desired_encoding='passthrough')
        ros_cam2 = self.cv_bridge.imgmsg_to_cv2(ros_cam2, desired_encoding='passthrough')
        cam1_res = self.yolo_model.inference(ros_cam1)
        cam2_res = self.yolo_model.inference(ros_cam2)

        # bounding boxes binary masks
        if cam1_res:
            ros_cam1 = self.get_bboxes_mask(ros_cam1, cam1_res)
        if cam2_res:
            ros_cam2 = self.get_bboxes_mask(ros_cam2, cam2_res)
        # check if security area breached
        sec_area_breached = self.sec_area_breached(ros_cam1, ros_cam2)

        if self.visualize:
            cam1_tpl = (ros_cam1, cam1_res)
            cam2_tpl = (ros_cam2, cam2_res)

            for img in [cam1_tpl, cam2_tpl]:
                for bbox in img[1]:
                    c = (255, 0, 0) if sec_area_breached else (0, 255, 0)
                    plot_one_box(bbox, img[0], label='person', color=c, line_thickness=2)
            cv2.imshow('cam1', cam1_tpl[0])
            cv2.imshow('cam2', cam2_tpl[0])
            cv2.waitKey(10)

        # put the result into ros medssage
        self.security_trigger_msg.data = sec_area_breached

        self.sec_trigger_publisher.publish(self.security_trigger_msg)

    def set_masks(self):
        path = Path(__file__).absolute()
        print(path)
        self.cam1_mask = cv2.imread(
            '/home/avena/PycharmProjects/avena_system_yolov5/vision/rgb_diff/rgb_diff/sceurity_area_masks/cam1_mask.png')
        self.cam2_mask = cv2.imread(
            '/home/avena/PycharmProjects/avena_system_yolov5/vision/rgb_diff/rgb_diff/sceurity_area_masks/cam2_mask.png')


def main(args=None):
    rclpy.init(args=args)

    rgb_diff = Rgb_Diff()

    rclpy.spin(rgb_diff)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    rgb_diff.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()