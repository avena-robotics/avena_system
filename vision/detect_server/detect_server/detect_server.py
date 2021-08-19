from threading import Thread, Lock

import cv_bridge
import rclpy
from rclpy.qos import QoSProfile
from rclpy.qos import HistoryPolicy
from rclpy.action import ActionServer
from rclpy.node import Node
from custom_interfaces.action import SimpleAction
from custom_interfaces.msg import Detections
from custom_interfaces.srv import DataStoreCamerasDataSelect, DataStoreCamerasDataInsert, DataStoreDetectronSelect, \
    DataStoreDetectronInsert
from .DetectronInference import DetectronInference
from ament_index_python.packages import get_package_share_directory
import time
import os
from os.path import expanduser
from builtins import float
from std_msgs.msg import Float64


import pydevd_pycharm
pydevd_pycharm.settrace('localhost', port=1090, stdoutToServer=True, stderrToServer=True)

class InsertRgbClientAsync(Node):

    def __init__(self):
        super().__init__('rgb_insert_client_async')
        self.ds_insert_rgb = self.create_client(DataStoreCamerasDataInsert, 'cameras_data_insert')
        while not self.ds_insert_rgb.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Datastore insert rgb service not available, waiting again...')
        self.rgb_ins_req = DataStoreCamerasDataInsert.Request()

    def send_request(self):
        self.ins_rgb_future = self.ds_insert_rgb.call_async(self.rgb_ins_req)


class InsertDetectronClientAsync(Node):

    def __init__(self):
        super().__init__('detectron_insert_client_async')
        self.ds_insert_detectron = self.create_client(DataStoreDetectronInsert, 'detectron_insert')
        while not self.ds_insert_detectron.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Datastore detectron result service not available, waiting again...')
        self.detectron_ins_req = DataStoreDetectronInsert.Request()

    def send_request(self, detections_result):
        detections_msg = Detections()
        detections_msg.cam1_masks = list()
        detections_msg.cam1_labels = list()
        detections_msg.cam2_masks = list()
        detections_msg.cam2_labels = list()

        for k,v in detections_result[0].items():
            for i in range(len(v)):
                detections_msg.cam1_masks.append(v[i])
                detections_msg.cam1_labels.append(k.lower())

        for k,v in detections_result[1].items():
            for j in range(len(v)):
                detections_msg.cam2_masks.append(v[j])
                detections_msg.cam2_labels.append(k.lower())


        self.detectron_ins_req.data = detections_msg
        ts = Float64()
        ts.data = float(time.time())
        self.detectron_ins_req.time_stamp = ts
        self.future = self.ds_insert_detectron.call_async(self.detectron_ins_req)


class SelectRgbClientAsync(Node):

    def __init__(self):
        super().__init__('rgb_select_client_async')
        self.db_sel_rgb = self.create_client(DataStoreCamerasDataSelect, 'cameras_data_select')
        while not self.db_sel_rgb.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.rgb_sel_req = DataStoreCamerasDataSelect.Request()

    def send_request(self):
        self.sel_rgb_future = self.db_sel_rgb.call_async(self.rgb_sel_req)


class DetectActionServer(Node):

    def __init__(self):
        super().__init__('detect_action_server')

        self._action_server = ActionServer(
            self,
            SimpleAction,
            'detect_action',
            self.execute_callback)
        self.qos = QoSProfile(depth=1)
        self.qos.history = HistoryPolicy.RMW_QOS_POLICY_HISTORY_KEEP_LAST
        # self.detections_publisher = self.create_publisher(DataStoreDetectronSelect.Response, 'filter_detections',
        #                                                   qos_profile=self.qos)

        # init detectron instances
        self.package_share_directory = get_package_share_directory('detect_server')
        self.detectron_cam1 = DetectronInference(os.path.join(str(expanduser("~")), 'detectron2_weights'))
        self.detectron_cam2 = DetectronInference(os.path.join(str(expanduser("~")), 'detectron2_weights'))
        self.lock = Lock()
        self.cv_bridge = cv_bridge.CvBridge()

        # datastore insert client
        self.ds_ins_client = InsertRgbClientAsync()

        # datastore select client
        self.ds_sel_client = SelectRgbClientAsync()

        # datastore detectron result insert client
        self.detectron_ins_client = InsertDetectronClientAsync()

    def make_detction(self, lock, detectron_instance, image, result, cam_no):
        res = detectron_instance.detect_image(image)
        lock.acquire()
        result[cam_no] = res
        lock.release()

    def get_rgb_from_datastore(self):
        self.ds_sel_client.send_request()
        while rclpy.ok():
            rclpy.spin_once(self.ds_sel_client)
            if self.ds_sel_client.sel_rgb_future.done():
                try:
                    response = self.ds_sel_client.sel_rgb_future.result()
                    return response
                except Exception as e:
                    self.ds_sel_client.get_logger().info(
                        'Service call failed %r' % (e,))
                break

    def save_detectron_result_to_datastore(self, results):

        self.detectron_ins_client.send_request(results)
        while rclpy.ok():
            rclpy.spin_once(self.detectron_ins_client)
            if self.detectron_ins_client.future.done():
                try:
                    response = self.detectron_ins_client.future.result()
                except Exception as e:
                    self.detectron_ins_client.get_logger().info(
                        'Service call failed %r' % (e,))
                else:
                    self.detectron_ins_client.get_logger().info(str(response.result))
                break

    def execute_callback(self, goal_handle):
        # start = time.time()
        self.get_logger().info('Executing goal...')

        # read pictures from datastore
        rgb_images = self.get_rgb_from_datastore()

        # ds = time.time()
        # print("Getting images from datastore: ", ds - start)

        # perform inference on images
        cam1_img = self.cv_bridge.imgmsg_to_cv2(rgb_images.cam1_rgb)
        cam2_img = self.cv_bridge.imgmsg_to_cv2(rgb_images.cam2_rgb)

        # rerad_time = time.time()
        # print("Preparing both images: ", rerad_time-start)

        result = [None, None]

        # det = time.time()
        # self.make_detction(self.lock, self.detectron_cam1, cam1_img, result, 0)
        # self.make_detction(self.lock, self.detectron_cam2, cam2_img, result, 1)
        t1 = Thread(target=self.make_detction, args=(self.lock, self.detectron_cam1, cam1_img, result, 0,))
        t2 = Thread(target=self.make_detction, args=(self.lock, self.detectron_cam2, cam2_img, result, 1,))
        #
        t1.start()
        t2.start()
        t1.join()
        t2.join()

        # det_end = time.time()
        # print("Detections time: ", det_end - det)

        assert result[0] is not None, "Detectron didnt succesfully return results for cam1"
        assert result[1] is not None, "Detectron didnt succesfully return results for cam2"

        # pub_start = time.time()

        # save results to datastore
        self.save_detectron_result_to_datastore(results=result)

        # pub_end = time.time()
        # print("Publishing time: ", pub_end-pub_start)

        goal_handle.succeed()
        result = SimpleAction.Result()

        # end = time.time()
        # print("Execute callback time: ", end - start)

        return result


def main(args=None):
    rclpy.init(args=args)

    detect_action_server = DetectActionServer()

    rclpy.spin(detect_action_server)


if __name__ == '__main__':
    main()
