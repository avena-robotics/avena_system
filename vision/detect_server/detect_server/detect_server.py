from threading import Thread, Lock

import cv_bridge
import rclpy
from rclpy.qos import QoSProfile
from rclpy.qos import HistoryPolicy
from rclpy.action import ActionServer
from rclpy.node import Node
from custom_interfaces.action import SimpleAction
from custom_interfaces.msg import Detections, DetectionsList
from custom_interfaces.srv import DataStoreRgbdSyncSelect, DataStoreDetectronInsert
from .DetectronInference import DetectronInference
from ament_index_python.packages import get_package_share_directory
import time
import os
from os.path import expanduser
from builtins import float
from std_msgs.msg import Float64
from builtin_interfaces.msg import Time
from rcl_interfaces.srv import GetParameters
import json

import cv2 as cv
# import pydevd_pycharm
# pydevd_pycharm.settrace('localhost', port=1090, stdoutToServer=True, stderrToServer=True)

# class InsertRgbClientAsync(Node):
# import pydevd_pycharm
# pydevd_pycharm.settrace('localhost', port=1090, stdoutToServer=True, stderrToServer=True)

#     def __init__(self):
#         super().__init__('rgb_insert_client_async')
#         self.ds_insert_rgb = self.create_client(DataStoreCamerasDataInsert, 'cameras_data_insert')
#         while not self.ds_insert_rgb.wait_for_service(timeout_sec=1.0):
#             self.get_logger().info('Datastore insert rgb service not available, waiting again...')
#         self.rgb_ins_req = DataStoreCamerasDataInsert.Request()

#     def send_request(self):
#         self.ins_rgb_future = self.ds_insert_rgb.call_async(self.rgb_ins_req)


class InsertDetectronClientAsync(Node):

    def __init__(self):
        super().__init__('detectron_insert_client_async')
        self.ds_insert_detectron = self.create_client(
            DataStoreDetectronInsert, 'detectron_insert')
        while not self.ds_insert_detectron.wait_for_service(timeout_sec=1.0):
            self.get_logger().info(
                'Datastore detectron result service not available, waiting again...')
        self.detectron_ins_req = DataStoreDetectronInsert.Request()

    def send_request(self, detections_result):
        detections_list = DetectionsList()
        for cam in detections_result:
            detections_msg = Detections()
            detections_msg.masks = list()
            detections_msg.labels = list()
            for k, v in cam.items():
                for i in range(len(v)):
                    detections_msg.masks.append(v[i])
                    detections_msg.labels.append(k.lower())
            detections_list.cameras.append(detections_msg)


        self.detectron_ins_req.data = detections_list
        ts = Float64()
        ts.data = float(time.time())
        self.detectron_ins_req.time_stamp = ts
        # !! : Does not work sometimes :self.detectron_ins_req.data.header.stamp = self.get_clock().now().to_msg()
        data_header_time = Time(sec=1, nanosec=1)
        self.detectron_ins_req.data.header.stamp = data_header_time
        self.future = self.ds_insert_detectron.call_async(
            self.detectron_ins_req)


class SelectRgbClientAsync(Node):

    def __init__(self):
        super().__init__('rgb_select_client_async')
        self.db_sel_rgb = self.create_client(
            DataStoreRgbdSyncSelect, 'rgbd_sync_select')
        while not self.db_sel_rgb.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.rgb_sel_req = DataStoreRgbdSyncSelect.Request()

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
        self.parameters_server_client = self.create_client(GetParameters, '/parameters_server/get_parameters')
        if not self.parameters_server_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('parameter server not available')
            exit(-1)
        self.labels_map = {}
        self.load_label_map()

        # init detectron instances
        self.package_share_directory = get_package_share_directory(
            'detect_server')
        self.detectron_cam1 = DetectronInference(
            os.path.join(str(expanduser("~")), 'detectron2_weights'), self.labels_map)
        self.detectron_cam2 = DetectronInference(
            os.path.join(str(expanduser("~")), 'detectron2_weights'), self.labels_map)
        self.lock = Lock()
        self.cv_bridge = cv_bridge.CvBridge()

        # datastore insert client
        # self.ds_ins_client = InsertRgbClientAsync()

        # datastore select client
        self.ds_sel_client = SelectRgbClientAsync()

        # datastore detectron result insert client
        self.detectron_ins_client = InsertDetectronClientAsync()



    def make_detction(self, lock, detectron_instance, image, result, cam_no):
        image = image[:,:,:3]
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
                    return response.data.rgbs
                except Exception as e:
                    self.ds_sel_client.get_logger().info(
                        'Service call failed %r' % (e,))
                break

    def load_label_map(self):
        response = None
        req = GetParameters.Request()
        req.names = ['labels']
        future = self.parameters_server_client.call_async(req)
        while rclpy.ok():
            rclpy.spin_once(self)
            if future.done():
                try:
                    response = future.result()
                except Exception as e:
                    get_logger().info(
                        'Service call failed %r' % (e,)
                    )
                break

        if response is not None:
            # self.get_logger().info(str(response))
            labels_json = json.loads(response.values[0].string_value)
            # self.get_logger().info(str(self.labels_map))
            counter = 0
            for label_desc in labels_json:
                if label_desc["detection_id"] < 0:
                    continue
                if label_desc['label'] in self.labels_map:
                    self.get_logger().info(str(label_desc['label']))
                    self.get_logger().error("u have not unique indexes in labels yamls in parameter server for label " + str(label_desc['label'])) 
                    exit(-1)
                self.labels_map[label_desc["detection_id"]] = label_desc['label']                
        else:
            self.get_logger().error("service did not respond in required time.") 
        pass


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

    def run_detections(self ,cam1_img, cam2_img):
        result = [None, None]

        t1 = Thread(target=self.make_detction, args=(
            self.lock, self.detectron_cam1, cam1_img, result, 0,))
        t1.start()


        if isinstance(cam2_img, type(cam1_img)) :
            t2 = Thread(target=self.make_detction, args=(
                self.lock, self.detectron_cam2, cam2_img, result, 1,))
            t2.start()
            t2.join()

        t1.join()

        assert result[0] is not None, "Detectron didnt succesfully return results for instance 1"
        if isinstance(cam2_img, type(cam1_img)) :
            assert result[1] is not None, "Detectron didnt succesfully return results for instance 2"


        if not isinstance(cam2_img, type(cam1_img)) :
            result.pop()

        return result



    def execute_callback(self, goal_handle):
        self.get_logger().info('Executing goal...')

        # read pictures from datastore
        rgb_images = self.get_rgb_from_datastore()
        detections_result = []

        for x in range(0, len(rgb_images), 2):
            cam1_img = self.cv_bridge.imgmsg_to_cv2(rgb_images[x])
            if(x+1 < len(rgb_images)):
                cam2_img = self.cv_bridge.imgmsg_to_cv2(rgb_images[x+1])
            else:
                cam2_img = None
            detections_result += self.run_detections(cam1_img, cam2_img)
        
        # save results to datastore
        self.save_detectron_result_to_datastore(results=detections_result)
        goal_handle.succeed()
        result = SimpleAction.Result()
        return result


def main(args=None):
    rclpy.init(args=args)

    detect_action_server = DetectActionServer()

    rclpy.spin(detect_action_server)


if __name__ == '__main__':
    main()
