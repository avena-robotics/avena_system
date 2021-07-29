import sys
from custom_interfaces.srv import DataStoreItemCam1Select
import rclpy
from rclpy.node import Node


class MinimalClientAsync(Node):

    def __init__(self):
        super().__init__('rgb_delete_client_async')
        self.cli = self.create_client(DataStoreItemCam1Select, 'item_cam_1_select')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = DataStoreItemCam1Select.Request()

    def send_request(self):
        self.req.time_stamp.data = float(sys.argv[1])
        self.future = self.cli.call_async(self.req)


def main(args=None):
    rclpy.init(args=args)

    minimal_client = MinimalClientAsync()
    minimal_client.send_request()

    while rclpy.ok():
        rclpy.spin_once(minimal_client)
        if minimal_client.future.done():
            try:
                response = minimal_client.future.result()
            except Exception as e:
                minimal_client.get_logger().info(
                    'Service call failed %r' % (e,))
            else:
                minimal_client.get_logger().info(str(response.time_stamp.data))
                for label in response.item_cam1_labels:
                    minimal_client.get_logger().info(label.data)
                for mask in response.item_cam1_masks:
                    minimal_client.get_logger().info(mask.encoding)                    
            break

    minimal_client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()