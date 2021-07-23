import sys
from custom_interfaces.srv import DataStoreRgbDataSelect
import rclpy
from rclpy.node import Node


class MinimalClientAsync(Node):

    def __init__(self):
        super().__init__('rgb_delete_client_async')
        self.cli = self.create_client(DataStoreRgbDataSelect, 'rgb_data_select')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = DataStoreRgbDataSelect.Request()

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
                minimal_client.get_logger().info(str(response.camera_1_rgb.encoding))
                minimal_client.get_logger().info(str(response.camera_2_rgb.encoding))

            break

    minimal_client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()