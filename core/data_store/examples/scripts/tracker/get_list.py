from custom_interfaces.srv import DataStoreTrackerGetList
import rclpy
from rclpy.node import Node


class MinimalClientAsync(Node):

    def __init__(self):
        super().__init__('tracker_get_list_client_async')
        self.cli = self.create_client(DataStoreTrackerGetList, 'tracker_get_list')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')item_status
        self.req = DataStoreTrackerGetList.Request()

    def send_request(self):
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
                if len(response.time_stamps)!=0:
                    for time_stamp in response.time_stamps:
                        minimal_client.get_logger().info(str(time_stamp))
                else:
                    minimal_client.get_logger().info(str(response.time_stamps))
            break

    minimal_client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()