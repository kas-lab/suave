import rclpy
from rclpy.node import Node

from system_modes_msgs.srv import ChangeMode
from mros2_msgs.srv import NodeMode

class BridgeService(Node):
    def __init__(self):
        super.__init__('System_mode_bridge')
        self.srv = self.create_service(
                NodeMode,
                '/ros_reasoner/change_node_mode',
                self.change_mode_cb)

    def change_mode_cb(self, request, response):
        system_modes_cli = self.create_client(
                ChangeMode,
                '/' + request.node_name + '/change_mode',
                callback_group=ReentrantCallbackGroup()) 
        try:
            req = ChangeMode.Request()
            req.mode_name = request.mode_name
            response = system_modes_cli.call_async(req)
        except Exception as e:
            self.get_logger().info('Request creation failed %r' % (e,))
            return None
        else:
            return response

def main():
    rclpy.init()

    bridge_service = BridgeService()

    rclpy.spin(bridge_service)
    
    rclpy.shutdown()

if __name__ == '__main__':
    main()
