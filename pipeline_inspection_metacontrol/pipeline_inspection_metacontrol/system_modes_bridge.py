import rclpy
from rclpy.node import Node

from system_modes_msgs.srv import ChangeMode
from mros2_msgs.srv import MetacontrolFD

# TODO: Make everything fit together: node names, mode names, function_design
# names. Currently there is only one node available and the fd names do not
# match with the system_modes. Files to change: sytem_modes config file, the
# dict of this node, the ontology. 

class BridgeService(Node):
    def __init__(self):
        super().__init__('System_mode_bridge')
        self.srv = self.create_service(
                MetacontrolFD,
                '/ros_reasoner/change_node_mode',
                self.change_mode_cb)
        # Dictionary to map fd names to system_mode modes and nodes
        self.node_mapping = {'fd_spiral_low': ('spiral_lc_node', 'SLOW'),
                'fd_spiral_medium': ('spiral_lc_node', '__DEFAULT__'),
                'fd_spiral_high': ('spiral_lc_node', 'FAST')
                }
    
    def change_mode_cb(self, request, response):
        # Get node name and mode name from dictionary
        if request.required_fd_name in self.node_mapping:
            (node_name, mode_name) = self.node_mapping[request.reqired_fd_name]
        else:
            (node_name, mode_name) = self.node_mapping['fd_spiral_medium']

        system_modes_cli = self.create_client(
                ChangeMode,
                '/' + node_name + '/change_mode',
                callback_group=ReentrantCallbackGroup()) 

        try:
            req = ChangeMode.Request()
            req.mode_name = mode_name
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
