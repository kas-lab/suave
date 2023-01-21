import rclpy

from mros2_reasoner.ros_reasoner import RosReasoner
from rclpy.executors import MultiThreadedExecutor

from system_modes_msgs.srv import ChangeMode, GetAvailableModes
import threading
import numpy as np

class RandomReasoner(Node):
    def __init__(self):
        super().__init__('random_reasoner')

        timer_period = 5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.generate_path_changemode_cli = self.create_client(
                ChangeMode,
                '/f_generate_search_path/change_mode')

        self.inspect_pipeline_changemode_cli = self.create_client(
                ChangeMode,
                '/f_inspect_pipeline/change_mode')

        self.generate_path_availmodes_cli = self.create_client(
                GetAvailableModes,
                '/f_generate_search_path/get_available_modes')

        self.inspect_pipeline_availmodes_cli = self.create_client(
                GetAvailableModes,
                '/f_inspect_pipeline/get_available_modes')

        self.pipeline_detected = False
        self.pipeline_detected_sub = self.create_subscription(
            Bool, 'pipeline/detected', self.pipeline_detected_cb, 10)

        self.pipeline_inspected = False
        self.pipeline_inspected_sub = self.create_subscription(
            Bool, 'pipeline/inspected', self.pipeline_inspected_cb, 10)


    def pipeline_detected_cb(self, msg):
        self.pipeline_detected = msg.data

    def pipeline_inspected_cb(self, msg):
        self.pipeline_inspected = msg.data

    def timer_callback(self):
        self.get_logger().info('\nCalled!!!\n')
       
        if not self.pipeline_detected:
            self.get_logger().info('\Pipeline not detected!!!\n')

            available_modes = self.generate_path_availmodes_cli.call(GetAvailableModes.Request())
            
            self.get_logger().info('Available modes {}'.format(str(available_modes)))
            
            new_mode = np.random.choice(available_modes)
            
            self.get_logger().info('Random mode {}'.format(str(new_mode)))

            change_req = ChangeMode.Request()
            req.mode_name = new_mode
            is_success = self.generate_path_sm_cli.call(change_req)

            self.get_logger().info('Mode change success?: {}'.format(str(is_success)))

        
        if not self.pipeline_inspected:
            available_modes = self.inspect_pipeline_availmodes_cli.call(GetAvailableModes.Request())
            new_mode = np.random.choice(available_modes)

            change_req = ChangeMode.Request()
            req.mode_name = new_mode
            self.inspect_pipeline_changemode_cli.call(change_req)
        




def main(args=None):

    rclpy.init(args=args)

    random_reasoner = RandomReasoner()

    rclpy.spin(random_reasoner)

    # mt_executor = MultiThreadedExecutor() I don't know if it needs to multi-threaded or what

    # if random_reasoner.is_initialized is not True:
    #     random_reasoner.get_logger().info(
    #         "There was an error in the reasoner initialization")
    #     return

    # thread = threading.Thread(
    #     target=rclpy.spin,
    #     args=[random_reasoner, mt_executor],
    #     daemon=True)
    # thread.start()

    # thread.join()
    
    random_reasoner.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
