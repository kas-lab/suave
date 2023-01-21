import rclpy


from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from system_modes_msgs.srv import ChangeMode, GetAvailableModes
import threading
import numpy as np
from std_msgs.msg import Bool


class RandomReasoner(Node):
    def __init__(self):
        super().__init__('random_reasoner')

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

        self.dat_req = GetAvailableModes.Request()
        print(self.dat_req)
            #available_modes = self.generate_path_availmodes_cli.call(dat_req)

        self.detect_modes = self.send_request(self.generate_path_availmodes_cli).available_modes
        self.inspect_modes = self.send_request(self.inspect_pipeline_availmodes_cli).available_modes

        self.get_logger().info('Available modes detect {}'.format(str(self.detect_modes)))
        self.get_logger().info('Available modes inspect {}'.format(str(self.inspect_modes)))


        

    def send_request(self, cli):
        while not cli.wait_for_service():
            self.get_logger().info('service not available, waiting again...')
        self.future = cli.call_async(self.dat_req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()

    def pipeline_detected_cb(self, msg):
        self.pipeline_detected = msg.data

    def pipeline_inspected_cb(self, msg):
        self.pipeline_inspected = msg.data

    def change_mode_request(self):
        self.get_logger().info('\nCalled!!!\n')
        #timer = self.create_rate(0.1)
        #timer.sleep()
       
        if not self.pipeline_detected:
            self.get_logger().info('\Pipeline not detected!!!\n')
            change_client = self.generate_path_changemode_cli
            modes = self.detect_modes
        
        if not self.pipeline_inspected:
            self.get_logger().info('\Pipeline detected!!!\n')
            change_client = self.inspect_pipeline_changemode_cli
            modes = self.inspect_modes


        new_mode = np.random.choice(modes)
        
        self.get_logger().info('Random mode {}'.format(str(new_mode)))

        change_req = ChangeMode.Request()
        change_req.mode_name = new_mode
        self.future = change_client.call_async(change_req)

        rclpy.spin_until_future_complete(self, self.future)            
        return self.future.result()





def main(args=None):

    rclpy.init(args=args)

    random_reasoner = RandomReasoner()


    #while not random_reasoner.pipeline_inspected:
    response = random_reasoner.change_mode_request()
    random_reasoner.get_logger().info('Change mode success? {}'.format(str(response)))
        

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
    #thread.join()


if __name__ == '__main__':
    main()
