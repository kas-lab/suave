import rclpy
import sys

from rclpy.executors import MultiThreadedExecutor
from rclpy.lifecycle import Node
from rclpy.lifecycle import Publisher
from rclpy.lifecycle import State
from rclpy.lifecycle import TransitionCallbackReturn


from diagnostic_msgs.msg import DiagnosticArray
from diagnostic_msgs.msg import DiagnosticStatus
from diagnostic_msgs.msg import KeyValue
from rcl_interfaces.msg import Parameter
from rcl_interfaces.msg import ParameterType
from rcl_interfaces.srv import SetParameters


class RecoverThrustersLC(Node):
    def __init__(self, node_name, **kwargs):
        super().__init__(node_name, **kwargs)
        self.trigger_configure()

    def on_configure(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info('on_configure() is called.')
        self.diagnostics_publisher = self.create_publisher(
            DiagnosticArray, '/diagnostics', 10)
        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info("on_activate() is called.")
        self.executor.create_task(self.recover_thrusters)
        return super().on_activate(state)

    def on_deactivate(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info("on_deactivate() is called.")
        return super().on_deactivate(state)

    def on_cleanup(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info('on_cleanup() is called.')
        return TransitionCallbackReturn.SUCCESS

    def on_shutdown(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info('on_shutdown() is called.')
        return TransitionCallbackReturn.SUCCESS

    def call_service(self, srv_type, srv_name, request):
        service = self.create_client(srv_type, srv_name)
        while not service.wait_for_service(timeout_sec=1.0):
            self.get_logger().info(srv_name + ' not available, waiting...')
        future = service.call_async(request)
        return future

    def recover_thrusters(self):
        publish_rate = self.create_rate(4)
        rate = self.create_rate(0.1)
        rate.sleep()
        for thruster in range(1, 7):
            parameter = Parameter()
            parameter.name = 'SERVO' + str(thruster) + '_FUNCTION'
            parameter.value.type = ParameterType.PARAMETER_INTEGER
            parameter.value.integer_value = thruster + 32

            req = SetParameters.Request()
            req.parameters.append(parameter)

            self.call_service(
                SetParameters, 'mavros/param/set_parameters', req)

            key_value = KeyValue()
            key_value.key = 'c_thruster_{}'.format(thruster)
            key_value.value = 'RECOVERED'

            status_msg = DiagnosticStatus()
            status_msg.level = DiagnosticStatus.OK
            status_msg.name = ''
            status_msg.message = 'Component status'
            status_msg.values.append(key_value)

            diag_msg = DiagnosticArray()
            diag_msg.header.stamp = self.get_clock().now().to_msg()
            diag_msg.status.append(status_msg)

            # TODO: wait service to complete to send component state
            self.diagnostics_publisher.publish(diag_msg)
            publish_rate.sleep()
        self.get_logger().info("Thrusters recovered!")


def main():
    rclpy.init(args=sys.argv)
    recover_thrusters_node = RecoverThrustersLC('f_maintain_motion_node')
    mt_executor = MultiThreadedExecutor()
    rclpy.spin(recover_thrusters_node, mt_executor)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
