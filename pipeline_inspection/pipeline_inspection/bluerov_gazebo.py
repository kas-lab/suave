from geometry_msgs.msg import Pose
from std_msgs.msg import Bool
from mavros_wrapper.ardusub_wrapper import BlueROVArduSubWrapper


class BlueROVGazebo(BlueROVArduSubWrapper):
    def __init__(self, node_name='bluerov_gz'):
        super().__init__(node_name)
        self.gazebo_pos_sub = self.create_subscription(
            Pose, 'model/bluerov2/pose', self.gazebo_pos_cb, 10)
        self.first_gazebo_pos_msg = True

        # TODO: make this a ros param
        self.ground_depth_gz = -20
        self.altitude = 1.25

        # TODO: REMOVE this
        self.start_follow = False
        # TODO: REMOVE this
        self.start_follow_sub = self.create_subscription(
            Bool, 'start_follow_pipe', self.follow_pipe_cb, 10)

    def gazebo_pos_cb(self, msg):
        self.gazebo_pos = msg
        if self.first_gazebo_pos_msg and self.local_pos_received:
            self.gz_local_delta_pos = [
                self.local_pos.pose.position.x - msg.position.x,
                self.local_pos.pose.position.y - msg.position.y,
                self.local_pos.pose.position.z - msg.position.z,
            ]
            self.first_gazebo_pos_msg = False

    def convert_gz_to_local_pos(self, gz_pose):
        local_pose = Pose()
        local_pose.position.x = gz_pose.position.x + self.gz_local_delta_pos[0]
        local_pose.position.y = gz_pose.position.y + self.gz_local_delta_pos[1]
        local_pose.position.z = gz_pose.position.z + self.gz_local_delta_pos[2]

        return local_pose

    def setpoint_position_gz(self, gz_pose, fixed_altitude=True):
        if fixed_altitude:
            gz_pose.position.z = self.ground_depth_gz + self.altitude

        local_pose = self.convert_gz_to_local_pos(gz_pose)
        return self.setpoint_position_local(
            x=local_pose.position.x,
            y=local_pose.position.y,
            z=local_pose.position.z)

    def setpoint_position_local(
     self, x=.0, y=.0, z=.0, rx=.0, ry=.0, rz=.0, rw=1.0, fixed_altitude=True):
        if fixed_altitude:
            z = self.ground_depth_gz + self.altitude \
                + self.gz_local_delta_pos[2]
        return super().setpoint_position_local(x, y, z)

    # TODO: REMOVE this
    def follow_pipe_cb(self, msg):
        self.start_follow = msg.data
