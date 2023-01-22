from geometry_msgs.msg import Pose
from std_msgs.msg import Bool
from mavros_wrapper.ardusub_wrapper import BlueROVArduSubWrapper


class BlueROVGazebo(BlueROVArduSubWrapper):
    def __init__(self, node_name='bluerov_gz'):
        super().__init__(node_name)
        self.gz_to_local_pose_delta = None
        self.first_gz_pose = True
        self.initial_x = None
        self.initial_y = None

        self.gazebo_pos_sub = self.create_subscription(
            Pose, 'model/bluerov2/pose', self.gazebo_pos_cb, 10)

        # TODO: make this a ros param
        self.ground_depth_gz = -20
        self.altitude = 1.25

    def gazebo_pos_cb(self, msg):
        self.gazebo_pos = msg
        if self.first_gz_pose is True:
            self.first_gz_pose = False
            self.initial_x = msg.position.x
            self.initial_y = msg.position.y

        if self.local_pos_received and self.status.mode == 'GUIDED':
            self.gz_to_local_pose_delta = [
                self.local_pos.pose.position.x - msg.position.x,
                self.local_pos.pose.position.y - msg.position.y,
                self.local_pos.pose.position.z - msg.position.z,
            ]
            self.destroy_subscription(self.gazebo_pos_sub)

    def convert_gz_to_local_pose(self, gz_pose):
        if self.gz_to_local_pose_delta is not None:
            local_pose = Pose()
            local_pose.position.x = \
                gz_pose.position.x + self.gz_to_local_pose_delta[0]
            local_pose.position.y = \
                gz_pose.position.y + self.gz_to_local_pose_delta[1]
            local_pose.position.z = \
                gz_pose.position.z + self.gz_to_local_pose_delta[2]
            return local_pose
        else:
            return gz_pose

    def setpoint_position_gz(self, gz_pose, fixed_altitude=True):
        if self.gz_to_local_pose_delta is None:
            return None

        if fixed_altitude:
            gz_pose.position.z = self.ground_depth_gz + self.altitude

        local_pose = self.convert_gz_to_local_pose(gz_pose)
        return self.setpoint_position_local(
            x=local_pose.position.x,
            y=local_pose.position.y,
            z=local_pose.position.z)

    def setpoint_position_local(
     self, x=.0, y=.0, z=.0, rx=.0, ry=.0, rz=.0, rw=1.0, fixed_altitude=True):
        if fixed_altitude and self.gz_to_local_pose_delta is None:
            return None

        if fixed_altitude:
            z = self.ground_depth_gz + self.altitude \
                + self.gz_to_local_pose_delta[2]
        return super().setpoint_position_local(x, y, z)
