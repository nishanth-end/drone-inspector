"""
PX4 offboard mission: fly a lawnmower patrol over a predefined rectangle.

Flow: warm up -> take off -> visit every waypoint in order -> return home -> land.
While flying it publishes JSON progress events to the ground station over UDP.
Run in SITL first. Real flight needs a separate safety review.
"""

import math
from enum import Enum, auto

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_srvs.srv import Trigger

from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleCommand, VehicleOdometry

from px4_orbit_inspection.area_patrol import lawnmower_waypoints
from px4_orbit_inspection.ground_link import GroundLink, make_event


class State(Enum):
    WAITING = auto()
    WARMUP = auto()
    CLIMB = auto()
    PATROL = auto()
    RETURN = auto()
    LAND = auto()
    FINISHED = auto()


class AreaPatrol(Node):

    def __init__(self):
        super().__init__('area_patrol_mission')

        defaults = {
            'north_min_m': -20.0, 'north_max_m': 20.0,
            'east_min_m': -20.0, 'east_max_m': 20.0,
            'lane_spacing_m': 8.0, 'altitude_m': 6.0,
            'speed_m_s': 2.0, 'acceptance_m': 1.0,
            'setpoint_rate_hz': 20.0, 'telemetry_rate_hz': 1.0,
            'ground_host': '127.0.0.1', 'ground_port': 14560,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        self.p = {name: self.get_parameter(name).value for name in defaults}

        self.waypoints = lawnmower_waypoints(
            self.p['north_min_m'], self.p['north_max_m'],
            self.p['east_min_m'], self.p['east_max_m'],
            self.p['lane_spacing_m'], self.p['altitude_m'])
        self.index = 0

        self.state = State.WAITING
        self.warmup_ticks = 0
        self.measured = (0.0, 0.0, 0.0)  # NED position from PX4 odometry
        self.setpoint = (0.0, 0.0, 0.0)  # NED position we currently command
        self.target = (0.0, 0.0, -self.p['altitude_m'])
        self.yaw = 0.0

        self.link = GroundLink(self.p['ground_host'], self.p['ground_port'])

        sensor_qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                                durability=DurabilityPolicy.TRANSIENT_LOCAL,
                                history=HistoryPolicy.KEEP_LAST, depth=1)

        self.heartbeat_pub = self.create_publisher(OffboardControlMode, '/fmu/in/offboard_control_mode', 10)
        self.setpoint_pub = self.create_publisher(TrajectorySetpoint, '/fmu/in/trajectory_setpoint', 10)
        self.command_pub = self.create_publisher(VehicleCommand, '/fmu/in/vehicle_command', 10)
        self.odom_sub = self.create_subscription(VehicleOdometry, '/fmu/out/vehicle_odometry', self._odom, sensor_qos)

        self.start_srv = self.create_service(Trigger, '/patrol/start', self._start)

        self.create_timer(1.0 / float(self.p['setpoint_rate_hz']), self._tick)
        self.create_timer(1.0 / float(self.p['telemetry_rate_hz']), self._telemetry)

        self.get_logger().info(f'Patrol ready: {len(self.waypoints)} waypoints. Call /patrol/start to fly.')

    # ----- time and message helpers -------------------------------------------
    def _now(self): return self.get_clock().now().nanoseconds / 1e9
    def _stamp_us(self): return int(self.get_clock().now().nanoseconds / 1000)

    def _odom(self, msg):
        self.measured = tuple(float(v) for v in msg.position)

    def _start(self, request, response):
        if self.state is not State.WAITING:
            response.success, response.message = False, 'Patrol already started.'
            return response
        self.state = State.WARMUP
        response.success, response.message = True, 'Patrol started.'
        return response

    def _heartbeat(self):
        m = OffboardControlMode(); m.timestamp = self._stamp_us(); m.position = True
        for field in ('velocity', 'acceleration', 'attitude', 'body_rate', 'thrust_and_torque', 'direct_actuator'):
            if hasattr(m, field): setattr(m, field, False)
        self.heartbeat_pub.publish(m)

    def _send_setpoint(self, n, e, d, yaw):
        self.setpoint = (float(n), float(e), float(d))
        m = TrajectorySetpoint(); m.timestamp = self._stamp_us(); m.position = list(self.setpoint)
        m.velocity = [math.nan] * 3; m.acceleration = [math.nan] * 3; m.jerk = [math.nan] * 3
        m.yaw = float(yaw); m.yawspeed = math.nan
        self.setpoint_pub.publish(m)

    def _command(self, command, p1=0.0, p2=0.0):
        m = VehicleCommand(); m.timestamp = self._stamp_us(); m.command = int(command)
        m.param1 = float(p1); m.param2 = float(p2); m.target_system = 1; m.target_component = 1
        m.source_system = 1; m.source_component = 1; m.from_external = True
        self.command_pub.publish(m)

    def _offboard_and_arm(self):
        self._command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1.0, 6.0)
        self._command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0, 0.0)

    # ----- motion ---------------------------------------------------------------
    def _distance_to(self, point, here):
        return math.sqrt(sum((a - b) ** 2 for a, b in zip(point, here)))

    def _move_toward(self, target, dt):
        """Step the setpoint toward target at speed_m_s and return True once it has arrived."""
        dn, de, dd = (t - s for t, s in zip(target, self.setpoint))
        dist = math.sqrt(dn * dn + de * de + dd * dd)
        step = float(self.p['speed_m_s']) * dt
        if dist > step:
            k = step / dist
            new = (self.setpoint[0] + dn * k, self.setpoint[1] + de * k, self.setpoint[2] + dd * k)
        else:
            new = target
        # Face the direction of travel when moving horizontally.
        if abs(dn) + abs(de) > 1e-6:
            self.yaw = math.atan2(de, dn)
        self._send_setpoint(*new, self.yaw)
        return new == target and self._distance_to(target, self.measured) <= float(self.p['acceptance_m'])

    # ----- state machine --------------------------------------------------------
    def _tick(self):
        if self.state in (State.WAITING, State.FINISHED):
            return
        self._heartbeat()
        dt = 1.0 / float(self.p['setpoint_rate_hz'])
        alt = float(self.p['altitude_m'])

        if self.state is State.WARMUP:
            self._send_setpoint(0.0, 0.0, -0.5, 0.0)
            self.warmup_ticks += 1
            if self.warmup_ticks >= int(float(self.p['setpoint_rate_hz'])):
                self._offboard_and_arm()
                self.state = State.CLIMB
                self.get_logger().info('Offboard and armed. Climbing.')

        elif self.state is State.CLIMB:
            if self._move_toward((0.0, 0.0, -alt), dt):
                self.state = State.PATROL
                self.index = 0
                self.target = self._waypoint_ned(0)
                self.get_logger().info(f'Patrol: waypoint 1/{len(self.waypoints)}')

        elif self.state is State.PATROL:
            if self._move_toward(self.target, dt):
                self.index += 1
                if self.index >= len(self.waypoints):
                    self.state = State.RETURN
                    self.target = (0.0, 0.0, -alt)
                    self.get_logger().info('Patrol complete. Returning home.')
                else:
                    self.target = self._waypoint_ned(self.index)
                    self.get_logger().info(f'Patrol: waypoint {self.index + 1}/{len(self.waypoints)}')

        elif self.state is State.RETURN:
            if self._move_toward(self.target, dt):
                self.state = State.LAND

        elif self.state is State.LAND:
            self._command(VehicleCommand.VEHICLE_CMD_NAV_LAND)
            self.state = State.FINISHED
            self.get_logger().info('Landed. Mission finished.')

    def _waypoint_ned(self, i):
        n, e, alt = self.waypoints[i]
        return (n, e, -alt)

    def _telemetry(self):
        if self.state is State.WAITING:
            return
        self.link.send(make_event(
            'progress', state=self.state.name, waypoint=min(self.index + 1, len(self.waypoints)),
            total=len(self.waypoints), north_m=round(self.measured[0], 2),
            east_m=round(self.measured[1], 2), altitude_m=round(-self.measured[2], 2)))


def main(args=None):
    rclpy.init(args=args)
    node = AreaPatrol()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
