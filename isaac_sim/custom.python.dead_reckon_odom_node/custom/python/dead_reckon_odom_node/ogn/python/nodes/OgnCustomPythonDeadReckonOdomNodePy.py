"""
Dead Reckoning Odometry Node for Differential Drive Robot

Calculates robot pose from wheel encoder positions using dead reckoning.
"""

import math

from isaacsim.core.nodes import BaseResetNode
# from custom.python.dead_reckon_odom_node.ogn.OgnCustomPythonDeadReckonOdomNodePyDatabase import OgnCustomPythonDeadReckonOdomNodePyDatabase


class OgnCustomPythonDeadReckonOdomNodePyInternalState(BaseResetNode):
    """Maintains per-node state for dead reckoning odometry.

    Inherits from BaseResetNode to reset pose when timeline is stopped."""

    def __init__(self):
        """Initialize state variables"""
        self.initialized = False
        self.prev_left_pos = 0.0
        self.prev_right_pos = 0.0
        # Accumulated pose
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        # Call parent class to set up timeline event for custom reset
        super().__init__(initialize=False)

    def custom_reset(self):
        """Reset pose when timeline is stopped."""
        self.initialized = False
        self.prev_left_pos = 0.0
        self.prev_right_pos = 0.0
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0


def normalize_angle_delta(current: float, previous: float) -> float:
    """
    Calculate angle delta handling wrap-around from -2pi to 2pi.

    Args:
        current: Current wheel position in radians
        previous: Previous wheel position in radians

    Returns:
        The shortest angular difference
    """
    delta = current - previous
    # Handle wrap-around (positions range from -2pi to 2pi, so full range is 4pi)
    while delta > math.pi:
        delta -= 2.0 * math.pi
    while delta < -math.pi:
        delta += 2.0 * math.pi
    return delta


def euler_to_quaternion(yaw: float):
    """
    Convert yaw angle (rotation around Z-axis) to quaternion.

    Args:
        yaw: Rotation angle around Z-axis in radians

    Returns:
        Tuple (x, y, z, w) quaternion
    """
    half_yaw = yaw * 0.5
    qx = 0.0
    qy = 0.0
    qz = math.sin(half_yaw)
    qw = math.cos(half_yaw)
    return (qx, qy, qz, qw)


class OgnCustomPythonDeadReckonOdomNodePy:
    """Dead reckoning odometry node for differential drive robot"""

    @staticmethod
    def internal_state():
        """Returns an object that contains per-node state information"""
        return OgnCustomPythonDeadReckonOdomNodePyInternalState()

    @staticmethod
    def compute(db) -> bool:
        """Compute odometry based on wheel positions"""
        state = db.per_instance_state

        try:
            # Read inputs
            # dt = db.inputs.dt
            wheel_pos = db.inputs.wheel_pos
            wheel_dist = db.inputs.wheel_dist
            wheel_rad = db.inputs.wheel_rad

            # Validate inputs
            if len(wheel_pos) < 2:
                db.log_error("wheel_pos must have at least 2 elements [left, right]")
                return False

            left_pos = wheel_pos[0]
            right_pos = wheel_pos[1]

            # Initialize on first run
            if not state.initialized:
                state.prev_left_pos = left_pos
                state.prev_right_pos = right_pos
                state.initialized = True
                # Output initial pose
                db.outputs.pos = [0.0, 0.0, 0.0]
                db.outputs.orient = [0.0, 0.0, 0.0, 1.0]
                return True

            # Calculate wheel angle deltas (handling wrap-around)
            delta_left = normalize_angle_delta(left_pos, state.prev_left_pos)
            delta_right = normalize_angle_delta(right_pos, state.prev_right_pos)

            # Convert angle deltas to linear displacements
            left_displacement = delta_left * wheel_rad
            right_displacement = delta_right * wheel_rad

            # Calculate robot motion
            linear_displacement = (left_displacement + right_displacement) / 2.0
            angular_displacement = (right_displacement - left_displacement) / wheel_dist

            # Update pose using midpoint integration
            # Use the average orientation during the motion for better accuracy
            theta_mid = state.theta + angular_displacement / 2.0

            state.x += linear_displacement * math.cos(theta_mid)
            state.y += linear_displacement * math.sin(theta_mid)
            state.theta += angular_displacement

            # Normalize theta to [-pi, pi]
            while state.theta > math.pi:
                state.theta -= 2.0 * math.pi
            while state.theta < -math.pi:
                state.theta += 2.0 * math.pi

            # Store current positions for next iteration
            state.prev_left_pos = left_pos
            state.prev_right_pos = right_pos

            # Output position [x, y, z]
            db.outputs.pos = [state.x, state.y, 0.0]

            # Output orientation as quaternion [x, y, z, w]
            qx, qy, qz, qw = euler_to_quaternion(state.theta)
            db.outputs.orient = [qx, qy, qz, qw]

        except Exception as e:
            db.log_error(f"Computation error: {e}")
            return False

        return True

    @staticmethod
    def release(node):
        """Release per-node state information."""
        try:
            # state = OgnCustomPythonDeadReckonOdomNodePyDatabase.per_instance_internal_state(node)
            pass
        except Exception:
            return
        # Reset state
        state.reset()
        state.initialized = False
