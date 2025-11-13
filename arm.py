"""
5DOF+Gripper RRRRR Robot Arm Simulator
This module provides a class to represent and control a 5-degree-of-freedom
robot arm with a gripper end-effector.
"""

import time
import numpy as np
from typing import Dict, Tuple, Optional, List, Any
from dataclasses import dataclass


@dataclass
class JointLimits:
    """Joint limits in servo units (0-1000)"""
    min: int = 0
    max: int = 1000
    neutral: int = 500


@dataclass
class LinkDimensions:
    """Link dimensions in millimeters"""
    L1: float = 110.0  # J2 to end-of-gripper
    L2: float = 58.0   # J3 to J2
    L3: float = 95.0   # J4 to J3
    L4: float = 109.0  # J5 to J4
    base_height: float = 58.0  # Ground to J5 center
    gripper_max_width: float = 98.0  # Maximum opening width between fingers
    gripper_min_width: float = 55.0  # Minimum opening width (closed)


class RobotArm:
    """
    Represents a 5DOF+Gripper RRRRR robot arm with end-effector gripper.
    
    Joint Configuration:
    - J1: Gripper (open/close)
    - J2: Gripper Roll (rotation)
    - J3: Wrist (pitch)
    - J4: Elbow (pitch)
    - J5: Shoulder (pitch)
    - J6: Base Yaw (rotation)
    
    Servo units range from 0-1000, with 500 typically being neutral position.
    """
    
    def __init__(self):
        """Initialize the robot arm with default neutral positions."""
        # Joint positions in servo units (0-1000)
        self.joints: Dict[str, int] = {
            'J1': 500,  # Gripper
            'J2': 500,  # Gripper Roll
            'J3': 500,  # Wrist
            'J4': 500,  # Elbow
            'J5': 500,  # Shoulder
            'J6': 500,  # Base Yaw
        }
        
        # Link dimensions
        self.links = LinkDimensions()
        
        # Joint limits
        self.limits = JointLimits()
        
        # Base dimensions (mm)
        self.base_length = 291.0
        self.base_width = 159.0
        self.base_j5j6_offset = 80.0  # Distance from short edge
        
        # Workspace constraints
        self.restricted_zone_enabled = True
        self.keep_gripper_horizontal = False
        
        # Hardware integration
        self.hardware_board: Optional[Any] = None
        self.hardware_enabled = False
        self.hardware_speed = 0.5
        self.Board = None
        self.BOARD_AVAILABLE = False
        self._setup_board_class()

    def _setup_board_class(self):
        """Lazy import the hardware Board SDK if available."""
        if self.Board is not None:
            return
        try:
            from ros_robot_controller_sdk import Board  # type: ignore
            self.Board = Board
            self.BOARD_AVAILABLE = True
        except Exception:
            self.Board = None
            self.BOARD_AVAILABLE = False
        
    def set_joint(self, joint_name: str, value: int) -> bool:
        """
        Set a joint to a specific servo value.
        
        Args:
            joint_name: Joint identifier (J1-J6)
            value: Servo value (0-1000)
            
        Returns:
            True if successful, False if invalid
        """
        if joint_name not in self.joints:
            print(f"Error: Invalid joint name '{joint_name}'")
            return False
            
        if not self.limits.min <= value <= self.limits.max:
            print(f"Error: Value {value} out of range [{self.limits.min}, {self.limits.max}]")
            return False
            
        self.joints[joint_name] = value
        self._sync_hardware(targets=[joint_name])
        return True
    
    def attach_hardware(self, board: Any = None, speed: float = 0.5, auto_initialize: bool = True):
        """
        Attach a hardware board for real-servo control.
        
        Args:
            board: Instance compatible with ros_robot_controller_sdk.Board.
            speed: Movement speed parameter passed to the SDK (0-1).
            auto_initialize: When True, call servo_control.init_servo_positions.
        """
        self._setup_board_class()
        if not self.BOARD_AVAILABLE and board is None:
            print("Board SDK not available; cannot attach hardware.")
            return False
        
        if board is None:
            board = self.Board()
        
        self.hardware_board = board
        self.hardware_speed = float(speed)
        self.hardware_enabled = True
        
        if auto_initialize:
            self._init_servo_positions()
        else:
            self._sync_hardware()
        return True
    
    def detach_hardware(self):
        """Detach any connected hardware board."""
        self.hardware_board = None
        self.hardware_enabled = False
    
    def _sync_hardware(self, targets: Optional[List[str]] = None):
        """
        Push the current joint positions to the hardware when enabled.
        """
        if not (self.hardware_enabled and self.hardware_board):
            return
        
        joint_order = ['J6', 'J5', 'J4', 'J3', 'J2', 'J1']
        if targets is None:
            selected = joint_order
        else:
            selected = [name for name in joint_order if name in targets]
        for joint_name in selected:
            servo_id = int(joint_name[1])
            value = self.joints[joint_name]
            self._set_servo_position(servo=servo_id, position=value)
    
    def get_joint(self, joint_name: str) -> Optional[int]:
        """
        Get the current value of a joint.
        
        Args:
            joint_name: Joint identifier (J1-J6)
            
        Returns:
            Current servo value or None if invalid joint
        """
        return self.joints.get(joint_name)
    
    def set_all_joints(self, j1: int, j2: int, j3: int, j4: int, j5: int, j6: int) -> bool:
        """
        Set all joints at once.
        
        Args:
            j1-j6: Servo values for each joint (0-1000)
            
        Returns:
            True if all joints set successfully
        """
        values = [j1, j2, j3, j4, j5, j6]
        joint_names = ['J1', 'J2', 'J3', 'J4', 'J5', 'J6']
        
        # Validate all values first
        for value in values:
            if not self.limits.min <= value <= self.limits.max:
                print(f"Error: Value {value} out of range")
                return False
        
        # Set all joints
        for name, value in zip(joint_names, values):
            self.joints[name] = value
        self._sync_hardware()
        
        return True
    
    def reset_to_neutral(self):
        """Reset all joints to neutral position (500)."""
        for joint in self.joints:
            self.joints[joint] = self.limits.neutral
        self._sync_hardware()
    
    def servo_to_radians(self, servo_value: int, joint_name: str) -> float:
        """
        Convert servo value to radians based on joint characteristics.
        
        Most joints have a range of 135° on each side of neutral (270° total).
        
        Args:
            servo_value: Servo value (0-1000)
            joint_name: Joint identifier for context
            
        Returns:
            Angle in radians
        """
        # Normalize to -1 to 1 range (500 = 0)
        normalized = (servo_value - 500) / 500.0
        
        # 135° = 2.356 radians
        max_angle = np.deg2rad(135)
        
        return normalized * max_angle
    
    def radians_to_servo(self, angle_rad: float) -> int:
        """
        Convert radians to servo value.
        
        Args:
            angle_rad: Angle in radians (-2.356 to 2.356 for ±135°)
            
        Returns:
            Servo value (0-1000)
        """
        max_angle = np.deg2rad(135)
        normalized = np.clip(angle_rad / max_angle, -1.0, 1.0)
        servo_value = int(normalized * 500 + 500)
        return np.clip(servo_value, self.limits.min, self.limits.max)
    
    def get_joint_angles_radians(self) -> Dict[str, float]:
        """
        Get all joint angles in radians.
        
        Returns:
            Dictionary of joint names to angles in radians
        """
        angles = {}
        for joint_name, servo_value in self.joints.items():
            angles[joint_name] = self.servo_to_radians(servo_value, joint_name)
        return angles
    
    def forward_kinematics(self) -> Tuple[float, float, float]:
        """
        Calculate the end-effector position using forward kinematics.
        
        Returns:
            Tuple of (x, y, z) coordinates in millimeters
        """
        angles = self.get_joint_angles_radians()
        
        # Base yaw rotation (J6)
        theta6 = angles['J6']
        
        # Shoulder pitch (J5)
        theta5 = angles['J5']
        
        # Elbow pitch (J4)
        theta4 = angles['J4']
        
        # Wrist pitch (J3)
        theta3 = angles['J3']
        
        # Calculate position in the vertical plane (assuming base at neutral)
        # Starting from base height
        z = self.links.base_height
        
        # Shoulder link (L4) - J5 to J4
        y_plane = self.links.L4 * np.sin(theta5)
        z += self.links.L4 * np.cos(theta5)
        
        # Elbow link (L3) - J4 to J3
        # J4 moves in opposite direction: 0=+Y, 1000=-Y (inverted from J5)
        cumulative_angle = theta5 - theta4
        y_plane += self.links.L3 * np.sin(cumulative_angle)
        z += self.links.L3 * np.cos(cumulative_angle)
        
        # Wrist link (L2) - J3 to J2
        cumulative_angle += theta3
        y_plane += self.links.L2 * np.sin(cumulative_angle)
        z += self.links.L2 * np.cos(cumulative_angle)
        
        # Gripper link (L1) - J2 to end
        # J2 only rotates the gripper, does not change position
        y_plane += self.links.L1 * np.sin(cumulative_angle)
        z += self.links.L1 * np.cos(cumulative_angle)
        
        # Apply base yaw rotation to get x, y from y_plane
        x = y_plane * np.sin(theta6)
        y = y_plane * np.cos(theta6)
        
        return (x, y, z)

    def set_restricted_zone(self, enabled: bool):
        """Enable or disable avoidance of the base volume and negative heights."""
        self.restricted_zone_enabled = bool(enabled)
    
    def set_gripper_horizontal_mode(self, enabled: bool):
        """Force the gripper to stay parallel to the ground when enabled."""
        self.keep_gripper_horizontal = bool(enabled)

    def _joint_positions(self, angles: Dict[str, float]) -> List[np.ndarray]:
        """
        Compute cartesian positions of key joints (J5 base through end effector).
        Returns list [base(J5/J6), J4, J3, J2, end-effector].
        """
        base_pos = np.array([0.0, 0.0, self.links.base_height])
        positions = [base_pos]

        theta6 = angles['J6']
        theta5 = angles['J5']
        theta4 = angles['J4']
        theta3 = angles['J3']

        y_plane = self.links.L4 * np.sin(theta5)
        z = base_pos[2] + self.links.L4 * np.cos(theta5)
        j4_pos = np.array([y_plane * np.sin(theta6), y_plane * np.cos(theta6), z])
        positions.append(j4_pos)

        cumulative_angle = theta5 - theta4
        y_plane += self.links.L3 * np.sin(cumulative_angle)
        z += self.links.L3 * np.cos(cumulative_angle)
        j3_pos = np.array([y_plane * np.sin(theta6), y_plane * np.cos(theta6), z])
        positions.append(j3_pos)

        cumulative_angle += theta3
        y_plane += self.links.L2 * np.sin(cumulative_angle)
        z += self.links.L2 * np.cos(cumulative_angle)
        j2_pos = np.array([y_plane * np.sin(theta6), y_plane * np.cos(theta6), z])
        positions.append(j2_pos)

        y_plane += self.links.L1 * np.sin(cumulative_angle)
        z += self.links.L1 * np.cos(cumulative_angle)
        end_pos = np.array([y_plane * np.sin(theta6), y_plane * np.cos(theta6), z])
        positions.append(end_pos)

        return positions

    def _is_inside_base_volume(self, position: np.ndarray) -> bool:
        """
        Check if a point is inside the rectangular prism occupied by the base platform.
        """
        x, y, z = position
        if not (0.0 <= z <= self.links.base_height):
            return False
        if not (-self.base_width / 2 <= x <= self.base_width / 2):
            return False
        return -self.base_j5j6_offset <= y <= (self.base_length - self.base_j5j6_offset)

    def _violates_restricted_zone(self, angles: Dict[str, float]) -> bool:
        """
        Determine whether a configuration enters the forbidden base volume or dips below ground.
        """
        if not self.restricted_zone_enabled:
            return False

        positions = self._joint_positions(angles)
        # Skip the base joint since it sits exactly at the boundary of the base
        for pos in positions[1:]:
            if pos[2] < -1e-6:
                return True
            if pos[1] > 1e-6:
                return True
            if self._is_inside_base_volume(pos):
                return True
        return False

    @staticmethod
    def _normalize_angle(angle: float) -> float:
        """Normalize angle to [-pi, pi)."""
        return (angle + np.pi) % (2 * np.pi) - np.pi

    def _candidate_wrist_angles(self, dx: float, dz: float, current_angles: Dict[str, float]) -> List[float]:
        """
        Generate wrist pitch candidates (cumulative angle after J3) to explore IK solutions.
        """
        candidates: List[float] = []
        target_angle = np.arctan2(dx, dz)
        current_cumulative = current_angles['J5'] - current_angles['J4'] + current_angles['J3']
        seed_angles = [
            target_angle,
            target_angle + np.pi,
            target_angle + np.pi / 2,
            target_angle - np.pi / 2,
            current_cumulative,
            current_cumulative + np.pi / 2,
            current_cumulative - np.pi / 2,
            0.0,
        ]

        if self.keep_gripper_horizontal:
            # Prioritize horizontal orientations (±90°) with small offsets
            offsets = [0.0, np.deg2rad(5), -np.deg2rad(5), np.deg2rad(10), -np.deg2rad(10)]
            horizontal_targets = [np.pi / 2, -np.pi / 2]
            seed_angles = []
            for base_angle in horizontal_targets:
                for offset in offsets:
                    seed_angles.append(base_angle + offset)
        else:
            # Add a dense sweep across the circle to avoid missing feasible orientations
            for extra in np.linspace(-np.pi, np.pi, num=360, endpoint=False):
                seed_angles.append(extra)

        for angle in seed_angles:
            normalized = self._normalize_angle(angle)
            if all(abs(self._normalize_angle(normalized - existing)) > 1e-3 for existing in candidates):
                candidates.append(normalized)

        return candidates

    def _score_solution(self, servos: Dict[str, int]) -> Tuple[float, float, int]:
        """
        Score a servo configuration based on path length, smoothness, and number of joints moved.
        Returns a tuple that can be compared lexicographically.
        """
        tracked_joints = ['J3', 'J4', 'J5', 'J6']
        deltas = [abs(servos[j] - self.joints[j]) for j in tracked_joints]
        total_abs = float(sum(deltas))
        smoothness = float(sum(delta ** 2 for delta in deltas))
        joint_changes = sum(1 for delta in deltas if delta > 5)
        return (total_abs, smoothness, joint_changes)

    def _enumerate_ik_solutions(self, x: float, y: float, z: float) -> List[Dict]:
        """
        Enumerate feasible IK solutions for the requested cartesian position.
        Each solution includes joint angles, servo targets, and a smoothness score.
        """
        solutions: List[Dict] = []
        angles_current = self.get_joint_angles_radians()

        radius = np.hypot(x, y)
        dz = z - self.links.base_height
        max_angle = np.deg2rad(135)

        yaw_candidates: List[Tuple[float, float]] = []
        if radius <= 1e-6:
            yaw_candidates.append((angles_current['J6'], 0.0))
        else:
            theta_primary = self._normalize_angle(np.arctan2(x, y))
            options = [
                (theta_primary, radius),
                (self._normalize_angle(theta_primary + np.pi), -radius),
            ]
            seen_theta = set()
            for theta6, dx in options:
                key = round(theta6, 6)
                if key in seen_theta:
                    continue
                seen_theta.add(key)
                if abs(theta6) <= max_angle + 1e-6:
                    yaw_candidates.append((theta6, dx))
        if not yaw_candidates:
            return []

        yaw_candidates.sort(key=lambda item: abs(self._normalize_angle(item[0] - angles_current['J6'])))

        l1 = self.links.L4
        l2 = self.links.L3
        l3 = self.links.L2 + self.links.L1

        seen_configs = set()

        for theta6, dx in yaw_candidates:
            wrist_candidates = self._candidate_wrist_angles(dx, dz, angles_current)

            for phi3 in wrist_candidates:
                wx = dx - l3 * np.sin(phi3)
                wz = dz - l3 * np.cos(phi3)
                d2 = wx ** 2 + wz ** 2
                cos_q2_denom = 2 * l1 * l2
                if cos_q2_denom == 0:
                    continue
                cos_q2 = (d2 - l1 ** 2 - l2 ** 2) / cos_q2_denom
                if cos_q2 < -1.01 or cos_q2 > 1.01:
                    continue
                cos_q2 = np.clip(cos_q2, -1.0, 1.0)
                q2_candidates = [np.arccos(cos_q2), -np.arccos(cos_q2)]

                for q2 in q2_candidates:
                    denom = l1 + l2 * np.cos(q2)
                    q1 = np.arctan2(wx, wz) - np.arctan2(l2 * np.sin(q2), denom)
                    phi1 = q1
                    phi2 = phi1 + q2
                    theta5 = phi1
                    theta4 = -q2
                    theta3 = phi3 - phi2

                    angle_dict = {
                        'J3': theta3,
                        'J4': theta4,
                        'J5': theta5,
                        'J6': theta6,
                    }

                    if any(abs(angle) > max_angle + 1e-6 for angle in angle_dict.values()):
                        continue
                    if self._violates_restricted_zone(angle_dict):
                        continue

                    servo_targets = {
                        'J1': self.joints['J1'],
                        'J2': self.joints['J2'],
                        'J3': self.radians_to_servo(theta3),
                        'J4': self.radians_to_servo(theta4),
                        'J5': self.radians_to_servo(theta5),
                        'J6': self.radians_to_servo(theta6),
                    }

                    config_key = tuple(servo_targets[j] for j in ['J3', 'J4', 'J5', 'J6'])
                    if config_key in seen_configs:
                        continue
                    seen_configs.add(config_key)

                    solutions.append({
                        'angles': angle_dict,
                        'servos': servo_targets,
                        'score': self._score_solution(servo_targets),
                        'wrist_angle': phi3,
                    })

        solutions.sort(key=lambda item: item['score'])
        return solutions

    def inverse_kinematics(self, x: float, y: float, z: float, apply: bool = False) -> Optional[Dict[str, Dict]]:
        """
        Solve the inverse kinematics for an absolute cartesian target (x, y, z).

        The method enumerates all feasible wrist configurations, evaluates every
        reachable elbow-up and elbow-down solution, and ranks them by:
        1) total joint travel (shortest path),
        2) squared joint travel (smoothness),
        3) number of joints that need noticeable adjustment.

        Args:
            x, y, z: Absolute coordinates in millimeters.
            apply: When True, the best solution is written into self.joints.

        Returns:
            Dictionary with the best solution and the complete solution set, or
            None if the target is unreachable.
        """
        solutions = self._enumerate_ik_solutions(x, y, z)
        if not solutions:
            print("Inverse kinematics failed: target is outside the reachable workspace.")
            return None

        best = solutions[0]
        if apply:
            self.set_all_joints(
                best['servos']['J1'],
                best['servos']['J2'],
                best['servos']['J3'],
                best['servos']['J4'],
                best['servos']['J5'],
                best['servos']['J6'],
            )

        return {
            'best': best,
            'solutions': solutions,
        }

    def move_end_effector_line(self, start: Tuple[float, float, float], end: Tuple[float, float, float], steps: int = 20) -> bool:
        """
        Move the end-effector in a straight line between two points.

        Args:
            start: (x, y, z) coordinates of the starting waypoint.
            end: (x, y, z) coordinates of the ending waypoint.
            steps: Number of waypoints along the line (>=2).
        """
        if steps < 2:
            raise ValueError("steps must be >= 2 for line motion.")

        sx, sy, sz = start
        ex, ey, ez = end
        ts = np.linspace(0.0, 1.0, steps)
        for t in ts:
            px = sx + (ex - sx) * t
            py = sy + (ey - sy) * t
            pz = sz + (ez - sz) * t
            result = self.inverse_kinematics(px, py, pz, apply=True)
            if not result:
                print(f"Line motion aborted at t={t:.2f}.")
                return False
        return True
    
    def get_gripper_state(self) -> str:
        """
        Get the current gripper state.
        
        Returns:
            String describing gripper state (open, closed, or partial)
        """
        gripper_value = self.joints['J1']
        
        if gripper_value < 200:
            return "fully open"
        elif gripper_value > 800:
            return "fully closed"
        elif 400 <= gripper_value <= 600:
            return "neutral"
        elif gripper_value < 500:
            return "partially open"
        else:
            return "partially closed"
    
    def get_gripper_width(self) -> float:
        """
        Get the current gripper opening width in millimeters.
        
        The gripper width varies linearly from max_width (at servo=0) 
        to min_width (at servo=1000).
        
        Returns:
            Current gripper width in millimeters
        """
        gripper_value = self.joints['J1']
        # Linear interpolation: 0 = max_width (open), 1000 = min_width (closed)
        t = gripper_value / 1000.0
        width = self.links.gripper_max_width * (1 - t) + self.links.gripper_min_width * t
        return width
    
    def __str__(self) -> str:
        """String representation of the robot arm state."""
        lines = ["Robot Arm State:"]
        lines.append("=" * 40)
        
        joint_descriptions = {
            'J1': 'Gripper',
            'J2': 'Gripper Roll',
            'J3': 'Wrist',
            'J4': 'Elbow',
            'J5': 'Shoulder',
            'J6': 'Base Yaw'
        }
        
        for joint_name in ['J1', 'J2', 'J3', 'J4', 'J5', 'J6']:
            value = self.joints[joint_name]
            angle_rad = self.servo_to_radians(value, joint_name)
            angle_deg = np.rad2deg(angle_rad)
            desc = joint_descriptions[joint_name]
            lines.append(f"{joint_name} ({desc:14s}): {value:4d} ({angle_deg:6.1f}°)")
        
        lines.append("=" * 40)
        lines.append(f"Gripper State: {self.get_gripper_state()}")
        
        x, y, z = self.forward_kinematics()
        lines.append(f"End-Effector Position: ({x:.1f}, {y:.1f}, {z:.1f}) mm")
        
        return "\n".join(lines)
    
    def __repr__(self) -> str:
        """Machine-readable representation."""
        joint_vals = ", ".join(f"{k}={v}" for k, v in self.joints.items())
        return f"RobotArm({joint_vals})"
    

    def _init_servo_positions(self):
        """
        Initialize servos to default positions using the attached board.
        """
        if not (self.hardware_enabled and self.hardware_board):
            return
        default_positions = [
            [6, 500],
            [5, 500],
            [4, 500],
            [3, 120],
            [2, 500],
            [1, 500],
        ]
        for servo, position in default_positions:
            self._set_servo_position(servo=servo, position=position)
            time.sleep(0.5)

    def _set_servo_position(self, servo: int, position: int):
        """
        Set a single servo position through the hardware board.
        """
        if not (self.hardware_enabled and self.hardware_board):
            return
        try:
            self.hardware_board.bus_servo_set_position(
                self.hardware_speed,
                [[int(servo), int(position)]]
            )
            time.sleep(0.02)
        except Exception as exc:
            print(f"Hardware write error (servo {servo}): {exc}")



if __name__ == "__main__":
    # Example usage
    arm = RobotArm()
    print("Initial state (all neutral):")
    print(arm)
    print()
    
    # Move some joints
    arm.set_joint('J6', 750)  # Rotate base clockwise
    arm.set_joint('J5', 700)  # Shoulder forward
    arm.set_joint('J1', 800)  # Close gripper
    
    print("\nAfter moving some joints:")
    print(arm)
    print()
    
    # Reset to neutral
    arm.reset_to_neutral()
    print("\nAfter reset:")
    print(arm)
