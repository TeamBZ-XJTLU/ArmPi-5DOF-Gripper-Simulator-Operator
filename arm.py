"""
5DOF+Gripper RRRRR Robot Arm Simulator
This module provides a class to represent and control a 5-degree-of-freedom
robot arm with a gripper end-effector.
"""

import numpy as np
from typing import Dict, Tuple, Optional
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
        return True
    
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
        
        return True
    
    def reset_to_neutral(self):
        """Reset all joints to neutral position (500)."""
        for joint in self.joints:
            self.joints[joint] = self.limits.neutral
    
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
