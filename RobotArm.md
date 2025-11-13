# 5DOF+Gripper RRRRR Robot Arm with End-Effector Gripper
===========================================

## Joint and Link Descriptions

### J1 - Gripper
- **Function**: Controls the gripper open and close
- **Servo Range**: 
  - 0: Fully open (wide)
  - 500: Neutral position
  - 1000: Fully closed

### J2 - Gripper Roll
- **Function**: Controls the gripper rotation (roll)
- **Servo Range**:
  - 500: Gripper horizontal (neutral)
  - 0: Rotate anti-clockwise 135°
  - 1000: Rotate clockwise 135°

### J3 - Wrist
- **Function**: Controls the wrist pitch
- **Servo Range**:
  - 500: J3→J2 link is vertical
  - 0: Moves along -Y axis 135°
  - 1000: Moves along +Y axis 135°

### J4 - Elbow
- **Function**: Controls the elbow pitch
- **Servo Range**:
  - 500: J4→J3 link is vertical
  - 0: Moves along -Y axis 135°
  - 1000: Moves along +Y axis 135°

### J5 - Shoulder
- **Function**: Controls the shoulder pitch
- **Servo Range**:
  - 500: J5→J4 link is vertical
  - 0: Moves along -Y axis 135°
  - 1000: Moves along +Y axis 135°

### J6 - Base Yaw
- **Function**: Controls the base rotation (yaw)
- **Servo Range**:
  - 500: Facing +Y axis (neutral)
  - 0: Rotate anti-clockwise 135°
  - 1000: Rotate clockwise 135°

## Link Lengths
- **L1 (J2 to End-of-gripper)**: 110 mm
- **L2 (J3 to J2)**: 58 mm
- **L3 (J4 to J3)**: 95 mm
- **L4 (J5 to J4)**: 109 mm

## Other Dimensions
- **Base Height (Ground to J5 center)**: 58 mm
- **Base Length**: 291 mm
- **Base Width**: 159 mm
- **J5/J6 Position**: 80 mm to the short edge, centred to the short edge
- **Gripper Width**: 55 mm (closed), 98 mm (fully open)

### Base Layout (Top View)
```
        ← 159 mm (Width) →
    ┌─────────────────────────┐  ↑
    │                         │  
    │                         │  
    │                         │  
    │                         │  
    │                         │  
    │                         │  
    │                         │  
    │                         │  
    │                         │  291 mm
    │                         │  (Length)
    │          J5/J6          │  
    │            ●            │
    │         (centered)      │
    │            ↑            │  
    │        80 mm            │  
    │            ↓            │
    │                         │  
    └─────────────────────────┘  ↓
    
    J5/J6 is centered on the width (159 mm / 2 = 79.5 mm from each side)
    J5/J6 is 80 mm from the short edge (159 mm edge)
```

