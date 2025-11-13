"""
Streamlit visualization for the 5DOF+Gripper RRRRR Robot Arm Simulator
"""

import streamlit as st
import numpy as np
import plotly.graph_objects as go
from arm import RobotArm

# Page configuration
st.set_page_config(
    page_title="Robot Arm Simulator",
    page_icon="🦾",
    layout="wide"
)

# Initialize robot arm in session state
if 'arm' not in st.session_state:
    st.session_state.arm = RobotArm()

arm = st.session_state.arm

# Title and description
st.title("🦾 5DOF+Gripper Robot Arm Simulator")
st.markdown("Interactive visualization and control of RRRRR robot arm with end-effector gripper")

# Sidebar for controls
st.sidebar.header("Joint Controls")
st.sidebar.markdown("Adjust servo values (0-1000, neutral at 500)")

# Joint sliders
j1 = st.sidebar.slider("J1 - Gripper", 0, 1000, arm.get_joint('J1'), 10, 
                       help="Controls gripper open/close")
j2 = st.sidebar.slider("J2 - Gripper Roll", 0, 1000, arm.get_joint('J2'), 10,
                       help="Controls gripper rotation")
j3 = st.sidebar.slider("J3 - Wrist", 0, 1000, arm.get_joint('J3'), 10,
                       help="Controls wrist pitch")
j4 = st.sidebar.slider("J4 - Elbow", 0, 1000, arm.get_joint('J4'), 10,
                       help="Controls elbow pitch")
j5 = st.sidebar.slider("J5 - Shoulder", 0, 1000, arm.get_joint('J5'), 10,
                       help="Controls shoulder pitch")
j6 = st.sidebar.slider("J6 - Base Yaw", 0, 1000, arm.get_joint('J6'), 10,
                       help="Controls base rotation")

# Reset button
if st.sidebar.button("Reset to Neutral", width="stretch"):
    arm.reset_to_neutral()
    st.rerun()

# Update arm with slider values
arm.set_all_joints(j1, j2, j3, j4, j5, j6)

# Main content area with two columns
col1, col2 = st.columns([2, 1])

with col1:
    st.subheader("3D Visualization")
    
    # Calculate joint positions for visualization
    angles = arm.get_joint_angles_radians()
    
    # Base position (J6/J5 location)
    base_pos = np.array([0, 0, arm.links.base_height])
    
    # Initialize positions array
    positions = [base_pos]
    
    # Calculate each joint position
    theta6 = angles['J6']
    theta5 = angles['J5']
    theta4 = angles['J4']
    theta3 = angles['J3']
    theta2 = angles['J2']
    
    # J5 to J4 (Shoulder link - L4)
    y_plane = arm.links.L4 * np.sin(theta5)
    z = base_pos[2] + arm.links.L4 * np.cos(theta5)
    j4_pos = np.array([y_plane * np.sin(theta6), y_plane * np.cos(theta6), z])
    positions.append(j4_pos)
    
    # J4 to J3 (Elbow link - L3)
    # J4 moves in opposite direction: 0=+Y, 1000=-Y (inverted from J5)
    cumulative_angle = theta5 - theta4
    y_plane += arm.links.L3 * np.sin(cumulative_angle)
    z += arm.links.L3 * np.cos(cumulative_angle)
    j3_pos = np.array([y_plane * np.sin(theta6), y_plane * np.cos(theta6), z])
    positions.append(j3_pos)
    
    # J3 to J2 (Wrist link - L2)
    cumulative_angle += theta3
    y_plane += arm.links.L2 * np.sin(cumulative_angle)
    z += arm.links.L2 * np.cos(cumulative_angle)
    j2_pos = np.array([y_plane * np.sin(theta6), y_plane * np.cos(theta6), z])
    positions.append(j2_pos)
    
    # J2 to End-effector (Gripper link - L1)
    # J2 only rotates the gripper, does not change position
    y_plane += arm.links.L1 * np.sin(cumulative_angle)
    z += arm.links.L1 * np.cos(cumulative_angle)
    end_effector_pos = np.array([y_plane * np.sin(theta6), y_plane * np.cos(theta6), z])
    positions.append(end_effector_pos)
    
    # Convert to arrays for plotting
    positions = np.array(positions)
    
    # Create 3D plot
    fig = go.Figure()
    
    # Draw base platform
    base_length = arm.base_length
    base_width = arm.base_width
    base_corners = np.array([
        [-base_width/2, -arm.base_j5j6_offset, 0],
        [base_width/2, -arm.base_j5j6_offset, 0],
        [base_width/2, base_length - arm.base_j5j6_offset, 0],
        [-base_width/2, base_length - arm.base_j5j6_offset, 0],
        [-base_width/2, -arm.base_j5j6_offset, 0]
    ])
    
    fig.add_trace(go.Scatter3d(
        x=base_corners[:, 0], y=base_corners[:, 1], z=base_corners[:, 2],
        mode='lines',
        line=dict(color='gray', width=4),
        name='Base Platform',
        showlegend=True
    ))
    
    # Draw arm links
    fig.add_trace(go.Scatter3d(
        x=positions[:, 0], y=positions[:, 1], z=positions[:, 2],
        mode='lines+markers',
        line=dict(color='blue', width=8),
        marker=dict(size=8, color='red'),
        name='Arm Links',
        showlegend=True
    ))
    
        
    # Draw gripper as a plane from J2 to end-of-gripper
    gripper_width = arm.get_gripper_width()
    
    # Calculate gripper orientation vectors
    # Direction along the arm (pointing direction from J2 to end effector)
    arm_direction = np.array([
        np.sin(theta6) * np.sin(cumulative_angle),
        np.cos(theta6) * np.sin(cumulative_angle),
        np.cos(cumulative_angle)
    ])
    
    # Up vector in world coordinates
    world_up = np.array([0, 0, 1])
    
    # Gripper opening direction (perpendicular to arm direction)
    # This is affected by the gripper roll (J2)
    gripper_opening_dir = np.cross(arm_direction, world_up)
    if np.linalg.norm(gripper_opening_dir) < 0.01:
        # If arm points straight up/down, use a default direction
        gripper_opening_dir = np.array([np.cos(theta6), -np.sin(theta6), 0])
    else:
        gripper_opening_dir = gripper_opening_dir / np.linalg.norm(gripper_opening_dir)
    
    # Rotate opening direction by gripper roll angle
    rotation_axis = arm_direction / (np.linalg.norm(arm_direction) + 1e-6)
    cos_roll = np.cos(theta2)
    sin_roll = np.sin(theta2)
    # Rodrigues' rotation formula
    gripper_opening_dir = (gripper_opening_dir * cos_roll + 
                          np.cross(rotation_axis, gripper_opening_dir) * sin_roll +
                          rotation_axis * np.dot(rotation_axis, gripper_opening_dir) * (1 - cos_roll))
    
    # Create a rectangular plane from J2 to end-effector with gripper width
    half_width = gripper_width / 2
    
    # Four corners of the gripper plane
    j2_corner1 = j2_pos + gripper_opening_dir * half_width
    j2_corner2 = j2_pos - gripper_opening_dir * half_width
    end_corner1 = end_effector_pos + gripper_opening_dir * half_width
    end_corner2 = end_effector_pos - gripper_opening_dir * half_width
    
    # Draw gripper plane as a mesh
    gripper_vertices = np.array([j2_corner1, j2_corner2, end_corner2, end_corner1])
    
    fig.add_trace(go.Mesh3d(
        x=gripper_vertices[:, 0],
        y=gripper_vertices[:, 1],
        z=gripper_vertices[:, 2],
        i=[0, 0],
        j=[1, 2],
        k=[2, 3],
        color='lightgreen',
        opacity=0.7,
        name='Gripper Plane',
        showlegend=True
    ))
    
    # Draw edges of the gripper plane for better visibility
    edge_points = np.array([j2_corner1, j2_corner2, end_corner2, end_corner1, j2_corner1])
    fig.add_trace(go.Scatter3d(
        x=edge_points[:, 0],
        y=edge_points[:, 1],
        z=edge_points[:, 2],
        mode='lines',
        line=dict(color='green', width=4),
        name='Gripper Edges',
        showlegend=False
    ))
    
    # Add labels for joints

    
    # Add labels for joints
    joint_labels = ['Base (J5/J6)', 'J4', 'J3', 'J2', 'End Effector']
    for i, (pos, label) in enumerate(zip(positions, joint_labels)):
        fig.add_trace(go.Scatter3d(
            x=[pos[0]], y=[pos[1]], z=[pos[2]],
            mode='text',
            text=[label],
            textposition='top center',
            showlegend=False,
            textfont=dict(size=10, color='black')
        ))
    
    # Set layout
    max_reach = arm.links.L1 + arm.links.L2 + arm.links.L3 + arm.links.L4
    fig.update_layout(
        scene=dict(
            xaxis=dict(range=[-max_reach, max_reach], title='X (mm)'),
            yaxis=dict(range=[-max_reach, max_reach], title='Y (mm)'),
            zaxis=dict(range=[0, max_reach + arm.links.base_height], title='Z (mm)'),
            aspectmode='cube',
            camera=dict(
                eye=dict(x=1.5, y=1.5, z=1.2)
            )
        ),
        height=600,
        margin=dict(l=0, r=0, t=30, b=0)
    )
    
    st.plotly_chart(fig, width="stretch")

with col2:
    st.subheader("Robot Status")
    
    # Joint angles table
    st.markdown("#### Joint Angles")
    angles_data = []
    joint_names = ['J1', 'J2', 'J3', 'J4', 'J5', 'J6']
    joint_desc = ['Gripper', 'Gripper Roll', 'Wrist', 'Elbow', 'Shoulder', 'Base Yaw']
    
    for jname, jdesc in zip(joint_names, joint_desc):
        servo_val = arm.get_joint(jname)
        angle_rad = arm.servo_to_radians(servo_val, jname)
        angle_deg = np.rad2deg(angle_rad)
        angles_data.append({
            'Joint': f"{jname} ({jdesc})",
            'Servo': servo_val,
            'Angle': f"{angle_deg:.1f}°"
        })
    
    st.table(angles_data)
    
    # Gripper state
    st.markdown("#### Gripper State")
    gripper_state = arm.get_gripper_state()
    gripper_width = arm.get_gripper_width()
    
    if gripper_state == "fully open":
        st.success(f"🟢 {gripper_state.upper()}")
    elif gripper_state == "fully closed":
        st.error(f"🔴 {gripper_state.upper()}")
    else:
        st.info(f"🟡 {gripper_state.upper()}")
    
    st.metric("Gripper Width", f"{gripper_width:.1f} mm")
    
    # Show gripper specs
    with st.expander("Gripper Specifications"):
        st.write(f"**Max Width (Open):** {arm.links.gripper_max_width:.1f} mm")
        st.write(f"**Min Width (Closed):** {arm.links.gripper_min_width:.1f} mm")
    
    # End-effector position
    st.markdown("#### End-Effector Position")
    x, y, z = arm.forward_kinematics()
    st.metric("X", f"{x:.1f} mm")
    st.metric("Y", f"{y:.1f} mm")
    st.metric("Z", f"{z:.1f} mm")
    
    # Distance from base
    distance = np.sqrt(x**2 + y**2 + (z - arm.links.base_height)**2)
    st.metric("Distance from Base", f"{distance:.1f} mm")
    
    # Quick presets
    st.markdown("#### Quick Presets")
    if st.button("Home Position", width="stretch"):
        arm.reset_to_neutral()
        st.rerun()
    
    if st.button("Reach Forward", width="stretch"):
        arm.set_all_joints(500, 500, 300, 300, 300, 500)
        st.rerun()
    
    if st.button("Reach Up", width="stretch"):
        arm.set_all_joints(500, 500, 500, 500, 500, 500)
        st.rerun()
    
    if st.button("Close Gripper", width="stretch"):
        arm.set_joint('J1', 900)
        st.rerun()
    
    if st.button("Open Gripper", width="stretch"):
        arm.set_joint('J1', 100)
        st.rerun()

# Footer with information
st.markdown("---")
st.markdown("""
**Robot Specifications:**
- **Type:** 5DOF + Gripper RRRRR Configuration
- **Link Lengths:** L1=110mm, L2=58mm, L3=95mm, L4=109mm
- **Base Height:** 58mm
- **Joint Range:** 0-1000 servo units (±135° from neutral)
""")
