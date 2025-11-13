#!/usr/bin/env python3
"""Interactive CLI controller for the RobotArm with IK and hardware support."""

from arm import RobotArm

HELP_TEXT = """Commands:
  target x y z            Move via IK to absolute position (mm)
  line x1 y1 z1 x2 y2 z2 [steps]  Move linearly between two points
  set JOINT value         Set a single joint to raw servo value (e.g., set J3 650)
  open                    Open the gripper (J1)
  close                   Close the gripper (J1)
  reset                   Reset all joints to neutral
  joints                  Print current joint servo values
  fk                      Print current end-effector position
  restrict on/off         Toggle restricted workspace guard
  horizontal on/off       Toggle gripper-horizontal constraint
  hardware on [speed]     Attach to hardware (speed optional 0-1)
  hardware off            Detach hardware
  help                    Show this text
  quit/exit               Leave the program
"""


def prompt_loop(arm: RobotArm):
    print("\nRobotic Arm CLI")
    print("Restricted zone + horizontal gripper modes are ENABLED.")
    print("Type 'help' to see available commands.\n")
    while True:
        try:
            line = input("arm> ").strip()
        except EOFError:
            print("\nExiting...")
            break
        except KeyboardInterrupt:
            print("\nInterrupted. Type 'quit' to exit.")
            continue

        if not line:
            continue

        lowered = line.lower()
        if lowered in {"quit", "exit"}:
            break
        if lowered == "help":
            print(HELP_TEXT)
            continue
        if lowered == "joints":
            print(arm)
            continue
        if lowered == "reset":
            arm.reset_to_neutral()
            print("All joints reset to neutral (500).")
            continue
        if lowered == "open":
            if arm.set_joint('J1', 100):
                print("Gripper opened.")
            continue
        if lowered == "close":
            if arm.set_joint('J1', 900):
                print("Gripper closed.")
            continue
        if lowered == "fk":
            x, y, z = arm.forward_kinematics()
            print(f"End-Effector: x={x:.2f} mm, y={y:.2f} mm, z={z:.2f} mm")
            continue
        if lowered.startswith("hardware"):
            parts = line.split()
            if len(parts) < 2 or parts[1] not in {"on", "off"}:
                print("Usage: hardware on [speed] | hardware off")
                continue
            if parts[1] == "off":
                arm.detach_hardware()
                print("Hardware detached.")
            else:
                try:
                    speed = float(parts[2]) if len(parts) > 2 else 0.5
                except ValueError:
                    print("Invalid speed value.")
                    continue
                if arm.attach_hardware(speed=speed):
                    print("Hardware attached (speed %.2f)." % speed)
                else:
                    print("Failed to attach hardware.")
            continue
        if lowered.startswith("target"):
            parts = line.split()
            if len(parts) != 4:
                print("Usage: target x y z")
                continue
            try:
                x_val = float(parts[1])
                y_val = float(parts[2])
                z_val = float(parts[3])
            except ValueError:
                print("Invalid numbers. Usage: target x y z")
                continue
            result = arm.inverse_kinematics(x_val, y_val, z_val, apply=True)
            if result:
                best = result['best']
                servos = ", ".join(f"{k}:{v}" for k, v in best['servos'].items())
                print(f"Moved to ({x_val:.1f}, {y_val:.1f}, {z_val:.1f}) mm -> {servos}")
            else:
                print("IK failed for that target.")
            continue
        if lowered.startswith("line"):
            parts = line.split()
            if len(parts) not in {7, 8}:
                print("Usage: line x1 y1 z1 x2 y2 z2 [steps]")
                continue
            try:
                x1 = float(parts[1])
                y1 = float(parts[2])
                z1 = float(parts[3])
                x2 = float(parts[4])
                y2 = float(parts[5])
                z2 = float(parts[6])
                steps = int(parts[7]) if len(parts) == 8 else 20
            except ValueError:
                print("Invalid numbers. Usage: line x1 y1 z1 x2 y2 z2 [steps]")
                continue
            if steps < 2:
                print("Steps must be >= 2.")
                continue
            if arm.move_end_effector_line((x1, y1, z1), (x2, y2, z2), steps):
                print(f"Completed line path from ({x1:.1f}, {y1:.1f}, {z1:.1f}) to ({x2:.1f}, {y2:.1f}, {z2:.1f}).")
            else:
                print("Line motion failed; see details above.")
            continue
        if lowered.startswith("restrict"):
            parts = line.split()
            if len(parts) != 2 or parts[1] not in {"on", "off"}:
                print("Usage: restrict on|off")
                continue
            flag = parts[1] == "on"
            arm.set_restricted_zone(flag)
            print(f"Restricted zone {'enabled' if flag else 'disabled'}.")
            continue
        if lowered.startswith("horizontal"):
            parts = line.split()
            if len(parts) != 2 or parts[1] not in {"on", "off"}:
                print("Usage: horizontal on|off")
                continue
            flag = parts[1] == "on"
            arm.set_gripper_horizontal_mode(flag)
            print(f"Horizontal gripper mode {'enabled' if flag else 'disabled'}.")
            continue
        if lowered.startswith("set "):
            parts = line.split()
            if len(parts) != 3:
                print("Usage: set JOINT value (e.g., set J3 650)")
                continue
            joint_name = parts[1].upper()
            if joint_name in {"1", "2", "3", "4", "5", "6"}:
                joint_name = f"J{joint_name}"
            if joint_name not in {"J1", "J2", "J3", "J4", "J5", "J6"}:
                print("Invalid joint. Use J1-J6 or 1-6.")
                continue
            try:
                value = int(parts[2])
            except ValueError:
                print("Invalid servo value. Must be integer 0-1000.")
                continue
            if arm.set_joint(joint_name, value):
                print(f"{joint_name} set to {value}.")
            else:
                print("Failed to set joint (out of range or hardware error).")
            continue

        print("Unknown command. Type 'help' for usage.")


def main():
    arm = RobotArm()
    arm.set_restricted_zone(True)
    arm.set_gripper_horizontal_mode(True)
    print("Initialized RobotArm with restricted zone + horizontal gripper.")
    prompt_loop(arm)


if __name__ == "__main__":
    main()
