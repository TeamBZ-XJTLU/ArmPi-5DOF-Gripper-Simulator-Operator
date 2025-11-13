import time

try:
    from ros_robot_controller_sdk import Board
    BOARD_AVAILABLE = True
except Exception:
    Board = None
    BOARD_AVAILABLE = False

def init_servo_positions(board=None, speed=0.5):
    """Initialize bus servos to default positions (best-effort).

    If the Board SDK isn't available this prints a message and returns.
    """
    try:
        if not BOARD_AVAILABLE and board is None:
            print("Board SDK not available; skipping init_servo_positions.")
            return
        if board is None:
            board = Board()
        default_positions = [
            [1, 500],
            [2, 500],
            [3, 120],
            [4, 500],
            [5, 500],
            [6, 500],
        ]
        for position in default_positions:
            print(f"Setting servo {position[0]} to position {position[1]}")
            board.bus_servo_set_position(speed, [position])
            time.sleep(0.5)
        print("Servos initialized to default positions.")
    except Exception as e:
        print(f"init_servo_positions error: {e}")


def set_servo_positions(board=None, speed=0.5, servo=1, position=500):
    """Set a specific bus servo to a given raw position (0..1000).

    If the Board SDK isn't available this prints a message and returns.
    """
    try:
        if not BOARD_AVAILABLE and board is None:
            print("[SIM] Board SDK not available; skipping hardware write.")
            return
        if board is None:
            board = Board()
        print(f"Setting servo {servo} -> raw {position}")
        board.bus_servo_set_position(speed, [[servo, int(position)]])
        time.sleep(0.02)
    except Exception as e:
        print(f"set_servo_positions error: {e}")


if __name__ == "__main__":
    init_servo_positions()
    while True:
        try:
            sid = int(input("Enter servo ID (1-6): "))
            pos = int(input("Enter target position (0-1000): "))
            set_servo_positions(servo=sid, position=pos)
        except KeyboardInterrupt:
            print("\nExiting...")
            break