import time
import threading
import curses

class RobotController:
    def __init__(self, target_waypoint):
        self.target_waypoint = target_waypoint
        self.key_pressed = None
        self.step_size = 0.01

    def control_robot(self, key):
        if key == curses.KEY_UP:
            self.target_waypoint[2] += self.step_size
        elif key == curses.KEY_DOWN:
            self.target_waypoint[2] -= self.step_size
        elif key == curses.KEY_LEFT:
            self.target_waypoint[0] -= self.step_size
        elif key == curses.KEY_RIGHT:
            self.target_waypoint[0] += self.step_size
        elif key == ord(','):
            self.target_waypoint[1] -= self.step_size
        elif key == ord('.'):
            self.target_waypoint[1] += self.step_size
        elif key == ord('q'):
            return False
        return True

    def keyboard_listener(self, stdscr):
        while True:
            key = stdscr.getch()
            if key != -1:
                self.key_pressed = key
            time.sleep(0.01)

    def main(self, stdscr):
        stdscr.clear()
        stdscr.nodelay(True)

        listener_thread = threading.Thread(target=self.keyboard_listener, args=(stdscr,))
        listener_thread.daemon = True
        listener_thread.start()

        running = True
        while running:
            if self.key_pressed is not None:
                running = self.control_robot(self.key_pressed)
                self.key_pressed = None
            time.sleep(1./240.)

    def start(self):
        curses.wrapper(self.main)

# Example usage
if __name__ == "__main__":
    target_waypoint = [0.0, 0.0, 0.0]  # Shared variable to store the target waypoint
    controller = RobotController(target_waypoint)
    controller.start()
    print("Target waypoint:", target_waypoint)
