from pyPS4Controller.controller import Controller
from config import settings


class DroneController(Controller):
    def __init__(self, shared_state) -> None:
        self.state = shared_state
        Controller.__init__(self, interface="/dev/input/js0", connecting_using_ds4drv=False)


    def _normalize(self, raw_value: int) -> float:
        """
        Converts hardware range [-32767, 32767] to float [-1.0, 1.0].
        Handles the Deadzone logic.
        """
        if abs(raw_value) <= settings.DEAD_ZONE:
            return 0.0
        return raw_value / settings.MAX_JOYSTICK_VAL


    # --- THROTTLE (Up/Down) ---
    def on_L3_up(self, value):
        throttle = -self._normalize(value)
        self.state.update_command("throttle", throttle)


    def on_L3_down(self, value):
        throttle = -self._normalize(value)
        self.state.update_command("throttle", throttle)


    # --- YAW (Rotate Left/Right) ---
    def on_L3_left(self, value):
        yaw = self._normalize(value)
        self.state.update_command("yaw", yaw)


    def on_L3_right(self, value):
        yaw = self._normalize(value)
        self.state.update_command("yaw", yaw)


    # --- PITCH (Forward/Back) ---
    def on_R3_up(self, value):
        pitch = -self._normalize(value)
        self.state.update_command("pitch", pitch)


    def on_R3_down(self, value):
        pitch = -self._normalize(value)
        self.state.update_command("pitch", pitch)


    # --- ROLL (Strafe Left/Right) ---
    def on_R3_left(self, value):
        roll = self._normalize(value)
        self.state.update_command("roll", roll)


    def on_R3_right(self, value):
        roll = self._normalize(value)
        self.state.update_command("roll", roll)
        

    # ---Buttons --- 
    def on_x_press(self):
        print("Disarming Drone...")
        self.state.update_command("armed", False)
        

    def on_triangle_press(self):
        print("ARMING DRONE!")
        self.state.update_command("armed", True)


def start_controller(shared_state) -> None:
    """Entry point for the Thread"""
    controller = DroneController(shared_state)
    controller.listen()