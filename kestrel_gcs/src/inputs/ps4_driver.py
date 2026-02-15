from pyPS4Controller.controller import Controller
from config import settings
from src.core import Commands, SharedState


class DroneController(Controller):
    def __init__(self, shared_state: SharedState) -> None:
        self.state = shared_state
        Controller.__init__(self, interface="/dev/input/js0", connecting_using_ds4drv=False)


    def _normalize(self, raw_value: int) -> float:
        """
        Converts hardware range [-32767, 32767] to float [-1.0, 1.0].
        Handles the Deadzone logic.
        """
        new_val = raw_value / settings.MAX_JOYSTICK_VAL
        if abs(new_val) <= settings.DEAD_ZONE:
            return 0.0
        
        # Exponential control on inputs to avoid input mistakes
        return (new_val * (1 - settings.EXPO_FACTOR)) + (new_val**3 * settings.EXPO_FACTOR)


    # --- THROTTLE (Up/Down) ---
    def on_L3_up(self, value):
        throttle = -self._normalize(value)
        self.state.update_command(Commands.THROTTLE, throttle)


    def on_L3_down(self, value):
        throttle = -self._normalize(value)
        self.state.update_command(Commands.THROTTLE, throttle)


    # --- YAW (Rotate Left/Right) ---
    def on_L3_left(self, value):
        yaw = self._normalize(value)
        self.state.update_command(Commands.YAW, yaw)


    def on_L3_right(self, value):
        yaw = self._normalize(value)
        self.state.update_command(Commands.YAW, yaw)


    # --- PITCH (Forward/Back) ---
    def on_R3_up(self, value):
        pitch = -self._normalize(value)
        self.state.update_command(Commands.PITCH, pitch)


    def on_R3_down(self, value):
        pitch = -self._normalize(value)
        self.state.update_command(Commands.PITCH, pitch)


    # --- ROLL (Strafe Left/Right) ---
    def on_R3_left(self, value):
        roll = self._normalize(value)
        self.state.update_command(Commands.ROLL, roll)


    def on_R3_right(self, value):
        roll = self._normalize(value)
        self.state.update_command(Commands.ROLL, roll)
        

    # ---Buttons --- 
    def on_x_press(self):
        print("DISARMING DRONE!")
        self.state.update_command(Commands.ARM_STATUS, False)
        

    def on_triangle_press(self):
        print("ARMING DRONE!")
        self.state.update_command(Commands.ARM_STATUS, True)


def start_controller(shared_state) -> None:
    """Entry point for the Thread"""
    print("Starting Controller!")
    controller = DroneController(shared_state)
    controller.listen()