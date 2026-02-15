from enum import Enum

class Commands(Enum):
    THROTTLE    = "throttle"
    YAW         = "yaw"
    PITCH       = "pitch"
    ROLL        = "roll"
    ARM_STATUS  = "arm"


class SharedState:
    def __init__(self) -> None:
        self.running_state = self._initialize_shared_state()


    def _initialize_shared_state(self) -> dict:
        """Creates a default state dict for the running state."""
        state = {}

        for command in Commands:
            default_val = False if command == Commands.ARM_STATUS else 0.0
            state[command.value] = default_val

        return state


    def update_command(self, command: Commands, value: float) -> None:
        print(self.running_state)
        self.running_state[command.value] = value

