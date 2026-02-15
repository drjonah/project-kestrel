import gpiod
from src.core import SharedState
from src.inputs import start_controller


def main():
    shared_state = SharedState()
    start_controller(shared_state)


if __name__ == "__main__":
    main()