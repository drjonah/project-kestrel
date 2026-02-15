import gpiod
from src.core import SharedState
from src.comms import RadioDriver
from src.inputs import start_controller


def main():
    # shared_state = SharedState()
    # start_controller(shared_state)

    radio_driver = RadioDriver()
    if radio_driver:
        print("Radio listening...")
        radio_driver.listen_forever()
    else:
        print("Error encountered while starting radio.")


if __name__ == "__main__":
    main()