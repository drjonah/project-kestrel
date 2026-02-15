import time
import struct
from pyrf24 import RF24, RF24_PA_LOW, RF24_DRIVER
from config import settings


class RadioDriver:

    def __init__(self) -> None:
        # Initialize radio
        self.radio = RF24(settings.RADIO_CE_PIN, settings.RADIO_CSN_PIN)
        
        # Start radio
        if not self.radio.begin():
            raise OSError("NRF24L01 Radio failed to initialize! Check wiring.")
        self._configure()
        

    def _configure(self) -> bool:
        # Set transmission strength to low
        self.radio.set_pa_level(RF24_PA_LOW)

        # Begin listening for transmissions
        address = b"00001"
        self.radio.open_rx_pipe(1, address)
        self.radio.start_listening()
        self.radio.print_details()


    def listen_forever(self) -> None:
        """Blocks and prints messages as they arrive."""
        print("Listening for radio packets...")

        while True:
            # Check if any data has arrived
            has_payload, pipe_number = self.radio.available_pipe()

            if has_payload:
                # Load received data
                payload_size = self.radio.get_dynamic_payload_size()
                received_data = self.radio.read(payload_size)

                # Decode and display data
                try:
                    text = received_data.decode("utf-8")
                    print(f"Received: {text}")
                except:
                    print(f"Received raw bytes: {received_data}")

            time.sleep(0.005)


    def send_message(self) -> None:
        raise NotImplementedError("Sending messages not implemented.")


    def close(self) -> None:
        self.radio.powerDown()

