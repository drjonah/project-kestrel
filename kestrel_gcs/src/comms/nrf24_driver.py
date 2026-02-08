import time
import struct
from pyrf24 import RF24, RF24_PA_LOW, RF24_DRIVER
from config import settings


class RadioDriver:

    def __init__(self) -> None:
        
        self.radio = RF24(settings.RADIO_CE_PIN, settings.RADIO_CSN_PIN)
        if not self._configure():
            raise RuntimeError("NRF24L01 Radio failed to initialize! Check wiring.")


    def _configure(self) -> bool:
        pass


    def send_message(self) -> None:
        pass


    def close(self) -> None:
        self.radio.powerDown()

