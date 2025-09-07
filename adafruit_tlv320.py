import circuitpython_es8311
import board
import digitalio

print('**** Warning **** "FAKE" adafruit_tlv320.py library is being used to')
print('allow applications written for the tlv320 device to run on the ESP8311 DAC')

pwr = digitalio.DigitalInOut(board.I2S_PWR)
pwr.direction=digitalio.Direction.OUTPUT
pwr.value=True
vol = digitalio.DigitalInOut(board.AMP_VOL)
vol.direction=digitalio.Direction.OUTPUT
vol.value=True
shdn = digitalio.DigitalInOut(board.AMP_SHDN)
shdn.direction=digitalio.Direction.OUTPUT
shdn.value=False

class TLV320DAC3100(circuitpython_es8311.ES8311):

    def __init__(self, i2c: I2C, address: int = 0x18) -> None:
        super().__init__(i2c,address)

    # ----------------------------------------------
    # Dummy methods for TLV320 library compatiblitiy
    # ----------------------------------------------
    @property
    def headphone_output(self) -> bool:
        return True

    @headphone_output.setter
    def headphone_output(self, enabled: bool) -> None:
        return

    @property
    def speaker_output(self) -> bool:
        return True
        
    @speaker_output.setter
    def speaker_output(self, enabled: bool) -> None:
        return

