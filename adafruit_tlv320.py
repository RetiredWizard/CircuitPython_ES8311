import circuitpython_es8311
import board
import digitalio
import time

print('**** Warning **** "FAKE" adafruit_tlv320.py library is being used to')
print('allow applications written for the tlv320 device to run on the ESP8311 DAC')

pwr = digitalio.DigitalInOut(board.I2S_PWR)
pwr.direction=digitalio.Direction.OUTPUT
#pwr.value=False
pwr.value=True
#pwr.value=False
vol = digitalio.DigitalInOut(board.AMP_VOL)
vol.direction=digitalio.Direction.OUTPUT
vol.value=True
shdn = digitalio.DigitalInOut(board.AMP_SHDN)
shdn.direction=digitalio.Direction.OUTPUT
shdn.value=False

TLV320DAC3100 = circuitpython_es8311.ES8311

