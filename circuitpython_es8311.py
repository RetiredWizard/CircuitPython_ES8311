# SPDX-License-Identifier: MIT
"""
CircuitPython driver for ES8311 I2S audio codec.
API loosely modeled after Adafruit's TLV320AIC23 driver for compatibility.

* Author(s): RetiredWizard

Resource Credit:
@pschatzmann for arduino-audio-driver (https://github.com/pschatzmann/arduino-audio-driver) and
arduino-audio-tools (https://github.com/pschatzmann/arduino-audio-tools)

Implementation Notes
--------------------

The Olimex RP2350pc board has a GPIO pin connected to the ES8311 power enable
pin (GPIO22). This pin must be enabled for output and a True (1) value placed on 
the pin for the DAC to be powered. In addition, the speaker connectors are run
through an on-board amplifier for which there is a Volume pin (GPIO34) and an
Amplifer shutdown pin (GPIO35). In order for audio to play over the speaker connection
the Volume pin must have a True (1) value and the Shutdown pin must have a False (0)
value. The headphone jack will operate regardless of the Volume and Shutdown pin 
status, however inserting a headphone jack will cause the amplifer to shutdown turning
off the speaker output.

Usage Examples
--------------

Olimex RP2340pc Mini-Speaker
^^^^^^^^^^^^^^^^^^^^^^^^^^^^

::

    import board
    import audiobusio
    import audiocore
    import digitalio
    from circuitpython_es8311 import ES8311

    pwr = digitalio.DigitalInOut(board.I2S_PWR)
    pwr.direction=digitalio.Direction.OUTPUT
    pwr.value=True

    vol = digitalio.DigitalInOut(board.AMP_VOL)
    vol.direction=digitalio.Direction.OUTPUT
    vol.value=True

    shdn = digitalio.DigitalInOut(board.AMP_SHDN)
    shdn.direction=digitalio.Direction.OUTPUT
    shdn.value=False

    i2c = board.I2C()
    dac = ES8311(i2c)

    dac.configure_clocks(sample_rate=44100, bit_depth=16)

    f = open('/sd/new.wav','rb')
    wav = audiocore.WaveFile(f)
    audio_bus = audiobusio.I2SOut(board.I2S_BCLK, board.I2S_WS, board.I2S_DIN)
    audio_bus.play(wav)
    print(audio_bus.playing)

API
---
"""

import time
from micropython import const
from adafruit_bus_device.i2c_device import I2CDevice

_DEFAULT_ADDRESS = const(0x18)

# Codec hifi mclk clock divider coefficients
#    sample rate
#    mclk frequency
PRE_DIV = 0     #    the pre divider with range from 1 to 8
PRE_MULTI = 1   #    the pre multiplier with x1, x2, x4 and x8 selection
ADC_DIV = 2     #    adcclk divider
DAC_DIV = 3     #    dacclk divider
FS_MODE = 4     #    fs_mode double speed or single speed, =0, ss, =1, ds
LRCK_H = 5      #    lrck_h adclrck divider and daclrck divider
LRCK_L = 6      #    lrck_l
BCLK_DIV = 7    #    bclk_div sclk divider
ADC_OSR = 8     #    adc osr
DAC_OSR = 9     #    dac_osr
#
#   rate    mclk   pre_div     adc_div     fs_mode lrch  lrcl bckdiv      dac_osr
#                          mult       dac_div                       adc_osr
coeff_div = {
    8000: {
        12288000: [0x06, 0x01, 0x01, 0x01, 0x00, 0x00, 0xff, 0x04, 0x10, 0x10],
        18432000: [0x03, 0x02, 0x03, 0x03, 0x00, 0x05, 0xff, 0x18, 0x10, 0x10],
        16384000: [0x08, 0x01, 0x01, 0x01, 0x00, 0x00, 0xff, 0x04, 0x10, 0x10],
        8192000 : [0x04, 0x01, 0x01, 0x01, 0x00, 0x00, 0xff, 0x04, 0x10, 0x10],
        6144000 : [0x03, 0x01, 0x01, 0x01, 0x00, 0x00, 0xff, 0x04, 0x10, 0x10],
        4096000 : [0x02, 0x01, 0x01, 0x01, 0x00, 0x00, 0xff, 0x04, 0x10, 0x10],
        3072000 : [0x01, 0x01, 0x01, 0x01, 0x00, 0x00, 0xff, 0x04, 0x10, 0x10],
        2048000 : [0x01, 0x01, 0x01, 0x01, 0x00, 0x00, 0xff, 0x04, 0x10, 0x10],
        1536000 : [0x03, 0x04, 0x01, 0x01, 0x00, 0x00, 0xff, 0x04, 0x10, 0x10],
        1024000 : [0x01, 0x02, 0x01, 0x01, 0x00, 0x00, 0xff, 0x04, 0x10, 0x10],
    },
    11025: {
        11289600: [0x04, 0x01, 0x01, 0x01, 0x00, 0x00, 0xff, 0x04, 0x10, 0x10],
        5644800 : [0x02, 0x01, 0x01, 0x01, 0x00, 0x00, 0xff, 0x04, 0x10, 0x10],
        2822400 : [0x01, 0x01, 0x01, 0x01, 0x00, 0x00, 0xff, 0x04, 0x10, 0x10],
        1411200 : [0x01, 0x02, 0x01, 0x01, 0x00, 0x00, 0xff, 0x04, 0x10, 0x10],
    },
    12000: {
        12288000: [0x04, 0x01, 0x01, 0x01, 0x00, 0x00, 0xff, 0x04, 0x10, 0x10],
        6144000 : [0x02, 0x01, 0x01, 0x01, 0x00, 0x00, 0xff, 0x04, 0x10, 0x10],
        3072000 : [0x01, 0x01, 0x01, 0x01, 0x00, 0x00, 0xff, 0x04, 0x10, 0x10],
        1536000 : [0x01, 0x02, 0x01, 0x01, 0x00, 0x00, 0xff, 0x04, 0x10, 0x10],
    },
    16000: {
        12288000: [0x03, 0x01, 0x01, 0x01, 0x00, 0x00, 0xff, 0x04, 0x10, 0x10],
        18432000: [0x03, 0x02, 0x03, 0x03, 0x00, 0x02, 0xff, 0x0c, 0x10, 0x10],
        16384000: [0x04, 0x01, 0x01, 0x01, 0x00, 0x00, 0xff, 0x04, 0x10, 0x10],
        8192000 : [0x02, 0x01, 0x01, 0x01, 0x00, 0x00, 0xff, 0x04, 0x10, 0x10],
        6144000 : [0x03, 0x02, 0x01, 0x01, 0x00, 0x00, 0xff, 0x04, 0x10, 0x10],
        4096000 : [0x01, 0x01, 0x01, 0x01, 0x00, 0x00, 0xff, 0x04, 0x10, 0x10],
        3072000 : [0x03, 0x04, 0x01, 0x01, 0x00, 0x00, 0xff, 0x04, 0x10, 0x10],
        2048000 : [0x01, 0x02, 0x01, 0x01, 0x00, 0x00, 0xff, 0x04, 0x10, 0x10],
        1536000 : [0x03, 0x08, 0x01, 0x01, 0x00, 0x00, 0xff, 0x04, 0x10, 0x10],
        1024000 : [0x01, 0x04, 0x01, 0x01, 0x00, 0x00, 0xff, 0x04, 0x10, 0x10],
    },
    22050: {
        11289600: [0x02, 0x01, 0x01, 0x01, 0x00, 0x00, 0xff, 0x04, 0x10, 0x10],
        5644800 : [0x01, 0x01, 0x01, 0x01, 0x00, 0x00, 0xff, 0x04, 0x10, 0x10],
        2822400 : [0x01, 0x02, 0x01, 0x01, 0x00, 0x00, 0xff, 0x04, 0x10, 0x10],
        1411200 : [0x01, 0x04, 0x01, 0x01, 0x00, 0x00, 0xff, 0x04, 0x10, 0x10],
    },
    24000: {
        12288000: [0x02, 0x01, 0x01, 0x01, 0x00, 0x00, 0xff, 0x04, 0x10, 0x10],
        18432000: [0x03, 0x01, 0x01, 0x01, 0x00, 0x00, 0xff, 0x04, 0x10, 0x10],
        6144000 : [0x01, 0x01, 0x01, 0x01, 0x00, 0x00, 0xff, 0x04, 0x10, 0x10],
        3072000 : [0x01, 0x02, 0x01, 0x01, 0x00, 0x00, 0xff, 0x04, 0x10, 0x10],
        1536000 : [0x01, 0x04, 0x01, 0x01, 0x00, 0x00, 0xff, 0x04, 0x10, 0x10],
    },
    32000: {
        12288000: [0x03, 0x02, 0x01, 0x01, 0x00, 0x00, 0xff, 0x04, 0x10, 0x10],
        18432000: [0x03, 0x04, 0x03, 0x03, 0x00, 0x02, 0xff, 0x0c, 0x10, 0x10],
        16384000: [0x02, 0x01, 0x01, 0x01, 0x00, 0x00, 0xff, 0x04, 0x10, 0x10],
        8192000 : [0x01, 0x01, 0x01, 0x01, 0x00, 0x00, 0xff, 0x04, 0x10, 0x10],
        6144000 : [0x03, 0x04, 0x01, 0x01, 0x00, 0x00, 0xff, 0x04, 0x10, 0x10],
        4096000 : [0x01, 0x02, 0x01, 0x01, 0x00, 0x00, 0xff, 0x04, 0x10, 0x10],
        3072000 : [0x03, 0x08, 0x01, 0x01, 0x00, 0x00, 0xff, 0x04, 0x10, 0x10],
        2048000 : [0x01, 0x04, 0x01, 0x01, 0x00, 0x00, 0xff, 0x04, 0x10, 0x10],
        1536000 : [0x03, 0x08, 0x01, 0x01, 0x01, 0x00, 0x7f, 0x02, 0x10, 0x10],
        1024000 : [0x01, 0x08, 0x01, 0x01, 0x00, 0x00, 0xff, 0x04, 0x10, 0x10],
    },
    44100: {
        11289600: [0x01, 0x01, 0x01, 0x01, 0x00, 0x00, 0xff, 0x04, 0x10, 0x10],
        5644800 : [0x01, 0x02, 0x01, 0x01, 0x00, 0x00, 0xff, 0x04, 0x10, 0x10],
        2822400 : [0x01, 0x04, 0x01, 0x01, 0x00, 0x00, 0xff, 0x04, 0x10, 0x10],
        1411200 : [0x01, 0x08, 0x01, 0x01, 0x00, 0x00, 0xff, 0x04, 0x10, 0x10],
    },
    48000: {
        12288000: [0x01, 0x01, 0x01, 0x01, 0x00, 0x00, 0xff, 0x04, 0x10, 0x10],
        18432000: [0x03, 0x02, 0x01, 0x01, 0x00, 0x00, 0xff, 0x04, 0x10, 0x10],
        6144000 : [0x01, 0x02, 0x01, 0x01, 0x00, 0x00, 0xff, 0x04, 0x10, 0x10],
        3072000 : [0x01, 0x04, 0x01, 0x01, 0x00, 0x00, 0xff, 0x04, 0x10, 0x10],
        1536000 : [0x01, 0x08, 0x01, 0x01, 0x00, 0x00, 0xff, 0x04, 0x10, 0x10],
    },
    64000: {
        12288000: [0x03, 0x04, 0x01, 0x01, 0x00, 0x00, 0xff, 0x04, 0x10, 0x10],
        18432000: [0x03, 0x04, 0x03, 0x03, 0x01, 0x01, 0x7f, 0x06, 0x10, 0x10],
        16384000: [0x01, 0x01, 0x01, 0x01, 0x00, 0x00, 0xff, 0x04, 0x10, 0x10],
        8192000 : [0x01, 0x02, 0x01, 0x01, 0x00, 0x00, 0xff, 0x04, 0x10, 0x10],
        6144000 : [0x01, 0x04, 0x03, 0x03, 0x01, 0x01, 0x7f, 0x06, 0x10, 0x10],
        4096000 : [0x01, 0x04, 0x01, 0x01, 0x00, 0x00, 0xff, 0x04, 0x10, 0x10],
        3072000 : [0x01, 0x08, 0x03, 0x03, 0x01, 0x01, 0x7f, 0x06, 0x10, 0x10],
        2048000 : [0x01, 0x08, 0x01, 0x01, 0x00, 0x00, 0xff, 0x04, 0x10, 0x10],
        1536000 : [0x01, 0x08, 0x01, 0x01, 0x01, 0x00, 0xbf, 0x03, 0x18, 0x18],
        1024000 : [0x01, 0x08, 0x01, 0x01, 0x01, 0x00, 0x7f, 0x02, 0x10, 0x10],
    },
    88200: {
        11289600: [0x01, 0x02, 0x01, 0x01, 0x00, 0x00, 0xff, 0x04, 0x10, 0x10],
        5644800 : [0x01, 0x04, 0x01, 0x01, 0x00, 0x00, 0xff, 0x04, 0x10, 0x10],
        2822400 : [0x01, 0x08, 0x01, 0x01, 0x00, 0x00, 0xff, 0x04, 0x10, 0x10],
        1411200 : [0x01, 0x08, 0x01, 0x01, 0x01, 0x00, 0x7f, 0x02, 0x10, 0x10],
    },
    96000: {
        12288000: [0x01, 0x02, 0x01, 0x01, 0x00, 0x00, 0xff, 0x04, 0x10, 0x10],
        18432000: [0x03, 0x04, 0x01, 0x01, 0x00, 0x00, 0xff, 0x04, 0x10, 0x10],
        6144000 : [0x01, 0x04, 0x01, 0x01, 0x00, 0x00, 0xff, 0x04, 0x10, 0x10],
        3072000 : [0x01, 0x08, 0x01, 0x01, 0x00, 0x00, 0xff, 0x04, 0x10, 0x10],
        1536000 : [0x01, 0x08, 0x01, 0x01, 0x01, 0x00, 0x7f, 0x02, 0x10, 0x10],
    },
}

# Clock Scheme Register definition
_ES8311_CLK_MANAGER_REG01 = const(0x01) # select clk src for mclk, enable clock for codec
_ES8311_CLK_MANAGER_REG02 = const(0x02) # clk divider and clk multiplier
_ES8311_CLK_MANAGER_REG03 = const(0x03) # adc fsmode and osr
_ES8311_CLK_MANAGER_REG04 = const(0x04) # dac osr
_ES8311_CLK_MANAGER_REG05 = const(0x05) # clk divier for adc and dac
_ES8311_CLK_MANAGER_REG06 = const(0x06) # bclk inverter and divider
_ES8311_CLK_MANAGER_REG07 = const(0x07) # tri-state, lrck divider
_ES8311_CLK_MANAGER_REG08 = const(0x08) # lrck divider

# SDP
_ES8311_SDPIN_REG09  = const(0x09) # dac serial digital port
_ES8311_SDPOUT_REG0A = const(0x0A) # adc serial digital port

# SYSTEM
_ES8311_SYSTEM_REG0B = const(0x0B) # system
_ES8311_SYSTEM_REG0C = const(0x0C) # system
_ES8311_SYSTEM_REG0D = const(0x0D) # system, power up/down
_ES8311_SYSTEM_REG0E = const(0x0E) # system, power up/down
_ES8311_SYSTEM_REG0F = const(0x0F) # system, low power
_ES8311_SYSTEM_REG10 = const(0x10) # system
_ES8311_SYSTEM_REG11 = const(0x11) # system
_ES8311_SYSTEM_REG12 = const(0x12) # system, Enable DAC
_ES8311_SYSTEM_REG13 = const(0x13) # system
_ES8311_SYSTEM_REG14 = const(0x14) # system, select DMIC, select analog pga gain

# ADC
_ES8311_ADC_REG15 = const(0x15) # ADC, adc ramp rate, dmic sense
_ES8311_ADC_REG16 = const(0x16) # ADC
_ES8311_ADC_REG17 = const(0x17) # ADC, volume
_ES8311_ADC_REG18 = const(0x18) # ADC, alc enable and winsize
_ES8311_ADC_REG19 = const(0x19) # ADC, alc maxlevel
_ES8311_ADC_REG1A = const(0x1A) # ADC, alc automute
_ES8311_ADC_REG1B = const(0x1B) # ADC, alc automute, adc hpf s1
_ES8311_ADC_REG1C = const(0x1C) # ADC, equalizer, hpf s2

# DAC
_ES8311_DAC_REG31 = const(0x31) # DAC, mute
_ES8311_DAC_REG32 = const(0x32) # DAC, volume
_ES8311_DAC_REG33 = const(0x33) # DAC, offset
_ES8311_DAC_REG34 = const(0x34) # DAC, drc enable, drc winsize
_ES8311_DAC_REG35 = const(0x35) # DAC, drc maxlevel, minilevel
_ES8311_DAC_REG37 = const(0x37) # DAC, ramprate

# GPIO
_ES8311_GPIO_REG44 = const(0x44) # GPIO, dac2adc for test
_ES8311_GP_REG45   = const(0x45) # GP CONTROL

_ES8311_RESET_REG00 = const(0x00) # reset digital,csm,clock manager etc.


class ES8311:
    """Driver for ES8311 I2S Audio Codec"""

    def __init__(self, i2c, address: int = _DEFAULT_ADDRESS, debug=False):
        """Initialize the ES8311.

        :param i2c: The I2C bus the device is connected to
        :param address: The I2C device address (default is 0x18)
        """
        self.i2c_device = I2CDevice(i2c, address)
        self._debug = debug

        self.sample_rate = None
        self._bit_depth = None

        self.reset()
        time.sleep(0.01)

    def reset(self) -> bool:
        # Reset ES8311 to its default
        self._write_register(_ES8311_RESET_REG00, 0x1F)
        time.sleep(.02)
        self._write_register(_ES8311_RESET_REG00, 0x00)
        return True

    # ---------------------------
    # Low-level I2C helpers
    # ---------------------------
    def _write_register(self, reg: int, value: int) -> None:
        """Write a value to a register.

        :param reg: The register address
        :param value: The value to write
        """
        with self.i2c_device as i2c:
            i2c.write(bytes([reg, value]))
        if self._debug:
            print(f"Write 0x{value:02X} -> reg 0x{reg:02X}")

    def _read_register(self, reg: int) -> int:
        """Value from a register.

        :param reg: The register address
        :return: The register value
        """
        result = bytearray(1)
        with self.i2c_device as i2c:
            i2c.write_then_readinto(bytes([reg]), result)
        if self._debug:
            print(f"Read 0x{result[0]:02X} <- reg 0x{reg:02X}")
        return result[0]

    # ---------------------------
    # Initialization
    # ---------------------------
    def configure_clocks(
        self, sample_rate: int, bit_depth: int = 16, mclk_freq: Optional[int] = None
    ):
        """Initalize the ES8311 and Configure the clock settings.

        This function configures all necessary clock settings including PLL, dividers,
        and interface settings to achieve the requested sample rate.

        :param sample_rate: The desired sample rate in Hz (e.g.: [48000)
        :param bit_depth: The bit depth (16, 20, 24, or 32), defaults to 16
        :param mclk_freq: The main clock frequency in Hz (e.g., 12000000 for 12MHz)
                         If None (default), BCLK will be used as the PLL input source
        :return: True if successful, False otherwise
        """

        """Initialize ES8311 for DAC playback."""

        _sample_rate = sample_rate
        self.sample_rate = sample_rate
        self.bit_depth = bit_depth
        
        if coeff_div.get(_sample_rate,None) == None:
            # Search for supported sample rate close to requested
            smldiff = -1
            for rate in coeff_div:
                if smldiff == -1 or smldiff > abs(rate - sample_rate):
                        
                    smldiff = abs(rate - sample_rate)
                    _sample_rate = rate
                    
            if abs(_sample_rate - sample_rate) > 15000:
                raise ValueError("Unsupported sample rate")

        self._write_register(_ES8311_CLK_MANAGER_REG01, 0x30)
        self._write_register(_ES8311_CLK_MANAGER_REG02, 0x00)
        self._write_register(_ES8311_CLK_MANAGER_REG03, 0x10)
        self._write_register(_ES8311_ADC_REG16, 0x24)
        self._write_register(_ES8311_CLK_MANAGER_REG04, 0x10)
        self._write_register(_ES8311_CLK_MANAGER_REG05, 0x00)
        self._write_register(_ES8311_SYSTEM_REG0B, 0x00)
        self._write_register(_ES8311_SYSTEM_REG0C, 0x00)
        self._write_register(_ES8311_SYSTEM_REG10, 0x1F)
        self._write_register(_ES8311_SYSTEM_REG11, 0x7F)
        self._write_register(_ES8311_RESET_REG00, 0x80)
        self._write_register(_ES8311_DAC_REG32, 0x80) # Volume 1000 0000

        # Set in Slave mode
        regv = self._read_register(_ES8311_RESET_REG00)
        regv &= 0xBF
        self._write_register(_ES8311_RESET_REG00, regv)
        self._write_register(_ES8311_CLK_MANAGER_REG01, 0x3F)

        # Select SCLK source for inernal mclk
        # Reg 0x01, bits 3:2 = 0b00 for DAC/ADC clocks
        # Enable DAC and ADC clocks (CLKDAC_ON=1, CLKADC_ON=1)
        # bit 4: BCLK on/off
        regv = self._read_register(_ES8311_CLK_MANAGER_REG01)
        regv |= 0x80
        self._write_register(_ES8311_CLK_MANAGER_REG01, regv)

        mclk_fre = _sample_rate * 256
        if coeff_div.get(_sample_rate,{}).get(mclk_fre) == None:
            raise ValueError(f"Unable to configure samle rate {_sample_rate}Hz with {mclk_fre}Hz MCLK")
        else:
            div_record = coeff_div[_sample_rate][mclk_fre]

        # Set clock parameters (44.1Khz)
        # {11289600: [0x01, 0x00, 0x01,   0x01,   0x00,   0x00, 0xff, 0x04, 0x10, 0x10},
        # {12288000: [0x01, 0x01, 0x01,   0x01,   0x00,   0x00, 0xff, 0x04, 0x10, 0x10},
        #  mclk     rate   pre_div mult adc_div dac_div fs_mode lrch  lrcl bckdiv adc_osr dac_osr
        regv = self._read_register(_ES8311_CLK_MANAGER_REG02) & 0x07
        regv |= (div_record[PRE_DIV] - 1) << 5

        # Only used when MCLK is supplied through the ES8311 MCLK pin
        #pre_multi = div_record[PRE_MULTI]
        #if pre_multi == 2:
        #    datmp = 1
        #elif pre_multi == 4:
        #    datmp = 2
        #elif pre_multi == 8:
        #    datmp = 3
        #else:
        #    datmp = 0

        # If in Slave mode/MCLK comes from the SCLK Pin
        datmp = 3

        regv |= datmp << 3
        self._write_register(_ES8311_CLK_MANAGER_REG02, regv)

        regv = self._read_register(_ES8311_CLK_MANAGER_REG05) & 0x00
        regv |= (div_record[ADC_DIV] - 1) << 4
        regv |= (div_record[DAC_DIV] - 1) << 0
        self._write_register(_ES8311_CLK_MANAGER_REG05, regv)

        regv = self._read_register(_ES8311_CLK_MANAGER_REG03) & 0x80
        #regv |= 0x10
        regv |= div_record[FS_MODE] << 6
        regv |= div_record[ADC_OSR] << 0
        self._write_register(_ES8311_CLK_MANAGER_REG03, regv)

        regv = self._read_register(_ES8311_CLK_MANAGER_REG04) & 0x80
        #regv |= 0x10
        regv |= div_record[DAC_OSR] << 0
        self._write_register(_ES8311_CLK_MANAGER_REG04, regv)

        regv = self._read_register(_ES8311_CLK_MANAGER_REG07) & 0xC0
        regv |= div_record[LRCK_H] << 0
        self._write_register(_ES8311_CLK_MANAGER_REG07, regv)

        regv = self._read_register(_ES8311_CLK_MANAGER_REG08) & 0x00
        #regv |= 0xFF
        regv |= div_record[LRCK_L] << 0
        self._write_register(_ES8311_CLK_MANAGER_REG08, regv)

        regv = self._read_register(_ES8311_CLK_MANAGER_REG06) & 0xE0
        #regv |= (0x04 - 1)
        if div_record[BCLK_DIV] < 19:
            regv |= (div_record[BCLK_DIV] - 1) << 0
        else:
            regv |= (div_record[BCLK_DIV]) << 0
        self._write_register(_ES8311_CLK_MANAGER_REG06, regv)

        # Don't invert MCLK/SCLK
        regv = self._read_register(_ES8311_CLK_MANAGER_REG01)
        regv &= ~(0x40)   # regv |= 0x40 for inverted
        self._write_register(_ES8311_CLK_MANAGER_REG01, regv)
        regv = self._read_register(_ES8311_CLK_MANAGER_REG06)
        regv &= ~(0x20)   # regv |= 0x20 for inverted
        self._write_register(_ES8311_CLK_MANAGER_REG06, regv)

        self._write_register(_ES8311_SYSTEM_REG13, 0x10)
        self._write_register(_ES8311_ADC_REG1B, 0x0A)
        self._write_register(_ES8311_ADC_REG1C, 0x6A)
        #self._write_register(_ES8311_GPIO_REG44, 0x08)

        # es8311_start
        # es_mode  = CODEC_MODE_DECODE
        dac_iface = self._read_register(_ES8311_SDPIN_REG09) & 0xBF
        adc_iface = self._read_register(_ES8311_SDPOUT_REG0A) & 0xBF
        adc_iface |= (1 << 6)
        dac_iface |= (1 << 6)
        # DECODE MODE
        dac_iface &= ~(1 << 6)

        self._write_register(_ES8311_SDPIN_REG09, dac_iface)
        self._write_register(_ES8311_SDPOUT_REG0A, adc_iface)

        self._write_register(_ES8311_ADC_REG17, 0xBF)
        self._write_register(_ES8311_SYSTEM_REG0E, 0x02)
        self._write_register(_ES8311_SYSTEM_REG12, 0x00)
        self._write_register(_ES8311_SYSTEM_REG14, 0x1A)

        # Disable digital MIC
        regv = self._read_register(_ES8311_SYSTEM_REG14)
        regv &= ~(0x40)
        self._write_register(_ES8311_SYSTEM_REG14, regv)

        self._write_register(_ES8311_SYSTEM_REG0D, 0x01)
        self._write_register(_ES8311_ADC_REG15, 0x40)
        self._write_register(_ES8311_DAC_REG37, 0x48)
        self._write_register(_ES8311_GP_REG45, 0x00)

    @staticmethod
    def _convert_reg_to_db(regval: int) -> float:
        """
        Convert a register value to decibel volume.

        :param regval: 8-bit register value
        :return: Volume in dB
        """

        return ((regval/255) * 127.5) - 95.5

    @staticmethod
    def _convert_db_to_reg(db: float) -> int:
        """
        Convert decibel volume to register value.

        :param db: Volume in dB (-95.5 to 32 dB)
        :return: 8-bit register value
        """
        if db > 32:
            db = 32
        elif db < -95.5:
            db = -95.5

        regval = int(((db + 95.5) / (127.5)) * 255)

        return regval & 0xFF

    # ---------------------------
    # Volume Control
    # ---------------------------
    @property
    def dac_volume(self) -> float:
        """Read current DAC volume """

        regval = self._read_register(_ES8311_DAC_REG32)

        return self._convert_reg_to_db(regval)

    @dac_volume.setter
    def dac_volume(self, volume_db: float) -> None:
        """Set DAC volume from -95.5 dB to -32 dB in 0.5 dB steps."""

        regval = self._convert_db_to_reg(volume_db)

        self._write_register(_ES8311_DAC_REG32, regval)

    def mute(self) -> None:
        """Mute DAC """
        regval = self._read_register(_ES8311_DAC_REG31)
        self._write_register(_ES8311_DAC_REG31, regval & 0x1F)

    def unmute(self) -> None:
        """Unmute DAC """
        regval = self._read_register(_ES8311_DAC_REG31)
        self._write_register(_ES8311_DAC_REG31, regval | 0xE0)
