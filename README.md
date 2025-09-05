# CircuitPython_ES8311
CircuitPython driver for ES8311 I2S audio codec

A debt of gratitued is owed to @pschatzmann for 
[arduino-audio-driver](https://github.com/pschatzmann/arduino-audio-driver) and [arduino-audio-tools](https://github.com/pschatzmann/arduino-audio-tools)


circuitpython_es8311.py - minimal i2s audio driver for es8311 codec

    * line in/mic input not supported
    * Currently no way to run es8311 in "Master" mode
    * MCLK must be derived from SCLK
    * Currently no way to utilize inverted clock signals (MCLK/SCLK)

adafruit_tlv320.py - Shell library to allow applications written to use the TLV320 I2S DAC chip to run on boards using the ES8311 chip without modification (assuming the above limitations of circuitpython_es8311.py don't apply).


Implementation Notes
--------------------

The [Olimex RP2350pc](https://www.olimex.com/Products/RaspberryPi/PICO/RP2350pc/)
board has a GPIO pin connected to the ES8311 power enable
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

Notes Regarding the Circuit Python firmware
-------------------------------------------
(Olimex_RP2350pc_Beta_CP10.x.zip)

The RP2350pc has a 4 port USB hub connected in parallel to the USB device port which prevents the HUB from being used
at the same time as the USB device (CIRCUITPY drive) in CircuitPython. This makes it a little more difficult to
transfer files/programs to the internal flash of the device, however a standard FTDI to serial (or similar) cable
can be connected to UEXT1 pins 2(GND), 3(GPIO0-TX) and 4(GPIO1-RX) over which Circuit Python will provide a serial REPL interface.
Once this serial connection has been made to a host computer, Thonny or other microcontroller IDEs can be used to transfer files and run
programs. In addition, CircuitPython will auto-mount any insrted SD card which can be used to transfer files as well. 

Another possible option would be to flash the RP2350pc with [Adafruit Feather RP2350](https://circuitpython.org/board/adafruit_feather_rp2350/)
firmware from circuitpython.org. Using the Feather
firmware the RP2350 peripherals will not work, however it will mount a CIRCUITPY drive. You can then copy the files you want
on the internal flash and re-flash with the RP2350pc firmware at which point you should be able to interact with the 
REPL over the HDMI display and a keyboard plugged in to a USB hub port. If one of the files you placed on the internal
flash in this manner was [PyDOS](https://github.com/RetiredWizard/PyDOS) (only the PyDOS.py file itself is required) then it
will be trivial to move future files back and forth between inserted SD cards.
