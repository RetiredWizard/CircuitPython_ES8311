# CircuitPython_ES8311
CircuitPython driver for ES8311 I2S audio codec

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
