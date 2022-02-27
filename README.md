# Driver for VL53L5CX in Python

Python transliteration of VL53L5CX ultra lite driver from C to Python.
More precisely, it is implementation from this github repository:
https://github.com/thingswebuilt/VL53L5CX/tree/minimal_mods_for_pi

(Thanks Mark for cracking i2c data size and addressing of chunks for Raspberry Pi)

Note: Current implementation is tested on Raspberry Pi with SMBus2 for i2c communication,
  but in theory it should work on any system that implements SMBus2.

I tried to preserve as much original comments in the code as well.


## Examples

Currently there is only one example for simple ranging. To run it on Raspberry Pi:

```bash
PYTHONPATH=`pwd` python3 examples/simple_ranging_example.py
```
