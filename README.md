# i2c_bus_analyzer
I2C bus analyzer for the Raspberry Pi Pico

# Connections
GPIO 0 must be connected to GPIO 2.
GPIO 1 must be connected to GPIO 3.
The device under test can be connected to GPIO 0 (SDA) and GPIO 1 (SCL).

# Python setup
Install the necessary modules first:
```bash
sudo apt install python3.11-venv
python3 -m venv pythonenv
source pythonenv/bin/activate
pip install pyserial
```

# Running the python script
You can specify the number of seconds to capture data. Or you can do a scan to find all the devices on the bus. Or you can do both.
```bash
python i2c.py 10
python i2c.py scan
python i2c.py 10 scan
```
