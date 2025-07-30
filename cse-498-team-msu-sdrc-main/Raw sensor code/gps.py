import time
from serial import Serial
from pynmeagps import NMEAReader
from pynmeagps import latlon2dms, latlon2dmm

SERIAL_PORT = "COM5"   #"/dev/ttyUSB0" 
BAUDRATE = 4800
TIMEOUT = 3

stream = Serial(SERIAL_PORT, BAUDRATE, timeout=TIMEOUT)
nmr = NMEAReader(stream)
'''python3 -m pip install --user --upgrade virtualenv
python3 -m virtualenv env
source env/bin/activate (or env\Scripts\activate on Windows)
...
deactivate'''

def gnss_data():
    while True:
        (raw, parsed) = nmr.read()
        if parsed:
            try:
                print(f"Latitude: {parsed.lat}, Longitude: {parsed.lon}")
            except:
                pass 

gnss_data()
