# SMWiredPy
[Sensemore](https://sensemore.io) - Wired vibration sensor Python interface 

![sm](img/sm.webp)


![Wired](img/wired.webp)


## Installing the library

- Using pip
```bash
	$ pip install SMWired
```
- Build from source

```bash
	#Inside the library folder
	$ pip install .
```
## Requirements
- pybind11
- pyserial
- SMComPy (pip install SMComPy)

## Example usage

``` python

from sensemore import SMWiredPy

wired_network = SMWiredPy.SMWired(port = "/dev/ttyUSB0", configure_network='auto', max_device_number=2)
#Dump the list of found available devices
print("Found available devices:",wired_network.get_available_devices())

devices = wired_network.get_available_devices()

#Print the version of the devices
for device in devices:
	print("Version of '%s' is %s"%(device,wired_network.get_version(device)))


#Take measurement from a specific device in the network

mac = 'CA:B8:31:00:00:55'
accelerometer_range = "16G"
sampling_frequency = 12800
sample_size = 100

measurement_result = wired_network.measure(mac,accelerometer_range,sampling_frequency,sample_size)

result_acc_x = measurement_result[0]
result_acc_y = measurement_result[1]
result_acc_z = measurement_result[2]

"""
Also there are telemetries calculated in wired, we can also take it by calling get_all_telemetry
"""

telemetries = wired_network.get_all_telemetry(mac)
print(telemetries)


```

## Available sampling frequencies

```
- 800 Hz
- 1600 Hz
- 3200 Hz
- 6400 Hz
- 12800 Hz
```

## Available accelerometer ranges

```
- 2G
- 4G
- 8G
- 16G
```


## Command line interface


- Updating the wired device via cli
```bash
python -m sensemore.SMWiredPy update --port=/dev/ttyUSB0 --mac=CA:B8:31:00:00:3C --file=Wiredv1_0_13.bin 
```
- Fast measurement via cli
```
python -m sensemore.SMWiredPy measure --port=/dev/ttyUSB0 --mac=CA:B8:31:00:00:3C --sample=1000 --freq=12800 -acc=16G -t
```

