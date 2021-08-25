# SMWiredPy

Sensemore Wired vibration sensor Python interface 

 ![Tux, the Linux mascot](img/wired_image.jpg)

## Installing the library

- Using pip
```python
	pip install SMWired
```
- Build from source

```python
	#Inside the library folder
	pip install .
```
## Requirements
- pybind11
- pyserial
- SMComPy (pip install SMComPy)

## Example usage

``` python

from sensemore import SMWired

wired_network = SMWired.SMWired(port = "/dev/ttyUSB0", configure_network='auto')
#Dump the list of found available devices
print(wired_network.get_available_devices())

devices = wired_network.get_available_devices()

#Print the version of the devices
for device in devices:
	print("Version of '%s' is %s"%(device,wired_network.get_version(device)))


#Take measurement from a specific device in the network

mac = 'CA:B8:31:00:00:55'
accelerometer_range = "16G"
sampling_frequency = 12800
sample_size = 10000

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
