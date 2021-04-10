adis16490
=====
Object-oriented library for working with a ADIS1649x sensors

Usage
-----


### Using the `Adis1649x` object

The `Adis1649x` module provides an object-oriented interface to the sensor,
making code development more intuitive. An example of its use:

```python
>>> from adis1649x import Adis1649x, SensorType, Axis
>>> sensor = Adis1649x(16490) 
>>> sensor.adis_prod_id
16490
>>> sensor.serial_num
128
>>> sensor.z_gyro
0.234
>>> sensor.x_accl 
986.237
>>> sensor.bias_set(SensorType.gyro, Axis.z, 0.75)
>>> sensor.reset
```
