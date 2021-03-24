"""SPI driver for the Analog Devices IMU sensors ADIS1649x.

Classes:

    SensorType
    Axis
    Adis1649x

Functions:

    _spi_read (object, int) -> int
    _spi_write(object, int, int)

"""
import time
from enum import Enum
import atexit
import spidev
import RPi.GPIO as GPIO


# Scale Factors [Gyroscope, Accelerometer, Thermometer]
coefficient_adis16490 = [0.005, 0.5, 0.01429]
coefficient_adis16495 = [0.00625, 0.25, 0.0125]
# coefficient_adis16495-2 = [0.025, 0.25]

# ADIS 16490 USER REGISTER MEMORY MAP
_PAGE_ID = 0x00  # Same for all pages

# PAGE 0x00
_DATA_CNT = 0x04
_SYS_E_FLAG = 0x08
_DIAG_STS = 0x0A
_TEMP_OUT = 0x0E
_X_GYRO_LOW = 0x10
_X_GYRO_OUT = 0x12
_Y_GYRO_LOW = 0x14
_Y_GYRO_OUT = 0x16
_Z_GYRO_LOW = 0x18
_Z_GYRO_OUT = 0x1A
_X_ACCL_LOW = 0x1C
_X_ACCL_OUT = 0x1E
_Y_ACCL_LOW = 0x20
_Y_ACCL_OUT = 0x22
_Z_ACCL_LOW = 0x24
_Z_ACCL_OUT = 0x26
_TIME_STAMP = 0x28
_X_DELTANG_LOW = 0x40
_X_DELTANG_OUT = 0x42
_Y_DELTANG_LOW = 0x44
_Y_DELTANG_OUT = 0x46
_Z_DELTANG_LOW = 0x48
_Z_DELTANG_OUT = 0x4A
_X_DELTVEL_LOW = 0x4C
_X_DELTVEL_OUT = 0x4E
_Y_DELTVEL_LOW = 0x50
_Y_DELTVEL_OUT = 0x52
_Z_DELTVEL_LOW = 0x54
_Z_DELTVEL_OUT = 0x56
_PROD_ID = 0x7E
_GYRO_ROW = [0x10, 0, 0x12, 0, 0x14, 0, 0x16, 0, 0x18, 0, 0x1A, 0, 0, 0, 0, 0]
_ACCL_ROW = [0x1C, 0, 0x1E, 0, 0x20, 0, 0x22, 0, 0x24, 0, 0x26, 0, 0, 0, 0, 0]


# PAGE 0x01 Reserved

# PAGE 0x02
_X_GYRO_SCALE = 0x04
_Y_GYRO_SCALE = 0x06
_Z_GYRO_SCALE = 0x08
_X_ACCL_SCALE = 0x0A
_Y_ACCL_SCALE = 0x0C
_Z_ACCL_SCALE = 0x0E
_XG_BIAS_LOW = 0x10
_XG_BIAS_HIGH = 0x12
_YG_BIAS_LOW = 0x14
_YG_BIAS_HIGH = 0x16
_ZG_BIAS_LOW = 0x18
_ZG_BIAS_HIGH = 0x1A
_XA_BIAS_LOW = 0x1C
_XA_BIAS_HIGH = 0x1E
_YA_BIAS_LOW = 0x20
_YA_BIAS_HIGH = 0x22
_ZA_BIAS_LOW = 0x24
_ZA_BIAS_HIGH = 0x26
_USER_SCR_1 = 0x74
_USER_SCR_2 = 0x76
_USER_SCR_3 = 0x78
_USER_SCR_4 = 0x7A
_FLSHCNT_LOW = 0x7C
_FLSHCNT_HIGH = 0x7E

# PAGE 0x03
_GLOB_CMD = 0x02
_FNCTIO_CTRL = 0x06
_GPIO_CTRL = 0x08
_CONFIG = 0x0A
_DEC_RATE = 0x0C
_NULL_CNFG = 0x0E
_SYNC_SCALE = 0x10
_FILTR_BANK_0 = 0x16
_FILTR_BANK_1 = 0x18
_FIRM_REV = 0x78
_FIRM_DM = 0x7A
_FIRM_Y = 0x7C
_BOOT_REV = 0x7E
# autocalib_pause = [0.02, 0.04, 0.07, 0.2, 0.3, 0.5, 1, 2, 4, 8, 16, 31, 62, 124]
autocalib_pause = [1, 1, 1, 1, 1, 1, 2, 3, 8, 10, 20, 40, 70, 130]

# PAGE 0x04
_CAL_SIGTR_LWR = 0x04
_CAL_SIGTR_UPR = 0x06
_CAL_DRVTN_LWR = 0x08
_CAL_DRVTN_UPR = 0x0A
_CODE_SIGTR_LWR = 0x0C
_CODE_SIGTR_UPR = 0x0E
_CODE_DRVTN_LWR = 0x10
_CODE_DRVTN_UPR = 0x12
_SERIAL_NUM = 0x20

IRQ = 6  # GPIO connect to Interrupt request
CS = 8  # GPIO connect to chip select
spi = spidev.SpiDev()  # Creating an SPI object
spi.open(0, 0)  # Selecting the port number and device number (CS) of the SPI bus
spi.max_speed_hz = 8000000  # Setting the maximum speed of the SPI bus
spi.mode = 3  # Selecting the SPI operation mode (from 0 to 3)
GPIO.setmode(GPIO.BCM)  # Selecting the GPIO pin numbering mode
GPIO.setup(IRQ, GPIO.IN)  # Interrupt request
GPIO.setup(CS, GPIO.OUT)  # Chip select
GPIO.output(CS, GPIO.HIGH)  # Setting the CS signal level to high


class SensorType(Enum):
    """Sensor Type Enumeration class"""
    gyro = 'gyro'
    accl = 'accl'


class Axis(Enum):
    """Axis Enumeration Class"""
    x = 'x'
    y = 'y'
    z = 'z'


class Adis1649x:
    """The class of the ADIS 1649x sensor that implements an interface for interacting with it

    Raises:
        AttributeError: Occurs if the read id does not match the sensor id
        TypeError: A variable of the wrong type was entered
        ValueError: The method gets an argument of the correct type, but an incorrect value
    """

    def __init__(self, prod_id=16490):
        global current_page, gyro_scale, accl_scale, temp_scale
        """Checking the sensor ID"""
        self._set(_PAGE_ID, 0x00)
        current_page = 0
        adis_prod_id = self._get(_PROD_ID)
        if adis_prod_id != prod_id:
            raise RuntimeError(
                f"Failed to find ADIS {prod_id}! Chip ID {adis_prod_id}")
        if prod_id == 16490:
            scale_factors = coefficient_adis16490
        elif prod_id == 16495:
            scale_factors = coefficient_adis16495

        gyro_scale = scale_factors[0]
        accl_scale = scale_factors[1]
        temp_scale = scale_factors[2]

    def _get(self, reg):
        """SPI bus data reading function

        Args:
            reg ([int]): The number of the register to read

        Returns:
            self.value [int]: Output value
        """
        GPIO.wait_for_edge(IRQ, GPIO.FALLING)
        spi.writebytes([reg, 0])  # Sending bytes over the SPI bus
        # We read 2 bytes over the SPI bus. As a result, we get a list of two values [x1, x2]
        value = spi.readbytes(2)
        # Converting a value to 16-bit
        self.value = ((value[0] << 8) | value[1])
        return self.value

    def _set(self, reg, value):
        """Function that records data over the SPI bus

        Args:
            spi ([object]): Created SpiDev object
            reg ([int]): The number of the register to write the data to
            value ([int]): Value to write to a register
        """
        GPIO.wait_for_edge(IRQ, GPIO.FALLING)
        send = [0]*2
        # Specify the highest bit for the record (OR 0x80)
        send[0] = 0x80 | reg
        send[1] = value
        spi.writebytes(send)  # Sending bytes over the SPI bus

    def _select_page(self, page):
        """Method for switching the register page

        Args:
            page ([int]): The number of the page to switch to
        """
        global current_page
        if current_page != page:
            self._set(_PAGE_ID, page)
            current_page = page

    def _comb_16_into_32(self, high, low):
        """A method for combining 16 bit numbers into 32 bits by shifting the highest bits to the left and the logical sum

        Args:
            high ([int]): High 16 bits
            low ([int]): Low 16 bits

        Returns:
            [int]: 32 bit number
        """
        bit32 = ((high << 16) | (low & 0xFFFF))
        return bit32

    def _comb_8_into_32(self, low_senior, low_junior, high_senior, high_junior):
        """A method for combining 8 bit numbers into 32 bits by shifting the highest bits to the left and the logical sum

        Args:
            low_senior ([int]): The highest 8 bits of the lowest 16 bit number
            low_junior ([int]): The lower 8 bits of the lower 16 bit number
            high_senior ([int]): The highest 8 bits of the highest 16 bit number
            high_junior ([int]): The lower 8 bits of the higher 16 bit number

        Returns:
            [int]: 32 bit number
        """
        low16 = ((low_senior << 8) | low_junior)
        high16 = ((high_senior << 8) | high_junior)
        bit32 = ((high16 << 16) | (low16 & 0xFFFF))
        return bit32

    def _check(self, value, bits):
        """Checking the value for a sign. If the last bit of the number is equal to one,
        then we translate from the additional code to the direct code and change the last digit to a sign "-"

        Args:
            value ([int]): Value to check
            bits ([int]): The number of bits in the value number

        Returns:
            [int]: Converted value
        """
        if((value & (1 << (bits-1))) != 0):
            value = value - (1 << bits)
        return value

    @property
    def serial_num(self):
        self._select_page(0x04)
        self.serial_number = self._get(_SERIAL_NUM)
        return self.serial_number

    @property
    def temp(self):
        """Output of the temperature value
        Formula for representing a number from a dataset:
        0.01429 * temp_raw + 25

        Returns:
            [float]: Converted temperature value
        """
        self._select_page(0x00)
        temp_raw = self._get(_TEMP_OUT)
        temp_raw = self._check(temp_raw, 16)
        return temp_scale * temp_raw + 25

    @property
    def decrate(self):
        """Getter for reading the decrate value
        Decrate determines the frequency of values issued by the sensor.
        Formula: f = 4250 / (decrate + 1)

        Returns:
            [int]: The value of decorate
        """
        self._select_page(0x03)
        self.decrate_ = self._get(_DEC_RATE)
        return self.decrate_

    # Добавить проверку диапазона
    @decrate.setter
    def decrate(self, value):
        """Setter for changing the decorate

        Args:
            value ([int]): Input value

        Raises:
            TypeError: [description] The error occurs when the input value is not int
        """
        if not isinstance(value, int):
            raise TypeError("Decrate type must be int")
        self._decrate_low = value & 0xff
        self._decrate_high = (value >> 8) & 0xff
        self._select_page(0x03)
        self._set(_DEC_RATE, self._decrate_low)
        self._set(_DEC_RATE + 1, self._decrate_high)

    @property
    def x_gyro(self):
        """Getter for displaying the values of the X-axis gyroscope

        Returns:
            [float]: Преобразованое значение гироскопа
        """
        self._select_page(0x00)
        self.x_gyro_low = self._get(_X_GYRO_LOW)
        self.x_gyro_out = self._get(_X_GYRO_OUT)
        self.x_gyro_32 = self._comb_16_into_32(
            self.x_gyro_out, self.x_gyro_low)
        self.x_gyro_32 = self._check(
            self.x_gyro_32, 32)
        return self.x_gyro_32 * gyro_scale / 65536

    @property
    def x_accl(self):
        """Getter for displaying the values of the X-axis accelerometer

        Returns:
            [float]: Converted accelerometer value
        """
        self._select_page(0x00)
        self.x_accl_low = self._get(_X_ACCL_LOW)
        self.x_accl_out = self._get(_X_ACCL_OUT)
        self.x_accl_32 = self._comb_16_into_32(
            self.x_accl_out, self.x_accl_low)
        self.x_accl_32 = self._check(
            self.x_accl_32, 32)
        return self.x_accl_32 * accl_scale / 65536

    @property
    def y_gyro(self):
        """Getter for displaying the values of the Y-axis gyroscope

        Returns:
            [float]: Converted gyroscope value
        """
        self._select_page(0x00)
        self.y_gyro_low = self._get(_Y_GYRO_LOW)
        self.y_gyro_out = self._get(_Y_GYRO_OUT)
        self.y_gyro_32 = self._comb_16_into_32(
            self.y_gyro_out, self.y_gyro_low)
        self.y_gyro_32 = self._check(
            self.y_gyro_32, 32)
        return self.y_gyro_32 * gyro_scale / 65536

    @property
    def y_accl(self):
        """Getter for displaying the values of the Y-axis accelerometer

        Returns:
            [float]: Converted accelerometer value
        """
        self._select_page(0x00)
        self.y_accl_low = self._get(_Y_ACCL_LOW)
        self.y_accl_out = self._get(_Y_ACCL_OUT)
        self.y_accl_32 = self._comb_16_into_32(
            self.y_accl_out, self.y_accl_low)
        self.y_accl_32 = self._check(
            self.y_accl_32, 32)
        return self.y_accl_32 * accl_scale / 65536

    @property
    def z_gyro(self):
        """Getter for displaying the values of the Z-axis gyroscope

        Returns:
            [float]: Converted gyroscope value
        """
        self._select_page(0x00)
        self.z_gyro_low = self._get(_Z_GYRO_LOW)
        self.z_gyro_out = self._get(_Z_GYRO_OUT)
        self.z_gyro_32 = self._comb_16_into_32(
            self.z_gyro_out, self.z_gyro_low)
        self.z_gyro_32 = self._check(
            self.z_gyro_32, 32)
        return self.z_gyro_32 * gyro_scale / 65536

    @property
    def z_accl(self):
        """Getter for displaying the values of the Z-axis accelerometer

        Returns:
            [float]: Converted accelerometer value
        """
        self._select_page(0x00)
        self.z_accl_low = self._get(_Z_ACCL_LOW)
        self.z_accl_out = self._get(_Z_ACCL_OUT)
        self.z_accl_32 = self._comb_16_into_32(
            self.z_accl_out, self.z_accl_low)
        self.z_accl_32 = self._check(
            self.z_accl_32, 32)
        return self.z_accl_32 * accl_scale / 65536

    @property
    def gyro_axes(self):
        """Simultaneous reading of three gyroscope axes

        Returns:
            [list]: List of gyroscope values [X-Axis, Y-Axis, Z-Axis]
        """
        self._select_page(0x00)
        GPIO.wait_for_edge(IRQ, GPIO.FALLING)
        gyro_row = spi.xfer3(_GYRO_ROW)
        x_gyro_low_senior = gyro_row[4]
        x_gyro_low_junior = gyro_row[5]
        x_gyro_high_senior = gyro_row[6]
        x_gyro_high_junior = gyro_row[7]
        self.x_gyro_32 = self._comb_8_into_32(
            x_gyro_low_senior, x_gyro_low_junior, x_gyro_high_senior, x_gyro_high_junior)
        self.x_gyro_32 = self._check(
            self.x_gyro_32, 32)

        y_gyro_low_senior = gyro_row[8]
        y_gyro_low_junior = gyro_row[9]
        y_gyro_high_senior = gyro_row[10]
        y_gyro_high_junior = gyro_row[11]
        self.y_gyro_32 = self._comb_8_into_32(
            y_gyro_low_senior, y_gyro_low_junior, y_gyro_high_senior, y_gyro_high_junior)
        self.y_gyro_32 = self._check(
            self.y_gyro_32, 32)

        z_gyro_low_senior = gyro_row[12]
        z_gyro_low_junior = gyro_row[13]
        z_gyro_high_senior = gyro_row[14]
        z_gyro_high_junior = gyro_row[15]
        self.z_gyro_32 = self._comb_8_into_32(
            z_gyro_low_senior, z_gyro_low_junior, z_gyro_high_senior, z_gyro_high_junior)
        self.z_gyro_32 = self._check(
            self.z_gyro_32, 32)

        self.gyro_list = [(self.x_gyro_32 * gyro_scale / 65536), (self.y_gyro_32 *
                                                                  gyro_scale / 65536), (self.z_gyro_32 * gyro_scale / 65536)]
        return self.gyro_list

    # не доделал
    # Чтение трёх осей акселерометра
    @property
    def accl_axes(self):
        """Simultaneous reading of three accelerometer axes

        Returns:
            [list]: List of accelerometer values [X-Axis, Y-Axis, Z-Axis]
        """
        self._select_page(0x00)
        GPIO.wait_for_edge(IRQ, GPIO.FALLING)
        accl_row = spi.xfer3(_ACCL_ROW)
        x_accl_low_senior = accl_row[4]
        x_accl_low_junior = accl_row[5]
        x_accl_high_senior = accl_row[6]
        x_accl_high_junior = accl_row[7]
        self.x_accl_32 = self._comb_8_into_32(
            x_accl_low_senior, x_accl_low_junior, x_accl_high_senior, x_accl_high_junior)
        self.x_accl_32 = self._check(
            self.x_accl_32, 32)
        y_accl_low_senior = accl_row[8]
        y_accl_low_junior = accl_row[9]
        y_accl_high_senior = accl_row[10]
        y_accl_high_junior = accl_row[11]
        self.y_accl_32 = self._comb_8_into_32(
            y_accl_low_senior, y_accl_low_junior, y_accl_high_senior, y_accl_high_junior)
        self.y_accl_32 = self._check(
            self.y_accl_32, 32)
        z_accl_low_senior = accl_row[12]
        z_accl_low_junior = accl_row[13]
        z_accl_high_senior = accl_row[14]
        z_accl_high_junior = accl_row[15]
        self.z_accl_32 = self._comb_8_into_32(
            z_accl_low_senior, z_accl_low_junior, z_accl_high_senior, z_accl_high_junior)
        self.z_accl_32 = self._check(
            self.z_accl_32, 32)

        self.accl_list = [(self.x_accl_32 * accl_scale / 65536), (self.y_accl_32 *
                                                                  accl_scale / 65536), (self.z_accl_32 * accl_scale / 65536)]
        return self.accl_list

    def _scale(self, value):
        """Convert the entered 32 bit scale value to two 8 bit numbers

        Args:
            value ([int]): The entered scale value

        Returns:
            [int]: two 8 bit values
        """
        bits = 16
        # Scaling the number to the format from 32768 to 65535
        if (-1 <= value < 0):
            value = ((value + 1) * 32767) + 32768
        # Scaling the number to the format from 0 to 32767
        elif (0 <= value <= 1):
            value = (value * 32767)
        # Rounding and converting to integer
        value = int(round(value))
        # If the number is negative, then we translate it into additional code
        if value < 0:
            value = value + (1 << bits)
        self._scale_low = value & 0xff
        self._scale_high = (value >> 8) & 0xff
        return self._scale_low, self._scale_high

    def _bias(self, value, coefficient, bits):
        """Convert the entered 32 bit bias value to four 8 bit numbers

        Args:
            value ([int]): value to convert
            coefficient ([int]): conversion factor
            bits ([int]): number of bits in a number
        Returns:
            [int]: four 8 bit values
        """
        value = value / coefficient
        value = int(round(value))
        if(value < 0):
            value = value + (1 << bits)
        self._bias_low = value & 0xFFFF
        self._bias_high = (value >> 16) & 0xFFFF
        self._bias_low1 = self._bias_low & 0xff
        self._bias_low2 = (self._bias_low >> 8) & 0xff
        self._bias_high1 = self._bias_high & 0xff
        self._bias_high2 = (self._bias_high >> 8) & 0xff
        return self._bias_low1, self._bias_low2, self._bias_high1, self._bias_high2

    def scale_set(self, sensor_type, axis, value):
        """Method for writing the scale value entered by the user to the sensor memory

        Args:
            sensor_type ([SensorType]): the type of sensor in which the entered value will be written to the registers
            axis ([Axis]): name of the axis to write the value to
            value ([int]): value scale

        Raises:
            TypeError: The error occurs if the user entered an incorrect sensor type value or axis name
            ValueError: The error occurs if the entered value is out of bounds from the dataset.

        Returns:
            [str]: The user is returned the values they entered
        """
        if not (isinstance(sensor_type, SensorType) or isinstance(axis, Axis) or isinstance(value, (int, float))):
            raise TypeError("Invalid data type entered")
        if not (-1 <= value <= 1):
            raise ValueError("The value is out of the allowed range")
        self._scale_value = value
        self._scale_low, self._scale_high = self._scale(self._scale_value)
        self._select_page(0x02)
        if sensor_type == SensorType.gyro:
            if axis == Axis.x:
                self._set(_X_GYRO_SCALE, self._scale_low)
                self._set(_X_GYRO_SCALE + 1, self._scale_high)
            elif axis == Axis.y:
                self._set(_Y_GYRO_SCALE, self._scale_low)
                self._set(_Y_GYRO_SCALE + 1, self._scale_high)
            elif axis == Axis.z:
                self._set(_Z_GYRO_SCALE, self._scale_low)
                self._set(_Z_GYRO_SCALE + 1, self._scale_high)
        elif sensor_type == SensorType.accl:
            if axis == Axis.x:
                self._set(_X_ACCL_SCALE, self._scale_low)
                self._set(_X_ACCL_SCALE + 1, self._scale_high)
            elif axis == Axis.y:
                self._set(_Y_ACCL_SCALE, self._scale_low)
                self._set(_Y_ACCL_SCALE + 1, self._scale_high)
            elif axis == Axis.z:
                self._set(_Z_ACCL_SCALE, self._scale_low)
                self._set(_Z_ACCL_SCALE + 1, self._scale_high)
        return f'{sensor_type}, {axis}, {value}'

    def scale_get(self, sensor_type, axis):
        """Method for getting the scale values read from the sensor memory

        Args:
            sensor_type ([SensorType]): the type of sensor from which the value scale will be read from the register
            axis ([Axis]): the name of the axis from which the value will be read from the register

        Raises:
            TypeError: The error occurs if the user entered an incorrect sensor type value or axis name.

        Returns:
            [float]: the value read from the specified register
        """
        if not (isinstance(sensor_type, SensorType) or isinstance(axis, Axis)):
            raise TypeError("Invalid data type entered")
        self._select_page(0x02)
        if sensor_type == SensorType.gyro:
            if axis == Axis.x:
                self.scale_out = self._get(_X_GYRO_SCALE)
            elif axis == Axis.y:
                self.scale_out = self._get(_Y_GYRO_SCALE)
            elif axis == Axis.z:
                self.scale_out = self._get(_Z_GYRO_SCALE)
        elif sensor_type == SensorType.accl:
            if axis == Axis.x:
                self.scale_out = self._get(_X_ACCL_SCALE)
            elif axis == Axis.y:
                self.scale_out = self._get(_Y_ACCL_SCALE)
            elif axis == Axis.z:
                self.scale_out = self._get(_Z_ACCL_SCALE)
        # Масштабируем данные
        if self.scale_out <= 32767:
            self.scale_out = (self.scale_out / 32768)
        elif self.scale_out >= 32768:
            self.scale_out = ((self.scale_out - 32768) / 32768) - 1
        return self.scale_out

    def bias_set(self, sensor_type, axis, value):
        """Method for writing the bias value entered by the user into the sensor memory

        Args:
            sensor_type ([SensorType]): the type of sensor in which the entered value will be written to the registers
            axis ([Axis]): name of the axis to write the value to
            value ([int]): value bias

        Raises:
            TypeError: The error occurs if the user entered an incorrect sensor type value or axis name.

        Returns:
            [str]: The user is returned the values they entered
        """
        if not (isinstance(sensor_type, SensorType) or isinstance(axis, Axis) or isinstance(value, (int, float))):
            raise TypeError("Invalid data type entered")
        self._select_page(0x02)
        self._scale_value = value
        if sensor_type == SensorType.gyro:
            self._bias_low1, self._bias_low2, self._bias_high1, self._bias_high2 = self._bias(
                value, gyro_scale/65536, 32)
            if axis == Axis.x:
                self._set(_XG_BIAS_LOW, self._bias_low1)
                self._set(_XG_BIAS_LOW + 1, self._bias_low2)
                self._set(_XG_BIAS_HIGH, self._bias_high1)
                self._set(_XG_BIAS_HIGH + 1, self._bias_high2)
            elif axis == Axis.y:
                self._set(_YG_BIAS_LOW, self._bias_low1)
                self._set(_YG_BIAS_LOW + 1, self._bias_low2)
                self._set(_YG_BIAS_HIGH, self._bias_high1)
                self._set(_YG_BIAS_HIGH + 1, self._bias_high2)
            elif axis == Axis.z:
                self._set(_ZG_BIAS_LOW, self._bias_low1)
                self._set(_ZG_BIAS_LOW + 1, self._bias_low2)
                self._set(_ZG_BIAS_HIGH, self._bias_high1)
                self._set(_ZG_BIAS_HIGH + 1, self._bias_high2)
        elif sensor_type == SensorType.accl:
            self._bias_low1, self._bias_low2, self._bias_high1, self._bias_high2 = self._bias(
                value, accl_scale/65536, 32)
            if axis == Axis.x:
                self._set(_XA_BIAS_LOW, self._bias_low1)
                self._set(_XA_BIAS_LOW + 1, self._bias_low2)
                self._set(_XA_BIAS_HIGH, self._bias_high1)
                self._set(_XA_BIAS_HIGH + 1, self._bias_high2)
            elif axis == Axis.y:
                self._set(_YA_BIAS_LOW, self._bias_low1)
                self._set(_YA_BIAS_LOW + 1, self._bias_low2)
                self._set(_YA_BIAS_HIGH, self._bias_high1)
                self._set(_YA_BIAS_HIGH + 1, self._bias_high2)
            elif axis == Axis.z:
                self._set(_ZA_BIAS_LOW, self._bias_low1)
                self._set(_ZA_BIAS_LOW + 1, self._bias_low2)
                self._set(_ZA_BIAS_HIGH, self._bias_high1)
                self._set(_ZA_BIAS_HIGH + 1, self._bias_high2)
        return f'{sensor_type}, {axis}, {value}'

    def bias_get(self, sensor_type, axis):
        """Method for getting the bias values read from the sensor memory

        Args:
            sensor_type ([SensorType]): the type of sensor whose register the value will be read from scale
            axis ([Axis]): the name of the axis from which the value will be read from the register

        Raises:
            TypeError: The error occurs if the user entered an incorrect sensor type value or axis name

        Returns:
            [float]: the value read from the specified register
        """
        if not (isinstance(sensor_type, SensorType) or isinstance(axis, Axis)):
            raise TypeError("Invalid data type entered")
        self._select_page(0x02)
        if sensor_type == SensorType.gyro:
            if axis == Axis.x:
                self.xg_bias_low = self._get(_XG_BIAS_LOW)
                self.xg_bias_high = self._get(_XG_BIAS_HIGH)
                self.xg_bias_32 = self._comb_16_into_32(
                    self.xg_bias_high, self.xg_bias_low)
                self.xg_bias_32 = self._check(self.xg_bias_32, 32)
                return self.xg_bias_32 * gyro_scale / 65536
            elif axis == Axis.y:
                self.yg_bias_low = self._get(_YG_BIAS_LOW)
                self.yg_bias_high = self._get(_YG_BIAS_HIGH)
                self.yg_bias_32 = self._comb_16_into_32(
                    self.yg_bias_high, self.yg_bias_low)
                self.yg_bias_32 = self._check(self.yg_bias_32, 32)
                return self.yg_bias_32 * gyro_scale / 65536
            elif axis == Axis.z:
                self.zg_bias_low = self._get(_ZG_BIAS_LOW)
                self.zg_bias_high = self._get(_ZG_BIAS_HIGH)
                self.zg_bias_32 = self._comb_16_into_32(
                    self.zg_bias_high, self.zg_bias_low)
                self.zg_bias_32 = self._check(self.zg_bias_32, 32)
                return self.zg_bias_32 * gyro_scale / 65536
        elif sensor_type == SensorType.accl:
            if axis == Axis.x:
                self.xa_bias_low = self._get(_XA_BIAS_LOW)
                self.xa_bias_high = self._get(_XA_BIAS_HIGH)
                self.xa_bias_32 = self._comb_16_into_32(
                    self.xa_bias_high, self.xa_bias_low)
                self.xa_bias_32 = self._check(self.xa_bias_32, 32)
                return self.xa_bias_32 * accl_scale / 65536
            elif axis == Axis.y:
                self.ya_bias_low = self._get(_YA_BIAS_LOW)
                self.ya_bias_high = self._get(_YA_BIAS_HIGH)
                self.ya_bias_32 = self._comb_16_into_32(
                    self.ya_bias_high, self.ya_bias_low)
                self.ya_bias_32 = self._check(self.ya_bias_32, 32)
                return self.ya_bias_32 * accl_scale / 65536
            elif axis == Axis.z:
                self.za_bias_low = self._get(_ZA_BIAS_LOW)
                self.za_bias_high = self._get(_ZA_BIAS_HIGH)
                self.za_bias_32 = self._comb_16_into_32(
                    self.za_bias_high, self.za_bias_low)
                self.za_bias_32 = self._check(self.za_bias_32, 32)
                return self.za_bias_32 * accl_scale / 65536

    @property
    def config(self):
        """Getter for reading Miscellaneous Configuration values

        Returns:
            [str]: Output of values in binary code
        """
        self._select_page(0x03)
        self._config = self._get(_CONFIG)
        return f'{self._config:08b}'

    @config.setter
    def config(self, value):
        """Setter to Miscellaneous Configuration

        Args:
            value ([int]): Value Miscellaneous Configuration

        Raises:
            TypeError: The error occurs when the input value is not int
        """
        if not isinstance(value, int):
            raise TypeError("Сonfig type must be int")
        self._select_page(0x03)
        self._set(_CONFIG, value)
        self._set(_CONFIG + 1, 0x00)

    def autocalibration(self, tbc):
        """Auto bias estimation

        Args:
            tbc ([int]): [3:0] Time base control (TBC), range: 0 to 13 (default = 10);
        tB = 2TBC/4250, time base; tA = 64 × tB, average time
        """
        if not isinstance(tbc, int):
            raise TypeError("TBC must be type int")
        if not (0 <= tbc <= 13):
            raise ValueError("The value is out of the allowed range")
        self._select_page(0x03)
        self._set(_NULL_CNFG, tbc)
        self._set(_NULL_CNFG+1, 0b00000111)
        time.sleep(autocalib_pause[tbc])
        self._set(_GLOB_CMD, 0b00000001)
        self._set(_GLOB_CMD+1, 0x00)

    @property
    def reset(self):
        """Reset settings"""
        self._select_page(0x03)
        self._set(_GLOB_CMD, 0b01000000)
        self._set(_GLOB_CMD+1, 0x00)
        self._set(_DEC_RATE, 0x00)
        self._set(_DEC_RATE + 1, 0x00)
        self._set(_CONFIG, 0x00)
        self._set(_CONFIG + 1, 0x00)

    @property
    def burst_read(self):
        """The burst read function provides a method for reading a batch of data
        (status, temperature, gyroscopes, accelerometers, time stamp/data counter, and CRC code),
        which does not require a stall time between each 16-bit segment and only requires one
        command on the DIN line to initiate.

        Not supported by the sensor ADIS16490!
        """
        txData = [0x7C, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                  0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
        self._select_page(0x00)
        rxData = spi.xfer(txData)
        row = [self._sys_e_flag, self._temp_out, self._x_gyro, self._y_gyro,
               self._z_gyro, self._x_accl, self._y_accl, self._z_accl]
        return row

    @atexit.register
    def cleanup():
        """The function is automatically executed after the interpreter finishes
        """
        spi.close()
        GPIO.cleanup()


# sensor = Adis1649x(16490)
# x = sensor.accl_axes
# print(x)
