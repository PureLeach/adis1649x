from enum import Enum
import time
import spidev
import RPi.GPIO as GPIO


import random

# ADIS 16490 USER self.regISTER MEMORY MAP

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


IRQ = 6  # GPIO connect to data redy
spi = spidev.SpiDev()  # Создаем объект SPI
spi.open(0, 0)  # Выбор номера порта и номера устройства(CS) шины SPI
spi.max_speed_hz = 15000000  # Задаём максимальную скорость работы шины SPI
spi.mode = 3  # Выбор режима работы SPI (от 0 до 3)
GPIO.setmode(GPIO.BCM)  # Выбор режима нумерации выводов GPIO
GPIO.setup(IRQ, GPIO.IN)  # Инициализация GPIO6 на ввод

# Функция читающая данные по шине SPI


def _spi_read(spi, reg):  # Функция считывания данных по SPI
    send = [0]*2  # Создаём список из двух элементов
    # В 0 ячейку списка записываем адрес, который указываем в параметре reg
    send[0] = reg
    spi.writebytes(send)  # Отправляем байты по шине SPI
    # Считываем 2 байта по шине SPI. В итоге получаем список из двух значений [X, Y]
    resp = spi.readbytes(2)
    # Сдвигаем 8 бит ячейки 0 влево, затем используем лог.сложение с ячейкой 1
    resp = ((resp[0] << 8) | resp[1])
    return resp


# Функция записывающая данные по шине SPI
def _spi_write(spi, reg, value):  # Функция записи данных по SPI
    send = [0]*2  # Создаём список из двух элементов
    # В 0 ячейку списка записываем адрес, который указываем в параметре reg и с помощью лог.ИЛИ указываем старший бит на запись
    send[0] = 0x80 | reg
    send[1] = value  # В 1 ячейку отправляем данные которые нужно записать
    spi.writebytes(send)  # Отправляем байты по шине SPI


class SensorType(Enum):
    gyro = 'gyro'
    accl = 'accl'

class Axis(Enum):
    x = 'x'
    y = 'y'
    z = 'z'

# Основной класс датчика
class ADIS_16490:
# добавить проверку на ошибки
    def __init__(self):
        """Check the ADIS was found, read the coefficients and enable the sensor"""
        # Check device ID.
        # Ждём уровень спадающего фронта (по документации)
        GPIO.wait_for_edge(IRQ, GPIO.FALLING)
        _spi_write(spi, _PAGE_ID, 0x00)  # Переключаемся на 1 страницу
        GPIO.wait_for_edge(IRQ, GPIO.FALLING)
        # Считываем ID датчика, если не совпадает, то вызываем ошибку
        ADIS_PROD_ID = _spi_read(spi, _PROD_ID)
        if ADIS_PROD_ID != 16490:
            raise RuntimeError(
                f"Failed to find ADIS 16490! Chip ID {ADIS_PROD_ID}")

    # Метод для чтения данных по номеру адреса регистра
    def _get(self, reg):
        GPIO.wait_for_edge(IRQ, GPIO.FALLING)
        self.value = _spi_read(spi, reg)  # Считываем значение
        return self.value

    # Метод для записи данных по номеру адреса регистра
    def _set(self, reg, value):
        GPIO.wait_for_edge(IRQ, GPIO.FALLING)
        _spi_write(spi, reg, value)  # Записываем данные в регистр

    # Метод для изменения номера страниц датчика
    def _select_page(self, page):
        GPIO.wait_for_edge(IRQ, GPIO.FALLING)
        _spi_write(spi, _PAGE_ID, page)  # Переключаемся на страницу

    # Метод для объединения 16 битных чисел в 32 бита
    def _unity(self, high, low):
        bit32 = ((high << 16) | (low & 0xFFFF))
        return bit32

    # Метод проверки значения на знак
    def _check(self, value, bits):
        # Если отрицательное, то переводим в дополнительный код
        if((value & (1 << (bits-1))) != 0):
            value = value - (1 << bits)
        return value

    # Вывод температуры
    @property
    def temp(self):
        self._select_page(0x00)
        temp_raw = self._get(_TEMP_OUT)
        temp_raw = self._check(temp_raw, 16)  # Проверим число на знак
        return 0.01429 * temp_raw + 25  # Применяем формулу расчёта температуры из datasheet

    # Метод для чтения значения декрейта
    @property
    def decrate(self):
        self._select_page(0x03)
        self.decrate_ = self._get(_DEC_RATE)
        return self.decrate_

    # Метод для изменения параметра декрейта
    # Декрейт определяет частоту выдаваемых значений. f = 4250 / (decrate + 1)
    @decrate.setter
    def decrate(self, value):
        if not isinstance(value, int):
            raise Exception("Тип decrate должен быть int")
        # Разбиваем значение по 8 бит
        self._decrate_low = value & 0xff
        self._decrate_high = (value >> 8) & 0xff
        self._select_page(0x03)
        self._set(_DEC_RATE, self._decrate_low)
        # Номер регистра decrate_high на 1 больше
        self._set(_DEC_RATE + 1, self._decrate_high)

    # Метод для вывода значений гироскопа оси X
    @property
    def x_gyro(self):
        self._select_page(0x00)
        self.x_gyro_low = self._get(_X_GYRO_LOW)
        self.x_gyro_out = self._get(_X_GYRO_OUT)
        self.x_gyro_32 = self._unity(self.x_gyro_out, self.x_gyro_low)
        self.x_gyro_32 = self._check(
            self.x_gyro_32, 32)  # Проверим число на знак
        return self.x_gyro_32 * 0.005 / 65536

    # Метод для вывода значений акселерометра оси Y
    @property
    def x_accl(self):
        self._select_page(0x00)
        self.x_accl_low = self._get(_X_ACCL_LOW)
        self.x_accl_out = self._get(_X_ACCL_OUT)
        self.x_accl_32 = self._unity(self.x_accl_out, self.x_accl_low)
        self.x_accl_32 = self._check(
            self.x_accl_32, 32)  # Проверим число на знак
        return self.x_accl_32 * 0.5 / 65536

    # Метод для вывода значений гироскопа оси Y
    @property
    def y_gyro(self):
        self._select_page(0x00)
        self.y_gyro_low = self._get(_Y_GYRO_LOW)
        self.y_gyro_out = self._get(_Y_GYRO_OUT)
        self.y_gyro_32 = self._unity(self.y_gyro_out, self.y_gyro_low)
        self.y_gyro_32 = self._check(
            self.y_gyro_32, 32)  # Проверим число на знак
        return self.y_gyro_32 * 0.005 / 65536

    # Метод для вывода значений акселерометра оси Y
    @property
    def y_accl(self):
        self._select_page(0x00)
        self.y_accl_low = self._get(_Y_ACCL_LOW)
        self.y_accl_out = self._get(_Y_ACCL_OUT)
        self.y_accl_32 = self._unity(self.y_accl_out, self.y_accl_low)
        self.y_accl_32 = self._check(
            self.y_accl_32, 32)  # Проверим число на знак
        return self.y_accl_32 * 0.5 / 65536

    # Метод для вывода значений гироскопа оси Z
    @property
    def z_gyro(self):
        self._select_page(0x00)
        self.z_gyro_low = self._get(_Z_GYRO_LOW)
        self.z_gyro_out = self._get(_Z_GYRO_OUT)
        self.z_gyro_32 = self._unity(self.z_gyro_out, self.z_gyro_low)
        self.z_gyro_32 = self._check(
            self.z_gyro_32, 32)  # Проверим число на знак
        return self.z_gyro_32 * 0.005 / 65536

    # Метод для вывода значений акселерометра оси Z
    @property
    def z_accl(self):
        self._select_page(0x00)
        self.z_accl_low = self._get(_Z_ACCL_LOW)
        self.z_accl_out = self._get(_Z_ACCL_OUT)
        self.z_accl_32 = self._unity(self.z_accl_out, self.z_accl_low)
        self.z_accl_32 = self._check(
            self.z_accl_32, 32)  # Проверим число на знак
        return self.z_accl_32 * 0.5 / 65536

    # Запись в память датчика полученные значения Scale
    def _scale(self, value):
        coefficient = 1
        bits = 16
        # Масштабируем число в формат от 32768 до 65535
        if (-1 <= value < 0):  
            value = ((value + 1) * 32767) + 32768
        # Масштабируем число в формат от 0 до 32767
        elif (0 <= value <= 1):  
            value = (value * 32767)
        # Округляем и переводим в integer
        value = int(round(value))
        # Если число отрицательное, то переводим в дополнительный код
        if value < 0:  
            value = value + (1 << bits)
        # Далее, необходимо полученное число разделить на 2 части по 8 бит каждое
        self._scale_low = value & 0xff
        self._scale_high = (value >> 8) & 0xff
        return self._scale_low, self._scale_high
    
    def scale_set(self, sensor_type, axis, value):
        if not (isinstance(sensor_type, SensorType) or isinstance(axis, Axis) or isinstance(value, (int, float))):
            raise Exception("Введён не верный тип данных")
        if not (-1 <= value <= 1):
            raise Exception("Вышли за пределы допустимого интервала")
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
        if not (isinstance(sensor_type, SensorType) or isinstance(axis, Axis)):
            raise Exception("Введён не верный тип данных")
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


  


# x = SensorValue._value
# print(x)
sensor = ADIS_16490()  # Создание экземпляра класса
sensor.decrate = 3
# x = sensor.decrate
# print(x)
# n = 0

sensor.scale_set(SensorType.gyro, Axis.x, -0.5)
sensor.scale_set(SensorType.gyro, Axis.y, -0.58)



# print(sensor.z_accl)
# # Вывод параметров в консоль
while True:
    y = sensor.scale_get(SensorType.gyro, Axis.x)
    print(y)
    z = sensor.scale_get(SensorType.gyro, Axis.y)
    print(z)
#     n += 100
#     print(sensor.temp)
#     print(sensor.decrate)
#     print(sensor.x_accl)
#     print(sensor.y_accl)
#     print(sensor.z_accl)
#     print(sensor.x_gyro)
#     print(sensor.y_gyro)
#     print(sensor.z_gyro)
#     sensor.decrate = n
    time.sleep(1)
