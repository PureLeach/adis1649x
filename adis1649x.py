from enum import Enum
import spidev
import RPi.GPIO as GPIO


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


IRQ = 6  # GPIO connect to data ready
CS = 8  # GPIO connect to chip select
spi = spidev.SpiDev()  # Создаем объект SPI
spi.open(0, 0)  # Выбор номера порта и номера устройства(CS) шины SPI
spi.max_speed_hz = 8000000  # Задаём максимальную скорость работы шины SPI
spi.mode = 3  # Выбор режима работы SPI (от 0 до 3)
GPIO.setmode(GPIO.BCM)  # Выбор режима нумерации выводов GPIO
GPIO.setup(IRQ, GPIO.IN)  # Data ready
GPIO.setup(CS, GPIO.OUT)  # Chip select
GPIO.output(CS, GPIO.HIGH)


def _spi_read(spi, reg):
    """ Функция чтения данных по шине SPI

    Args:
        spi ([object]): Созданный объект SpiDev
        reg ([int]): Номер регистра

    Returns:
       resp [int]: Выходное значение
    """

    send = [0]*2  # Создаём список из двух элементов
    # В 0 ячейку списка записываем адрес, который указываем в параметре reg
    send[0] = reg
    spi.writebytes(send)  # Отправляем байты по шине SPI
    # Считываем 2 байта по шине SPI. В итоге получаем список из двух значений [x1, x2]
    resp = spi.readbytes(2)
    # Сдвигаем 8 бит ячейки 0 влево, затем используем лог.сложение с ячейкой 1
    resp = ((resp[0] << 8) | resp[1])
    return resp


def _spi_write(spi, reg, value):
    """ Функция записывающая данные по шине SPI

    Args:
        spi ([object]): Созданный объект SpiDev
        reg ([int]): Номер регистра
        value ([int]): Значение для записи в регистр
    """
    send = [0]*2  # Создаём список из двух элементов
    # В 0 ячейку списка записываем адрес, который указываем в параметре reg и
    # с помощью лог.ИЛИ указываем старший бит на запись
    send[0] = 0x80 | reg
    send[1] = value  # В 1 ячейку отправляем данные которые нужно записать
    spi.writebytes(send)  # Отправляем байты по шине SPI


class SensorType(Enum):
    """ Класс перечесления типов сенсоров """
    gyro = 'gyro'
    accl = 'accl'


class Axis(Enum):
    """ Класс перечесления осей """
    x = 'x'
    y = 'y'
    z = 'z'


class Adis1649x:
    """Класс датчика ADIS 1649x в котором реализован интерфейс для взаимодействия с ним.

    Raises:
        AttributeError: Возникает, если прочтённое id не совпадает с id датчика
        TypeError: Введёна переменная несоответствующего типа
        ValueError: Метод получает аргумент правильного типа, но некорректного значения

    Returns:
        [type]: [description]
    """

    # добавить проверку на ошибки
    def __init__(self, prod_id=16490):
        """Check the ADIS was found, read the coefficients and enable the sensor"""
        # Check device ID.
        # Ждём уровень спадающего фронта (по документации)
        GPIO.wait_for_edge(IRQ, GPIO.FALLING)
        _spi_write(spi, _PAGE_ID, 0x00)  # Переключаемся на 1 страницу
        GPIO.wait_for_edge(IRQ, GPIO.FALLING)
        # Считываем ID датчика, если не совпадает, то вызываем ошибку
        adis_prod_id = _spi_read(spi, _PROD_ID)
        if adis_prod_id != prod_id:
            raise RuntimeError(
                f"Failed to find ADIS {prod_id}! Chip ID {adis_prod_id}")

    # Метод для чтения данных по номеру адреса регистра
    def _get(self, reg):
        """ Дожидаемя сигнала data ready и отправляем данные во внешнюю функцию чтения

        Args:
            reg ([int]): Номер регистра, который необходима прочитать

        Returns:
            [int]: Полученный ответ
        """

        GPIO.wait_for_edge(IRQ, GPIO.FALLING)
        self.value = _spi_read(spi, reg)
        return self.value

    # Метод для записи данных по номеру адреса регистра
    def _set(self, reg, value):
        """ Отправляем данные во внешнюю функцию записи"""
        # GPIO.wait_for_edge(IRQ, GPIO.FALLING)
        _spi_write(spi, reg, value)

    def _select_page(self, page):
        """ Метод для переключения страницы регистров

        Args:
            page ([int]): Номер страницы на которую необходимо переключиться
        """
        GPIO.wait_for_edge(IRQ, GPIO.FALLING)
        _spi_write(spi, _PAGE_ID, page)

    def _comb_16_into_32(self, high, low):
        """ Метод для объединения 16 битных чисел в 32 бита
        путём операций смещения старших битов влево и логической суммы

        Args:
            high ([int]): Старшие 16 бита
            low ([int]): Младшие 16 бита

        Returns:
            [int]: 32 битное число
        """
        bit32 = ((high << 16) | (low & 0xFFFF))
        return bit32

    # Метод для объединения 8 битных чисел в 32 бита
    def _comb_8_into_32(self, low_senior, low_junior, high_senior, high_junior):
        """ Метод для объединения 8 битных чисел в 32 бита
        путём операций смещения старших битов влево и логической суммы

        Args:
            low_senior ([int]): Старшие 8 бит из младшего 16 битного числа
            low_junior ([int]): Млашие 8 бит из младшего 16 битного числа
            high_senior ([int]): Старшие 8 бит из старшего 16 битного числа
            high_junior ([int]): Младшие 8 бит из старшего 16 битного числа

        Returns:
            [int]: 32 битное число
        """
        low = ((low_senior << 8) | low_junior)  # 16 бит
        out = ((high_senior << 8) | high_junior)  # 16 бит
        bit32 = ((out << 16) | (low & 0xFFFF))
        return bit32

    def _check(self, value, bits):
        """ Проверка значения на знак. Если последний бит числа равен единице,
        то переводим из дополнительного кода в прямой код и последний разряд изменяем на занк "-"

        Args:
            value ([int]): Значение, которое необходимо проверить
            bits ([int]): Количество бит в числе value

        Returns:
            [int]: Переведённое значение
        """
        if((value & (1 << (bits-1))) != 0):
            value = value - (1 << bits)
        return value

    @property
    def temp(self):
        """ Вывод значения температуры
        Формула для представления числа из datasheet:
        0.01429 * temp_raw + 25

        Returns:
            [float]: Преобразованное значение температуры
        """
        self._select_page(0x00)
        temp_raw = self._get(_TEMP_OUT)
        temp_raw = self._check(temp_raw, 16)
        return 0.01429 * temp_raw + 25

    @property
    def decrate(self):
        """ Геттер для чтения значения декрейта
        Декрейт определяет частоту выдаваемых датчиком значений.
        Формула: f = 4250 / (decrate + 1)

        Returns:
            [int]: Значение decrate
        """
        self._select_page(0x03)
        self.decrate_ = self._get(_DEC_RATE)
        return self.decrate_

    # Добавить проверку диапазона
    @decrate.setter
    def decrate(self, value):
        """ Сеттер для изменения параметра декрейта

        Args:
            value ([int]): Входное значение

        Raises:
            TypeError: [description] Ошибка возникает, когда вводимое значение не int
        """

        if not isinstance(value, int):
            raise TypeError("decrate должен быть тип int")
        self._decrate_low = value & 0xff
        self._decrate_high = (value >> 8) & 0xff
        self._select_page(0x03)
        self._set(_DEC_RATE, self._decrate_low)
        self._set(_DEC_RATE + 1, self._decrate_high)

    @property
    def x_gyro(self):
        """ Геттер для вывода значений гироскопа оси X

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
        return self.x_gyro_32 * 0.005 / 65536

    @property
    def x_accl(self):
        """ Геттер для вывода значений акселерометра оси X

        Returns:
            [float]: Преобразованое значение акселерометра
        """
        self._select_page(0x00)
        self.x_accl_low = self._get(_X_ACCL_LOW)
        self.x_accl_out = self._get(_X_ACCL_OUT)
        self.x_accl_32 = self._comb_16_into_32(
            self.x_accl_out, self.x_accl_low)
        self.x_accl_32 = self._check(
            self.x_accl_32, 32)
        return self.x_accl_32 * 0.5 / 65536

    @property
    def y_gyro(self):
        """ Геттер для вывода значений гироскопа оси Y

        Returns:
            [float]: Преобразованое значение гироскопа
        """
        self._select_page(0x00)
        self.y_gyro_low = self._get(_Y_GYRO_LOW)
        self.y_gyro_out = self._get(_Y_GYRO_OUT)
        self.y_gyro_32 = self._comb_16_into_32(
            self.y_gyro_out, self.y_gyro_low)
        self.y_gyro_32 = self._check(
            self.y_gyro_32, 32)
        return self.y_gyro_32 * 0.005 / 65536

    @property
    def y_accl(self):
        """ Геттер для вывода значений акселерометра оси Y

        Returns:
            [float]: Преобразованое значение акселерометра
        """
        self._select_page(0x00)
        self.y_accl_low = self._get(_Y_ACCL_LOW)
        self.y_accl_out = self._get(_Y_ACCL_OUT)
        self.y_accl_32 = self._comb_16_into_32(
            self.y_accl_out, self.y_accl_low)
        self.y_accl_32 = self._check(
            self.y_accl_32, 32)
        return self.y_accl_32 * 0.5 / 65536

    @property
    def z_gyro(self):
        """ Геттер для вывода значений гироскопа оси Z

        Returns:
            [float]: Преобразованое значение гироскопа
        """
        self._select_page(0x00)
        self.z_gyro_low = self._get(_Z_GYRO_LOW)
        self.z_gyro_out = self._get(_Z_GYRO_OUT)
        self.z_gyro_32 = self._comb_16_into_32(
            self.z_gyro_out, self.z_gyro_low)
        self.z_gyro_32 = self._check(
            self.z_gyro_32, 32)
        return self.z_gyro_32 * 0.005 / 65536

    @property
    def z_accl(self):
        """ Геттер для вывода значений акселерометра оси Z

        Returns:
            [float]: Преобразованое значение акселерометра
        """
        self._select_page(0x00)
        self.z_accl_low = self._get(_Z_ACCL_LOW)
        self.z_accl_out = self._get(_Z_ACCL_OUT)
        self.z_accl_32 = self._comb_16_into_32(
            self.z_accl_out, self.z_accl_low)
        self.z_accl_32 = self._check(
            self.z_accl_32, 32)
        return self.z_accl_32 * 0.5 / 65536

# не доделано!
    @property
    def gyro_axes(self):
        """ Одновременное чтение трёх осей гироскопа

        Returns:
            [list]: Список из значений гироскопа [Ось X, Ось Y, Ось Z]
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

        return self.z_accl_32 * 0.5 / 65536

    # не доделано
    # Чтение трёх осей акселерометра
    @property
    def accl_axes(self):
        pass
        return self.z_accl_32 * 0.5 / 65536

    def _scale(self, value):
        """ Преобразование введёного пользователем значения scale
        в два 8 битных числа

        Args:
            value ([int]): Введённое значение scale

        Returns:
            [int]: Два 8 битных значения
        """
        bits = 16
        # Масштабируем число в формат от 32768 до 65535
        if (-1 <= value < 0):
            value = ((value + 1) * 32767) + 32768
        # Масштабируем число в формат от 0 до 32767
        elif (0 <= value <= 1):
            value = (value * 32767)
        # Округляем и переводим в integer
        value = int(round(value))
        # Если число отрицательное, то переводим в дополниельный код
        if value < 0:
            value = value + (1 << bits)
        # Далее, необходимо полученное число разделить на 2 части по 8 бит каждое
        self._scale_low = value & 0xff
        self._scale_high = (value >> 8) & 0xff
        return self._scale_low, self._scale_high

    # Запись в память датчика значения Bias
    def _bias(self, value, coefficient, bits):
        """ Преобразование введёного 32 битного значение bias
        в четыре 8 битных числа

        Args:
            value ([int]): значение для преобразования
            coefficient ([int]): коэффицент для преобразования
            bits ([int]): количиство бит в числе

        Returns:
            [int]: четыре 8 битных значения
        """
        value = value / coefficient  # Делим на коэф
        # Округляем, так как если сразу перевести число в int часть значений пропадает
        value = int(round(value))  # Переводим из float в  integer
        if(value < 0):  # Если число отрицательное, то переводим в дополнительный код
            value = value + (1 << bits)
        # Далее, необходимо полученное 32 битное число разделить на 2 части по 16 бит каждое
        self._bias_low = value & 0xFFFF
        self._bias_high = (value >> 16) & 0xFFFF
        # 16 бит делим по 8 бит
        self._bias_low1 = self._bias_low & 0xff  # Младшие 8 бит
        self._bias_low2 = (self._bias_low >> 8) & 0xff  # Старшие 8 бит
        # 16 бит делим по 8 бит
        self._bias_high1 = self._bias_high & 0xff  # Младшие 8 бит
        self._bias_high2 = (self._bias_high >> 8) & 0xff  # Старшие 8 бит
        return self._bias_low1, self._bias_low2, self._bias_high1, self._bias_high2

    def scale_set(self, sensor_type, axis, value):
        """ Метод для записи значения scale введёное пользователем
        в память датчика

        Args:
            sensor_type ([SensorType]): тип датчика в регистры которого будет записано введёное значение
            axis ([Axis]): название оси в которую будет записано значение
            value ([int]): значение scale

        Raises:
            TypeError: Ошибка возникает, если пользователь ввёл неправильно занчение
            типа сенсора или название оси
            ValueError: Ошибка возникает, если введёное значение выходит за рамки допустимого из datasheet

        Returns:
            [str]: Пользователю возвращаются введёные им значения
        """
        if not (isinstance(sensor_type, SensorType) or isinstance(axis, Axis) or isinstance(value, (int, float))):
            raise TypeError("Введён не верный тип данных")
        if not (-1 <= value <= 1):
            raise ValueError("Вышли за пределы допустимого интервала")
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
        """ Метод для получения значения scale прочитаные из памяти датчика

        Args:
            sensor_type ([SensorType]): тип датчика из регистра которого будет прочитано значение scale
            axis ([Axis]): [Axis] название оси из регистра которого будет прочитано значение

        Raises:
            TypeError: Ошибка возникает, если пользователь ввёл неправильно занчение
            типа сенсора или название оси

        Returns:
            [float]: значение прочитаное из заданного регистра
        """
        if not (isinstance(sensor_type, SensorType) or isinstance(axis, Axis)):
            raise TypeError("Введён не верный тип данных")
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
        """ Метод для записи значения bias введёное пользователем
        в память датчика

        Args:
            sensor_type ([SensorType]): тип датчика в регистры которого будет записано введёное значение
            axis ([Axis]): название оси в которую будет записано значение
            value ([int]): значение bias

        Raises:
            TypeError: Ошибка возникает, если пользователь ввёл неправильно занчение
            типа сенсора или название оси

        Returns:
            [str]: Пользователю возвращаются введёные им значения
        """
        if not (isinstance(sensor_type, SensorType) or isinstance(axis, Axis) or isinstance(value, (int, float))):
            raise TypeError("Введён не верный тип данных")
        self._select_page(0x02)
        self._scale_value = value
        if sensor_type == SensorType.gyro:
            self._bias_low1, self._bias_low2, self._bias_high1, self._bias_high2 = self._bias(
                value, 0.005/65536, 32)
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
                value, 0.5/65536, 32)
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
        """ Метод для получения значения bias прочитаные из памяти датчика

        Args:
            sensor_type ([SensorType]): тип датчика из регистра которого будет прочитано значение scale
            axis ([Axis]): [Axis] название оси из регистра которого будет прочитано значение

        Raises:
            TypeError: Ошибка возникает, если пользователь ввёл неправильно занчение
            типа сенсора или название оси

        Returns:
            [float]: значение прочитаное из заданного регистра
        """
        if not (isinstance(sensor_type, SensorType) or isinstance(axis, Axis)):
            raise TypeError("Введён не верный тип данных")
        self._select_page(0x02)
        if sensor_type == SensorType.gyro:
            if axis == Axis.x:
                self.xg_bias_low = self._get(_XG_BIAS_LOW)
                self.xg_bias_high = self._get(_XG_BIAS_HIGH)
                self.xg_bias_32 = self._comb_16_into_32(
                    self.xg_bias_high, self.xg_bias_low)
                self.xg_bias_32 = self._check(self.xg_bias_32, 32)
                return self.xg_bias_32 * 0.005 / 65536
            elif axis == Axis.y:
                self.yg_bias_low = self._get(_YG_BIAS_LOW)
                self.yg_bias_high = self._get(_YG_BIAS_HIGH)
                self.yg_bias_32 = self._comb_16_into_32(
                    self.yg_bias_high, self.yg_bias_low)
                self.yg_bias_32 = self._check(self.yg_bias_32, 32)
                return self.yg_bias_32 * 0.005 / 65536
            elif axis == Axis.z:
                self.zg_bias_low = self._get(_ZG_BIAS_LOW)
                self.zg_bias_high = self._get(_ZG_BIAS_HIGH)
                self.zg_bias_32 = self._comb_16_into_32(
                    self.zg_bias_high, self.zg_bias_low)
                self.zg_bias_32 = self._check(self.zg_bias_32, 32)
                return self.zg_bias_32 * 0.005 / 65536
        elif sensor_type == SensorType.accl:
            if axis == Axis.x:
                self.xa_bias_low = self._get(_XA_BIAS_LOW)
                self.xa_bias_high = self._get(_XA_BIAS_HIGH)
                self.xa_bias_32 = self._comb_16_into_32(
                    self.xa_bias_high, self.xa_bias_low)
                self.xa_bias_32 = self._check(self.xa_bias_32, 32)
                return self.xa_bias_32 * 0.5 / 65536
            elif axis == Axis.y:
                self.ya_bias_low = self._get(_YA_BIAS_LOW)
                self.ya_bias_high = self._get(_YA_BIAS_HIGH)
                self.ya_bias_32 = self._comb_16_into_32(
                    self.ya_bias_high, self.ya_bias_low)
                self.ya_bias_32 = self._check(self.ya_bias_32, 32)
                return self.ya_bias_32 * 0.5 / 65536
            elif axis == Axis.z:
                self.za_bias_low = self._get(_ZA_BIAS_LOW)
                self.za_bias_high = self._get(_ZA_BIAS_HIGH)
                self.za_bias_32 = self._comb_16_into_32(
                    self.za_bias_high, self.za_bias_low)
                self.za_bias_32 = self._check(self.za_bias_32, 32)
                return self.za_bias_32 * 0.5 / 65536

    # Miscellaneous Configuration
    @property
    def config(self):
        """ Геттер для чтения значений Miscellaneous Configuration

        Returns:
            [str]: Вывод значений в двоичном коде
        """
        self._select_page(0x03)
        self._config = self._get(_CONFIG)
        return f'{self._config:08b}'

    @config.setter
    def config(self, value):
        """ Сеттер для настройки Miscellaneous Configuration

        Args:
            value ([int]): Значение Miscellaneous Configuration

        Raises:
            TypeError: Ошибка возникает, когда вводимое значение не int
        """
        if not isinstance(value, int):
            raise TypeError("Тип config должен быть int")
        self._select_page(0x03)
        self._set(_CONFIG, value)
        self._set(_CONFIG + 1, 0x00)

    @property
    def reset(self):
        """  Cброс всех параметров """
        self._select_page(0x02)
        self._set(_X_GYRO_SCALE, 0x00)
        self._set(_X_GYRO_SCALE + 1, 0x00)
        self._set(_X_ACCL_SCALE, 0x00)
        self._set(_X_ACCL_SCALE + 1, 0x00)
        self._set(_Y_GYRO_SCALE, 0x00)
        self._set(_Y_GYRO_SCALE + 1, 0x00)
        self._set(_Y_ACCL_SCALE, 0x00)
        self._set(_Y_ACCL_SCALE + 1, 0x00)
        self._set(_Z_GYRO_SCALE, 0x00)
        self._set(_Z_GYRO_SCALE + 1, 0x00)
        self._set(_Z_ACCL_SCALE, 0x00)
        self._set(_Z_ACCL_SCALE + 1, 0x00)
        self._set(_XG_BIAS_LOW, 0x00)
        self._set(_XG_BIAS_LOW + 1, 0x00)
        self._set(_XG_BIAS_HIGH, 0x00)
        self._set(_XG_BIAS_HIGH + 1, 0x00)
        self._set(_YG_BIAS_LOW, 0x00)
        self._set(_YG_BIAS_LOW + 1, 0x00)
        self._set(_YG_BIAS_HIGH, 0x00)
        self._set(_YG_BIAS_HIGH + 1, 0x00)
        self._set(_ZG_BIAS_LOW, 0x00)
        self._set(_ZG_BIAS_LOW + 1, 0x00)
        self._set(_ZG_BIAS_HIGH, 0x00)
        self._set(_ZG_BIAS_HIGH + 1, 0x00)
        self._set(_XA_BIAS_LOW, 0x00)
        self._set(_XA_BIAS_LOW + 1, 0x00)
        self._set(_XA_BIAS_HIGH, 0x00)
        self._set(_XA_BIAS_HIGH + 1, 0x00)
        self._set(_YA_BIAS_LOW, 0x00)
        self._set(_YA_BIAS_LOW + 1, 0x00)
        self._set(_YA_BIAS_HIGH, 0x00)
        self._set(_YA_BIAS_HIGH + 1, 0x00)
        self._set(_ZA_BIAS_LOW, 0x00)
        self._set(_ZA_BIAS_LOW + 1, 0x00)
        self._set(_ZA_BIAS_HIGH, 0x00)
        self._set(_ZA_BIAS_HIGH + 1, 0x00)
        self._select_page(0x03)
        self._set(_DEC_RATE, 0x00)
        self._set(_DEC_RATE + 1, 0x00)
        self._set(_CONFIG, 0x00)
        self._set(_CONFIG + 1, 0x00)

    # Считывание всех параметров
    @property
    def burst_read(self):
        self._select_page(0x00)
        self._data_cnt = self._get(_DATA_CNT)
        self._sys_e_flag = hex(self._get(_SYS_E_FLAG))
        self._diag_sts = self._get(_DIAG_STS)
        self._temp = self.temp
        self._x_gyro = self.x_gyro
        self._y_gyro = self.y_gyro
        self._z_gyro = self.z_gyro
        self._x_accl = self.x_accl
        self._y_accl = self.y_accl
        self._z_accl = self.z_accl
        self._time_stamp = self._get(_TIME_STAMP)
        self._prod_id = self._get(_PROD_ID)

        self._select_page(0x02)
        self._x_gyro_scale = self.scale_get(SensorType.gyro, Axis.x)
        self._y_gyro_scale = self.scale_get(SensorType.gyro, Axis.y)
        self._z_gyro_scale = self.scale_get(SensorType.gyro, Axis.z)
        self._x_accl_scale = self.scale_get(SensorType.accl, Axis.x)
        self._y_accl_scale = self.scale_get(SensorType.accl, Axis.y)
        self._z_accl_scale = self.scale_get(SensorType.accl, Axis.z)
        self._x_gyro_bias = self.bias_get(SensorType.gyro, Axis.x)
        self._y_gyro_bias = self.bias_get(SensorType.gyro, Axis.y)
        self._z_gyro_bias = self.bias_get(SensorType.gyro, Axis.z)
        self._x_accl_bias = self.bias_get(SensorType.accl, Axis.x)
        self._y_accl_bias = self.bias_get(SensorType.accl, Axis.y)
        self._z_accl_bias = self.bias_get(SensorType.accl, Axis.z)

        self._select_page(0x03)
        self._glob_cmd = hex(self._get(_GLOB_CMD))
        self._fnctio_ctrl = hex(self._get(_FNCTIO_CTRL))
        self._gpio_ctrl = hex(self._get(_GPIO_CTRL))
        self._config = f'{self._get(_CONFIG)}'
        self._decrate = self._get(_DEC_RATE)
        self._null_cnfg = self._get(_NULL_CNFG)

        self._select_page(0x04)
        self._serial_num = self._get(_SERIAL_NUM)

        self._select_page(0x05)
        self._fir_coef_A000 = hex(self._get(0x08))
        self._fir_coef_A001 = hex(self._get(0x0A))
        self._fir_coef_A002 = hex(self._get(0x0C))
        self._fir_coef_A059 = hex(self._get(0x7E))

        self._select_page(0x06)
        self._fir_coef_A060 = hex(self._get(0x08))
        self._fir_coef_A061 = hex(self._get(0x0A))
        self._fir_coef_A062 = hex(self._get(0x0C))
        self._fir_coef_A119 = hex(self._get(0x7E))

        self._select_page(0x07)
        self._fir_coef_B000 = hex(self._get(0x08))
        self._fir_coef_B001 = hex(self._get(0x0A))
        self._fir_coef_B002 = hex(self._get(0x0C))
        self._fir_coef_B059 = hex(self._get(0x7E))

        self._select_page(0x08)
        self._fir_coef_B060 = hex(self._get(0x08))
        self._fir_coef_B061 = hex(self._get(0x0A))
        self._fir_coef_B062 = hex(self._get(0x0C))
        self._fir_coef_B119 = hex(self._get(0x7E))

        self._select_page(0x09)
        self._fir_coef_C000 = hex(self._get(0x08))
        self._fir_coef_C001 = hex(self._get(0x0A))
        self._fir_coef_C002 = hex(self._get(0x0C))
        self._fir_coef_C059 = hex(self._get(0x7E))

        self._select_page(0x0A)
        self._fir_coef_C060 = hex(self._get(0x08))
        self._fir_coef_C061 = hex(self._get(0x0A))
        self._fir_coef_C062 = hex(self._get(0x0C))
        self._fir_coef_C119 = hex(self._get(0x7E))

        self._select_page(0x0B)
        self._fir_coef_D000 = hex(self._get(0x08))
        self._fir_coef_D001 = hex(self._get(0x0A))
        self._fir_coef_D002 = hex(self._get(0x0C))
        self._fir_coef_D059 = hex(self._get(0x7E))

        self._select_page(0x0C)
        self._fir_coef_D060 = hex(self._get(0x08))
        self._fir_coef_D061 = hex(self._get(0x0A))
        self._fir_coef_D062 = hex(self._get(0x0C))
        self._fir_coef_D119 = hex(self._get(0x7E))

        dic = [[self._prod_id, self._serial_num, self._decrate, self._config, self._x_gyro_bias,
                self._y_gyro_bias, self._z_gyro_bias, self._x_gyro_scale, self._y_gyro_scale, self._z_gyro_scale],
               [self._fir_coef_A000, self._fir_coef_A001, self._fir_coef_A002, self._fir_coef_A059,
                self._fir_coef_A060, self._fir_coef_A061, self._fir_coef_A062, self._fir_coef_A119],
               [self._fir_coef_B000, self._fir_coef_B001, self._fir_coef_B002, self._fir_coef_B059,
                self._fir_coef_B060, self._fir_coef_B061, self._fir_coef_B062, self._fir_coef_B119],
               [self._fir_coef_C000, self._fir_coef_C001, self._fir_coef_C002, self._fir_coef_C059,
                self._fir_coef_C060, self._fir_coef_C061, self._fir_coef_C062, self._fir_coef_C119],
               [self._fir_coef_D000, self._fir_coef_D001, self._fir_coef_D002, self._fir_coef_D059,
                self._fir_coef_D060, self._fir_coef_D061, self._fir_coef_D062, self._fir_coef_D119]]
        return dic
