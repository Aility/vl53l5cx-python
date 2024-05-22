
import time
import sysconfig
import pathlib
from smbus2 import SMBus, i2c_msg
from ctypes import CDLL, CFUNCTYPE, POINTER, Structure, byref, c_int, c_int8, c_uint8, c_int16, c_uint16, c_uint32


__version__ = '0.0.3'

DEFAULT_I2C_ADDRESS = 0x29

NB_TARGET_PER_ZONE = 1

RESOLUTION_4X4 = 16  # For completeness, feels nicer just to use 4*4
RESOLUTION_8X8 = 64

TARGET_ORDER_CLOSEST = 1
TARGET_ORDER_STRONGEST = 2

RANGING_MODE_CONTINUOUS = 1
RANGING_MODE_AUTONOMOUS = 3

POWER_MODE_SLEEP = 0
POWER_MODE_WAKEUP = 1

STATUS_OK = 0
STATUS_TIMEOUT = 1
STATUS_MCU_ERROR = 66
STATUS_INVALID_PARAM = 127
STATUS_ERROR = 255

STATUS_RANGE_NOT_UPDATED = 0
STATUS_RANGE_LOW_SIGNAL = 1
STATUS_RANGE_TARGET_PHASE = 2
STATUS_RANGE_SIGMA_HIGH = 3
STATUS_RANGE_TARGET_FAILED = 4
STATUS_RANGE_VALID = 5
STATUS_RANGE_NOWRAP = 6
STATUS_RANGE_RATE_FAILED = 7
STATUS_RANGE_SIGNAL_RATE_LOW = 8
STATUS_RANGE_VALID_LARGE_PULSE = 9
STATUS_RANGE_VALID_NO_TARGET = 10
STATUS_RANGE_MEASUREMENT_FAILED = 11
STATUS_RANGE_TARGET_BLURRED = 12
STATUS_RANGE_TARGET_INCONSISTENT = 13
STATUS_RANGE_NO_TARGET = 255


XTALK_BUFFER_SIZE = 776

_I2C_CHUNK_SIZE = 2048

_I2C_RD_FUNC = CFUNCTYPE(c_int, c_uint8, c_uint16, POINTER(c_uint8), c_uint32)
_I2C_WR_FUNC = CFUNCTYPE(c_int, c_uint8, c_uint16, POINTER(c_uint8), c_uint32)
_SLEEP_FUNC = CFUNCTYPE(c_int, c_uint32)

# Path to the library dir
_PATH = pathlib.Path(__file__).parent.parent.absolute()

# System OS/Arch dependent module name suffix
_SUFFIX = sysconfig.get_config_var('EXT_SUFFIX')

# Library name
_NAME = pathlib.Path("vl53l5cx_ctypes").with_suffix(_SUFFIX)

# Load the DLL
_VL53 = CDLL(_PATH / _NAME)


class VL53L5CX_MotionData(Structure):
    _fields_ = [
        ("global_indicator_1", c_uint32),
        ("global_indicator_2", c_uint32),
        ("status", c_uint8),
        ("nb_of_detected_aggregates", c_uint8),
        ("nb_of_aggregates", c_uint8),
        ("spare", c_uint8),
        ("motion", c_uint32 * 32)
    ]


class VL53L5CX_ResultsData(Structure):
    _fields_ = [
        ("silicon_temp_degc", c_int8),
        ("ambient_per_spad", c_uint32 * 64),
        ("nb_target_detected", c_uint8 * 64),
        ("nb_spads_enabled", c_uint32 * 64),
        ("signal_per_spad", c_uint32 * 64 * NB_TARGET_PER_ZONE),
        ("range_sigma_mm", c_uint16 * 64 * NB_TARGET_PER_ZONE),
        ("distance_mm", c_int16 * 64 * NB_TARGET_PER_ZONE),
        ("reflectance", c_uint8 * 64 * NB_TARGET_PER_ZONE),
        ("target_status", c_uint8 * 64 * NB_TARGET_PER_ZONE),
        ("motion_indicator", VL53L5CX_MotionData)
    ]


class VL53L5CX:
    def __init__(self, i2c_addr=DEFAULT_I2C_ADDRESS, i2c_dev=None, skip_init=False):
        """Initialise VL53L5CX.

        :param i2c_addr: Sensor i2c address. (defualt: 0x29)
        :param skip_init: Skip (slow) sensor init (if it has not been power cycled).

        """
        self._configuration = None
        self._motion_configuration = None

        def _i2c_read(address, reg, data_p, length):
            msg_w = i2c_msg.write(address, [reg >> 8, reg & 0xff])
            msg_r = i2c_msg.read(address, length)
            self._i2c.i2c_rdwr(msg_w, msg_r)

            for index in range(length):
                data_p[index] = ord(msg_r.buf[index])

            return 0

        def _i2c_write(address, reg, data_p, length):
            # Copy the ctypes pointer data into a Python list
            data = []
            for i in range(length):
                data.append(data_p[i])

            for offset in range(0, length, _I2C_CHUNK_SIZE):
                chunk = data[offset:offset + _I2C_CHUNK_SIZE]
                msg_w = i2c_msg.write(address, [(reg + offset) >> 8, (reg + offset) & 0xff] + chunk)
                self._i2c.i2c_rdwr(msg_w)

            return 0

        def _sleep(ms):
            time.sleep(ms / 1000.0)
            return 0

        self._i2c = i2c_dev or SMBus(1)
        self._i2c_rd_func = _I2C_RD_FUNC(_i2c_read)
        self._i2c_wr_func = _I2C_WR_FUNC(_i2c_write)
        self._sleep_func = _SLEEP_FUNC(_sleep)
        self._configuration = _VL53.get_configuration(i2c_addr << 1, self._i2c_rd_func, self._i2c_wr_func, self._sleep_func)

        if not self.is_alive():
            raise RuntimeError(f"VL53L5CX not detected on 0x{i2c_addr:02x}")

        if not skip_init:
            if not self.init():
                raise RuntimeError("VL53L5CX init failed!")

    def init(self):
        """Initialise VL53L5CX."""
        return _VL53.vl53l5cx_init(self._configuration) == STATUS_OK

    def __del__(self):
        if self._configuration:
            _VL53.cleanup_configuration(self._configuration)
        if self._motion_configuration:
            _VL53.cleanup_motion_configuration(self._motion_configuration)

    def enable_motion_indicator(self, resolution=64):
        """Enable motion indicator.

        Switch on motion data output.

        :param resolution: Either 4*4 or 8*8 (default: 8*8)

        """
        if self._motion_configuration is None:
            self._motion_configuration = _VL53.get_motion_configuration()
        return _VL53.vl53l5cx_motion_indicator_init(self._configuration, self._motion_configuration, resolution) == 0

    def set_motion_distance(self, distance_min, distance_max):
        """Set motion indicator detection distance.

        :param distance_min: Minimum distance (mm), must be >= 400
        :param distance_max: Maximum distance (mm), distance_max - distance_min must be < 1500

        """
        if self._motion_configuration is None:
            raise RuntimeError("Enable motion first.")
        if distance_min < 400:
            raise ValueError("distance_min must be >= 400mm")
        if distance_max - distance_min > 1500:
            raise ValueError("distance between distance_min and distance_max must be < 1500mm")
        return _VL53.vl53l5cx_motion_indicator_set_distance_motion(self._configuration, self._motion_configuration, distance_min, distance_max)

    def set_motion_resolution(self, resolution):
        """Set motion resolition

        :param resolution: resolution one of RESOLUTION_4X4, RESOLUTION_8x8
        """
        if resolution not in [RESOLUTION_4X4, RESOLUTION_8X8]:
            raise RuntimeError(f"Resolution should be one of [{RESOLUTION_4X4}, {RESOLUTION_8X8}]")
        return _VL53.vl53l5cx_motion_indicator_set_resolution(self._configuration, self._motion_configuration, resolution)

    def calibrate_xtalk(self, reflectance_percent, nb_samples, distance_mm):
        """This function starts the VL53L5CX sensor in order to calibrate Xtalk.
        This calibration is recommended is user wants to use a coverglass.
        
        :param reflectance_percent : Target reflectance in percent. This
        value is include integers between 1 and 99%. For a better efficiency, ST recommends a
        3% target reflectance.
        :param  nb_samples : Number of samples used for calibration. A higher
        number of samples means a higher accuracy, but it increases the calibration
        time. Minimum is 1 and maximum is 16.
        :param distance_mm : Target distance in mm. The minimum allowed
        distance is 600mm, and maximum is 3000mm in integer. The target must stay in Full FOV,
        so short distance are easier for calibration.
        """
        if not (0 < reflectance_percent and 100 > reflectance_percent and type(reflectance_percent) is int):
            raise ValueError("reflectance_percent should be integer between 1 and 99")
        if not (0 < nb_samples and nb_samples <= 16 and type(nb_samples) is int):
            raise ValueError("nb_sambples should be integer between 1 and 16")
        if not (600 <= distance_mm and distance_mm <=  3000):
            raise ValueError("distance_mm should be a number between 600 and 3000")
        return _VL53.vl53l5cx_calibrate_xtalk(self._configuration, reflectance_percent, nb_samples, distance_mm)

    def get_caldata_xtalk(self, p_xtalk_data):
        """This function gets the Xtalk buffer. The buffer is available after
        using the function vl53l5cx_calibrate_xtalk().
        :return: p_xtalk_data in case everything succeeds else -1
        """
        p_xtalk_data = c_uint8 * XTALK_BUFFER_SIZE
        status = _VL53.vl53l5cx_get_caldata_xtalk(self._configuration, p_xtalk_data)
        if status == STATUS_OK:
            return p_xtalk_data[:]
        else:
            return -1

    def set_caldata_xtalk(self, p_xtalk_data):
        """This function sets the Xtalk buffer. This function can be used to
        override default Xtalk buffer.
        :param p_xtalk_data : Buffer with a size defined by macro VL53L5CX_XTALK_SIZE.
        """
        return _VL53.vl53l5cx_set_caldata_xtalk(self._configuration, p_xtalk_data)
    
    def get_xtalk_margin(self ):
        """This function gets the Xtalk margin. This margin is used to increase
        the Xtalk threshold. It can also be used to avoid false positives after the Xtalk calibration.
        The default value is 50 kcps/spads.
        :return: xtalk_margin in case everything succeeds else -1
        """
        p_xtalk_margin = c_uint32 * 1
        status = _VL53.vl53l5cx_get_xtalk_margin(self._configuration, p_xtalk_margin)
        if status == STATUS_OK:
            return p_xtalk_margin[0]
        else:
            return -1

    def set_xtalk_margin(self, xtalk_margin=50):
        """This function sets the Xtalk margin. This margin is used to increase
        the Xtalk threshold. It can also be used to avoid false positives after the
        Xtalk calibration. The default value is 50 kcps/spads.
        :param xtalk_margin : New Xtalk margin in kcps/spads. Min value is 0 kcps/spads, and max is 10.000 kcps/spads
        """
        if not(xtalk_margin >= 0 and xtalk_margin <= 10000 and type(xtalk_margin) is int):
            raise ValueError("xtalk_margin should be between 0 kcps/spads and 10.000 kcps/spads")
        return _VL53.vl53l5cx_set_xtalk_margin(self._configuration, xtalk_margin)
    
    def is_alive(self):
        """Check sensor is connected.

        Attempts to read and validate device and revision IDs from the sensor.

        """
        is_alive = c_int(0)
        status = _VL53.vl53l5cx_is_alive(self._configuration, byref(is_alive))
        return status == STATUS_OK and is_alive.value == 1

    def start_ranging(self):
        """Start ranging."""
        _VL53.vl53l5cx_start_ranging(self._configuration)

    def stop_ranging(self):
        """Stop ranging."""
        _VL53.vl53l5cx_stop_ranging(self._configuration)

    def set_i2c_address(self, i2c_address):
        """Change the i2c address."""
        return _VL53.vl53l5cx_set_i2c_address(self._configuration, i2c_address << 1) == STATUS_OK

    def set_ranging_mode(self, ranging_mode):
        """Set ranging mode.

        :param ranging_mode: Either Continuous (RANGING_MODE_CONTINUOUS) or Autonomous (RANGING_MODE_AUTONOMOUS).

        """
        _VL53.vl53l5cx_set_ranging_mode(self._configuration, ranging_mode)

    def set_ranging_frequency_hz(self, ranging_frequency_hz):
        """Set ranging frequency.

        Set the frequency of ranging data output in continuous mode.

        :param ranging_frequency_hz: Frequency in hz from 1-60Hz at 4*4 and 1-15Hz at 8*8.

        """
        _VL53.vl53l5cx_set_ranging_frequency_hz(self._configuration, ranging_frequency_hz)

    def set_resolution(self, resolution):
        """Set sensor resolution.

        Set the sensor resolution for ranging.

        :param resolution: Either 4*4 or 8*8. The lower resolution supports a faster output data rate,

        """
        _VL53.vl53l5cx_set_resolution(self._configuration, resolution)

    def set_integration_time_ms(self, integration_time_ms):
        """Set sensor integration time.

        :param integration_time_ms: From 2ms to 1000ms. Must be lower than the ranging period.

        """
        _VL53.vl53l5cx_set_integration_time_ms(self._configuration, integration_time_ms)

    def set_sharpener_percent(self, sharpener_percent):
        """Set sharpener intensity.

        Sharpen the rolloff on the edges of closer targets to prevent them occluding more distant targets.

        :param sharpener_percent: From 0 (off) to 99 (full) (hardware default: 5%)

        """
        _VL53.vl53l5cx_set_sharpener_percent(self._configuration, sharpener_percent)

    def set_target_order(self, target_order):
        """Set target order.

        Strongest prefers targets with a higher return signal (reflectance) versus Closest preferring targets that are closer.

        :param target_order: Either Strongest (default, TARGET_ORDER_STRONGEST) or Closest (TARGET_ORDER_CLOSEST)

        """
        _VL53.vl53l5cx_set_target_order(self._configuration, target_order)

    def set_power_mode(self, power_mode):
        """Set power mode.

        :param power_mode: One of Sleep (POWER_MODE_SLEEP) or Wakeup (POWER_MODE_WAKEUP)

        """
        _VL53.vl53l5cx_set_power_mode(self._configuration, power_mode)

    def data_ready(self):
        """Check if data is ready."""
        ready = c_int(0)
        status = _VL53.vl53l5cx_check_data_ready(self._configuration, byref(ready))
        return ready.value and status == STATUS_OK

    def get_data(self):
        """Get data."""
        results = VL53L5CX_ResultsData()
        status = _VL53.vl53l5cx_get_ranging_data(self._configuration, byref(results))
        if status != STATUS_OK:
            raise RuntimeError("Error reading data.")
        return results
