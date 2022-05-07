import time

from typing import List
from .api import *
from .buffers import Buffers


DEBUG_IO = False
DEBUG_LOW_LEVEL_LOGIC = False
DEBUG_LOW_LEVEL_LOGIC_START_RANGING = False
DEBUG_LOW_LEVEL_LOGIC_SEND_OFFSET_DATA = False
DEBUG_LOW_LEVEL_LOGIC_GET_RANGING_DATA = False
DEFENSIVE_CODE = False
PRINT_SIZE_MAX = 1024

VL53L5CX_COMMS_CHUNK_SIZE = 4096  # Mark's original value was 1024, but 4096 works as well


def to_long_uint(data: List[int], i: int) -> int:
    return data[i] + data[i + 1] * 0x100 + data[i + 2] * 0x10000 + data[i + 3] * 0x1000000


def ulong_to_buffer(v: int, buf: List[int], pos: int = 0) -> None:
    for i in range(4):
        buf[i + pos] = v % 256
        v = v >> 8


def to_short_int(data: List[int], i: int) -> int:
    d = data[i] + data[i + 1] * 256
    d = (d - 65536) if d > 32767 else d
    return d


def short_to_buffer(v: int, buf: List[int], pos: int = 0) -> None:
    v = (v + 65536) if v < 0 else v
    buf[pos] = v & 0xff
    buf[pos + 1] = (v >> 8) & 0xff


def long_array_to_bytes(long_array: List[int]) -> List[int]:
    long_array_len = len(long_array)
    res = [0] * long_array_len * 4
    for i in range(long_array_len):
        v = long_array[i]
        k = i * 4
        res[k] = v & 0xff
        res[k + 1] = (v >> 8) & 0xff
        res[k + 2] = (v >> 16) & 0xff
        res[k + 3] = (v >> 24) & 0xff

    return res


def short_array_to_bytes(short_array: List[int]) -> List[int]:
    short_array_len = len(short_array)
    res = [0] * short_array_len * 2
    for i in range(short_array_len):
        v = short_array[i]
        k = i * 2
        res[k] = v & 0xff
        res[k + 1] = (v >> 8) & 0xff

    return res


def to_ulong_array(destination: List[int], source: List[int], source_index: int, source_size: int) -> None:
    for i in range(source_size // 4):
        k = source_index + i * 4
        destination[i] = source[k] | source[k + 1] << 8 | source[k + 2] << 16 | source[k + 3] << 24


def to_uint_array(destination: List[int], source: List[int], source_index: int, source_size: int) -> None:
    for i in range(source_size // 2):
        k = source_index + i * 2
        destination[i] = source[k] | source[k + 1] << 8


def to_int_array(destination: List[int], source: List[int], source_index: int, source_size: int) -> None:
    for i in range(source_size // 2):
        k = source_index + i * 2
        d = source[k] | source[k + 1] << 8
        d = (d - 65536) if d >= 32768 else d
        destination[i] = d


class VL53L5CXException(Exception):
    def __init__(self, status: int) -> None:
        super().__init__(status)


class VL53L5CXResultsData:
    def __init__(self, nb_target_per_zone: int) -> None:
        # Internal sensor silicon temperature */
        self.silicon_temp_degc: int = 0

        # Ambient noise in kcps/spads - originally # ifndef VL53L5CX_DISABLE_AMBIENT_PER_SPAD
        self.ambient_per_spad = [0] * VL53L5CX_RESOLUTION_8X8

        # Number of valid target detected for 1 zone - originally # ifndef VL53L5CX_DISABLE_NB_TARGET_DETECTED
        self.nb_target_detected = [0] * VL53L5CX_RESOLUTION_8X8

        # Number of spads enabled for this ranging - originally # ifndef VL53L5CX_DISABLE_NB_SPADS_ENABLED
        self.nb_spads_enabled = [0] * VL53L5CX_RESOLUTION_8X8

        # Signal returned to the sensor in kcps/spads - originally # ifndef VL53L5CX_DISABLE_SIGNAL_PER_SPAD
        self.signal_per_spad = [0] * (VL53L5CX_RESOLUTION_8X8 * nb_target_per_zone)

        # Sigma of the current distance in mm - originally # ifndef VL53L5CX_DISABLE_RANGE_SIGMA_MM
        self.range_sigma_mm = [0] * (VL53L5CX_RESOLUTION_8X8 * nb_target_per_zone)

        # Measured distance in mm - originally # ifndef VL53L5CX_DISABLE_DISTANCE_MM
        self.distance_mm = [0] * (VL53L5CX_RESOLUTION_8X8 * nb_target_per_zone)

        # Estimated reflectance in percent - originally # ifndef VL53L5CX_DISABLE_REFLECTANCE_PERCENT
        self.reflectance = [0] * (VL53L5CX_RESOLUTION_8X8 * nb_target_per_zone)

        # Status indicating the measurement validity (5 & 9 means ranging OK) - originally # ifndef VL53L5CX_DISABLE_TARGET_STATUS
        self.target_status = [0] * (VL53L5CX_RESOLUTION_8X8 * nb_target_per_zone)

        # Motion detector results - originally # ifndef VL53L5CX_DISABLE_MOTION_INDICATOR
        # This was originally motion_indicator structure {
        self.global_indicator_1: int = 0
        self.global_indicator_2: int = 0
        self.status: int = 0
        self.nb_of_detected_aggregates: int = 0
        self.nb_of_aggregates: int = 0
        self.spare: int = 0
        self.motion = [0] * 32
        # } motion_indicator

    def update_motion_indicator(self, data: List[int], ptr: int, size: int) -> None:
        if size >= 4:
            self.global_indicator_1 = to_long_uint(data, ptr)
            size -= 4
            ptr += 4
        if size >= 4:
            self.global_indicator_2 = to_long_uint(data, ptr)
            size -= 4
            ptr += 4
        if size >= 1:
            self.status = data[ptr]
            size -= 1
            ptr += 1
        if size >= 1:
            self.nb_of_detected_aggregates = data[ptr]
            size -= 1
            ptr += 1
        if size >= 1:
            self.nb_of_aggregates = data[ptr]
            size -= 1
            ptr += 1
        if size >= 1:
            self.spare = data[ptr]
            size -= 1
            ptr += 1
        if size >= 0:
            i = 0
            while size >= 4:
                self.motion[i] = to_long_uint(data, ptr)
                i += 1
                ptr += 4
                size -= 4


class VL53L5CX:
    def __init__(
            self,
            i2c_bus=None,
            bus_id: int = 1,
            use_raw_format: bool = False,
            nb_target_per_zone: int = 1,
            disable_ambient_per_spad: bool = False,
            disable_nb_spads_enabled: bool = False,
            disable_nb_target_detected: bool = False,
            disable_signal_per_spad: bool = False,
            disable_range_sigma_mm: bool = False,
            disable_distance_mm: bool = False,
            disable_reflectance_percent: bool = False,
            disable_target_status: bool = False,
            disable_motion_indicator: bool = False) -> None:

        self.buffers = Buffers(nb_target_per_zone)
        self.use_raw_format = use_raw_format
        self.nb_target_per_zone = nb_target_per_zone

        self.disable_ambient_per_spad = disable_ambient_per_spad
        self.disable_nb_spads_enabled = disable_nb_spads_enabled
        self.disable_nb_target_detected = disable_nb_target_detected
        self.disable_signal_per_spad = disable_signal_per_spad
        self.disable_range_sigma_mm = disable_range_sigma_mm
        self.disable_distance_mm = disable_distance_mm
        self.disable_reflectance_percent = disable_reflectance_percent
        self.disable_target_status = disable_target_status
        self.disable_motion_indicator = disable_motion_indicator

        if disable_ambient_per_spad:
            self.L5CX_AMB_SIZE = 0
        else:
            self.L5CX_AMB_SIZE = 260

        if disable_nb_spads_enabled:
            self.L5CX_SPAD_SIZE = 0
        else:
            self.L5CX_SPAD_SIZE = 260

        if disable_nb_target_detected:
            self.L5CX_NTAR_SIZE = 0
        else:
            self.L5CX_NTAR_SIZE = 68

        if disable_signal_per_spad:
            self.L5CX_SPS_SIZE = 0
        else:
            self.L5CX_SPS_SIZE = ((256 * nb_target_per_zone) + 4)

        if disable_range_sigma_mm is None:
            self.L5CX_SIGR_SIZE = ((128 * nb_target_per_zone) + 4)
        else:
            self.L5CX_SIGR_SIZE = 0

        if disable_distance_mm:
            self.L5CX_DIST_SIZE = 0
        else:
            self.L5CX_DIST_SIZE = ((128 * nb_target_per_zone) + 4)

        if disable_reflectance_percent:
            self.L5CX_RFLEST_SIZE = 0
        else:
            self.L5CX_RFLEST_SIZE = ((64 * nb_target_per_zone) + 4)

        if disable_target_status:
            self.L5CX_STA_SIZE = 0
        else:
            self.L5CX_STA_SIZE = ((64 * nb_target_per_zone) + 4)

        if disable_motion_indicator:
            self.L5CX_MOT_SIZE = 0
        else:
            self.L5CX_MOT_SIZE = 144

        self.VL53L5CX_MAX_RESULTS_SIZE = (40
                                          + self.L5CX_AMB_SIZE + self.L5CX_SPAD_SIZE + self.L5CX_NTAR_SIZE + self.L5CX_SPS_SIZE
                                          + self.L5CX_SIGR_SIZE + self.L5CX_DIST_SIZE + self.L5CX_RFLEST_SIZE + self.L5CX_STA_SIZE
                                          + self.L5CX_MOT_SIZE + 8)

        if self.VL53L5CX_MAX_RESULTS_SIZE < 1024:
            self.VL53L5CX_TEMPORARY_BUFFER_SIZE = 1024
        else:
            self.VL53L5CX_TEMPORARY_BUFFER_SIZE = self.VL53L5CX_MAX_RESULTS_SIZE

        if nb_target_per_zone == 1:
            self.VL53L5CX_START_BH = 0x0000000D
            self.VL53L5CX_METADATA_BH = 0x54B400C0
            self.VL53L5CX_COMMONDATA_BH = 0x54C00040
            self.VL53L5CX_AMBIENT_RATE_BH = 0x54D00104
            self.VL53L5CX_SPAD_COUNT_BH = 0x55D00404
            self.VL53L5CX_NB_TARGET_DETECTED_BH = 0xCF7C0401
            self.VL53L5CX_SIGNAL_RATE_BH = 0xCFBC0404
            self.VL53L5CX_RANGE_SIGMA_MM_BH = 0xD2BC0402
            self.VL53L5CX_DISTANCE_BH = 0xD33C0402
            self.VL53L5CX_REFLECTANCE_BH = 0xD43C0401
            self.VL53L5CX_TARGET_STATUS_BH = 0xD47C0401
            self.VL53L5CX_MOTION_DETECT_BH = 0xCC5008C0

            self.VL53L5CX_METADATA_IDX = 0x54B4
            self.VL53L5CX_SPAD_COUNT_IDX = 0x55D0
            self.VL53L5CX_AMBIENT_RATE_IDX = 0x54D0
            self.VL53L5CX_NB_TARGET_DETECTED_IDX = 0xCF7C
            self.VL53L5CX_SIGNAL_RATE_IDX = 0xCFBC
            self.VL53L5CX_RANGE_SIGMA_MM_IDX = 0xD2BC
            self.VL53L5CX_DISTANCE_IDX = 0xD33C
            self.VL53L5CX_REFLECTANCE_EST_PC_IDX = 0xD43C
            self.VL53L5CX_TARGET_STATUS_IDX = 0xD47C
            self.VL53L5CX_MOTION_DETEC_IDX = 0xCC50

        else:
            self.VL53L5CX_START_BH = 0x0000000D
            self.VL53L5CX_METADATA_BH = 0x54B400C0
            self.VL53L5CX_COMMONDATA_BH = 0x54C00040
            self.VL53L5CX_AMBIENT_RATE_BH = 0x54D00104
            self.VL53L5CX_NB_TARGET_DETECTED_BH = 0x57D00401
            self.VL53L5CX_SPAD_COUNT_BH = 0x55D00404
            self.VL53L5CX_SIGNAL_RATE_BH = 0x58900404
            self.VL53L5CX_RANGE_SIGMA_MM_BH = 0x64900402
            self.VL53L5CX_DISTANCE_BH = 0x66900402
            self.VL53L5CX_REFLECTANCE_BH = 0x6A900401
            self.VL53L5CX_TARGET_STATUS_BH = 0x6B900401
            self.VL53L5CX_MOTION_DETECT_BH = 0xCC5008C0

            self.VL53L5CX_METADATA_IDX = 0x54B4
            self.VL53L5CX_SPAD_COUNT_IDX = 0x55D0
            self.VL53L5CX_AMBIENT_RATE_IDX = 0x54D0
            self.VL53L5CX_NB_TARGET_DETECTED_IDX = 0x57D0
            self.VL53L5CX_SIGNAL_RATE_IDX = 0x5890
            self.VL53L5CX_RANGE_SIGMA_MM_IDX = 0x6490
            self.VL53L5CX_DISTANCE_IDX = 0x6690
            self.VL53L5CX_REFLECTANCE_EST_PC_IDX = 0x6A90
            self.VL53L5CX_TARGET_STATUS_IDX = 0x6B90
            self.VL53L5CX_MOTION_DETEC_IDX = 0xCC50

        self.i2c_address = VL53L5CX_DEFAULT_I2C_ADDRESS

        if i2c_bus is None:
            from smbus2 import SMBus, i2c_msg
            self._i2c_bus = SMBus(bus_id)
            self.i2c_msg = i2c_msg
        else:
            self._i2c_bus = i2c_bus

        self.streamcount: int = 0
        self.data_read_size: int = 0
        self.default_configuration: int = 0
        self.default_xtalk: int = 0
        self.offset_data = [0] * VL53L5CX_OFFSET_BUFFER_SIZE
        self.xtalk_data = [0] * VL53L5CX_XTALK_BUFFER_SIZE
        self.temp_buffer = [0] * self.VL53L5CX_TEMPORARY_BUFFER_SIZE

    @staticmethod
    def swap_buffer(buffer: List[int], size: int) -> None:
        # Original code:
        # 	for(i = 0; i < size; i = i + 4)
        # 	{
        # 		tmp = (
        # 		  buffer[i]<<24)
        # 		|(buffer[i+1]<<16)
        # 		|(buffer[i+2]<<8)
        # 		|(buffer[i+3]);
        #
        # 		memcpy(&(buffer[i]), &tmp, 4);
        # 	}
        for i in range(0, min(size, len(buffer)), 4):
            t = buffer[i]
            buffer[i] = buffer[i + 3]
            buffer[i + 3] = t
            t = buffer[i + 1]
            buffer[i + 1] = buffer[i + 2]
            buffer[i + 2] = t

    def rd_multi(self, addr: int, buffer: List[int], size: int) -> None:
        write_addr = self.i2c_msg.write(self.i2c_address, [addr >> 8 & 0xff, addr & 0xff])
        read_data = self.i2c_msg.read(self.i2c_address, size)

        self._i2c_bus.i2c_rdwr(write_addr, read_data)

        read_size = len(read_data)

        if read_size > 0:
            read_buffer = list(read_data)
            buffer[:read_size] = read_buffer[:read_size]
            if DEBUG_IO:
                print(f"rd_multi addr={addr:#0{6}x}, len={{}}, size={size}. read_size={read_size}, buf_len= read=[", end="")
                print_size = read_size
                if print_size > PRINT_SIZE_MAX:
                    print_size = PRINT_SIZE_MAX
                for i in range(print_size):
                    if i > 0:
                        print(", ", end="")
                    print(f"{read_buffer[i]:#0{2}x}", end="")
                if print_size != read_size:
                    print(", ...", end="")
                print("]")
        else:
            raise Exception("Couldn't read any bytes")

    def wr_multi(self, addr: int, buffer: List[int], size: int) -> None:
        position = 0
        while position < size:
            data_size = VL53L5CX_COMMS_CHUNK_SIZE - 2 if size - position > VL53L5CX_COMMS_CHUNK_SIZE - 2 else size - position

            buf = [0] * (data_size + 2)
            buf[0] = addr >> 8
            buf[1] = addr & 0xff
            buf[2:] = buffer[position:position + data_size]
            write = self.i2c_msg.write(self.i2c_address, buf)
            self._i2c_bus.i2c_rdwr(write)

            if DEBUG_IO:
                print(f"wr_multi addr={addr:#0{6}x}, len={{}}, size={size}. write_size={data_size} [", end="")
                print_size = data_size
                if print_size > PRINT_SIZE_MAX:
                    print_size = PRINT_SIZE_MAX
                for i in range(print_size):
                    if i > 0:
                        print(", ", end="")
                    print(f"{buffer[position + i]:#0{2}x}", end="")
                if print_size != data_size:
                    print(", ...", end="")
                print("]")
            addr += data_size
            position += data_size

    def rd_byte(self, addr: int) -> int:
        write_addr = self.i2c_msg.write(self.i2c_address, [addr >> 8 & 0xff, addr & 0xff])
        read_data = self.i2c_msg.read(self.i2c_address, 1)
        self._i2c_bus.i2c_rdwr(write_addr, read_data)
        b = read_data.buf[0][0]
        if DEBUG_IO:
            print(f"rd_byte addr={addr:#0{6}x}, byte={b:#0{4}x}")
        return b

    def wr_byte(self, addr: int, value: int) -> None:
        write_addr_and_value = self.i2c_msg.write(self.i2c_address, [addr >> 8 & 0xff, addr & 0xff, value])
        self._i2c_bus.i2c_rdwr(write_addr_and_value)
        if DEBUG_IO:
            print(f"wr_byte addr={addr:#0{6}x}, byte={value:#0{4}x}")

    @staticmethod
    def wait_ms(ms: int) -> None:
        time.sleep(ms / 1000)

    def _poll_for_answer(
            self,
            size: int, pos: int, address: int, mask: int, expected_value: int) -> None:
        """
        Inner function, not available outside this file. This function is used
        to wait for an answer from VL53L5CX sensor.
        """

        timeout = 0

        first = True
        while first or (self.temp_buffer[pos] & mask) != expected_value:
            first = False
            self.rd_multi(address, self.temp_buffer, size)
            if DEBUG_LOW_LEVEL_LOGIC:
                print(f"Polling for answer {address:#0{6}x} size={size} pos={pos} mask={mask:#0{4}x} expected_value={expected_value:#0{4}x}\n     result: answer={self.temp_buffer[:size]} (len=)")
            self.wait_ms(10)

            if timeout >= 200:  # 2s timeout
                raise VL53L5CXException(self.temp_buffer[2])
            elif size >= 4 and self.temp_buffer[2] >= 0x7f:
                raise VL53L5CXException(VL53L5CX_MCU_ERROR)
            else:
                timeout += 1

    def _poll_for_mcu_boot(self) -> None:
        """
        Inner function, not available outside this file. This function is used to
        wait for the MCU to boot.
        """

        status = VL53L5CX_STATUS_OK
        timeout = 0

        while timeout < 500:
            go2_status0 = self.rd_byte(0x06)
            if go2_status0 & 0x80 != 0:
                go2_status1 = self.rd_byte(0x07)
                status |= go2_status1
                break

            self.wait_ms(1)
            timeout += 1

            if go2_status0 & 0x1 != 0:
                break

        if status != 0:
            raise VL53L5CXException(status)

    def _send_offset_data(self, resolution: int) -> None:
        """
        @brief Inner function, not available outside this file. This function is used
        to set the offset data gathered from NVM.
        """

        signal_grid: List[int] = [0] * 64 * 4
        range_grid: List[int] = [0] * 64 * 2
        dss_4x4 = [0x0F, 0x04, 0x04, 0x00, 0x08, 0x10, 0x10, 0x07]
        footer = [0x00, 0x00, 0x00, 0x0F, 0x03, 0x01, 0x01, 0xE4]

        self.temp_buffer[:VL53L5CX_OFFSET_BUFFER_SIZE] = self.offset_data[:VL53L5CX_OFFSET_BUFFER_SIZE]

        # Data extrapolation is required for 4X4 offset
        if resolution == VL53L5CX_RESOLUTION_4X4:
            self.temp_buffer[16:16 + len(dss_4x4)] = dss_4x4[:]

            self.swap_buffer(self.temp_buffer, VL53L5CX_OFFSET_BUFFER_SIZE)

            if DEBUG_LOW_LEVEL_LOGIC_SEND_OFFSET_DATA:
                print(f"_send_offset_data: (pre) signal_grid_len={len(signal_grid)}, range_grid_len={len(range_grid)}, temp_buffer_len={len(self.temp_buffer)}")

            signal_grid[:] = self.temp_buffer[0x3C:0x3C + len(signal_grid)]

            range_grid[:] = self.temp_buffer[0x140:0x140 + len(range_grid)]

            if DEBUG_LOW_LEVEL_LOGIC_SEND_OFFSET_DATA:
                print(f"_send_offset_data: signal_grid_len={len(signal_grid)}, range_grid_len={len(range_grid)}")
            if DEBUG_LOW_LEVEL_LOGIC_SEND_OFFSET_DATA:
                print(f"_send_offset_data: signal_grid=[", end="")
                for i in range(0, len(signal_grid), 4):
                    if i > 0:
                        print(", ", end="")
                    print(f"{signal_grid[i + 3]:0{2}x}{signal_grid[i + 2]:0{2}x}{signal_grid[i + 1]:0{2}x}{signal_grid[i + 0]:0{2}x}", end="")
                print("]")
            if DEBUG_LOW_LEVEL_LOGIC_SEND_OFFSET_DATA:
                print(f"_send_offset_data: range_grid=[", end="")
                for i in range(0, len(range_grid), 2):
                    if i > 0:
                        print(", ", end="")
                    print(f"{range_grid[i + 1]:0{2}x}{range_grid[i + 0]:0{2}x}", end="")
                print("]")

            for j in range(4):
                for i in range(4):
                    k = (i + 4 * j) * 4
                    l = ((2 * i) + (16 * j)) * 4
                    v = (to_long_uint(signal_grid, l + 0)
                        + to_long_uint(signal_grid, l + 1 * 4)
                        + to_long_uint(signal_grid, l + 8 * 4)
                        + to_long_uint(signal_grid, l + 9 * 4)) // 4
                    ulong_to_buffer(v, signal_grid, k)

                    k = (i + 4 * j) * 2
                    l = ((2 * i) + (16 * j)) * 2

                    v = (to_short_int(range_grid, l + 0)
                        + to_short_int(range_grid, l + 1 * 2)
                        + to_short_int(range_grid, l + 8 * 2)
                        + to_short_int(range_grid, l + 9 * 2))
                    if v < 0:
                        v = -(-v // 4)
                    else:
                        v = v // 4
                    short_to_buffer(v, range_grid, k)

            range_grid[0x10 * 2:0x10 * 2 + 96] = [0] * 96
            signal_grid[0x10 * 4:0x10 * 4 + 192] = [0] * 192
            if DEBUG_LOW_LEVEL_LOGIC_SEND_OFFSET_DATA:
                print(f"_send_offset_data: signal_grid=[", end="")
                for i in range(0, len(signal_grid), 4):
                    if i > 0:
                        print(", ", end="")
                    print(f"{signal_grid[i + 3]:0{2}x}{signal_grid[i + 2]:0{2}x}{signal_grid[i + 1]:0{2}x}{signal_grid[i + 0]:0{2}x}", end="")
                print("]")

            if DEBUG_LOW_LEVEL_LOGIC_SEND_OFFSET_DATA:
                print(f"_send_offset_data: range_grid=[", end="")
                for i in range(0, len(range_grid), 2):
                    if i > 0:
                        print(", ", end="")
                    print(f"{range_grid[i + 1]:0{2}x}{range_grid[i + 0]:0{2}x}", end="")
                print("]")

            self.temp_buffer[0x3C: 0x3C + len(signal_grid)] = signal_grid[:]
            self.temp_buffer[0x140: 0x140 + len(range_grid)] = range_grid[:]

            self.swap_buffer(self.temp_buffer, VL53L5CX_OFFSET_BUFFER_SIZE)

        for k in range(VL53L5CX_OFFSET_BUFFER_SIZE - 4):
            self.temp_buffer[k] = self.temp_buffer[k + 8]

        self.temp_buffer[0x1E0: 0x1E0 + 8] = footer[:]
        self.wr_multi(0x2e18, self.temp_buffer, VL53L5CX_OFFSET_BUFFER_SIZE)
        self._poll_for_answer(4, 1, VL53L5CX_UI_CMD_STATUS, 0xff, 0x03)

    def _send_xtalk_data(self, resolution: int) -> None:
        """
        @brief Inner function, not available outside this file. This function is used
        to set the Xtalk data from generic configuration, or user's calibration.
        """

        res4x4 = [0x0F, 0x04, 0x04, 0x17, 0x08, 0x10, 0x10, 0x07]
        dss_4x4 = [0x00, 0x78, 0x00, 0x08, 0x00, 0x00, 0x00, 0x08]
        profile_4x4 = [0xA0, 0xFC, 0x01, 0x00]
        signal_grid = [0] * 64 * 4

        self.temp_buffer[:VL53L5CX_XTALK_BUFFER_SIZE] = self.xtalk_data[0: VL53L5CX_XTALK_BUFFER_SIZE]

        # Data extrapolation is required for 4X4 Xtalk
        if resolution == VL53L5CX_RESOLUTION_4X4:
            self.temp_buffer[0x8: 0x8 + len(res4x4)] = res4x4[:]
            self.temp_buffer[0x020: 0x020 + len(dss_4x4)] = dss_4x4[:]

            self.swap_buffer(self.temp_buffer, VL53L5CX_XTALK_BUFFER_SIZE)
            signal_grid[:] = self.temp_buffer[0x34:0x34 + len(signal_grid)]

            for j in range(4):
                for i in range(4):
                    k = (i + 4 * j) * 4
                    l = ((2 * i) + (16 * j)) * 4
                    v = (to_long_uint(signal_grid, l + 0)
                        + to_long_uint(signal_grid, l + 1 * 4)
                        + to_long_uint(signal_grid, l + 8 * 4)
                        + to_long_uint(signal_grid, l + 9 * 4)) // 4
                    ulong_to_buffer(v, signal_grid, k)

            signal_grid[0x10 * 4:0x10 * 4 + 192] = [0] * 192
            self.temp_buffer[0x34: 0x34 + len(signal_grid)] = signal_grid[:]
            self.swap_buffer(self.temp_buffer, VL53L5CX_XTALK_BUFFER_SIZE)
            self.temp_buffer[0x134:0x134 + len(profile_4x4)] = profile_4x4[:]
            self.temp_buffer[0x078:0x078 + 4] = [0] * 4

        self.wr_multi(0x2cf8, self.temp_buffer, VL53L5CX_XTALK_BUFFER_SIZE)
        self._poll_for_answer(4, 1, VL53L5CX_UI_CMD_STATUS, 0xff, 0x03)

    def is_alive(self) -> bool:
        self.wr_byte(0x7fff, 0x00)
        device_id = self.rd_byte(0)
        revision_id = self.rd_byte(1)
        self.wr_byte(0x7fff, 0x02)

        return device_id == 0xF0 and revision_id == 0x02

    def init(self) -> None:
        pipe_ctrl = [self.nb_target_per_zone, 0x00, 0x01, 0x00]
        # single_range = [0, 0, 0, 0x01]
        single_range = [0x01, 0, 0, 0]

        self.default_xtalk = self.buffers.VL53L5CX_DEFAULT_XTALK
        self.default_configuration = self.buffers.VL53L5CX_DEFAULT_CONFIGURATION

        # reboot sequence
        self.wr_byte(0x7fff, 0x00)
        self.wr_byte(0x0009, 0x04)
        self.wr_byte(0x000F, 0x40)
        self.wr_byte(0x000A, 0x03)
        # tmp = self.rd_byte(0x7FFF)
        self.rd_byte(0x7FFF)
        self.wr_byte(0x000C, 0x01)

        self.wr_byte(0x0101, 0x00)
        self.wr_byte(0x0102, 0x00)
        self.wr_byte(0x010A, 0x01)
        self.wr_byte(0x4002, 0x01)
        self.wr_byte(0x4002, 0x00)
        self.wr_byte(0x010A, 0x03)
        self.wr_byte(0x0103, 0x01)
        self.wr_byte(0x000C, 0x00)
        self.wr_byte(0x000F, 0x43)
        self.wait_ms(1)

        self.wr_byte(0x000F, 0x40)
        self.wr_byte(0x000A, 0x01)
        self.wait_ms(100)

        # Wait for sensor booted (several ms required to get sensor ready )
        self.wr_byte(0x7fff, 0x00)
        self._poll_for_answer(1, 0, 0x06, 0xff, 1)

        self.wr_byte(0x000E, 0x01)
        self.wr_byte(0x7fff, 0x02)

        # Enable FW access
        self.wr_byte(0x03, 0x0D)
        self.wr_byte(0x7fff, 0x01)
        self._poll_for_answer(1, 0, 0x21, 0x10, 0x10)
        self.wr_byte(0x7fff, 0x00)

        # Enable host access to GO1
        # tmp = self.rd_byte(0x7fff)
        # self.rd_byte(0x7fff)
        self.wr_byte(0x0C, 0x01)

        # Power ON status
        self.wr_byte(0x7fff, 0x00)
        self.wr_byte(0x101, 0x00)
        self.wr_byte(0x102, 0x00)
        self.wr_byte(0x010A, 0x01)
        self.wr_byte(0x4002, 0x01)
        self.wr_byte(0x4002, 0x00)
        self.wr_byte(0x010A, 0x03)
        self.wr_byte(0x103, 0x01)
        self.wr_byte(0x400F, 0x00)
        self.wr_byte(0x21A, 0x43)
        self.wr_byte(0x21A, 0x03)
        self.wr_byte(0x21A, 0x01)
        self.wr_byte(0x21A, 0x00)
        self.wr_byte(0x219, 0x00)
        self.wr_byte(0x21B, 0x00)

        # Wake up MCU
        self.wr_byte(0x7fff, 0x00)
        # tmp = self.rd_byte(0x7fff)
        # self.rd_byte(0x7fff)
        self.wr_byte(0x0C, 0x00)
        self.wr_byte(0x7fff, 0x01)
        self.wr_byte(0x20, 0x07)
        self.wr_byte(0x20, 0x06)

        # Download FW into VL53L5
        self.wr_byte(0x7fff, 0x09)
        self.wr_multi(0, self.buffers.VL53L5CX_FIRMWARE[0:0x8000], 0x8000)
        self.wr_byte(0x7fff, 0x0a)
        self.wr_multi(0, self.buffers.VL53L5CX_FIRMWARE[0x8000:0x10000], 0x8000)
        self.wr_byte(0x7fff, 0x0b)
        self.wr_multi(0, self.buffers.VL53L5CX_FIRMWARE[0x10000:0x15000], 0x5000)
        self.wr_byte(0x7fff, 0x01)

        # Check if FW correctly downloaded
        self.wr_byte(0x7fff, 0x02)
        self.wr_byte(0x03, 0x0D)
        self.wr_byte(0x7fff, 0x01)
        self._poll_for_answer(1, 0, 0x21, 0x10, 0x10)
        # self.wr_byte(0x7fff, 0x00)
        # tmp = self.rd_byte(0x7fff)
        self.wr_byte(0x7fff, 0x00)
        self.wr_byte(0x0C, 0x01)

        # Reset MCU and wait boot
        self.wr_byte(0x7FFF, 0x00)
        self.wr_byte(0x114, 0x00)
        self.wr_byte(0x115, 0x00)
        self.wr_byte(0x116, 0x42)
        self.wr_byte(0x117, 0x00)
        self.wr_byte(0x0B, 0x00)
        # tmp = self.rd_byte(0x7fff)
        # self.rd_byte(0x7fff)
        self.wr_byte(0x0C, 0x00)
        self.wr_byte(0x0B, 0x01)
        self._poll_for_answer(1, 0, 0x06, 0xff, 0x00)
        # self._poll_for_mcu_boot()
        self.wr_byte(0x7fff, 0x02)

        # Get offset NVM data and store them into the offset buffer
        self.wr_multi(0x2fd8, self.buffers.VL53L5CX_GET_NVM_CMD, len(self.buffers.VL53L5CX_GET_NVM_CMD))
        self._poll_for_answer(4, 0, VL53L5CX_UI_CMD_STATUS, 0xff, 2)
        self.rd_multi(VL53L5CX_UI_CMD_START, self.temp_buffer, VL53L5CX_NVM_DATA_SIZE)
        self.offset_data[:VL53L5CX_OFFSET_BUFFER_SIZE] = self.temp_buffer[:VL53L5CX_OFFSET_BUFFER_SIZE]
        self._send_offset_data(VL53L5CX_RESOLUTION_4X4)

        # Set default Xtalk shape. Send Xtalk to sensor
        self.xtalk_data[:VL53L5CX_XTALK_BUFFER_SIZE] = self.buffers.VL53L5CX_DEFAULT_XTALK[:VL53L5CX_XTALK_BUFFER_SIZE]
        self._send_xtalk_data(VL53L5CX_RESOLUTION_4X4)

        # Send default configuration to VL53L5CX firmware
        self.wr_multi(0x2c34,
                      self.default_configuration,
                      len(self.buffers.VL53L5CX_DEFAULT_CONFIGURATION))
        self._poll_for_answer(4, 1, VL53L5CX_UI_CMD_STATUS, 0xff, 0x03)

        self.dci_write_data(pipe_ctrl, VL53L5CX_DCI_PIPE_CONTROL, len(pipe_ctrl))

        if self.nb_target_per_zone != 1:
            tmp = [self.nb_target_per_zone]
            self.dci_replace_data(self.temp_buffer, VL53L5CX_DCI_FW_NB_TARGET, 16, tmp, 1, 0x0C)

        self.dci_write_data(single_range, VL53L5CX_DCI_SINGLE_RANGE, len(single_range))

    def set_i2c_address(self, i2c_address: int) -> None:
        self.wr_byte(0x7fff, 0x00)
        self.wr_byte(0x4, i2c_address)
        self.i2c_address = i2c_address
        self.wr_byte(0x7fff, 0x02)

    def get_power_mode(self) -> int:

        self.wr_byte(0x7FFF, 0x00)
        tmp = self.rd_byte(0x009)

        if tmp == 0x4:
            p_power_mode = VL53L5CX_POWER_MODE_WAKEUP
        elif tmp == 0x2:
            p_power_mode = VL53L5CX_POWER_MODE_SLEEP
        else:
            raise VL53L5CXException(VL53L5CX_STATUS_ERROR)

        self.wr_byte(0x7FFF, 0x02)

        return p_power_mode

    def set_power_mode(self, power_mode: int) -> None:
        current_power_mode = self.get_power_mode()
        if power_mode != current_power_mode:
            if power_mode == VL53L5CX_POWER_MODE_WAKEUP:
                self.wr_byte(0x7FFF, 0x00)
                self.wr_byte(0x09, 0x04)
                self._poll_for_answer(1, 0, 0x06, 0x01, 1)
            elif power_mode == VL53L5CX_POWER_MODE_SLEEP:
                self.wr_byte(0x7FFF, 0x00)
                self.wr_byte(0x09, 0x02)
                self._poll_for_answer(1, 0, 0x06, 0x01, 0)
            else:
                raise VL53L5CXException(VL53L5CX_STATUS_ERROR)

            self.wr_byte(0x7FFF, 0x02)

    def start_ranging(self) -> None:
        header_config = [0, 0]

        # union Block_header *bh_ptr
        #    uint32_t bytes
        #    struct {
        #        uint32_t type : 4
        #        uint32_t size : 12
        #        uint32_t idx : 16
        #    }
        cmd = [0x00, 0x03, 0x00, 0x00]

        resolution = self.get_resolution()
        self.data_read_size = 0
        self.streamcount = 255

        # Enable mandatory output (meta and common data)
        output_bh_enable = [0x00000007, 0x00000000, 0x00000000, 0xC0000000]

        # Send addresses of possible output
        output = [self.VL53L5CX_START_BH,
                  self.VL53L5CX_METADATA_BH,
                  self.VL53L5CX_COMMONDATA_BH,
                  self.VL53L5CX_AMBIENT_RATE_BH,
                  self.VL53L5CX_SPAD_COUNT_BH,
                  self.VL53L5CX_NB_TARGET_DETECTED_BH,
                  self.VL53L5CX_SIGNAL_RATE_BH,
                  self.VL53L5CX_RANGE_SIGMA_MM_BH,
                  self.VL53L5CX_DISTANCE_BH,
                  self.VL53L5CX_REFLECTANCE_BH,
                  self.VL53L5CX_TARGET_STATUS_BH,
                  self.VL53L5CX_MOTION_DETECT_BH]

        # Enable selected outputs in the 'platform.h' file
        if not self.disable_ambient_per_spad:
            output_bh_enable[0] += 8

        if not self.disable_nb_spads_enabled:
            output_bh_enable[0] += 16

        if not self.disable_nb_target_detected:
            output_bh_enable[0] += 32
        if not self.disable_signal_per_spad:
            output_bh_enable[0] += 64
        if not self.disable_range_sigma_mm:
            output_bh_enable[0] += 128
        if not self.disable_distance_mm:
            output_bh_enable[0] += 256
        if not self.disable_reflectance_percent:
            output_bh_enable[0] += 512
        if not self.disable_target_status:
            output_bh_enable[0] += 1024
        if not self.disable_motion_indicator:
            output_bh_enable[0] += 2048

        DIVIDE_FACTOR = 32

        if DEBUG_LOW_LEVEL_LOGIC_START_RANGING:
            print(f"vl53l5cx_start_ranging:    output_bh_enable[0]={output_bh_enable[0]:0{8}x}")

        # Update data size
        total_output_len = len(output)
        for i in range(total_output_len):
            if (output[i] == 0) or ((output_bh_enable[i // DIVIDE_FACTOR] & (1 << (i % 32))) == 0):
                if DEBUG_LOW_LEVEL_LOGIC_START_RANGING:
                    print(f"vl53l5cx_start_ranging:    continue output[{i}]={output[i]:0{8}x}, output_bh_enable[{i // DIVIDE_FACTOR}]={output_bh_enable[i // DIVIDE_FACTOR]:0{8}x}")
                continue

            bh_ptr_type = output[i] & 0x0f
            bh_ptr_idx = (output[i] >> 16) & 0xffff
            if DEBUG_LOW_LEVEL_LOGIC_START_RANGING:
                print(f"vl53l5cx_start_ranging:    output[{i}]={output[i]:0{8}x}, bh_ptr_type={bh_ptr_type:0{4}x} bh_ptr_idx={bh_ptr_idx:0{8}x}")

            if 0x1 <= bh_ptr_type < 0x0d:
                if 0x54d0 <= bh_ptr_idx < (0x54d0 + 960):
                    bh_ptr_size = resolution
                else:
                    bh_ptr_size = resolution * self.nb_target_per_zone

                # bh_ptr_size back to output!
                output[i] = output[i] & 0xffff000f | (bh_ptr_size << 4) & 0xfff0

                self.data_read_size += bh_ptr_type * bh_ptr_size
                if DEBUG_LOW_LEVEL_LOGIC_START_RANGING:
                    print(f"vl53l5cx_start_ranging:    output[{i}]={output[i]:0{8}x}, data_read_size={self.data_read_size}")
            else:
                bh_ptr_size = (output[i] >> 4) & 0xfff
                self.data_read_size += bh_ptr_size
            self.data_read_size += 4
            if DEBUG_LOW_LEVEL_LOGIC_START_RANGING:
                print(f"vl53l5cx_start_ranging:  data_read_size={self.data_read_size}")

        self.data_read_size += 20
        if DEBUG_LOW_LEVEL_LOGIC_START_RANGING:
            print(f"vl53l5cx_start_ranging:  final data_read_size={self.data_read_size}")

        output_bytes = long_array_to_bytes(output)
        self.dci_write_data(output_bytes, VL53L5CX_DCI_OUTPUT_LIST, len(output_bytes))

        header_config[0] = self.data_read_size
        header_config[1] = total_output_len + 1

        if DEBUG_LOW_LEVEL_LOGIC_START_RANGING:
            print(f"vl53l5cx_start_ranging:  header_config={header_config[0]:0{8}x} {header_config[1]:0{8}x}")

        header_config_bytes = long_array_to_bytes(header_config)
        self.dci_write_data(header_config_bytes, VL53L5CX_DCI_OUTPUT_CONFIG, len(header_config_bytes))

        output_bh_enable_bytes = long_array_to_bytes(output_bh_enable)
        self.dci_write_data(output_bh_enable_bytes, VL53L5CX_DCI_OUTPUT_ENABLES, len(output_bh_enable_bytes))

        # Start xshut bypass (interrupt mode)
        self.wr_byte(0x7fff, 0x00)
        self.wr_byte(0x09, 0x05)
        self.wr_byte(0x7fff, 0x02)

        # Start ranging session
        self.wr_multi(VL53L5CX_UI_CMD_END - (4 - 1), cmd, len(cmd))
        self._poll_for_answer(4, 1, VL53L5CX_UI_CMD_STATUS, 0xff, 0x03)

        # Read ui range data content and compare if data size is the correct one
        # self.dci_read_data(self.temp_buffer, 0x5440, 12)
        # # memcpy(&tmp, &(p_dev->temp_buffer[0x8]), sizeof(tmp))
        #
        # size = self.temp_buffer[8] + self.temp_buffer[9] * 0x100
        #
        # if size != self.data_read_size:
        #     print(f"vl53l5cx_start_ranging: size={size} != data_read_size={self.data_read_size}, temp_buffer={self.temp_buffer[:32]}")
        #     # raise VL53L5CXException(VL53L5CX_STATUS_ERROR)

    def stop_ranging(self) -> None:
        timeout = 0
        buf = [0, 0, 0, 0]

        self.rd_multi(0x2FFC, buf, 4)
        auto_stop_flag = buf[0] + buf[1] * 0x100 + buf[2] * 0x10000 + buf[3] * 0x1000000

        if auto_stop_flag != 0x4FF:
            self.wr_byte(0x7fff, 0x00)

            # Provoke MCU stop
            self.wr_byte(0x15, 0x16)
            self.wr_byte(0x14, 0x01)

            # Poll for G02 status 0 MCU stop
            tmp = 0

            while ((tmp & 0x80) >> 7) == 0x00:
                tmp = self.rd_byte(0x6)
                self.wait_ms(10)
                timeout += 1  # Timeout reached after 5 seconds

                if timeout > 500 and tmp != 0:
                    raise VL53L5CXException(tmp)

        # Check GO2 status 1 if status is still OK
        tmp = self.rd_byte(0x6)
        if tmp & 0x80 != 0:
            tmp = self.rd_byte(0x7)
            if tmp != 0x84 and tmp != 0x85:
                raise VL53L5CXException(tmp)

        # Undo MCU stop
        self.wr_byte(0x7fff, 0x00)
        self.wr_byte(0x14, 0x00)
        self.wr_byte(0x15, 0x00)

        # Stop xshut bypass
        self.wr_byte(0x09, 0x04)
        self.wr_byte(0x7fff, 0x02)

    def check_data_ready(self) -> bool:
        self.rd_multi(0x0, self.temp_buffer, 4)

        if ((self.temp_buffer[0] != self.streamcount)
                and (self.temp_buffer[0] != 255)
                and (self.temp_buffer[1] == 0x5)
                and ((self.temp_buffer[2] & 0x5) == 0x5)
                and ((self.temp_buffer[3] & 0x10) == 0x10)):
            self.streamcount = self.temp_buffer[0]
            return True

        if self.temp_buffer[3] & 0x80 != 0:
            # Return GO2 error status
            raise VL53L5CXException(self.temp_buffer[2])

        if DEBUG_LOW_LEVEL_LOGIC:
            print(f"vl53l5cx_check_data_ready: buf={self.temp_buffer[:4]}, streamcount={self.streamcount}")
        return False

    def get_ranging_data(self) -> VL53L5CXResultsData:
        p_results = VL53L5CXResultsData(self.nb_target_per_zone)

        if DEBUG_LOW_LEVEL_LOGIC_GET_RANGING_DATA:
            print(f"vl53l5cx_get_ranging_data: data_read_size={self.data_read_size}")
        self.rd_multi(0x0, self.temp_buffer, self.data_read_size)
        self.streamcount = self.temp_buffer[0]
        self.swap_buffer(self.temp_buffer, self.data_read_size)
        if DEBUG_LOW_LEVEL_LOGIC_GET_RANGING_DATA:
            print(f"vl53l5cx_get_ranging_data: streamcount={self.streamcount}")

        # Start conversion at position 16 to avoid headers
        for i in range(16, self.data_read_size, 4):
            bh_ptr_type = self.temp_buffer[i] & 0x0f
            bh_ptr_size = (self.temp_buffer[i] >> 4) & 0xf | (self.temp_buffer[i + 1] << 4)
            if 0x1 < bh_ptr_type < 0xd:
                msize = bh_ptr_type * bh_ptr_size
            else:
                msize = bh_ptr_size

            bh_ptr_idx = self.temp_buffer[i + 2] + self.temp_buffer[i + 3] * 256

            if bh_ptr_idx == self.VL53L5CX_METADATA_IDX:
                p_results.silicon_temp_degc = self.temp_buffer[i + 12]
            elif not self.disable_ambient_per_spad and bh_ptr_idx == self.VL53L5CX_AMBIENT_RATE_IDX:
                to_ulong_array(p_results.ambient_per_spad, self.temp_buffer, i + 4, msize)
            elif not self.disable_nb_spads_enabled and bh_ptr_idx == self.VL53L5CX_SPAD_COUNT_IDX:
                to_ulong_array(p_results.nb_spads_enabled, self.temp_buffer, i + 4, msize)
            elif not self.disable_nb_target_detected and bh_ptr_idx == self.VL53L5CX_NB_TARGET_DETECTED_IDX:
                p_results.nb_target_detected[:msize] = self.temp_buffer[i + 4: i + 4 + msize]
            elif not self.disable_signal_per_spad and bh_ptr_idx == self.VL53L5CX_SIGNAL_RATE_IDX:
                to_ulong_array(p_results.signal_per_spad, self.temp_buffer, i + 4, msize)
            elif not self.disable_range_sigma_mm and bh_ptr_idx == self.VL53L5CX_RANGE_SIGMA_MM_IDX:
                to_uint_array(p_results.range_sigma_mm, self.temp_buffer, i + 4, msize)
            elif not self.disable_distance_mm and bh_ptr_idx == self.VL53L5CX_DISTANCE_IDX:
                to_uint_array(p_results.distance_mm, self.temp_buffer, i + 4, msize)
            elif not self.disable_reflectance_percent and bh_ptr_idx == self.VL53L5CX_REFLECTANCE_EST_PC_IDX:
                p_results.reflectance[:msize] = self.temp_buffer[i + 4: i + 4 + msize]
            elif not self.disable_target_status and bh_ptr_idx == self.VL53L5CX_TARGET_STATUS_IDX:
                p_results.target_status[:msize] = self.temp_buffer[i + 4: i + 4 + msize]
            elif not self.disable_motion_indicator and bh_ptr_idx == self.VL53L5CX_MOTION_DETEC_IDX:

                if DEBUG_LOW_LEVEL_LOGIC_GET_RANGING_DATA:
                    print(f"vl53l5cx_get_ranging_data: i+4={i + 4} msize={msize}, len(self.temp_buffer)={len(self.temp_buffer)}")
                p_results.update_motion_indicator(self.temp_buffer, i + 4, msize)
            i += msize

        if not self.use_raw_format:
            # Convert data into their real format */
            if not self.disable_ambient_per_spad:
                for i in range(VL53L5CX_RESOLUTION_8X8):
                    p_results.ambient_per_spad[i] /= 2048

            for i in range((VL53L5CX_RESOLUTION_8X8 * self.nb_target_per_zone)):
                if not self.disable_distance_mm:
                    p_results.distance_mm[i] /= 4
                    if p_results.distance_mm[i] < 0:
                        p_results.distance_mm[i] = 0
                if not self.disable_range_sigma_mm:
                    p_results.range_sigma_mm[i] /= 128
                if not self.disable_signal_per_spad:
                    p_results.signal_per_spad[i] /= 2048

            # TODO optimise inner IF - take it out!!!
            # Set target status to 255 if no target is detected for this zone
            if not self.disable_nb_target_detected:
                for i in range(VL53L5CX_RESOLUTION_8X8):
                    if p_results.nb_target_detected[i] == 0:
                        if not self.disable_target_status:
                            for j in range(self.nb_target_per_zone):
                                p_results.target_status[(self.nb_target_per_zone * i) + j] = 255

            if not self.disable_motion_indicator:
                for i in range(32):
                    p_results.motion[i] /= 65535

        return p_results

    def get_resolution(self) -> int:
        self.dci_read_data(self.temp_buffer, VL53L5CX_DCI_ZONE_CONFIG, 8)
        return self.temp_buffer[0x00] * self.temp_buffer[0x01]

    def set_resolution(self, resolution: int) -> None:
        if resolution == VL53L5CX_RESOLUTION_4X4:
            self.dci_read_data(self.temp_buffer, VL53L5CX_DCI_DSS_CONFIG, 16)
            self.temp_buffer[0x04] = 64
            self.temp_buffer[0x06] = 64
            self.temp_buffer[0x09] = 4
            self.dci_write_data(self.temp_buffer, VL53L5CX_DCI_DSS_CONFIG, 16)

            self.dci_read_data(self.temp_buffer, VL53L5CX_DCI_ZONE_CONFIG, 8)
            self.temp_buffer[0x00] = 4
            self.temp_buffer[0x01] = 4
            self.temp_buffer[0x04] = 8
            self.temp_buffer[0x05] = 8
            self.dci_write_data(self.temp_buffer, VL53L5CX_DCI_ZONE_CONFIG, 8)
        elif resolution == VL53L5CX_RESOLUTION_8X8:
            self.dci_read_data(self.temp_buffer, VL53L5CX_DCI_DSS_CONFIG, 16)
            self.temp_buffer[0x04] = 16
            self.temp_buffer[0x06] = 16
            self.temp_buffer[0x09] = 1
            self.dci_write_data(self.temp_buffer, VL53L5CX_DCI_DSS_CONFIG, 16)

            self.dci_read_data(self.temp_buffer, VL53L5CX_DCI_ZONE_CONFIG, 8)
            self.temp_buffer[0x00] = 8
            self.temp_buffer[0x01] = 8
            self.temp_buffer[0x04] = 4
            self.temp_buffer[0x05] = 4
            self.dci_write_data(self.temp_buffer, VL53L5CX_DCI_ZONE_CONFIG, 8)
        else:
            raise VL53L5CXException(VL53L5CX_STATUS_INVALID_PARAM)

        self._send_offset_data(resolution)
        self._send_xtalk_data(resolution)

    def get_ranging_frequency_hz(self) -> int:
        self.dci_read_data(self.temp_buffer, VL53L5CX_DCI_FREQ_HZ, 4)
        return self.temp_buffer[0x01]

    def set_ranging_frequency_hz(self, frequency_hz: int) -> None:
        frequency_hz_buf = [frequency_hz]
        self.dci_replace_data(self.temp_buffer, VL53L5CX_DCI_FREQ_HZ, 4, frequency_hz_buf, 1, 0x01)

    def get_integration_time_ms(self) -> int:
        self.dci_read_data(self.temp_buffer, VL53L5CX_DCI_INT_TIME, 20)
        return to_long_uint(self.temp_buffer, 0) // 1000

    def set_integration_time_ms(self, integration_time_ms: int) -> None:
        # Integration time must be between 2ms and 1000ms
        if integration_time_ms < 2 or integration_time_ms > 1000:
            raise VL53L5CXException(VL53L5CX_STATUS_INVALID_PARAM)

        integration = integration_time_ms * 1000
        buf = [0, 0, 0, 0]
        ulong_to_buffer(integration, buf)

        self.dci_replace_data(self.temp_buffer, VL53L5CX_DCI_INT_TIME, 20, buf, 4, 0x00)

    def get_sharpener_percent(self) -> float:
        self.dci_read_data(self.temp_buffer, VL53L5CX_DCI_SHARPENER, 16)

        return (self.temp_buffer[0xD] * 100) / 255

    def set_sharpener_percent(self, sharpener_percent: int) -> None:

        if sharpener_percent >= 100:
            raise VL53L5CXException(VL53L5CX_STATUS_INVALID_PARAM)

        sharpener = [(sharpener_percent * 255) // 100]
        self.dci_replace_data(self.temp_buffer, VL53L5CX_DCI_SHARPENER, 16, sharpener, 1, 0xD)

    def get_target_order(self) -> int:
        self.dci_read_data(self.temp_buffer, VL53L5CX_DCI_TARGET_ORDER, 4)
        return self.temp_buffer[0x0]

    def set_target_order(self, target_order: int) -> None:
        if target_order == VL53L5CX_TARGET_ORDER_CLOSEST or target_order == VL53L5CX_TARGET_ORDER_STRONGEST:
            target_order_buf = [target_order]
            self.dci_replace_data(self.temp_buffer, VL53L5CX_DCI_TARGET_ORDER, 4, target_order_buf, 1, 0x0)
        else:
            raise VL53L5CXException(VL53L5CX_STATUS_INVALID_PARAM)

    def get_ranging_mode(self) -> int:
        self.dci_read_data(self.temp_buffer, VL53L5CX_DCI_RANGING_MODE, 8)

        if self.temp_buffer[0x01] == 0x1:
            return VL53L5CX_RANGING_MODE_CONTINUOUS

        return VL53L5CX_RANGING_MODE_AUTONOMOUS

    def set_ranging_mode(self, ranging_mode: int) -> None:
        self.dci_read_data(self.temp_buffer, VL53L5CX_DCI_RANGING_MODE, 8)

        if ranging_mode == VL53L5CX_RANGING_MODE_CONTINUOUS:
            self.temp_buffer[0x01] = 0x1
            self.temp_buffer[0x03] = 0x3
            single_range = 0x00
        elif ranging_mode == VL53L5CX_RANGING_MODE_AUTONOMOUS:
            self.temp_buffer[0x01] = 0x3
            self.temp_buffer[0x03] = 0x2
            single_range = 0x01
        else:
            raise VL53L5CXException(VL53L5CX_STATUS_INVALID_PARAM)

        self.dci_write_data(self.temp_buffer, VL53L5CX_DCI_RANGING_MODE, 8)

        buf = [single_range, 0, 0, 0]
        self.dci_write_data(buf, VL53L5CX_DCI_SINGLE_RANGE, 4)

    def dci_read_data(self,
                               data: List[int],
                               index: int,
                               data_size: int) -> None:

        rd_size = data_size + 12
        cmd = [0x00, 0x00, 0x00, 0x00,
               0x00, 0x00, 0x00, 0x0f,
               0x00, 0x02, 0x00, 0x08]

        # Check if tmp buffer is large enough
        if (data_size + 12) > self.VL53L5CX_TEMPORARY_BUFFER_SIZE:
            raise VL53L5CXException(VL53L5CX_STATUS_ERROR)
        else:
            cmd[0] = (index >> 8) & 0xff
            cmd[1] = index & 0xff
            cmd[2] = ((data_size & 0xff0) >> 4) & 0xff
            cmd[3] = ((data_size & 0xf) << 4) & 0xff

            # Request data reading from FW
            self.wr_multi(VL53L5CX_UI_CMD_END - 11, cmd, len(cmd))
            self._poll_for_answer(4, 1, VL53L5CX_UI_CMD_STATUS, 0xff, 0x03)

            # Read new data sent (4 bytes header + data_size + 8 bytes footer)
            self.rd_multi(VL53L5CX_UI_CMD_START, self.temp_buffer, rd_size)
            self.swap_buffer(self.temp_buffer, data_size + 12)

            # Copy data from FW into input structure (-4 bytes to remove header)
            for i in range(data_size):
                data[i] = self.temp_buffer[i + 4]
            # data[:data_size] = self.temp_buffer[4: data_size + 4]

    def dci_write_data(self,
                                data: List[int],
                                index: int,
                                data_size: int) -> None:

        headers = [0x00, 0x00, 0x00, 0x00]
        footer = [0x00, 0x00, 0x00, 0x0f, 0x05, 0x01,
                  (data_size + 8) >> 8,
                  (data_size + 8) & 0xFF]

        address = VL53L5CX_UI_CMD_END - (data_size + 12) + 1

        # Check if cmd buffer is large enough */
        if data_size + 12 > self.VL53L5CX_TEMPORARY_BUFFER_SIZE:
            raise VL53L5CXException(VL53L5CX_STATUS_ERROR)
        else:
            headers[0] = index >> 8 & 0xff
            headers[1] = index & 0xff
            headers[2] = ((data_size & 0xff0) >> 4)
            headers[3] = (data_size & 0xf) << 4

            # Copy data from structure to FW format (+4 bytes to add header)
            self.swap_buffer(data, data_size)
            for i in range(data_size - 1, -1, -1):
                self.temp_buffer[i + 4] = data[i]

            # Add headers and footer
            self.temp_buffer[:len(headers)] = headers[:]
            self.temp_buffer[data_size + 4: data_size + 4 + len(footer)] = footer[:]

            # Send data to FW
            self.wr_multi(address, self.temp_buffer, data_size + 12)
            self._poll_for_answer(4, 1, VL53L5CX_UI_CMD_STATUS, 0xff, 0x03)

            self.swap_buffer(data, data_size)

    def dci_replace_data(self,
                                  data: List[int],
                                  index: int,
                                  data_size: int,
                                  new_data: List[int],
                                  new_data_size: int,
                                  new_data_pos: int) -> None:

        self.dci_read_data(data, index, data_size)
        data[new_data_pos: new_data_pos + data_size] = new_data[:new_data_size]
        self.dci_write_data(data, index, data_size)
