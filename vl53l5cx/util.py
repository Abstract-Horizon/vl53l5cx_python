import struct
from typing import List


def swap_buffer(buffer: List[int], size: int) -> None:
    for i in range(0, min(size, len(buffer)), 4):
        buffer[i:i+4] = reversed(buffer[i:i+4])


def to_long_uint(data: List[int], i: int) -> int:
    return data[i] + data[i + 1] * 0x100 + data[i + 2] * 0x10000 + data[i + 3] * 0x1000000


def ulong_to_buffer(v: int, buf: List[int], pos: int = 0) -> None:
    for i in range(4):
        buf[i + pos] = v % 256
        v = v >> 8


def to_short_int(data: List[int], i: int) -> int:
    return struct.unpack("h", bytes(data[i:i+2]))[0]


def short_to_buffer(v: int, buf: List[int], pos: int = 0) -> None:
    buf[pos:pos+2] = struct.pack("h", v)


def long_array_to_bytes(long_array: List[int]) -> List[int]:
    return list(struct.pack(f"{len(long_array)}I", *long_array))


def short_array_to_bytes(short_array: List[int]) -> List[int]:
    return list(struct.pack(f"{len(short_array)}H", *short_array))


def to_ulong_array(destination: List[int], source: List[int], source_index: int, source_size: int) -> None:
    buf = source[source_index:source_index + source_size]
    destination[0:source_size // 4] = struct.unpack(f"{int(source_size // 4)}I", bytes(buf))


def to_uint_array(destination: List[int], source: List[int], source_index: int, source_size: int) -> None:
    buf = source[source_index:source_index + source_size]
    destination[0:source_size // 2] = struct.unpack(f"{int(source_size // 2)}H", bytes(buf))


def to_int_array(destination: List[int], source: List[int], source_index: int, source_size: int) -> None:
    buf = source[source_index:source_index + source_size]
    destination[0:source_size // 2] = struct.unpack(f"{int(source_size // 2)}h", bytes(buf))
