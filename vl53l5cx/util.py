from typing import List


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
