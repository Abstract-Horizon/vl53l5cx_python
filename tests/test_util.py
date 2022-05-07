def test_to_ulong_array():
    from vl53l5cx import util
    src = [2, 0, 0, 0, 1, 0, 0, 0]
    dst = [None, None]

    util.to_ulong_array(dst, src, 0, 8)
    assert dst == [2, 1]

    dst = [None]
    util.to_ulong_array(dst, src, 4, 4)
    assert dst == [1]

    dst = [None]
    util.to_ulong_array(dst, src, 0, 4)
    assert dst == [2]


def test_to_uint_array():
    from vl53l5cx import util
    src = [2, 0, 1, 0]
    dst = [None, None]

    util.to_uint_array(dst, src, 0, 4)
    assert dst == [2, 1]

    dst = [None]
    util.to_uint_array(dst, src, 2, 2)
    assert dst == [1]

    dst = [None]
    util.to_uint_array(dst, src, 0, 2)
    assert dst == [2]


def test_to_int_array():
    from vl53l5cx import util
    src = [255, 255, 1, 0]
    dst = [None, None]

    util.to_int_array(dst, src, 0, 4)
    assert dst == [-1, 1]

    dst = [None]
    util.to_int_array(dst, src, 2, 2)
    assert dst == [1]

    dst = [None]
    util.to_int_array(dst, src, 0, 2)
    assert dst == [-1]


def test_short_to_buffer():
    from vl53l5cx import util
    dst = [None, None, None, None, None, None]

    util.short_to_buffer(-1, dst, 0)
    util.short_to_buffer(32767, dst, 2)
    util.short_to_buffer(-32768, dst, 4)

    assert dst == [255, 255, 255, 127, 0, 128]


def test_to_short_int():
    from vl53l5cx import util
    src = [255, 255, 255, 127, 0, 128]

    assert util.to_short_int(src, 0) == -1
    assert util.to_short_int(src, 2) == 32767
    assert util.to_short_int(src, 4) == -32768

def test_long_array_to_bytes():
    from vl53l5cx import util
    src = [0xFFFFFFFF, 0, 0x80706050]

    assert util.long_array_to_bytes(src) == [255, 255, 255, 255, 0, 0, 0, 0, 0x50, 0x60, 0x70, 0x80]

def test_short_array_to_bytes():
    from vl53l5cx import util
    src = [0xFFFF, 0, 0x8000, 0x0080]

    assert util.short_array_to_bytes(src) == [0xff, 0xff, 0x00, 0x00, 0x00, 0x80, 0x80, 0x00]