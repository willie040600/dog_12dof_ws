from robot_dog_agent.frames import to_rgb

RED, GREEN = bytes([255, 0, 0]), bytes([0, 255, 0])


def test_rgb8_passes_through():
    assert to_rgb('rgb8', 2, 1, 6, RED + GREEN) == RED + GREEN


def test_bgr8_is_swapped():
    assert to_rgb('bgr8', 1, 1, 3, bytes([0, 0, 255])) == RED


def test_alpha_is_dropped():
    assert to_rgb('rgba8', 1, 1, 4, bytes([255, 0, 0, 9])) == RED
    assert to_rgb('bgra8', 1, 1, 4, bytes([0, 0, 255, 9])) == RED


def test_mono8_becomes_grey():
    assert to_rgb('mono8', 2, 1, 2, bytes([0, 200])) == bytes([0, 0, 0, 200, 200, 200])


def test_row_padding_is_removed():
    row1, row2, pad = RED + GREEN, GREEN + RED, bytes([7, 7])
    assert to_rgb('rgb8', 2, 2, 8, row1 + pad + row2 + pad) == row1 + row2


def test_bad_frames_give_none():
    for args in [('yuv422', 1, 1, 2, b'ab'), ('rgb8', 2, 1, 6, RED), ('rgb8', 0, 1, 0, b''),
                 ('rgb8', 2, 1, 5, RED + GREEN), ('rgb8', -1, 1, 3, RED), ('rgb8', 1, 0, 3, RED)]:
        assert to_rgb(*args) is None, args


def test_the_result_is_bytes_of_the_right_length():
    out = to_rgb('rgb8', 640, 480, 1920, bytes(640 * 480 * 3))
    assert isinstance(out, bytes) and len(out) == 921600


def test_other_buffer_types_are_accepted():
    import array
    assert to_rgb('rgb8', 1, 1, 3, array.array('B', [255, 0, 0])) == RED      # what rclpy hands over
    assert to_rgb('rgb8', 1, 1, 3, bytearray(RED)) == RED
