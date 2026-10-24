import numpy as np

from real_piper_yolo_graspnet.yolo_detector import (
    TABLETOP_CLASS_NAMES,
    decode_yolov8_segmentation,
    letterbox,
)


def test_letterbox_640_by_480_has_vertical_padding():
    image = np.zeros((480, 640, 3), dtype=np.uint8)
    tensor, transform = letterbox(image, 640)
    assert tensor.shape == (1, 3, 640, 640)
    assert transform.scale == 1.0
    assert transform.left == 0
    assert transform.top == 80
    assert transform.resized_width == 640
    assert transform.resized_height == 480


def test_decode_one_segmentation_and_restore_original_shape():
    _, transform = letterbox(np.zeros((480, 640, 3), dtype=np.uint8), 640)
    predictions = np.zeros((1, 52, 8400), dtype=np.float32)
    predictions[0, 0:4, 0] = [320.0, 320.0, 200.0, 160.0]
    predictions[0, 4 + 10, 0] = 0.90  # cup
    predictions[0, 4 + len(TABLETOP_CLASS_NAMES), 0] = 10.0
    prototypes = np.zeros((1, 32, 160, 160), dtype=np.float32)
    prototypes[0, 0, :, :] = 1.0

    detections = decode_yolov8_segmentation(
        predictions,
        prototypes,
        transform,
        confidence_threshold=0.25,
    )

    assert len(detections) == 1
    detection = detections[0]
    assert detection.class_id == 10
    assert detection.class_name == 'cup'
    assert detection.mask.shape == (480, 640)
    assert 25_000 < int(np.count_nonzero(detection.mask)) < 40_000
    np.testing.assert_allclose(detection.box_xyxy, [220, 160, 420, 320], atol=1.0)
