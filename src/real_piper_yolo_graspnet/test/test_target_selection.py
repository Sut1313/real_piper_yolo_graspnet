import numpy as np

from real_piper_yolo_graspnet.d435i_yolo_graspnet_bridge import select_target_detections
from real_piper_yolo_graspnet.yolo_detector import Detection


def detection(name, class_id, confidence, region):
    mask = np.zeros((20, 30), dtype=bool)
    y0, y1, x0, x1 = region
    mask[y0:y1, x0:x1] = True
    return Detection(
        class_id=class_id,
        class_name=name,
        confidence=confidence,
        box_xyxy=np.array([x0, y0, x1, y1], dtype=np.float32),
        mask=mask,
    )


def select(items, fail_on_multiple=True):
    return select_target_detections(
        items,
        'cup',
        np.full((20, 30), 600, dtype=np.uint16),
        1000.0,
        False,
        0.15,
        1.20,
        10,
        10,
        fail_on_multiple,
    )


def test_selects_requested_class_only():
    selected, reason = select([
        detection('bottle', 11, 0.99, (1, 10, 1, 10)),
        detection('cup', 10, 0.75, (4, 14, 10, 22)),
    ])
    assert reason == 'selected'
    assert [item.class_name for item in selected] == ['cup']


def test_multiple_matching_targets_fail_closed_by_default():
    selected, reason = select([
        detection('cup', 10, 0.90, (1, 11, 1, 11)),
        detection('cup', 10, 0.80, (5, 15, 15, 25)),
    ])
    assert selected == []
    assert reason == 'ambiguous_target_count=2'


def test_invalid_depth_is_rejected():
    item = detection('cup', 10, 0.90, (1, 11, 1, 11))
    selected, reason = select_target_detections(
        [item], 'cup', np.zeros((20, 30), dtype=np.uint16), 1000.0, False,
        0.15, 1.20, 10, 10, True)
    assert selected == []
    assert reason == 'cup:depth=0'
