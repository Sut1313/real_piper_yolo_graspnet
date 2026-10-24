import json

import numpy as np
import scipy.io as scio

from real_piper_yolo_graspnet.frame_writer import FrameBundleWriter, load_instance_archive
from real_piper_yolo_graspnet.yolo_detector import Detection


def make_detection():
    mask = np.zeros((48, 64), dtype=bool)
    mask[12:32, 20:44] = True
    return Detection(
        class_id=10,
        class_name='cup',
        confidence=0.83,
        box_xyxy=np.array([20, 12, 44, 32], dtype=np.float32),
        mask=mask,
    )


def test_frame_bundle_contract_and_busy_gate(tmp_path):
    writer = FrameBundleWriter(str(tmp_path), write_overlay=True)
    color = np.zeros((48, 64, 3), dtype=np.uint8)
    depth = np.full((48, 64), 500, dtype=np.uint16)
    intrinsic = np.array([[100, 0, 32], [0, 100, 24], [0, 0, 1]], dtype=float)

    assert writer.write(
        color,
        depth,
        intrinsic,
        1000.0,
        [make_detection()],
        color_stamp_ns=1_000_000,
        depth_stamp_ns=1_010_000,
        frame_id='camera_color_optical_frame',
        model_path='/models/model.onnx',
        model_sha256='abc123',
        backend_name='test',
    )
    assert writer.busy
    assert not writer.write(
        color,
        depth,
        intrinsic,
        1000.0,
        [],
        color_stamp_ns=2,
        depth_stamp_ns=2,
        frame_id='camera_color_optical_frame',
        model_path='/models/model.onnx',
        model_sha256='abc123',
        backend_name='test',
    )

    assert (tmp_path / '.ready').read_text().strip() == '1'
    manifest = json.loads((tmp_path / 'frame_manifest.json').read_text())
    assert manifest['frame_sequence'] == 1
    assert manifest['detection_count'] == 1
    assert manifest['stamp_delta_ns'] == 10_000
    archive = load_instance_archive(str(tmp_path / 'instances.npz'), (48, 64))
    assert archive['masks'].shape == (1, 48, 64)
    assert archive['names'].tolist() == ['cup_0']
    assert int(np.count_nonzero(archive['masks'][0])) == 480
    meta = scio.loadmat(tmp_path / 'meta.mat')
    np.testing.assert_allclose(meta['intrinsic_matrix'], intrinsic)
    assert (tmp_path / 'color.png').is_file()
    assert (tmp_path / 'depth.png').is_file()
    assert (tmp_path / 'yolo_overlay.png').is_file()


def test_empty_detection_archive_is_well_formed(tmp_path):
    writer = FrameBundleWriter(str(tmp_path), write_overlay=False)
    writer.write(
        np.zeros((10, 12, 3), dtype=np.uint8),
        np.full((10, 12), 700, dtype=np.uint16),
        np.eye(3),
        1000.0,
        [],
        color_stamp_ns=1,
        depth_stamp_ns=1,
        frame_id='camera',
        model_path='model.onnx',
        model_sha256='hash',
        backend_name='test',
    )
    archive = load_instance_archive(str(tmp_path / 'instances.npz'), (10, 12))
    assert archive['masks'].shape == (0, 10, 12)
    assert archive['boxes_xyxy'].shape == (0, 4)
