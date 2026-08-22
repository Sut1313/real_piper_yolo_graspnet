import numpy as np
import pytest
from real_piper_perception.sdk_safe_execute import ForbiddenZone

zone = ForbiddenZone()

def test_box_inside():
    assert zone.point_in_base_box(np.array([0.0, 0.0, 0.06]))       # 盒中心
    assert zone.point_in_base_box(np.array([0.07, 0.07, 0.05]))      # 盒内角部

def test_box_outside():
    assert not zone.point_in_base_box(np.array([0.20, 0.0, 0.06]))   # x 超出
    assert not zone.point_in_base_box(np.array([0.0, 0.0, 0.20]))    # z 超出
    assert not zone.point_in_base_box(np.array([0.0, 0.0, -0.05]))   # 地面以下

def test_box_buffer():
    # 盒面外 3mm（<5mm 缓冲）视为进入
    assert zone.point_in_base_box(np.array([0.078, 0.0, 0.06]))
    # 盒面外 8mm（>5mm 缓冲）不进入
    assert not zone.point_in_base_box(np.array([0.083, 0.0, 0.06]))

def test_cyl_and_z():
    assert zone.point_in_cyl(np.array([0.10, 0.0, 0.20]))    # xy=0.10<0.25, z=0.20<0.40
    assert not zone.point_in_cyl(np.array([0.30, 0.0, 0.20]))  # xy 超出
    assert not zone.point_in_cyl(np.array([0.10, 0.0, 0.50]))  # z 超出
    assert zone.z_below_min(np.array([0.4, 0.0, 0.17]))      # z=0.17<0.18
    assert not zone.z_below_min(np.array([0.4, 0.0, 0.19]))

def test_box_clearance():
    assert abs(zone.box_clearance(np.array([0.075, 0.0, 0.06]))) < 1e-9  # 盒面距离 0
    assert zone.box_clearance(np.array([0.10, 0.0, 0.06])) > 0.02        # 盒外正距离
    assert zone.box_clearance(np.array([0.0, 0.0, 0.06])) < 0            # 盒内负距离
