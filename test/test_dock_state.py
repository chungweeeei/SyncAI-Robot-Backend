"""Tests for DockStateService: the last DOCK target and the on-dock check."""

import math
from types import SimpleNamespace

from syncai_backend.services.dock_state import (
    ON_DOCK_XY_TOLERANCE_M,
    DockStateService,
)


def _state(x=0.0, y=0.0, theta_deg=0.0, localized=True):
    return SimpleNamespace(
        localization_valid=localized,
        localization_status=SimpleNamespace(
            position=SimpleNamespace(x=x, y=y, yaw=math.radians(theta_deg))
        ),
    )


def _docked(map_name="lab"):
    svc = DockStateService()
    svc.record(map_name, 1.0, 2.0, 90.0)
    return svc


def test_no_record_is_not_on_a_dock():
    assert DockStateService().is_on_dock(_state(), "lab") is False


def test_within_the_tolerances_is_on_the_dock():
    svc = _docked()
    assert svc.is_on_dock(_state(1.1, 2.1, 100.0), "lab") is True
    assert svc.current() is not None


def test_yaw_is_compared_across_the_wrap():
    svc = DockStateService()
    svc.record("lab", 0.0, 0.0, 179.0)
    assert svc.is_on_dock(_state(0.0, 0.0, -179.0), "lab") is True


def test_too_far_away_drops_the_record():
    svc = _docked()
    assert svc.is_on_dock(_state(1.0 + ON_DOCK_XY_TOLERANCE_M + 0.05, 2.0, 90.0), "lab") is False
    assert svc.current() is None


def test_turned_away_drops_the_record():
    svc = _docked()
    assert svc.is_on_dock(_state(1.0, 2.0, 150.0), "lab") is False
    assert svc.current() is None


def test_another_map_drops_the_record():
    svc = _docked()
    assert svc.is_on_dock(_state(1.0, 2.0, 90.0), "warehouse") is False
    assert svc.current() is None


def test_an_unknown_map_is_not_a_mismatch():
    assert _docked(map_name=None).is_on_dock(_state(1.0, 2.0, 90.0), "lab") is True
    assert _docked().is_on_dock(_state(1.0, 2.0, 90.0), None) is True


def test_no_pose_proves_nothing_and_keeps_the_record():
    svc = _docked()
    assert svc.is_on_dock(None, "lab") is False
    assert svc.is_on_dock(_state(localized=False), "lab") is False
    assert svc.current() is not None


def test_clear_forgets():
    svc = _docked()
    svc.clear()
    assert svc.current() is None
