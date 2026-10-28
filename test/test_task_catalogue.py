"""Tests for the step catalogue (tasks/catalogue.py) a client builds its editor from.

What these hold is that the catalogue cannot say something the validators
then refuse: its body rule is BODY_STEP_TYPES, its limits are the params
models' own, and every step a template stores is in it but UNDOCK, which a
MOVE does by itself.
"""

import pytest

pytest.importorskip("pydantic")

from syncai_backend.tasks.catalogue import step_catalogue  # noqa: E402
from syncai_backend.tasks.schema import (  # noqa: E402
    BODY_STEP_TYPES,
    POSE_STEP_TYPES,
    DemoParams,
    InspectionId,
    SPEAK_REPEAT_MAX,
    SpeakParams,
    StepType,
    WaitParams,
)


def _spec(step_type):
    return next(spec for spec in step_catalogue().steps if spec.type == step_type)


def _field(step_type, key):
    return next(field for field in _spec(step_type).params if field.key == key)


def test_lists_every_step_an_operator_authors():
    listed = [spec.type for spec in step_catalogue().steps]
    assert set(listed) == set(StepType) - {StepType.UNDOCK}
    assert len(listed) == len(set(listed))


def test_occupies_the_body_exactly_where_a_group_refuses_a_second():
    for spec in step_catalogue().steps:
        assert spec.occupies_body == (spec.type in BODY_STEP_TYPES), spec.type


def test_offers_only_speech_on_the_walk_to_a_stop():
    # A body step there is refused (one per group); a WAIT is legal but only
    # makes the walk last at least that long, which no operator means.
    offered = [spec.type for spec in step_catalogue().steps if spec.while_walking]
    assert offered == [StepType.SPEAK]
    for spec in step_catalogue().steps:
        assert not (spec.while_walking and spec.occupies_body), spec.type


def test_offers_a_waypoint_for_exactly_the_steps_that_drive_to_a_pose():
    for spec in step_catalogue().steps:
        has_waypoint = any(field.kind == "waypoint" for field in spec.params)
        assert has_waypoint == (spec.type in POSE_STEP_TYPES), spec.type
    assert _field(StepType.DOCK, "target").filter == "charger"


def test_reads_its_limits_off_the_params_models():
    # A limit moved in a params model has to move here too, or the console
    # refuses what the robot takes, or lets through what it 422s.
    assert _field(StepType.SPEAK, "text").max_length == 1000
    repeat = _field(StepType.SPEAK, "repeat")
    assert (repeat.min, repeat.max, repeat.default) == (1, SPEAK_REPEAT_MAX, 1)
    seconds = _field(StepType.WAIT, "seconds")
    assert (seconds.min, seconds.min_exclusive, seconds.max) == (0, True, 3600)

    # And the models agree with what the catalogue says at its edges.
    SpeakParams(text="x" * 1000, repeat=SPEAK_REPEAT_MAX)
    with pytest.raises(ValueError):
        SpeakParams(text="x" * 1001)
    with pytest.raises(ValueError):
        WaitParams(seconds=0)
    WaitParams(seconds=3600)


def test_names_only_fields_the_params_models_take():
    # `target` is the waypoint, spread into x / y / theta / z on the wire.
    for step_type, model in ((StepType.SPEAK, SpeakParams), (StepType.WAIT, WaitParams)):
        for field in _spec(step_type).params:
            assert field.key in model.model_fields, (step_type, field.key)


def test_serialises_in_the_shape_the_console_reads():
    body = step_catalogue().model_dump(mode="json")
    speak = next(spec for spec in body["steps"] if spec["type"] == "SPEAK")
    assert speak["occupies_body"] is False
    assert speak["params"][0] == {
        "kind": "text",
        "key": "text",
        "label": "Say",
        "max_length": 1000,
        "placeholder": "Delivery arrived — please take your items.",
    }


def test_offers_every_inspection_result_a_demo_step_takes():
    (field,) = _spec(StepType.DEMO).params
    assert field.kind == "choice"
    assert field.key in DemoParams.model_fields
    assert [option.value for option in field.options] == [item.value for item in InspectionId]
