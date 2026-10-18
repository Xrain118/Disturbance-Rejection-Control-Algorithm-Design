"""URDF -> flight metadata / PX4 / applied joint-limit contract checks."""
from pathlib import Path
import json
import xml.etree.ElementTree as ET
import numpy as np
import pytest
from dragonfly_flight.model import generate
from dragonfly_flight.native import px4_parameters
from dragonfly_flight.effort import ordered_arm_effort_limits, arm_effort_ratios
from dragonfly_flight.core import ARM_NAMES


NAMES = [name for side in ('left', 'right') for name in ARM_NAMES[side]]
LIMITS = np.tile([2.941995, 5.883990, 2.941995, 2.941995, 2.941995], 2)


@pytest.fixture(scope='module')
def assets(tmp_path_factory):
    source = Path(__file__).resolve().parents[2]
    output = tmp_path_factory.mktemp('drawing_flight')
    metadata = generate(source / 'dragonfly_arm', source / 'dragonfly_flight', output, 'px4_native')
    return output, metadata


def test_mass_and_named_efforts_reach_all_generated_interfaces(assets):
    output, metadata = assets
    assert metadata['mass'] == pytest.approx(5.415, abs=1e-12, rel=0)
    assert np.linalg.eigvalsh(metadata['inertia']).min() > 0
    assert np.linalg.eigvalsh(metadata['stowed_inertia']).min() > 0
    assert 'arm_effort' not in metadata['config']
    assert ordered_arm_effort_limits(metadata) == pytest.approx(LIMITS)
    assert json.loads((output / 'model.json').read_text()) == metadata
    robot = ET.parse(output / 'robot.urdf').getroot()
    # Gazebo's literal gz:systems:priority tag is a TinyXML extension, not an
    # XML namespace accepted by ElementTree. Rename only that tag for inspection.
    world = ET.fromstring((output / 'world.sdf').read_text().replace(
        'gz:systems:priority', 'gazebo_systems_priority'))
    physics = world.find(".//plugin[@name='dragonfly::Physics']")
    applied = {node.get('joint'): float(node.text) for node in physics.findall('arm_effort_limit')}
    assert applied == metadata['arm_effort_limits']
    for name, expected in zip(NAMES, LIMITS):
        assert float(robot.find(f"joint[@name='{name}']/limit").get('effort')) == pytest.approx(expected)
        interface = robot.find(f"ros2_control/joint[@name='{name}']/command_interface")
        assert interface.get('name') == 'effort'
        assert float(interface.find("param[@name='min']").text) == pytest.approx(-expected)
        assert float(interface.find("param[@name='max']").text) == pytest.approx(expected)
    params = px4_parameters(metadata)
    assert params['DF_MASS'] == pytest.approx(5.415, abs=1e-12, rel=0)
    assert [params['DF_COM_' + axis] for axis in 'XYZ'] == pytest.approx(metadata['com'])
    inertia = np.asarray(metadata['inertia'])
    for row, col in ((0, 0), (1, 1), (2, 2), (0, 1), (0, 2), (1, 2)):
        assert params['DF_I' + 'XYZ'[row] + 'XYZ'[col]] == pytest.approx(inertia[row, col])


def test_joint_names_control_order_and_saturation_instead_of_global_maximum():
    mapping = {name: limit for name, limit in reversed(list(zip(NAMES, LIMITS)))}
    limits = ordered_arm_effort_limits({'arm_effort_limits': mapping})
    efforts = np.zeros(10)
    efforts[1] = 4.0  # Below the two-servo shoulder peak.
    assert max(arm_effort_ratios(efforts, limits)) < .99
    efforts[2] = -2.941995  # Elbow is saturated even though shoulder limit is higher.
    assert max(arm_effort_ratios(efforts, limits)) == pytest.approx(1.0)


def test_scalar_limit_only_allowed_for_historical_reading():
    old = {'config': {'arm_effort': 5.0}}
    assert ordered_arm_effort_limits(old, allow_legacy=True) == pytest.approx(np.full(10, 5.0))
    with pytest.raises(ValueError):
        ordered_arm_effort_limits(old)
    for bad in ({}, {name: float('nan') for name in NAMES}, {name: 0 for name in NAMES}):
        with pytest.raises(ValueError):
            ordered_arm_effort_limits({'arm_effort_limits': bad}, allow_legacy=True)
