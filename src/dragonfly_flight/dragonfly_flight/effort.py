"""Named arm peak limits; scalar support is only for reading historical runs."""
import numpy as np
from .core import ARM_NAMES


def ordered_arm_effort_limits(metadata, *, allow_legacy=False):
    names = [name for side in ('left', 'right') for name in ARM_NAMES[side]]
    mapping = metadata.get('arm_effort_limits')
    if mapping is None and allow_legacy:
        scalar = metadata.get('config', {}).get('arm_effort')
        if scalar is not None:
            mapping = {name: scalar for name in names}
    if not isinstance(mapping, dict) or set(mapping) != set(names):
        raise ValueError('Arm effort limits must name exactly all ten arm joints')
    values = np.asarray([mapping[name] for name in names], dtype=float)
    if not np.all(np.isfinite(values)) or np.any(values <= 0):
        raise ValueError('Arm effort limits must be finite and positive')
    return values


def arm_effort_ratios(efforts, limits):
    values, limits = np.asarray(efforts, dtype=float), np.asarray(limits, dtype=float)
    if values.shape != (10,) or limits.shape != (10,) or not np.all(np.isfinite(values)):
        raise ValueError('Expected ten finite arm effort samples and limits')
    if not np.all(np.isfinite(limits)) or np.any(limits <= 0):
        raise ValueError('Arm effort limits must be finite and positive')
    return np.abs(values) / limits
