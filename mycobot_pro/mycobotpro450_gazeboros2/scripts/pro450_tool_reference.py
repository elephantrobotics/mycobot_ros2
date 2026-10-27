"""Public-owner tool/frame settings scoped to GUI or Cartesian keyboard leases."""
import math


GRIPPER_TOOL_REFERENCE = (0., 0., 165., 0., 0., 0.)  # mm / degrees, in flange axes.


class ToolReferenceContext:
    def __init__(self, sdk):
        self.sdk = sdk
        self.saved = None

    @staticmethod
    def _reference(value):
        if (not isinstance(value, (list, tuple)) or len(value) != 6 or
                any(type(v) not in (int, float) or not math.isfinite(v) for v in value)):
            raise RuntimeError(f'Invalid SDK tool reference: {value!r}')
        return list(value)

    def _read(self, keys):
        result = {}
        for key in keys:
            if key == 'tool':
                result[key] = self._reference(self.sdk.get_tool_reference())
            else:
                value = getattr(self.sdk, 'get_' + key)()
                if type(value) is not int or value not in (0, 1):
                    raise RuntimeError(f'Invalid SDK {key}: {value!r}')
                result[key] = value
        return result

    def _set(self, settings):
        # The vendor encoder truncates int(value * scale). Preserve original
        # readback ticks when binary floats such as 0.29 fall just below a tick.
        for key, value in settings.items():
            if key == 'tool':
                encoded = []
                for index, number in enumerate(value):
                    scale = 10 if index < 3 else 100
                    ticks = round(number * scale)
                    wire = ticks / scale
                    if int(wire * scale) != ticks:
                        wire = math.nextafter(wire, math.inf if ticks > 0 else -math.inf)
                    encoded.append(wire)
                self.sdk.set_tool_reference(encoded)
            else:
                getattr(self.sdk, 'set_' + key)(value)
        actual = self._read(settings)
        for key, value in settings.items():
            matches = (all(abs(a-b) <= 1e-6 for a,b in zip(actual[key],value))
                       if key == 'tool' else actual[key] == value)
            if not matches:
                raise RuntimeError(f'SDK {key} was not confirmed: {actual[key]!r}')

    def apply(self, *, with_gripper=True, reference_frame=None, interpolation=False):
        if self.saved is not None:
            raise RuntimeError('Previous control tool reference has not been restored')
        settings = {}
        if with_gripper:
            settings['tool'] = GRIPPER_TOOL_REFERENCE
        settings['end_type'] = 1 if with_gripper else 0
        if reference_frame is not None:
            if reference_frame not in (0, 1):
                raise ValueError('Invalid coordinate reference frame')
            settings['reference_frame'] = reference_frame
        if interpolation:
            settings['fresh_mode'] = 0
        self.saved = self._read(settings)
        try:
            self._set(settings)
        except Exception as exc:
            try:
                self.restore()
            except Exception as cleanup:
                raise RuntimeError(f'Tool setup failed: {exc}; restoration failed: {cleanup}') from exc
            raise RuntimeError(f'Tool setup failed: {exc}') from exc

    def restore(self):
        if self.saved is None:
            return
        self._set(self.saved)
        self.saved = None
