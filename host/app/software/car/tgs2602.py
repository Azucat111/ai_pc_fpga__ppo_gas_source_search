"""PC-side TGS2602 streaming features for the car RL observation."""
from collections import deque
import math


def _clip(value, low=-1.0, high=1.0):
    return max(low, min(high, float(value)))


class TGS2602Preprocessor:
    """Stateful, timestamp-aware two-channel TGS feature extractor.

    The input is the decoded 0x81 TGS list. Samples are de-duplicated using
    board timestamp minus per-channel age. Baselines are session parameters;
    this class never adapts them while a plume is present.

    ``noise_response`` uses normalized response units (voltage delta divided
    by ``response_scale``), not volts.
    """

    FEATURE_NAMES = (
        'tgs0_norm', 'tgs1_norm', 'tgs0_delta', 'tgs1_delta',
        'diff_raw', 'diff_corrected', 'max_response', 'gradient_strength',
        'tgs0_trend', 'tgs1_trend', 'diff_trend', 'plume_detected',
        'direction_confidence',
    )
    EVENT_FEATURE_NAMES = ('plume_enter', 'plume_stable', 'plume_lost')

    def __init__(self, baseline_tgs0=0.2325, baseline_tgs1=0.2083125,
                 window_size=20, response_scale=0.10, slope_scale=0.05,
                 ema_tau_s=0.6, plume_enter_sigma=4.0,
                 plume_exit_sigma=2.0, plume_enter_samples=3,
                 plume_exit_samples=5, noise_response=0.0028125,
                 trigger_enter_z=4.0, trigger_exit_z=2.0,
                 trigger_common_z=2.5, trigger_max_gap_s=0.5):
        if (window_size < 3 or response_scale <= 0 or slope_scale <= 0 or
                noise_response <= 0 or trigger_enter_z <= 0 or
                trigger_exit_z <= 0 or trigger_common_z <= 0 or
                trigger_max_gap_s <= 0):
            raise ValueError('invalid TGS preprocessor parameters')
        self.baseline_tgs0 = float(baseline_tgs0)
        self.baseline_tgs1 = float(baseline_tgs1)
        self.window_size = int(window_size)
        self.response_scale = float(response_scale)
        self.slope_scale = float(slope_scale)
        self.ema_tau_s = float(ema_tau_s)
        self.plume_enter_sigma = float(plume_enter_sigma)
        self.plume_exit_sigma = float(plume_exit_sigma)
        self.plume_enter_samples = int(plume_enter_samples)
        self.plume_exit_samples = int(plume_exit_samples)
        self.noise_response = float(noise_response)
        self.trigger_enter_z = float(trigger_enter_z)
        self.trigger_exit_z = float(trigger_exit_z)
        self.trigger_common_z = float(trigger_common_z)
        self.trigger_max_gap_s = float(trigger_max_gap_s)
        self.reset()

    def reset(self):
        self.ema0 = None
        self.ema1 = None
        self.last_sample_ms = [None, None]
        self._pending_channels = [False, False]
        self.times = deque(maxlen=self.window_size)
        self.responses0 = deque(maxlen=self.window_size)
        self.responses1 = deque(maxlen=self.window_size)
        self.diffs = deque(maxlen=self.window_size)
        self.sample_count = 0
        self.plume_state = False
        self._enter_count = 0
        self._exit_count = 0
        self.last_features = None
        self.last_boot_id = None

    @staticmethod
    def _sample_time_ms(frame, sample):
        timestamp = frame.get('timestamp_ms')
        age = sample.get('age_ms')
        if timestamp is None or age is None:
            return None
        return (int(timestamp) - int(age)) & 0xffffffff

    @staticmethod
    def _voltage(sample):
        if not sample or not sample.get('valid'):
            return None
        value = sample.get('voltage_v')
        if value is None or not math.isfinite(float(value)):
            return None
        return float(value)

    def _ema(self, value, previous, dt_s):
        if previous is None:
            return value
        dt_s = max(0.001, min(2.0, float(dt_s)))
        alpha = 1.0 - math.exp(-dt_s / self.ema_tau_s)
        return previous + alpha * (value - previous)

    @staticmethod
    def _slope(times, values):
        if len(values) < 3:
            return 0.0
        t0 = times[0]
        xs = [t - t0 for t in times]
        xm = sum(xs) / len(xs)
        ym = sum(values) / len(values)
        denom = sum((x - xm) ** 2 for x in xs)
        return sum((x - xm) * (y - ym) for x, y in zip(xs, values)) / denom if denom else 0.0

    def _features(self, now_s):
        if not self.responses0 or not self.responses1 or not self.diffs:
            raise RuntimeError('TGS features requested before the first sample')
        r0 = self.responses0[-1]
        r1 = self.responses1[-1]
        common = (r0 + r1) / 2.0
        diff = (r0 - r1) / 2.0
        slope0 = self._slope(self.times, self.responses0)
        slope1 = self._slope(self.times, self.responses1)
        diff_slope = self._slope(self.times, self.diffs)
        common_slope = (slope0 + slope1) / 2.0
        noise = max(1e-6, self.noise_response)
        common_z = common / noise
        max_abs_z = max(abs(r0), abs(r1)) / noise
        same_direction = r0 * r1 >= 0
        trigger_ready = (same_direction and max_abs_z >= self.trigger_enter_z and
                         abs(common_z) >= self.trigger_common_z)
        trigger_release = (max_abs_z <= self.trigger_exit_z or
                           abs(common_z) <= self.trigger_exit_z)
        confidence = _clip(abs(diff) / (noise * 4.0), 0.0, 1.0)
        return {
            'tgs0_norm': _clip(r0), 'tgs1_norm': _clip(r1),
            'tgs0_delta': _clip(r0), 'tgs1_delta': _clip(r1),
            'diff_raw': _clip((self.ema0 - self.ema1) / self.response_scale),
            'diff_corrected': _clip(diff),
            'max_response': _clip(max(r0, r1), 0.0, 1.0),
            'gradient_strength': _clip(abs(diff)),
            'tgs0_trend': _clip(slope0 / self.slope_scale),
            'tgs1_trend': _clip(slope1 / self.slope_scale),
            'diff_trend': _clip(diff_slope / self.slope_scale),
            'plume_detected': float(self.plume_state),
            'direction_confidence': confidence,
            'response_left': r0, 'response_right': r1,
            'common_response': common, 'differential_response': diff,
            'common_slope': common_slope,
            'differential_slope': diff_slope,
            'common_z': common_z, 'max_abs_z': max_abs_z,
            'same_direction': same_direction,
            'trigger_ready': trigger_ready,
            'trigger_release': trigger_release,
            'sample_time_s': now_s, 'sample_count': self.sample_count,
        }

    def process_frame(self, frame):
        sensors = frame.get('tgs2602') or []
        if self.last_boot_id is not None and frame.get('boot_id') != self.last_boot_id:
            self.reset()
        self.last_boot_id = frame.get('boot_id')
        if len(sensors) < 2:
            return {'valid': False, 'updated': False, 'reason': 'missing_channels',
                    'quality': 'missing', 'features': self.last_features}
        values = [self._voltage(sensors[0]), self._voltage(sensors[1])]
        times = [self._sample_time_ms(frame, sensors[i]) for i in (0, 1)]
        if any(value is None or times[i] is None for i, value in enumerate(values)):
            return {'valid': False, 'updated': False, 'reason': 'invalid_sample',
                    'quality': 'invalid', 'features': self.last_features}
        changed = [times[i] != self.last_sample_ms[i] for i in (0, 1)]
        for i in (0, 1):
            if changed[i]:
                self.last_sample_ms[i] = times[i]
                self._pending_channels[i] = True
        if not all(self._pending_channels):
            return {'valid': True, 'updated': False,
                    'reason': 'awaiting_pair' if any(changed) else 'duplicate_sample',
                    'quality': 'partial' if any(changed) else 'good',
                    'sample_time_ms': max(times), 'features': self.last_features}
        if not any(changed) and self.last_features is not None:
            return {'valid': True, 'updated': False, 'reason': 'duplicate_sample',
                    'quality': 'good', 'sample_time_ms': max(times),
                    'features': self.last_features}
        now_s = max(times) / 1000.0
        previous_s = self.times[-1] if self.times else now_s
        dt_s = max(0.001, now_s - previous_s)
        self.ema0 = self._ema(values[0], self.ema0, dt_s)
        self.ema1 = self._ema(values[1], self.ema1, dt_s)
        self._pending_channels = [False, False]
        self.sample_count += 1
        r0 = (self.ema0 - self.baseline_tgs0) / self.response_scale
        r1 = (self.ema1 - self.baseline_tgs1) / self.response_scale
        self.times.append(now_s)
        self.responses0.append(r0)
        self.responses1.append(r1)
        self.diffs.append((r0 - r1) / 2.0)
        common = (r0 + r1) / 2.0
        enter = common >= self.plume_enter_sigma * 0.01
        leave = common <= self.plume_exit_sigma * 0.01
        self._enter_count = self._enter_count + 1 if enter else 0
        self._exit_count = self._exit_count + 1 if leave else 0
        previous_plume_state = self.plume_state
        if not self.plume_state and self._enter_count >= self.plume_enter_samples:
            self.plume_state = True
            self._enter_count = 0
        if self.plume_state and self._exit_count >= self.plume_exit_samples:
            self.plume_state = False
            self._exit_count = 0
        features = self._features(now_s)
        features['plume_enter'] = float(not previous_plume_state and self.plume_state)
        features['plume_stable'] = float(self.plume_state)
        features['plume_lost'] = float(previous_plume_state and not self.plume_state)
        self.last_features = features
        return {'valid': True, 'updated': True, 'reason': 'new_sample',
                'quality': 'good', 'sample_time_ms': max(times),
                'features': features}

    def metadata(self):
        return {
            'version': 'tgs2602_features_v1',
            'baseline_tgs0_v': self.baseline_tgs0,
            'baseline_tgs1_v': self.baseline_tgs1,
            'window_size_samples': self.window_size,
            'response_scale_v': self.response_scale,
            'slope_scale_per_s': self.slope_scale,
            'noise_response': self.noise_response,
            'noise_response_units': 'normalized_response',
            'trigger_enter_z': self.trigger_enter_z,
            'trigger_exit_z': self.trigger_exit_z,
            'trigger_common_z': self.trigger_common_z,
            'trigger_max_gap_s': self.trigger_max_gap_s,
            'deduplication': 'boot_id plus board timestamp_ms minus channel age_ms',
            'feature_names': list(self.FEATURE_NAMES + self.EVENT_FEATURE_NAMES),
            'event_semantics': {
                'plume_enter': 'one-sample rising edge after enter hysteresis',
                'plume_stable': 'current hysteresis state',
                'plume_lost': 'one-sample falling edge after exit hysteresis',
            },
        }
