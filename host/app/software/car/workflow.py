"""AI PC-side GAS10 classification and RL observation adapters."""
import math

from .config import velocity_targets
from .protocol import command
from .tgs2602 import TGS2602Preprocessor

RL_FEATURES_18 = (
    'alcohol_prob', 'gas_resistance_norm', 'temperature', 'humidity',
    'tgs_left_norm', 'tgs_right_norm', 'tgs_diff_norm', 'tgs_mean',
    'tgs_slope', 'plume_enter', 'plume_stable', 'plume_lost',
    'linear_velocity', 'angular_velocity', 'gyro_z',
    'last_action', 'action_age', 'communication_quality',
)
RL_FEATURES_17 = tuple(name for name in RL_FEATURES_18 if name != 'gas_resistance_norm')
RL_POSITION_FEATURES = ('x_norm', 'y_norm', 'sin_heading', 'cos_heading')

TGS_POLICY_FEATURES = {
    'tgs_left_norm': 'tgs0_norm',
    'tgs_right_norm': 'tgs1_norm',
    'tgs_diff_norm': 'diff_corrected',
    'tgs_mean': 'common_response',
    'tgs_slope': 'common_slope',
    'plume_enter': 'plume_enter',
    'plume_stable': 'plume_stable',
    'plume_lost': 'plume_lost',
}


def apply_short_tgs_hold(observation, frame, cache, max_hold_ms):
    """Hold the last paired TGS features briefly; keep the imputation explicit."""
    tgs = observation.get('tgs2602') or {}
    features = tgs.get('features')
    sample_time = tgs.get('sample_time_ms')
    if tgs.get('valid') and features and sample_time is not None:
        cache['features'] = dict(features)
        cache['sample_time_ms'] = int(sample_time)
        observation['imputed_features'] = []
        observation['tgs_hold_valid'] = False
        return False
    if not cache.get('features') or cache.get('sample_time_ms') is None:
        observation['imputed_features'] = []
        observation['tgs_hold_valid'] = False
        return False
    age_ms = (int(frame['timestamp_ms']) - int(cache['sample_time_ms'])) & 0xffffffff
    if age_ms > max_hold_ms:
        observation['imputed_features'] = []
        observation['tgs_hold_valid'] = False
        return False
    held_names = []
    for policy_name, source_name in TGS_POLICY_FEATURES.items():
        value = cache['features'].get(source_name)
        if value is None:
            continue
        if policy_name in ('plume_enter', 'plume_lost'):
            value = 0.0
        observation['features'][policy_name] = value
        observation['valid'][policy_name] = True
        held_names.append(policy_name)
    tgs['features'] = dict(cache['features'])
    tgs['held_sample_age_ms'] = age_ms
    observation['imputed_features'] = held_names
    observation['tgs_hold_valid'] = bool(held_names)
    return bool(held_names)
RL_FEATURES_21 = RL_FEATURES_17 + RL_POSITION_FEATURES
RL_FEATURES_22 = RL_FEATURES_18 + RL_POSITION_FEATURES
# The deployed single-car policy uses the position-aware 21D contract.
RL_FEATURES = RL_FEATURES_21
ACTION_NAMES = (
    'STOP', 'FORWARD_SLOW', 'FORWARD', 'TURN_LEFT', 'TURN_RIGHT',
    'ARC_LEFT', 'ARC_RIGHT',
)
VELOCITY_MAP = (
    (0.0, 0.0), (0.1, 0.0), (0.2, 0.0), (0.0, 0.8),
    (0.0, -0.8), (0.15, 0.5), (0.15, -0.5),
)


def _wrap_angle(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


class RelativePoseEstimator:
    """Estimate episode-relative x/y from wheel odometry and IMU yaw.

    The first valid frame is the map origin (the physical map center).  This is
    odometry, not absolute localization; validity is exposed to the policy so
    a missing geometry or IMU frame cannot silently become a false position.
    """

    def __init__(self, config, field_width_m=3.0, field_height_m=3.0):
        self.config = config
        self.field_width_m = float(field_width_m)
        self.field_height_m = float(field_height_m)
        if self.field_width_m <= 0 or self.field_height_m <= 0:
            raise ValueError('field dimensions must be positive')
        self.reset()

    def reset(self):
        self.x_m = 0.0
        self.y_m = 0.0
        self.heading_rad = 0.0
        self._prev_left = None
        self._prev_right = None
        self._prev_yaw = None
        self.boot_id = None
        self.valid = False

    def update(self, frame):
        boot_id = frame.get('boot_id')
        left = frame.get('encoder_left_count')
        right = frame.get('encoder_right_count')
        yaw = frame.get('imu_yaw')
        geometry_valid = bool(frame.get('geometry_valid', True))
        if boot_id != self.boot_id:
            self.reset()
            self.boot_id = boot_id
        if left is None or right is None or yaw is None or not geometry_valid:
            self.valid = False
            return self.snapshot()
        if self._prev_left is None:
            self._prev_left = left
            self._prev_right = right
            self._prev_yaw = yaw
            self.heading_rad = 0.0
            self.valid = True
            return self.snapshot()

        dl_counts = left - self._prev_left
        dr_counts = right - self._prev_right
        self._prev_left = left
        self._prev_right = right
        delta_yaw = _wrap_angle(yaw - self._prev_yaw)
        self._prev_yaw = yaw
        self.heading_rad = _wrap_angle(self.heading_rad + delta_yaw)
        left_m = dl_counts * math.pi * self.config['wheel_diameter_left_m'] / self.config['counts_per_revolution_left']
        right_m = dr_counts * math.pi * self.config['wheel_diameter_right_m'] / self.config['counts_per_revolution_right']
        distance_m = (left_m + right_m) / 2.0
        midpoint = self.heading_rad - delta_yaw / 2.0
        self.x_m += distance_m * math.cos(midpoint)
        self.y_m += distance_m * math.sin(midpoint)
        self.valid = True
        return self.snapshot()

    def snapshot(self):
        return {
            'valid': self.valid,
            'x_m': self.x_m,
            'y_m': self.y_m,
            'heading_rad': self.heading_rad,
            'x_norm': self.x_m / (self.field_width_m / 2.0),
            'y_norm': self.y_m / (self.field_height_m / 2.0),
            'sin_heading': math.sin(self.heading_rad),
            'cos_heading': math.cos(self.heading_rad),
            'reference': 'map_center_at_first_valid_frame',
            'source': 'wheel_odometry_plus_imu_yaw',
        }


def boundary_safe_action(action, position, map_width_m, map_height_m,
                         decision_dt_s=0.5, safety_margin_m=0.05,
                         warning_margin_m=0.30):
    """Apply warning-band slowdown and hard predicted-boundary avoidance."""
    if not position or not position.get('valid', False):
        return action, None
    action_id = ACTION_NAMES.index(action) if isinstance(action, str) else int(action)
    v, omega = VELOCITY_MAP[action_id]
    if v <= 0:
        return action, None
    heading = math.atan2(position['sin_heading'], position['cos_heading'])
    next_heading = heading + omega * decision_dt_s
    next_x = position['x_m'] + v * math.cos(next_heading) * decision_dt_s
    next_y = position['y_m'] + v * math.sin(next_heading) * decision_dt_s
    margin = min(
        map_width_m / 2.0 + next_x - safety_margin_m,
        map_width_m / 2.0 - next_x - safety_margin_m,
        map_height_m / 2.0 + next_y - safety_margin_m,
        map_height_m / 2.0 - next_y - safety_margin_m,
    )
    current_margin = min(
        map_width_m / 2.0 + position['x_m'],
        map_width_m / 2.0 - position['x_m'],
        map_height_m / 2.0 + position['y_m'],
        map_height_m / 2.0 - position['y_m'],
    )
    if current_margin < 0:
        return 'STOP', {
            'reason': 'boundary_breached',
            'requested_action': action,
            'executed_action': 'STOP',
            'margin_m': current_margin,
        }
    if margin >= warning_margin_m:
        return action, None
    target = math.atan2(-position['y_m'], -position['x_m'])
    error = _wrap_angle(target - heading)
    if margin < 0:
        reason = 'predicted_boundary_crossing'
    else:
        reason = 'boundary_warning'
    if error > 0.35:
        replacement = 'TURN_LEFT'
    elif error < -0.35:
        replacement = 'TURN_RIGHT'
    else:
        replacement = 'FORWARD_SLOW'
    return replacement, {
        'reason': reason,
        'requested_action': action,
        'executed_action': replacement,
        'margin_m': margin,
    }


def classify_gas10_scan(scan, frame, predictor, max_age_ms=5000):
    """Infer one fresh, complete scan; never claim unverified model equivalence."""
    if scan is None:
        return {'status': 'waiting_for_gas10', 'classification_valid': False,
                'model_profile_equivalence_verified': False, 'probabilities': None}
    age_ms = (frame['timestamp_ms'] - scan['scan_end_ms']) & 0xffffffff
    result = {
        'status': 'invalid_gas10', 'classification_valid': False,
        'model_profile_equivalence_verified': False, 'probabilities': None,
        'gas10_seq': scan['seq'], 'scan_id': scan['scan_id'],
        'boot_id': scan['boot_id'], 'scan_end_ms': scan['scan_end_ms'],
        'age_ms': age_ms,
    }
    if scan['boot_id'] != frame['boot_id']:
        result['reason'] = 'boot_id_mismatch'
        return result
    if not scan['acquisition_valid'] or scan['gas_valid_mask'] != 1023:
        result['reason'] = 'incomplete_or_unknown_profile'
        return result
    if age_ms > max_age_ms:
        result['status'] = 'stale_gas10'
        result['reason'] = 'scan_too_old'
        return result
    try:
        prediction = predictor.predict(scan['gas10_ohm'])
    except (ValueError, RuntimeError, OSError) as exc:
        result['status'] = 'inference_error'
        result['reason'] = f'{type(exc).__name__}: {exc}'
        return result
    result.update(prediction)
    result.update({
        'status': 'inference_unverified_profile',
        'classification_valid': False,
        'model_profile_equivalence_verified': False,
        'probability_semantics': 'softmax_logits_uncalibrated',
        'gas10_seq': scan['seq'], 'scan_id': scan['scan_id'],
        'boot_id': scan['boot_id'], 'scan_end_ms': scan['scan_end_ms'],
        'age_ms': age_ms,
    })
    return result


def build_rl_input(frame, classifier=None, history=(), gas10=None,
                   tgs_processor=None, last_action=0.0,
                   communication_quality=None, feature_count=21, position=None):
    """Preserve missing/unvalidated values and raw scan in a versioned envelope."""
    if feature_count not in (17, 18, 21, 22):
        raise ValueError('feature_count must be 17, 18, 21 or 22')
    features_order = {
        17: RL_FEATURES_17, 18: RL_FEATURES_18,
        21: RL_FEATURES_21, 22: RL_FEATURES_22,
    }[feature_count]
    bme = frame['bme688']
    classifier = classifier or {
        'status': 'waiting_for_gas10', 'probabilities': None,
        'classification_valid': False,
    }
    probabilities = classifier.get('probabilities')
    tgs_result = (tgs_processor.process_frame(frame)
                  if tgs_processor is not None else None)
    tgs_features = tgs_result.get('features') if tgs_result and tgs_result.get('valid') else None
    # A zero command sequence means the board has not received any action in
    # this boot. Its raw age is then uptime, not the age of a valid action;
    # represent the initial safety STOP as age zero for software-only policy
    # validation. Once a command exists, preserve the board-reported age.
    has_action_history = bool(frame.get('last_command_seq', 0))
    # The PL keeps the previous action_start when a STOP arrives while the
    # controller is already stopped. STOP is nevertheless the current safe
    # action, so its age must not block the next complete observation window.
    action_age_s = (frame['action_age_ms'] / 1000.0
                    if has_action_history and frame.get('last_action') != 'stop'
                    else 0.0)
    values = {
        'alcohol_prob': probabilities.get('alcohol') if probabilities else None,
        'gas_resistance_norm': bme['gas_resistance_ohm'] / 100000.0
        if bme['valid'] and bme['gas_resistance_ohm'] is not None else None,
        'temperature': (bme['temperature_c'] - 25.0) / 10.0 if bme['valid'] else None,
        'humidity': (bme['humidity_percent'] - 50.0) / 20.0 if bme['valid'] else None,
        'tgs_left_norm': tgs_features.get('tgs0_norm') if tgs_features else None,
        'tgs_right_norm': tgs_features.get('tgs1_norm') if tgs_features else None,
        'tgs_diff_norm': tgs_features.get('diff_corrected') if tgs_features else None,
        'tgs_mean': tgs_features.get('common_response') if tgs_features else None,
        'tgs_slope': tgs_features.get('common_slope') if tgs_features else None,
        'plume_enter': tgs_features.get('plume_enter') if tgs_features else None,
        'plume_stable': tgs_features.get('plume_stable') if tgs_features else None,
        'plume_lost': tgs_features.get('plume_lost') if tgs_features else None,
        'linear_velocity': frame['linear_velocity'] / 0.3 if frame['linear_velocity'] is not None else None,
        'angular_velocity': frame['angular_velocity'],
        'gyro_z': frame['imu_gyro_z'] if frame['imu_yaw'] is not None else None,
        'last_action': last_action,
        'action_age': action_age_s,
        'communication_quality': communication_quality,
    }
    position = position or {}
    for name in RL_POSITION_FEATURES:
        values[name] = position.get(name) if position.get('valid', False) else None
    valid = {key: values[key] is not None for key in features_order}
    if not classifier.get('classification_valid', False):
        valid['alcohol_prob'] = False
    history = list(history)
    history_ready = len(history) == 8 and all(all(value is not None for value in row) for row in history)
    missing = [key for key, is_valid in valid.items() if not is_valid]
    return {
        'schema': 'car_rl_observation_v4' if feature_count in (21, 22) else 'car_rl_observation_v3',
        'feature_count': feature_count,
        'source': {
            'protocol_version': frame['protocol_version'], 'boot_id': frame['boot_id'],
            'frame_seq': frame['seq'], 'board_timestamp_ms': frame['timestamp_ms'],
            'time_resolution_ms': 1,
        },
        'features_order': list(features_order), 'features': values, 'valid': valid,
        'history': {'shape': [len(history), feature_count], 'values': history,
                    'required_steps': 8, 'ready': history_ready},
        'history_ready': history_ready, 'policy_ready': not missing,
        'missing_features': missing, 'classifier': classifier, 'gas10': gas10,
        'position': position,
        'tgs2602': {
            'valid': bool(tgs_result and tgs_result.get('valid')),
            'updated': bool(tgs_result and tgs_result.get('updated')),
            'reason': tgs_result.get('reason') if tgs_result else 'processor_not_configured',
            'quality': tgs_result.get('quality') if tgs_result else 'not_configured',
            'sample_time_ms': tgs_result.get('sample_time_ms') if tgs_result else None,
            'features': tgs_features,
            'metadata': tgs_processor.metadata() if tgs_processor is not None else None,
        },
        'raw_sensors': {'bme688': bme, 'tgs2602': frame['tgs2602'],
                        'imu_yaw': frame['imu_yaw'], 'imu_gyro_z': frame['imu_gyro_z']},
    }


def policy_state(observation, history=(), fill_value=0.0):
    """Build the numeric PPO state while preserving an explicit imputation list.

    Values are retained even when their validity flag is false so an unverified
    classifier result can be traced through the black-box policy test. Missing
    fields are filled with a neutral value and reported to the caller.
    """
    rows = list(history)
    current = observation['features']
    missing = []
    current_row = []
    for name in observation['features_order']:
        value = current.get(name)
        if value is None:
            value = fill_value
            missing.append(name)
        current_row.append(float(value))
    numeric_rows = []
    for row in rows[-8:]:
        numeric_row = []
        for index, value in enumerate(row):
            if value is None:
                value = fill_value
                if observation['features_order'][index] not in missing:
                    missing.append(observation['features_order'][index])
            numeric_row.append(float(value))
        numeric_rows.append(numeric_row)
    while len(numeric_rows) < 8:
        numeric_rows.insert(0, list(current_row))
    if numeric_rows:
        numeric_rows[-1] = list(current_row)
    missing.extend(name for name in observation.get('imputed_features', ())
                   if name not in missing)
    return {
        'state': numeric_rows,
        'shape': [8, len(observation['features_order'])],
        'imputed_features': missing,
        'status': 'imputed_unverified' if missing or not observation['valid'].get('alcohol_prob', False)
                  else 'complete',
    }


def policy_gate(observation, numeric_policy, max_action_age_s=2.0):
    """Return whether a state is safe to send to the deployed PPO policy."""
    reasons = []
    if not observation.get('history_ready', False):
        reasons.append('history_not_ready')
    allowed_hold = (set(observation.get('imputed_features', ()))
                    if observation.get('tgs_hold_valid') else set())
    blocked_imputations = set(numeric_policy.get('imputed_features') or ()) - allowed_hold
    if blocked_imputations:
        reasons.append('missing_or_invalid_features')
    action_age = observation['features'].get('action_age')
    if action_age is None or action_age < 0 or action_age > max_action_age_s:
        reasons.append('stale_action_telemetry')
    return {'ready': not reasons, 'reasons': reasons}


def action_command(action, frame, config, command_seq=None):
    """Encode the seven-action API; transport is handled by the caller."""
    if isinstance(action, int) and not isinstance(action, bool) and 0 <= action < len(ACTION_NAMES):
        action = ACTION_NAMES[action]
    if action not in ACTION_NAMES:
        raise ValueError(f'unsupported RL action: {action}')
    action_id = ACTION_NAMES.index(action)
    requested_v, requested_omega = VELOCITY_MAP[action_id]
    if action_id == 0:
        mode, p0, p1, applied_scale = 0, 0, 0, 1.0
    else:
        half_track = config['track_width_m'] / 2
        left_rpm = (requested_v - requested_omega * half_track) * 60 / (math.pi * config['wheel_diameter_left_m'])
        right_rpm = (requested_v + requested_omega * half_track) * 60 / (math.pi * config['wheel_diameter_right_m'])
        peak = max(abs(left_rpm), abs(right_rpm))
        applied_scale = min(1.0, config['max_wheel_rpm'] / peak) if peak else 1.0
        p0, p1 = velocity_targets(requested_v * applied_scale, requested_omega * applied_scale, config)
        mode = 2 if action_id in (1, 2) else 1
    seq = command_seq if command_seq is not None else (frame['last_command_seq'] + 1) & 0xffffffff
    packet = command(seq, frame['boot_id'], frame['timestamp_ms'], mode, p0, p1, ttl_ms=100)
    return {
        'schema': 'car_action_command_v1', 'action': action, 'action_id': action_id,
        'requested_linear_velocity_m_s': requested_v, 'requested_angular_velocity_rad_s': requested_omega,
        'limit_scale': applied_scale, 'mode': mode, 'p0': p0, 'p1': p1,
        'seq': seq, 'boot_id': frame['boot_id'], 'issued_ms': frame['timestamp_ms'],
        'ttl_ms': 100, 'packet_hex': packet.hex(), 'transport': 'simulated_not_sent',
        'mapping_status': 'PPO_ACTION_MAP_SIMULATION_ONLY_NOT_PHYSICAL_ACTION_VALIDATION',
    }
