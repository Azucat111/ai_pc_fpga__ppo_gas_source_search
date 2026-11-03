"""Task-level search state machine and bounded coverage controller.

This module sits above PPO and below the UI.  It decides which mode is allowed;
the safety controller remains the final action arbiter.
"""

from enum import Enum
import math
import random

class TaskState(str, Enum):
    COVERAGE_SEARCH = 'coverage_search'
    PPO_TRACKING = 'ppo_tracking'
    PPO_TIMEOUT_WAIT = 'ppo_timeout_wait'
    SOURCE_VERIFY_STOP = 'source_verify_stop'
    SOURCE_CONFIRMED_WAIT = 'source_confirmed_wait'
    RETURN_HOME = 'return_home'
    PAUSED = 'paused'
    STOPPED = 'stopped'


class CoverageController:
    """Low-speed random-reflection coverage controller.

    The controller is deliberately conservative: boundary protection is still
    applied by ``boundary_safe_action`` after this controller selects an action.
    """

    def __init__(self, seed=None, turn_interval_s=3.0,
                 field_width_m=3.0, field_height_m=3.0):
        self.rng = random.Random(seed)
        self.turn_interval_s = float(turn_interval_s)
        self.field_width_m = float(field_width_m)
        self.field_height_m = float(field_height_m)
        self.next_turn_at = None
        self.turn_action = None

    def reset(self, now=0.0):
        self.next_turn_at = float(now) + self.rng.uniform(1.5, self.turn_interval_s)
        self.turn_action = None

    def action(self, position, now):
        if self.next_turn_at is None:
            self.reset(now)
        if position and position.get('valid'):
            x = position['x_m']
            y = position['y_m']
            heading = math.atan2(position['sin_heading'], position['cos_heading'])
            near_x = abs(x) >= 0.5 * self.field_width_m - 0.3
            near_y = abs(y) >= 0.5 * self.field_height_m - 0.3
            if near_x or near_y:
                inward = math.atan2(-y, -x)
                error = math.atan2(math.sin(inward - heading), math.cos(inward - heading))
                return 'TURN_LEFT' if error >= 0 else 'TURN_RIGHT'
        if now >= self.next_turn_at:
            self.turn_action = self.rng.choice(('TURN_LEFT', 'TURN_RIGHT'))
            self.next_turn_at = now + self.rng.uniform(1.5, self.turn_interval_s)
        if self.turn_action:
            action = self.turn_action
            self.turn_action = None
            return action
        return self.rng.choice(('FORWARD_SLOW', 'FORWARD')) if self.rng.random() < 0.15 else 'FORWARD'


def return_home_action(position):
    """Return-home primitive; boundary arbitration still has final authority."""
    if not position or not position.get('valid'):
        return 'STOP'
    heading = math.atan2(position['sin_heading'], position['cos_heading'])
    target = math.atan2(-position['y_m'], -position['x_m'])
    error = math.atan2(math.sin(target - heading), math.cos(target - heading))
    if abs(error) > 0.35:
        return 'TURN_LEFT' if error > 0 else 'TURN_RIGHT'
    return 'FORWARD_SLOW'


class TaskStateMachine:
    """Human-confirmed task progression above the policy and safety layers."""

    def __init__(self, ppo_timeout_s=90.0, alcohol_threshold=0.8,
                 confirmation_samples=3, tgs_trigger_duration_s=3.0,
                 tgs_max_gap_s=0.75, verify_linear_speed_m_s=0.02,
                 verify_angular_speed_rad_s=0.05, verify_stationary_s=0.5):
        self.ppo_timeout_s = float(ppo_timeout_s)
        self.alcohol_threshold = float(alcohol_threshold)
        self.confirmation_samples = int(confirmation_samples)
        self.tgs_trigger_duration_s = float(tgs_trigger_duration_s)
        self.tgs_max_gap_s = float(tgs_max_gap_s)
        self.verify_linear_speed_m_s = float(verify_linear_speed_m_s)
        self.verify_angular_speed_rad_s = float(verify_angular_speed_rad_s)
        self.verify_stationary_s = float(verify_stationary_s)
        if (self.ppo_timeout_s <= 0 or not 0 < self.alcohol_threshold <= 1 or
                self.tgs_trigger_duration_s <= 0 or self.tgs_max_gap_s <= 0 or
                self.verify_linear_speed_m_s < 0 or
                self.verify_angular_speed_rad_s < 0 or self.verify_stationary_s <= 0):
            raise ValueError('invalid task thresholds')
        if self.confirmation_samples < 1:
            raise ValueError('confirmation_samples must be positive')
        self.state = TaskState.COVERAGE_SEARCH
        self.ppo_started_at = None
        self.positive_count = 0
        self.source_count = 0
        self.verify_count = 0
        self.verify_stationary_since = None
        self.verify_ready = False
        self.verify_start_scan_seq = None
        self.trigger_armed = True
        self.tgs_trigger_started_ms = None
        self.tgs_last_sample_ms = None
        self.last_event = None

    def _event(self, name, detail=None):
        self.last_event = {'event': name, 'detail': detail}
        return self.last_event

    def update_tgs_trigger(self, features, sample_time_ms):
        """Start PPO after a continuous TGS response interval."""
        if self.state != TaskState.COVERAGE_SEARCH or not features:
            return None
        sample_time_ms = int(sample_time_ms)
        if not self.trigger_armed:
            self.tgs_last_sample_ms = sample_time_ms
            self.tgs_trigger_started_ms = None
            if features.get('trigger_release', False):
                self.trigger_armed = True
                return self._event('tgs_trigger_rearmed')
            return None
        if (self.tgs_last_sample_ms is not None and
                sample_time_ms - self.tgs_last_sample_ms > self.tgs_max_gap_s * 1000):
            self.tgs_trigger_started_ms = None
        self.tgs_last_sample_ms = sample_time_ms
        if self.tgs_trigger_started_ms is None:
            if not features.get('trigger_ready', False):
                return None
            self.tgs_trigger_started_ms = sample_time_ms
        elif features.get('trigger_release', False):
            self.tgs_trigger_started_ms = None
            return None
        duration_s = (sample_time_ms - self.tgs_trigger_started_ms) / 1000.0
        if duration_s >= self.tgs_trigger_duration_s:
            self.state = TaskState.PPO_TRACKING
            self.ppo_started_at = None
            self.trigger_armed = False
            return self._event('tgs_triggered_ppo', {
                'duration_s': duration_s,
                'sample_time_ms': sample_time_ms,
                'max_abs_z': features.get('max_abs_z'),
                'common_z': features.get('common_z'),
            })
        return None

    def update_detection(self, classifier, gas10=None):
        """Legacy 688 trigger is retained only for compatibility, not used."""
        valid = bool(classifier and classifier.get('classification_valid'))
        probability = ((classifier or {}).get('probabilities') or {}).get('alcohol')
        complete = bool(gas10 and gas10.get('acquisition_valid') and
                        gas10.get('gas_valid_mask') == 1023)
        positive = valid and complete and probability is not None and probability > self.alcohol_threshold
        if positive:
            self.positive_count += 1
        else:
            self.positive_count = 0
        return None

    def update_source_candidate(self, classifier, tgs_valid, tgs_features, gas10=None):
        """Require repeated class and plume evidence before stopping for retest."""
        valid = bool(classifier and classifier.get('classification_valid'))
        probability = ((classifier or {}).get('probabilities') or {}).get('alcohol')
        complete = bool(gas10 and gas10.get('acquisition_valid') and
                        gas10.get('gas_valid_mask') == 1023)
        plume_stable = bool(tgs_features and tgs_features.get('plume_stable'))
        evidence = (valid and complete and tgs_valid and probability is not None and
                    probability > self.alcohol_threshold and plume_stable)
        if self.state != TaskState.PPO_TRACKING:
            self.source_count = 0
            return False
        self.source_count = self.source_count + 1 if evidence else 0
        return self.source_count >= self.confirmation_samples

    def begin_source_verification(self, scan_seq):
        if self.state != TaskState.PPO_TRACKING:
            return None
        self.state = TaskState.SOURCE_VERIFY_STOP
        self.verify_count = 0
        self.verify_stationary_since = None
        self.verify_ready = False
        self.verify_start_scan_seq = int(scan_seq)
        self.source_count = 0
        return self._event('source_candidate_stop_for_retest', {
            'candidate_scan_seq': int(scan_seq),
        })

    def observe_verification_motion(self, frame, now, scan_seq=None):
        if self.state != TaskState.SOURCE_VERIFY_STOP:
            return False
        linear = frame.get('linear_velocity')
        angular = frame.get('angular_velocity')
        stopped = (linear is not None and angular is not None and
                   abs(float(linear)) <= self.verify_linear_speed_m_s and
                   abs(float(angular)) <= self.verify_angular_speed_rad_s)
        if not stopped:
            self.verify_stationary_since = None
            self.verify_ready = False
            return False
        if self.verify_stationary_since is None:
            self.verify_stationary_since = float(now)
        if now - self.verify_stationary_since >= self.verify_stationary_s:
            if not self.verify_ready and scan_seq is not None:
                self.verify_start_scan_seq = int(scan_seq)
            self.verify_ready = True
        self._now = float(now)
        return self.verify_ready

    def update_stationary_verification(self, classifier, tgs_valid,
                                       tgs_features, gas10=None, scan_seq=None):
        """Confirm at rest; reject a valid negative retest by resuming PPO."""
        if self.state != TaskState.SOURCE_VERIFY_STOP or not self.verify_ready:
            return None
        if scan_seq is None or scan_seq == self.verify_start_scan_seq:
            return None
        valid_scan = bool(gas10 and gas10.get('acquisition_valid') and
                          gas10.get('gas_valid_mask') == 1023 and
                          classifier and classifier.get('classification_valid'))
        if not valid_scan or not tgs_valid:
            return self._resume_ppo_after_retest('invalid_or_stale_retest_input')
        probability = ((classifier.get('probabilities') or {}).get('alcohol'))
        plume_stable = bool(tgs_features and tgs_features.get('plume_stable'))
        if probability is None or probability <= self.alcohol_threshold or not plume_stable:
            return self._resume_ppo_after_retest('stationary_retest_not_confirmed')
        self.verify_count += 1
        self.verify_start_scan_seq = int(scan_seq)
        if self.verify_count >= self.confirmation_samples:
            self.state = TaskState.SOURCE_CONFIRMED_WAIT
            self.verify_count = 0
            self.verify_ready = False
            return self._event('source_confirmed_after_stationary_retest')
        return self._event('stationary_retest_scan_accepted', {
            'accepted': self.verify_count,
            'required': self.confirmation_samples,
        })

    def _resume_ppo_after_retest(self, reason):
        self.state = TaskState.PPO_TRACKING
        self.verify_count = 0
        self.verify_stationary_since = None
        self.verify_ready = False
        self.verify_start_scan_seq = None
        self.source_count = 0
        return self._event('stationary_retest_resume_ppo', {'reason': reason})

    def start_ppo(self, now):
        if self.state == TaskState.PPO_TRACKING and self.ppo_started_at is None:
            self.ppo_started_at = float(now)
            return self._event('ppo_started')
        return None

    def update_ppo(self, now, source_confirmed=False):
        self._now = float(now)
        if self.state not in (TaskState.PPO_TRACKING, TaskState.SOURCE_VERIFY_STOP):
            return None
        if self.ppo_started_at is not None and now - self.ppo_started_at >= self.ppo_timeout_s:
            self.state = TaskState.PPO_TIMEOUT_WAIT
            self.verify_ready = False
            return self._event('ppo_timeout', {'timeout_s': self.ppo_timeout_s})
        return None

    def acknowledge_timeout(self):
        if self.state != TaskState.PPO_TIMEOUT_WAIT:
            return False
        self.state = TaskState.COVERAGE_SEARCH
        self.positive_count = 0
        self.ppo_started_at = None
        self._event('timeout_acknowledged_resume_coverage')
        return True

    def acknowledge_source_processed(self, return_home):
        if self.state != TaskState.SOURCE_CONFIRMED_WAIT:
            return False
        self.state = TaskState.RETURN_HOME if return_home else TaskState.COVERAGE_SEARCH
        self.source_count = 0
        self.verify_count = 0
        self.ppo_started_at = None
        self.tgs_trigger_started_ms = None
        self.trigger_armed = False
        self._event('source_processed_return_home' if return_home
                    else 'source_processed_continue_here')
        return True

    def at_home(self, position, tolerance_m=0.15):
        if self.state != TaskState.RETURN_HOME or not position or not position.get('valid'):
            return False
        return math.hypot(position['x_m'], position['y_m']) <= tolerance_m

    def complete_return_home(self, position, tolerance_m=0.15):
        if not self.at_home(position, tolerance_m):
            return False
        self.state = TaskState.COVERAGE_SEARCH
        self.positive_count = 0
        self.ppo_started_at = None
        self.tgs_trigger_started_ms = None
        self.trigger_armed = False
        self._event('returned_home_resume_coverage')
        return True

    def pause(self):
        if self.state in (TaskState.PAUSED, TaskState.STOPPED):
            return False
        self._resume_state = self.state
        self.state = TaskState.PAUSED
        self._event('task_paused')
        return True

    def resume(self):
        if self.state != TaskState.PAUSED:
            return False
        self.state = getattr(self, '_resume_state', TaskState.COVERAGE_SEARCH)
        self._event('task_resumed')
        return True

    def stop(self):
        if self.state == TaskState.STOPPED:
            return False
        self.state = TaskState.STOPPED
        self._event('task_stopped')
        return True

    def status(self):
        elapsed = None
        if self.ppo_started_at is not None:
            elapsed = max(0.0, self._now - self.ppo_started_at) if hasattr(self, '_now') else None
        return {'state': self.state.value, 'ppo_elapsed_s': elapsed,
                'positive_count': self.positive_count,
                'source_count': self.source_count,
                'verify_count': self.verify_count,
                'verify_required': self.confirmation_samples,
                'verify_ready': self.verify_ready,
                'verify_stationary_elapsed_s': (
                    max(0.0, self._now - self.verify_stationary_since)
                    if self.verify_stationary_since is not None and hasattr(self, '_now')
                    else 0.0),
                'tgs_trigger_armed': self.trigger_armed,
                'tgs_trigger_started_ms': self.tgs_trigger_started_ms,
                'tgs_last_sample_ms': self.tgs_last_sample_ms,
                'last_event': self.last_event}

    def tick(self, now):
        self._now = float(now)
        return self.update_ppo(now)

