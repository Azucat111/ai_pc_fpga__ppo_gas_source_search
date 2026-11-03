#!/usr/bin/env python3
"""Read car data, classify GAS10 scans, and run simulated or live RL I/O."""
import argparse
from collections import deque
from datetime import datetime, timezone
import json
from pathlib import Path
import sys
import struct
import time

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'app' / 'software'))
from car.config import load_config
from car.host import Client
from car.protocol import signed32
from car.workflow import (ACTION_NAMES, RelativePoseEstimator, action_command,
                          apply_short_tgs_hold, boundary_safe_action, build_rl_input,
                          classify_gas10_scan, policy_gate, policy_state)
from car.ppo_policy import PPOPolicy
from car.safety import SafetyController, SafetyState
from car.source_event import QUERY, SOURCE_CLEARED, SOURCE_CONFIRMED, SourceIndicator
from car.task_state import (CoverageController, TaskState, TaskStateMachine,
                            return_home_action)
from car.tgs2602 import TGS2602Preprocessor


def _read_control_command(path, last_seq):
    """Read one operator command from the GUI control file."""
    if not path or not path.exists():
        return None, last_seq
    try:
        item = json.loads(path.read_text(encoding='utf-8'))
        seq = int(item.get('seq', 0))
    except (OSError, ValueError, TypeError, json.JSONDecodeError):
        return None, last_seq
    if seq <= last_seq:
        return None, last_seq
    return str(item.get('command', '')).strip().lower(), seq


def _write_status(path, payload):
    if not path:
        return
    temporary = path.with_suffix(path.suffix + '.tmp')
    temporary.write_text(json.dumps(payload, ensure_ascii=False, allow_nan=False) + '\n',
                         encoding='utf-8')
    temporary.replace(path)


def _finalize_status(status, stop_result):
    if not status:
        return status
    status['shutdown_stop'] = stop_result
    if stop_result and stop_result.get('transport') == 'udp_acknowledged':
        status['action'] = {
            'action': 'STOP', 'action_id': 0,
            'source': 'workflow_shutdown',
        }
        status['frame'] = stop_result.get('stationary_frame') or stop_result['acknowledged_frame']
    status['finished'] = True
    return status


def _fresh_tgs_confirmation(frame, tgs_observation, max_age_ms):
    samples = frame.get('tgs2602') or []
    if not tgs_observation or not tgs_observation.get('valid') or len(samples) < 2:
        return False
    return all(sample.get('valid') and not sample.get('saturated', False) and
               sample.get('age_ms') is not None and
               0 <= int(sample['age_ms']) <= max_age_ms
               for sample in samples[:2])


class PhysicalActionSender:
    """Send the seven-action mapping through the live client lease."""

    def __init__(self, client, stationary_linear_m_s=0.02,
                 stationary_angular_rad_s=0.05, stationary_dwell_s=0.5,
                 stop_verify_timeout_s=3.0):
        self.client = client
        self.active_seq = None
        self.pending_action_seq = None
        self.last_tx = 0.0
        self.pending = {}
        self.pending_encoded = None
        self.stationary_linear_m_s = float(stationary_linear_m_s)
        self.stationary_angular_rad_s = float(stationary_angular_rad_s)
        self.stationary_dwell_s = float(stationary_dwell_s)
        self.stop_verify_timeout_s = float(stop_verify_timeout_s)

    def _transmit(self, mode, p0=0, p1=0, heartbeat_for=None):
        seq, boot, packet = self.client._make_packet(
            mode, p0, p1, refresh=False)
        self.client.sock.sendto(packet, self.client.address)
        now = time.monotonic()
        self.last_tx = now
        self.pending[seq] = {
            'sent_at': now,
            'last_sent_at': now,
            'packet': packet,
            'heartbeat_for': heartbeat_for,
        }
        return seq, boot, packet

    def send_action(self, encoded):
        seq, boot, packet = self.client._make_packet(
            encoded['mode'], encoded['p0'], encoded['p1'], refresh=False)
        now = time.monotonic()
        self.last_tx = now
        self.pending[seq] = {'sent_at': now, 'last_sent_at': now,
                             'packet': packet, 'heartbeat_for': None,
                             'action_id': encoded['action_id']}
        deadline = time.monotonic() + 1.0
        last_tx = -1.0
        acknowledged = None
        while time.monotonic() < deadline:
            now = time.monotonic()
            if now - last_tx >= 0.1:
                self.client.sock.sendto(packet, self.client.address)
                last_tx = now
            try:
                frame = self.client.receive(timeout=min(0.1, deadline - now))
            except TimeoutError:
                continue
            if frame.get('boot_id') != boot:
                raise RuntimeError('FPGA restarted; motion has been cancelled')
            if frame.get('last_command_seq') == seq:
                if encoded['action_id'] != 0 and frame.get('last_action') == 'stop':
                    raise RuntimeError(
                        f"FPGA stopped action seq={seq}: {frame.get('stop_reason')}"
                    )
                acknowledged = frame
                break
            if (frame.get('last_checked_command_seq') == seq and
                    frame.get('command_status')):
                raise RuntimeError(
                    f"FPGA rejected command seq={seq}, "
                    f"status={frame.get('command_status')}"
                )
        if acknowledged is None:
            raise TimeoutError(f'FPGA action acknowledgement timeout for seq={seq}')
        self.pending.pop(seq, None)
        self.pending_action_seq = None
        self.pending_encoded = None
        self.active_seq = None if encoded['action_id'] == 0 else seq
        result = dict(encoded)
        result.update({
            'seq': seq,
            'boot_id': boot,
            'issued_ms': struct.unpack_from('<I', packet, 12)[0],
            'ttl_ms': 500,
            'packet_hex': packet.hex(),
            'transport': 'udp_sent',
            'mapping_status': 'PPO_ACTION_MAP_PHYSICAL_MODE1_MODE2',
            'ack_pending_seq': None,
            'acknowledged': True,
        })
        return result

    def maintain(self, frame):
        now = time.monotonic()
        checked = frame.get('last_checked_command_seq')
        if checked in self.pending and frame.get('command_status'):
            pending = self.pending[checked]
            stale_heartbeat = (
                frame['command_status'] == 5 and
                pending['heartbeat_for'] is not None and
                self.active_seq is not None and
                signed32(self.active_seq - pending['heartbeat_for']) > 0
            )
            if stale_heartbeat:
                del self.pending[checked]
            else:
                raise RuntimeError(
                    f"FPGA rejected command seq={checked}, "
                    f"status={frame['command_status']}, "
                    f"heartbeat_for={pending['heartbeat_for']}, "
                    f"board_action={frame.get('action_id')}, "
                    f"last_action={frame.get('last_action')}, "
                    f"last_command_seq={frame.get('last_command_seq')}"
                )
        last_seq = frame.get('last_command_seq')
        if self.pending_action_seq is not None:
            action_seq = self.pending_action_seq
            pending_action = self.pending[action_seq]
            acknowledged = (last_seq is not None and
                            signed32(last_seq - action_seq) >= 0)
            if acknowledged:
                expected_stop = pending_action['action_id'] == 0
                action_matches = (
                    frame.get('last_action') == 'stop' if expected_stop else
                    frame.get('action_id') == action_seq and
                    frame.get('last_action') != 'stop'
                )
                if not action_matches:
                    raise RuntimeError(
                        f"FPGA acknowledged action seq={action_seq} but did not "
                        f"keep it active; board_action={frame.get('action_id')}, "
                        f"last_action={frame.get('last_action')}, "
                        f"stop_reason={frame.get('stop_reason')}"
                    )
                self.active_seq = None if expected_stop else action_seq
                self.pending_action_seq = None
                self.pending_encoded = None
            elif now - pending_action['sent_at'] >= 3.0:
                raise TimeoutError(
                    f"FPGA action acknowledgement timeout for seq={action_seq}; "
                    f"last_command_seq={last_seq}, "
                    f"last_checked_command_seq={checked}, "
                    f"command_status={frame.get('command_status')}"
                )
            elif now - pending_action['last_sent_at'] >= 0.1:
                self.client.sock.sendto(pending_action['packet'], self.client.address)
                pending_action['last_sent_at'] = now
                self.last_tx = now
            if acknowledged and last_seq is not None:
                for seq in list(self.pending):
                    if signed32(last_seq - seq) >= 0:
                        del self.pending[seq]
        if last_seq is not None:
            for seq in list(self.pending):
                if seq == self.pending_action_seq:
                    continue
                if signed32(last_seq - seq) >= 0:
                    del self.pending[seq]
        if self.active_seq is not None and now - self.last_tx >= 0.1:
            self._transmit(4, signed32(self.active_seq), 0,
                           heartbeat_for=self.active_seq)

    def stop(self):
        if self.client.latest is None:
            return None
        frame = self.client.send(0)
        self.active_seq = None
        self.pending_action_seq = None
        self.pending_encoded = None
        acknowledged_frame = frame
        stationary_since = None
        deadline = time.monotonic() + self.stop_verify_timeout_s
        stationary_confirmed = False
        while True:
            now = time.monotonic()
            linear = frame.get('linear_velocity')
            angular = frame.get('angular_velocity')
            stationary = (
                frame.get('last_action') == 'stop' and
                frame.get('action_id') == 0 and
                linear is not None and angular is not None and
                abs(float(linear)) <= self.stationary_linear_m_s and
                abs(float(angular)) <= self.stationary_angular_rad_s
            )
            if stationary:
                if stationary_since is None:
                    stationary_since = now
                if now - stationary_since >= self.stationary_dwell_s:
                    stationary_confirmed = True
                    break
            else:
                stationary_since = None
            remaining = deadline - now
            if remaining <= 0:
                break
            try:
                frame = self.client.receive(timeout=min(0.1, remaining))
            except TimeoutError:
                continue
            if frame.get('boot_id') != acknowledged_frame.get('boot_id'):
                break
        stationary_duration_s = (
            max(0.0, time.monotonic() - stationary_since)
            if stationary_since is not None else 0.0
        )
        return {
            'action': 'STOP', 'action_id': 0,
            'seq': acknowledged_frame['last_command_seq'],
            'boot_id': acknowledged_frame['boot_id'],
            'transport': 'udp_acknowledged',
            'acknowledged_last_action': acknowledged_frame['last_action'],
            'acknowledged_stop_reason': acknowledged_frame['stop_reason'],
            'acknowledged_linear_velocity_m_s': acknowledged_frame['linear_velocity'],
            'acknowledged_angular_velocity_rad_s': acknowledged_frame['angular_velocity'],
            'acknowledged_frame': acknowledged_frame,
            'stationary_confirmed': stationary_confirmed,
            'stationary_duration_s': stationary_duration_s,
            'stationary_frame': frame,
        }


def run(args):
    config = load_config(args.config)
    predictor = None
    if not args.no_classifier:
        bme_root = ROOT.parent / 'bme688' / 'training'
        sys.path.insert(0, str(bme_root))
        try:
            from predict_air_alcohol import AirAlcoholPredictor
            predictor = AirAlcoholPredictor(args.model)
        except (ImportError, OSError, RuntimeError, ValueError) as exc:
            raise RuntimeError(f'Cannot load BME688 classifier: {exc}') from exc
    ppo = None
    if args.enable_rl:
        dqn_root = ROOT.parent / 'ppo'
        try:
            ppo = PPOPolicy(args.ppo_model, dqn_root, feature_count=args.feature_count)
        except (ImportError, OSError, RuntimeError, ValueError) as exc:
            raise RuntimeError(f'Cannot load PPO policy: {exc}') from exc

    output = Path(args.output)
    if output.exists():
        if not (args.control_file and args.status_file):
            raise FileExistsError(output)
    else:
        output.mkdir(parents=True, exist_ok=False)
    client = Client(args.ip, config, args.port)
    source_indicator = SourceIndicator(client)
    source_led = {
        'supported': None, 'status': '等待车端能力查询',
        'board_confirmed': None, 'last_ack': None, 'error': None,
    }
    source_led_flow = {
        'boot_id': None, 'query_needed': True, 'retry_after': 0.0,
        'pending_event': None, 'target_event': None,
    }
    sender = PhysicalActionSender(
        client,
        stationary_linear_m_s=args.verify_linear_speed_m_s,
        stationary_angular_rad_s=args.verify_angular_speed_rad_s,
        stationary_dwell_s=args.verify_stationary_s,
    ) if args.send_actions else None
    safety = SafetyController(
        require_position=args.feature_count in (21, 22),
        telemetry_timeout_s=args.telemetry_timeout_ms / 1000.0,
    )
    pose_estimator = RelativePoseEstimator(
        config, field_width_m=args.map_width, field_height_m=args.map_height)
    tgs_processor = TGS2602Preprocessor(
        baseline_tgs0=args.tgs_baseline_left,
        baseline_tgs1=args.tgs_baseline_right,
        window_size=args.tgs_window_samples,
        response_scale=args.tgs_response_scale,
        noise_response=args.tgs_noise_response,
        trigger_enter_z=args.tgs_enter_z,
        trigger_exit_z=args.tgs_exit_z,
        trigger_common_z=args.tgs_common_z,
        trigger_max_gap_s=args.tgs_max_gap_s,
    )
    task = TaskStateMachine(
        ppo_timeout_s=args.ppo_timeout_s,
        alcohol_threshold=args.alcohol_threshold,
        confirmation_samples=args.detection_samples,
        tgs_trigger_duration_s=args.tgs_trigger_duration_s,
        tgs_max_gap_s=args.tgs_max_gap_s,
        verify_linear_speed_m_s=args.verify_linear_speed_m_s,
        verify_angular_speed_rad_s=args.verify_angular_speed_rad_s,
        verify_stationary_s=args.verify_stationary_s,
    ) if args.task_state_machine else None
    coverage = CoverageController(
        seed=args.coverage_seed,
        field_width_m=args.map_width,
        field_height_m=args.map_height,
    ) if task else None
    deadline = time.monotonic() + args.seconds
    next_decision = 0.0
    history = deque(maxlen=8)
    tgs_hold_cache = {}
    classified_scan_seq = None
    source_scan_seq = None
    latest_classifier = None
    last_action_id = 0
    control_seq = 0
    last_status = {}
    previous_frame_seq = None
    received_frames = 0
    estimated_missing_frames = 0
    records = 0
    try:
        with ((output / 'workflow.jsonl').open('w', encoding='utf-8') as log,
              (output / 'safety_events.jsonl').open('w', encoding='utf-8') as safety_log,
              (output / 'source_led_events.jsonl').open('w', encoding='utf-8') as led_log):
            def safety_event(event, detail=None):
                item = {
                    'received_at_utc': datetime.now(timezone.utc).isoformat(),
                    'event': event,
                    'detail': detail,
                    'status': safety.status(),
                }
                safety_log.write(json.dumps(item, ensure_ascii=False) + '\n')
                safety_log.flush()
                print(json.dumps({'safety_event': item}, ensure_ascii=False), flush=True)

            def source_led_event(event, detail=None):
                item = {
                    'received_at_utc': datetime.now(timezone.utc).isoformat(),
                    'event': event, 'detail': detail,
                }
                led_log.write(json.dumps(item, ensure_ascii=False) + '\n')
                led_log.flush()
                print(json.dumps({'source_led_event': item}, ensure_ascii=False), flush=True)

            def source_led_snapshot():
                return dict(source_led)

            def service_source_led(frame, task_state=None):
                now = time.monotonic()
                boot_id = frame.get('boot_id')
                if source_led_flow['boot_id'] != boot_id:
                    if source_led_flow['boot_id'] is not None:
                        source_indicator.pending = None
                        source_indicator.result = None
                        client.latest_source_ack = None
                        source_led.update(board_confirmed=None, last_ack=None,
                                          status='板端重启，等待重新查询', error=None)
                        source_led_flow.update(query_needed=True, pending_event=None,
                                               retry_after=now)
                        source_led_event('board_rebooted', {'boot_id': boot_id})
                    source_led_flow['boot_id'] = boot_id

                supported = bool(frame.get('source_led_supported'))
                source_led['supported'] = supported
                if not supported:
                    source_led.update(status='车端固件不支持 LD5 事件', error=None)
                    return

                ack = None
                if source_indicator.pending is not None:
                    try:
                        ack = source_indicator.poll()
                    except (OSError, RuntimeError, TimeoutError, ValueError) as exc:
                        source_led.update(status='板端状态未知，等待查询', error=str(exc))
                        source_led_flow.update(query_needed=True, pending_event=None,
                                               retry_after=now + 1.0)
                        source_led_event('event_failed', {'error': str(exc)})
                    else:
                        if ack is not None:
                            source_led.update(board_confirmed=ack['confirmed'],
                                              last_ack=ack, error=None)
                            event = source_led_flow['pending_event']
                            source_led_flow['pending_event'] = None
                            if event == QUERY:
                                source_led['status'] = (
                                    '板端确认闪烁状态' if ack['confirmed'] else '板端确认关闭状态')
                                if (task_state == TaskState.SOURCE_CONFIRMED_WAIT.value and
                                        not ack['confirmed']):
                                    source_led_flow['target_event'] = SOURCE_CONFIRMED
                            elif event == SOURCE_CONFIRMED:
                                source_led['status'] = '板端已确认闪烁（2 Hz）'
                                if source_led_flow['target_event'] == SOURCE_CONFIRMED:
                                    source_led_flow['target_event'] = None
                            elif event == SOURCE_CLEARED:
                                source_led['status'] = '板端已确认关闭'
                                if source_led_flow['target_event'] == SOURCE_CLEARED:
                                    source_led_flow['target_event'] = None
                            source_led_event('acknowledged', ack)

                if source_indicator.pending is not None or now < source_led_flow['retry_after']:
                    return
                event = QUERY if source_led_flow['query_needed'] else source_led_flow['target_event']
                if event is None:
                    return
                try:
                    source_indicator.request(event)
                except (OSError, RuntimeError, ValueError) as exc:
                    source_led.update(status='等待新鲜遥测后查询/下发', error=str(exc))
                    source_led_flow['retry_after'] = now + 0.2
                    return
                source_led_flow['query_needed'] = False
                source_led_flow['pending_event'] = event
                source_led['status'] = '等待板端查询应答' if event == QUERY else '等待板端事件应答'
                source_led['error'] = None
                source_led_event('request_sent', {
                    'event': event, 'sequence': source_indicator.sequence,
                    'boot_id': source_indicator.boot,
                    'packet_hex': source_indicator.pending.hex(),
                })

            while time.monotonic() < deadline:
                remaining = deadline - time.monotonic()
                try:
                    frame = client.receive(timeout=min(1.0, max(0.1, remaining)))
                except TimeoutError:
                    if time.monotonic() < deadline:
                        safety.latch('communication_timeout', 'telemetry receive timeout')
                        safety_event('safety_latched', safety.status())
                        if sender:
                            sender.stop()
                        break
                    break
                received_frames += 1
                if previous_frame_seq is not None:
                    gap = (frame['seq'] - previous_frame_seq - 1) & 0xffffffff
                    if gap < 0x80000000:
                        estimated_missing_frames += gap
                previous_frame_seq = frame['seq']
                service_source_led(frame, task.state.value if task else None)
                safety.observe_frame(frame)
                position = pose_estimator.update(frame)
                safety.observe_position(position)
                command_name, control_seq = _read_control_command(
                    args.control_file, control_seq)
                if command_name:
                    if command_name == 'pause' and task:
                        task.pause()
                        safety.request_pause()
                    elif command_name == 'resume' and task:
                        task.resume()
                        safety.resume()
                    elif command_name == 'stop':
                        if task:
                            task.stop()
                        safety.request_stop()
                    elif command_name == 'confirm_timeout' and task:
                        task.acknowledge_timeout()
                        safety.resume()
                    elif command_name == 'source_processed_return_home' and task:
                        if task.acknowledge_source_processed(return_home=True):
                            source_led_flow['target_event'] = SOURCE_CLEARED
                            if coverage:
                                coverage.reset(time.monotonic())
                    elif command_name == 'source_processed_continue' and task:
                        if task.acknowledge_source_processed(return_home=False):
                            source_led_flow['target_event'] = SOURCE_CLEARED
                            if coverage:
                                coverage.reset(time.monotonic())
                    elif command_name == 'reset_fault':
                        safety.reset_fault(frame=frame, position=position)
                    safety_event('operator_command', {'command': command_name})
                    _write_status(args.status_file, {
                        'schema': 'car_workflow_status_v1',
                        'updated_utc': datetime.now(timezone.utc).isoformat(),
                        'task': task.status() if task else None,
                        'safety': {'motion_allowed': safety.action_allowed(),
                                   'status': safety.status()},
                        'action': {'action': 'STOP', 'source': 'operator_command'},
                        'position': position,
                        'observation': {}, 'downlink': None,
                        'source_led': source_led_snapshot(),
                        'received_frames': received_frames, 'records': records,
                    })
                    service_source_led(frame, task.state.value if task else None)
                if not safety.action_allowed():
                    if safety.consume_stop_request():
                        safety_event('safety_stop_requested', safety.status())
                        if sender:
                            sender.stop()
                    if safety.state in (SafetyState.LATCHED_FAULT,
                                        SafetyState.STOPPED):
                        break
                    # Intentional pause remains connected so the GUI can resume.
                    time.sleep(0.02)
                    continue
                if sender:
                    sender.maintain(frame)
                now = time.monotonic()
                scan = client.latest_gas10
                if predictor and scan and scan['seq'] != classified_scan_seq:
                    latest_classifier = classify_gas10_scan(
                        scan, frame, predictor, max_age_ms=args.max_gas10_age_ms
                    )
                    classified_scan_seq = scan['seq']
                if now < next_decision:
                    continue

                classifier = dict(latest_classifier) if latest_classifier else {
                    'status': 'waiting_for_gas10', 'classification_valid': False,
                    'model_profile_equivalence_verified': False, 'probabilities': None,
                }
                if scan and classifier.get('gas10_seq') == scan['seq']:
                    classifier['age_ms'] = (frame['timestamp_ms'] - scan['scan_end_ms']) & 0xffffffff
                    if classifier['age_ms'] > args.max_gas10_age_ms:
                        classifier['status'] = 'stale_gas10'
                        classifier['classification_valid'] = False
                task_event = None
                observation = build_rl_input(
                    frame, classifier=classifier, gas10=dict(scan) if scan else None,
                    tgs_processor=tgs_processor,
                    last_action=last_action_id / 6.0,
                    communication_quality=(received_frames /
                                           max(1, received_frames + estimated_missing_frames)),
                    feature_count=args.feature_count, position=position)
                apply_short_tgs_hold(
                    observation, frame, tgs_hold_cache,
                    int(args.tgs_invalid_grace_s * 1000))
                tgs_trigger_observation = observation['tgs2602']
                history.append([
                    observation['features'][name] if observation['valid'][name] else None
                    for name in observation['features_order']
                ])
                observation = build_rl_input(
                    frame, classifier=classifier, history=history,
                    gas10=dict(scan) if scan else None, tgs_processor=tgs_processor,
                    last_action=last_action_id / 6.0,
                    communication_quality=(received_frames /
                                           max(1, received_frames + estimated_missing_frames)),
                    feature_count=args.feature_count,
                    position=position,
                )
                apply_short_tgs_hold(
                    observation, frame, tgs_hold_cache,
                    int(args.tgs_invalid_grace_s * 1000))
                if task and tgs_trigger_observation.get('updated'):
                    features = tgs_trigger_observation.get('features')
                    task_event = task.update_tgs_trigger(
                        features, tgs_trigger_observation.get('sample_time_ms'))
                    if task.state == TaskState.PPO_TRACKING:
                        task_event = task_event or task.start_ppo(now)
                numeric_policy = policy_state(observation, history=history)
                gate = policy_gate(observation, numeric_policy)
                if task:
                    new_scan = scan and scan.get('seq') != source_scan_seq
                    if new_scan:
                        source_scan_seq = scan.get('seq')
                    tgs_confirmation_valid = _fresh_tgs_confirmation(
                        frame, observation['tgs2602'], args.tgs_max_gap_s * 1000)
                    if (new_scan and task.state == TaskState.PPO_TRACKING and
                            task.update_source_candidate(
                                classifier, tgs_confirmation_valid,
                                observation['tgs2602'].get('features'), scan)):
                        task_event = task.begin_source_verification(scan['seq'])
                    elif new_scan and task.state == TaskState.SOURCE_VERIFY_STOP:
                        task_event = task.update_stationary_verification(
                            classifier, tgs_confirmation_valid,
                            observation['tgs2602'].get('features'), scan, scan['seq'])
                    if task.state == TaskState.SOURCE_VERIFY_STOP:
                        task.observe_verification_motion(frame, now, scan.get('seq') if scan else None)
                    task_event = task.update_ppo(now) or task_event
                    if task_event:
                        safety_event('task_' + task_event['event'], task_event)
                        if task_event['event'] == 'source_confirmed_after_stationary_retest':
                            source_led_flow['target_event'] = SOURCE_CONFIRMED
                            service_source_led(frame, task.state.value)
                policy_output = None
                if task and task.state == TaskState.COVERAGE_SEARCH:
                    action = coverage.action(position, now)
                    decision_source = 'coverage_search'
                elif task and task.state == TaskState.RETURN_HOME:
                    if task.complete_return_home(position):
                        coverage.reset(now)
                        action = coverage.action(position, now)
                        decision_source = 'coverage_search_after_return_home'
                    else:
                        action = return_home_action(position)
                        decision_source = 'return_home_controller'
                elif task and task.state in (TaskState.PPO_TIMEOUT_WAIT,
                                             TaskState.SOURCE_VERIFY_STOP,
                                             TaskState.SOURCE_CONFIRMED_WAIT,
                                             TaskState.PAUSED, TaskState.STOPPED):
                    action = 'STOP'
                    decision_source = 'task_waiting_for_human'
                elif ppo and gate['ready']:
                    policy_output = ppo.predict(numeric_policy['state'])
                    action = ACTION_NAMES[policy_output['action_id']]
                    decision_source = 'ppo_policy'
                elif ppo:
                    action = 'STOP'
                    decision_source = 'policy_blocked_missing_input'
                else:
                    action = args.action
                    decision_source = 'fixed_io_test_stub_not_rl_policy'
                requested_action = action
                safety_override = None
                if args.feature_count in (21, 22):
                    action, safety_override = boundary_safe_action(
                        action, position, args.map_width, args.map_height,
                        decision_dt_s=1.0 / args.decision_hz,
                    )
                    if (safety_override and
                            safety_override.get('reason') == 'boundary_breached'):
                        safety.latch('boundary_breached', safety_override)
                last_action_id = ACTION_NAMES.index(action)
                downlink = action_command(action, frame, config)
                if sender:
                    downlink = sender.send_action(downlink)
                record = {
                    'received_at_utc': datetime.now(timezone.utc).isoformat(),
                    'decision_source': decision_source,
                    'observation': observation,
                    'policy_input': numeric_policy,
                    'policy_output': policy_output,
                    'action_output': {'action': action, 'action_id': ACTION_NAMES.index(action),
                                      'source': decision_source,
                                      'requested_action': requested_action,
                                      'safety_override': safety_override},
                    'safety': {
                        'motion_allowed': bool(safety.action_allowed() and (not ppo or gate['ready'])),
                        'blocked_reason': (None if safety.action_allowed() and (not ppo or gate['ready'])
                                           else (gate['reasons'] if ppo and not gate['ready']
                                                 else ['safety_state_' + safety.state.value])),
                        'status': safety.status(),
                    },
                    'task': task.status() if task else None,
                    'downlink': downlink,
                }
                last_status = {
                    'schema': 'car_workflow_status_v1',
                    'updated_utc': datetime.now(timezone.utc).isoformat(),
                    'task': record['task'],
                    'safety': record['safety'],
                    'action': record['action_output'],
                    'position': position,
                    'frame': frame,
                    'observation': {
                        'tgs2602': observation.get('tgs2602'),
                        'classifier': observation.get('classifier'),
                        'gas10': observation.get('gas10'),
                        'imputed_features': observation.get('imputed_features', []),
                        'tgs_hold_valid': observation.get('tgs_hold_valid', False),
                    },
                    'downlink': downlink,
                    'source_led': source_led_snapshot(),
                    'received_frames': received_frames,
                    'records': records + 1,
                }
                _write_status(args.status_file, last_status)
                line = json.dumps(record, ensure_ascii=False, allow_nan=False)
                print(line, flush=True)
                log.write(line + '\n')
                log.flush()
                records += 1
                next_decision = now + 1.0 / args.decision_hz
    finally:
        stop_result = {'transport': 'simulated_not_sent'}
        if sender:
            try:
                stop_result = sender.stop()
            except Exception as exc:
                stop_result = {'transport': 'stop_unconfirmed', 'error': str(exc)}
        if last_status:
            _finalize_status(last_status, stop_result)
            _write_status(args.status_file, last_status)
        client.close()
    return records


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--ip', default='192.168.4.1')
    parser.add_argument('--port', type=int, default=4242)
    parser.add_argument('--config', type=Path, default=ROOT / 'config/car.json')
    parser.add_argument('--seconds', type=float, default=30.0)
    parser.add_argument('--decision-hz', type=float, default=2.0)
    parser.add_argument('--model', type=Path, default=ROOT.parent / 'bme688' /
                        'model/best_model.pth')
    parser.add_argument('--max-gas10-age-ms', type=int, default=5000)
    parser.add_argument('--tgs-baseline-left', type=float, default=0.2199375,
                        help='session baseline voltage for TGS channel 0')
    parser.add_argument('--tgs-baseline-right', type=float, default=0.1959375,
                        help='session baseline voltage for TGS channel 1')
    parser.add_argument('--tgs-window-samples', type=int, default=20,
                        help='TGS feature history length at the actual sample rate')
    parser.add_argument('--tgs-response-scale', type=float, default=0.10,
                        help='voltage response mapped to normalized magnitude 1')
    parser.add_argument('--tgs-noise-response', type=float, default=0.0028125,
                        help='normalized response noise scale (voltage delta / response scale); 6-count enter candidate uses 0.0028125')
    parser.add_argument('--tgs-enter-z', type=float, default=4.0,
                        help='TGS trigger enter threshold in noise units')
    parser.add_argument('--tgs-exit-z', type=float, default=2.0,
                        help='TGS trigger release threshold in noise units (3-count candidate)')
    parser.add_argument('--tgs-common-z', type=float, default=2.5,
                        help='minimum absolute common-mode response in noise units')
    parser.add_argument('--tgs-max-gap-s', type=float, default=0.75,
                        help='maximum paired-sample gap before TGS trigger timing resets; includes 2 Hz scheduler jitter')
    parser.add_argument('--tgs-invalid-grace-s', type=float, default=0.5,
                        help='maximum age for hold-last TGS PPO features during brief invalid samples')
    parser.add_argument('--no-classifier', action='store_true')
    parser.add_argument('--action', choices=ACTION_NAMES, default='STOP',
                        help='Fixed action when --enable-rl is not used')
    parser.add_argument('--enable-rl', action='store_true',
                        help='Run PPO inference; incomplete input produces STOP')
    parser.add_argument('--send-actions', action='store_true',
                        help='Send mode=1/2 actions to the car; default is simulation only')
    parser.add_argument('--feature-count', type=int, choices=(17, 18, 21, 22), default=21,
                        help='RL input width; 21/22 include relative x/y and sin/cos heading')
    parser.add_argument('--legacy-18d', action='store_true',
                        help='Compatibility alias for --feature-count 18')
    parser.add_argument('--ppo-model', type=Path,
                        default=ROOT.parent / 'ppo' / 'models' / 'best_model.pth')
    parser.add_argument('--legacy-18d-model', type=Path,
                        default=ROOT.parent / 'ppo' / 'models' / 'best_model.pth')
    parser.add_argument('--map-width', type=float, default=3.0,
                        help='map width in metres for relative position normalization')
    parser.add_argument('--map-height', type=float, default=3.0,
                        help='map height in metres for relative position normalization')
    parser.add_argument('--telemetry-timeout-ms', type=int, default=600,
                        help='latch safety stop after this telemetry silence')
    parser.add_argument('--task-state-machine', action='store_true',
                        help='run coverage search, alcohol trigger, PPO timeout and task states')
    parser.add_argument('--control-file', type=Path,
                        help='operator command JSON file used by the system GUI')
    parser.add_argument('--status-file', type=Path,
                        help='latest workflow status JSON file used by the system GUI')
    parser.add_argument('--ppo-timeout-s', type=float, default=90.0,
                        help='maximum PPO tracking time before human-confirmed stop')
    parser.add_argument('--alcohol-threshold', type=float, default=0.8,
                        help='alcohol probability threshold for final source confirmation')
    parser.add_argument('--detection-samples', type=int, default=3,
                        help='consecutive joint GAS10/TGS scans required for candidate and stationary retest')
    parser.add_argument('--verify-linear-speed-m-s', type=float, default=0.02,
                        help='maximum absolute linear speed treated as stationary during source retest')
    parser.add_argument('--verify-angular-speed-rad-s', type=float, default=0.05,
                        help='maximum absolute angular speed treated as stationary during source retest')
    parser.add_argument('--verify-stationary-s', type=float, default=0.5,
                        help='required stationary dwell before counting retest scans')
    parser.add_argument('--tgs-trigger-duration-s', type=float, default=3.0,
                        help='continuous TGS response duration required to start PPO')
    parser.add_argument('--coverage-seed', type=int, default=None,
                        help='optional seed for reproducible coverage motion')
    parser.add_argument('--output', type=Path, default=ROOT / 'reports' /
                        ('car_workflow_' + datetime.now(timezone.utc).strftime('%Y%m%d_%H%M%S')))
    args = parser.parse_args()
    if args.legacy_18d:
        args.feature_count = 18
        args.ppo_model = args.legacy_18d_model
    if not 1 <= args.seconds <= 86400:
        parser.error('--seconds must be in 1..86400')
    if not 0.1 <= args.decision_hz <= 10:
        parser.error('--decision-hz must be in 0.1..10')
    if args.max_gas10_age_ms <= 0:
        parser.error('--max-gas10-age-ms must be positive')
    if args.map_width <= 0 or args.map_height <= 0:
        parser.error('--map-width and --map-height must be positive')
    if args.telemetry_timeout_ms < 100:
        parser.error('--telemetry-timeout-ms must be at least 100')
    if args.ppo_timeout_s <= 0:
        parser.error('--ppo-timeout-s must be positive')
    if not 0 < args.alcohol_threshold <= 1:
        parser.error('--alcohol-threshold must be in (0, 1]')
    if args.detection_samples < 1:
        parser.error('--detection-samples must be positive')
    if (args.verify_linear_speed_m_s < 0 or
            args.verify_angular_speed_rad_s < 0 or
            args.verify_stationary_s <= 0):
        parser.error('source verification motion thresholds must be non-negative and dwell positive')
    if (args.tgs_noise_response <= 0 or args.tgs_enter_z <= 0 or
            args.tgs_exit_z <= 0 or args.tgs_common_z <= 0 or
            args.tgs_max_gap_s <= 0 or args.tgs_invalid_grace_s <= 0 or
            args.tgs_trigger_duration_s <= 0):
        parser.error('TGS thresholds and durations must be positive')
    try:
        count = run(args)
    except FileExistsError:
        parser.error('Output path already exists; choose a fresh path')
    except (OSError, RuntimeError, ValueError, TimeoutError) as exc:
        parser.exit(1, f'Workflow failed: {exc}\n')
    suffix = 'live action commands were sent.' if args.send_actions else 'simulated commands only; no car commands were sent.'
    print(f'Workflow complete: {count} observations; {suffix}')


if __name__ == '__main__':
    main()
