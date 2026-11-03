#!/usr/bin/env python3
"""Run a reproducible checkpoint evaluation and export publication figures."""

import argparse
import csv
import hashlib
import json
import sys
from pathlib import Path

import matplotlib as mpl
import matplotlib.pyplot as plt
import numpy as np
import torch

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from config.config import EnvironmentConfig
from environment.robot_env_v3 import OdorSourceLocalizationEnvV3
from models.ppo_network import PPOAgent, PPOPositionResidualNetwork


class EvalConfig:
    DEVICE = 'cuda' if torch.cuda.is_available() else 'cpu'
    LEARNING_RATE = 3e-4
    PPO_EPOCHS = 1
    CLIP_EPSILON = 0.2
    VALUE_COEF = 0.5
    ENTROPY_COEF = 0.01
    MAX_GRAD_NORM = 0.5


def wilson_interval(successes, trials, z=1.959963984540054):
    if trials == 0:
        return (float('nan'), float('nan'))
    p = successes / trials
    denom = 1 + z * z / trials
    center = (p + z * z / (2 * trials)) / denom
    half = z * np.sqrt(p * (1 - p) / trials + z * z / (4 * trials * trials)) / denom
    return center - half, center + half


def load_policy(checkpoint_path, feature_count):
    checkpoint = torch.load(checkpoint_path, map_location='cpu', weights_only=False)
    if checkpoint.get('architecture') == 'position_residual_v1':
        network = PPOPositionResidualNetwork((8, 21), 7)
        network.load_state_dict(checkpoint['network_state_dict'])
        network.eval()

        def act(state):
            with torch.no_grad():
                logits, _ = network(torch.from_numpy(state).float().unsqueeze(0))
            return int(logits.argmax(dim=-1).item())

        return act

    agent = PPOAgent((8, feature_count), 7, EvalConfig)
    agent.load(str(checkpoint_path))
    agent.network.eval()
    return lambda state: agent.select_action(state, training=False)


def run_evaluation(checkpoint_path, feature_count, episodes, seed_offset, safety_enabled):
    act = load_policy(checkpoint_path, feature_count)
    env_config = EnvironmentConfig()
    env_config.INCLUDE_GAS_RESISTANCE = feature_count in (18, 22)
    env_config.INCLUDE_POSITION = feature_count in (21, 22)
    env_config.BOUNDARY_SAFETY_ENABLED = safety_enabled
    rows = []

    for episode in range(episodes):
        episode_seed = seed_offset + episode
        np.random.seed(episode_seed)
        env = OdorSourceLocalizationEnvV3(env_config)
        state = env.reset(random_seed=episode_seed)
        overrides = 0
        for step in range(env.max_steps):
            action = act(state)
            state, _, done, info = env.step(action)
            overrides += int(info.get('safety_override', False))
            if done:
                terminal = ('success' if info['success'] else
                            'collision' if info['collision'] else 'timeout')
                rows.append({
                    'episode': episode + 1,
                    'seed': episode_seed,
                    'termination': terminal,
                    'success': int(bool(info['success'])),
                    'steps': step + 1,
                    'safety_overrides': overrides,
                })
                break
        else:
            rows.append({
                'episode': episode + 1,
                'seed': episode_seed,
                'termination': 'rollout_limit',
                'success': 0,
                'steps': env.max_steps,
                'safety_overrides': overrides,
            })
    return rows


def draw_figure(rows, checkpoint_path, output_stem, seed_offset, safety_enabled):
    n = len(rows)
    counts = {name: sum(row['termination'] == name for row in rows)
              for name in ('success', 'timeout', 'collision', 'rollout_limit')}
    rate = counts['success'] / n
    ci_low, ci_high = wilson_interval(counts['success'], n)
    success_steps = np.array([r['steps'] for r in rows if r['success']], dtype=int)

    mpl.rcParams.update({
        'font.family': 'Noto Sans CJK SC', 'font.size': 8.5,
        'axes.labelsize': 8.5, 'axes.titlesize': 9.5,
        'xtick.labelsize': 8, 'ytick.labelsize': 8,
        'pdf.fonttype': 42, 'ps.fonttype': 42,
        'svg.fonttype': 'none', 'axes.spines.top': False,
        'axes.spines.right': False,
    })
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(7.2, 3.75), layout='constrained')
    fig.patch.set_facecolor('white')

    outcome_colors = {
        'success': '#0072B2', 'timeout': '#D55E00',
        'collision': '#CC79A7', 'rollout_limit': '#7A858C',
    }
    outcome_labels = {
        'success': '成功', 'timeout': '超时',
        'collision': '碰撞', 'rollout_limit': '达到步数上限',
    }
    left = 0.0
    for outcome in ('success', 'timeout', 'collision', 'rollout_limit'):
        width = counts[outcome] / n * 100
        if width:
            ax1.barh([0], [width], left=[left], color=outcome_colors[outcome],
                     height=0.48, label=outcome_labels[outcome], zorder=2)
        left += width
    ax1.errorbar(rate * 100, 0, xerr=[[max(0, (rate-ci_low)*100)],
                                     [max(0, (ci_high-rate)*100)]],
                 fmt='o', color='#202020', capsize=4, markersize=4, linewidth=1.2, zorder=3)
    ax1.set_xlim(0, 100)
    ax1.set_ylim(-0.8, 0.78)
    ax1.set_yticks([0], ['全部评估回合'])
    ax1.set_xlabel('回合占比（%）')
    ax1.set_title('A  回合结果', loc='left', fontweight='bold')
    ax1.grid(axis='x', color='#D9DEE3', linewidth=0.6, zorder=0)
    ax1.text(0, -0.22,
             f"成功 {counts['success']}/{n}（{rate*100:.1f}%）  |  "
             f"超时 {counts['timeout']}  |  碰撞 {counts['collision']}",
             transform=ax1.transAxes, ha='left', va='top', fontsize=7.2, color='#303840')
    ax1.text(0, -0.34, f'成功率 95% Wilson 置信区间：{ci_low*100:.1f}%–{ci_high*100:.1f}%',
             transform=ax1.transAxes, ha='left', va='top', fontsize=7.2, color='#303840')
    ax1.legend(frameon=False, loc='lower center', bbox_to_anchor=(0.5, 1.0),
               ncol=4, fontsize=7.2, handlelength=1.0, columnspacing=0.9)

    if success_steps.size:
        sorted_steps = np.sort(success_steps)
        ecdf = np.arange(1, len(sorted_steps) + 1) / len(sorted_steps)
        ax2.step(sorted_steps, ecdf * 100, where='post', color='#009E73', linewidth=1.8)
        median = float(np.median(success_steps))
        q1, q3 = np.percentile(success_steps, [25, 75])
        ax2.axvline(median, color='#CC79A7', linestyle='--', linewidth=1.2,
                    label=f'中位数 {median:.0f} 步')
        ax2.axvspan(q1, q3, color='#009E73', alpha=0.12,
                    label=f'四分位距 {q1:.0f}–{q3:.0f} 步')
        ax2.legend(frameon=False, loc='lower right', fontsize=8)
    else:
        ax2.text(0.5, 0.5, '无成功回合', transform=ax2.transAxes,
                 ha='center', va='center')
    ax2.set_xlabel('成功回合步数（决策步）')
    ax2.set_ylabel('累计比例（%）')
    ax2.set_ylim(0, 100)
    ax2.set_title('B  成功回合步数分布', loc='left', fontweight='bold')
    ax2.grid(color='#D9DEE3', linewidth=0.6)

    policy = '边界安全层开启' if safety_enabled else '仅策略（未启用边界安全层）'
    fig.suptitle(f'PPO 气味源定位仿真评估 | {policy} | n={n}',
                 fontsize=10, fontweight='bold')
    for fmt in ('pdf', 'svg', 'png'):
        kwargs = {'format': fmt, 'facecolor': 'white'}
        if fmt == 'png':
            kwargs['dpi'] = 601
        fig.savefig(output_stem.with_suffix('.' + fmt), **kwargs)
    plt.close(fig)
    return {
        'episodes': n,
        'seed_offset': seed_offset,
        'safety_enabled': safety_enabled,
        'termination_counts': counts,
        'success_rate': rate,
        'success_rate_wilson_95ci': [ci_low, ci_high],
        'success_steps_median': float(np.median(success_steps)) if success_steps.size else None,
        'success_steps_q1_q3': [float(v) for v in np.percentile(success_steps, [25, 75])]
            if success_steps.size else None,
        'checkpoint': str(checkpoint_path),
        'checkpoint_sha256': hashlib.sha256(checkpoint_path.read_bytes()).hexdigest(),
        'evaluation': 'deterministic argmax policy; simulated environment; independent boundary safety setting',
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--checkpoint', type=Path, default=ROOT / 'models/best_model.pth')
    parser.add_argument('--feature-count', type=int, choices=(17, 18, 21, 22), default=21)
    parser.add_argument('--episodes', type=int, default=1000)
    parser.add_argument('--seed-offset', type=int, default=500000)
    parser.add_argument('--no-boundary-safety', action='store_true')
    parser.add_argument('--from-csv', type=Path,
                        help='仅使用已有逐回合 CSV 重绘；不重新评估或覆盖 CSV/manifest')
    parser.add_argument('--output-dir', type=Path, default=ROOT / 'figures')
    args = parser.parse_args()
    args.output_dir.mkdir(parents=True, exist_ok=True)
    checkpoint = args.checkpoint.resolve()
    stem = args.output_dir / 'ppo_evaluation'
    if args.from_csv:
        with args.from_csv.open(newline='', encoding='utf-8') as f:
            rows = list(csv.DictReader(f))
        if not rows:
            parser.error(f'CSV 中没有评估回合：{args.from_csv}')
        for row in rows:
            for key in ('episode', 'seed', 'success', 'steps', 'safety_overrides'):
                row[key] = int(row[key])
        manifest_path = args.from_csv.with_name('ppo_evaluation_manifest.json')
        safety_enabled = not args.no_boundary_safety
        seed_offset = rows[0]['seed']
        if manifest_path.exists():
            existing = json.loads(manifest_path.read_text(encoding='utf-8'))
            safety_enabled = bool(existing.get('safety_enabled', safety_enabled))
            seed_offset = int(existing.get('seed_offset', seed_offset))
        summary = draw_figure(rows, checkpoint, stem, seed_offset, safety_enabled)
        print(json.dumps({'rendered_from_csv': str(args.from_csv),
                          'outputs': [str(stem.with_suffix(ext))
                                      for ext in ('.pdf', '.svg', '.png')],
                          'evaluation_summary': summary}, ensure_ascii=False, indent=2))
        return

    rows = run_evaluation(checkpoint, args.feature_count, args.episodes,
                          args.seed_offset, not args.no_boundary_safety)
    with (args.output_dir / 'ppo_episode_results.csv').open('w', newline='', encoding='utf-8') as f:
        writer = csv.DictWriter(f, fieldnames=rows[0].keys())
        writer.writeheader()
        writer.writerows(rows)
    summary = draw_figure(rows, checkpoint, stem, args.seed_offset,
                          not args.no_boundary_safety)
    (args.output_dir / 'ppo_evaluation_manifest.json').write_text(
        json.dumps(summary, ensure_ascii=False, indent=2) + '\n', encoding='utf-8')
    print(json.dumps(summary, ensure_ascii=False, indent=2))


if __name__ == '__main__':
    main()
