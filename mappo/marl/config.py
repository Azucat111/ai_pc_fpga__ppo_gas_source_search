from dataclasses import dataclass


@dataclass
class SimulationConfig:
    scenario: str = "small_2m_3agents"
    width: float = 2.0
    height: float = 2.0
    agents: int = 3
    episodes: int = 20
    decision_hz: float = 2.0
    max_seconds: float = 90.0
    source_min_center_distance: float = 0.5
    source_max_center_distance: float = 1.2
    source_boundary_margin: float = 0.25
    source_count: int = 1
    required_sources: int = 1
    source_min_separation: float = 0.4
    source_layout: str = "random"
    dynamic_source_fraction: float = 0.0
    dynamic_appearance_min_s: float = 0.0
    dynamic_appearance_max_s: float = 0.0
    dynamic_source_layout: str = "stratified"
    success_radius: float = 0.25
    success_probability: float = 0.5
    sensor_noise: float = 0.035
    wind_x: float = 0.0
    wind_y: float = 0.0
    communication_range: float = 1.5
    plume_spread: float = 0.16
    sensor_offset: float = 0.07
    motion_scale: float = 1.0
    coverage_response_threshold: float = 0.18
    tracking_response_threshold: float = 0.50
    coverage_radius_fraction: float = 0.35
    coverage_radii: tuple = (0.22, 0.38, 0.50)
    coverage_pattern: str = "radial"
    coverage_period_s: float = 25.0
    coordination_enabled: bool = True
    communication_enabled: bool = True
    communication_period_s: float = 1.0
    communication_event_threshold: float = 0.15
    communication_packet_bytes: int = 64
    belief_timeout_s: float = 30.0
    seed: int = 20261030

    @property
    def dt(self):
        return 1.0 / self.decision_hz

    @property
    def max_steps(self):
        return int(self.max_seconds * self.decision_hz)


def scenario_config(name="small_2m_3agents", **overrides):
    """Return a named benchmark configuration with optional CLI overrides."""
    presets = {
        "small_2m_3agents": dict(),
        "moderate_10m_10agents": dict(
            width=10.0, height=10.0, agents=10, max_seconds=120.0,
            source_count=3, source_min_center_distance=2.0,
            required_sources=3,
            source_max_center_distance=4.2, source_boundary_margin=1.0,
            success_radius=0.5, communication_range=4.0,
            plume_spread=0.65, sensor_offset=0.30, motion_scale=1.0,
        ),
        "competition_20m_10agents": dict(
            width=20.0, height=20.0, agents=10, max_seconds=240.0,
            source_count=5, required_sources=3,
            source_min_center_distance=2.5, source_max_center_distance=6.5,
            source_boundary_margin=2.0, source_min_separation=2.0,
            success_radius=0.8, communication_range=7.0,
            plume_spread=1.8, sensor_offset=0.50, motion_scale=1.0,
        ),
        "challenge_30m_10agents_10sources": dict(
            width=30.0, height=30.0, agents=10, max_seconds=1500.0,
            source_count=10, required_sources=10,
            source_layout="stratified",
            dynamic_source_fraction=0.30,
            dynamic_appearance_min_s=180.0, dynamic_appearance_max_s=450.0,
            source_min_center_distance=4.0, source_max_center_distance=12.0,
            source_boundary_margin=3.0, source_min_separation=2.5,
            success_radius=3.0, communication_range=10.0,
            plume_spread=3.0, sensor_offset=0.55, motion_scale=1.0,
            coverage_response_threshold=0.18, coverage_radius_fraction=0.42,
            tracking_response_threshold=0.50,
            coverage_radii=(0.22, 0.38, 0.50),
            coverage_pattern="radial", coverage_period_s=90.0,
        ),
        "transfer_hard_50m_10agents_20sources": dict(
            width=50.0, height=50.0, agents=10, max_seconds=1800.0,
            source_count=20, required_sources=20,
            source_layout="random", dynamic_source_layout="random",
            dynamic_source_fraction=0.30,
            dynamic_appearance_min_s=240.0, dynamic_appearance_max_s=800.0,
            source_min_center_distance=8.0, source_max_center_distance=22.0,
            source_boundary_margin=5.0, source_min_separation=4.0,
            success_radius=1.50, communication_range=8.0,
            plume_spread=1.60, sensor_offset=0.65, motion_scale=1.0,
            sensor_noise=0.08, wind_x=0.0, wind_y=0.0,
            coverage_response_threshold=0.22, tracking_response_threshold=0.62,
            coverage_radii=(0.25, 0.40, 0.52), coverage_pattern="radial",
            coverage_period_s=75.0, communication_period_s=2.0,
            communication_event_threshold=0.20, belief_timeout_s=20.0,
        ),
        "large_100m_10agents": dict(
            width=100.0, height=100.0, agents=10, max_seconds=300.0,
            source_count=3,
            required_sources=3,
            source_min_center_distance=20.0, source_max_center_distance=45.0,
            source_boundary_margin=5.0, success_radius=2.0,
            sensor_noise=0.035, communication_range=30.0,
            plume_spread=5.0, sensor_offset=2.0, motion_scale=1.0,
        ),
    }
    if name not in presets:
        raise ValueError(f"unknown scenario: {name}")
    values = presets[name].copy()
    values.update({key: value for key, value in overrides.items() if value is not None})
    values["scenario"] = name
    return SimulationConfig(**values)
