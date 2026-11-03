import math
import numpy as np


class OdorField:
    """Small reproducible plume model; source truth is never an observation."""

    def __init__(self, width, height, rng, wind=(0.0, 0.0),
                 source_min_center_distance=0.5, source_max_center_distance=1.2,
                 source_boundary_margin=0.3, plume_spread=0.16,
                 sensor_offset=0.07, source_count=1, source_min_separation=0.4,
                 source_layout="random", dynamic_source_fraction=0.0,
                 dynamic_appearance_min_s=0.0, dynamic_appearance_max_s=0.0,
                 dynamic_source_layout="stratified"):
        self.width, self.height, self.rng = width, height, rng
        margin = min(source_boundary_margin, width * 0.2, height * 0.2)
        center = np.array([width / 2, height / 2])
        dynamic_count = int(round(source_count * dynamic_source_fraction))
        dynamic_count = min(max(dynamic_count, 0), source_count)
        static_count = source_count - dynamic_count
        self.sources = []
        self.activation_times = []
        self.source_sectors = []
        self.source_count = source_count
        self.dynamic_source_layout = dynamic_source_layout
        self.dynamic_indices = set(range(static_count, source_count))
        max_distance = min(source_max_center_distance, min(width, height) * 0.6)
        for index in range(static_count if source_layout == "stratified" else 0):
            base_angle = 2.0 * math.pi * index / max(source_count, 1)
            for _ in range(1000):
                angle = base_angle + rng.uniform(-math.pi / max(source_count, 1) * 0.35, math.pi / max(source_count, 1) * 0.35)
                radius = rng.uniform(source_min_center_distance, max_distance)
                point = center + radius * np.array([math.cos(angle), math.sin(angle)])
                if (margin <= point[0] <= width - margin and margin <= point[1] <= height - margin and
                        all(np.linalg.norm(point - other) >= source_min_separation for other in self.sources)):
                    self.sources.append(point)
                    self.source_sectors.append(index)
                    break
            self.activation_times.append(0.0)
        for _ in range(10000 if source_layout == "random" else 0):
            point = np.array([rng.uniform(margin, width - margin),
                              rng.uniform(margin, height - margin)])
            distance = np.linalg.norm(point - center)
            if (source_min_center_distance <= distance <= max_distance and
                    all(np.linalg.norm(point - other) >= source_min_separation for other in self.sources)):
                self.sources.append(point)
                self.source_sectors.append(len(self.source_sectors))
                self.activation_times.append(0.0)
                if len(self.sources) == source_count:
                    break
        while len(self.sources) < static_count:
            point = np.array([rng.uniform(margin, width - margin),
                              rng.uniform(margin, height - margin)])
            distance = np.linalg.norm(point - center)
            if (source_min_center_distance <= distance <= max_distance and
                    all(np.linalg.norm(point - other) >= source_min_separation for other in self.sources)):
                self.sources.append(point)
                self.source_sectors.append(len(self.source_sectors))
                self.activation_times.append(0.0)
        while len(self.sources) < source_count:
            self.sources.append(None)
            self.source_sectors.append(len(self.source_sectors))
            low = dynamic_appearance_min_s
            high = max(dynamic_appearance_max_s, low)
            self.activation_times.append(float(rng.uniform(low, high)))
        self.source = next((source for source in self.sources if source is not None), None)
        self.wind = np.asarray(wind, dtype=float)
        self.plume_spread = plume_spread
        self.sensor_offset = sensor_offset

    def _random_dynamic_position(self, source_index):
        margin = min(3.0, self.width * 0.2, self.height * 0.2)
        centre = np.array([self.width / 2.0, self.height / 2.0])
        max_distance = min(max(self.width, self.height), min(self.width, self.height) * 0.6)
        for _ in range(10000):
            if self.dynamic_source_layout == "random":
                point = np.array([self.rng.uniform(margin, self.width - margin),
                                  self.rng.uniform(margin, self.height - margin)])
            else:
                base_angle = 2.0 * math.pi * source_index / max(self.source_count, 1)
                angle = base_angle + self.rng.uniform(-math.pi / max(self.source_count, 1) * 0.35,
                                                      math.pi / max(self.source_count, 1) * 0.35)
                radius = self.rng.uniform(4.0, max_distance)
                point = centre + radius * np.array([math.cos(angle), math.sin(angle)])
            distance = np.linalg.norm(point - centre)
            existing = [source for source in self.sources if source is not None]
            if (margin <= point[0] <= self.width - margin and
                    margin <= point[1] <= self.height - margin and
                    0.1 <= distance <= max_distance and
                    all(np.linalg.norm(point - other) >= 0.4 for other in existing)):
                return point
        return centre.copy()

    def update(self, time_s):
        for index in self.dynamic_indices:
            if self.sources[index] is None and time_s >= self.activation_times[index]:
                self.sources[index] = self._random_dynamic_position(index)
        self.source = next((source for source in self.sources if source is not None), None)

    def is_active(self, index, time_s):
        self.update(time_s)
        return self.sources[index] is not None and time_s >= self.activation_times[index]

    def concentration(self, position, time_s=0.0):
        self.update(time_s)
        p = np.asarray(position, dtype=float)
        # Wind shifts the plume downwind while preserving a bounded, smooth field.
        concentrations = []
        for source in self.sources:
            if source is None:
                continue
            shifted = source + self.wind * min(time_s, 20.0) * 0.08
            delta = p - shifted
            downwind = float(np.dot(delta, self.wind))
            lateral = float(np.linalg.norm(delta - self.wind * downwind)) if np.linalg.norm(self.wind) else float(np.linalg.norm(delta))
            spread = self.plume_spread + 0.012 * max(downwind, 0.0)
            concentrations.append(np.exp(-0.5 * (lateral / spread) ** 2) * np.exp(-max(downwind, 0.0) / 1.8) * np.exp(-max(np.linalg.norm(delta) - 0.08, 0.0) / 1.4))
        return float(max(concentrations))

    def sample(self, position, heading, rng, noise, time_s=0.0):
        c = self.concentration(position, time_s)
        left = self.concentration(np.asarray(position) + self.sensor_offset * np.array([-math.sin(heading), math.cos(heading)]), time_s)
        right = self.concentration(np.asarray(position) + self.sensor_offset * np.array([math.sin(heading), -math.cos(heading)]), time_s)
        values = np.maximum(0.0, np.array([left, right, c]) + rng.normal(0.0, noise, 3))
        return values
