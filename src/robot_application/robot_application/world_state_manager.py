"""Pick/drop world state and catalog management for seasonal task composition."""

import os
import time
from typing import Any, Dict, List, Optional, Tuple

import yaml
from ament_index_python.packages import get_package_share_directory


class WorldStateManager:
    """Owns pick/drop catalog loading and runtime occupancy state."""

    def __init__(self, package_name: str = 'robot_application'):
        self.package_name = package_name
        self.pick_locations_catalog: Dict[str, Dict[str, Any]] = {}
        self.drop_locations_catalog: Dict[str, Dict[str, Any]] = {}
        self.pick_state: Dict[str, Dict[str, Any]] = {}
        self.drop_state: Dict[str, Dict[str, Any]] = {}
        self.empty_pick_locations = set()
        self.occupied_drop_locations = set()
        self.reload_catalog_and_state()

    def reload_catalog_and_state(self) -> Tuple[List[Dict[str, Any]], List[Dict[str, Any]]]:
        """Reload pick/drop catalog from config and reset runtime state."""
        pick_locations, drop_locations = self._load_pick_drop_locations()
        self.pick_locations_catalog = {str(pick['id']): pick for pick in pick_locations}
        self.drop_locations_catalog = {str(drop['id']): drop for drop in drop_locations}
        self.reset_runtime_state()
        return pick_locations, drop_locations

    def reset_runtime_state(self):
        """Reset dynamic pick/drop occupancy while preserving catalog data."""
        now = time.time()
        self.pick_state = {
            pick_id: {
                'empty': False,
                'last_update': now,
            }
            for pick_id in self.pick_locations_catalog
        }
        self.drop_state = {
            drop_id: {
                'occupancy': 0,
                'capacity': int(drop.get('capacity', 1)),
                'is_full': False,
                'last_known_color': 'unknown',
                'color_confidence': 0.0,
                'last_update': now,
            }
            for drop_id, drop in self.drop_locations_catalog.items()
        }
        self.empty_pick_locations = set()
        self.occupied_drop_locations = set()

    def all_pick_locations(self) -> List[Dict[str, Any]]:
        return list(self.pick_locations_catalog.values())

    def all_drop_locations(self) -> List[Dict[str, Any]]:
        return list(self.drop_locations_catalog.values())

    def task_pick_id(self, task_parameters: Dict[str, Any]) -> str:
        pick_location = task_parameters.get('pick_location', {})
        if isinstance(pick_location, dict):
            return str(pick_location.get('id', ''))
        return ''

    def task_drop_candidates(self, task_parameters: Dict[str, Any]) -> List[Dict[str, Any]]:
        primary_drop = task_parameters.get('drop_location')
        if isinstance(primary_drop, dict) and primary_drop:
            return [primary_drop]
        return []

    def is_drop_available(self, drop_id: str) -> bool:
        drop = self.drop_state.get(drop_id)
        if drop is None:
            return False
        return not bool(drop.get('is_full', False))

    def get_full_drop_ids(self) -> List[str]:
        return [drop_id for drop_id in self.drop_state if not self.is_drop_available(drop_id)]

    def mark_pick_empty(self, pick_id: str):
        if pick_id not in self.pick_state:
            return
        self.pick_state[pick_id]['empty'] = True
        self.pick_state[pick_id]['last_update'] = time.time()
        self.empty_pick_locations.add(pick_id)

    def mark_drop_full(self, drop_id: str):
        drop = self.drop_state.get(drop_id)
        if drop is None:
            return
        drop['occupancy'] = int(drop.get('capacity', 1))
        drop['is_full'] = True
        drop['last_update'] = time.time()
        self.occupied_drop_locations.add(drop_id)

    def mark_drop_occupied(self, drop_id: str):
        drop = self.drop_state.get(drop_id)
        if drop is None:
            return

        capacity = max(1, int(drop.get('capacity', 1)))
        occupancy = min(capacity, int(drop.get('occupancy', 0)) + 1)
        drop['occupancy'] = occupancy
        drop['is_full'] = occupancy >= capacity
        drop['last_update'] = time.time()

        if drop['is_full']:
            self.occupied_drop_locations.add(drop_id)
        else:
            self.occupied_drop_locations.discard(drop_id)

    @staticmethod
    def distance_between_locations(from_location: Dict[str, float], to_location: Dict[str, float]) -> float:
        from_x = float(from_location.get('x', 0.0))
        from_y = float(from_location.get('y', 0.0))
        to_x = float(to_location.get('x', 0.0))
        to_y = float(to_location.get('y', 0.0))
        return ((from_x - to_x) ** 2 + (from_y - to_y) ** 2) ** 0.5

    def _load_pick_drop_locations(self) -> Tuple[List[Dict[str, Any]], List[Dict[str, Any]]]:
        config_path = None
        try:
            package_share = get_package_share_directory(self.package_name)
            config_path = os.path.join(package_share, 'config', 'pick_drop_locations.yaml')
            with open(config_path, 'r', encoding='utf-8') as config_file:
                config = yaml.safe_load(config_file) or {}
        except Exception:
            config_path = os.path.abspath(
                os.path.join(os.path.dirname(__file__), '..', 'config', 'pick_drop_locations.yaml')
            )
            with open(config_path, 'r', encoding='utf-8') as config_file:
                config = yaml.safe_load(config_file) or {}

        pick_locations_raw = config.get('pick_locations', config.get('pick', []))
        drop_locations_raw = config.get('drop_locations', config.get('drop', []))

        pick_locations = self._normalize_locations(pick_locations_raw)
        drop_locations = self._normalize_locations(drop_locations_raw)

        if not pick_locations or not drop_locations:
            raise RuntimeError(f'pick_drop_locations.yaml is missing picks or drops: {config_path}')

        return pick_locations, drop_locations

    def _normalize_locations(self, locations: List[Dict[str, Any]]) -> List[Dict[str, Any]]:
        normalized = []
        for location in locations:
            approach_positions = []
            for approach in location.get('approach_positions', []):
                pose = approach.get('pose', approach)
                approach_positions.append({
                    'id': approach.get('id'),
                    'priority': approach.get('priority', 1),
                    'x': float(pose.get('x', 0.0)),
                    'y': float(pose.get('y', 0.0)),
                    'theta': float(pose.get('theta', 0.0)),
                })

            approach_positions = sorted(
                approach_positions,
                key=lambda approach: int(approach.get('priority', 1)),
            )
            normalized.append({
                'id': location.get('id'),
                'name': location.get('name', location.get('id', 'location')),
                'priority': int(location.get('priority', 0)),
                'capacity': int(location.get('capacity', 1)),
                'approach_positions': approach_positions,
            })

        return normalized
