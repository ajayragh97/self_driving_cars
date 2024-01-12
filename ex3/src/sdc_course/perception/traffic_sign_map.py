from typing import Tuple, Dict, List, Union
import numpy as np

from .utils import CATEGORIES, transform_to_numpy, TrafficSign


class TrafficSignMap:
    """Integrate detection into a 2D map representation that records the location of traffic signs."""

    def __init__(self):
        self._map: List[TrafficSign] = []

    def update_map(
        self, position: Union[np.array, Tuple[float, float]], sign_type: str, confidence: float
    ):
        """Integrate given type at the specified 2d world position.

        We have to account for the confidence values and the position of the traffic sign.

        Args:
          position: two-dimensional world position of the traffic sign, i.e., (x,y)-point in UE coordinate system (x-forward, y-right, z-up)
          sign_type: type of sign from CATEGORIES.
          confidences: confidence value from the detection approach.
        """
        assert sign_type in CATEGORIES
        if isinstance(position, (tuple, list)):
            position = np.array(position)

        traffic_sign = self.get_closest_sign(position, 0.2)
        #######################################################################
        ######################### TODO: IMPLEMENT THIS ########################
        #######################################################################
        ## TODO: add logic to update the map with with 2D location. Initialize traffic sign location if needed.

    def get_closest_sign(
        self, position: Union[np.array, Tuple[float, float]], max_distance: float = 5.0
    ) -> TrafficSign:
        """gets closest traffic sign to a given position within a specific radius.

        Args:
          position: query position.
          max_distance: maximal distance of traffic sign for query.

        Return:
          closest traffic sign or TrafficSign with default values, i.e., category = "none"
        """
        if isinstance(position, (tuple, list)):
            position = np.array(position)

        sqr_closest_distance = max_distance * max_distance
        result = TrafficSign()

        # one could use proper neighbor search, like a quad tree, but this should work for small examples:
        for entry in self._map:
            sqr_distance = np.dot(position - entry.position, position - entry.position)
            if sqr_distance < sqr_closest_distance:
                sqr_closest_distance = sqr_distance
                result = entry

        return result

    def get_traffic_signs(self):
        """get list of traffic signs represented in the map."""
        return self._map
