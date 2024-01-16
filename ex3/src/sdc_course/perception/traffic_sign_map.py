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
        curr_distribution = traffic_sign.distribution
        new_confidence = confidence
        curr_category = traffic_sign.category
        observed_category = sign_type
        max_distance = 0.2
        if sign_type in CATEGORIES:
            # calculating the new distribution
            new_distribution = np.ones_like(curr_distribution) * ((1 - new_confidence)/ (len(CATEGORIES)- 1))
            new_distribution[CATEGORIES.index(sign_type)] = confidence

            traffic_sign.distribution =  new_distribution
            # print(f"new_distribution: {traffic_sign.distribution}")
            # calclating the new position as the average of previous and current position
            if traffic_sign.category == "none":
                traffic_sign.position = position
            else:
                traffic_sign.position = (traffic_sign.position + position)/2

            # setting new category as the category with highest confidence 
            new_category = CATEGORIES[np.argmax(new_distribution)]
            traffic_sign.category = new_category

            # Updating the corresponding traffic sign in the map
            sqr_closest_distance = max_distance * max_distance
            closest_idx = None
            for idx, entry in enumerate(self._map):
                sqr_distance = np.dot(position - entry.position, position - entry.position)
                if sqr_distance < sqr_closest_distance:
                    sqr_closest_distance = sqr_distance
                    closest_idx = idx
            
            if closest_idx == None:
                self._map.append(traffic_sign)
            else:
                self._map[closest_idx] = traffic_sign



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
