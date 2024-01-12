import numpy as np
import carla

CATEGORIES = ["left_turn", "right_turn", "max_speed30", "max_speed50", "max_speed60"]


class BoundingBox:
    """Bounding box of a detected traffic sign in image coordinates.

    Represented by (x,y)-center width width and height of the bounding box.
    category is in {"left_turn", "right_turn", "max_speed30", "max_speed50", "max_speed60"}
    """

    def __init__(self, x: int, y: int, width: int, height: int, category: str, confidence: float):
        self.x = int(x)
        self.y = int(y)
        self.width = int(width)
        self.height = int(height)
        self.category = category
        self.confidence = confidence

        self.world_position = np.array([0, 0, 0])


class TrafficSign:
    """representation of a traffic sign, with a 2D world position, a category (class), and a confidence."""

    def __init__(
        self,
        sign_position: np.array = np.array([0, 0]),
        sign_category: str = "none",
        confidence: float = 1.0 / len(CATEGORIES),
    ):
        self.position = sign_position
        self.category = sign_category
        self.integrated = False

        # probability distribution over categories given the observations.
        self.distribution = np.array([(1 - confidence) / (len(CATEGORIES) - 1)] * len(CATEGORIES))
        if sign_category in CATEGORIES:
            self.distribution[CATEGORIES.index(sign_category)] = confidence


def transform_to_numpy(transform: carla.Transform):
    """convert carla.transform into numpy array."""
    rotation = transform.rotation
    location = transform.location

    c_y = np.cos(np.radians(rotation.yaw))
    s_y = np.sin(np.radians(rotation.yaw))
    c_r = np.cos(np.radians(rotation.roll))
    s_r = np.sin(np.radians(rotation.roll))
    c_p = np.cos(np.radians(rotation.pitch))
    s_p = np.sin(np.radians(rotation.pitch))

    matrix = np.identity(4)
    matrix[0, 3] = location.x
    matrix[1, 3] = location.y
    matrix[2, 3] = location.z
    matrix[0, 0] = c_p * c_y
    matrix[0, 1] = c_y * s_p * s_r - s_y * c_r
    matrix[0, 2] = -c_y * s_p * c_r - s_y * s_r
    matrix[1, 0] = s_y * c_p
    matrix[1, 1] = s_y * s_p * s_r + c_y * c_r
    matrix[1, 2] = -s_y * s_p * c_r + c_y * s_r
    matrix[2, 0] = s_p
    matrix[2, 1] = -c_p * s_r
    matrix[2, 2] = c_p * c_r

    return matrix




def image_to_world(world, point: np.array, distance: float) -> np.array:
    """transform a point in image coordinates (u,v) to corresponding world coordinates (x,y,z).

    Hint: 1. consider that the transformation from image to camera coordinates results in camera coordinates
             system, i.e., right-handed coordinates: x - right, y - down, z - forward
          2. Projecting a point [x, y, z, 1] uses: s * [u, v, 1] = K * T_cam_from_world * [x, y, z, 1].
    """
    point_world = np.array([0, 0, 0])
    # print(point_world.shape)
    #######################################################################
    ######################### TODO: IMPLEMENT THIS ########################
    #######################################################################
    vehicle = world.get_vehicle()
    camera = vehicle.get_camera()
    camera_calib = camera.calibration
    cam_transform = transform_to_numpy(camera.get_transform())

    # transform point from image to camera coordinates
    k_inv = np.linalg.inv(camera_calib)
    point_homo = np.ones((3,1))
    point_homo[0, 0] = point[0]
    point_homo[1, 0] = point[1]
    point_camera = (k_inv @ point_homo) * distance 
    # converting the point in camera fram from the camera coordinate system to unreal engine world coordinate system
    point_camera_world = np.zeros_like(point_camera)
    point_camera_world[0, 0] = point_camera[2, 0]
    point_camera_world[1, 0] = point_camera[0, 0]
    point_camera_world[2, 0] = -point_camera[1, 0]

    # transforming the point into unreal engine world coordinates using the camera transform from camera to world
    point_world_homo = np.ones((4, 1))
    point_world_homo[0:3, 0] = point_camera_world[:, 0]

    point_world = cam_transform @ point_world_homo

    point_world = np.squeeze(point_world)

    
    return point_world
