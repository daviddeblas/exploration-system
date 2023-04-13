import cv2
import numpy as np
from nav_msgs.msg import OccupancyGrid
import rospy

def reformat_map(data):
    # Convertir le message OccupancyGrid vers une image RGBA OpenCV
    map_array = np.array(data.data, dtype=np.int8).reshape(
        (data.info.height, data.info.width))

    # Enlever tous les lignes et colonnes contenant seulement des -1
    non_free_rows, non_free_cols = np.nonzero(map_array != -1)
    cropped_array = map_array[
        non_free_rows.min(): non_free_rows.max() + 1,
        non_free_cols.min(): non_free_cols.max() + 1,
    ]

    # Ajuster la carte pour que les coordonnées soient respectées
    cropped_origin_x = data.info.origin.position.x + \
        non_free_cols.min() * data.info.resolution
    cropped_origin_y = data.info.origin.position.y + \
        non_free_rows.min() * data.info.resolution
    cropped_height, cropped_width = cropped_array.shape
    cropped_info = OccupancyGrid(
        header=data.header,
        info=data.info,
        data=cropped_array.flatten().tolist(),
    )
    cropped_info.info.width = cropped_width
    cropped_info.info.height = cropped_height
    cropped_info.info.origin.position.x = cropped_origin_x
    cropped_info.info.origin.position.y = cropped_origin_y

    # Créer l'image de couleur
    map_image = np.zeros((cropped_height, cropped_width, 4), dtype=np.uint8)

    # Mapper les valeurs de OccupancyGrid vers les valeurs des images
    map_image[cropped_array == -1] = [128, 128, 128, 0]   # Inconnues
    map_image[(cropped_array >= 0) & (cropped_array <= 50)] = [
        255, 255, 255, 255]    # Espace ouvert
    map_image[(cropped_array > 50) & (cropped_array <= 100)] = [
        0, 0, 0, 255]    # Espace Occupé

    return map_image, cropped_origin_x, cropped_origin_y, cropped_info

def map_generation_limo(data, map_frame, tfBuffer, cognifly_exists, limo_exists):
    map_image, cropped_origin_x, cropped_origin_y, cropped_info = reformat_map(data)

    if limo_exists:
        trans, rot = tfBuffer.lookupTransform(
            map_frame, "base_footprint", rospy.Time())

        robot_pos_x = int((trans[0] - cropped_origin_x) /
                        cropped_info.info.resolution)
        robot_pos_y = int((trans[1] - cropped_origin_y) /
                        cropped_info.info.resolution)

        # Dessine un rectangle au niveau de la position du limo
        cv2.rectangle(map_image, (robot_pos_x-2, robot_pos_y-2),
                    (robot_pos_x+2, robot_pos_y+2), (255, 0, 0, 255), thickness=-1)

    if cognifly_exists:

        trans_cognifly, rot_cognifly = tfBuffer.lookupTransform(
            map_frame, "simple_quad_base_link_global", rospy.Time())

        cognifly_pos_x = int(((trans_cognifly[0]) - cropped_origin_x) /
                        cropped_info.info.resolution)
        cognifly_pos_y = int(((trans_cognifly[1]) - cropped_origin_y) /
                        cropped_info.info.resolution)

        # Dessine un rectangle au niveau de la position du cognifly
        cv2.rectangle(map_image, (cognifly_pos_x-2, cognifly_pos_y-2),
                    (cognifly_pos_x+2, cognifly_pos_y+2), (0, 255, 0, 255), thickness=-1)

    # Encoder l'image en PNG
    ret, png = cv2.imencode('.png', map_image)

    return png

def map_generation_cognifly(data, map_frame, tfBuffer):
    map_image, cropped_origin_x, cropped_origin_y, cropped_info = reformat_map(data)

    trans_cognifly, rot_cognifly = tfBuffer.lookupTransform(
        map_frame, "simple_quad_base_link", rospy.Time())

    cognifly_pos_x = int(((trans_cognifly[0]) - cropped_origin_x) /
                    cropped_info.info.resolution)
    cognifly_pos_y = int(((trans_cognifly[1]) - cropped_origin_y) /
                    cropped_info.info.resolution)

    # Dessine un rectangle au niveau de la position du cognifly
    cv2.rectangle(map_image, (cognifly_pos_x-1, cognifly_pos_y-1),
                (cognifly_pos_x+1, cognifly_pos_y+1), (0, 255, 0, 255), thickness=-1)

    # Encoder l'image en PNG
    ret, png = cv2.imencode('.png', map_image)

    return png
