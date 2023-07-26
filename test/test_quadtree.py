import cv2
import numpy as np

from erl_env.geometry import IncrementalQuadtree
from erl_env.geometry import NodeContainerMultiTypes
from erl_env.geometry import Node
from erl_env.geometry import Aabb2D
from erl_env.common.storage import GridMapInfo2D
from erl_env.env.data.house_expo.sequence import HouseExpoSequence
from erl_env.env.data.house_expo.list_data import get_map_and_traj_files
import vedo
import screeninfo


class QuadtreeApp:
    def __init__(self):
        vedo.settings.immediate_rendering = False
        # window top-left position on screen
        pos = None
        for monitor in screeninfo.get_monitors():
            if monitor.height > monitor.width:
                # portrait mode
                continue
            pos = [monitor.width // 2 - 200, monitor.height // 2 - 100]

        self.plt = vedo.Plotter(pos=pos)
        self.vedo_quadtree_image = None

        self.quadtree_setting = IncrementalQuadtree.Setting()
        self.quadtree_setting.min_half_area_size = 0.02
        print(self.quadtree_setting.as_yaml_string())
        self.node_container_setting = NodeContainerMultiTypes.Setting()
        self.node_container_setting.node_type_min_squared_distance[0] = 0.0004
        print(self.node_container_setting.as_yaml_string())
        self.init_area = Aabb2D(center=np.zeros(2), half_size=5.0)

        map_file, traj_file = get_map_and_traj_files()[4]
        print(map_file)
        self.sequence = HouseExpoSequence(map_file, traj_file, lidar_mode="kDdf")
        vertices = self.sequence.map.meter_space.surface.vertices
        self.map_resolution = 0.01
        self.padding = 15
        self.grid_map_info = GridMapInfo2D(
            min=np.min(vertices, axis=1),
            max=np.max(vertices, axis=1),
            resolution=np.array([self.map_resolution, self.map_resolution]),
            padding=np.array([self.padding, self.padding]),
        )

        self.quadtree = IncrementalQuadtree(self.quadtree_setting, self.init_area, self.node_container_constructor)
        self.insert_surface_points()
        self.plt.add_callback("mouse move", self.mouse_move_callback)
        self.plt.interactive().close()

    def node_container_constructor(self):
        return NodeContainerMultiTypes(self.node_container_setting)

    def insert_surface_points(self):

        cnt = 0
        traj = []
        for frame in self.sequence:
            traj.append(np.array([frame.x, frame.y]))
            for angle, r in zip(frame.angles, frame.ranges):
                theta = angle + frame.theta
                position = np.array(
                    [
                        np.cos(theta) * r + frame.x,
                        np.sin(theta) * r + frame.y,
                    ]
                )
                new_root, cluster = self.quadtree.insert(Node(0, position))
                if new_root is not None:
                    self.quadtree = new_root

            # if cnt % 10 == 0:
            #     image = self.quadtree.plot(
            #         grid_map_info=self.grid_map_info,
            #         node_types=[0],
            #         node_type_colors={0: np.array([0, 0, 255])},
            #         node_type_radius={0: 4},
            #     )
            #     to_remove = []
            #     if self.vedo_quadtree_image is not None:
            #         to_remove.append(self.vedo_quadtree_image)
            #     path = self.grid_map_info.meter_to_pixel_for_points(np.array(traj).T).T
            #     cv2.polylines(image, [path], False, (0, 255, 0), 2)
            #     # cv2.imshow("image", image)
            #     # cv2.waitKey(1)
            #     self.vedo_quadtree_image = vedo.Picture(image)
            #     to_add = [self.vedo_quadtree_image]
            #     self.plt.remove(to_remove)
            #     self.plt.show(*to_add, interactive=False)
            #     self.plt.render()
            #
            # cnt += 1

        image = self.quadtree.plot(
            grid_map_info=self.grid_map_info,
            node_types=[0],
            node_type_colors={0: np.array([0, 0, 255])},
            node_type_radius={0: 4},
        )
        to_remove = []
        if self.vedo_quadtree_image is not None:
            to_remove.append(self.vedo_quadtree_image)
        path = self.grid_map_info.meter_to_pixel_for_points(np.array(traj).T).T
        cv2.polylines(image, [path], False, (0, 255, 0), 2)
        # cv2.imshow("image", image)
        # cv2.waitKey(1)
        self.vedo_quadtree_image = vedo.Picture(image)
        to_add = [self.vedo_quadtree_image]
        self.plt.remove(to_remove)
        self.plt.show(*to_add, interactive=False)
        self.plt.render()

    def mouse_move_callback(self, event):
        self.plt.at(event.at)

        if not event.actor:
            return

        cur_pos = np.array(event.picked3d[:2])
        ray_origin = self.grid_map_info.grid_to_meter_for_points(cur_pos)
        theta = 0
        ray_dir = np.array([np.cos(theta), np.sin(theta)])
        ray_travel_distance, hit_node = self.quadtree.ray_tracing(ray_origin, ray_dir, hit_distance_threshold=0.1)
        if hit_node is not None:
            print(ray_travel_distance)


if __name__ == "__main__":
    app = QuadtreeApp()
