import sys
from dataclasses import dataclass

import numpy as np
import pyvista as pv
import trimesh

@dataclass
class PointCloudData:
    points: np.ndarray
    colors: np.ndarray


def load_glb_pointcloud(path:str) -> PointCloudData:
    scene = trimesh.load(path, force="scene")

    point_blocks = []
    color_blocks = []
    for geom in scene.dump(concatenate=False):
        if not hasattr(geom, "vertices"):
            continue
        points = np.asarray(geom.vertices)
        if len(points) == 0:
            continue
        colors = None
        if hasattr(geom, "colors") and geom.colors is not None:
            colors = np.asarray(geom.colors)
        elif hasattr(geom, "visual") and hasattr(geom.visual, "vertex_colors"):
            colors = np.asarray(geom.visual.vertex_colors)
        
        if colors is None or len(colors) != len(points):
            colors = np.full((len(points), 3),255, dtype=np.uint8)
        point_blocks.append(points)
        color_blocks.append(colors[:,:3])
    
    points = np.ascontiguousarray(np.vstack(point_blocks), dtype=np.float64)
    colors = np.ascontiguousarray(np.vstack(color_blocks))

    return PointCloudData(points=points, colors = colors)

class PointCloudViewer:
    def __init__(self, data: PointCloudData):
        self.data = data
        self.cloud = pv.PolyData(data.points)
        self.cloud["rgb"] = data.colors
        self.plotter = pv.Plotter()
        self.selected_idx = None
        self.setup_scene()
    def setup_scene(self):
        self.plotter.add_points(
            self.cloud,
            scalars = "rgb",
            rgb=True,
            point_size=3,
            render_points_as_spheres=False,
            name="cloud",
            pickable=True,
        )
        self.plotter.add_axes()
        #self.plotter.show_bounds(grid="front",location="outer")

        self.plotter.enable_custom_trackball_style(
            left="pan",
            middle="pan",
            right="rotate",
            shift_left="pan",
            control_right="pan"
        )

        self.plotter.enable_point_picking(
            callback = self.on_pick,
            left_clicking=True,
            pickers="point",
            use_picker=True,
            show_point=False,
            show_message=False,
            tolerance=0.01,
        )
    
    def on_pick(self, picked_point, picker):
        idx = picker.GetPointId()
        if idx < 0:
            return
        self.selected_idx = idx
        point = self.data.points[idx]
        # color = self.data.colors[idx]
        # print(f"color={color}")
        self.show_selected_point(point)

    def show_selected_point(self, point):
        self.plotter.remove_actor("selected_point", render=False)
        marker = pv.PolyData(np.array([point]))
        self.plotter.add_points(
            marker,
            color="red",
            point_size=12,
            render_points_as_spheres=True,
            name="selected_point",
            pickable=False,
        )
        self.plotter.render()
    def run(self):
        self.plotter.show()
    

def main():
    path = sys.argv[1] if len(sys.argv) > 1 else "scene.glb"
    data  = load_glb_pointcloud(path)
    viewer = PointCloudViewer(data)
    viewer.run()

if __name__ == "__main__":
    main()