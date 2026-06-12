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

@dataclass
class SelectionState:
    p1_idx: int | None = None
    p2_idx: int | None = None

class PointCloudViewer:
    def __init__(self, data: PointCloudData):
        self.data = data
        self.cloud = pv.PolyData(data.points)
        self.cloud["rgb"] = data.colors
        self.plotter = pv.Plotter()
        self.selection = SelectionState()
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
    def is_shift_pressed(self) -> bool:
        iren = self.plotter.iren
        interactor = getattr(iren, "interactor", iren)
        return bool(interactor.GetShiftKey())
    
    def on_pick(self, picked_point, picker):
        idx = picker.GetPointId()
        if idx < 0:
            return
        if self.is_shift_pressed():
            self.set_p2(idx)
        else:
            self.set_p1(idx)
        self.update_active_annotation()

    def set_p1(self, idx):
        self.selection.p1_idx = idx
        self.selection.P2_idx = None
        p = self.data.points[idx]
        # print(p)

    def set_p2(self, idx):
        if self.selection.p1_idx == None:
            return
        self.selection.p2_idx = idx
        p = self.data.points[idx]
        # print(p)
        self.print_measurement()
    
    def print_measurement(self):
        p1 = self.data.points[self.selection.p1_idx]
        p2 = self.data.points[self.selection.p2_idx]
        delta = p2 - p1
        dist = np.linalg.norm(delta)
        print(f"dx={delta[0]}")
        print(f"dy={delta[1]}")
        print(f"dz={delta[2]}")
        print(f"d ={dist}")

    def update_active_annotation(self):
        self.plotter.remove_actor("p1_marker", render=False)
        self.plotter.remove_actor("p2_marker", render=False)
        self.plotter.remove_actor("active_line", render=False)
        self.plotter.remove_actor("active_label", render=False)

        if self.selection.p1_idx is None:
            self.plotter.render()
            return
        p1 = self.data.points[self.selection.p1_idx]
        self.plotter.add_points(
            pv.PolyData(np.array([p1])),
            color="red",
            point_size=12,
            render_points_as_spheres=True,
            name="p1_marker",
            pickable=False,
        )
        if self.selection.p2_idx is None:
            self.plotter.render()
            return
        p2 = self.data.points[self.selection.p2_idx]
        self.plotter.add_points(
            pv.PolyData(np.array([p2])),
            color="blue",
            point_size=12,
            render_points_as_spheres=True,
            name="p2_marker",
            pickable=False,
        )
        self.plotter.add_lines(
            np.array([p1, p2]),
            color="red",
            width=4,
            name="active_line",
            connected=True,
        )
        delta = p2 - p1
        dist = np.linalg.norm(delta)
        mid = (p1+p2)/2
        label = (
            f"d ={dist}\n"
            f"dx={delta[0]}\n"
            f"dy={delta[1]}\n"
            f"dz={delta[2]}\n"
        )
        self.plotter.add_point_labels(
            np.array([mid]),
            [label],
            name = "active_label",
            font_size = 14,
            point_size=0,
            shape_opacity=0.5,
            always_visible=True,
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