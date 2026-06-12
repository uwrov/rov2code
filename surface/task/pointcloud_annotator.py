import sys
from dataclasses import dataclass

import numpy as np
import pyvista as pv
import trimesh
import tkinter as tk
from tkinter import simpledialog
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
    override_text: str = ""

@dataclass
class Annotation:
    id: int
    p1_idx: int
    p2_idx: int
    override_cm: float | None = None

class PointCloudViewer:
    def __init__(self, data: PointCloudData):
        self.data = data
        self.cloud = pv.PolyData(data.points)
        self.cloud["rgb"] = data.colors
        self.plotter = pv.Plotter()
        self.selection = SelectionState()
        self.annotations: list[Annotation] = []
        self.next_annotation_id = 1
        self.selected_annotation_id = None
        self.global_scale_cm_per_unit: float | None = None
        self.tk_root = tk.Tk()
        self.tk_root.withdraw()
        self.tk_root.attributes("-topmost", True)
        self.setup_scene()
    def setup_scene(self):
        self.plotter.add_points(
            self.cloud,
            scalars = "rgb",
            rgb=True,
            point_size=5,
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
            picker="point",
            use_picker=True,
            show_point=False,
            show_message=False,
            tolerance=0.01,
        )
        self.plotter.add_key_event("Return", self.commit_active_annotation)
        self.plotter.add_key_event("BackSpace", self.delete_annotation)
        self.plotter.add_key_event("Delete", self.delete_annotation)
        # self.plotter.add_key_event("Return", self.commit_active_annotation)

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
        self.selection.p2_idx = None
        p = self.data.points[idx]
        # print(p)

    def set_p2(self, idx):
        if self.selection.p1_idx == None:
            return
        self.selection.p2_idx = idx
        p = self.data.points[idx]
        # print(p)
        # self.print_measurement()
    def ask_override_cm(self):
        text = simpledialog.askstring(
            "Distance override",
            "known distance in cm",
            parent=self.tk_root,
        )
        if text is None or text.strip() == "":
            return None
        try:
            value = float(text)
        except ValueError:
            return None
        return value
    def point_distance(self, p1_idx, p2_idx) -> float:
        return float(np.linalg.norm(self.data.points[p2_idx] - self.data.points[p1_idx]))
    def annotation_distance(self, ann:Annotation) -> float:
        return self.point_distance(ann.p1_idx, ann.p2_idx)
    def update_global_scale(self):
        # print("updating")
        samples = []
        for ann in self.annotations:
            if ann.override_cm is None:
                continue
            model_dist = self.annotation_distance(ann)
            if model_dist > 0:
                samples.append(ann.override_cm / model_dist)
        if samples:
            self.global_scale_cm_per_unit = float(np.mean(samples))
        else:
            self.global_scale_cm_per_unit = None
    def display_distance(self, model_distance) -> str:
        if self.global_scale_cm_per_unit is None:
            return f"{model_distance:.4g} unitless"
        return f"{model_distance * self.global_scale_cm_per_unit:.4g} cm"
    def annotation_label_text(self, ann):
        model_dist = self.annotation_distance(ann)
        label = f"A{ann.id}\nd={self.display_distance(model_dist)}"
        if ann.override_cm is not None:
            label += f"\nref={ann.override_cm:.4g} cm"
        return label
    # def print_measurement(self):
    #     p1 = self.data.points[self.selection.p1_idx]
    #     p2 = self.data.points[self.selection.p2_idx]
    #     delta = p2 - p1
    #     dist = np.linalg.norm(delta)
    #     print(f"dx={delta[0]}")
    #     print(f"dy={delta[1]}")
    #     print(f"dz={delta[2]}")
    #     print(f"d ={dist}")
    def delete_annotation(self):
        if self.selection.p1_idx is not None or self.selection.p2_idx is not None:
            self.clear_active_annotation()
            self.plotter.render()
            return()
        
        if not self.annotations:
            return
        ann = self.annotations.pop()
        ann_id = self.selected_annotation_id
        self.plotter.remove_actor(f"annotation_line_{ann_id}", render=False)
        self.plotter.remove_actor(f"annotation_points_{ann_id}", render=False)
        self.plotter.remove_actor(f"annotation_label_{ann_id}", render=False)
        self.update_global_scale()
        self.update_all_annotation_labels()
        self.plotter.render()
        

    def set_active_from_annotation(self, ann: Annotation):
        self.selection.p1_idx = ann.p1_idx
        self.selection.p2_idx = ann.p2_idx
        self.selected_annotation_id = ann.id
        self.update_active_annotation()
    
    def commit_active_annotation(self):
        if self.selection.p1_idx is None or self.selection.p2_idx is None:
            return
        override_cm = self.ask_override_cm()
        current_annotation = Annotation(
            id = self.next_annotation_id,
            p1_idx=self.selection.p1_idx,
            p2_idx=self.selection.p2_idx,
            override_cm=override_cm,
        )
        self.annotations.append(current_annotation)
        self.next_annotation_id +=1
        self.update_global_scale()
        self.draw_annotation(current_annotation)
        self.update_all_annotation_labels()
        # self.selected_annotation_id = current_annotation.id
        self.clear_active_annotation()
        self.plotter.render()

    def clear_active_annotation(self):
        self.selection = SelectionState()
        self.plotter.remove_actor("p1_marker", render=False)
        self.plotter.remove_actor("p2_marker", render=False)
        self.plotter.remove_actor("active_line", render=False)
        self.plotter.remove_actor("active_label", render=False)
    def draw_annotation_label(self, ann):
        p1 = self.data.points[ann.p1_idx]
        p2 = self.data.points[ann.p2_idx]
        mid = (p1+p2)/2
        self.plotter.remove_actor(f"annotation_label_{ann.id}", render=False)
        self.plotter.add_point_labels(
            np.array([mid]),
            [self.annotation_label_text(ann)],
            name=f"annotation_label_{ann.id}",
            font_size=14,
            point_size=0,
            shape_opacity=0.5,
            always_visible=True,
            pickable=False,
        )
    def update_all_annotation_labels(self):
        # print(self.global_scale_cm_per_unit)
        for ann in self.annotations:
            self.draw_annotation_label(ann)
    def draw_annotation(self, ann: Annotation):
        p1 = self.data.points[ann.p1_idx]
        p2 = self.data.points[ann.p2_idx]
        self.plotter.add_lines(
            np.array([p1, p2]),
            color="yellow",
            width=4,
            name=f"annotation_line_{ann.id}",
            connected=True,
        )
        self.plotter.add_points(
            pv.PolyData(np.array([p1, p2])),
            color="yellow",
            point_size=12,
            render_points_as_spheres=True,
            name=f"annotation_points_{ann.id}",
            pickable=False,
        )
        self.draw_annotation_label(ann)

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
        dist = np.linalg.norm(p2-p1)
        mid = (p1+p2)/2
        label = (
            f"d ={dist}\n"
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