"""World (meters, y-up) <-> pixel (y-down) mapping shared by the cv2-based
tools (visualize_trajectory.py, edit_walls.py)."""

PADDING_M = 1.0
MAX_CANVAS_PX = 1100


class View:
    def __init__(self, xs, ys, padding_m=PADDING_M, max_canvas_px=MAX_CANVAS_PX):
        if not xs:
            xs, ys = [0.0, 1.0], [0.0, 1.0]
        min_x, max_x = min(xs) - padding_m, max(xs) + padding_m
        min_y, max_y = min(ys) - padding_m, max(ys) + padding_m
        width_m, height_m = max(max_x - min_x, 0.1), max(max_y - min_y, 0.1)

        self.scale = max_canvas_px / max(width_m, height_m)
        self.min_x, self.min_y = min_x, min_y
        self.canvas_w = max(int(width_m * self.scale), 1)
        self.canvas_h = max(int(height_m * self.scale), 1)

    def to_px(self, x: float, y: float):
        return (int((x - self.min_x) * self.scale), int(self.canvas_h - (y - self.min_y) * self.scale))

    def to_world(self, px: float, py: float):
        x = px / self.scale + self.min_x
        y = (self.canvas_h - py) / self.scale + self.min_y
        return x, y
