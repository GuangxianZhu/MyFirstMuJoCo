"""离屏渲染 + 轨道相机。返回 numpy 图像，不依赖 Qt。"""
from __future__ import annotations

import mujoco
import numpy as np


class SceneRenderer:
    def __init__(self, model: mujoco.MjModel, width: int = 960, height: int = 600, camera: dict | None = None):
        self.model = model
        self.width = width
        self.height = height
        self._renderer = mujoco.Renderer(model, height=height, width=width)

        self.camera = mujoco.MjvCamera()
        mujoco.mjv_defaultFreeCamera(model, self.camera)
        self.camera.azimuth = 135.0
        self.camera.elevation = -22.0
        self.camera.distance = 1.9
        self.camera.lookat[:] = (0.15, 0.0, 0.3)
        if camera:  # 场景可以自带初始相机
            self.camera.azimuth = camera.get("azimuth", self.camera.azimuth)
            self.camera.elevation = camera.get("elevation", self.camera.elevation)
            self.camera.distance = camera.get("distance", self.camera.distance)
            if "lookat" in camera:
                self.camera.lookat[:] = camera["lookat"]
        self._home = (self.camera.azimuth, self.camera.elevation, self.camera.distance, self.camera.lookat.copy())

        self.option = mujoco.MjvOption()

    def set_camera(self, cam: dict) -> None:
        """切到指定相机（字段同初始相机配置）；"复位视角"仍回到场景的初始相机。"""
        self.camera.azimuth = cam.get("azimuth", self.camera.azimuth)
        self.camera.elevation = cam.get("elevation", self.camera.elevation)
        self.camera.distance = cam.get("distance", self.camera.distance)
        if "lookat" in cam:
            self.camera.lookat[:] = cam["lookat"]

    def reset_camera(self) -> None:
        az, el, dist, look = self._home
        self.camera.azimuth, self.camera.elevation, self.camera.distance = az, el, dist
        self.camera.lookat[:] = look

    # ------------------------------------------------------------------ 相机
    # 相机移动自己实现，不依赖 mujoco.mjv_moveCamera（各版本 Python 绑定的参数不一致）。
    def rotate(self, dx: float, dy: float) -> None:
        """dx, dy 为渲染图像上的像素位移。"""
        self.camera.azimuth = (self.camera.azimuth - 180.0 * dx / self.height) % 360.0
        self.camera.elevation = float(np.clip(self.camera.elevation - 180.0 * dy / self.height, -89.0, 89.0))

    def pan(self, dx: float, dy: float) -> None:
        az, el = np.radians(self.camera.azimuth), np.radians(self.camera.elevation)
        forward = np.array([np.cos(el) * np.cos(az), np.cos(el) * np.sin(az), np.sin(el)])
        right = np.array([np.sin(az), -np.cos(az), 0.0])
        up = np.cross(right, forward)
        k = 0.83 * self.camera.distance / self.height  # 0.83 ≈ 2*tan(fovy/2)，fovy=45°
        self.camera.lookat[:] = self.camera.lookat - right * dx * k + up * dy * k

    def zoom(self, steps: float) -> None:
        """steps > 0 拉近，< 0 拉远。"""
        self.camera.distance = float(np.clip(self.camera.distance * (0.9 ** steps), 0.3, 12.0))

    # ------------------------------------------------------------------ 渲染
    def render(self, data: mujoco.MjData) -> np.ndarray:
        self._renderer.update_scene(data, camera=self.camera, scene_option=self.option)
        return self._renderer.render()

    def close(self) -> None:
        self._renderer.close()
