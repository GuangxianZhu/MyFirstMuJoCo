# 下一阶段开发指示：阶段 B（桌子 + 物体 + 抓取指标 + 两个抓取演示）

> 写给接手开发的模型/人。**先通读第 0～2 节再动手**。每个任务（B1～B7）都写明了：做什么、改哪些文件、怎么做、验收标准。
> 标"**实测**"的数字是我在本仓库代码上跑出来的；标"**建议**"的是起点，请自己测了再定，**不要为了让结果好看去调参数**，测出来是什么就如实写进总结。

---

## 0. 目标与范围

**目标**：让手能"抓起桌上的物体"，并能**量化**抓得怎么样；用两个演示把"阻抗控制的好处"讲清楚。

| 编号 | 交付物 | 一句话 |
|---|---|---|
| B1 | 桌子 + 物体场景 | 新场景文件 `hand_table.xml`，不改动现有 `hand.xml` |
| B2 | 抓取指标 `GraspMonitor` + 界面"抓取"页签 | 物体位移/打滑、各指和总握力、接触统计、抓取成功判定 |
| B3 | 捏取序列器 `PinchGraspSequence` | 靠近→下降→闭合→抬起→搬运→放下→松开 的自动状态机 |
| B4 | 演示 3：轻拿轻放易碎物 | 柔顺 vs 用力，易碎物会"碎" |
| B5 | 演示 4：位置控制 vs 阻抗控制抓物体 | 有**决策门**，见 B5 |
| B6 | 界面接线 | 抓取页签、场景切换、相机 |
| B7 | 测试 / README / 素材 / 提交 | 见 B7 |

**不在本阶段做**（别顺手做，留给后面）：手腕键盘/鼠标拖动与预设轨迹（M4）、手动分步抓取界面与包握（M5）、运动中的抓握（M6）、录制回放、视频导出、单片机逻辑。

**硬性约束**
1. 仿真核心（`app/core/`）**不许 import Qt**；能无界面批量跑。
2. **不改** `hand.xml` 里已有的关节名、执行器名、几何体的碰撞设置；现有测试必须全部继续通过（命令见 B7）。
3. 界面文字、注释、提交信息用中文，和现有代码一致。
4. 每完成一个任务就提交一次（`git commit`，中文提交信息），最后推送到 `main`。

---

## 1. 现状速览（接手前必须知道的）

仓库：`GuangxianZhu/MyFirstMuJoCo`，Python + MuJoCo 3.x + PySide6 + pyqtgraph。运行：`python main.py`。

### 1.1 模块地图

| 文件 | 作用 | 本阶段会碰吗 |
|---|---|---|
| `app/core/sim.py` | `Simulation`：加载模型、按真实时间推进、**控制链**、演示接口 | 要加 `grasp` 属性 |
| `app/core/tactile.py` | `TactileSensor.read(data, with_points)` → `TactileReading`（16 段法向力/三维力/接触数） | 要加"只统计与某物体的接触" |
| `app/core/impedance.py` | `JointImpedance`：`τ = K(目标−角) − D·角速度`，限幅；`PRESETS` 三组预设 | 只调用 |
| `app/core/controllers/manual.py` | `ManualController`：`targets`（按滑块顺序）、`set_target`、`move_to` | 只调用，**注意陷阱 T1** |
| `app/core/demo.py` | `Demo` 基类 | 只继承 |
| `app/core/scope.py` | 示波器环形缓冲 | 参考其写法 |
| `app/scenes/hand/hand.xml` | 手模型（**不改**） | 被 include |
| `app/scenes/hand/hand_extras.py` | 手势/协同/触觉段/阻抗默认值/演示注册 `_demos()` | 要复用 |
| `app/scenes/hand/demos.py` | 现有两个演示（`PressDemo`、`ImpedanceCompareDemo`） | 参考写法 |
| `app/ui/main_window.py` | 主窗口，页签 触觉/曲线/演示 | 加"抓取"页签 |
| `docs/prototypes/pinch_cube_proto.py` | **可运行的捏取方块原型**（本阶段 B3 的参考实现） | 先跑一遍 |

### 1.2 控制链（每个控制周期，默认 4 ms；物理步长 2 ms）

```
Simulation._control_tick():
  demo.update(sim, t)            # 演示脚本：在这里改目标/阻抗参数
  controller.update(obs)         # 手动/节奏控制器 → 目标角度 targets（按滑块顺序）
  drive.compute(targets,q,qd)    # 手指：力矩 = K(目标−角)−D·角速度，限幅；手腕：位置执行器，透传
  data.ctrl[...] = cmd
  tactile_reading = tactile.read(data)   # 注意：demo.update 读到的是"上一周期"的触觉
  scope.push(...)
```
你的 `GraspMonitor.update()` 要挂在 `tactile.read` 之后。

### 1.3 关节与执行器（滑块顺序 = `sim.actuators` 顺序）

- 手腕 6 个位置执行器：`wrist_x/y`（±0.3 m）、`wrist_z`（−0.295 ~ +0.30 m，**世界高度 = 0.30 + wrist_z**）、`wrist_yaw/pitch/roll`（弧度）。零位：手指沿 +x，**手掌朝下**，拇指在 +y 侧。
- 手指 20 个**力矩**执行器（`motor`，ctrlrange ±1）：`{index,middle,ring,little}_{spread,mcp,pip,dip}`、`thumb_{rot,cmc,mcp,ip}`。按名字取下标：`sim.joint_index("index_mcp")`；写目标：`sim.manual.targets[k] = 弧度`。
- 弯曲轴 +y，正方向 = 朝手掌弯曲。手指弯曲度 c(0~1) ↔ 关节角：`hand_extras.curl_pose(finger, c)`。
- 触觉段：`palm` + `{thumb,index,middle,ring,little}_{prox,mid,dist}`，`sim.tactile.names` 给出顺序；**统计的是"该段刚体上的所有接触"，不区分对方是地面、桌子还是物体**（所以要抓取指标就得按对方过滤，见 B2）。

### 1.4 手的几何（做捏取/放置计算要用）

- 指尖是胶囊：食指 `index_dist` 胶囊轴长 0.024 m、半径 0.0077 m；拇指 `thumb_dist` 轴长 0.026 m、半径 0.0085 m。
- ⚠️ **`index_tip` / `thumb_tip` 这两个 site 在胶囊最外端（球面顶点），不是半球球心**。算接触距离必须用**球心** = `body.xpos + body.xmat[:,0] * 轴长`（原型里的 `tip_centers`）。这个坑我踩过：用 site 算会偏 ~8 mm，捏球捏不到。
- 手部几何体 `contype=1 conaffinity=0`：手内部互不碰撞，只和 `conaffinity` 含 1 的东西碰撞（地面、桌子、物体都必须设 `contype="1" conaffinity="1"`）。

---

## 2. 陷阱清单（都是真实踩过的）

| # | 陷阱 | 怎么避免 |
|---|---|---|
| **T1** | `ManualController.set_target()` 会**取消**正在进行的 `move_to`（整体姿态过渡）。如果你每个周期都 `set_target(wrist_z)`，同时调了 `set_pose(..., duration>0)` 想转手腕，过渡会被一直取消。`PressDemo` 里踩过。 | 一个演示里**要么**全用 `set_pose(duration)` 做过渡，**要么**全自己每周期写 `sim.manual.targets[k]`（并自己插值）。序列器用后者。 |
| **T2** | `sim.reset()` 调 `mj_resetData`：模型里的自由物体回到 XML 里写的初始位姿，mocap 回到 XML 位置。演示靠这一点保证可重复。 | 物体的初始位置写在 XML 里；演示 `setup()` 里不要假设上一次演示的残留。 |
| **T3** | 方块在指尖间会**绕捏取轴转动**然后掉下去，因为默认 `condim=3` 没有扭转摩擦。 | 物体 geom 必须 `condim="4"` 且 `friction="1.0 0.02 0.0001"`（第二项是扭转摩擦系数）。**实测**：`condim=3` 抬不起来；`condim=4, 0.02` 抬起并保持；`condim=6` 反而失败，别用。 |
| **T4** | 球放在平桌面上，被指尖一碰就滚走，捏不住（**实测**：6 组参数全部失败）。 | 见 B5 决策门。 |
| **T5** | 触觉读数包含手与桌面/地面的接触。 | 抓取指标必须只统计"手 ↔ 目标物体"的接触（B2）。 |
| **T6** | 自由物体会自动出现在"物理参数 → 连杆质量"面板里（该面板按"质量>0、非世界固定、有几何"过滤刚体）。这是好事（可调物体质量），但物体的 `mass` 要写在 geom 或 inertial 上，别写成 0。 | 验收时打开面板确认物体在列。 |
| **T7** | 手指靠"关节力矩 = K·误差"出力，`s` 到 1.0 后再要更大的力是要不到的。**实测**：默认阻抗下 `s=1` 时每指最大约 2.25 N。 | 要更大握力：换预设（刚硬）或把姿态 B 做得更"深"。 |
| **T8** | 无头环境渲染要环境变量：`MUJOCO_GL=egl PYOPENGL_PLATFORM=egl EGL_PLATFORM=surfaceless`，界面测试再加 `QT_QPA_PLATFORM=offscreen`。 | 写进你的命令里。Windows 有显示器时不需要。 |
| **T9** | 速度快就容易把东西弹飞（**实测**：闭合速度太快/先碰的指尖把球打到 2 米外）。 | 闭合用"力终止 + 低速"（s 速率 0.15/s 起），下降速度 ≤ 0.04 m/s。 |

---

## 3. 任务 B1：桌子 + 物体场景

### 做什么
新增场景 `app/scenes/hand/hand_table.xml`（桌子 + 若干物体），配套 `hand_table_extras.py`。**不要改 `hand.xml`**，原因：现有演示/测试都假设手下面是地面。

### 怎么做
1. `hand_table.xml` 用 MJCF `<include>` 复用手模型。**已验证可行**（沙箱里 MuJoCo 3.14 加载成功，执行器/关节数不变）：
   ```xml
   <mujoco model="hand_table">
     <include file="hand.xml"/>
     <worldbody>
       <!-- 桌面高度 0.10 m；x 方向 -0.25~0.45，y 方向 ±0.35 -->
       <geom name="table" type="box" size="0.35 0.35 0.05" pos="0.10 0 0.05"
             material="grid" contype="1" conaffinity="1" friction="1.0 0.005 0.0001"/>
       <!-- 物体：见下表。每个物体一个自由刚体，名字固定 -->
       <body name="cube" pos="0.15 0 0.1155">
         <freejoint name="cube_free"/>
         <geom name="cube_geom" type="box" size="0.015 0.015 0.015" mass="0.03" rgba="0.9 0.35 0.3 1"
               contype="1" conaffinity="1" condim="4" friction="1.0 0.02 0.0001"/>
       </body>
     </worldbody>
   </mujoco>
   ```
   （`material="grid"` 在 `hand.xml` 里定义，include 之后可直接用。）
2. 物体清单（**建议**，质量上限见 T7：**实测** 0.03 kg 能抓起，0.06 kg 在 F=1.5 N 下失败）：

   | body 名 | 形状 | 尺寸 | 质量 | 初始位置 (x,y) | 备注 |
   |---|---|---|---|---|---|
   | `cube` | 方块 | 半边长 0.015 | 0.03 kg | (0.15, 0.0) | B3 的基线物体、B5 的备选，原型已验证 |
   | `fragile` | 方块（黄色，外观区别于 cube） | 半边长 0.015 | 0.03 kg | (0.15, 0.12) | B4 的易碎物；"碎"是 Python 里判定，不是物理破碎（B2/B4） |
   | `ball` | 球 | 半径 0.02 | 0.03 kg | 由 B5 决定 | 仅 B5 用，见决策门 |

   所有物体 geom 都要 `contype="1" conaffinity="1" condim="4" friction="1.0 0.02 0.0001"`（T3）。
3. 物体初始 z = 桌面 0.10 + 半尺寸 + 0.0005（防止初始穿插）。
4. `hand_table_extras.py`：
   ```python
   from app.scenes.hand.hand_extras import create as _create_hand
   def create():
       ex = _create_hand()
       ex.demos = [...]            # B4、B5 完成后填；先保留 hand_extras 里原有两个
       ex.grasp = GraspConfig(...)  # B2 定义的配置（物体名、易碎阈值等）
       return ex
   ```
   `scene_extras.SceneExtras` 要新增字段 `grasp: dict | None = None`（用 dataclass 的 `field`/默认 None，不影响旧场景）。
5. 相机：沿用 `hand_extras` 的相机即可，桌面场景把 `lookat` 的 z 调到 ~0.18，`distance` ~0.55。

### 验收
- `python main.py --model app/scenes/hand/hand_table.xml` 能打开，桌子和方块可见，手能手动移到方块上方。
- `tests/test_table_scene.py`（新）：加载成功；26 个执行器；方块静止 2 s 后位置漂移 < 0.5 mm 且 z ≈ 0.1155（±0.5 mm）；`sim.reset()` 后方块回到初始位姿（T2）；方块出现在 `sim.physics` 质量组里（T6）。

---

## 4. 任务 B2：抓取指标 `GraspMonitor` + "抓取"页签

### 做什么
新文件 `app/core/grasp.py`（不依赖 Qt）。每个控制周期统计"手 ↔ 目标物体"的接触，给出下面的指标和一个**判定状态**。

### 怎么做

**4.1 只统计与目标物体的接触（T5）** —— 在 `tactile.py` 里新增方法，不要改现有 `read` 的行为：
```python
def read_against(self, data, body_ids: set[int], with_points=False) -> TactileReading:
    """同 read，但只统计"手的某段 ↔ body_ids 里某个刚体"的接触。"""
```
实现：复用 `read` 的循环，在算力之前判断"另一方的 `geom_bodyid` ∈ body_ids"，否则 `continue`。物体若有多个 geom，用该物体刚体 id（`mj_name2id(m, mjOBJ_BODY, "cube")`）。**要多个刚体时传集合**。

**4.2 `GraspMonitor`**
```python
class GraspMonitor:
    def __init__(self, sim, object_body: str, fragile_limit: float | None = None): ...
    def reset(self): ...                 # sim.reset() 时调用（Simulation.reset 里加一行）
    def update(self) -> None: ...        # 每个控制周期调用一次
    reading: TactileReading              # 最近一次"手↔物体"触觉
    def metrics(self) -> dict: ...       # 见下表，所有值都是 float/int/str/bool，便于界面和测试直接用
    state: str                           # 判定状态，见 4.3
    history(seconds) -> dict[str, ndarray]   # 简单环形缓冲（容量 6000 条），键同 metrics 里的数值项 + "t"
```
`Simulation` 里：`self.grasp: GraspMonitor | None`，由 `extras.grasp`（`{"object": "cube", "fragile_limit": None}`）创建；`_control_tick` 里在 `tactile.read` 之后 `if self.grasp: self.grasp.update()`；`reset()` 里 `self.grasp.reset()`；加 `sim.set_grasp_object(name, fragile_limit=None)` 供演示切换物体。

**4.3 指标定义（务必按这个算，测试按这个写）**

| 指标 | 键 | 定义 |
|---|---|---|
| 物体位置/速度 | `obj_pos`、`obj_speed` | `data.xpos[body]`；速度取 `data.cvel` 或相邻周期差分（选一种，写进注释） |
| 抬升高度 | `lift` | `z_obj − z_rest`，`z_rest` 是 `reset()` 后的初始 z |
| 总握力 | `grip_total` | `reading.total_normal()`（手各段对物体的法向力之和，N） |
| 各指握力 | `grip_thumb`、`grip_index`、`grip_middle`、`grip_ring`、`grip_little`、`grip_palm` | `reading.group_normal(prefix)` |
| 接触统计 | `n_contacts`、`n_segments`、`fingers_touching` | 接触点总数；有接触的触觉段数；有接触的手指/掌心名字列表 |
| 位移 | `displacement` | `|p_obj(t) − p_obj(t_ref)|`，`t_ref` = "建立抓取"时刻（见下） |
| **打滑** | `slip` | 物体在**掌坐标系**里的位移：`p_rel = R_palmᵀ·(p_obj − p_palm)`，`slip = |p_rel(t) − p_rel(t_ref)|`（米）。手带着物体一起动不算打滑；物体相对手移动才算 |
| 打滑速度 | `slip_rate` | `slip` 的 50 ms 滑动平均导数 (m/s) |
| 易碎超限 | `broken` | 见 4.4 |

"建立抓取"`t_ref`：第一次满足 `grip_total ≥ 0.5 N` 且 `len(fingers_touching) ≥ 2`，且持续 ≥ 50 ms。之前 `displacement/slip = 0`。`reset()` 后清零。

**判定状态 `state`**（互斥，按优先级从上到下判）：

| 状态 | 条件 |
|---|---|
| `碎了` | `broken` 为真（只对有 `fragile_limit` 的物体），一旦成立保持到 reset |
| `滑落` | 曾经 `lift ≥ 0.02`，现在 `lift < 0.01` 且 `grip_total < 0.1` 持续 ≥ 0.2 s |
| `抓牢` | `lift ≥ 0.05` 且 最近 1.0 s 内 `slip ≤ 0.004` 且 `grip_total ≥ 0.5` 且 `len(fingers_touching) ≥ 2` |
| `抬起` | `lift ≥ 0.02` 且 `grip_total ≥ 0.1` |
| `接触` | `grip_total ≥ 0.05` |
| `未接触` | 其余 |

> 阈值（0.5 N、0.02/0.05 m、4 mm、1.0 s…）集中放在文件顶部的一个 dataclass `Thresholds` 里，别散落在代码里。

**4.4 易碎判定**：`fragile_limit`（N）= **单个手指对物体的法向力**上限（取 `max(grip_thumb, grip_index, …)`）。超过 `fragile_limit` **持续 ≥ 20 ms** 才算碎（过滤接触瞬间的尖峰）。碎了之后：`broken=True` 锁存，并把物体 geom 的 `model.geom_rgba` 改成红色（`reset()` 或演示 `teardown` 时还原成原色，**保存原色**）。**建议**易碎阈值 1.8 N（实测默认阻抗、`s=1` 的最大握力 2.25 N，> 1.8；轻抓 1.0~1.5 N 能抬起）。

**4.5 界面"抓取"页签**（新文件 `app/ui/grasp_view.py`，`main_window` 里加页签，只在 `sim.grasp` 不为 None 时出现）：
- 顶部一行大字：`state`（用颜色区分：抓牢=绿、碎了/滑落=红、其余=灰/黄）。
- 左：数字表（总握力、各指握力、抬升高度 mm、位移 mm、打滑 mm、打滑速度 mm/s、接触点数、接触手指）。
- 右：pyqtgraph 曲线（最近 5 秒）：各指握力 + 总握力；若有 `fragile_limit` 画一条水平虚线。
- 刷新 ~15 Hz（和曲线页签一致，页签不可见时不刷新）。
- 一个"重置物体"按钮：`sim.reset()`（不要只摆物体）。

### 验收（写进 `tests/test_grasp.py`，无界面）
1. `read_against`：手压地面时 `read_against({cube})` 全 0；把方块直接放在指尖下方压住时有力且和 `read` 的对应段一致。
2. 用 `docs/prototypes/pinch_cube_proto.py` 的流程（或 B3 完成后的序列器）抓起方块：最终 `state == "抓牢"`，`lift ≥ 0.05`，`slip ≤ 0.004`，`grip_index ≈ grip_thumb`（相差 < 15%）。
3. **负例**：把方块 `condim` 改成 3（`model.geom_condim`）重做同样流程 → 最终 `state` 是 `滑落`（或至少不是 `抓牢`）。
4. 易碎：同流程用刚硬预设 + 3 N 目标力 → `state == "碎了"` 且 `geom_rgba` 变红；`reset()` 后还原。
5. `t_ref` 之前 `slip == 0`；`reset()` 后所有累计量清零。

---

## 5. 任务 B3：捏取序列器 `PinchGraspSequence`

### 做什么
把原型 `docs/prototypes/pinch_cube_proto.py` 的流程写成正式模块 `app/scenes/hand/grasp_sequence.py`（手专用，所以放 scenes 而不是 core），供 B4、B5 的演示复用。**先把原型跑一遍，看懂再写。**

### 原型的**实测**结论（方块 3 cm、0.03 kg、桌面 0.10 m、默认阻抗）
| 实验 | 结果 |
|---|---|
| 基线（`F_T=1.5 N`、闭合速度 0.15/s、`condim=4`、扭转 0.02） | 抬升 82.7 mm 并保持，指尖力 1.45/1.45 N，`s_c=0.65`，手腕目标 (0.033, −0.048, −0.113) |
| `condim=3` | 抬不起来（方块在指尖间转动） |
| 方块位置 (0.12,0.02)/(0.18,−0.02)/(0.20,0)/(0.10,−0.04) | 全部成功（工作空间够用） |
| `F_T = 0.6 N` | 失败（力不够，抬不起） |
| `F_T = 1.0 N` | 成功（抬 81.2 mm） |
| `F_T = 3.0 N` | 成功，但 `s` 走到 1.0，实际最大力 2.25 N（T7） |
| 质量 0.06 kg / 0.12 kg（`F_T=1.5`） | **失败**（要更大握力或更深的姿态 B） |

### 怎么做
**5.1 手指姿态族**（原型里的 `pinch_pose(s)`）：用一个参数 `s∈[0,1]` 在 A（张开）和 B（捏紧）之间线性插值。
- 食指：`c = 0.30 + 0.25·s`，三个关节取 `curl_pose("index", c)`。
- 拇指：`opp = 0.50 + 0.35·s`，`thumb_rot = 1.026·opp`、`thumb_cmc = 0.628·opp`、`thumb_mcp = 0.312·opp`、`thumb_ip = 0.392·opp`（比例来自 `hand_extras.OK_THUMB`）。
- 中/无名/小指：`mcp 0.1, pip 0.1, dip 0.05`（稍收起，避免碰桌面）。

**5.2 捏取点与手腕位置**（正运动学，用一份草稿 `MjData`，不要动 `sim.data`）：
1. 接触时两指尖**球心**距离 `gap = 物体宽度 + 0.0077 + 0.0085`。在 `s∈[0,1]`（101 点）里找 `|tipA − tipB|` 最接近 `gap` 的 `s_c`。
2. `M = (tipA + tipB)/2`（球心中点）。手腕零位下 `M` 的位置已知，要让 `M` 落在物体中心：`wrist_xyz = obj_center − M`。（`wrist_z` 的目标就是这个差，因为 `wrist_z` 零位对应世界高度 0.30，FK 草稿里手腕就在零位。）
3. 方块要**转到捏取轴方向**：`yaw = atan2((tipB−tipA)_y, (tipB−tipA)_x)`，让两个受力面的法线沿指尖连线（方块是在 `setup()` 里设自由关节的四元数；球不需要）。

**5.3 状态机**（`update(sim, t)` 每周期调用；**每周期直接写 `sim.manual.targets`，不用 `set_pose`（T1）**）：

| 状态 | 做什么 | 进入下一状态的条件 | 失败条件 |
|---|---|---|---|
| `APPROACH` | 手指置 `s=0`；手腕到 `(wx, wy, wz+0.06)`，用最小加加速度/smoothstep 插值，速度 ≤ 0.10 m/s | 手腕实际位置误差 < 2 mm 且等 0.3 s | 10 s 超时 |
| `DESCEND` | `wrist_z` 以 0.04 m/s 降到 `wz` | 到位 | 手的**非指尖**段（palm、mid、prox）对桌子法向力 > 0.3 N → 失败（说明位置不对） |
| `CLOSE` | `s` 以 `close_rate`（默认 0.15/s）增大，直到 `min(食指力, 拇指力) ≥ F_target` | 力达标，保持 `s` 不变 | `s` 到 1.0 仍不达标，或 6 s 超时 |
| `SETTLE` | 等 0.4 s，让力稳定 | — | 力 < 0.5·F_target → 失败 |
| `LIFT` | `wrist_z` 以 0.08 m/s 升 `lift`（默认 0.10 m） | 到位 | 状态变成 `滑落` / `碎了` |
| `CARRY`（可选） | `wrist_x/y` 以 ≤ 0.10 m/s 平移到放置点 | 到位 | 同上 |
| `LOWER`（可选） | 以 0.04 m/s 降到放置高度 | 物体—桌面接触力 ≥ 0.1 N（落地） | 同上 |
| `RELEASE` | `s` 以 0.30/s 减小 | 手指对物体的力 < 0.05 N | 4 s 超时 |
| `RETREAT` | 升 0.06 m | 到位 → `DONE` | — |

`F_target` 的力用 B2 的 `sim.grasp.reading`（只含手↔物体）取食指/拇指。参数集中成 dataclass `PinchParams(F_target, close_rate, lift, lift_speed, carry_speed, place_xy=None)`。序列器对外：`start(sim, object_body, params)`、`update(sim, t)`、`state`、`failed: str | None`、`done: bool`。

### 验收（`tests/test_pinch_sequence.py`）
- 基线参数下序列跑完到 `DONE`，B2 的最终 `state` 在 LIFT 结束时是 `抓牢`。
- 方块放 4 个不同位置都成功（用上表的位置）。
- `F_target=0.6` → `failed` 不为空且信息可读（如"力不够，物体没抬起"）；质量 0.12 kg → 失败而不是卡死/崩溃。
- 序列器**不改** `sim.drive` 的参数（阻抗预设由演示负责，序列器只管运动和力终止）。
- 任何失败路径都不抛异常，演示能收尾（teardown 还原）。

---

## 6. 任务 B4：演示 3「轻拿轻放易碎物」

### 做什么
`app/scenes/hand/demos.py` 新增 `FragilePickPlaceDemo`（`key="fragile"`），在 `hand_table_extras.demos` 里注册。

### 怎么做
- 场景：`fragile` 方块初始在 A 点 (0.15, 0.12)（`cube` 在 (0.15, 0.0) 不动；搬运时手抬到 0.10 m，指尖离 cube 顶面约 7 cm，不会碰），目标放置点 B = (0.15, −0.12)。B 点在桌上画一个浅色标记 geom（`contype="0" conaffinity="0"`，无碰撞，只表示放置区）。
- 共 **两次试验**，依次进行，演示内部用一个"试验索引"切换（参考 `ImpedanceCompareDemo` 的按 `t // trial_len` 切换；但本演示每次试验时长不固定，改用状态：`TRIAL_1 → RESET_OBJECT → TRIAL_2`）：
  1. **轻拿轻放**：阻抗预设 `PRESETS[0]`（柔顺），`F_target = 1.2 N`。
  2. **用力抓**：预设 `PRESETS[2]`（刚硬），`F_target = 3.0 N`（**建议**，目标是越过易碎阈值 1.8 N）。
- 试验之间要把物体放回 A 点并把手复位：**不能调 `sim.reset()`**（它会把演示状态也重置）。做法：直接写物体自由关节的 `qpos/qvel`（`data.qpos[adr:adr+7]`，四元数 `[1,0,0,0]`，速度清零）+ 手腕/手指目标回到初始，再 `mj_forward`；并让 `GraspMonitor.reset()`。初始位姿取 `model.qpos0[adr:adr+7]`（就是 XML 里写的位置，不要硬编码）。把这段写成 `sim.reset_object(name)` 方法放进 `Simulation`（B2 的 `set_grasp_object` 旁边）。
- 每次试验的序列：`PinchGraspSequence`（含 CARRY/LOWER/RELEASE，放置点 B）。
- 记录：每次试验 `peak_finger_force`、是否 `碎了`、`state` 终态、放置位置误差 `|obj_xy − B_xy|`、落地冲击 `peak_table_force`（物体—桌面接触法向力峰值，落地后 0.3 s 内；用 `mj_contactForce` 对 `geom_table × geom_obj` 的接触，写个小辅助函数）、总耗时。
- `summary()`：每次试验一行：`轻拿轻放（柔顺, 目标1.2N）：成功，峰值握力 x.xx N，放置误差 x.x mm，落地冲击 x.xx N，用时 x.x s`；失败/碎了要写原因。
- `result_curves()`：两次试验各自的 `grip_total` 对"试验内时间"的曲线，再加一条水平线 `易碎阈值 1.8 N`（常数曲线）。
- `camera`：`{"azimuth": 135, "elevation": -22, "distance": 0.55, "lookat": (0.15, -0.05, 0.16)}`（建议，自己看画面调）。
- `teardown`：还原阻抗预设（`sim.drive.reset()`）、物体颜色、放回物体。

### 验收（`tests/test_demo_fragile.py`）
- 两次试验都能收尾；`demo.finished` 为真且总仿真时间 < 60 s。
- 第 1 次：不碎、`state` 为"抓牢"（LIFT 结束时）、放置误差 < 8 mm。
- 第 2 次：峰值握力 > 第 1 次峰值握力；若判为"碎了"则 `geom_rgba` 为红色。（**如果实测没碎**：不要改阈值凑结果，如实报告峰值力，并把"用力抓"的目标力或姿态 B 调到能说明问题；调整理由写进演示的 `description`。）
- 结束后 `sim.drive.k_scale == 1.0`，物体颜色已还原。

---

## 7. 任务 B5：演示 4「位置控制 vs 阻抗控制抓物体」（含决策门）

### 背景（必读）
用户要的是"抓球"。**实测**（见附录 A）：球放在平桌面上，指尖一碰就被推走，6 组参数全部失败；加了一圈 4 块板的"座"后球不再被推走，但指尖碰不到球（力≈0），原因没查清。所以这一节有**决策门**。

### 决策门 G1（先做它，限时）
目标：**一个能稳定被拇指+食指捏起来的"球"**（成功率 ≥ 4/5，改变初始位置 ±1 cm 仍成立）。

按顺序尝试，**总共不超过 3 轮实验**（每轮 = 改一处 + 跑 5 个位置）：
1. **水平对齐的捏取姿态族**（最可能有效）。现在的 A→B 插值里，食指/拇指指尖的高度差随 `s` 变化（**实测** `s≈0.6` 时拇指尖比食指尖高约 13 mm），捏球时接触法线会带上竖直分量，球被挤出去。做法：写一个小 IK（高斯-牛顿，有限差分雅可比，阻尼最小二乘），变量 = 食指 `mcp/pip/dip` + 拇指 `rot/cmc/mcp/ip` 共 7 个，约束 = ①两指尖球心距离 = 目标开口 `a`；②两指尖球心 z 相同；③延续上一个 `a` 的解做初值（连续路径）。对 `a ∈ [0.03, 0.08]` 预先算一张表（0.005 步长），闭合 = `a` 减小。仓库里没有保存当年求 OK 姿态的 IK 代码，需要自己写（OK 姿态的结果见 `hand_extras.OK_THUMB/OK_INDEX`，可当校验：该姿态下指尖间距 < 25 mm，`tests/test_hand.py::test_ok_gesture_touches_thumb_and_index` 在测）。**验收这一步**：表里每个 `a`，两指尖球心 z 差 < 2 mm。
2. 球放在**座**里：座做成中间有凹陷的东西（4 块板围成的框，内口边长 ≈ 球半径的 1.2 倍；或用细立柱 + 环）。注意座的高度要低于指尖底部（指尖半径 8 mm）至少 8 mm，否则指尖被座挡住——上次失败很可能是这个原因，**先打印指尖底部 z 和座顶 z 再下结论**。
3. 球加"滚动阻尼"：自由关节 `damping="0.02"` 之类（让被推时变"迟钝"），和扭转摩擦一起试。

**若 3 轮后仍不稳定 → 走降级方案**（并在 README 路线里如实写"球放弃，用圆柱"）：把物体换成**直立圆柱**（半径 0.015、高 0.04、`condim=4`，不会滚），演示名叫"位置控制 vs 阻抗控制抓圆柱"，界面文字**不要叫球**。

无论哪条路，G1 的结论（成功/降级、试了什么、数据）写进 `docs/NEXT_STAGE_B.md` 附录 A 末尾，并且在演示的 `description` 里一句话说明。

### 演示设计（物体确定后）
- 比较两组控制：**位置式**（`PRESETS[2]` 刚硬，K×5/D×2.5/力矩上限 1.0）vs **阻抗式**（`PRESETS[0]` 柔顺，K×0.5/D×0.7/上限 0.3）。同一个 `PinchGraspSequence`，`F_target` 都取 1.5 N（让对比只在"控制刚度"上）。
- 每组跑 3 个**摆放误差** `e ∈ {0, 4, 8} mm`：手腕目标沿捏取轴偏离物体中心 `e`（模拟视觉/定位误差）。共 6 次试验。
- 每次试验记录：`impact_peak`（闭合过程中手对物体的最大法向力，N）、`steady_force`（SETTLE 阶段最后 0.2 s 的平均总握力）、`push_away`（闭合阶段物体位移峰值，mm）、`final_state`（B2 判定）、`lifted`（是否 LIFT 到位）。
- `summary()`：6 行表格 + 一句话总结（**只写测到的事实**，比如"误差 8 mm 时刚硬把物体推走了 x mm，柔顺只推了 y mm"；如果结果和直觉相反，也如实写）。
- `result_curves()`：`e=8 mm` 时两组的 `grip_total` 曲线（以闭合开始为 0 点对齐）。
- 在 `hand_table_extras.demos` 里注册。

### 验收（`tests/test_demo_compare.py`）
- 6 次试验全部能收尾，总仿真时间 < 90 s，无异常。
- 数值断言只做**不依赖结论方向**的：每次试验的指标都是有限数；`e=0` 两组至少有一组 `lifted=True`；`summary()` 含 6 行。
- 另写一个**报告脚本** `tools/report_compare.py`：跑演示并把 6 行指标打印成表（人工看趋势用），把结果贴进附录 A。

---

## 8. 任务 B6：界面接线

1. `main.py`：默认模型仍是 `hand.xml`（不破坏现有行为），README 里写清 `--model app/scenes/hand/hand_table.xml`。（**可选加分**：主窗口加一个"场景"下拉框，切换时重建 `MainWindow` 内容；做不做都行，不要为此改动大量代码。）
2. `main_window.py`：`sim.grasp` 不为 None 时加"抓取"页签；`tick()` 里页签可见时刷新；`_after_reset()` 里同步。
3. 演示面板：演示开始前若该演示声明了 `needs_object=True`（给 `Demo` 基类加这个类属性，默认 False），而当前场景没有 `grasp`，则按钮灰掉并提示"需要 hand_table 场景"（B4/B5 只在 table 场景注册，所以这条主要是防御）。
4. 相机：演示的 `camera` 属性已有机制（`MainWindow._start_demo`），复用。

验收：`tests/smoke_stage_b.py`（新，无头）打开 table 场景 → 切到抓取页签 → 跑 B4 演示到结束 → 截图；断言抓取页签文字出现"抓牢"或"碎了"之一。

---

## 9. 任务 B7：测试、文档、素材、提交

**必须全部通过**（无头环境变量见 T8）：
```bash
python tests/test_core.py
python tests/test_hand.py
python tests/test_stage_a.py
python tests/test_table_scene.py
python tests/test_grasp.py
python tests/test_pinch_sequence.py
python tests/test_demo_fragile.py
python tests/test_demo_compare.py
QT_QPA_PLATFORM=offscreen python tests/smoke_ui.py out.png
QT_QPA_PLATFORM=offscreen python tests/smoke_hand_ui.py out.png
QT_QPA_PLATFORM=offscreen python tests/smoke_stage_a.py outdir
QT_QPA_PLATFORM=offscreen python tests/smoke_stage_b.py outdir
```
- 新测试沿用现有风格：既能 `python tests/xxx.py` 直接跑（文件末尾 `if __name__ == "__main__"` 依次调用 `test_*`），也兼容 `pytest`。

**README**：路线图里把 3.6 勾上；"演示"表格加两行；"目录"补新文件；写明 table 场景的启动命令；把 B5 决策门的结论写进去。

**素材**：给 `tools/make_readme_media.py` 加两段（B4 轻拿轻放 GIF、B5 对比 GIF），重新生成 `docs/media/`，README 开头的演示表格补上；GIF 单个 ≤ 3 MB。素材必须是真实界面录的。

**提交**：每个任务一个提交，中文信息，格式 `B1：…`。最后推送 `main`。

---

## 10. 完成定义（Definition of Done）

- [ ] 上面第 9 节所有命令通过。
- [ ] 新场景里手动操作可以把手移到方块上方，"抓取"页签实时显示握力/抬升/打滑/判定。
- [ ] 演示 3 能完整跑完，两次试验的结果与总结文字一致，且物体颜色/位置/阻抗参数在结束后还原。
- [ ] 演示 4 的 G1 有明确结论（成功或降级），数据写进附录 A。
- [ ] README、素材已更新并推送。
- [ ] 没有为了让演示"好看"而调整判定阈值或手工挑选结果；与预期不符的结果如实写在总结里。

---

## 附录 A：实验记录（别重复做）

环境：沙箱 MuJoCo 3.14，Python 3.13，物理步长 2 ms，控制周期 4 ms。手腕零位，默认阻抗（K=1.5 N·m/rad、D=0.08、力矩上限 0.6 N·m）。

| # | 实验 | 结果 | 结论 |
|---|---|---|---|
| 1 | 手掌朝下，球（半径 3 cm）在桌上，五指一起往下抓；`pitch∈{0,0.25}`，球 x∈{0.09,0.11,0.13} | 6/6 抬不起来，球被碰飞或没接触 | 手指是平行排列、同平面弯曲，钻不到球下面；这条路不通 |
| 2 | 手指朝下的"爪式"，`pitch∈{0.9,1.2,1.5}`，手腕偏移 0.03~0.09 | 9/9 失败；闭合时球被打飞（最远到 x=−2.2 m） | 同上，且闭合速度快会弹飞物体（T9） |
| 3 | 拇指+食指捏球（半径 1.5/2 cm，桌上，用 `*_tip` site 算接触点） | 全部失败：球被推到别处，指尖力 ≈0 | 部分原因是 site 在胶囊顶点而非球心（见第 1.4 节），算偏约 8 mm |
| 4 | 同 3，但物体换**方块**，`condim=3` | 夹得住（1.44/1.44 N）抬起 ~8 cm，然后 0.7 s 内力衰减到 0，方块掉落 | 方块绕捏取轴转动：需要扭转摩擦（T3） |
| 5 | 方块 + `condim=4`、扭转 0.005 | 抬起 76 mm，略低于 80 mm 的判据（临界） | 系数偏小，用 0.02 |
| 6 | 方块 + `condim=4`、扭转 0.02 | **成功**：抬 83 mm，保持 1 s，每指 1.4 N | **基线** |
| 7 | 方块 + `condim=6`、扭转 0.02 | 失败 | 别用 6 |
| 8 | 球 + `condim=4`、扭转 0.02（桌上，用球心算接触距离） | 失败：球被推走（x 最远到 0.38），力 ≈0 | 平桌面上的球捏不住（T4） |
| 9 | 球放在 4 块板围的"座"上（内口 24×24 mm，板高 20 mm，球半径 2 cm） | 球留在座里不动，但手指力 ≈0.1 N，没夹住 | 原因未查清；怀疑指尖/拇指近节被座或球挡住，或拇指/食指指尖高度差 13 mm 导致接触法线偏斜（B5 G1 第 1、2 条） |
| 10 | 原型位置/力/质量扫描（见 B3 表） | 位置全部成功；`F_T≥1.0 N` 成功；质量 ≥ 0.06 kg 失败 | 现有姿态 B 的握力上限约 2.25 N/指 |

附：工作空间事实（**实测**）——桌面 0.10 m、方块在 x∈[0.10,0.20]、y∈[−0.04,0.02] 内，捏取时手腕 `wrist_z ≈ −0.113`（指尖球心高度 ≈ 方块中心 0.1155）。

## 附录 B：给后续阶段的提示（不要现在做）

- M4 手腕运动：键盘/鼠标在视窗里拖动手腕（拖动=改 `wrist_x/y/z` 目标）、预设轨迹（直线/圆/抬升/摇动）。
- M5 静止物体抓握的"手动分步 + 包握"：需要不同的手指姿态族；包握（power grasp）在手掌朝下的构型下抓桌面物体**行不通**（实验 1、2），要么把物体放在悬空的支架上，要么加手腕大角度姿态。
- M6 运动中的抓握：物体在传送带/滚动平台上，需要线性预判；会遇到球滚动问题，G1 的结论会直接影响它。
