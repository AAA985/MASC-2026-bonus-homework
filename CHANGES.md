# SYSU 无人机编队队形切换实现说明文档

## 目录

1. [任务背景与目标](#1-任务背景与目标)
2. [原框架分析](#2-原框架分析)
3. [整体改动架构](#3-整体改动架构)
4. [改动详解](#4-改动详解)
   - 4.1 [新增字母队形定义](#41-新增字母队形定义)
   - 4.2 [添加运行时队形切换接口](#42-添加运行时队形切换接口)
   - 4.3 [参数化配置文件路径](#43-参数化配置文件路径)
   - 4.4 [SYSU 专用配置文件](#44-sysu-专用配置文件)
   - 4.5 [队形启动文件](#45-队形启动文件)
   - 4.6 [自动队形调度脚本](#46-自动队形调度脚本)
5. [字母队形坐标设计](#5-字母队形坐标设计)
6. [数据流与时序](#6-数据流与时序)
7. [使用说明](#7-使用说明)
8. [文件变更清单](#8-文件变更清单)

---

## 1. 任务背景与目标

本作业基于 **Swarm-Formation** 分布式编队轨迹优化框架，要求 7 架无人机在带有随机障碍物的仿真地图中保持移动，同时按以下顺序切换队形：

```
S 形（3 秒）→ Y 形（3 秒）→ S 形（3 秒）→ U 形（保持）
```

关键约束：

- 队形切换过程中无人机**持续移动**，不得悬停等待
- 队形保持期间需体现**障碍物避障**能力
- 不得修改 `map_generator` 代码及 `normal_hexagon.launch` 中地图生成节点的参数

---

## 2. 原框架分析

### 2.1 框架核心机制

Swarm-Formation 使用**图论相似度代价函数**来约束编队形状。每架无人机在轨迹优化时，通过最小化当前编队与期望编队的**对称归一化拉普拉斯矩阵（SNL）** Frobenius 范数差来维持队形：

```
cost_formation = ||L̂_current - L̂_desired||²_F
```

其中 SNL 矩阵 L̂ 仅描述编队的**图结构**（拓扑关系），对平移、旋转、尺度具有不变性。这意味着只要定义期望队形的节点坐标，优化器会自动让无人机趋向该形状，无论整体位置在何处。

### 2.2 原有队形支持

原框架仅支持两种队形类型，定义于 `poly_traj_optimizer.h`：

```cpp
enum FORMATION_TYPE {
    NONE_FORMATION  = 0,   // 无约束
    REGULAR_HEXAGON = 1    // 正六边形（含中心，共 7 点）
};
```

期望队形节点由 `setDesiredFormation(int type)` 在**初始化时一次性设置**，运行期间无法修改。

### 2.3 目标确定机制

当 `flight_type = 3`（`SWARM_MANUAL_TARGET` 模式）时：

1. 每架无人机订阅 `/move_base_simple/goal` 获取集群中心点
2. 根据各自的 `relative_pos[id]`（来自 YAML）计算个体目标点：
   ```
   end_pt[id] = central_pos + swarm_scale × relative_pos[id]
   ```
3. 无人机在飞向各自目标点的同时，受编队约束保持字母形状

### 2.4 问题所在

原框架缺少以下能力：

| 缺失功能 | 影响 |
|---------|------|
| S/Y/U 字母队形定义 | 无法产生字母形状 |
| 运行时动态切换队形 | 只能在启动时固定一种队形 |
| 可配置的 YAML 路径 | 每种任务必须修改源码中的硬编码路径 |
| 自动定时触发机制 | 需要人工手动发布队形切换命令 |

---

## 3. 整体改动架构

```
┌─────────────────────────────────────────────────────────┐
│                    新增 / 改动组件                        │
├──────────────────────────┬──────────────────────────────┤
│  formation_scheduler.py  │  自动定时发布队形切换命令      │
│  （新增）                │  /formation_type (Int32)      │
└──────────────┬───────────┴──────────────────────────────┘
               │ 订阅 /formation_type
               ▼
┌─────────────────────────────────────────────────────────┐
│  ego_replan_fsm.cpp（改动）                              │
│  formationTypeCallback → changeFormation(type)           │
│  运行时动态切换当前编队约束                               │
└──────────────┬──────────────────────────────────────────┘
               │ 调用
               ▼
┌─────────────────────────────────────────────────────────┐
│  poly_traj_optimizer.h（改动）                           │
│  新增 S_SHAPE / Y_SHAPE / U_SHAPE 枚举与坐标定义         │
│  新增 public changeFormation() 方法                      │
│  更新 swarm_graph_ 期望队形节点（setDesiredForm）         │
└──────────────┬──────────────────────────────────────────┘
               │ 优化时使用新的期望 SNL 矩阵
               ▼
┌─────────────────────────────────────────────────────────┐
│  轨迹优化器（swarmGraphGradCostP）                        │
│  计算 ||L̂_current - L̂_desired||² 并施加梯度              │
│  无人机自然收敛到新字母形状，同时保持移动与避障            │
└─────────────────────────────────────────────────────────┘
```

**关键设计原则**：

- **向后兼容**：所有修改对 `normal_hexagon.launch` 完全透明，原有仿真不受影响
- **最小侵入**：不修改轨迹优化的核心算法，只扩展队形定义与触发接口
- **解耦调度**：调度逻辑封装在独立的 Python 节点，与规划器松耦合

---

## 4. 改动详解

### 4.1 新增字母队形定义

**文件**：`src/planner/traj_opt/include/optimizer/poly_traj_optimizer.h`

#### 4.1.1 枚举扩展

```cpp
// 改动前
enum FORMATION_TYPE {
    NONE_FORMATION  = 0,
    REGULAR_HEXAGON = 1
};

// 改动后
enum FORMATION_TYPE {
    NONE_FORMATION  = 0,
    REGULAR_HEXAGON = 1,
    S_SHAPE         = 2,   // 新增：S 形
    Y_SHAPE         = 3,   // 新增：Y 形
    U_SHAPE         = 4    // 新增：U 形
};
```

#### 4.1.2 `setDesiredFormation()` 扩展

在原有 `switch-case` 中新增三个 `case`，每个 `case` 向 `swarm_des` 向量中推入 7 个 `Eigen::Vector3d` 节点坐标（单位：米，相对编队中心），然后调用 `swarm_graph_->setDesiredForm(swarm_des)` 更新 SNL 矩阵。

#### 4.1.3 新增公有切换方法

原 `setDesiredFormation()` 在 `private` 作用域，外部无法调用。新增一个公有包装方法：

```cpp
public:
    void changeFormation(int type)
    {
        use_formation_ = true;
        setDesiredFormation(type);
        ROS_INFO("[PolyTrajOpt] Formation changed to type %d", type);
    }
```

`use_formation_ = true` 确保切换到 `S/Y/U` 时编队约束代价重新激活（防止此前被 `NONE_FORMATION` 关闭的情况）。

---

### 4.2 添加运行时队形切换接口

涉及文件：

- `src/planner/plan_manage/include/plan_manage/ego_replan_fsm.h`
- `src/planner/plan_manage/src/ego_replan_fsm.cpp`

#### 4.2.1 头文件改动

新增头文件引用与成员声明：

```cpp
// 新增头文件
#include <std_msgs/Int32.h>

// 新增成员变量
ros::Subscriber formation_type_sub_;

// 新增回调声明
void formationTypeCallback(const std_msgs::Int32ConstPtr &msg);
```

#### 4.2.2 订阅者注册（`init()` 函数末尾）

```cpp
// 在所有 flight_type 分支之后统一注册，所有无人机都订阅
formation_type_sub_ = nh.subscribe(
    "/formation_type", 1,
    &EGOReplanFSM::formationTypeCallback, this
);
```

选择在所有 `flight_type` 分支之外注册，是因为队形切换与飞行模式无关，任何模式下都应该响应。

#### 4.2.3 回调函数实现

```cpp
void EGOReplanFSM::formationTypeCallback(const std_msgs::Int32ConstPtr &msg)
{
    planner_manager_->ploy_traj_opt_->changeFormation(msg->data);
    ROS_INFO("[FSM] Drone %d: formation changed to type %d",
             planner_manager_->pp_.drone_id, msg->data);
}
```

**工作机制**：

1. 调度脚本发布 `/formation_type` 消息
2. 所有 7 架无人机的 FSM 同时收到回调
3. 各自调用优化器的 `changeFormation()`，更新各自内部的期望 SNL 矩阵
4. 下次轨迹重规划（约每 0.1s 触发一次）时，新的编队约束代价开始生效
5. 无人机在保持移动与避障的同时，逐渐收敛到新字母形状

**线程安全性**：ROS 默认单线程回调队列，定时器回调与订阅回调不并发执行，无需额外锁保护。

---

### 4.3 参数化配置文件路径

涉及文件：

- `src/planner/plan_manage/launch/advanced_param.xml`
- `src/planner/plan_manage/launch/run_in_sim.launch`

#### 4.3.1 问题

`advanced_param.xml` 原来硬编码了 YAML 路径：

```xml
<!-- 原来（硬编码） -->
<rosparam file="$(find ego_planner)/config/normal_hexagon.yaml"/>
```

不同任务需要不同配置（如不同初始队形、不同相对位置），但修改这行会破坏原有 `normal_hexagon.launch`。

#### 4.3.2 解决方案

在 `advanced_param.xml` 中新增带默认值的参数：

```xml
<!-- advanced_param.xml 改动 -->
<arg name="config_file" default="$(find ego_planner)/config/normal_hexagon.yaml"/>
...
<rosparam file="$(arg config_file)"/>
```

在 `run_in_sim.launch` 中同样新增并透传：

```xml
<!-- run_in_sim.launch 改动 -->
<arg name="config_file" default="$(find ego_planner)/config/normal_hexagon.yaml"/>
...
<include file="$(find ego_planner)/launch/advanced_param.xml">
    <arg name="config_file" value="$(arg config_file)"/>
    ...
</include>
```

**向后兼容性**：默认值为原来的 `normal_hexagon.yaml`，`normal_hexagon.launch` 无需做任何修改即可继续正常运行。

---

### 4.4 SYSU 专用配置文件

**文件**：`src/planner/plan_manage/config/sysu_formation.yaml`（新增）

```yaml
global_goal:
  swarm_scale: 2.0
  # 相对位置按 S 形散开，供无人机计算各自的初始目标点
  relative_pos_0: {x:  0.0,  y:  2.0,  z: 0.0}   # 上中
  relative_pos_1: {x:  0.5,  y:  2.0,  z: 0.0}   # 上右
  relative_pos_2: {x: -0.5,  y:  1.0,  z: 0.0}   # 中左上
  relative_pos_3: {x:  0.0,  y:  0.0,  z: 0.0}   # 中心
  relative_pos_4: {x:  0.5,  y: -1.0,  z: 0.0}   # 中右下
  relative_pos_5: {x: -0.5,  y: -2.0,  z: 0.0}   # 下左
  relative_pos_6: {x:  0.0,  y: -2.0,  z: 0.0}   # 下中

optimization:
  formation_type      : 2        # 初始为 S 形（运行时由调度脚本动态覆盖）
  weight_obstacle     : 50000.0
  weight_swarm        : 50000.0
  weight_feasibility  : 10000.0
  weight_sqrvariance  : 10000.0
  weight_time         : 80.0
  weight_formation    : 15000.0
  obstacle_clearance  : 0.5
  swarm_clearance     : 0.5

fsm:
  replan_trajectory_time : 0.1
```

**设计说明**：

- `relative_pos` 定义各无人机相对于集群中心的目标偏移（乘以 `swarm_scale=2.0` 后为实际距离），用于分散无人机、避免初始碰撞
- `relative_pos` 的散开形状与 S 形近似，使初始飞行方向与第一个队形一致，减少到位时间
- `optimization/formation_type: 2` 是初始化时的默认队形，调度脚本启动后会立即重新发布，此处值对最终行为影响很小

---

### 4.5 队形启动文件

**文件**：`src/planner/plan_manage/launch/sysu_formation.launch`（新增）

与 `normal_hexagon.launch` 结构相同，主要差异：

| 参数 | normal_hexagon.launch | sysu_formation.launch |
|------|-----------------------|-----------------------|
| 配置文件 | `normal_hexagon.yaml`（硬编码） | `sysu_formation.yaml`（通过参数传入） |
| 附加节点 | 无 | `formation_scheduler` 节点 |
| 地图参数 | 相同 | 相同（保持随机种子一致） |
| 无人机初始位置 | 相同 | 相同 |

调度节点的启动配置：

```xml
<node pkg="ego_planner" name="formation_scheduler"
      type="formation_scheduler.py" output="screen" required="false">
    <param name="init_delay" value="3.0" type="double"/>
    <param name="central_x"  value="26.0" type="double"/>
    <param name="central_y"  value="0.0"  type="double"/>
    <param name="central_z"  value="0.5"  type="double"/>
</node>
```

`required="false"` 表示调度节点退出后不会关闭整个 launch，有利于调试。

---

### 4.6 自动队形调度脚本

**文件**：`src/planner/plan_manage/scripts/formation_scheduler.py`（新增）

#### 完整执行流程

```
节点启动
    │
    ▼
等待 init_delay（默认 3.0s）
── 等待所有无人机完成初始化、订阅建立完毕 ──
    │
    ▼
发布集群中心目标 → /move_base_simple/goal
── 触发所有无人机计算各自目标点并开始飞行 ──
    │
    ▼
等待 0.5s（确保目标点被规划器接收）
    │
    ├─▶ 发布 S 形 (type=2) → /formation_type
    │   等待 3.0s
    │
    ├─▶ 发布 Y 形 (type=3) → /formation_type
    │   等待 3.0s
    │
    ├─▶ 发布 S 形 (type=2) → /formation_type
    │   等待 3.0s
    │
    └─▶ 发布 U 形 (type=4) → /formation_type
        rospy.spin()（保持节点运行，锁存最后消息）
```

#### 关键实现细节

- 使用 `latch=True` 的发布者，新订阅者加入时自动获得最新消息，防止无人机因重启而丢失队形状态
- 队形序列用 `(type, name)` 列表表示，易于扩展和修改
- 通过 `rospy.get_param` 支持在 launch 文件中覆盖 `init_delay`、`central_x/y/z` 参数

---

## 5. 字母队形坐标设计

所有坐标以编队几何中心为原点，单位为米，Z 轴坐标均为 0（水平编队）。7 架无人机的下标（D0~D6）对应各自的 `drone_id`。

### 5.1 S 形

从俯视视角（RViz 顶视图）看，X 轴向右，Y 轴向上：

```
Y
↑
2 │  D0   D1          ← 顶部右横（S 上半弧右侧）
1 │D2                 ← 中左（S 上半弧左侧）
0 │     D3            ← 中心（两段弧的连接点）
-1│          D4       ← 中右下（S 下半弧右侧）
-2│D5   D6            ← 底部左横（S 下半弧左侧）
  └──────────────── X
    -1   0   1
```

坐标表：

| 无人机 | X    | Y    | 作用 |
|--------|------|------|------|
| D0     |  0.0 |  2.0 | 顶中 |
| D1     |  1.0 |  2.0 | 顶右 |
| D2     | -1.0 |  1.0 | 上左 |
| D3     |  0.0 |  0.0 | 中心 |
| D4     |  1.0 | -1.0 | 下右 |
| D5     | -1.0 | -2.0 | 底左 |
| D6     |  0.0 | -2.0 | 底中 |

### 5.2 Y 形

```
Y
↑
2 │D0          D4     ← 两臂末端
1 │   D1    D3        ← 两臂中段
0 │      D2           ← Y 字结点
-1│      D5           ← 茎上段
-2│      D6           ← 茎下端
  └──────────────── X
    -2  -1   0   1   2
```

坐标表：

| 无人机 | X    | Y    | 作用 |
|--------|------|------|------|
| D0     | -2.0 |  2.0 | 左臂末端 |
| D1     | -1.0 |  1.0 | 左臂中段 |
| D2     |  0.0 |  0.0 | 结点 |
| D3     |  1.0 |  1.0 | 右臂中段 |
| D4     |  2.0 |  2.0 | 右臂末端 |
| D5     |  0.0 | -1.0 | 茎上段 |
| D6     |  0.0 | -2.0 | 茎下端 |

### 5.3 U 形

```
Y
↑
2 │D0               D6   ← U 形顶部两端（开口）
1 │
0 │D1               D5   ← U 形两侧中段
-1│
-2│   D2   D3   D4       ← U 形底部弧线
  └────────────────────── X
    -2  -1   0   1   2
```

坐标表：

| 无人机 | X    | Y    | 作用 |
|--------|------|------|------|
| D0     | -2.0 |  2.0 | 顶左 |
| D1     | -2.0 |  0.0 | 左侧中 |
| D2     | -1.0 | -2.0 | 底左弧 |
| D3     |  0.0 | -2.0 | 底中 |
| D4     |  1.0 | -2.0 | 底右弧 |
| D5     |  2.0 |  0.0 | 右侧中 |
| D6     |  2.0 |  2.0 | 顶右 |

### 5.4 坐标选取原则

1. **最近邻间距 ≥ 1m**：防止编队约束与防碰撞约束互相冲突（`swarm_clearance = 0.5m`）
2. **覆盖范围与正六边形相当**（约 4m 直径）：避免与现有优化权重不匹配
3. **拓扑结构差异明显**：S/Y/U 的图连通性各不相同，SNL 矩阵差异大，便于优化器区分目标

---

## 6. 数据流与时序

```
时间轴（秒）
  0       1       2       3       4       5       6       7       8       9      10      11      12
  │───────────────│───────────────│───────────────│───────────────│───────────────│────────────►
  
  [启动 sysu_formation.launch]
  │
  ├── 所有组件初始化（地图、无人机节点、调度器）
  │
  t≈3s ── [调度器] 发布 /move_base_simple/goal (26, 0, 0.5)
           │
           ├── [FSM × 7] formationWaypointCallback → 计算各自目标点 → have_target_=true
           ├── [FSM × 7] 进入 SEQUENTIAL_START → planFromGlobalTraj → EXEC_TRAJ
           │
  t≈3.5s ─ [调度器] 发布 /formation_type = 2 (S 形)
           │
           ├── [FSM × 7] formationTypeCallback → changeFormation(2)
           ├── [优化器 × 7] 更新期望 SNL → 编队向 S 形收敛
           │              （收敛时间约 1~2s，取决于初始散布）
           │
  ◄── S 形保持阶段（约 3s）────────────────────────────────────────►
           │
  t≈6.5s ─ [调度器] 发布 /formation_type = 3 (Y 形)
           │
           ├── [优化器 × 7] 更新期望 SNL → 编队向 Y 形过渡
           │
  ◄── Y 形保持阶段（约 3s）────────────────────────────────────────►
           │
  t≈9.5s ─ [调度器] 发布 /formation_type = 2 (S 形)
           │
  ◄── S 形保持阶段（约 3s）────────────────────────────────────────►
           │
  t≈12.5s ─ [调度器] 发布 /formation_type = 4 (U 形)
            │
  ◄── U 形保持（持续）────────────────────────────────────────────►
```

**说明**：全程 7 架无人机以最大 1.5m/s 向目标点飞行，同时受编队约束和避障约束联合优化，轨迹每约 0.1s 重规划一次。

---

## 7. 使用说明

### 7.1 编译

```bash
cd /path/to/MASC-2026-bonus-homework
catkin_make
source devel/setup.bash
```

### 7.2 运行 SYSU 编队任务

打开两个终端：

```bash
# 终端 1：RViz 可视化
source devel/setup.bash
roslaunch ego_planner rviz.launch

# 终端 2：启动仿真（含自动调度）
source devel/setup.bash
roslaunch ego_planner sysu_formation.launch
```

启动后无需任何手动操作，调度脚本会自动：
1. 等待 3s 初始化
2. 发布集群目标（无人机开始飞行）
3. 按 3s 间隔切换 S→Y→S→U 队形

### 7.3 运行原六边形任务（不受影响）

```bash
source devel/setup.bash
roslaunch ego_planner normal_hexagon.launch
```

完全与原来相同，在 RViz 中使用 **2D Nav Goal** 发布目标点。

### 7.4 参数调整

**修改队形切换时间**：在 `sysu_formation.launch` 中调整 `formation_scheduler` 节点的参数，或直接修改 `formation_scheduler.py` 中的 `HOLD_DURATION`。

**修改集群目标位置**：在 `sysu_formation.launch` 中调整 `central_x/y/z` 参数。

**修改初始化等待时间**：在 `sysu_formation.launch` 中调整 `init_delay` 参数。

### 7.5 查看调试信息

```bash
# 观察队形切换日志
rostopic echo /rosout | grep "Formation"

# 查看无人机状态
rostopic echo /drone_0_planning/trajectory
```

---

## 8. 文件变更清单

### 修改的文件

| 文件路径 | 修改内容 |
|---------|---------|
| `src/planner/traj_opt/include/optimizer/poly_traj_optimizer.h` | 新增 S/Y/U 枚举；新增 setDesiredFormation case；新增 public changeFormation() |
| `src/planner/plan_manage/include/plan_manage/ego_replan_fsm.h` | 新增 `#include <std_msgs/Int32.h>`；新增 `formation_type_sub_`；新增回调声明 |
| `src/planner/plan_manage/src/ego_replan_fsm.cpp` | 注册 `/formation_type` 订阅者；实现 `formationTypeCallback` |
| `src/planner/plan_manage/launch/advanced_param.xml` | 新增 `config_file` 参数；`rosparam` 改用参数路径 |
| `src/planner/plan_manage/launch/run_in_sim.launch` | 新增 `config_file` 参数并透传给 `advanced_param.xml` |
| `src/planner/plan_manage/CMakeLists.txt` | 新增 `catkin_install_python` 安装调度脚本 |

### 新增的文件

| 文件路径 | 说明 |
|---------|------|
| `src/planner/plan_manage/config/sysu_formation.yaml` | SYSU 任务配置（初始 S 形、优化权重、相对位置） |
| `src/planner/plan_manage/launch/sysu_formation.launch` | 7 架无人机完整启动文件 + 调度节点 |
| `src/planner/plan_manage/scripts/formation_scheduler.py` | 自动队形调度脚本（S→Y→S→U，各 3s） |
| `results/` | 存放 demo.gif 和 report.pdf 的目录 |
