# PAROL6 使用指南
# PAROL6 Usage Guide

## 🎉 更新说明

已成功修复PAROL6机械臂显示问题！现在所有部件（L1-L6和底座）都能正常显示，场景与8.smolvla_parol6.ipynb完全一致。

## 📁 新文件结构

```
lerobot-mujoco/
├── asset/
│   ├── example_scene_parol6.xml     ✨ 新！完整场景文件（推荐使用）
│   └── parol6/
│       ├── parol6.xml               ✨ 新！PAROL6机械臂模块
│       ├── README.md                ✨ 新！详细文档
│       └── meshes/
│           ├── base_link.STL        ✨ 机械臂底座（现在可见！）
│           ├── L1.STL               ✨ 关节1（现在可见！）
│           ├── L2.STL               ✨ 关节2（现在可见！）
│           ├── L3.STL               ✨ 关节3（现在可见！）
│           ├── L4.STL               ✨ 关节4（现在可见！）
│           ├── L5.STL               ✨ 关节5（现在可见！）
│           ├── L6.STL               ✨ 关节6（现在可见！）
│           ├── gripper_base.stl     ✨ 夹爪基座
│           └── gripper_jaw.stl      ✨ 夹爪手指
├── 9.test_parol6_gripper.ipynb      ✅ 夹爪测试Notebook
└── 01-Parol6/
    ├── parol6_full.xml              ⚠️ 旧文件（建议使用新的）
    └── GRIPPER_INTEGRATION_SUMMARY.md
```

## 🚀 快速开始

### 方法1：使用完整场景（推荐）⭐

这是最简单的方法，包含所有场景元素：

```python
import mujoco
import os

# ⚠️ 重要：MuJoCo需要绝对路径来正确解析includes
xml_path = './asset/example_scene_parol6.xml'
full_xml_path = os.path.abspath(os.path.join(os.getcwd(), xml_path))

# 加载完整场景
model = mujoco.MjModel.from_xml_path(full_xml_path)
data = mujoco.MjData(model)

print(f"✅ 场景加载成功！")
print(f"   总关节数: {model.njnt}")  # 应该是11
print(f"   总执行器: {model.nu}")    # 应该是8
print(f"   摄像头数: {model.ncam}")  # 应该是4
```

**⚠️ 重要提示**: MuJoCo在解析包含`<include>`标签的XML文件时，必须使用**绝对路径**，否则会出现路径解析错误（如`asset/asset/`路径重复）。上述代码使用`os.path.abspath()`来确保路径正确。

**场景包含：**
- ✅ 天空和地面（Isaac样式）
- ✅ 木质桌子
- ✅ PAROL6机械臂（6自由度，彩色编码）
- ✅ 真实夹爪（2自由度）
- ✅ 3个可交互物体（2个杯子 + 1个盘子）
- ✅ 4个摄像头（agentview, topview, sideview, gripper_cam）

### 方法2：在Jupyter中使用

修改你的Notebook，使用新的场景文件：

```python
# 在 8.smolvla_parol6.ipynb 或其他Notebook中
from mujoco_env.y_env2 import SimpleEnv2

# 使用新的PAROL6场景
xml_path = './asset/example_scene_parol6.xml'
PnPEnv = SimpleEnv2(xml_path, action_type='joint_angle')

print(f"✅ 环境创建成功！")
print(f"   关节数量: {len(PnPEnv.joint_names)}")
print(f"   关节名称: {PnPEnv.joint_names}")
```

### 方法3：在自定义场景中引入

如果你想创建自己的场景：

```xml
<mujoco model="My Custom Scene">
    <!-- 你的配置 -->
    <size memory="500M"/>
    <option integrator="RK4"/>

    <!-- 引入场景元素 -->
    <include file="./tabletop/object/floor_isaac_style.xml"/>
    <include file="./tabletop/object/object_table.xml"/>

    <!-- 引入PAROL6机械臂 -->
    <include file="./parol6/parol6.xml"/>

    <!-- 添加你的物体 -->
</mujoco>
```

## 🎨 新特性：彩色编码

现在每个关节都有独特的颜色，便于在仿真中识别：

```
🤖 PAROL6机械臂颜色编码：
├─ 底座 (base_link)  ⚪ 浅灰色
├─ 关节1 (L1)        🔵 蓝色
├─ 关节2 (L2)        🔴 红色
├─ 关节3 (L3)        🟢 绿色
├─ 关节4 (L4)        🟡 黄色
├─ 关节5 (L5)        🟣 品红色
├─ 关节6 (L6)        🔷 青色
└─ 夹爪 (Gripper)    ⚫ 深灰色
```

这样在运行仿真时，你可以清楚地看到每个关节的运动！

## 🎮 控制接口

### 动作空间（8维）

```python
import numpy as np

# 创建动作向量
action = np.zeros(8)

# 机械臂关节 (索引0-5)
action[0] = 0.5   # L1: 基座旋转
action[1] = 0.3   # L2: 肩部俯仰
action[2] = -0.2  # L3: 肘部俯仰
action[3] = 0.1   # L4: 腕部翻转
action[4] = 0.0   # L5: 腕部俯仰
action[5] = 0.0   # L6: 腕部旋转

# 夹爪 (索引6-7)
action[6] = 0.0   # 左手指: 0=闭合, 0.03=打开
action[7] = 0.0   # 右手指: 0=闭合, 0.03=打开

# 执行动作
PnPEnv.step(action)
```

### 观测空间

```python
# 获取关节状态
state = PnPEnv.get_joint_state()
print(f"关节角度: {state[:6]}")  # 前6个是机械臂
print(f"夹爪状态: {state[6:8]}")  # 后2个是夹爪

# 获取图像
agent_image, wrist_image = PnPEnv.grab_image()
print(f"固定视角图像: {agent_image.shape}")  # (H, W, 3)
print(f"夹爪视角图像: {wrist_image.shape}")  # (H, W, 3)
```

## 🧪 测试和验证

### 1. 基础测试

```bash
# 测试模型加载
cd /home/user/lerobot-mujoco/asset
python3 -c "
import mujoco
model = mujoco.MjModel.from_xml_path('example_scene_parol6.xml')
print('✅ 模型加载成功！')
print(f'   关节数: {model.njnt}')
print(f'   执行器: {model.nu}')
print(f'   摄像头: {model.ncam}')
"
```

### 2. 使用测试Notebook

我们提供了完整的测试Notebook：

```bash
jupyter notebook 9.test_parol6_gripper.ipynb
```

这个Notebook包含：
- ✅ 环境变量设置
- ✅ 模型加载测试
- ✅ 夹爪闭合/打开测试
- ✅ 动态开合动画
- ✅ 机械臂与夹爪协同控制
- ✅ 多视角可视化
- ✅ 自动化验收检查

### 3. 快速可视化测试

```python
import mujoco
import matplotlib.pyplot as plt

# 加载模型
model = mujoco.MjModel.from_xml_path('./asset/example_scene_parol6.xml')
data = mujoco.MjData(model)

# 渲染图像
renderer = mujoco.Renderer(model, height=480, width=640)
renderer.update_scene(data, camera='agentview')
image = renderer.render()

# 显示
plt.imshow(image)
plt.title('PAROL6 Scene')
plt.axis('off')
plt.show()
```

## 📊 与旧版本的对比

### 旧版本问题（01-Parol6/parol6_full.xml）

❌ L1-L5和底座不显示
❌ 单一灰色，难以区分关节
❌ 所有内容混在一个文件里
❌ 难以在其他场景中复用

### 新版本优势（asset/example_scene_parol6.xml）

✅ 所有部件正常显示
✅ 彩色编码，易于识别
✅ 模块化结构，便于复用
✅ 与OMY机械臂风格一致
✅ 完整的场景配置

## 🔄 迁移指南

如果你在使用旧的XML文件，请按以下步骤迁移：

### 步骤1：更新XML路径

**旧代码：**
```python
xml_path = './01-Parol6/parol6_full.xml'
```

**新代码：**
```python
xml_path = './asset/example_scene_parol6.xml'
```

### 步骤2：验证功能

```python
# 测试环境
PnPEnv = SimpleEnv2(xml_path, action_type='joint_angle')
PnPEnv.reset()

# 测试图像获取
agent_img, wrist_img = PnPEnv.grab_image()
print(f"✅ 图像大小: {agent_img.shape}")

# 测试动作执行
action = np.zeros(8)
state = PnPEnv.step(action)
print(f"✅ 动作执行成功")
```

### 步骤3：检查摄像头名称

新场景使用与原始场景相同的摄像头名称：
- `agentview` ✅ (固定视角)
- `topview` ✅ (俯视图)
- `sideview` ✅ (侧视图)
- `gripper_cam` ✅ (夹爪视角)

## 📚 更多资源

- 📖 **详细文档**: `asset/parol6/README.md`
- 🧪 **测试Notebook**: `9.test_parol6_gripper.ipynb`
- 📝 **夹爪集成总结**: `01-Parol6/GRIPPER_INTEGRATION_SUMMARY.md`
- 🔧 **XML生成脚本**: `01-Parol6/02-urdf_to_mujoco_with_objects.py`

## ❓ 常见问题

### Q1: 为什么要创建新的XML文件？

**A**: 旧的`parol6_full.xml`存在显示问题，且结构不够模块化。新的结构：
- 更易维护
- 更易复用
- 与项目其他机械臂（如OMY）风格一致
- 便于在不同场景中使用

### Q2: 旧的XML文件还能用吗？

**A**: 可以，但不推荐。旧文件存在以下问题：
- 部分部件不显示
- 没有彩色编码
- 结构不够清晰

建议使用新的`example_scene_parol6.xml`。

### Q3: 如何确认新场景是否工作正常？

**A**: 运行以下测试：
```python
import mujoco

model = mujoco.MjModel.from_xml_path('./asset/example_scene_parol6.xml')
data = mujoco.MjData(model)

# 应该看到：
assert model.njnt == 11  # 11个关节
assert model.nu == 8     # 8个执行器
assert model.ncam == 4   # 4个摄像头

print("✅ 所有检查通过！")
```

### Q4: 动作维度有变化吗？

**A**: 没有变化，仍然是8维：
- 0-5: 机械臂6个关节
- 6-7: 夹爪2个手指

### Q5: 如何在训练中使用新场景？

**A**: 修改配置文件或代码中的XML路径：
```python
# 在你的训练脚本或Notebook中
xml_path = './asset/example_scene_parol6.xml'
env = SimpleEnv2(xml_path, action_type='joint_angle')
```

### Q6: 为什么会出现 `asset/asset/` 路径重复错误？⚠️

**A**: 这是MuJoCo在处理包含`<include>`标签的XML文件时的一个路径解析问题。

**错误示例：**
```python
# ❌ 错误：使用相对路径
model = mujoco.MjModel.from_xml_path('asset/example_scene_parol6.xml')
# 报错: Error opening file 'asset/asset/objaverse/mug_5/visual/model_normalized_0.obj'
```

**解决方法：**
```python
# ✅ 正确：使用绝对路径
import os
xml_path = './asset/example_scene_parol6.xml'
full_xml_path = os.path.abspath(os.path.join(os.getcwd(), xml_path))
model = mujoco.MjModel.from_xml_path(full_xml_path)
```

**原理说明：**
- MuJoCo在解析XML文件时，会基于文件所在目录解析相对路径
- 如果传入相对路径（如`asset/example_scene_parol6.xml`），MuJoCo会将其基目录设为`asset/`
- 当XML文件包含`<include file="./objaverse/..."/>`时，MuJoCo会错误地将路径解析为`asset/asset/objaverse/...`
- 使用绝对路径可以避免这个问题

**注意：**
- ✅ 在Jupyter Notebook中使用`SimpleEnv2`时不需要担心这个问题，因为`MuJoCoParserClass`已经自动处理了路径转换
- ✅ 直接使用`mujoco.MjModel.from_xml_path()`时需要手动转换为绝对路径
- ✅ 测试脚本`test_parol6_fix.py`演示了正确的加载方法

## 🎯 下一步

1. **数据采集**: 使用新场景采集演示数据
   ```bash
   jupyter notebook 1.collect_data.ipynb
   # 修改XML路径为 './asset/example_scene_parol6.xml'
   ```

2. **模型训练**: 在新场景中训练VLA模型
   ```bash
   python train_model.py --config_path smolvla_omy.yaml
   ```

3. **策略部署**: 在新场景中测试训练好的模型
   ```bash
   jupyter notebook 8.smolvla_parol6.ipynb
   # 修改XML路径为 './asset/example_scene_parol6.xml'
   ```

## 📝 更新日志

**2025-11-05**
- ✨ 创建模块化PAROL6机械臂模型
- ✨ 创建完整场景文件
- ✨ 添加彩色编码
- ✨ 修复显示问题
- 📚 添加详细文档

---

**问题反馈**: 如有任何问题，请查看 `asset/parol6/README.md` 或创建Issue。

**享受使用PAROL6机械臂！** 🎉🤖
