#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
完整版：PAROL6机械臂 + 物体 + 夹爪 + 摄像头
功能：
- 添加3个物体（盘子、红色杯子、蓝色杯子）
- 添加简单几何夹爪（4自由度平行夹爪）
- 添加2个摄像头（固定视角 + 夹爪视角）
"""

import xml.etree.ElementTree as ET
from pathlib import Path
import numpy as np

def parse_origin(origin_elem):
    """解析URDF中的origin标签"""
    if origin_elem is None:
        return [0, 0, 0], [0, 0, 0]
    xyz = origin_elem.get('xyz', '0 0 0')
    pos = [float(x) for x in xyz.split()]
    rpy = origin_elem.get('rpy', '0 0 0')
    euler = [float(x) for x in rpy.split()]
    return pos, euler

def rpy_to_quat(roll, pitch, yaw):
    """欧拉角转四元数"""
    cr = np.cos(roll * 0.5)
    sr = np.sin(roll * 0.5)
    cp = np.cos(pitch * 0.5)
    sp = np.sin(pitch * 0.5)
    cy = np.cos(yaw * 0.5)
    sy = np.sin(yaw * 0.5)
    
    w = cr * cp * cy + sr * sp * sy
    x = sr * cp * cy - cr * sp * sy
    y = cr * sp * cy + sr * cp * sy
    z = cr * cp * sy - sr * sp * cy
    
    return [w, x, y, z]

def parse_inertial(inertial_elem):
    """解析惯性参数"""
    if inertial_elem is None:
        return 0.1, [0, 0, 0], [0.001, 0.001, 0.001]
    
    mass_elem = inertial_elem.find('mass')
    mass = float(mass_elem.get('value', '0.1'))
    
    origin_elem = inertial_elem.find('inertial')
    pos, _ = parse_origin(origin_elem)
    
    inertia_elem = inertial_elem.find('inertia')
    if inertia_elem is not None:
        ixx = float(inertia_elem.get('ixx', '0.001'))
        iyy = float(inertia_elem.get('iyy', '0.001'))
        izz = float(inertia_elem.get('izz', '0.001'))
    else:
        ixx = iyy = izz = 0.001
    
    return mass, pos, [ixx, iyy, izz]

def convert_urdf_to_mujoco_full(urdf_file, output_file):
    """
    完整版转换：添加物体、夹爪、摄像头
    """
    print("="*80)
    print("🤖 PAROL6完整版转换: 机械臂 + 物体 + 夹爪 + 摄像头")
    print("="*80)
    
    # 解析URDF
    print(f"📖 读取URDF: {urdf_file}")
    tree = ET.parse(urdf_file)
    root = tree.getroot()
    
    links = {link.get('name'): link for link in root.findall('link')}
    joints = {joint.get('name'): joint for joint in root.findall('joint')}
    
    print(f"   ✓ 找到 {len(links)} 个连杆, {len(joints)} 个关节")
    
    # 生成XML
    xml_lines = []
    xml_lines.append('<?xml version="1.0"?>')
    xml_lines.append('<!-- PAROL6完整版: 机械臂 + 物体 + 夹爪 + 摄像头 -->')
    xml_lines.append('<mujoco model="parol6_full">')
    xml_lines.append('')
    
    # 编译器配置
    xml_lines.append('    <!-- 编译器配置 -->')
    xml_lines.append('    <compiler angle="radian" meshdir="meshes" autolimits="true" eulerseq="xyz"/>')
    xml_lines.append('')
    
    # 可视化配置
    xml_lines.append('    <!-- 可视化配置 -->')
    xml_lines.append('    <visual>')
    xml_lines.append('        <headlight diffuse="0.6 0.6 0.6" ambient="0.3 0.3 0.3" specular="0 0 0"/>')
    xml_lines.append('        <rgba haze="0.15 0.25 0.35 1"/>')
    xml_lines.append('        <global offwidth="2560" offheight="1440"/>')
    xml_lines.append('        <quality shadowsize="4096"/>')
    xml_lines.append('        <map stiffness="700" shadowscale="0.5" fogstart="10" fogend="15" zfar="40"/>')
    xml_lines.append('    </visual>')
    xml_lines.append('')
    
    # 物理引擎配置
    xml_lines.append('    <!-- 物理引擎配置 -->')
    xml_lines.append('    <option timestep="0.002" iterations="50" solver="Newton" gravity="0 0 -9.81" cone="elliptic"/>')
    xml_lines.append('')
    
    # ========== 资产定义 ==========
    xml_lines.append('    <!-- 资产定义 -->')
    xml_lines.append('    <asset>')
    xml_lines.append('        <!-- 地面纹理 -->')
    xml_lines.append('        <texture name="grid" type="2d" builtin="checker" width="512" height="512"')
    xml_lines.append('                 rgb1="0.1 0.2 0.3" rgb2="0.2 0.3 0.4"/>')
    xml_lines.append('        <material name="grid" texture="grid" texrepeat="1 1" texuniform="true" reflectance="0.2"/>')
    xml_lines.append('')
    
    # 机械臂网格
    xml_lines.append('        <!-- 机械臂STL网格 -->')
    for link_name in ['base_link', 'L1', 'L2', 'L3', 'L4', 'L5', 'L6']:
        mesh_name = link_name.lower()
        xml_lines.append(f'        <mesh name="{mesh_name}" file="{link_name}.STL" scale="0.001 0.001 0.001"/>')
    xml_lines.append('')
    
    # 物体纹理和材质
    xml_lines.append('        <!-- 物体纹理 -->')
    xml_lines.append('        <texture name="plate_tex" type="2d" builtin="flat" width="512" height="512" rgb1="0.9 0.9 0.9"/>')
    xml_lines.append('        <material name="plate_mat" texture="plate_tex" specular="0.3" shininess="0.3"/>')
    xml_lines.append('        <texture name="mug_red_tex" type="2d" builtin="flat" width="512" height="512" rgb1="0.8 0.1 0.1"/>')
    xml_lines.append('        <material name="mug_red_mat" texture="mug_red_tex" specular="0.5" shininess="0.5"/>')
    xml_lines.append('        <texture name="mug_blue_tex" type="2d" builtin="flat" width="512" height="512" rgb1="0.1 0.1 0.8"/>')
    xml_lines.append('        <material name="mug_blue_mat" texture="mug_blue_tex" specular="0.5" shininess="0.5"/>')
    xml_lines.append('    </asset>')
    xml_lines.append('')
    
    # 默认参数
    xml_lines.append('    <!-- 默认参数 -->')
    xml_lines.append('    <default>')
    xml_lines.append('        <joint damping="0.5" armature="0.01"/>')
    xml_lines.append('        <geom contype="1" conaffinity="1" condim="3" friction="0.8 0.1 0.1"/>')
    xml_lines.append('        <motor ctrlrange="-1 1" ctrllimited="true"/>')
    xml_lines.append('        <position ctrlrange="-3.14 3.14" kp="10"/>')
    xml_lines.append('    </default>')
    xml_lines.append('')
    
    # ========== 世界环境 ==========
    xml_lines.append('    <!-- 世界环境 -->')
    xml_lines.append('    <worldbody>')
    xml_lines.append('        <!-- 地面 -->')
    xml_lines.append('        <geom name="floor" type="plane" size="2 2 0.1" material="grid"')
    xml_lines.append('              condim="3" contype="1" conaffinity="1"/>')
    xml_lines.append('')
    
    # 光源
    xml_lines.append('        <!-- 光源 -->')
    xml_lines.append('        <light pos="0 0 3" dir="0 0 -1" directional="false" diffuse="0.8 0.8 0.8"/>')
    xml_lines.append('        <light pos="2 2 3" dir="-1 -1 -1" directional="false" diffuse="0.4 0.4 0.4"/>')
    xml_lines.append('')
    
    # ========== 物体：盘子和杯子 ==========
    xml_lines.append('        <!-- 物体 -->')
    xml_lines.append('        <!-- 盘子 -->')
    xml_lines.append('        <body name="body_obj_plate_11" pos="0.3 -0.25 0.82">')
    xml_lines.append('            <freejoint/>')
    xml_lines.append('            <inertial pos="0 0 0" mass="0.5" diaginertia="0.001 0.001 0.001"/>')
    xml_lines.append('            <geom name="plate" type="cylinder" size="0.09 0.01" material="plate_mat"/>')
    xml_lines.append('        </body>')
    xml_lines.append('')
    
    xml_lines.append('        <!-- 红色杯子 -->')
    xml_lines.append('        <body name="body_obj_mug_5" pos="0.32 0.0 0.83">')
    xml_lines.append('            <freejoint/>')
    xml_lines.append('            <inertial pos="0 0 0.03" mass="0.2" diaginertia="0.0005 0.0005 0.0003"/>')
    xml_lines.append('            <geom name="mug_red_body" type="cylinder" size="0.035 0.05" pos="0 0 0.05" material="mug_red_mat"/>')
    xml_lines.append('            <geom name="mug_red_handle" type="capsule" size="0.008 0.03" pos="0.04 0 0.04" ')
    xml_lines.append('                  quat="0.707 0 0.707 0" material="mug_red_mat"/>')
    xml_lines.append('        </body>')
    xml_lines.append('')
    
    xml_lines.append('        <!-- 蓝色杯子 -->')
    xml_lines.append('        <body name="body_obj_mug_6" pos="0.29 0.2 0.83">')
    xml_lines.append('            <freejoint/>')
    xml_lines.append('            <inertial pos="0 0 0.03" mass="0.2" diaginertia="0.0005 0.0005 0.0003"/>')
    xml_lines.append('            <geom name="mug_blue_body" type="cylinder" size="0.035 0.05" pos="0 0 0.05" material="mug_blue_mat"/>')
    xml_lines.append('            <geom name="mug_blue_handle" type="capsule" size="0.008 0.03" pos="0.04 0 0.04"')
    xml_lines.append('                  quat="0.707 0 0.707 0" material="mug_blue_mat"/>')
    xml_lines.append('        </body>')
    xml_lines.append('')
    
    # ========== PAROL6机械臂 ==========
    xml_lines.append('        <!-- PAROL6机械臂 -->')
    base_link = links['base_link']
    base_inertial = base_link.find('inertial')
    mass, com_pos, inertia = parse_inertial(base_inertial)
    
    com_str = ' '.join([f'{x:.4f}' for x in com_pos])
    inertia_str = ' '.join([f'{x:.6f}' for x in inertia])
    
    xml_lines.append('        <body name="base_link" pos="0 0 0">')
    xml_lines.append(f'            <inertial pos="{com_str}" mass="{mass:.4f}" diaginertia="{inertia_str}"/>')
    xml_lines.append('            <geom type="mesh" mesh="base_link" rgba="0.75 0.75 0.75 1"/>')
    xml_lines.append('')
    
    # 关节链
    joint_chain = [
        ('L1', [0, 0, 1], [-1.7, 1.7], 100, 1.0),
        ('L2', [0, 0, 1], [-0.98, 1.0], 100, 1.0),
        ('L3', [0, 0, -1], [-2.0, 1.3], 100, 1.0),
        ('L4', [0, 0, -1], [-2.0, 2.0], 80, 1.0),
        ('L5', [0, 0, -1], [-2.1, 2.1], 80, 1.0),
        ('L6', [0, 0, -1], [-3.14, 3.14], 50, 0.3),
    ]
    
    current_indent = 3
    for i, (joint_name, axis, limits, kp, damping) in enumerate(joint_chain):
        ind = '    ' * current_indent
        
        joint = joints[joint_name]
        origin = joint.find('origin')
        pos_urdf, rpy = parse_origin(origin)
        
        quat = rpy_to_quat(rpy[0], rpy[1], rpy[2])
        quat_str = ' '.join([f'{q:.4f}' for q in quat])
        pos_str = ' '.join([f'{p:.4f}' for p in pos_urdf])
        
        link = links[joint_name]
        inertial_elem = link.find('inertial')
        mass, com_pos, inertia = parse_inertial(inertial_elem)
        
        com_str = ' '.join([f'{x:.4f}' for x in com_pos])
        inertia_str = ' '.join([f'{x:.6f}' for x in inertia])
        axis_str = ' '.join([f'{a:.1f}' for a in axis])
        range_str = f'{limits[0]:.2f} {limits[1]:.2f}'
        
        xml_lines.append(f'{ind}<!-- 关节{i+1}: {joint_name} -->')
        xml_lines.append(f'{ind}<body name="{joint_name}" pos="{pos_str}" quat="{quat_str}">')
        xml_lines.append(f'{ind}    <inertial pos="{com_str}" mass="{mass:.4f}" diaginertia="{inertia_str}"/>')
        
        if joint_name == 'L6':
            xml_lines.append(f'{ind}    <joint name="{joint_name}" type="hinge" axis="{axis_str}" limited="false" damping="{damping}"/>')
        else:
            xml_lines.append(f'{ind}    <joint name="{joint_name}" type="hinge" axis="{axis_str}" range="{range_str}" damping="{damping}"/>')
        
        mesh_name = joint_name.lower()
        xml_lines.append(f'{ind}    <geom type="mesh" mesh="{mesh_name}" rgba="0.75 0.75 0.75 1"/>')
        
        # 在L6上添加site和摄像头
        if joint_name == 'L6':
            xml_lines.append(f'{ind}    <site name="end_effector" pos="0 0 -0.08" size="0.01"/>')
            xml_lines.append(f'{ind}    <!-- 夹爪视角摄像头 -->')
            xml_lines.append(f'{ind}    <camera name="gripper_cam" pos="0 -0.1 -0.05" xyaxes="0 0 1 -1 0 0" mode="fixed"/>')
        
        xml_lines.append('')
        current_indent += 1
    
    # ========== 夹爪（简单几何体版） ==========
    ind = '    ' * current_indent
    xml_lines.append(f'{ind}<!-- 夹爪 -->')
    xml_lines.append(f'{ind}<body name="gripper_base" pos="0 0 -0.08">')
    xml_lines.append(f'{ind}    <inertial pos="0 0 0" mass="0.05" diaginertia="0.0001 0.0001 0.0001"/>')
    xml_lines.append(f'{ind}    <geom name="gripper_palm" type="box" size="0.02 0.025 0.01" rgba="0.3 0.3 0.3 1"/>')
    xml_lines.append('')
    
    # 左侧夹爪指
    xml_lines.append(f'{ind}    <!-- 左侧夹爪指 -->')
    xml_lines.append(f'{ind}    <body name="gripper_left_outer" pos="0 0.025 0">')
    xml_lines.append(f'{ind}        <inertial pos="0 0.015 0" mass="0.01" diaginertia="0.00001 0.00001 0.00001"/>')
    xml_lines.append(f'{ind}        <joint name="rh_r1" type="slide" axis="0 1 0" range="0 0.04" damping="0.5"/>')
    xml_lines.append(f'{ind}        <geom name="left_outer" type="box" size="0.005 0.02 0.04" pos="0 0.02 -0.04" rgba="0.2 0.2 0.2 1"/>')
    xml_lines.append(f'{ind}    </body>')
    xml_lines.append('')
    
    xml_lines.append(f'{ind}    <body name="gripper_left_inner" pos="0 0.015 0">')
    xml_lines.append(f'{ind}        <inertial pos="0 0.015 0" mass="0.01" diaginertia="0.00001 0.00001 0.00001"/>')
    xml_lines.append(f'{ind}        <joint name="rh_l1" type="slide" axis="0 1 0" range="0 0.04" damping="0.5"/>')
    xml_lines.append(f'{ind}        <geom name="left_inner" type="box" size="0.005 0.02 0.04" pos="0 0.02 -0.04" rgba="0.2 0.2 0.2 1"/>')
    xml_lines.append(f'{ind}    </body>')
    xml_lines.append('')
    
    # 右侧夹爪指
    xml_lines.append(f'{ind}    <!-- 右侧夹爪指 -->')
    xml_lines.append(f'{ind}    <body name="gripper_right_outer" pos="0 -0.025 0">')
    xml_lines.append(f'{ind}        <inertial pos="0 -0.015 0" mass="0.01" diaginertia="0.00001 0.00001 0.00001"/>')
    xml_lines.append(f'{ind}        <joint name="rh_r2" type="slide" axis="0 -1 0" range="0 0.04" damping="0.5"/>')
    xml_lines.append(f'{ind}        <geom name="right_outer" type="box" size="0.005 0.02 0.04" pos="0 -0.02 -0.04" rgba="0.2 0.2 0.2 1"/>')
    xml_lines.append(f'{ind}    </body>')
    xml_lines.append('')
    
    xml_lines.append(f'{ind}    <body name="gripper_right_inner" pos="0 -0.015 0">')
    xml_lines.append(f'{ind}        <inertial pos="0 -0.015 0" mass="0.01" diaginertia="0.00001 0.00001 0.00001"/>')
    xml_lines.append(f'{ind}        <joint name="rh_l2" type="slide" axis="0 -1 0" range="0 0.04" damping="0.5"/>')
    xml_lines.append(f'{ind}        <geom name="right_inner" type="box" size="0.005 0.02 0.04" pos="0 -0.02 -0.04" rgba="0.2 0.2 0.2 1"/>')
    xml_lines.append(f'{ind}    </body>')
    xml_lines.append(f'{ind}</body>')  # gripper_base结束
    
    # 关闭所有body
    for i in range(current_indent - 3):
        ind = '    ' * (current_indent - 1 - i)
        xml_lines.append(f'{ind}</body>')
    xml_lines.append('        </body>')  # base_link结束
    xml_lines.append('')
    
    # ========== 摄像头 ==========
    xml_lines.append('        <!-- 固定摄像头 -->')
    xml_lines.append('        <camera name="fixed" pos="1.5 1.5 1.5" xyaxes="-1 1 0 -0.5 -0.5 1" mode="fixed"/>')
    xml_lines.append('        <camera name="top" pos="0 0 2" xyaxes="1 0 0 0 1 0" mode="fixed"/>')
    xml_lines.append('    </worldbody>')
    xml_lines.append('')
    
    # ========== 执行器 ==========
    xml_lines.append('    <!-- 执行器 -->')
    xml_lines.append('    <actuator>')
    
    # 机械臂执行器
    xml_lines.append('        <!-- 机械臂关节 -->')
    for joint_name, _, limits, kp, _ in joint_chain:
        range_str = f'{limits[0]:.2f} {limits[1]:.2f}'
        xml_lines.append(f'        <position name="{joint_name}_motor" joint="{joint_name}" kp="{kp}" ctrlrange="{range_str}"/>')
    
    # 夹爪执行器
    xml_lines.append('')
    xml_lines.append('        <!-- 夹爪关节 -->')
    for joint_name in ['rh_r1', 'rh_l1', 'rh_r2', 'rh_l2']:
        xml_lines.append(f'        <position name="{joint_name}_motor" joint="{joint_name}" kp="50" ctrlrange="0 0.04"/>')
    xml_lines.append('    </actuator>')
    xml_lines.append('')
    
    # ========== 传感器 ==========
    xml_lines.append('    <!-- 传感器 -->')
    xml_lines.append('    <sensor>')
    xml_lines.append('        <!-- 机械臂关节传感器 -->')
    for i in range(1, 7):
        xml_lines.append(f'        <jointpos name="L{i}_pos" joint="L{i}"/>')
    for i in range(1, 7):
        xml_lines.append(f'        <jointvel name="L{i}_vel" joint="L{i}"/>')
    
    xml_lines.append('')
    xml_lines.append('        <!-- 夹爪传感器 -->')
    for joint_name in ['rh_r1', 'rh_l1', 'rh_r2', 'rh_l2']:
        xml_lines.append(f'        <jointpos name="{joint_name}_pos" joint="{joint_name}"/>')
    xml_lines.append('    </sensor>')
    xml_lines.append('')
    xml_lines.append('</mujoco>')
    
    # 写入文件
    output_content = '\n'.join(xml_lines)
    with open(output_file, 'w', encoding='utf-8') as f:
        f.write(output_content)
    
    print(f"✅ 完整版XML已生成: {output_file}")
    print(f"   文件大小: {len(output_content)} 字节")
    print(f"   总行数: {len(xml_lines)} 行")
    print("📋 添加的内容:")
    print("   ✓ 3个物体 (盘子、红色杯子、蓝色杯子)")
    print("   ✓ 4自由度夹爪 (简单几何体)")
    print("   ✓ 2个摄像头 (固定视角 + 夹爪视角)")
    print("   ✓ 10个执行器 (6个机械臂 + 4个夹爪)")
    
    return output_file

# 执行转换
if __name__ == "__main__":
    base_dir = Path("/home/wzy/lerobot-mujoco/01-Parol6")
    urdf_file = base_dir / "urdf" / "parol6.urdf"
    output_file = base_dir / "parol6_full.xml"
    
    try:
        output_path = convert_urdf_to_mujoco_full(str(urdf_file), str(output_file))
        
        # 测试加载
        print("🧪 测试MuJoCo模型...")
        import mujoco
        model = mujoco.MjModel.from_xml_path(output_path)
        data = mujoco.MjData(model)
        
        print("   ✓ 模型加载成功!")
        print(f"   ✓ 总关节数: {model.njnt} (6机械臂 + 4夹爪)")
        print(f"   ✓ 总执行器: {model.nu} (6机械臂 + 4夹爪)")
        print(f"   ✓ 总物体: {model.nbody} (8body)")
        print(f"   ✓ 摄像头: {model.ncam} (3个)")
        
        # 测试仿真
        mujoco.mj_step(model, data)
        print("   ✓ 仿真步进测试通过!")
        
        # 显示摄像头信息
        print("📷 摄像头列表:")
        for i in range(model.ncam):
            cam_name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_CAMERA, i)
            print(f"   {i+1}. {cam_name}")
        
        print("" + "="*80)
        print("✅ 转换和测试全部成功!")
        print("="*80)
        print("💡 下一步:")
        print("   1. 在Jupyter中使用: xml_path = 'parol6_full.xml'")
        print("   2. 如需替换夹爪模型，提供STL文件路径")
        print("   3. 摄像头名称: 'fixed', 'top', 'gripper_cam'")
        
    except Exception as e:
        print(f"❌ 错误: {e}")
        import traceback
        traceback.print_exc()