#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
修复版：PAROL6 URDF转MuJoCo XML转换脚本
修复了XML注释格式问题
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
    """将欧拉角转换为四元数"""
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
    
    origin_elem = inertial_elem.find('origin')
    pos, _ = parse_origin(origin_elem)
    
    inertia_elem = inertial_elem.find('inertia')
    if inertia_elem is not None:
        ixx = float(inertia_elem.get('ixx', '0.001'))
        iyy = float(inertia_elem.get('iyy', '0.001'))
        izz = float(inertia_elem.get('izz', '0.001'))
    else:
        ixx = iyy = izz = 0.001
    
    return mass, pos, [ixx, iyy, izz]

def convert_urdf_to_mujoco(urdf_file, output_file):
    """将URDF转换为MuJoCo XML - 修复版"""
    print("="*80)
    print("🤖 开始转换 PAROL6 URDF → MuJoCo XML (修复版)")
    print("="*80)
    
    # 解析URDF
    print(f"📖 读取URDF: {urdf_file}")
    tree = ET.parse(urdf_file)
    root = tree.getroot()
    
    links = {link.get('name'): link for link in root.findall('link')}
    joints = {joint.get('name'): joint for joint in root.findall('joint')}
    
    print(f"   ✓ 找到 {len(links)} 个连杆, {len(joints)} 个关节")
    
    # 生成XML - 注意：注释要放在标签外面！
    xml_lines = []
    xml_lines.append('<?xml version="1.0"?>')
    xml_lines.append('<!-- PAROL6 6自由度机械臂 MuJoCo模型 -->')
    xml_lines.append('<!-- 从URDF自动转换生成 -->')
    xml_lines.append('<mujoco model="parol6">')
    xml_lines.append('')
    
    # 编译器选项 - 注释放在外面
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
    
    # 物理引擎选项
    xml_lines.append('    <!-- 物理引擎配置 -->')
    xml_lines.append('    <option timestep="0.002" iterations="50" solver="Newton" gravity="0 0 -9.81" cone="elliptic"/>')
    xml_lines.append('')
    
    # 资产定义
    xml_lines.append('    <!-- 资产定义 -->')
    xml_lines.append('    <asset>')
    xml_lines.append('        <!-- 地面纹理 -->')
    xml_lines.append('        <texture name="grid" type="2d" builtin="checker" width="512" height="512"')
    xml_lines.append('                 rgb1="0.1 0.2 0.3" rgb2="0.2 0.3 0.4"/>')
    xml_lines.append('        <material name="grid" texture="grid" texrepeat="1 1" texuniform="true" reflectance="0.2"/>')
    xml_lines.append('')
    xml_lines.append('        <!-- 机械臂STL网格 (scale: mm->m) -->')
    for link_name in ['base_link', 'L1', 'L2', 'L3', 'L4', 'L5', 'L6']:
        mesh_name = link_name.lower()
        xml_lines.append(f'        <mesh name="{mesh_name}" file="{link_name}.STL" scale="0.001 0.001 0.001"/>')
    xml_lines.append('    </asset>')
    xml_lines.append('')
    
    # 默认配置
    xml_lines.append('    <!-- 默认参数 -->')
    xml_lines.append('    <default>')
    xml_lines.append('        <joint damping="0.5" armature="0.01"/>')
    xml_lines.append('        <geom contype="1" conaffinity="1" condim="3" friction="0.8 0.1 0.1"/>')
    xml_lines.append('        <motor ctrlrange="-1 1" ctrllimited="true"/>')
    xml_lines.append('        <position ctrlrange="-3.14 3.14" kp="10"/>')
    xml_lines.append('    </default>')
    xml_lines.append('')
    
    # 世界环境
    xml_lines.append('    <!-- 世界环境 -->')
    xml_lines.append('    <worldbody>')
    xml_lines.append('        <!-- 地面 -->')
    xml_lines.append('        <geom name="floor" type="plane" size="2 2 0.1" material="grid"')
    xml_lines.append('              condim="3" contype="1" conaffinity="1"/>')
    xml_lines.append('')
    xml_lines.append('        <!-- 光源 -->')
    xml_lines.append('        <light pos="0 0 3" dir="0 0 -1" directional="false" diffuse="0.8 0.8 0.8"/>')
    xml_lines.append('        <light pos="2 2 3" dir="-1 -1 -1" directional="false" diffuse="0.4 0.4 0.4"/>')
    xml_lines.append('')
    
    # 机械臂基座
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
    
    # 构建关节链
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
        
        # 获取关节信息
        joint = joints[joint_name]
        origin = joint.find('origin')
        pos_urdf, rpy = parse_origin(origin)
        
        # 转换姿态
        quat = rpy_to_quat(rpy[0], rpy[1], rpy[2])
        quat_str = ' '.join([f'{q:.4f}' for q in quat])
        pos_str = ' '.join([f'{p:.4f}' for p in pos_urdf])
        
        # 获取连杆信息
        link = links[joint_name]
        inertial_elem = link.find('inertial')
        mass, com_pos, inertia = parse_inertial(inertial_elem)
        
        com_str = ' '.join([f'{x:.4f}' for x in com_pos])
        inertia_str = ' '.join([f'{x:.6f}' for x in inertia])
        axis_str = ' '.join([f'{a:.1f}' for a in axis])
        range_str = f'{limits[0]:.2f} {limits[1]:.2f}'
        
        # 生成body
        xml_lines.append(f'{ind}<!-- 关节{i+1}: {joint_name} -->')
        xml_lines.append(f'{ind}<body name="{joint_name}" pos="{pos_str}" quat="{quat_str}">')
        xml_lines.append(f'{ind}    <inertial pos="{com_str}" mass="{mass:.4f}" diaginertia="{inertia_str}"/>')
        
        # 关节定义
        if joint_name == 'L6':
            xml_lines.append(f'{ind}    <joint name="{joint_name}" type="hinge" axis="{axis_str}" limited="false" damping="{damping}"/>')
        else:
            xml_lines.append(f'{ind}    <joint name="{joint_name}" type="hinge" axis="{axis_str}" range="{range_str}" damping="{damping}"/>')
        
        # 网格
        mesh_name = joint_name.lower()
        xml_lines.append(f'{ind}    <geom type="mesh" mesh="{mesh_name}" rgba="0.75 0.75 0.75 1"/>')
        
        # L6末端添加site
        if joint_name == 'L6':
            xml_lines.append(f'{ind}    <site name="end_effector" pos="0 0 -0.08" size="0.01"/>')
        
        xml_lines.append('')
        current_indent += 1
    
    # 关闭所有body
    for i in range(current_indent - 3):
        ind = '    ' * (current_indent - 1 - i)
        xml_lines.append(f'{ind}</body>')
    xml_lines.append('        </body>')
    xml_lines.append('')
    
    # 摄像头
    xml_lines.append('        <!-- 摄像头 -->')
    xml_lines.append('        <camera name="fixed" pos="1.5 1.5 1.5" xyaxes="-1 1 0 -0.5 -0.5 1" mode="fixed"/>')
    xml_lines.append('        <camera name="top" pos="0 0 2" xyaxes="1 0 0 0 1 0" mode="fixed"/>')
    xml_lines.append('        <camera name="side" pos="2 0 0.5" xyaxes="0 1 0 0 0 1" mode="fixed"/>')
    xml_lines.append('    </worldbody>')
    xml_lines.append('')
    
    # 执行器
    xml_lines.append('    <!-- 执行器 -->')
    xml_lines.append('    <actuator>')
    for joint_name, _, limits, kp, _ in joint_chain:
        range_str = f'{limits[0]:.2f} {limits[1]:.2f}'
        xml_lines.append(f'        <position name="{joint_name}_motor" joint="{joint_name}" kp="{kp}" ctrlrange="{range_str}"/>')
    xml_lines.append('    </actuator>')
    xml_lines.append('')
    
    # 传感器
    xml_lines.append('    <!-- 传感器 -->')
    xml_lines.append('    <sensor>')
    for i in range(1, 7):
        xml_lines.append(f'        <jointpos name="L{i}_pos" joint="L{i}"/>')
    for i in range(1, 7):
        xml_lines.append(f'        <jointvel name="L{i}_vel" joint="L{i}"/>')
    xml_lines.append('    </sensor>')
    xml_lines.append('')
    xml_lines.append('</mujoco>')
    
    # 写入文件
    output_content = '\n'.join(xml_lines)
    with open(output_file, 'w', encoding='utf-8') as f:
        f.write(output_content)
    
    print(f"✅ XML文件已生成: {output_file}")
    print(f"   文件大小: {len(output_content)} 字节")
    print(f"   总行数: {len(xml_lines)} 行")
    
    return output_file

# 执行转换
base_dir = Path("/home/wzy/lerobot-mujoco/01-Parol6")
urdf_file = base_dir / "urdf" / "parol6.urdf"
output_file = base_dir / "parol6.xml"

try:
    output_path = convert_urdf_to_mujoco(str(urdf_file), str(output_file))
    
    # 测试加载
    print("🧪 测试MuJoCo模型...")
    import mujoco
    model = mujoco.MjModel.from_xml_path(output_path)
    data = mujoco.MjData(model)
    
    print("   ✓ 模型加载成功!")
    print(f"   ✓ 关节数: {model.njnt}")
    print(f"   ✓ 执行器数: {model.nu}")
    print(f"   ✓ 传感器数: {model.nsensor}")
    
    # 测试仿真
    mujoco.mj_step(model, data)
    print("   ✓ 仿真步进测试通过!")
    
    print("" + "="*80)
    print("✅ 转换和测试全部成功!")
    print("="*80)
    
except Exception as e:
    print(f"❌ 错误: {e}")
    import traceback
    traceback.print_exc()