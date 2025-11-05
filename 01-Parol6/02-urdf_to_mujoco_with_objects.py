#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
完整版：PAROL6机械臂 + 使用原始场景（桌子、物体等）
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

def convert_urdf_to_mujoco_full(urdf_file, output_file):
    """
    完整版转换：使用原始场景 + 机械臂 + 夹爪
    """
    print("="*80)
    print("🤖 PAROL6完整版: 原始场景 + 机械臂 + 夹爪")
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
    xml_lines.append('<!-- PAROL6完整版: 原始场景 + 机械臂 + 夹爪 -->')
    xml_lines.append('<mujoco model="parol6_full">')
    xml_lines.append('')
    
    # 内存配置
    xml_lines.append('    <!-- 内存配置 -->')
    xml_lines.append('    <size memory="500M"/>')
    xml_lines.append('')
    
    # 编译器配置（使用相对路径）
    xml_lines.append('    <!-- 编译器配置 -->')
    xml_lines.append('    <compiler angle="radian" autolimits="true" eulerseq="xyz"/>')
    xml_lines.append('')
    
    # 物理引擎配置（使用原始设置）
    xml_lines.append('    <!-- 物理引擎配置 -->')
    xml_lines.append('    <option integrator="RK4" noslip_iterations="20"/>')
    xml_lines.append('')
    
    # 默认参数
    xml_lines.append('    <!-- 默认参数 -->')
    xml_lines.append('    <default>')
    xml_lines.append('        <joint armature="0.1" damping="0.1"/>')
    xml_lines.append('        <default class="visual_only">')
    xml_lines.append('            <geom type="mesh" contype="0" conaffinity="0" group="2"/>')
    xml_lines.append('        </default>')
    xml_lines.append('        <default class="no_visual_collision">')
    xml_lines.append('            <geom type="capsule" solimp=".9 .99 .001" solref=".015 1" group="3"/>')
    xml_lines.append('        </default>')
    xml_lines.append('        <default class="visual_collision">')
    xml_lines.append('            <geom type="mesh" solimp=".9 .99 .001" solref=".015 1" group="2"/>')
    xml_lines.append('        </default>')
    xml_lines.append('    </default>')
    xml_lines.append('')
    
    # ========== 引入原始场景文件 ==========
    xml_lines.append('    <!-- 天空和地面 -->')
    xml_lines.append('    <include file="../asset/tabletop/object/floor_isaac_style.xml"/>')
    xml_lines.append('')
    
    xml_lines.append('    <!-- 桌子 -->')
    xml_lines.append('    <include file="../asset/tabletop/object/object_table.xml"/>')
    xml_lines.append('')
    
    xml_lines.append('    <!-- 物体 -->')
    xml_lines.append('    <include file="../asset/objaverse/mug_5/model_new.xml"/>')
    xml_lines.append('    <include file="../asset/objaverse/plate_11/model_new.xml"/>')
    xml_lines.append('    <include file="../asset/objaverse/mug_6/model_new.xml"/>')
    xml_lines.append('')
    
    # ========== 机械臂资产 ==========
    xml_lines.append('    <!-- 机械臂资产 -->')
    xml_lines.append('    <asset>')
    xml_lines.append('        <!-- 机械臂STL网格 -->')
    for link_name in ['base_link', 'L1', 'L2', 'L3', 'L4', 'L5', 'L6']:
        mesh_name = link_name.lower()
        xml_lines.append(f'        <mesh name="{mesh_name}" file="meshes/{link_name}.STL" scale="0.001 0.001 0.001"/>')

    xml_lines.append('')
    xml_lines.append('        <!-- 夹爪STL网格 -->')
    xml_lines.append('        <mesh name="gripper_base" file="meshes/gripper_base.stl" scale="0.001 0.001 0.001"/>')
    xml_lines.append('        <mesh name="gripper_jaw" file="meshes/gripper_jaw.stl" scale="0.001 0.001 0.001"/>')
    xml_lines.append('    </asset>')
    xml_lines.append('')
    
    # ========== 机械臂世界 ==========
    xml_lines.append('    <!-- 机械臂 -->')
    xml_lines.append('    <worldbody>')
    
    # 机械臂基座（放在桌子上）
    base_link = links['base_link']
    base_inertial = base_link.find('inertial')
    mass, com_pos, inertia = parse_inertial(base_inertial)
    
    com_str = ' '.join([f'{x:.4f}' for x in com_pos])
    inertia_str = ' '.join([f'{x:.6f}' for x in inertia])
    
    xml_lines.append('        <!-- PAROL6机械臂基座 -->')
    xml_lines.append('        <body name="base_link" pos="0.2 0 0.8">')
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
    
    current_indent = 2
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
        
        xml_lines.append(f'{ind}    <!-- 关节{i+1}: {joint_name} -->')
        xml_lines.append(f'{ind}    <body name="{joint_name}" pos="{pos_str}" quat="{quat_str}">')
        xml_lines.append(f'{ind}        <inertial pos="{com_str}" mass="{mass:.4f}" diaginertia="{inertia_str}"/>')
        
        if joint_name == 'L6':
            xml_lines.append(f'{ind}        <joint name="{joint_name}" type="hinge" axis="{axis_str}" limited="false" damping="{damping}"/>')
        else:
            xml_lines.append(f'{ind}        <joint name="{joint_name}" type="hinge" axis="{axis_str}" range="{range_str}" damping="{damping}"/>')
        
        mesh_name = joint_name.lower()
        xml_lines.append(f'{ind}        <geom type="mesh" mesh="{mesh_name}" rgba="0.75 0.75 0.75 1"/>')
        
        # 在L6上添加site和夹爪视角摄像头
        if joint_name == 'L6':
            xml_lines.append(f'{ind}        <site name="end_effector" pos="0 0 -0.13" size="0.01"/>')
            xml_lines.append(f'{ind}        <!-- 夹爪视角摄像头 -->')
            xml_lines.append(f'{ind}        <camera name="gripper_cam" pos="0 -0.1 -0.05" xyaxes="0 0 1 -1 0 0" mode="fixed" fovy="60"/>')

        xml_lines.append('')
        current_indent += 1

    # ========== 夹爪（真实STL模型） ==========
    ind = '    ' * current_indent
    xml_lines.append(f'{ind}    <!-- 夹爪（真实STL模型） -->')
    xml_lines.append(f'{ind}    <body name="gripper_base" pos="0 0 -0.02">')
    xml_lines.append(f'{ind}        <inertial pos="0 0 0" mass="0.1" diaginertia="0.0001 0.0001 0.0001"/>')
    xml_lines.append(f'{ind}        <geom type="mesh" mesh="gripper_base" rgba="0.4 0.4 0.4 1"/>')
    xml_lines.append('')

    # 左侧夹爪指
    xml_lines.append(f'{ind}        <!-- 左侧夹爪指 -->')
    xml_lines.append(f'{ind}        <body name="gripper_left" pos="0 0.01 -0.11">')
    xml_lines.append(f'{ind}            <inertial pos="0 0.02 0" mass="0.05" diaginertia="0.00005 0.00005 0.00005"/>')
    xml_lines.append(f'{ind}            <joint name="rh_l1" type="slide" axis="0 1 0" range="0 0.03" damping="1.0"/>')
    xml_lines.append(f'{ind}            <geom type="mesh" mesh="gripper_jaw" rgba="0.3 0.3 0.3 1"/>')
    xml_lines.append(f'{ind}        </body>')
    xml_lines.append('')

    # 右侧夹爪指
    xml_lines.append(f'{ind}        <!-- 右侧夹爪指 -->')
    xml_lines.append(f'{ind}        <body name="gripper_right" pos="0 -0.01 -0.11" euler="0 0 3.14159">')
    xml_lines.append(f'{ind}            <inertial pos="0 0.02 0" mass="0.05" diaginertia="0.00005 0.00005 0.00005"/>')
    xml_lines.append(f'{ind}            <joint name="rh_r1" type="slide" axis="0 1 0" range="0 0.03" damping="1.0"/>')
    xml_lines.append(f'{ind}            <geom type="mesh" mesh="gripper_jaw" rgba="0.3 0.3 0.3 1"/>')
    xml_lines.append(f'{ind}        </body>')
    xml_lines.append(f'{ind}    </body>')  # gripper_base结束
    
    # 关闭所有body
    for i in range(current_indent - 2):
        ind = '    ' * (current_indent - 1 - i)
        xml_lines.append(f'{ind}    </body>')
    xml_lines.append('        </body>')  # base_link结束
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
    for joint_name in ['rh_l1', 'rh_r1']:
        xml_lines.append(f'        <position name="{joint_name}_motor" joint="{joint_name}" kp="100" ctrlrange="0 0.03"/>')
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
    for joint_name in ['rh_l1', 'rh_r1']:
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
    print("\n📋 场景内容:")
    print("   ✓ 原始场景 (天空、地面、桌子)")
    print("   ✓ 3个物体 (盘子、红色杯子、蓝色杯子)")
    print("   ✓ PAROL6机械臂 (6自由度)")
    print("   ✓ 真实夹爪模型 (2自由度，左右手指)")
    print("   ✓ 2个摄像头 (agentview + gripper_cam)")

    return output_file

# 执行转换
if __name__ == "__main__":
    base_dir = Path("/home/user/lerobot-mujoco/01-Parol6")
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
        print(f"   ✓ 总关节数: {model.njnt} (6机械臂 + 2夹爪)")
        print(f"   ✓ 总执行器: {model.nu} (6机械臂 + 2夹爪)")
        print(f"   ✓ 总body数: {model.nbody}")
        print(f"   ✓ 摄像头数: {model.ncam}")
        
        # 测试仿真
        mujoco.mj_step(model, data)
        print("   ✓ 仿真步进测试通过!")
        
        # 显示摄像头信息
        print("📷 摄像头列表:")
        for i in range(model.ncam):
            cam_name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_CAMERA, i)
            print(f"   {i+1}. {cam_name}")
        
        print("\n" + "="*80)
        print("✅ 转换和测试全部成功!")
        print("="*80)
        print("\n💡 使用方法:")
        print("   xml_path = '/home/user/lerobot-mujoco/01-Parol6/parol6_full.xml'")
        print("   PnPEnv = SimpleEnv2(xml_path, seed=SEED)")
        
    except Exception as e:
        print(f"\n❌ 错误: {e}")
        import traceback
        traceback.print_exc()