#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""PAROL6完整版 - 固定夹爪"""
import xml.etree.ElementTree as ET
from pathlib import Path
import numpy as np

def parse_origin(o):
    if o is None: return [0,0,0],[0,0,0]
    xyz,rpy = o.get('xyz','0 0 0'),o.get('rpy','0 0 0')
    return [float(x) for x in xyz.split()],[float(x) for x in rpy.split()]

def rpy_to_quat(r,p,y):
    cr,sr = np.cos(r*0.5),np.sin(r*0.5)
    cp,sp = np.cos(p*0.5),np.sin(p*0.5)
    cy,sy = np.cos(y*0.5),np.sin(y*0.5)
    return [cr*cp*cy+sr*sp*sy,sr*cp*cy-cr*sp*sy,cr*sp*cy+sr*cp*sy,cr*cp*sy-sr*sp*cy]

def parse_inertial(ie):
    if ie is None: return 0.1,[0,0,0],[0.001]*3
    m = float(ie.find('mass').get('value','0.1'))
    pos,_ = parse_origin(ie.find('origin'))
    i_elem = ie.find('inertia')
    if i_elem:
        ixx = float(i_elem.get('ixx','0.001'))
        iyy = float(i_elem.get('iyy','0.001'))
        izz = float(i_elem.get('izz','0.001'))
    else:
        ixx = iyy = izz = 0.001
    return m,pos,[ixx,iyy,izz]

def convert_urdf_to_mujoco_fixed_gripper(urdf_file,output_file):
    print("="*80)
    print("🤖 PAROL6固定夹爪版本")
    print("="*80)
    
    tree = ET.parse(urdf_file)
    root = tree.getroot()
    links = {l.get('name'):l for l in root.findall('link')}
    joints = {j.get('name'):j for j in root.findall('joint')}
    
    lines = ['<?xml version="1.0"?>','<mujoco model="parol6_fixed_gripper">',
             '    <size memory="500M"/>','    <compiler angle="radian" autolimits="true" eulerseq="xyz"/>',
             '    <option integrator="RK4" noslip_iterations="20"/>','','    <default>',
             '        <joint armature="0.1" damping="0.1"/>',
             '        <default class="visual_only">',
             '            <geom type="mesh" contype="0" conaffinity="0" group="2"/>',
             '        </default>','        <default class="visual_collision">',
             '            <geom type="mesh" solimp=".9 .99 .001" solref=".015 1" group="2"/>',
             '        </default>','    </default>','','    <include file="../asset/tabletop/object/floor_isaac_style.xml"/>',
             '    <include file="../asset/tabletop/object/object_table.xml"/>',
             '    <include file="../asset/objaverse/mug_5/model_new.xml"/>',
             '    <include file="../asset/objaverse/plate_11/model_new.xml"/>',
             '    <include file="../asset/objaverse/mug_6/model_new.xml"/>','','    <asset>']
    
    for l in ['base_link','L1','L2','L3','L4','L5','L6']:
        lines.append(f'        <mesh name="{l.lower()}" file="01-Parol6/meshes/{l}.STL" scale="0.001 0.001 0.001"/>')
    lines.extend(['    </asset>','','    <worldbody>'])
    
    bl = links['base_link']
    bi = bl.find('inertial')
    m,c,i = parse_inertial(bi)
    lines.extend(['        <body name="base_link" pos="0.2 0 0.8">',
                  f'            <inertial pos="{c[0]:.4f} {c[1]:.4f} {c[2]:.4f}" mass="{m:.4f}" diaginertia="{i[0]:.6f} {i[1]:.6f} {i[2]:.6f}"/>',
                  '            <geom type="mesh" mesh="base_link" rgba="0.75 0.75 0.75 1"/>'])
    
    jc = [('L1',[0,0,1],[-1.7,1.7],100,1.0),('L2',[0,0,1],[-0.98,1.0],100,1.0),
          ('L3',[0,0,-1],[-2.0,1.3],100,1.0),('L4',[0,0,-1],[-2.0,2.0],80,1.0),
          ('L5',[0,0,-1],[-2.1,2.1],80,1.0),('L6',[0,0,-1],[-3.14,3.14],50,0.3)]
    
    ind = 2
    for jn,ax,lim,kp,d in jc:
        s = '    '*ind
        j = joints[jn]
        pos,rpy = parse_origin(j.find('origin'))
        q = rpy_to_quat(*rpy)
        l = links[jn]
        m,c,i = parse_inertial(l.find('inertial'))
        if jn=='L6': m += 0.1
        
        lines.append(f'{s}    <body name="{jn}" pos="{pos[0]:.4f} {pos[1]:.4f} {pos[2]:.4f}" quat="{q[0]:.4f} {q[1]:.4f} {q[2]:.4f} {q[3]:.4f}">')
        lines.append(f'{s}        <inertial pos="{c[0]:.4f} {c[1]:.4f} {c[2]:.4f}" mass="{m:.4f}" diaginertia="{i[0]:.6f} {i[1]:.6f} {i[2]:.6f}"/>')
        
        if jn=='L6':
            lines.append(f'{s}        <joint name="{jn}" type="hinge" axis="{ax[0]} {ax[1]} {ax[2]}" limited="false" damping="{d}"/>')
        else:
            lines.append(f'{s}        <joint name="{jn}" type="hinge" axis="{ax[0]} {ax[1]} {ax[2]}" range="{lim[0]:.2f} {lim[1]:.2f}" damping="{d}"/>')
        
        lines.append(f'{s}        <geom type="mesh" mesh="{jn.lower()}" rgba="0.75 0.75 0.75 1"/>')
        
        if jn=='L6':
            lines.append(f'{s}        <site name="end_effector" pos="0 0 -0.10" size="0.01"/>')
            lines.append(f'{s}        <camera name="gripper_cam" pos="0 -0.1 -0.08" xyaxes="0 0 1 -1 0 0" mode="fixed" fovy="60"/>')
        ind += 1
    
    for i in range(ind-2):
        s = '    '*(ind-1-i)
        lines.append(f'{s}    </body>')
    lines.extend(['        </body>','    </worldbody>','','    <actuator>'])
    
    for jn,_,lim,kp,_ in jc:
        lines.append(f'        <position name="{jn}_motor" joint="{jn}" kp="{kp}" ctrlrange="{lim[0]:.2f} {lim[1]:.2f}"/>')
    lines.extend(['    </actuator>','','    <sensor>'])
    
    for i in range(1,7):
        lines.extend([f'        <jointpos name="L{i}_pos" joint="L{i}"/>',f'        <jointvel name="L{i}_vel" joint="L{i}"/>'])
    lines.extend(['    </sensor>','</mujoco>'])
    
    with open(output_file,'w',encoding='utf-8') as f:
        f.write('\n'.join(lines))
    
    print(f"✅ 生成: {output_file}")
    print("   • L6.STL包含夹爪（固定）")
    print("   • 2个摄像头")
    return output_file

if __name__=="__main__":
    from pathlib import Path
    base_dir = Path("/home/wzy/lerobot-mujoco/01-Parol6")
    convert_urdf_to_mujoco_fixed_gripper(str(base_dir/"urdf"/"parol6.urdf"),str(base_dir/"parol6_full.xml"))
    
    try:
        print("\n🧪 测试...")
        import mujoco
        model = mujoco.MjModel.from_xml_path(str(base_dir/"parol6_full.xml"))
        data = mujoco.MjData(model)
        mujoco.mj_step(model,data)
        print(f"   ✓ 关节:{model.njnt} 执行器:{model.nu} 摄像头:{model.ncam}")
        print("\n📷 摄像头:")
        for i in range(model.ncam):
            print(f"   • {mujoco.mj_id2name(model,mujoco.mjtObj.mjOBJ_CAMERA,i)}")
        print("\n✅ 成功！")
    except Exception as e:
        print(f"❌ {e}")
