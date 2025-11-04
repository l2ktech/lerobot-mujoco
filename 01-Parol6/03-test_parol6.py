#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
PAROL6 MuJoCo模型测试脚本
功能: 测试模型加载、可视化、关节控制
"""

import numpy as np
import mujoco
import mujoco.viewer
import time

def test_model_loading():
    """测试模型加载"""
    print("="*80)
    print("🧪 测试1: 模型加载")
    print("="*80)
    
    try:
        model = mujoco.MjModel.from_xml_path('01-Parol6/parol6_full.xml')
        data = mujoco.MjData(model)
        
        print("✅ 模型加载成功!")
        print(f"   关节数: {model.njnt}")
        print(f"   执行器数: {model.nu}")
        print(f"   传感器数: {model.nsensor}")
        print(f"   刚体数: {model.nbody}")
        
        return model, data
        
    except Exception as e:
        print(f"❌ 加载失败: {e}")
        return None, None


def test_simulation(model, data):
    """测试仿真步进"""
    print("\n" + "="*80)
    print("🧪 测试2: 仿真步进")
    print("="*80)
    
    try:
        # 运行100步仿真
        for i in range(100):
            mujoco.mj_step(model, data)
        
        print("✅ 仿真步进测试通过!")
        print(f"   仿真时间: {data.time:.3f} 秒")
        
        return True
        
    except Exception as e:
        print(f"❌ 仿真失败: {e}")
        return False


def test_joint_info(model):
    """显示关节信息"""
    print("\n" + "="*80)
    print("📋 关节信息")
    print("="*80)
    
    print("\n关节列表:")
    print(f"{'ID':<4} {'名称':<10} {'范围下限':>10} {'范围上限':>10} {'类型':<10}")
    print("-"*55)
    
    for i in range(model.njnt):
        joint_name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, i)
        jnt_type = model.jnt_type[i]
        
        # 检查是否有限位
        if model.jnt_limited[i]:
            jnt_range = model.jnt_range[i]
            range_str = f"{jnt_range[0]:10.2f} {jnt_range[1]:10.2f}"
        else:
            range_str = f"{'无限制':>10} {'无限制':>10}"
        
        type_name = "hinge" if jnt_type == mujoco.mjtJoint.mjJNT_HINGE else "other"
        print(f"{i:<4} {joint_name:<10} {range_str} {type_name:<10}")


def test_actuators(model):
    """显示执行器信息"""
    print("\n执行器列表:")
    print(f"{'ID':<4} {'名称':<15} {'控制下限':>10} {'控制上限':>10} {'增益Kp':>8}")
    print("-"*55)
    
    for i in range(model.nu):
        actuator_name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_ACTUATOR, i)
        ctrl_range = model.actuator_ctrlrange[i]
        # 获取增益（如果是位置执行器）
        kp = model.actuator_gainprm[i, 0]
        
        print(f"{i:<4} {actuator_name:<15} {ctrl_range[0]:10.2f} {ctrl_range[1]:10.2f} {kp:8.0f}")


def test_visualization(model, data):
    """测试可视化 (可选)"""
    print("\n" + "="*80)
    print("🎨 测试3: 可视化 (5秒)")
    print("="*80)
    print("提示: 使用鼠标拖拽查看模型, 按ESC或关闭窗口退出")
    
    try:
        # 设置初始姿态 (所有关节回零)
        data.qpos[:] = 0
        
        # 启动被动查看器
        with mujoco.viewer.launch_passive(model, data) as viewer:
            start_time = time.time()
            
            # 运行5秒
            while viewer.is_running() and (time.time() - start_time) < 5.0:
                step_start = time.time()
                
                # 简单的正弦波运动测试
                t = data.time
                data.ctrl[0] = 0.5 * np.sin(2 * np.pi * 0.2 * t)  # L1关节
                data.ctrl[1] = 0.3 * np.sin(2 * np.pi * 0.3 * t)  # L2关节
                
                # 执行仿真步进
                mujoco.mj_step(model, data)
                
                # 同步查看器
                viewer.sync()
                
                # 保持20Hz频率
                time_until_next_step = model.opt.timestep - (time.time() - step_start)
                if time_until_next_step > 0:
                    time.sleep(time_until_next_step)
        
        print("✅ 可视化测试完成!")
        return True
        
    except Exception as e:
        print(f"⚠️  可视化失败 (可能是无显示环境): {e}")
        return False


def main():
    """主函数"""
    print("\n" + "="*80)
    print("🤖 PAROL6 MuJoCo模型完整测试")
    print("="*80)
    
    # 1. 测试加载
    model, data = test_model_loading()
    if model is None:
        print("\n❌ 测试失败: 无法加载模型")
        return
    
    # 2. 测试仿真
    if not test_simulation(model, data):
        print("\n❌ 测试失败: 仿真错误")
        return
    
    # 3. 显示信息
    test_joint_info(model)
    test_actuators(model)
    
    # 4. 测试可视化 (可选，如果有显示)
    # test_visualization(model, data)
    
    print("\n" + "="*80)
    print("✅ 所有测试通过!")
    print("="*80)
    print("\n💡 下一步:")
    print("   1. 在Jupyter中集成: 参考 8.smolvla_parol6.ipynb")
    print("   2. 测试可视化: 取消注释 test_visualization()")
    print("   3. 采集数据: 使用 5.language_env.ipynb")


if __name__ == "__main__":
    main()
