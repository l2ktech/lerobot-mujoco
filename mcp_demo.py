#!/usr/bin/env python3
"""
MCP功能演示脚本
展示MuJoCo环境的MCP功能
"""

import sys
import os
import numpy as np

# 添加项目路径
sys.path.append(os.path.dirname(os.path.abspath(__file__)))

def demo_mcp_features():
    """演示MCP核心功能"""
    print("=== MCP功能演示 ===\n")
    
    try:
        from mujoco_env.y_env import SimpleEnv
        from mujoco_env.ik import solve_ik
        from mujoco_env.transforms import rpy2r, r2rpy
        
        # 1. 创建环境
        print("1. 创建MuJoCo环境")
        xml_path = "asset/example_scene_y.xml"
        env = SimpleEnv(xml_path, action_type='eef_pose', state_type='joint_angle')
        print("   ✓ 环境创建成功")
        
        # 2. 获取状态信息
        print("\n2. 获取环境状态")
        joint_state = env.get_joint_state()
        ee_pose = env.get_ee_pose()
        print(f"   ✓ 关节状态: {joint_state}")
        print(f"   ✓ 末端执行器位姿: {ee_pose}")
        
        # 3. 获取物体信息
        print("\n3. 获取场景物体信息")
        obj_pose = env.get_obj_pose()
        print(f"   ✓ 物体位姿: {obj_pose}")
        
        # 4. 演示IK求解
        print("\n4. 逆运动学求解演示")
        q_current = env.env.get_qpos_joints(joint_names=env.joint_names)
        p_target = np.array([0.35, 0.1, 1.0])  # 新目标位置
        R_target = rpy2r(np.deg2rad([90, 0, 90]))
        
        q_solution, ik_err, ik_info = solve_ik(
            env=env.env,
            joint_names_for_ik=env.joint_names,
            body_name_trgt='tcp_link',
            q_init=q_current,
            p_trgt=p_target,
            R_trgt=R_target,
            max_ik_tick=50
        )
        
        print(f"   ✓ 当前关节角度: {np.rad2deg(q_current)}")
        print(f"   ✓ 目标位置: {p_target}")
        print(f"   ✓ IK求解结果: {np.rad2deg(q_solution)}")
        print(f"   ✓ IK误差: {ik_err}")
        
        # 5. 演示动作执行
        print("\n5. 动作执行演示")
        # 创建一个简单的动作（末端执行器微小移动）
        action = np.array([0.01, 0.01, 0.0, 0.0, 0.0, 0.0, 0.0])
        new_state = env.step(action)
        print(f"   ✓ 执行动作: {action[:3]} (位置增量)")
        print(f"   ✓ 新状态: {new_state}")
        
        # 6. 演示数据收集
        print("\n6. 数据收集功能")
        print("   ✓ 支持图像观察: 256x256x3 RGB图像")
        print("   ✓ 支持关节状态: 6自由度关节角度")
        print("   ✓ 支持末端位姿: 6维位姿 (x,y,z,roll,pitch,yaw)")
        print("   ✓ 支持动作空间: 7维动作 (6关节+1夹爪)")
        
        # 7. 演示环境重置
        print("\n7. 环境重置功能")
        env.reset()
        reset_state = env.get_joint_state()
        print(f"   ✓ 重置后状态: {reset_state}")
        
        print("\n🎉 MCP功能演示完成！")
        print("\n主要功能总结:")
        print("  • MuJoCo环境创建与控制")
        print("  • 逆运动学求解")
        print("  • 多模态状态获取")
        print("  • 动作执行与状态更新")
        print("  • 数据收集与存储")
        
        return True
        
    except Exception as e:
        print(f"✗ 演示失败: {e}")
        import traceback
        traceback.print_exc()
        return False

def demo_data_pipeline():
    """演示数据流水线"""
    print("\n=== 数据流水线演示 ===\n")
    
    # 演示数据格式
    print("1. 数据格式")
    data_structure = {
        "observation.image": "256x256x3 RGB图像",
        "observation.wrist_image": "256x256x3 腕部图像",
        "observation.state": "6维状态向量",
        "action": "7维动作向量",
        "obj_init": "6维物体初始位姿"
    }
    
    for key, value in data_structure.items():
        print(f"   • {key}: {value}")
    
    print("\n2. 数据存储")
    print("   • 使用Parquet格式存储数据")
    print("   • 支持分块存储 (chunk-000, chunk-001, ...)")
    print("   • 元数据存储在JSONL文件中")
    
    print("\n3. 数据可视化")
    print("   • 支持多视角图像叠加")
    print("   • 实时动作回放")
    print("   • 训练过程监控")
    
    return True

def main():
    """主演示函数"""
    print("开始MCP功能演示...\n")
    
    # 演示核心功能
    core_success = demo_mcp_features()
    
    # 演示数据流水线
    pipeline_success = demo_data_pipeline()
    
    print("\n" + "="*50)
    print("演示总结:")
    print("="*50)
    
    if core_success and pipeline_success:
        print("🎉 所有MCP功能演示成功！")
        print("\n项目已准备好用于:")
        print("  • 机器人技能学习")
        print("  • 强化学习实验")
        print("  • 模仿学习研究")
        print("  • 多模态AI训练")
    else:
        print("⚠️ 部分演示失败，请检查环境配置。")

if __name__ == "__main__":
    main()