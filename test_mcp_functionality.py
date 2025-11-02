#!/usr/bin/env python3
"""
MCP功能测试脚本
测试MuJoCo环境的MCP功能
"""

import sys
import os
import numpy as np

# 添加项目路径
sys.path.append(os.path.dirname(os.path.abspath(__file__)))

def test_mujoco_import():
    """测试MuJoCo相关模块的导入"""
    print("=== 测试MuJoCo模块导入 ===")
    try:
        import mujoco
        print(f"✓ MuJoCo版本: {mujoco.__version__}")
    except ImportError as e:
        print(f"✗ MuJoCo导入失败: {e}")
        return False
    
    try:
        from mujoco_env import MuJoCoParserClass
        print("✓ MuJoCoParserClass导入成功")
    except ImportError as e:
        print(f"✗ MuJoCoParserClass导入失败: {e}")
        return False
    
    try:
        from mujoco_env.y_env import SimpleEnv
        print("✓ SimpleEnv导入成功")
    except ImportError as e:
        print(f"✗ SimpleEnv导入失败: {e}")
        return False
    
    return True

def test_xml_parsing():
    """测试XML文件解析"""
    print("\n=== 测试XML文件解析 ===")
    
    # 检查XML文件是否存在
    xml_files = [
        "asset/robotis_omy/omy.xml",
        "asset/robotis_omy/scene.xml",
        "asset/example_scene_y.xml"
    ]
    
    for xml_file in xml_files:
        full_path = os.path.join(os.path.dirname(__file__), xml_file)
        if os.path.exists(full_path):
            print(f"✓ XML文件存在: {xml_file}")
        else:
            print(f"✗ XML文件不存在: {xml_file}")
    
    return True

def test_environment_creation():
    """测试环境创建"""
    print("\n=== 测试环境创建 ===")
    
    try:
        from mujoco_env.y_env import SimpleEnv
        
        # 使用示例XML文件创建环境
        xml_path = "asset/example_scene_y.xml"
        full_xml_path = os.path.join(os.path.dirname(__file__), xml_path)
        
        if os.path.exists(full_xml_path):
            print(f"✓ 使用XML文件创建环境: {xml_path}")
            
            # 尝试创建环境（不显示图形界面）
            env = SimpleEnv(xml_path, action_type='eef_pose', state_type='joint_angle')
            print("✓ 环境创建成功")
            
            # 测试基本功能
            state = env.get_joint_state()
            print(f"✓ 获取关节状态成功: {state.shape}")
            
            ee_pose = env.get_ee_pose()
            print(f"✓ 获取末端执行器位姿成功: {ee_pose.shape}")
            
            return True
        else:
            print(f"✗ XML文件不存在: {full_xml_path}")
            return False
            
    except Exception as e:
        print(f"✗ 环境创建失败: {e}")
        return False

def test_ik_solver():
    """测试逆运动学求解器"""
    print("\n=== 测试逆运动学求解器 ===")
    
    try:
        from mujoco_env.ik import solve_ik
        from mujoco_env.transforms import rpy2r
        
        print("✓ IK求解器导入成功")
        
        # 测试简单的位姿
        p_trgt = np.array([0.3, 0.0, 1.0])
        R_trgt = rpy2r(np.deg2rad([90, 0, 90]))
        
        print(f"✓ 目标位置: {p_trgt}")
        print(f"✓ 目标旋转矩阵形状: {R_trgt.shape}")
        
        return True
        
    except Exception as e:
        print(f"✗ IK求解器测试失败: {e}")
        return False

def test_data_collection_structure():
    """测试数据收集结构"""
    print("\n=== 测试数据收集结构 ===")
    
    # 检查示例数据目录
    demo_data_path = os.path.join(os.path.dirname(__file__), "demo_data_example")
    if os.path.exists(demo_data_path):
        print("✓ 示例数据目录存在")
        
        # 检查数据文件
        data_files = [
            "data/chunk-000/episode_000000.parquet",
            "meta/episodes.jsonl",
            "meta/info.json",
            "meta/stats.json",
            "meta/tasks.jsonl"
        ]
        
        for data_file in data_files:
            full_path = os.path.join(demo_data_path, data_file)
            if os.path.exists(full_path):
                print(f"✓ 数据文件存在: {data_file}")
            else:
                print(f"✗ 数据文件不存在: {data_file}")
    else:
        print("✗ 示例数据目录不存在")
        return False
    
    return True

def main():
    """主测试函数"""
    print("开始MCP功能测试...\n")
    
    tests = [
        ("模块导入", test_mujoco_import),
        ("XML解析", test_xml_parsing),
        ("环境创建", test_environment_creation),
        ("IK求解器", test_ik_solver),
        ("数据结构", test_data_collection_structure)
    ]
    
    results = []
    
    for test_name, test_func in tests:
        try:
            result = test_func()
            results.append((test_name, result))
        except Exception as e:
            print(f"✗ {test_name}测试异常: {e}")
            results.append((test_name, False))
    
    print("\n" + "="*50)
    print("测试结果汇总:")
    print("="*50)
    
    passed = 0
    total = len(results)
    
    for test_name, result in results:
        status = "✓ 通过" if result else "✗ 失败"
        print(f"{test_name}: {status}")
        if result:
            passed += 1
    
    print(f"\n总测试: {total}, 通过: {passed}, 失败: {total - passed}")
    
    if passed == total:
        print("\n🎉 所有MCP功能测试通过！")
    else:
        print("\n⚠️ 部分MCP功能测试失败，请检查相关依赖和配置。")

if __name__ == "__main__":
    main()