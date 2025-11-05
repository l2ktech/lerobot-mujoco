#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
PAROL6路径问题修复测试脚本
Fix and test PAROL6 XML path issues
"""

import os
import sys
import mujoco

def test_old_xml_with_fix():
    """测试旧XML文件的修复方案"""
    print("="*80)
    print("🔧 测试方案1: 修复旧XML文件的加载")
    print("="*80)

    # 关键修复：设置正确的工作目录
    original_dir = os.getcwd()
    print(f"原始工作目录: {original_dir}")

    # 切换到项目根目录
    project_root = "/home/user/lerobot-mujoco"
    os.chdir(project_root)
    print(f"切换工作目录: {os.getcwd()}")

    try:
        # 使用绝对路径加载
        xml_path = os.path.join(project_root, "01-Parol6", "parol6_full.xml")
        print(f"\n📖 加载XML: {xml_path}")

        model = mujoco.MjModel.from_xml_path(xml_path)
        data = mujoco.MjData(model)

        print("\n✅ 旧XML文件加载成功！")
        print(f"   总关节数: {model.njnt}")
        print(f"   总执行器: {model.nu}")
        print(f"   摄像头数: {model.ncam}")

        # 测试仿真
        mujoco.mj_step(model, data)
        print("   仿真测试: ✅")

        return True

    except Exception as e:
        print(f"\n❌ 加载失败: {e}")
        return False

    finally:
        # 恢复原始工作目录
        os.chdir(original_dir)


def test_new_xml():
    """测试新XML文件（推荐方案）"""
    print("\n" + "="*80)
    print("🚀 测试方案2: 使用新的场景文件（推荐）")
    print("="*80)

    original_dir = os.getcwd()

    try:
        # 关键修复：使用绝对路径！
        project_root = "/home/user/lerobot-mujoco"
        os.chdir(project_root)

        xml_path = "./asset/example_scene_parol6.xml"
        # MuJoCo需要绝对路径来正确解析includes
        full_xml_path = os.path.abspath(os.path.join(os.getcwd(), xml_path))
        print(f"\n📖 加载XML: {xml_path}")
        print(f"   绝对路径: {full_xml_path}")

        model = mujoco.MjModel.from_xml_path(full_xml_path)
        data = mujoco.MjData(model)

        print("\n✅ 新XML文件加载成功！")
        print(f"   总关节数: {model.njnt}")
        print(f"   总执行器: {model.nu}")
        print(f"   摄像头数: {model.ncam}")

        # 显示关节列表
        print("\n🔧 关节列表:")
        for i in range(min(model.njnt, 10)):
            joint_name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, i)
            if joint_name:
                print(f"   {i}: {joint_name}")

        # 显示摄像头列表
        print("\n📷 摄像头列表:")
        for i in range(model.ncam):
            cam_name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_CAMERA, i)
            print(f"   {i+1}. {cam_name}")

        # 测试仿真
        mujoco.mj_step(model, data)
        print("\n   仿真测试: ✅")

        return True

    except Exception as e:
        print(f"\n❌ 加载失败: {e}")
        import traceback
        traceback.print_exc()
        return False

    finally:
        os.chdir(original_dir)


def main():
    """主函数"""
    print("\n" + "🤖 PAROL6 XML路径问题修复测试".center(80, "="))
    print()

    # 测试旧文件（带修复）
    result1 = test_old_xml_with_fix()

    # 测试新文件
    result2 = test_new_xml()

    # 总结
    print("\n" + "="*80)
    print("📊 测试总结")
    print("="*80)
    print(f"旧XML文件（带修复）: {'✅ 通过' if result1 else '❌ 失败'}")
    print(f"新XML文件（推荐）:   {'✅ 通过' if result2 else '❌ 失败'}")

    if result2:
        print("\n💡 建议:")
        print("   1. 使用新的 asset/example_scene_parol6.xml（推荐）")
        print("   2. 如果必须使用旧文件，确保设置正确的工作目录")
        print("   3. 查看 PAROL6_USAGE_GUIDE.md 获取详细说明")

    print("\n" + "="*80)

    return 0 if (result1 and result2) else 1


if __name__ == "__main__":
    sys.exit(main())
