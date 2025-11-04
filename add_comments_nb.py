#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import json
import os

# 读取notebook
with open('8.smolvla_parol6.ipynb', 'r', encoding='utf-8') as f:
    nb = json.load(f)

# 为每个代码单元格添加详细注释
code_cell_count = 0
for cell in nb['cells']:
    if cell['cell_type'] == 'code':
        code_cell_count += 1
        exec_count = cell.get('execution_count')
        
        # 获取原始代码
        original_code = ''.join(cell['source']) if isinstance(cell['source'], list) else cell['source']
        
        # 根据execution_count添加对应的注释
        if exec_count == 1:
            # 保存修改后的代码
            cell['source'] = [
                "# Cell 1 - 设置环境变量(必须第一个运行)\n",
                "import os\n",
                "\n",
                "# 1. 设置DISPLAY\n",
                "os.environ['DISPLAY'] = ':0'\n",
                "os.environ['XAUTHORITY'] = os.path.expanduser('~/.Xauthority')\n",
                "print(f\"✓ DISPLAY设置为: {os.environ['DISPLAY']}\")\n",
                "\n",
                "# 2. 强制使用GPU渲染(关键!)\n",
                "os.environ['MUJOCO_GL'] = 'egl'  # EGL后端GPU加速\n",
                "print(f\"✓ MUJOCO_GL: egl (GPU硬件加速)\")\n",
                "\n",
                "# 3. NVIDIA GPU优化\n",
                "os.environ['__GL_SYNC_TO_VBLANK'] = '0'  # 关闭垂直同步\n",
                "os.environ['__GL_YIELD'] = 'NOTHING'      # 减少CPU等待\n",
                "print(\"✓ NVIDIA GPU优化已启用\")\n",
                "\n",
                "# 4. OpenGL性能优化\n",
                "os.environ['__GL_FSAA_MODE'] = '0'        # 关闭抗锯齿\n",
                "os.environ['__GL_LOG_MAX_ANISO'] = '0'    # 关闭各向异性过滤\n",
                "print(\"✓ OpenGL性能优化已启用\")"
            ]

# 备份原文件
os.system('cp 8.smolvla_parol6.ipynb 8.smolvla_parol6.ipynb.bak')

# 保存修改后的notebook
with open('8.smolvla_parol6.ipynb', 'w', encoding='utf-8') as f:
    json.dump(nb, f, ensure_ascii=False, indent=1)

print(f"✓ 已处理 {code_cell_count} 个代码单元格")
print("✓ 已备份原文件为: 8.smolvla_parol6.ipynb.bak")
print("✓ 已保存带注释的文件: 8.smolvla_parol6.ipynb")
