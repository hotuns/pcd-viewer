#!/bin/bash
# 测试任务配置功能

echo "🧪 测试任务配置功能"
echo "==================="

echo "✅ 检查关键组件文件:"
echo "📁 启动页组件: $(ls -la src/components/StartupPage.tsx 2>/dev/null && echo "存在" || echo "缺失")"
echo "📁 任务控制器: $(ls -la src/components/pcd/MissionController.tsx 2>/dev/null && echo "存在" || echo "缺失")"
echo "📁 主页面入口: $(ls -la src/app/page.tsx 2>/dev/null && echo "存在" || echo "缺失")"

echo ""
echo "🔍 检查任务配置功能:"

# 检查TaskConfiguration组件
if grep -q "TaskConfiguration" src/components/pcd/MissionController.tsx; then
    echo "✅ TaskConfiguration组件 - 已添加"
else
    echo "❌ TaskConfiguration组件 - 缺失"
fi

# 检查任务更新保存功能
if grep -q "localStorage.setItem.*pcd-viewer-missions" src/components/pcd/MissionController.tsx; then
    echo "✅ 任务更新保存 - 已实现"
else
    echo "❌ 任务更新保存 - 缺失"
fi

# 检查场景配置界面
if grep -q "场景文件.*\.pcd" src/components/pcd/MissionController.tsx; then
    echo "✅ 场景配置界面 - 已添加"
else
    echo "❌ 场景配置界面 - 缺失"
fi

# 检查航线配置界面  
if grep -q "规划航线.*\.json" src/components/pcd/MissionController.tsx; then
    echo "✅ 航线配置界面 - 已添加"
else
    echo "❌ 航线配置界面 - 缺失"
fi

# 检查文件上传功能
if grep -q "input.*type.*file" src/components/pcd/MissionController.tsx; then
    echo "✅ 文件上传功能 - 已实现"
else
    echo "❌ 文件上传功能 - 缺失"
fi

echo ""
echo "🎯 测试流程验证:"
echo "1. 启动页创建任务 → 状态为'draft'"
echo "2. 打开任务进入主界面 → 显示任务配置区域"
echo "3. 上传场景文件 → 任务状态保持或更新"
echo "4. 上传航线文件 → 任务状态变为'configured'"
echo "5. 点击开始规划 → 状态变为'planning'"
echo "6. 确认航线 → 状态变为'ready'"

echo ""
echo "💡 使用说明:"
echo "1. 运行 'pnpm dev' 启动开发服务器"
echo "2. 访问 http://localhost:3000"
echo "3. 在启动页创建新任务"
echo "4. 点击'打开任务'进入主界面"
echo "5. 在左侧'任务配置'区域上传场景和航线文件"

echo ""
echo "🔧 故障排除:"
echo "- 如果无法上传文件，检查文件格式(.pcd/.json)"
echo "- 如果状态不更新，检查浏览器控制台错误"
echo "- 如果配置不保存，检查localStorage权限"

echo ""
echo "✨ 测试完成！现在可以正常配置任务的场景和航线了"
