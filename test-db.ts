/**
 * 数据库功能测试脚本
 * 运行: npx tsx test-db.ts
 */

import { createMission, getAllMissions, getMissionById, updateMission, deleteMission, getMissionStats } from './src/lib/missionDao';
import type { Mission } from './src/types/mission';

console.log('🧪 Testing SQLite Database Integration\n');

// 测试任务数据
const testMission: Mission = {
  id: 'test-mission-1',
  name: '测试任务1',
  status: 'draft',
  createdAt: new Date(),
  waypoints: [
    {
      id: 'wp-1',
      index: 0,
      x: 0,
      y: 0,
      z: 0,
      status: 'pending',
    },
    {
      id: 'wp-2',
      index: 1,
      x: 1,
      y: 1,
      z: 0.5,
      status: 'pending',
    },
  ],
  executionLog: [
    {
      timestamp: new Date(),
      event: 'mission_created',
      details: { user: 'test' },
    },
  ],
};

try {
  // 1. 创建任务
  console.log('1️⃣ Creating mission...');
  createMission(testMission);
  console.log('✅ Mission created\n');

  // 2. 获取所有任务
  console.log('2️⃣ Getting all missions...');
  const allMissions = getAllMissions();
  console.log(`✅ Found ${allMissions.length} mission(s)`);
  console.log(allMissions);
  console.log();

  // 3. 获取单个任务
  console.log('3️⃣ Getting mission by ID...');
  const mission = getMissionById('test-mission-1');
  console.log('✅ Mission retrieved:');
  console.log(mission);
  console.log();

  // 4. 更新任务
  console.log('4️⃣ Updating mission...');
  updateMission('test-mission-1', {
    status: 'running',
    startedAt: new Date(),
  });
  const updatedMission = getMissionById('test-mission-1');
  console.log('✅ Mission updated:');
  console.log(updatedMission);
  console.log();

  // 5. 获取统计
  console.log('5️⃣ Getting mission stats...');
  const stats = getMissionStats();
  console.log('✅ Mission statistics:');
  console.log(stats);
  console.log();

  // 6. 删除任务
  console.log('6️⃣ Deleting mission...');
  deleteMission('test-mission-1');
  const deletedMission = getMissionById('test-mission-1');
  console.log(`✅ Mission deleted: ${deletedMission === null ? 'Yes' : 'No'}`);
  console.log();

  console.log('🎉 All tests passed!');
} catch (error) {
  console.error('❌ Test failed:', error);
  process.exit(1);
}
