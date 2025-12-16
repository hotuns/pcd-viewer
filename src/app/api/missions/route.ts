import { NextRequest, NextResponse } from 'next/server';
import { getAllMissions, getMissionStats, createMission } from '@/lib/missionDao';

export async function GET(request: NextRequest) {
  try {
    const { searchParams } = new URL(request.url);
    const stats = searchParams.get('stats');

    if (stats === 'true') {
      const missionStats = getMissionStats();
      return NextResponse.json({ stats: missionStats });
    }

    const missions = getAllMissions();
    return NextResponse.json({ missions });
  } catch (error) {
    console.error('Error fetching missions:', error);
    return NextResponse.json(
      { error: 'Failed to fetch missions' },
      { status: 500 }
    );
  }
}

export async function POST(request: NextRequest) {
  try {
    const body = await request.json();
    const { id, name, status, scene, trajectory, waypoints } = body;

    // 创建 Mission 对象
    const mission = {
      id: id || Date.now().toString(),
      name: name || '未命名任务',
      status: status || 'draft',
      scene: scene?.type === 'url' ? scene : undefined,
      trajectory: trajectory?.type === 'url' ? trajectory : undefined,
      waypoints: waypoints || [],
      createdAt: new Date(),
    };

    // 创建任务
    createMission(mission);

    return NextResponse.json(
      { 
        success: true, 
        missionId: mission.id,
        message: 'Mission created successfully' 
      },
      { status: 201 }
    );
  } catch (error) {
    console.error('Error creating mission:', error);
    return NextResponse.json(
      { error: 'Failed to create mission' },
      { status: 500 }
    );
  }
}
