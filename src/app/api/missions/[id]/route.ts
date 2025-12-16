import { NextRequest, NextResponse } from 'next/server';
import { getMissionById, createMission, updateMission, deleteMission } from '@/lib/missionDao';

export async function GET(
  request: NextRequest,
  { params }: { params: Promise<{ id: string }> }
) {
  try {
    const { id } = await params;
    const mission = getMissionById(id);
    
    if (!mission) {
      return NextResponse.json(
        { error: 'Mission not found' },
        { status: 404 }
      );
    }

    return NextResponse.json({ mission });
  } catch (error) {
    console.error('Error fetching mission:', error);
    return NextResponse.json(
      { error: 'Failed to fetch mission' },
      { status: 500 }
    );
  }
}

export async function PUT(
  request: NextRequest,
  { params }: { params: Promise<{ id: string }> }
) {
  try {
    const { id } = await params;
    const updates = await request.json();
    
    updateMission(id, updates);
    
    return NextResponse.json({ success: true });
  } catch (error) {
    console.error('Error updating mission:', error);
    return NextResponse.json(
      { error: 'Failed to update mission' },
      { status: 500 }
    );
  }
}

export async function DELETE(
  request: NextRequest,
  { params }: { params: Promise<{ id: string }> }
) {
  try {
    const { id } = await params;
    deleteMission(id);
    
    return NextResponse.json({ success: true });
  } catch (error) {
    console.error('Error deleting mission:', error);
    return NextResponse.json(
      { error: 'Failed to delete mission' },
      { status: 500 }
    );
  }
}

export async function POST(request: NextRequest) {
  try {
    const mission = await request.json();
    createMission(mission);
    
    return NextResponse.json({ success: true });
  } catch (error) {
    console.error('Error creating mission:', error);
    return NextResponse.json(
      { error: 'Failed to create mission' },
      { status: 500 }
    );
  }
}
