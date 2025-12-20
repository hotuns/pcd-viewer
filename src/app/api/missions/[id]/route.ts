import { NextRequest, NextResponse } from 'next/server';
import { getMissionById, createMission, updateMission, deleteMission } from '@/lib/missionDao';
import type { Mission } from '@/types/mission';

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
    const updates = normalizeMissionPayload(await request.json()) as Partial<Mission>;
    
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
    const mission = normalizeMissionPayload(await request.json()) as Mission;
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

type MissionPayload = Partial<Omit<Mission, "createdAt" | "startedAt" | "completedAt" | "executionLog">> & {
  createdAt?: string | number | Date | null;
  startedAt?: string | number | Date | null;
  completedAt?: string | number | Date | null;
  executionLog?: Array<{ timestamp: string | number | Date; event: string; details?: Record<string, unknown> }>;
};

function normalizeMissionPayload(payload: MissionPayload) {
  if (!payload || typeof payload !== "object") return payload;
  const normalized: MissionPayload = { ...payload };
  const parseDate = (value: unknown) => {
    if (value === undefined) return undefined;
    if (value === null) return null;
    if (value instanceof Date) return value;
    if (typeof value === "number" || typeof value === "string") {
      const parsed = new Date(value);
      return Number.isNaN(parsed.getTime()) ? undefined : parsed;
    }
    return undefined;
  };

  (["createdAt", "startedAt", "completedAt"] as const).forEach((key) => {
    if (key in normalized) {
      const parsed = parseDate(normalized[key]);
      if (parsed === undefined) {
        delete normalized[key];
      } else {
        normalized[key] = parsed;
      }
    }
  });

  if (Array.isArray(normalized.executionLog)) {
    normalized.executionLog = normalized.executionLog.map((log) => {
      const parsedTs = parseDate(log?.timestamp);
      return parsedTs ? { ...log, timestamp: parsedTs } : log;
    });
  }

  return normalized;
}
