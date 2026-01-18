import { MissionHomePosition } from "@/types/mission";
import { Label } from "@/components/ui/label";
import { Input } from "@/components/ui/input";

interface MissionHomeFormProps {
  value: MissionHomePosition;
  onChange: (value: MissionHomePosition) => void;
  disabled?: boolean;
  label?: string;
}

export function MissionHomeForm({ value, onChange, disabled, label = "HomePos（机库/返航点）" }: MissionHomeFormProps) {
  const updatePosition = (axis: "x" | "y" | "z", next: number) => {
    onChange({
      ...value,
      position: {
        ...value.position,
        [axis]: next,
      },
    });
  };

  const updateYaw = (nextDeg: number) => {
    onChange({
      ...value,
      yaw: (Number.isNaN(nextDeg) ? 0 : nextDeg) * Math.PI / 180,
    });
  };

  return (
    <div className="space-y-3">
      <div className="flex items-center justify-between">
        <Label className="text-xs text-slate-300">{label}</Label>
      </div>
      <div className="grid grid-cols-4 gap-2">
        {(["x", "y", "z"] as const).map((axis) => (
          <div className="space-y-1" key={axis}>
            <Label className="text-[10px] uppercase tracking-wide text-slate-500">{axis.toUpperCase()}</Label>
            <Input
              type="number"
              step="0.1"
              value={value.position[axis]}
              onChange={(e) => updatePosition(axis, Number(e.target.value))}
              disabled={disabled}
              className="h-8 text-xs bg-slate-900 border-slate-700 text-slate-100"
            />
          </div>
        ))}
        <div className="space-y-1">
          <Label className="text-[10px] uppercase tracking-wide text-slate-500">Yaw (°)</Label>
          <Input
            type="number"
            step="1"
            value={(value.yaw * 180 / Math.PI).toFixed(1)}
            onChange={(e) => updateYaw(Number(e.target.value))}
            disabled={disabled}
            className="h-8 text-xs bg-slate-900 border-slate-700 text-slate-100"
          />
        </div>
      </div>
    </div>
  );
}
