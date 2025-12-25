import type { TaskType } from "@/types/mission";

export const TASK_TYPE_VALUES: TaskType[] = [0, 1, 2, 3, 4, 5];

export const TASK_TYPE_LABELS: Record<TaskType, string> = {
  0: "无动作",
  1: "前视拍摄",
  2: "左视拍摄",
  3: "右视拍摄",
  4: "RFID",
  5: "迫降",
};

export const TASK_TYPE_OPTIONS = TASK_TYPE_VALUES.map((value) => ({
  value,
  label: `${value} - ${TASK_TYPE_LABELS[value]}`,
}));

export const normalizeTaskType = (value: unknown): TaskType | undefined => {
  if (typeof value === "number" && TASK_TYPE_VALUES.includes(value as TaskType)) {
    return value as TaskType;
  }
  if (typeof value === "string" && value.trim() !== "") {
    const parsed = Number(value);
    if (!Number.isNaN(parsed) && TASK_TYPE_VALUES.includes(parsed as TaskType)) {
      return parsed as TaskType;
    }
  }
  return undefined;
};

export const serializeTaskType = (value?: TaskType): string => {
  if (typeof value === "number" && TASK_TYPE_VALUES.includes(value as TaskType)) {
    return value.toString();
  }
  return "0";
};
