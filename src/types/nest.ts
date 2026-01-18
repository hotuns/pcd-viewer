export interface NestStatusTelemetry {
  temperature?: number | null;
  humidity?: number | null;
  /** 手动模式为1，自动模式为0 */
  optMode?: string | null;
}

export interface NestBatteryTelemetry {
  soc?: number | null;
  chargingVoltage?: number | null;
  chargingCurrent?: number | null;
  batteryTemp1?: number | null;
  batteryTemp2?: number | null;
  pcbTemp1?: number | null;
  highestCellVolt?: number | null;
  maxCellCount?: number | null;
  cellVoltages?: number[];
  batteryChargedState?: string | null;
  highestCellNo?: string | null;
  lowestCellNo?: string | null;
  batterySupplyState?: string | null;
  batteryBalanceState?: string | null;
  chargingOk?: string | null;
}

export interface NestMotorTelemetry {
  xMotorAlarm?: string | null;
  yMotorAlarm?: string | null;
  xAxisStatus?: string | null;
  yAxisStatus?: string | null;
}

export interface NestAutoModeTelemetry {
  /** 自动离巢模式为1，自动进巢模式为0 */
  mode?: string | null;
}

export interface NestUavPresenceTelemetry {
  /** 0 在巢，1 离巢，2 异常 */
  state?: string | null;
}

export interface NestTelemetry {
  status?: NestStatusTelemetry | null;
  battery?: NestBatteryTelemetry | null;
  motor?: NestMotorTelemetry | null;
  autoMode?: NestAutoModeTelemetry | null;
  uavPresence?: NestUavPresenceTelemetry | null;
}
