float64 battery_soc #电池电量，%
float64 charging_volt #充电电压，精度0.01V
float64 charging_current #充电电流，精度0.01A
float64 battery_temp1 #电池温度1， 精度0.1℃
float64 battery_temp2 #电池温度2， 精度0.1℃
float64 pcb_temp1 #板卡温度1， 精度0.1℃
float64 highest_cell_volt #单体电压最大压差, 精度0.01V
int max_cell_no #最大电池编号，默认为6,表示下部单体电压个数
float64[] cell_volt #单体电压最大压差, 精度0.01V
string battery_charged #电池充电状态，0是关闭，1是打开
string highest_cell_no #单体电池最高电压编号
string lowest_cell_no #单体电池最低电压编号
string battery_supply #电池供电状态，0是关闭，1是打开
string battery_balance #电池均衡状态，0是关闭，1是打开
string charging_OK #充电口触点是否正常接触,1为正常接触，0为未接触
