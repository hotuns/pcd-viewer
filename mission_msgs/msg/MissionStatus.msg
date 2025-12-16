int32 id                      #任务ID
int32 status                  #任务状态 执行/暂停/完成/返航
geometry_msgs/PoseStamped HomePos   #起飞与返回点
int32 PosNum                  #航点数量
WaypointPosition[] PosList          #航点列表