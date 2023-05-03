#pragma

struct RaderObstacle {
  uint32_t                      id;                                   //跟踪后物体的id 
  ObstacleTypeEnum              type;                                 //障碍物类型
  Point3D                       position;                             //车辆坐标系下的相对位置坐标，单位m
  Point3D                       position_stdev;                       //位置坐标标准差
  Point3D                       velocity;                             //车辆坐标系下相对速度，单位m/s
  Point3D                       velocity_stdev;                       //速度标准差
  Point3D                       acceleration;                         //车辆坐标系下相对加速度，单位m/s2
  float                         heading_angle;                        //航向角度，相对车辆，单位rad
  PointSize                     size;                                 //3D尺寸，包括长度、宽度、高度，单位m
  uint32_t                      polygon_vertex_num;                   //聚类多边形框顶点，最多200点
  Point3D                       polygon_vertex[200];   //各顶点坐标，用于聚类位置物体
  TrackStatusEnum               track_status;                         //跟踪状态
  uint32_t                                    confidence;                           // 置信度分数0-100
};
