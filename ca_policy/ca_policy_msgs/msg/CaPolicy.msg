Header header
uint8 CAPOLICY_NORMAL=0
uint8 CAPOLICY_PEOPLE=1
uint8 CAPOLICY_ROBOT=2

uint8 id
string name
uint32 robot_id  #id identifying the robot publishing the topic
                 # in case to multiple robots are sharing data.
                 # robot_id==0 means the self robot.

