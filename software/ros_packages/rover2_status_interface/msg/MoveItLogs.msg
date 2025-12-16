# The header specifies the time at which the joint states were recorded. All the joint states in one message have to be recorded at the same time.
std_msgs/Header header
string[] name # Each joint is uniquely identified by its name
float64[] position
float64[] velocity
float64[] effort
int8 status #not directly associated with info in header but time stamp is possibly useful and close enough hopefully if anyone cares
