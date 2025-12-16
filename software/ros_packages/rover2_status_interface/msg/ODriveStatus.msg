#Message timestamp, not using header since this isn't associated with a tf frame
builtin_interfaces/Time timestamp
string bus #Assume the bus is the same for all drives in this message

#All the data for the relevant drives
#Defined as arrays so we can send multiple drives data at once:
int32[] id
string[] disarm_reason
float32[] t_fet
float32[] t_motor
float32[] velocity
float32[] position
float32[] iq_set
float32[] iq_measured
float32[] bus_voltage
float32[] bus_current
