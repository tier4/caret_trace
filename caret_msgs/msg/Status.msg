# request constants
int8 UNINITIALIZED=0
int8 WAIT=1
int8 PREPARE=2
int8 RECORD=3

# request fields
string caret_node_name
int8 status
string[] node_names # reserved
int64 pid # reserved
