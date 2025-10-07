# class of the resource (gives a hint to how it should be used)
int8 				type

# type of the resource (STATIC|DYNAMIC)
int8                mode

# whether or not the resource is in use
bool 				locked

# number of nodes currently using the resource (should be 0 unless multitasking)
int16 				owners

# proper name of the resource (e.g. 'left_arm')
string 				name

# channel for resource usage (control should always be exclusive unless multitasking)
int8 				channel 

# resource class constants
int8 				GRIPPER=1
int8				HEAD=2
int8 				BASE=3

# resource mode constants
int8                STATIC=1 # cannot be added or removed at runtime
int8                DYNAMIC=2 # can be added or removed at runtime 

# resource usage channel constants
int8				EXCLUSIVE=1
int8				SHARED=2