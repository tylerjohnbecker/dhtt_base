### FOR USE BY MAIN SERVER ONLY
# defines the activity to perform for this service (only works for the list of constants below)
int8 				control_code

# start command is usually only for the root node
int8				START=1
int8				GRACEFULSTOP=2
int8				IMMEDIATESTOP=3
