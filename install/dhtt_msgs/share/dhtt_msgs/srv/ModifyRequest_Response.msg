
# returns whether or not a node was found, and whether it could be modified successfully
bool 				success

string				error_msg

# optional: for adding nodes their names will be changed as they are added to ensure they are unique. Therefore they will also be sent back here.
string[]			added_nodes

# optional: in case a node fails to be removed all successfully removed nodes will have their names given here
string[] 			removed_nodes