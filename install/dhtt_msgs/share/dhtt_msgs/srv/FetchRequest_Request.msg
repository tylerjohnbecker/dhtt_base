# if this parameter is true then each time a candidate node is found it's entire subtree of children will be returned as well
bool 				return_full_subtree

# optional: find nodes which contain this common name 
string 				common_name

# optional: find node with this name
string 				node_name

# optional: find nodes of this type
int8 				node_type

# node type declarations (for specific behavior types use the common name parameter as well)
int8 				ROOT=1
int8 				AND=2
int8 				THEN=3
int8 				OR=4
int8 				BEHAVIOR=5
