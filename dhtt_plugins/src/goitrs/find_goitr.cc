#include "dhtt_plugins/goitrs/find_goitr.hpp"

namespace dhtt_plugins 
{
	void FindGoitr::init_derived(std::string node_name, std::vector<std::string> params)
	{
		(void) node_name;
		(void) params;
 	}

	void FindGoitr::destruct_derived()
	{

	}

	void FindGoitr::parent_service_callback(std::shared_ptr<dhtt_msgs::srv::GoitrRequest::Request> req, std::shared_ptr<dhtt_msgs::srv::GoitrRequest::Response> res)
	{
		(void) req;
		(void) res;
	}

	void FindGoitr::knowledge_update_callback(std::shared_ptr<std_msgs::msg::String> updated_knowledge)
	{

		// pull the object's last known location from param server
		std::string object_loc = "";

		// if we found the object at a different location we should change our destination
		if ( strcmp( this->current_destination.c_str(), object_loc.c_str() ) )
			this->change_destination(object_loc);

		// pull the robot's current location from the param server
		std::string robot_loc = "";

		// if the robot is at the destination already then skip the move behavior
		if ( not strcmp( this->current_destination.c_str(), robot_loc.c_str() ) )
			this->change_attached_queue_index(1);
	}

	void FindGoitr::child_finished_callback(std::string finished_child, bool success)
	{
		(void) finished_child;

		if ( not strcmp(finished_child.c_str(), this->look_behavior_name.c_str()) and not success )
		{
			std::string next_destination = "";

			for ( int i = 0; i < (int) this->destinations_checked.size(); i++ )
			{
				if ( not this->destinations_checked[i] )
				{
					next_destination = this->destinations_list[i];
					break;
				}
			}

			if ( not strcmp(next_destination.c_str(), "") )
			{
				// this should send a message to the parent goitr saying that the object is not at a known location in the environment 
				return;
			}

			this->change_destination(next_destination);
		}
		else if ( not strcmp(finished_child.c_str(), this->move_behavior_name.c_str()) and not success )
		{
			//here we can either change the destination or just let the tree try it again, realistically 
			// the move behavior should only fail in dynamic environments
			return;
		}

		return;

	}

	void FindGoitr::change_attached_queue_index(int new_index)
	{
		(void) new_index;
	}

	void FindGoitr::change_destination(std::string new_destination)
	{
		(void) new_destination;
	}

	void FindGoitr::change_target_object(std::string new_object)
	{
		(void) new_object;
	}

}