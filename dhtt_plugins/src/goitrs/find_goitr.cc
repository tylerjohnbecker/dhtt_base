#include "dhtt_plugins/goitrs/find_goitr.hpp"

namespace dhtt_plugins 
{
	void FindGoitr::init_derived(std::string node_name, std::vector<std::string> params)
	{
		(void) node_name;
		(void) params;

		this->destinations_list = this->params_client_ptr->get_parameters({"world.locations"})[0].as_string_array();
		this->destinations_checked.resize(this->destinations_list.size());

		std::fill(this->destinations_checked.begin(), this->destinations_checked.end(), false);

		// for ( auto iter : params )
		// 	RCLCPP_FATAL(this->sub_srv_ptr->get_logger(), "\t\t\t PARAM: %s", iter.c_str());

		// get current destination
		auto separator_pos = params[2].find(": ");
		std::string value = params[2].substr(separator_pos + 2, params[2].size() - separator_pos); 

		this->current_destination = value;

		// get target object
		separator_pos = params[3].find(": ");
		value = params[3].substr(separator_pos + 2, params[3].size() - separator_pos); 

		this->object_target = value;
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
		(void) updated_knowledge;

		// pull the object's last known location from param server
		std::string object_loc = this->params_client_ptr->get_parameter<std::string>("world.objects." + this->object_target + ".location");
		double object_prob = this->params_client_ptr->get_parameter<double>("world.objects." + this->object_target + ".probability");

		bool object_found = false;

		// if we found the object at a different location we should change our destination
		if ( strcmp( this->current_destination.c_str(), object_loc.c_str() ) and object_prob > OBJECT_PROB_THRESHOLD )
		{
			this->change_destination(object_loc);
		
			object_found = true;
		}

		// pull the robot's current location from the param server
		std::string robot_loc = this->params_client_ptr->get_parameter<std::string>("world.robot.location");

		if ( object_found )
		{
			// if the robot is at the destination already then skip the move behavior
			if ( not strcmp( this->current_destination.c_str(), robot_loc.c_str() ) or object_prob > OBJECT_PROB_THRESHOLD )
				this->change_attached_queue_index(2);
			else
				this->change_attached_queue_index(0);
		}
		else if ( not strcmp( this->current_destination.c_str(), robot_loc.c_str() ) )
		{
			// std::string next_destination = this->current_destination;

			// for ( int i = 0; i < (int) this->destinations_list.size(); i++ )
			// {
			// 	if ( not strcmp( this->destinations_list[i].c_str(), robot_loc.c_str() ) )
			// 	{
			// 		this->destinations_checked[i] = true;

			// 		next_destination = this->destinations_list[i + 1];

			// 		break; 
			// 	}
			// }

			// this->change_destination(next_destination);
			// this->change_attached_queue_index(0);
		}
	}

	void FindGoitr::child_finished_callback(std::string finished_child, bool success)
	{
		if ( not strcmp(finished_child.c_str(), this->look_behavior_name.c_str()) and not success )
		{
			std::string robot_loc = this->params_client_ptr->get_parameter<std::string>("world.robot.location");
			
			std::string next_destination = "";

			for ( int i = 0; i < (int) this->destinations_checked.size(); i++ )
			{
				// mark the robot location as checked
				if ( not strcmp(robot_loc.c_str(), this->destinations_list[i].c_str()) )
					this->destinations_checked[i] = true;

				// the next unchecked one should be the new destination
				if ( not this->destinations_checked[i] )
				{
					next_destination = this->destinations_list[i];
				}
			}

			if ( not strcmp(next_destination.c_str(), "") )
			{
				// this should send a message to the parent goitr saying that the object is not at a known location in the environment 
				return;
			}

			this->change_destination(next_destination);
			this->change_attached_queue_index(0);
			return;
		}
		else if ( not strcmp(finished_child.c_str(), this->move_behavior_name.c_str()) and not success )
		{
			//here we can either change the destination or just let the tree try it again, realistically 
			// the move behavior should only fail in dynamic environments
			return;
		}

		return;

	}

	void FindGoitr::first_activation_callback()
	{

		this->sub_srv_ptr->build_subtree();

		this->block_for_thread();

		// now fetch the tree to get the names of the child nodes
		std::shared_ptr<dhtt_msgs::srv::FetchRequest::Response> res = std::make_shared<dhtt_msgs::srv::FetchRequest::Response>();

		bool tree = this->sub_srv_ptr->fetch_subtree(res);

		this->block_for_thread();

		if ( not tree or res->found_subtrees.size() < 1)
		{
			RCLCPP_ERROR(this->sub_srv_ptr->get_logger(), "Could not fetch subtree throwing exception");
			throw std::exception();
		}

		this->move_behavior_name = res->found_subtrees[0].tree_nodes[1].node_name;
		this->look_behavior_name = res->found_subtrees[0].tree_nodes[2].node_name;

		RCLCPP_INFO(this->sub_srv_ptr->get_logger(), "Registered behaviors %s and %s as children...", this->move_behavior_name.c_str(), this->look_behavior_name.c_str());

		// this->change_attached_queue_index(0);

		return;
	}

	void FindGoitr::change_attached_queue_index(int new_index)
	{
		RCLCPP_INFO(this->sub_srv_ptr->get_logger(), "Sending change queue index %d request for %s", new_index, this->node_name.c_str());

		std::vector<std::string> n_params;
		n_params.push_back("child_queue_index: " + std::to_string(new_index));

		this->sub_srv_ptr->change_params(this->node_name, n_params);
	}

	void FindGoitr::change_destination(std::string new_destination)
	{
		RCLCPP_INFO(this->sub_srv_ptr->get_logger(), "Sending change destination request for %s", this->move_behavior_name.c_str());

		this->current_destination = new_destination;

		std::vector<std::string> n_params;
		n_params.push_back("activation_potential: -1");
		n_params.push_back("dest: " + this->current_destination);

		this->sub_srv_ptr->change_params(this->move_behavior_name, n_params);
	}

	void FindGoitr::change_target_object(std::string new_object)
	{
		RCLCPP_INFO(this->sub_srv_ptr->get_logger(), "Sending change target request for %s", this->look_behavior_name.c_str());

		this->object_target = new_object;

		std::vector<std::string> n_params;
		n_params.push_back("activation_potential: -1");
		n_params.push_back("object: " + this->object_target);

		this->sub_srv_ptr->change_params(this->look_behavior_name, n_params);
	}

}