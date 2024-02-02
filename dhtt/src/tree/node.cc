#include "dhtt/tree/node.hpp"

namespace dhtt
{
	Node::Node(std::string name, std::string type, std::vector<std::string> params, std::string p_name) : rclcpp::Node( name ), node_type_loader("dhtt", "dhtt::NodeType"), name(name), parent_name(p_name)
	{
		this->error_msg = "";
		this->successful_load = true;

		try
		{
			this->logic = node_type_loader.createSharedInstance(type);
		}
		catch (pluginlib::PluginlibException& ex)
		{
			this->error_msg = "Error when loading plugin " + type + ": " + ex.what();
			this->successful_load = false;

			return;
		}

		this->logic->initialize(params);

		// set up services and action servers
		std::string my_register_topic = std::string(TREE_PREFIX) + name + REGISTER_CHILD_POSTFIX;

		this->register_server = this->create_service<dhtt_msgs::srv::InternalServiceRegistration>(my_register_topic, std::bind(&Node::register_child_callback, this, std::placeholders::_1, std::placeholders::_2));

		this->status.state = dhtt_msgs::msg::NodeStatus::WAITING;
	}

	Node::~Node()
	{
		// empty for now
	}

	bool Node::loaded_successfully()
	{
		return this->successful_load;
	}

	std::string Node::get_error_msg()
	{
		return this->error_msg;
	}

	void Node::register_with_parent()
	{
		// should only evaluate to false in the case of ROOT
		if ( strcmp(this->parent_name.c_str(), "NONE") )
		{
			std::shared_ptr<dhtt_msgs::srv::InternalServiceRegistration::Request> req = std::make_shared<dhtt_msgs::srv::InternalServiceRegistration::Request>();
			req->node_name = this->name;

			std::string parent_register_topic = std::string(TREE_PREFIX) + this->parent_name + REGISTER_CHILD_POSTFIX;

			rclcpp::Client<dhtt_msgs::srv::InternalServiceRegistration>::SharedPtr client = this->create_client<dhtt_msgs::srv::InternalServiceRegistration>(parent_register_topic);

			// utilizing the scope herre to avoid namespacing issues
			{
				using namespace std::chrono_literals;
				while ( not client->wait_for_service(1s) and rclcpp::ok() );

				if (rclcpp::ok())
				{

					auto result = client->async_send_request(req);

					RCLCPP_INFO(this->get_logger(), "%s", parent_register_topic.c_str());

					if ( result.wait_for(2s) == std::future_status::ready)
					{

						if (not result.get()->success)
						{
							this->error_msg = "Failed to register as a child of parent " + parent_name + ". Is it allowed to have children?";
							this->successful_load = false;
						}
					}
					else
					{
						this->error_msg = "Timed out waiting for response from service " + parent_register_topic + ". Returning in error.";
						this->successful_load = false;
					}
				}
				else
				{
					this->error_msg = "Exited while waiting for service " + parent_register_topic + " to start. Returning in error";
					this->successful_load = false;
				}
			}
		}
	}

	bool Node::remove_child(std::string child_name)
	{
		auto found_name = [&]( std::string to_check ) { return not strcmp(to_check.c_str(), child_name.c_str()); };

		std::remove_if(this->child_names.begin(), this->child_names.end(), found_name);

		return true;
	}

	void Node::register_child_callback(std::shared_ptr<dhtt_msgs::srv::InternalServiceRegistration::Request> request, std::shared_ptr<dhtt_msgs::srv::InternalServiceRegistration::Response> response)
	{
		RCLCPP_INFO(this->get_logger(), "I am not allowed to have children? %d", this->logic->can_add_child());

		if (not this->logic->can_add_child())
		{
			response->success = false;

			return;
		}

		this->child_names.push_back(request->node_name);
		response->success = true;
	}
}