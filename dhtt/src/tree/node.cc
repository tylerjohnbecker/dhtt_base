#include "dhtt/tree/node.hpp"

namespace dhtt
{
	Node::Node(std::string type, std::vector<std::string> params, std::string p_name) : parent_name(p_name)
	{
		pluginlib::ClassLoader<NodeType> node_type_loader("dhtt", "dhtt::NodeType");

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
		}

		this->logic->initialize(params);

		this->status.state = dhtt_msgs::msg::NodeStatus::WAITING;

		// register with parent
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
};