#include "dhtt/planning/goitr_type.hpp"

namespace dhtt
{
	void GoitrType::initialize(std::string node_name, std::vector<std::string> params) 
	{
		this->sub_srv_ptr = std::make_shared<SubServer>(node_name);

		this->node_name = node_name;

		this->start_servers();

		this->init_derived(node_name, params);
	}

	void GoitrType::init_derived(std::string node_name, std::vector<std::string> params)
	{
		(void) node_name;
		(void) params;

		return;
	}

	bool GoitrType::start_servers()
	{

		// just a random hopefully unique name
		const void * address = static_cast<const void*>(this);
		std::stringstream ss;
		ss << "goitr_" << address;

		// make a node and executor pair
		this->executor = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
		this->executor->add_node(this->sub_srv_ptr);

		// start parent_service
		this->parent_service = this->sub_srv_ptr->create_service<dhtt_msgs::srv::GoitrRequest>(this->node_name + "/goitr_parent_interface",
									 std::bind(&GoitrType::parent_service_callback, this, std::placeholders::_1, std::placeholders::_2));
 
		// start topic subscribers

		return true;
	}

}