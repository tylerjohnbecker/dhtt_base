#include "dhtt/planning/goitr_type.hpp"

namespace dhtt
{
	void GoitrType::initialize(std::string node_name, std::vector<std::string> params) 
	{
		this->node_name = node_name;
		this->tree_built = false;
		this->keep_spinning = true;

		if ( (int) params.size() > 0 )
			this->sub_srv_ptr = std::make_shared<SubServer>(node_name, params[0], std::vector<std::string>(params.begin() + 1 , params.end()));
		else
			this->sub_srv_ptr = std::make_shared<SubServer>(node_name, "", std::vector<std::string>());

		this->start_servers();

		this->init_derived(node_name, params);

		this->spin_thread = std::make_shared<std::thread>(&GoitrType::async_spin, this);
	}

	void GoitrType::destruct()
	{
		this->keep_spinning = false;
		this->spin_thread->join();

		this->destruct_derived();

		return;
	}

	void GoitrType::init_derived(std::string node_name, std::vector<std::string> params)
	{
		(void) node_name;
		(void) params;

		return;
	}

	void GoitrType::destruct_derived()
	{
		return;
	}

	bool GoitrType::start_servers()
	{
		// make a node and executor pair
		this->executor = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
		this->executor->add_node(this->sub_srv_ptr);

		// start parent_service
		this->parent_service = this->sub_srv_ptr->create_service<dhtt_msgs::srv::GoitrRequest>(this->node_name + "/goitr_parent_interface",
									 std::bind(&GoitrType::parent_service_callback, this, std::placeholders::_1, std::placeholders::_2));
 
		// start topic subscribers

		return true;
	}

	void GoitrType::async_spin()
	{
		while ( rclcpp::ok() and this->keep_spinning )
			this->executor->spin_once(std::chrono::milliseconds(4));
	}

}