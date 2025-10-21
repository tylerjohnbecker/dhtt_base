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

		this->param_node_ptr = std::make_shared<rclcpp::Node>(node_name + "_params", rclcpp::NodeOptions().allow_undeclared_parameters(true).automatically_declare_parameters_from_overrides(true));

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

	void GoitrType::first_activation_callback()
	{
		this->sub_srv_ptr->build_subtree();

		this->block_for_thread();
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
		this->knowledge_server_subscriber = this->sub_srv_ptr->create_subscription<std_msgs::msg::String>("/updated_knowledge", 10,  
									std::bind(&GoitrType::knowledge_update_callback, this, std::placeholders::_1));

		// start sync params client
		this->params_client_ptr = std::make_shared<rclcpp::SyncParametersClient>(this->param_node_ptr, "/param_node");

		{
			using namespace std::chrono_literals;

			while ( not this->params_client_ptr->wait_for_service(1s) and rclcpp::ok() ); 
		};

		// this->params_executor = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
		// this->executor->add_node(param_node_ptr);

		return true;
	}

	void GoitrType::block_for_thread()
	{
		while ( rclcpp::ok() and this->sub_srv_ptr->threads_running > 0 );
	}

	void GoitrType::async_spin()
	{
		while ( rclcpp::ok() and this->keep_spinning )
		{
			this->executor->spin_once(std::chrono::milliseconds(4));
			// this->params_executor->spin_node_once(this->param_node_ptr, std::chrono::milliseconds(4));
		}
	}

}