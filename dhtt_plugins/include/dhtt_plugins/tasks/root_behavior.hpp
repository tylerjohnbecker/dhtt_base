#ifndef ROOT_BEHAVIOR_HPP
#define ROOT_BEHAVIOR_HPP

#include "dhtt_plugins/visibility_control.h"

#include "rclcpp/rclcpp.hpp"

#include "yaml-cpp/yaml.h"
#include "yaml-cpp/exceptions.h"

#include "dhtt/tree/node.hpp"
#include "dhtt/tree/node_type.hpp"

#include "dhtt_msgs/msg/resource.hpp"
#include "dhtt_msgs/msg/resources.hpp"

#include <vector>
#include <string>

namespace dhtt_plugins
{
	class RootBehavior : public dhtt::NodeType
	{
	public:

		void initialize(std::vector<std::string> params) override;

		std::shared_ptr<dhtt_msgs::action::Activation::Result> auction_callback( dhtt::Node* container ) override;
		std::shared_ptr<dhtt_msgs::action::Activation::Result> work_callback( dhtt::Node* container ) override;

		void parse_params( std::vector<std::string> params ) override;

		double get_perceived_efficiency() override;

		std::vector<dhtt_msgs::msg::Resource> get_retained_resources( dhtt::Node* container ) override;
		std::vector<dhtt_msgs::msg::Resource> get_released_resources( dhtt::Node* container ) override;

		bool is_done() override;

	protected:
		void load_resources_from_yaml();
		void publish_resources();

		std::vector<dhtt_msgs::msg::Resource> give_resources(std::vector<dhtt_msgs::msg::Resource> to_give);
		void release_resources(std::vector<dhtt_msgs::msg::Resource> to_release);

		std::string robot_resources_file_path;

		std::vector<dhtt_msgs::msg::Resource> canonical_resources_list;
		std::string robot_name;

		bool children_done;

		std::shared_ptr<rclcpp::Node> pub_node_ptr;
		std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> resource_executor;
		rclcpp::Publisher<dhtt_msgs::msg::Resources>::SharedPtr status_pub;

	private:
	};
}

#endif //ROOT_BEHAVIOR_HPP