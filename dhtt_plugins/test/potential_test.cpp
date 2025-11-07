#include <gtest/gtest.h>

#include "pluginlib/class_loader.hpp"
#include "rclcpp/rclcpp.hpp"

#include "dhtt/server/communication_aggregator.hpp"
#include "dhtt/server/main_server.hpp"
#include "dhtt/tree/node.hpp"
#include "dhtt/tree/potential_type.hpp"
#include "dhtt_plugins/potentials/resource_potential.hpp"
#include "dhtt_plugins/potentials/test_potential.hpp"

class TestMainServer : public dhtt::MainServer
{
  public:
	TestMainServer(std::string node_name,
				   std::shared_ptr<rclcpp::executors::MultiThreadedExecutor> spinner,
				   bool slow = false)
		: dhtt::MainServer(node_name, spinner, slow) {};

	std::shared_ptr<dhtt_msgs::srv::ModifyRequest::Response> add_from_file(std::string path)
	{
		const auto req = std::make_shared<dhtt_msgs::srv::ModifyRequest::Request>();
		auto res = std::make_shared<dhtt_msgs::srv::ModifyRequest::Response>();

		req->type = dhtt_msgs::srv::ModifyRequest::Request::ADD_FROM_FILE;
		req->to_modify = {"ROOT_0"};
		req->force = true;
		req->to_add = path;

		this->modify_callback(req, res);

		return res;
	}

	std::unordered_map<std::string, std::shared_ptr<dhtt::Node>> test_get_node_map() const
	{
		return this->get_node_map();
	};

	std::shared_ptr<dhtt::CommunicationAggregator> test_get_com_agg() const
	{
		return this->get_com_agg();
	}
};

class TestNode : public dhtt::Node
{
  public:
	TestNode(std::shared_ptr<dhtt::CommunicationAggregator> com_agg, std::string name,
			 std::string type, std::vector<std::string> params, std::string parent_name,
			 std::string socket_type = "dhtt_plugins::PtrBranchSocket", std::string goitr_type = "",
			 std::string potential_type = "dhtt_plugins::EfficiencyPotential")
		: dhtt::Node(com_agg, name, type, params, parent_name, socket_type, goitr_type,
					 potential_type)
	{
	}
	~TestNode() override = default;

	size_t fake_num_resources = 0;
	size_t fake_locked_resources = 0;
	size_t fake_subtree_resources = 0;

	void check_fake_values() const
	{
		ASSERT_LE(fake_locked_resources, fake_num_resources);
		ASSERT_LE(fake_subtree_resources, fake_num_resources);
	}

	std::vector<dhtt_msgs::msg::Resource> get_resource_state() override
	{
		check_fake_values();
		std::vector<dhtt_msgs::msg::Resource> res(fake_num_resources, dhtt_msgs::msg::Resource());
		for (auto i = 0; i < fake_locked_resources; ++i)
		{
			res[i].locked = true;
		}
		return res;
	}

	int get_subtree_resources() override
	{
		check_fake_values();
		return static_cast<int>(fake_subtree_resources);
	}
};

class TestMainServerF : public testing::Test
{
  public:
	std::shared_ptr<rclcpp::executors::MultiThreadedExecutor> spinner;
	std::shared_ptr<TestMainServer> test_main_server;

  protected:
	void SetUp() override
	{
		this->spinner = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
		this->test_main_server =
			std::make_shared<TestMainServer>("dHTT_server", this->spinner, false);

		RCLCPP_INFO(this->test_main_server->get_logger(), "Fixture Server started...");
		this->spinner->add_node(this->test_main_server);
	}

	void TearDown() override {}
};

TEST_F(TestMainServerF, test_fixture)
{
	RCLCPP_INFO(test_main_server->get_logger(), "Foo");
	RCLCPP_INFO(test_main_server->test_get_com_agg()->get_logger(), "Bar");
	std::cout << test_main_server->test_get_node_map().begin()->first << std::endl;
}

TEST_F(TestMainServerF, test_make_node)
{
	auto node = std::make_shared<dhtt::Node>(
		test_main_server->test_get_com_agg(), "Foo", "dhtt_plugins::TestBehavior",
		std::vector<std::string>({"activation_potential: 0.5"}), "ROOT_0",
		"dhtt_plugins::PtrBranchSocket", "", "dhtt_plugins::EfficiencyPotential");

	auto res = node->get_logic()->get_perceived_efficiency(node.get());
	std::cout << res << std::endl;
	ASSERT_DOUBLE_EQ(res, 0.5);
}

TEST_F(TestMainServerF, test_EfficiencyPotential)
{
	// This is a somewhat redundant test as instantiating a dhtt:Node requires loading a plugin,
	// which by default is EfficiencyPotential, so obviously it loads correctly. That said, it does
	// check the trivial functionality of returning Node::get_perceived_efficiency
	auto node = std::make_shared<dhtt::Node>(
		test_main_server->test_get_com_agg(), "Foo", "dhtt_plugins::TestBehavior",
		std::vector<std::string>({"activation_potential: 0.5"}), "ROOT_0",
		"dhtt_plugins::PtrBranchSocket", "", "dhtt_plugins::EfficiencyPotential");

	auto potential_plugin =
		pluginlib::ClassLoader<dhtt::PotentialType>("dhtt", "dhtt::PotentialType")
			.createUniqueInstance("dhtt_plugins::EfficiencyPotential");

	ASSERT_TRUE(potential_plugin);

	double res = potential_plugin->compute_activation_potential(node.get());
	std::cout << res << std::endl;
	ASSERT_DOUBLE_EQ(res, 0.5);
}

TEST_F(TestMainServerF, test_ResourcePotential)
{
	auto node = std::make_shared<TestNode>(
		test_main_server->test_get_com_agg(), "Foo", "dhtt_plugins::TestBehavior",
		std::vector<std::string>({"activation_potential: 0.5"}), "ROOT_0",
		"dhtt_plugins::PtrBranchSocket", "", "dhtt_plugins::EfficiencyPotential");

	auto potential_plugin =
		pluginlib::ClassLoader<dhtt::PotentialType>("dhtt", "dhtt::PotentialType")
			.createUniqueInstance("dhtt_plugins::ResourcePotential");

	ASSERT_TRUE(potential_plugin);

	// test below threshold
	node->fake_num_resources = 10;
	node->fake_locked_resources = (node->fake_num_resources * RESOURCE_USAGE_THRESHOLD) - 1;
	ASSERT_LT(static_cast<double>(node->fake_locked_resources) /
				  static_cast<double>(node->fake_num_resources),
			  RESOURCE_USAGE_THRESHOLD);

	auto res = potential_plugin->compute_activation_potential(node.get());
	std::cout << res << std::endl;
	ASSERT_DOUBLE_EQ(res, 0.5);

	// test below threshold
	node->fake_num_resources = 10;
	node->fake_locked_resources = (node->fake_num_resources * RESOURCE_USAGE_THRESHOLD) + 1;
	node->fake_subtree_resources = 1;
	ASSERT_GT(static_cast<double>(node->fake_locked_resources) /
				  static_cast<double>(node->fake_num_resources),
			  RESOURCE_USAGE_THRESHOLD);

	res = potential_plugin->compute_activation_potential(node.get());
	std::cout << res << std::endl;
	ASSERT_NE(res, 0.5);
}

TEST_F(TestMainServerF, test_TestPotential)
{
	auto node = std::make_shared<dhtt::Node>(
		test_main_server->test_get_com_agg(), "Foo", "dhtt_plugins::TestBehavior",
		std::vector<std::string>({"activation_potential: 0.5"}), "ROOT_0",
		"dhtt_plugins::PtrBranchSocket", "", "dhtt_plugins::EfficiencyPotential");

	auto potential_plugin =
		pluginlib::ClassLoader<dhtt::PotentialType>("dhtt", "dhtt::PotentialType")
			.createUniqueInstance("dhtt_plugins::TestPotential");

	ASSERT_TRUE(potential_plugin);

	auto res = potential_plugin->compute_activation_potential(node.get());
	std::cout << res << std::endl;
	ASSERT_DOUBLE_EQ(res, 1.0);

	auto potential_plugin_2 =
		pluginlib::ClassLoader<dhtt::PotentialType>("dhtt", "dhtt::PotentialType")
			.createUniqueInstance("dhtt_plugins::TestPotential");
	auto res1 = potential_plugin->compute_activation_potential(node.get());
	auto res2 = potential_plugin_2->compute_activation_potential(node.get());
	std::cout << "Res1: " << res1 << " Res2: " << res2 << std::endl;
	ASSERT_LT(res1, res2);
	ASSERT_DOUBLE_EQ(res2, 1.0);
}

int main(int argc, char **argv)
{
	rclcpp::init(0, nullptr);
	testing::InitGoogleTest(&argc, argv);
	auto res = RUN_ALL_TESTS();
	rclcpp::shutdown();
	return res;
}