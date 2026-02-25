#include <filesystem>
#include <set>

#include <gtest/gtest.h>

#include "rclcpp/rclcpp.hpp"

#include "dhtt/server/communication_aggregator.hpp"
#include "dhtt/server/main_server.hpp"
#include "dhtt/tree/node.hpp"

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

	std::shared_ptr<dhtt_msgs::srv::FetchRequest::Response> fetch()
	{
		const auto req = std::make_shared<dhtt_msgs::srv::FetchRequest::Request>();
		const auto res = std::make_shared<dhtt_msgs::srv::FetchRequest::Response>();

		req->return_full_subtree = true;
		this->fetch_callback(req, res);
		return res;
	}

	std::shared_ptr<dhtt_msgs::srv::ControlRequest::Response> start()
	{
		const auto req = std::make_shared<dhtt_msgs::srv::ControlRequest::Request>();
		const auto res = std::make_shared<dhtt_msgs::srv::ControlRequest::Response>();

		req->type = dhtt_msgs::srv::ControlRequest::Request::START;
		this->control_callback(req, res);
		return res;
	}

	std::shared_ptr<dhtt_msgs::srv::ControlRequest::Response> reset()
	{
		const auto req = std::make_shared<dhtt_msgs::srv::ControlRequest::Request>();
		const auto res = std::make_shared<dhtt_msgs::srv::ControlRequest::Response>();

		req->type = dhtt_msgs::srv::ControlRequest::Request::RESET;
		this->control_callback(req, res);
		return res;
	}

	std::shared_ptr<dhtt_msgs::srv::ModifyRequest::Response>
	reparent(const std::string &to_reparent, const std::string &new_parent)
	{
		const auto req = std::make_shared<dhtt_msgs::srv::ModifyRequest::Request>();
		const auto res = std::make_shared<dhtt_msgs::srv::ModifyRequest::Response>();

		req->type = dhtt_msgs::srv::ModifyRequest::Request::REPARENT;
		req->to_modify = {to_reparent};
		req->new_parent = new_parent;

		this->modify_callback(req, res);
		return res;
	}

	std::shared_ptr<dhtt_msgs::srv::ModifyRequest::Response> reweight(const std::string &to_modify,
																	  const double weight)
	{
		const auto req = std::make_shared<dhtt_msgs::srv::ModifyRequest::Request>();
		auto res = std::make_shared<dhtt_msgs::srv::ModifyRequest::Response>();

		req->type = dhtt_msgs::srv::ModifyRequest::Request::REWEIGHT;
		req->to_modify = {to_modify};
		req->weight = weight;

		this->modify_callback(req, res);
		return res;
	}

	std::shared_ptr<dhtt_msgs::srv::ModifyRequest::Response> rebias(const std::string &to_modify,
																	const double bias)
	{
		const auto req = std::make_shared<dhtt_msgs::srv::ModifyRequest::Request>();
		auto res = std::make_shared<dhtt_msgs::srv::ModifyRequest::Response>();

		req->type = dhtt_msgs::srv::ModifyRequest::Request::REBIAS;
		req->to_modify = {to_modify};
		req->bias = bias;

		this->modify_callback(req, res);
		return res;
	}

	auto test_get_node_map() const { return this->get_node_map(); }

	auto test_get_node_list() const { return this->get_node_list(); }

	std::shared_ptr<dhtt::CommunicationAggregator> test_get_com_agg() const
	{
		return this->get_com_agg();
	}

	auto find_parent(std::string child_name)
	{
		for (const auto &[node_name, node_ptr] : this->get_node_map())
		{
			const auto &child_names = node_ptr->get_child_names();
			if (std::find(child_names.cbegin(), child_names.cend(), child_name) !=
				child_names.cend())
			{
				return node_ptr;
			}
		}
		throw std::runtime_error("Cannot find a parent of " + child_name);
	}
};

class TestMainServerF : public testing::Test
{
  private:
	std::shared_ptr<rclcpp::executors::MultiThreadedExecutor> spinner;
	std::thread t;

  public:
	std::shared_ptr<TestMainServer> test_main_server;

	static auto find_node_in_list(const dhtt_msgs::msg::Subtree &subtree, const std::string &name)
	{
		return std::find_if(subtree.tree_nodes.cbegin(), subtree.tree_nodes.cend(),
							[name](const auto &check) { return check.node_name == name; });
	}

	static auto
	wait_for_done(const std::unordered_map<std::string, std::shared_ptr<dhtt::Node>> &map)
	{
		while (map.at("ROOT_0")->get_status().state != dhtt_msgs::msg::NodeStatus::DONE)
			;
	}

  protected:
	void SetUp() override
	{
		this->spinner = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
		this->test_main_server =
			std::make_shared<TestMainServer>("dHTT_server", this->spinner, false);

		RCLCPP_INFO(this->test_main_server->get_logger(), "Fixture Server started...");
		this->spinner->add_node(this->test_main_server);

		// Has to spin in its own thread because spinner->spin() blocks until cancel()
		t = std::thread(&rclcpp::executors::MultiThreadedExecutor::spin, spinner);
	}

	void TearDown() override
	{
		this->spinner->cancel();
		t.join();
	}
};

class TestMainServerYAMLF : public TestMainServerF
{
  private:
	static constexpr auto SIMPLE_AND = R"(
NodeList:
  - 'ParentAnd'
  - 'ChildAnd'
  - 'FirstTask'
  - 'SecondTask'

Nodes:
  ParentAnd:
    type: 1
    behavior_type: 'dhtt_plugins::AndBehavior'
    robot: 0
    parent: 'NONE'
    params: []
  ChildAnd:
    type: 1
    behavior_type: 'dhtt_plugins::AndBehavior'
    robot: 0
    parent: 'ParentAnd'
    params: []
  SecondTask:
    type: 4
    behavior_type: 'dhtt_plugins::TestBehavior'
    robot: 0
    parent: 'ChildAnd'
    params: ["activation_potential: 5.0"]
  FirstTask:
    type: 4
    behavior_type: 'dhtt_plugins::TestBehavior'
    robot: 0
    parent: 'ParentAnd'
    params: ["activation_potential: 10.0"]
)";

	std::ofstream file;

  protected:
	void SetUp() override
	{
		TestMainServerF::SetUp();
		file.open(PATH);
		file << SIMPLE_AND;
		file.close();
	}

	void TearDown() override
	{
		TestMainServerF::TearDown();
		if (auto ret = std::filesystem::remove(PATH); not ret)
		{
			std::cout << "failed to delete file" << std::endl;
			;
		}
	}

  public:
	const std::filesystem::path PATH = std::filesystem::temp_directory_path() / "simple_and.yaml";
};

TEST_F(TestMainServerF, test_fixture)
{
	RCLCPP_INFO(test_main_server->get_logger(), "Foo");
	RCLCPP_INFO(test_main_server->test_get_com_agg()->get_logger(), "Bar");
	std::cout << test_main_server->test_get_node_map().begin()->first << std::endl;
}

TEST_F(TestMainServerYAMLF, test_add_from_file)
{
	test_main_server->add_from_file(PATH);
	const auto &node_map = test_main_server->test_get_node_map();

	static const auto NODES =
		std::set<std::string>{"SecondTask_4", "ChildAnd_2", "FirstTask_3", "ParentAnd_1", "ROOT_0"};
	for (const auto &[x, _] : node_map)
	{
		EXPECT_NE(NODES.find(x), NODES.cend());
	}

	ASSERT_THROW(test_main_server->find_parent("foo"), std::runtime_error);
	ASSERT_EQ(test_main_server->find_parent("FirstTask_3")->get_node_name(), "ParentAnd_1");
}

TEST_F(TestMainServerYAMLF, test_reparent)
{
	static constexpr auto TO_REPARENT = "FirstTask_3";
	static constexpr auto OLD_PARENT = "ParentAnd_1";
	static constexpr auto NEW_PARENT = "ChildAnd_2";

	test_main_server->add_from_file(PATH);
	const auto &node_map_before = test_main_server->test_get_node_map();

	ASSERT_EQ(test_main_server->find_parent(TO_REPARENT)->get_node_name(), OLD_PARENT);
	ASSERT_EQ(test_main_server->find_parent(TO_REPARENT),
			  node_map_before.at(OLD_PARENT)); // should point to same object too

	auto res = test_main_server->reparent(TO_REPARENT, NEW_PARENT);
	ASSERT_TRUE(res->success and res->error_msg.empty());

	const auto &node_map_after = test_main_server->test_get_node_map();
	// Should see the new parent
	ASSERT_EQ(test_main_server->find_parent(TO_REPARENT)->get_node_name(), NEW_PARENT);
	ASSERT_EQ(test_main_server->find_parent(TO_REPARENT),
			  node_map_after.at(NEW_PARENT)); // should point to same object too
	// Should not see the old parent
	ASSERT_NE(test_main_server->find_parent(TO_REPARENT)->get_node_name(), OLD_PARENT);
	ASSERT_NE(test_main_server->find_parent(TO_REPARENT),
			  node_map_after.at(OLD_PARENT)); // should point to same object too
}

TEST_F(TestMainServerYAMLF, test_reweight_init)
{
	static std::string TO_MODIFY = "FirstTask_3";

	test_main_server->add_from_file(PATH);
	const auto &node_map_before = test_main_server->test_get_node_map();
	const auto &node_before = node_map_before.at(TO_MODIFY);
	const auto &[_ap, w_before, b_before] = node_before->get_activation_potential();
	const auto &node_list_before = test_main_server->test_get_node_list();
	const auto &node_msg_before = find_node_in_list(node_list_before, TO_MODIFY);

	ASSERT_DOUBLE_EQ(w_before, 1.0);
	ASSERT_DOUBLE_EQ(b_before, 0.0);
	ASSERT_DOUBLE_EQ(node_msg_before->weight, 1.0);
	ASSERT_DOUBLE_EQ(node_msg_before->bias, 0.0);

	test_main_server->start();
	wait_for_done(node_map_before);
	const auto &ap_before = std::get<0>(node_before->get_activation_potential());
	ASSERT_DOUBLE_EQ(ap_before, 10.0);
}

TEST_F(TestMainServerYAMLF, test_reweight)
{
	static std::string TO_MODIFY = "FirstTask_3";

	test_main_server->add_from_file(PATH);

	const auto &res_1{test_main_server->reweight(TO_MODIFY, 5.0)};
	ASSERT_TRUE(res_1->success and res_1->error_msg.empty());
	const auto &res_2{test_main_server->rebias(TO_MODIFY, 1.0)};
	ASSERT_TRUE(res_2->success and res_2->error_msg.empty());

	const auto &node_map_before = test_main_server->test_get_node_map();
	const auto &node_before = node_map_before.at(TO_MODIFY);
	const auto &[_ap, w_before, b_before] = node_before->get_activation_potential();
	const auto &node_list_before = test_main_server->test_get_node_list();
	const auto &node_msg_before = find_node_in_list(node_list_before, TO_MODIFY);

	ASSERT_DOUBLE_EQ(w_before, 5.0);
	ASSERT_DOUBLE_EQ(b_before, 1.0);
	ASSERT_DOUBLE_EQ(node_msg_before->weight, 5.0);
	ASSERT_DOUBLE_EQ(node_msg_before->bias, 1.0);

	test_main_server->start();
	wait_for_done(node_map_before);
	const auto &ap_before = std::get<0>(node_before->get_activation_potential());
	ASSERT_DOUBLE_EQ(ap_before, 51.0);
}

TEST_F(TestMainServerYAMLF, test_reweight_negative)
{
	static std::string TO_MODIFY = "FirstTask_3";

	test_main_server->add_from_file(PATH);

	// Empty to_modify list
	const auto &res_0{test_main_server->reweight({}, 1.0)};
	ASSERT_FALSE(res_0->success);
	ASSERT_FALSE(res_0->error_msg.empty());

	// Negative weight
	const auto &res_1{test_main_server->reweight(TO_MODIFY, -1.0)};
	ASSERT_FALSE(res_1->success);
	ASSERT_FALSE(res_1->error_msg.empty());

	// Zero weight should be allowed, visual check for roslog warning
	const auto &res_2{test_main_server->reweight(TO_MODIFY, 0.0)};
	ASSERT_TRUE(res_2->success);
	ASSERT_TRUE(res_2->error_msg.empty());

	// Negative bias should be allowed, visual check for roslog warning
	const auto &res_3{test_main_server->rebias(TO_MODIFY, -10.0)};
	ASSERT_TRUE(res_3->success);
	ASSERT_TRUE(res_3->error_msg.empty());

	// Should clamp negative activation to 0
	test_main_server->start();
	const auto &node_map_before = test_main_server->test_get_node_map();
	const auto &node_before = node_map_before.at(TO_MODIFY);
	std::this_thread::sleep_for(std::chrono::milliseconds(
		15)); // Root node with impossible children will stay in WAITING state
	const auto &ap_before = std::get<0>(node_before->get_activation_potential());
	ASSERT_DOUBLE_EQ(ap_before, 0.0);
}

// TODO more tests

int main(int argc, char **argv)
{
	rclcpp::init(0, nullptr);
	testing::InitGoogleTest(&argc, argv);
	auto res = RUN_ALL_TESTS();
	rclcpp::shutdown();
	return res;
}