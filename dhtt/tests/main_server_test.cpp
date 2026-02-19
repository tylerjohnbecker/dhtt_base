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

	std::shared_ptr<dhtt_msgs::srv::ModifyRequest::Response>
	reparent(const std::string &to_reparent, const std::string &new_parent)
	{
		const auto req = std::make_shared<dhtt_msgs::srv::ModifyRequest::Request>();
		auto res = std::make_shared<dhtt_msgs::srv::ModifyRequest::Response>();

		req->type = dhtt_msgs::srv::ModifyRequest::Request::REPARENT;
		req->to_modify = {to_reparent};
		req->new_parent = new_parent;

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

// TODO more tests

int main(int argc, char **argv)
{
	rclcpp::init(0, nullptr);
	testing::InitGoogleTest(&argc, argv);
	auto res = RUN_ALL_TESTS();
	rclcpp::shutdown();
	return res;
}