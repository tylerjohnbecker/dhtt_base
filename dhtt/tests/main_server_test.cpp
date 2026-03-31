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
	add(const std::vector<std::string> &to_modify, const dhtt_msgs::msg::Node &add_node)
	{
		const auto req{std::make_shared<dhtt_msgs::srv::ModifyRequest::Request>()};
		auto res{std::make_shared<dhtt_msgs::srv::ModifyRequest::Response>()};

		req->type = dhtt_msgs::srv::ModifyRequest::Request::ADD;
		req->to_modify = to_modify;
		req->add_node = add_node;

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

	std::shared_ptr<dhtt_msgs::srv::ControlRequest::Response>
	save(const std::filesystem::path &path)
	{
		const auto req{std::make_shared<dhtt_msgs::srv::ControlRequest::Request>()};
		auto res{std::make_shared<dhtt_msgs::srv::ControlRequest::Response>()};

		req->type = dhtt_msgs::srv::ControlRequest::Request::SAVE;
		req->file_path = path.parent_path();
		req->file_name = path.filename();

		this->control_callback(req, res);
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
	std::string YAML;

	virtual void set_yaml() { this->YAML = SIMPLE_AND; }

	void SetUp() override
	{
		TestMainServerF::SetUp();
		set_yaml();

		file.open(PATH);
		file << YAML;
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

class TestMainServerYAML_LABELF : public TestMainServerYAMLF
{
  private:
	static constexpr auto LABEL = R"(
NodeList:
  - 'ParentAnd'
  - 'ChildAnd'

Nodes:
  ParentAnd:
    type: 1
    behavior_type: 'dhtt_plugins::AndBehavior'
    robot: 0
    parent: 'NONE'
    params: []
    labels: ['FOO']
  ChildAnd:
    type: 1
    behavior_type: 'dhtt_plugins::AndBehavior'
    robot: 0
    parent: 'ParentAnd'
    params: []
)";

	void set_yaml() override { this->YAML = LABEL; }
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

TEST_F(TestMainServerYAML_LABELF, test_label)
{
	test_main_server->add_from_file(PATH);

	const auto &node_list = test_main_server->test_get_node_list();
	const auto &node_msg_parent = find_node_in_list(node_list, "ParentAnd_1");
	const auto &node_msg_child = find_node_in_list(node_list, "ChildAnd_2");

	ASSERT_TRUE(node_msg_parent->labels[0] == "FOO");
	ASSERT_TRUE(node_msg_child->labels.empty());
}

TEST_F(TestMainServerF, test_label_add)
{
	dhtt_msgs::msg::Node my_node;
	my_node.node_name = "foo";
	my_node.parent_name = "ROOT_0";
	my_node.type = dhtt_msgs::msg::Node::AND;
	my_node.plugin_name = "dhtt_plugins::AndBehavior";
	my_node.labels = {"bar"};

	const auto res{test_main_server->add({"ROOT_0"}, my_node)};
	const auto real_node{test_main_server->test_get_node_list().tree_nodes[1]};
	ASSERT_TRUE(res->success);
	ASSERT_EQ(real_node.labels, my_node.labels);
}

TEST_F(TestMainServerYAMLF, test_save)
{
	const std::string target_file(std::filesystem::temp_directory_path() / "test_save.yaml");

	test_main_server->add_from_file(PATH);
	const auto &first_node_list = test_main_server->test_get_node_list();

	const auto save_res{test_main_server->save(target_file)};
	test_main_server->reset();
	const auto add_res{test_main_server->add_from_file(target_file)};
	const auto &reloaded_node_list = test_main_server->test_get_node_list();

	ASSERT_TRUE(save_res->success);
	ASSERT_TRUE(save_res->error_msg.empty());
	ASSERT_TRUE(add_res->success);
	ASSERT_TRUE(add_res->error_msg.empty());

	auto strip_name{[](const std::string &x) { return x.substr(0, x.find('_')); }};
	auto strip_node{[strip_name](dhtt_msgs::msg::Node &x)
					{
						x.node_name = strip_name(x.node_name);
						x.parent_name = strip_name(x.parent_name);
						for (auto &y : x.child_name)
						{
							y = strip_name(y);
						};
					}};
	auto find_index{[](const auto& list, const auto &match)
	{
		return std::distance(list.cbegin(), std::find_if(list.cbegin(), list.cend(), [match](const auto &x){return x == match;}));
	}};

	const auto _parent_and{std::find_if(reloaded_node_list.tree_nodes.cbegin(),
								 reloaded_node_list.tree_nodes.cend(), [strip_name](const auto &a)
								 { return strip_name(a.node_name) == "ParentAnd"; })};
	auto parent_and{*_parent_and};
	strip_node(parent_and);

	const auto child_and_idx{find_index(parent_and.child_name, "ChildAnd")};
	const auto child_first_task_idx{find_index(parent_and.child_name, "FirstTask")};

	// goes child < first
	ASSERT_LT(child_and_idx, child_first_task_idx);

	auto first_sorted{first_node_list.tree_nodes};
	auto second_sorted{reloaded_node_list.tree_nodes};

	const auto comp{[](const auto &a, const auto &b) { return a.node_name < b.node_name; }};
	std::sort(first_sorted.begin(), first_sorted.end(), comp);
	std::sort(second_sorted.begin(), second_sorted.end(), comp);

	for (auto &x : first_sorted)
	{
		std::sort(x.child_name.begin(), x.child_name.end());
		strip_node(x);
	}
	for (auto &x : second_sorted)
	{
		std::sort(x.child_name.begin(), x.child_name.end());
		strip_node(x);
	}

	ASSERT_EQ(first_sorted.size(), first_sorted.size());

#define myexpect(field) EXPECT_EQ(first.field, second.field)
	for (size_t i{0}; i < first_node_list.tree_nodes.size(); ++i)
	{
		const auto &first{first_sorted[i]};
		const auto &second{second_sorted[i]};

		myexpect(node_name);
		// myexpect(parent); Saved order doesn't need to be identical to the first
		myexpect(parent_name);
		// myexpect(children); Saved order doesn't need to be identical to the first
		myexpect(child_name);
		myexpect(params);
		myexpect(type);
		myexpect(plugin_name);
		myexpect(goitr_name);
		myexpect(potential_type);
		myexpect(weight);
		myexpect(bias);
		myexpect(owned_resources);
		myexpect(subtree_owned_resources);
		myexpect(node_status);
		myexpect(preconditions);
		myexpect(postconditions);
		myexpect(labels);
	}
}

TEST_F(TestMainServerYAML_LABELF, test_save_label)
{
	const std::string target_file(std::filesystem::temp_directory_path() / "test_save.yaml");

	test_main_server->add_from_file(PATH);
	const auto save_res{test_main_server->save(target_file)};
	test_main_server->reset();
	const auto add_res{test_main_server->add_from_file(target_file)};
	const auto &reloaded_node_list = test_main_server->test_get_node_list();

	ASSERT_TRUE(save_res->success);
	ASSERT_TRUE(save_res->error_msg.empty());
	ASSERT_TRUE(add_res->success);
	ASSERT_TRUE(add_res->error_msg.empty());

	ASSERT_TRUE((reloaded_node_list.tree_nodes.cend() - 1)->labels.empty());
	ASSERT_EQ((reloaded_node_list.tree_nodes.cend() - 2)->labels[0], "FOO");
}

TEST_F(TestMainServerYAMLF, test_save_no_underscore_and_dedup)
{
	const std::string target_file(std::filesystem::temp_directory_path() / "test_save.yaml");
	test_main_server->add_from_file(PATH);

	dhtt_msgs::msg::Node to_add;
	to_add.type = dhtt_msgs::msg::Node::AND;
	to_add.node_name = "ParentAnd";
	to_add.plugin_name = "dhtt_plugins::AndBehavior";
	auto add_res{test_main_server->add({"ROOT_0"}, to_add)};
	ASSERT_TRUE(add_res->success);

	const auto save_res{test_main_server->save(target_file)};

	YAML::Node root{YAML::LoadFile(target_file)};

	const auto node_list{root["NodeList"].as<std::vector<std::string>>()};
	std::vector<std::string> node_names;
	std::vector<std::string> node_parents;

	for (const auto &x : root["Nodes"])
	{
		node_names.push_back(x.first.as<std::string>());
		node_parents.push_back(x.second["parent"].as<std::string>());
	}

	// Shouldn't have the underscore suffix
	for (const auto &vec : {node_list, node_names, node_parents})
	{
		for (const auto &x : vec)
		{
			ASSERT_EQ(x.find('_'), std::string::npos);
		}
	}

	// Shouldn't have duplicates
	for (auto x{node_list.begin()}; x < node_list.end(); ++x)
	{
		for (auto y{x + 1}; y < node_list.end(); ++y)
		{
			ASSERT_NE(*x, *y);
		}
	}
	for (auto x{node_names.begin()}; x < node_names.end(); ++x)
	{
		for (auto y{x + 1}; y < node_names.end(); ++y)
		{
			ASSERT_NE(*x, *y);
		}
	}
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