#ifndef COOKING_BEHAVIOR_HPP
#define COOKING_BEHAVIOR_HPP

#include "dhtt_msgs/msg/cooking_observation.hpp"
#include "dhtt_msgs/srv/cooking_request.hpp"
#include "dhtt_plugins/behaviors/action_type.hpp"
#include "rclcpp/rclcpp.hpp"
#include <float.h>

namespace dhtt_plugins
{
class CookingBehavior : public ActionType
{
  public:
	// TODO refactor subclass do_work() for redundant action requests
	// void do_work(...)

	/**
	 * \brief parses activation_potential and destination parameters for the move behavior
	 *
	 * The meaning of each parameter is as follows:
	 *		- activation_potential: a given activation potential for the behavior (for testing
	 *specific behavior orders)
	 * 		- coord: a coordinate in format 'x, y'. Leave empty "" if not used
	 * 		- object: a cooking object type name to automatically find the closest of that
	 * 		- conditions: an optional comma-separated string of conditions the object needs to have.
	 * 		- mark: an optional taint to look for on the paramserver
	 *i.e. 'conditions: Chopped, Toasted'. Also supports 'Free' for objects containing nothing, or
	 *'Contains' for objects containing something. Further, specifying 'Contains+"listofobjects"'
	 *allows a list of objects to be contained,
	 *i.e. params: [ 'object: Plate', 'conditions: Contains+"Lettuce, Tomato"', 'mark: A1' ]
	 *
	 * \param params a vector of string params parsed from the yaml description
	 *
	 * \return void
	 */
	void parse_params(std::vector<std::string> params) override;

	/**
	 * Some handy checks common to CookingBehavior-types. Individual behaviors need to implement
	 * their own metric.
	 * @return 0 if the checks failed, 1 otherwise.
	 */
	virtual double get_perceived_efficiency(dhtt::Node* container) override;

  protected:
	std::string destination_type;  // key: coordinate or closest object
	std::string destination_value; // value: of destination_type, i.e. either '1,1' or 'Toaster'
	std::string destination_conditions; // conditions for closest object
	std::string destination_mark;
	geometry_msgs::msg::Point destination_point;
	dhtt_msgs::msg::CookingObject destination_object; // closest object
	bool destination_is_good = false;
	bool should_unmark = false;

	bool updated = false;

	static double point_distance(const geometry_msgs::msg::Point &point1,
								 const geometry_msgs::msg::Point &point2);
	double agent_point_distance(const geometry_msgs::msg::Point &point) const;
	void observation_callback(std::shared_ptr<dhtt_msgs::msg::CookingObservation> msg);

	/**
	 * Calls base initialize() then perform some CookingBehavior-specific initialization
	 (Client and Subscriber creation).
	 */
	void initialize(std::vector<std::string> params) override;

	/**
	 * Using this->destination_value, this->destination_conds, and this->last_obs, set
	 * this->destination_point
	 */
	void set_destination_to_closest_object();

	static std::string which_arm(dhtt::Node *container);

	/**
	 * @return whether activation potential and destination_is_good are correct for us to work
	 */
	bool can_work() const;

	/**
	 * @param obj CookingObject to check it's world_id and this->destination_mark against marked
	 * objects on the paramserver
	 * @return '0' if obj is marked with a different taint, '1' if obj is marked with our
	 * this->destination_mark, '2' if obj is not marked. Note that these are character '1' not 1.
	 */
	// TODO use an enum for this
	char check_mark(const dhtt_msgs::msg::CookingObject &obj) const;

	bool mark_object(unsigned long object_id, const std::string &mark) const;

	// Assumes static object and this->destination_object have the same mark
	bool unmark_static_object_under_obj(const dhtt_msgs::msg::CookingObject &obj) const;

	// TODO clear all on tree reset
	bool unmark_object(unsigned long object_id) const;
	
	/**
	 * \brief sends a client request to the cooking server and blocks for a response update message
	 * 
	 * \param to_send request for the server
	 * 
	 * \return response from the server
	 * 
	 */
	std::shared_future<dhtt_msgs::srv::CookingRequest::Response::SharedPtr> send_request_and_update(dhtt_msgs::srv::CookingRequest::Request::SharedPtr to_send);

  std::string cooking_observation_subscriber;

	rclcpp::Client<dhtt_msgs::srv::CookingRequest>::SharedPtr cooking_request_client;

	std::shared_ptr<rclcpp::Node> param_node_ptr;
	std::shared_ptr<rclcpp::SyncParametersClient> params_client_ptr;

	std::shared_ptr<dhtt_msgs::msg::CookingObservation> last_obs;

	static constexpr auto PARAM_ACTIVATION_POTENTIAL = "activation_potential";
	static constexpr auto PARAM_COORDINATE = "coord";
	static constexpr auto PARAM_OBJECT_TYPE = "object";
	static constexpr auto PARAM_OBJECT_CONDITIONS = "conditions";
	static constexpr auto PARAM_OBJECT_MARK = "mark";
	static constexpr auto PARAM_MARK_OBJECTS = "world.marked_objects_ids";
	static constexpr auto PARAM_MARK_OBJECTS_TAINTS = "world.marked_objects_taints";
	static constexpr auto PARAM_MARK_OBJECTS_TYPES = "world.marked_objects_types";
	static constexpr auto PARAM_UNMARK = "unmark";
	const int LEVEL_SIZE = 6;

  private:
	/**
	 *
	 * @param conds_str an optional comma-separated string of conditions the object needs to have.
	 * i.e. 'conditions: Chopped, Toasted'. Also supports 'Free' for objects containing nothing, or
	 * 'Contains' for objects containing something. Further, specifying 'Contains+"listofobjects"'
	 * allows a list of objects to be contained, i.e. params: [ 'object: Plate', 'conditions:
	 * Contains+"Lettuce, Tomato"' ]
	 * @return vector of each condition (i.e. separate out the csv). Contains+"..." is one value and
	 * needs to be parsed separately.
	 */
	static std::vector<std::string> parse_conds_string(const std::string &conds_str);

	/**
	 *
	 * @param cond_string Condition string formatted as 'Contains+"item1, item2, ..."' the double
	 * quotes are included, single quotes not. Further alternate lists can be delimited with a pipe
	 * '|' as a logical or.
	 * @param obj_contained_ids ids of the objects contained by this object (i.e.
	 * CookingObject->content_ids)
	 * @return Whether the list of objects (or one of the alternate lists) is all contained by the
	 * object
	 */
	bool check_contains_list(const std::string &cond_string,
							 const std::vector<unsigned long> &obj_contained_ids) const;

	bool check_conds(const std::vector<std::string> &conds,
					 const dhtt_msgs::msg::CookingObject &obj) const;

	static std::pair<std::string, std::string> extract_keyval(const std::string &param_string);
};
} // namespace dhtt_plugins

#endif // COOKING_BEHAVIOR_HPP
