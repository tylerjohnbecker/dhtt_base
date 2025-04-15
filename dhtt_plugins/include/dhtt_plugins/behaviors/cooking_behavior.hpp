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
	/**
	 * \brief parses activation_potential and destination parameters for the move behavior
	 *
	 * The meaning of each parameter is as follows:
	 *		- activation_potential: a given activation potential for the behavior (for testing
	 *specific behavior orders)
	 * 		- coord: a coordinate in format 'x, y'. Leave empty "" if not used
	 * 		- object: a cooking object type name to automatically find the closest of that
	 * 		- conditions: an optional comma-separated string of conditions the object needs to have.
	 *i.e. 'conditions: Chopped, Toasted'. Also supports 'Free' for objects containing nothing, or
	 *'Contains' for objects containing something. Further, specifying 'Contains+"listofobjects"'
	 *allows a list of objects to be contained,
	 *i.e. params: [ 'object: Plate', 'conditions: Contains+"Lettuce, Tomato"' ]
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
	virtual double get_perceived_efficiency() override;

  protected:
	std::string destination_type;  // key: coordinate or closest object
	std::string destination_value; // value: of destination_type, i.e. either '1,1' or 'Toaster'
	std::string destination_conditions; // conditions for closest object
	geometry_msgs::msg::Point destination_point;
	bool destination_is_good = false;

	static double point_distance(const geometry_msgs::msg::Point &point1,
								 const geometry_msgs::msg::Point &point2);
	double agent_point_distance(const geometry_msgs::msg::Point &point) const;
	void observation_callback(std::shared_ptr<dhtt_msgs::msg::CookingObservation> msg);

	/**
	 * Perform some CookingBehavior-specific initialization (Client and Subscriber creation). Due to
	 * API quirks, this is called in parse_params()
	 */
	void initialize_();

	/**
	 * Using this->destination_value, this->destination_conds, and this->last_obs, set
	 * this->destination_point
	 */
	void set_destination_to_closest_object();

	static std::string which_arm(dhtt::Node *container);

	/**
	 * @return whether activation potential and dest_good are correct for us to work
	 */
    bool can_work() const;

	rclcpp::Subscription<dhtt_msgs::msg::CookingObservation>::SharedPtr
		cooking_observation_subscriber;

	rclcpp::Client<dhtt_msgs::srv::CookingRequest>::SharedPtr cooking_request_client;

	std::shared_ptr<dhtt_msgs::msg::CookingObservation> last_obs;

	const std::string PARAM_ACTIVATION_POTENTIAL = "activation_potential";
	const std::string PARAM_COORDINATE = "coord";
	const std::string PARAM_OBJECT_TYPE = "object";
	const std::string PARAM_OBJECT_CONDITIONS = "conditions";
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
};
} // namespace dhtt_plugins

#endif // COOKING_BEHAVIOR_HPP
