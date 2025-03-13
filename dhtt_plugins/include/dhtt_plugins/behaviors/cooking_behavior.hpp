#ifndef COOKING_BEHAVIOR_HPP
#define COOKING_BEHAVIOR_HPP

#include "dhtt_msgs/msg/cooking_observation.hpp"
#include "dhtt_msgs/srv/cooking_request.hpp"
#include "dhtt_plugins/behaviors/action_type.hpp"
#include "rclcpp/rclcpp.hpp"

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
	 *i.e. 'Chopped, Toasted'. Also supports 'Free' for objects containing nothing, or 'Contains'
	 *for objects containing something.
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
};
} // namespace dhtt_plugins

#endif // COOKING_BEHAVIOR_HPP
