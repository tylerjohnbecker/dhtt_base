#ifndef COOKING_MOVE_BEHAVIOR_HPP
#define COOKING_MOVE_BEHAVIOR_HPP

#include "dhtt_msgs/msg/cooking_observation.hpp"
#include "dhtt_msgs/srv/cooking_request.hpp"
#include "dhtt_plugins/behaviors/action_type.hpp"
#include "rclcpp/rclcpp.hpp"

namespace dhtt_plugins
{
class CookingMoveBehavior : public ActionType
{
  public:
	CookingMoveBehavior();

	/**
	 * \brief parses activation_potential and destination parameters for the move behavior
	 *
	 * The meaning of each parameter is as follows:
	 *		- activation_potential: a given activation potential for the behavior (for testing
	 *specific behavior orders)
	 * 		- coord: a coordinate in format 'x, y'. Leave empty "" if not used
	 * 		- object: a cooking object type name to automatically find the closest of that
	 * type. Leave empty "" if not used.
	 *
	 * \param params a vector of string params parsed from the yaml description
	 *
	 * \return void
	 */
	void parse_params(std::vector<std::string> params) override;

	double get_perceived_efficiency() override;
	void do_work(dhtt::Node *container) override;
	std::vector<dhtt_msgs::msg::Resource> get_retained_resources(dhtt::Node *container) override;
	std::vector<dhtt_msgs::msg::Resource> get_released_resources(dhtt::Node *container) override;

  protected:
	double activation_potential;
	std::string destination_type;
	std::string destination;
	geometry_msgs::msg::Point destination_point;

  private:
	void observation_callback(std::shared_ptr<dhtt_msgs::msg::CookingObservation> msg);
	void initialize_();
	void set_destination_to_closest_object();

	rclcpp::Subscription<dhtt_msgs::msg::CookingObservation>::SharedPtr
		cooking_observation_subscriber;

	rclcpp::Client<dhtt_msgs::srv::CookingRequest>::SharedPtr cooking_request_client;

	std::shared_ptr<dhtt_msgs::msg::CookingObservation> last_obs;

	const std::string PARAM_ACTIVATION_POTENTIAL = "activation_potential";
	const std::string PARAM_COORDINATE = "coord";
	const std::string PARAM_OBJECT_TYPE = "object";
};
} // namespace dhtt_plugins

#endif // COOKING_MOVE_BEHAVIOR_HPP
