#ifndef DEFER_BEHAVIOR_HPP_
#define DEFER_BEHAVIOR_HPP_

#include "dhtt_plugins/behaviors/cooking_behavior.hpp"

namespace dhtt_plugins
{
    /**
     * \brief This is an empty behavior with low activation potential that serves as a spacer for the tree
     * 
     * This class is useful when important actions need to take place and then we need to essentially wait for the rest of the tree to catch up.
     *  The use case this was designed for is when an exists behavior marks an object we sometimes want to continue processing that object in order
     *  to achieve a more consistent ordering of the tree. In this case the defer behavior ensures that the activation potential of the more important
     *  actions will be higher than that of the defer behavior.
     */
    class DeferBehavior : public dhtt_plugins::CookingBehavior
	{
	public:

        /**
         * \brief this behavior just yields a low activation potential for the deferring to work
         * 
         * \return .1
         */
        double get_perceived_efficiency(dhtt::Node* container) { return .1; };

		/**
		 * \brief this class does no work
         * 
		 * \param container similar to work_callback this is a pointer to the node which is running this logic.
		 * 
		 * \return void
		 */
		void do_work( dhtt::Node* container ) 
        {
            auto req = std::make_shared<dhtt_msgs::srv::CookingRequest::Request>();
            req->super_action = dhtt_msgs::srv::CookingRequest::Request::OBSERVE;
            auto res = this->send_request_and_update(req);

            this->done = true;
        };

		/**
		 * \brief does not retain resources
		 * 
		 * \param container similar to work_callback this is a pointer to the node which is running this logic.
		 * 
		 * \return list of any retained resources of this behavior
		 */
		std::vector<dhtt_msgs::msg::Resource> get_retained_resources( dhtt::Node* container ) { return {}; };

		/**
		 * \brief this behavior should be run only on activation potential
		 * 
		 * \return the list of resources required to run the behavior
		 */
		std::vector<dhtt_msgs::msg::Resource> get_necessary_resources() { return {}; };

        /**
         * \brief This behavior does not take or deal with params
         * 
         * \return void
         */
        void parse_params(std::vector<std::string> params) {};
	};

}

#endif // DEFER_BEHAVIOR_HPP_