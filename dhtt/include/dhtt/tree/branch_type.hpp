#ifndef BRANCH_TYPE_HPP_
#define BRANCH_TYPE_HPP_

// INCLUDES
#include <iostream>
#include <vector>
#include <string>
#include <functional>
#include <thread>
#include <future>

#include "dhtt_msgs/action/activation.hpp"
#include "dhtt_msgs/action/condition.hpp"

namespace dhtt
{
	//forward declarations
	class Node;

	// using statements for clarity
	using activation_goal = dhtt_msgs::action::Activation::Goal;
	using activation_result = dhtt_msgs::action::Activation::Result;

	using condition_goal = dhtt_msgs::action::Condition::Goal;
	using condition_result = dhtt_msgs::action::Condition::Result;

	/**
	 * \brief base class for the branch socket
	 * 
	 * Realistically this class is only relevant for the plug because it is intended to call the public methods of the node class and otherwise isn't necessary to be anywhere else.
	 * 	Added two base methods for posterity I guess in case we want heterogeneous plug/socket pairs. This facilitates communication for the activation pipeline and the maitnenance
	 * 	pipeline.
	 * 
	 * Note: I think I could make this more generalized to facilitate one of each communication pipeline, however, I'm not sure that this is the best way forward for the class
	 * 	representation. 
	 */
	class BranchSocketType
	{
	public:
		/**
		 * \brief initialize function for the socket type
		 * 
		 * This method should construct everything the socket needs to function which might involve essentially nothing, but for instance also might make an action server etc.
		 * 
		 * \param constructor_ptr shared_ptr of the constructing node that is meant to allow this class to call the public nodes.
		 * 
		 * \return void
		 */
		virtual void initialize(Node* constructor_ptr) = 0;

		/**
		 * \brief empty for this class
		 * 
		 * This should be called in the destructor of the node class and can be overriden in the case that child classes have more resources to collect
		 */
		virtual void destruct() {};

		/**
		 * \brief activation recieved callback
		 * 
		 * Overall, it shouldn't matter that this exists or not as the paired plug for this class will always know what to call in the child, however, if we have heterogeneous plug
		 * 	socket pairs this might be a useful interface design.
		 * 
		 * \param goal activation goal msg to pass to the node class
		 * 
		 * \return activation result msg
		 */
		virtual activation_result activation_received_callback(activation_goal goal) = 0;

		/**
		 * \brief maintenance request received callback
		 * 
		 * Again all work for maitenance will be resolved through the node class. The comments for activation received apply here as well. 
		 * 
		 * \param goal activation goal msg to pass to the node class
		 * 
		 * \return activation result msg
		 */
		virtual condition_result maintain_received_callback(condition_goal goal) = 0;

		/**
		 * \brief method to test whether the plug socket connection functions as intended
		 * 
		 * This is meant to be called after construction to help with testing. This really should only be accessed by the corresponding plug as this socket has no knowledge or way to contact
		 * 	the paired plug.
		 * 
		 * \return true if the connection is functional, false otherwise.
		 */
		virtual bool test_connection() { return true; };

	protected:
		Node* container;

	};

	/**
	 * \brief Base class for the plugs which provides the necessary multithreading capabilities
	 * 
	 * This class facilitates the communication between nodes by giving a general interface of methods they can call. For now, all calls are asynchronous and the synchronous calls are obfuscated
	 * 	from the user. 
	 */
	class BranchPlugType
	{
	public:
		/**
		 * \brief general initializer for the plug type
		 * 
		 * This class is meant to be created for each child and facilitates all communication from parent to child by connecting to its corresponding socket type. This class should contain
		 * 	all necessary systems for communication with the socket.
		 * 
		 * \param constructor_ptr will be saved and will enable this class to call public functions of the owner node
		 * \param socket_ptr for ptr based communication this will be the actual address of the socket, but for network it will be nullptr
		 * \param child_name actual node name for the child connection, useful for network based communication
		 * 
		 * \return void
		 */
		virtual void initialize(Node* constructor_ptr, std::shared_ptr<BranchSocketType> socket_ptr, std::string child_name) = 0;

		/**
		 * \brief collects resources and joins threads
		 * 
		 * This should be called in the destructor of the node class and can be overriden in the case that child classes have more resources to collect
		 */
		virtual void destruct()
		{
			if ( this->activation_thread_ptr != nullptr and this->activation_thread_ptr->joinable() )
			{
				this->activation_thread_ptr->join();
				this->activation_thread_ptr = nullptr;
			}

			if ( this->condition_thread_ptr != nullptr and this->condition_thread_ptr->joinable() )
			{
				this->condition_thread_ptr->join(); 
				this->condition_thread_ptr = nullptr;
			}

			this->container = nullptr;
		};

		/**
		 * \brief activate the child node attached to this plug's socket
		 * 
		 * The general method for getting responses should always be async_activate -> block_for_response -> get response. This will ensure that the state is managed correctly. If performed incorrectly
		 * 	some of these methods will fail. This essentially just starts the send_activate function below in another thread and keeps track of it through a thread_ptr and a future.
		 * 
		 * \param to_send activation goal to send to child
		 * 
		 * \return bool true if activation thread was created successfully false otherwise
		 */
		bool async_send_activate(activation_goal to_send)
		{
			if ( this->activation_thread_ptr != nullptr or this->activate_state > 0 )
				return false;

			this->activation_task_ptr = std::make_shared<std::packaged_task<activation_result(activation_goal)>>( std::bind( &BranchPlugType::send_activate, this, to_send ) );

			this->activation_result_future = this->activation_task_ptr->get_future();

			this->activation_thread_ptr = std::make_shared<std::thread>( std::move(*this->activation_task_ptr), to_send );

			this->activate_state = 1;

			return true;
		};
		
		/**
		 * \brief block for the activation response
		 * 
		 * \return void
		 */
		void block_for_activation_response()
		{
			if ( not this->activation_result_future.valid() or this->activate_state != 1 )
				return;

			this->activation_result_future.wait();
		
			this->activation_thread_ptr->join();

			this->activate_state = 2;
		};

		/**
		 * \brief returns the activation result received from child
		 * 
		 * If this was not called after blocking then a blank result is returned instead
		 * 
		 * \return activation result received from child
		 */
		activation_result get_activation_result()
		{
			if ( this->activate_state != 2 )
				return activation_result();

			auto to_ret = this->activation_result_future.get();
			
			// reset to default future so that it becomes invalid and reset thread to nullptr
			this->activation_thread_ptr = nullptr; 

			this->activate_state = 0;

			return to_ret;
		};

		/**
		 * \brief essentially the same as the activate function analogue
		 * 
		 * \param to_send maintenance goal msg to send to child
		 * 
		 * \return void
		 */
		bool async_send_maintain(condition_goal to_send)
		{
			if ( this->condition_thread_ptr != nullptr or this->condition_state != 0 )
				return false;

			this->condition_task_ptr = std::make_shared<std::packaged_task<condition_result(condition_goal)>>( std::bind( &BranchPlugType::send_maintain, this, to_send ) );

			this->condition_result_future = this->condition_task_ptr->get_future();

			this->condition_thread_ptr = std::make_shared<std::thread>( std::move(*this->condition_task_ptr), to_send );

			this->condition_state = 1;

			return true;

		};
		
		/**
		 * \brief essentially the same as the activation block method
		 * 
		 * \return void
		 */
		void block_for_maintain_response()
		{
			if ( this->condition_state != 1 )
				return;

			this->condition_result_future.wait();
		
			this->condition_thread_ptr->join();

			this->condition_state = 2;
		};

		/**
		 * \brief returns maitnenance result from child. Shares the same restrictions as the activation analogue method
		 * 
		 * \return condition result received from child
		 */
		condition_result get_maintain_result()
		{
			if ( not this->condition_result_future.valid() or this->condition_state != 2 )
				return condition_result();

			auto to_ret = this->condition_result_future.get();
			
			// reset to default future so that it becomes invalid and reset thread to nullptr
			this->condition_thread_ptr = nullptr; 

			this->condition_state = 0;

			return to_ret;
		}

		/**
		 * \brief returns whether this plug can connect to its socket
		 * 
		 * This method will be run from the parent node, and should be overloaded for each different connection type
		 * 
		 * \return true if it can connect, false otherwise
		 */
		virtual bool connection_is_good() { return true; };

	protected:

		/**
		 * \brief Method to activate child
		 * 
		 * This is the method where overloaded activation code should exist to hit the right functionality from the socket. This method will be called from
		 * 	a separate thread so that should be taken into account in design.
		 * 
		 * \param to_send activation goal for the child
		 * 
		 * \return activation result from the child
		 */
		virtual activation_result send_activate(activation_goal to_send) = 0;

		/**
		 * \brief Method to perform condition maintenance on child
		 * 
		 * This is the method where overloaded maintenance code should exist to hit the right functionality from the socket. This method will be called from
		 * 	a separate thread so that should be taken into account in design.
		 * 
		 * \param to_send condition goal for the child
		 * 
		 * \return condition result from the child
		 */
		virtual condition_result send_maintain(condition_goal to_send) = 0;

		Node* container;

	private:

		// These parameters are private because they only relate to the multithreading functionality of the plugtype
		std::shared_ptr<std::packaged_task<activation_result(activation_goal)>> activation_task_ptr;
		std::future<activation_result> activation_result_future;
		std::shared_ptr<std::thread> activation_thread_ptr;

		std::shared_ptr<std::packaged_task<condition_result(condition_goal)>> condition_task_ptr;
		std::future<condition_result> condition_result_future;
		std::shared_ptr<std::thread> condition_thread_ptr;

		int activate_state = 0;
		int condition_state = 0;
	};
}


#endif // BRANCH_TYPE_HPP_