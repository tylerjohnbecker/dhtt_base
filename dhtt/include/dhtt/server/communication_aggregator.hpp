#ifndef COMMUNICATION_AGGREGATOR_HPP_
#define COMMUNICATION_AGGREGATOR_HPP_

#include <memory>
#include <any>
#include <thread>
#include <future>
// #include <concepts>

#include "rclcpp/rclcpp.hpp"

// this file also includes the implementation due to templating shenanigans being frustrating to deal with

namespace dhtt
{

	// create a requirement for service like types from templates
	// template <typename service_type>
	// concept service_like = requires
	// {
	// 	typename service_type::Request;
	// 	typename service_type::Result;
	// };

	/**
	 * \brief Communication Aggregator for subscription and service calls coming from behaviors in the tree
	 * 
	 * As a result of ROS utilizing a DDS on the backend there is a small upper limit on the number of action servers, services, etc.
	 * 	which can be registered in one process (~130). Therefore, this class enables us to build large scale trees that have extensive
	 * 	connections through ROS without creating more than one subscriber/service client per topic. 
	 * 
	 * This class essentially creates a single subscriber/client per topic and then utilizes an internal callback to call all registered 
	 * 	functions for each topic. The registered functions are called in separate threads and there should be no latency below a 
	 * 	reasonable amount of callbacks, however as each callback will be run in a separate thread this class may have memory concerns 
	 * 	which will be addressed as they are found.
	 */
	class CommunicationAggregator : public rclcpp::Node
	{
	public:
		CommunicationAggregator(int queue_size = 10) : Node("CommunicationAggregator"), sub_qos(queue_size) {};

		/**
		 * \brief registers a subscriber callback 
		 * 
		 * If the subscriber does not already exist this function instantiates it through ROS. Otherwise, this function just
		 * 	registers a new callback which will be called by the generic subscriber function.
		 * 
		 * \tparam msg_type The message type of the subscriber topic
		 * 
		 * \param topic_name the name of the topic to subscribe to
		 * \param subscriber_name to save the callback to (each unique name can have one callback)
		 * \param to_register a function to call whenever the topic receives a message
		 * 
		 * \returns true if the callback is registered correctly. Does not currently have a fail state.
		 */
		template <typename msg_type>
		bool register_subscription(std::string topic_name, std::string subscriber_name, std::function<void(std::shared_ptr<msg_type>)> to_register)
		{
			// we have not yet registered a subscriber to this topic
			if ( this->function_tables.find(topic_name) == this->function_tables.end() )
			{
				std::map<std::string, std::function<void(std::shared_ptr<msg_type>)>> n_function_table{subscriber_name, to_register};
				this->function_tables[topic_name] = n_function_table;

				std::function<void(std::shared_ptr<msg_type>)> n_cb = std::bind(&CommunicationAggregator::generic_subscriber_callback<msg_type>, this, topic_name, std::placeholders::_1);
				std::shared_ptr<rclcpp::Subscription<msg_type>> n_subscription = this->create_subscription<msg_type>(topic_name, this->sub_qos, n_cb);

				this->subscription_ptrs[topic_name] = n_subscription;

				return true;
			}

			// topic already has a function table so just add to it (or overwrite in the case of a preexisting function with the same subscriber name)
			auto tmp = std::any_cast< std::map<std::string, std::function<void(std::shared_ptr<msg_type>)>> >(this->function_tables[topic_name]);
			tmp[subscriber_name] = to_register;

			this->function_tables[topic_name] = tmp;

			return true;
		}

		/**
		 * \brief unregisters callback on given topic name
		 * 
		 * This method either unregisters a single callback under topic_name and subscriber name, or unregisters all callbacks on a given topic if the 
		 * 	subscriber name is left blank.
		 * 
		 * \param topic_name string name of the subscriber's topic
		 * \param subscriber_name string given to the callback to delete (if blank completely clears the list of callbacks)
		 * 
		 * \return true if callback was removed, false if not
		 */
		bool unregister_subscription(std::string topic_name, std::string subscriber_name = "")
		{
			if ( this->function_tables.find(topic_name) == this->function_tables.end() )
				return false;

			if ( not strcmp(subscriber_name.c_str(), "") )
			{
				// deleting everything is easy
				this->function_tables.erase(topic_name);
				this->subscription_ptrs.erase(topic_name);

				return true;
			}

			// I think that this casting is ok since we don't care about using the function here (otherwise this function needs a template)
			auto tmp = std::any_cast< std::map<std::string, std::any> >(this->function_tables[topic_name]);

			if ( tmp.find(subscriber_name) == tmp.end() )
				return false;

			// erase in tmp variable and then save back to internal data structure
			tmp.erase(subscriber_name);

			// if its empty tho don't save it and delete the subscriber
			if ( tmp.empty() )
			{
				// deleting everything is easy
				this->function_tables.erase(topic_name);
				this->subscription_ptrs.erase(topic_name);

				return true;
			}

			this->function_tables[topic_name] = tmp;

			return true;
		}

		/**
		 * \brief registers service client unless it's already registered
		 * 
		 * \tparam service_type name of the message type used by the server (requires service_like)
		 * 
		 * \param service_topic_name name of the service to register
		 * 
		 * \return true if service client is succesful created, false if server cannot be reached
		 */
		template <typename service_type>
		bool register_service_client(std::string service_topic_name)
		{
			// if it doesn't already exist
			if ( this->service_client_ptrs.find(service_topic_name) == this->service_client_ptrs.end() )
			{
				auto n_client = this->create_client<service_type>(service_topic_name);

				{// new scope to prevent the using statement from leaking out
					using namespace std::chrono_literals;

					if ( not n_client->wait_for_service(10s) )
						return false;
				}

				this->service_client_ptrs[service_topic_name] = n_client;
			}

			// else we don't need to do anything
			return true;
		}

		/**
		 * \brief ungregisters service client with given topic name.
		 * 
		 * \param std::string service_topic_name name of the service client to unregister
		 * 
		 * \return true if service client removed. False if the given name was not already registered
		 */
		bool ungregister_service_client(std::string service_topic_name)
		{
			if ( this->service_client_ptrs.find(service_topic_name) == this->service_client_ptrs.end() )
				return false;

			this->service_client_ptrs.erase(service_topic_name);
			return true;
		}

		/**
		 * \brief a wrapper on the async_send_request method in rclcpp::Client
		 * 
		 * This function will fail if the client does not exist (has not been registered) and in that case it will return a blank future.
		 * 
		 * \tparam service_type this needs to have an instantiation of ::Result and ::Request (i.e. it should be a ROS service type)
		 * 
		 * \param service_topic_name name of the service topic (to retrieve the client internally)
		 * \param to_send the correctly typed request to send through the client
		 * 
		 * \return std::future from which the result from the server can be retreived
		 */
		template <typename service_type>
		std::future<typename service_type::Result> async_request_from_client(std::string service_topic_name, std::shared_ptr<typename service_type::Request> to_send)
		{
			// if we can't find the client then we have to return a blank future
			if ( this->service_client_ptrs.find(service_topic_name) == this->service_client_ptrs.end() )
				return std::future<typename service_type::Result>();

			// otherwise we do the cast and return the future like normal
			auto tmp_client = std::any_cast< std::shared_ptr < rclcpp::Client < service_type > > >(this->service_client_ptrs[service_topic_name]);

			return tmp_client->async_send_request(to_send);
		}

	private:

		/**
		 * \brief all purpose subscriber callback
		 * 
		 * This callback will be registered for all subscribers. Each time a msg is received it will be passed to each registered function
		 * 	in the function table in its own thread. At the end of this callback the threads will then be joined which for now means that 
		 * 	incoming information will be bottlenecked by the work of all callbacks completing. Therefore, for now all subscriber callbacks
		 * 	registered through this class should have low computational overhead in order to prevent information loss.
		 * 
		 * NOTE: if the information loss is substantial anyway this code will be refactored accordingly
		 * 
		 * \tparam msg_type type of the ROS message the subscriber accepts
		 * 
		 * \param topic_name bounded parameter to the topic name of the subscriber
		 * \param msg incoming information
		 * 
		 * \return void
		 */
		template <typename msg_type>
		void generic_subscriber_callback(std::string topic_name, std::shared_ptr<msg_type> msg)
		{
			// in case we unregister the subscriber during this callback being called (Not perfect but this should be safe enough)
			if ( this->function_tables.find(topic_name) == this->function_tables.end() )
				return;

			// now cast the function table to the correct type
			auto table = std::any_cast< std::map < std::string , std::function < void(std::shared_ptr< msg_type > ) > > > (this->function_tables[topic_name]); 

			// initialize and start a thread for every registered callback of this subscriber
			std::list<std::shared_ptr<std::thread>> thread_list;
			for ( const auto& pair : table )
			{
				auto thread_ptr = std::make_shared<std::thread>(pair.second, msg);

				thread_list.push_back(thread_ptr);
			}

			// join all threads before returning
			for ( auto iter : thread_list )
				iter->join();
		}

		// private members
		std::map<std::string, std::any> function_tables; // std::any = std::map < std::string,  std::function< void( std::shared_ptr< msg_type > ) > >

		std::map<std::string, std::any> service_client_ptrs; // std::any = std::shared_ptr < rclcpp::Client < service_type > >
		std::map<std::string, std::any> subscription_ptrs; // std::any = std::shared_ptr < rclcpp::Subscription < msg_type > > 

		rclcpp::QoS sub_qos;
	};

}

#endif // COMMUNICATION_AGGREGATOR_HPP_