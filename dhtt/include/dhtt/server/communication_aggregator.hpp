#ifndef COMMUNICATION_AGGREGATOR_HPP_
#define COMMUNICATION_AGGREGATOR_HPP_

#include <memory>
#include <any>
#include <thread>
#include <future>
#include <mutex>

#include "rclcpp/rclcpp.hpp"

#define SUBSCRIBER_CB_MAX 64

// this file also includes the implementation due to templating shenanigans being frustrating to deal with

namespace dhtt
{
	enum LOG_LEVEL
	{
		DEBUG=0,
		INFO=1,
		WARN=2,
		ERROR=3,
		FATAL=4
	};

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
		CommunicationAggregator(std::shared_ptr<rclcpp::executors::MultiThreadedExecutor> exec_ptr, int queue_size = 10) : 
								Node("ComAgg", rclcpp::NodeOptions().allow_undeclared_parameters(true).automatically_declare_parameters_from_overrides(true)),
								 sub_qos(queue_size), unique_id(0), local_ex_ptr(exec_ptr) 
		{
			this->declare_parameter("world.marked_objects_taints", std::vector<std::string>());
			this->declare_parameter("world.marked_objects_types", std::vector<std::string>());
			this->declare_parameter("world.marked_objects_ids", std::vector<long int>());
		};

		/**
		 * \brief registers a subscriber callback 
		 * 
		 * If the subscriber does not already exist this function instantiates it through ROS. Otherwise, this function just
		 * 	registers a new callback which will be called by the generic subscriber function.
		 * 
		 * \tparam msg_type The message type of the subscriber topic
		 * 
		 * \param topic_name the name of the topic to subscribe to
		 * \param subscriber_name to save the callback to (each unique name can have one callback) (leave blank to create a unique name here)
		 * \param to_register a function to call whenever the topic receives a message
		 * 
		 * \returns name of subscriber_name the function is saved under.
		 */
		template <typename msg_type>
		std::string register_subscription(std::string topic_name, std::string subscriber_name, std::function<void(std::shared_ptr<msg_type>)> to_register)
		{
			// grab the mutex first
			std::lock_guard<std::mutex> lock(this->register_mutex);

			// if it's an empty name then assign it a unique identifier
			std::string local_subscriber_name = subscriber_name;

			if ( not strcmp(local_subscriber_name.c_str(), "") )
			{
				local_subscriber_name = topic_name + "_subscriber_" + std::to_string(this->unique_id);
				this->unique_id++;
			}
			
			// we have not yet registered a subscriber to this topic so we initialize the first one
			if ( this->function_tables.find(topic_name) == this->function_tables.end() )
			{
				std::vector<std::map<std::string, std::function<void(std::shared_ptr<msg_type>)>>> n_function_table{{{local_subscriber_name, to_register}}};
				this->function_tables[topic_name] = n_function_table;// realistically we should statically size this to the max to avoid copy constructor slow downs ( for now it should be fine )

				std::function<void(std::shared_ptr<msg_type>)> n_cb = std::bind(&CommunicationAggregator::generic_subscriber_callback<msg_type>, this, topic_name, std::placeholders::_1, 0);
				std::vector<std::shared_ptr<rclcpp::Subscription<msg_type>>> n_subscription = {this->create_subscription<msg_type>(topic_name, this->sub_qos, n_cb)};

				this->subscription_ptrs[topic_name] = n_subscription;

				return local_subscriber_name;
			}

			//// we already have a subscriber in the dictionary so we check the number of callbacks assigned to it
															// std::map<std::string, std::function<void(std::shared_ptr<msg_type>)>>
			auto tmp_func_vector = std::any_cast< std::vector< std::map< std::string, std::function< void( std::shared_ptr<msg_type> ) > > > > ( this->function_tables[topic_name] );

			// find the first available index
			int n_index = 0;

			for ( ; n_index < (int) tmp_func_vector.size() ; n_index++ )
				if ( (int) tmp_func_vector[n_index].size() < SUBSCRIBER_CB_MAX )
					break;

			// found an existing subscriber with room
			if ( n_index < (int) tmp_func_vector.size() )
			{
				tmp_func_vector[n_index][local_subscriber_name] = to_register;
			}
			else // have to create a new one
			{
				std::function<void(std::shared_ptr<msg_type>)> n_cb = std::bind(&CommunicationAggregator::generic_subscriber_callback<msg_type>, this, topic_name, std::placeholders::_1, n_index);
				std::shared_ptr<rclcpp::Subscription<msg_type>> n_subscription = this->create_subscription<msg_type>(topic_name, this->sub_qos, n_cb);

				// register the new subscription
				auto tmp_sub_vector = std::any_cast< std::vector< std::shared_ptr< rclcpp::Subscription< msg_type > > > > (this->subscription_ptrs[topic_name]);
				tmp_sub_vector.push_back(n_subscription);

				this->subscription_ptrs[topic_name] = tmp_sub_vector;

				tmp_func_vector.push_back({{local_subscriber_name, to_register}});
			}

			this->function_tables[topic_name] = tmp_func_vector;

			return local_subscriber_name;
		}

		/**
		 * \brief unregisters callback on given topic name
		 * 
		 * This method either unregisters a single callback under topic_name and subscriber name, or unregisters all callbacks on a given topic if the 
		 * 	subscriber name is left blank.
		 * 
		 * \tparam msg_type The message type of the subscriber topic
		 * 
		 * \param topic_name string name of the subscriber's topic
		 * \param subscriber_name string given to the callback to delete (if blank completely clears the list of callbacks)
		 * 
		 * \return true if callback was removed, false if not
		 */
		template <typename msg_type>
		bool unregister_subscription(std::string topic_name, std::string subscriber_name = "")
		{
			// grab the mutex first
			std::lock_guard<std::mutex> lock(this->register_mutex);
			
			if ( this->function_tables.find(topic_name) == this->function_tables.end() )
				return false;

			auto tmp_func_vector = std::any_cast< std::vector < std::map < std::string,  std::function< void( std::shared_ptr< msg_type > ) > > > > (this->function_tables[topic_name]);
			auto tmp_sub_vector = std::any_cast< std::vector< std::shared_ptr< rclcpp::Subscription< msg_type > > > > (this->subscription_ptrs[topic_name]);

			if ( not strcmp(subscriber_name.c_str(), "") )
			{
				// deleting everything is easy doing the any cast first to ensure that destructors are called
				tmp_func_vector.clear();
				this->function_tables.erase(topic_name);

				tmp_sub_vector.clear();
				this->subscription_ptrs.erase(topic_name);

				return true;
			}

			// look in each of the different dictionaries for the guy
			typename std::vector<std::map<std::string, std::function< void( std::shared_ptr<msg_type> ) > > >::iterator to_delete = tmp_func_vector.begin(); 

			for ( ; to_delete != tmp_func_vector.end() ; to_delete++ )
			{
				if ( (*to_delete).find(subscriber_name) != (*to_delete).end() )
				{
					(*to_delete).erase(subscriber_name);
					break;
				}
			}

			// if we didn't find the subscriber_name just return
			if ( to_delete == tmp_func_vector.end() )
				return false;

			// if the resulting function table is empty free the memory from the subscriber
			if ( (*to_delete).empty() )
			{
				int index = (int) std::distance(tmp_func_vector.begin(), to_delete);

				tmp_func_vector.erase(to_delete);
				tmp_sub_vector.erase(tmp_sub_vector.begin() + index);
			}
			
			// finally save the changes
			this->function_tables[topic_name] = tmp_func_vector;
			this->subscription_ptrs[topic_name] = tmp_sub_vector;

			return true;

			// // I think that this casting is ok since we don't care about using the function here (otherwise this function needs a template)
			// auto tmp = std::any_cast< std::map<std::string, std::function< void( std::shared_ptr< msg_type > ) >> >(this->function_tables[topic_name]);

			// if ( tmp.find(subscriber_name) == tmp.end() )
			// 	return false;

			// // erase in tmp variable and then save back to internal data structure
			// tmp.erase(subscriber_name);

			// // if its empty tho don't save it and delete the subscriber
			// if ( tmp.empty() )
			// {
			// 	// deleting everything is easy
			// 	this->function_tables.erase(topic_name);
			// 	this->subscription_ptrs.erase(topic_name);

			// 	return true;
			// }

			// this->function_tables[topic_name] = tmp;

			return true;
		}

		/**
		 * \brief registers a publisher with the given topic name and returns a ptr to the publisher for outside use
		 * 
		 * After creating the publisher a weak_ptr will be kept locally which means that when the last ptr falls out of scope outside of this class it will destruct
		 * 	the publisher automatically and the weak_ptr here will be considered expired. This will only ever create a single publisher per topic name.
		 * 
		 * \tparam msg_type ROS message for the publisher type
		 * 
		 * \param topic_name string name of the topic to register a publisher on
		 * 
		 * \return shared_ptr to the registered publisher (or the publisher that was previously created)
		 */
		template<typename msg_type>
		std::shared_ptr<rclcpp::Publisher<msg_type>> register_publisher(std::string topic_name)
		{
			// grab the mutex first
			std::lock_guard<std::mutex> lock(this->register_mutex);

			// either we never added it, or the last shared_ptr to the old publisher has fallen out of scope
			if ( this->publisher_ptrs.find(topic_name) == this->publisher_ptrs.end() or 
					std::any_cast< std::weak_ptr < rclcpp::Publisher < msg_type > > >(this->publisher_ptrs[topic_name]).expired() )
			{
				auto n_publisher = this->create_publisher<msg_type>(topic_name, this->sub_qos);
				this->publisher_ptrs[topic_name] = std::weak_ptr<rclcpp::Publisher<msg_type>>(n_publisher);

				return n_publisher;
			}

			auto tmp = std::any_cast< std::weak_ptr<rclcpp::Publisher<msg_type>> >(this->publisher_ptrs[topic_name]);

			return tmp.lock();
		}

		/**
		 * \brief registers service client unless it's already registered and returns a shared_ptr to that publisher
		 * 
		 * \tparam service_type name of the message type used by the server (requires service_like)
		 * 
		 * \param service_topic_name name of the service to register
		 * 
		 * \return shared_ptr to the service client at the topic name. Returns nullptr if the server cannot be reached
		 */
		template <typename service_type>
		std::shared_ptr<rclcpp::Client<service_type>> register_service_client(std::string service_topic_name)
		{
			// grab the mutex first
			std::lock_guard<std::mutex> lock(this->register_mutex);

			// if it doesn't exist
			if ( this->service_client_ptrs.find(service_topic_name) == this->service_client_ptrs.end() or 
					std::any_cast< std::weak_ptr < rclcpp::Client < service_type > > >(this->service_client_ptrs[service_topic_name]).expired() )
			{
				auto n_client = this->create_client<service_type>(service_topic_name);

				{// new scope to prevent the using statement from annoying me later
					using namespace std::chrono_literals;

					if ( not n_client->wait_for_service(10s) )
						return nullptr;
				}

				this->service_client_ptrs[service_topic_name] = std::weak_ptr<rclcpp::Client<service_type>>(n_client);

				return n_client;
			}

			// else we don't need to do anything
			auto tmp = std::any_cast< std::weak_ptr<rclcpp::Client<service_type>> >(this->service_client_ptrs[service_topic_name]);

			return tmp.lock();
		}

		/**
		 * \brief Returns a SyncParametersClient ptr to the given node_name
		 * 
		 * This method only creates a new SyncParametersClient if one does not already exist for the given node_name. 
		 * 
		 * \param node_name string name of the node which we are grabbing params from
		 * 
		 * \return ptr to the param client or nullptr if it couldn't be reached
		 */
		std::shared_ptr<rclcpp::SyncParametersClient> register_param_client(std::string node_name)
		{
			if ( not this->param_client_node_ptr )
				this->param_client_node_ptr = std::make_shared<rclcpp::Node>("param_client_node");

			// if it doesn't exist already or has been deleted
			if ( this->param_client_ptrs.find(node_name) == this->param_client_ptrs.end() or
					this->param_client_ptrs[node_name].expired() )
			{
				auto n_param_client = std::make_shared<rclcpp::SyncParametersClient>(this->param_client_node_ptr, node_name);

				{// new scope to prevent the using statement from annoying me later
					using namespace std::chrono_literals;

					if ( not n_param_client->wait_for_service(1s) )
						return nullptr;
				}

				this->param_client_ptrs[node_name] = std::weak_ptr<rclcpp::SyncParametersClient>(n_param_client);

				return n_param_client;
			}

			// otherwise just return a ptr to the params client
			return this->param_client_ptrs[node_name].lock();
		}

		/**
		 * \brief thread safe function for making a param client request
		 * 
		 * This just takes the asynchronous call and waits for the future making it a synchronous wrapper.
		 * 
		 * \param request_from the param client ptr to make a thread safe request on
		 * 
		 * \return std::shared_future to the response efrom the server
		 */
		rclcpp::Parameter get_parameter_sync(std::string param_client_name, std::string param_name)
		{
			if ( this->param_client_ptrs[param_client_name].expired() )
				return rclcpp::Parameter();
			
			std::lock_guard<std::mutex> lock(this->param_mutex);

			std::shared_ptr<rclcpp::SyncParametersClient> local_ptr = this->param_client_ptrs[param_client_name].lock();

			std::vector<rclcpp::Parameter> to_ret = local_ptr->get_parameters({ param_name });
			
			return to_ret[0];
		}

		/**
		 * \brief synchronously grabs a parameter from this nodes param server
		 * 
		 * A wrapper on the overloaded method that uses a mutex for thread safety with read/write to params.
		 * 
		 * \param param_name name of the reqested param
		 * 
		 * \return rclcpp::Parameter containing the requested param (Empty if param doesn't exist on server)
		 */
		rclcpp::Parameter get_parameter_sync(std::string param_name)
		{
			std::lock_guard<std::mutex> lock(this->local_param_mutex);
			
			rclcpp::Parameter to_ret = this->get_parameter(param_name);

			return to_ret;
		}

		/**
		 * \brief synchronously sets the value of a param on the param server
		 * 
		 * Grabs a local mutex so that there are no async problems.
		 * 
		 * \param param_name name of the param to change
		 * \param n_val new value of the parameter
		 * 
		 * \return void
		 */
		void set_parameter_sync(rclcpp::Parameter n_val)
		{
			std::lock_guard<std::mutex> lock(this->local_param_mutex);

			this->set_parameter(n_val);
		}

		/**
		 * \brief async safe wrapper for the set_parameters function
		 */
		void set_parameters_sync(std::vector<rclcpp::Parameter> n_vals)
		{
			std::lock_guard<std::mutex> lock(this->local_param_mutex);

			this->set_parameters(n_vals);
		}

		/**
		 * \brief wrapper on RCLCPP_blank_STREAM functions to allow non-nodes to utilize them
		 * 
		 * This function is intended to be called through the macros shown in node.hpp.
		 * 
		 * \param severity level to log at (INFO, WARN, ...)
		 * \param to_print string stream of desired output
		 * 
		 * \return void
		 */
		void log_stream (LOG_LEVEL severity, std::stringstream& to_print )
		{
			std::string out = to_print.str();

			switch (severity)
			{
			case LOG_LEVEL::DEBUG:
				RCLCPP_DEBUG(this->get_logger(), "%s", out.c_str());
				break;
			case LOG_LEVEL::INFO:
				RCLCPP_INFO(this->get_logger(), "%s", out.c_str());
				break;
			case LOG_LEVEL::WARN:
				RCLCPP_WARN(this->get_logger(), "%s", out.c_str());
				break;
			case LOG_LEVEL::ERROR:
				RCLCPP_ERROR(this->get_logger(), "%s", out.c_str());
				break;
			case LOG_LEVEL::FATAL:
				RCLCPP_FATAL(this->get_logger(), "%s", out.c_str());
				break;
			default:
				throw std::runtime_error("Unknown logging level chosen. Was this run through a macro?");
			}
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
		void generic_subscriber_callback(std::string topic_name, std::shared_ptr<msg_type> msg, int function_table_index)
		{
			// in case we unregister the subscriber during this callback being called (Not perfect but this should be safe enough)
			if ( this->function_tables.find(topic_name) == this->function_tables.end() )
				return;

			// now cast the function table to the correct type
			auto table = std::any_cast< std::vector < std::map < std::string , std::function < void(std::shared_ptr< msg_type > ) > > > > (this->function_tables[topic_name])[function_table_index]; 

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

		//// private members
		// subscriber members
		std::map<std::string, std::any> function_tables; // std::any = std::vector < std::map < std::string,  std::function< void( std::shared_ptr< msg_type > ) > > >
		std::map<std::string, std::any> subscription_ptrs; // std::any = std::vector < std::shared_ptr < rclcpp::Subscription < msg_type > > >

		rclcpp::QoS sub_qos;
		int unique_id;

		// publisher members
		std::map<std::string, std::any> service_client_ptrs; // std::any = std::weak_ptr < rclcpp::Client < service_type > >
		
		// service client members
		std::map<std::string, std::any> publisher_ptrs; // std::any = std::weak_ptr < rclcpp::Publisher < msg_type > >

		// param client members
		std::shared_ptr<rclcpp::Node> param_client_node_ptr = nullptr;
		std::map<std::string, std::weak_ptr<rclcpp::SyncParametersClient>> param_client_ptrs; 
				
		// general thread safety
		std::mutex register_mutex;
		std::mutex param_mutex;
		std::mutex local_param_mutex;

		// executor ptr
		std::shared_ptr<rclcpp::executors::MultiThreadedExecutor> local_ex_ptr;
	};

}

#endif // COMMUNICATION_AGGREGATOR_HPP_