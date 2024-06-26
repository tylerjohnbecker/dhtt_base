#ifndef NODE_TYPE_HPP
#define NODE_TYPE_HPP

// INCLUDES
#include <vector>
#include <string>

// #include "dhtt/tree/node.hpp"

namespace dhtt
{

	/**
	 * \brief enum for the different types of goals 
	 * 
	 * Persistent goals are ones that once completed stay completed (placing an object). Transient goals can be uncompleted after they run (moving to a position).
	 * Not currently in use.
	 */
	enum goal_t
	{
		PERSISTENT,
		TRANSIENT
	};

	class Node;

	/**
	 * \brief Base class describing the logic of the nodes
	 * 
	 * This is an interface for how to write the logic of a node. This encompasses both task nodes and behavior nodes. Each node needs describe how to initialize, how to perform the auction,
	 * 	what work to perform, how to parse params, and how to calculate activation_potential.
	 */
	class NodeType
	{
	public:

		/**
		 * \brief abstract initialize function for all NodeType classes
		 * 
		 * NodeTypes can only initialize with some params given in a vector. The params should be described in the description file as shown in the sample_tasks in the dhtt package. the params
		 * 	should also in general be parsed in the parse_params function which is also used when parameters are modified.
		 * 
		 * \param params the params to initialize the NodeType with (taken from description file usually)
		 * 
		 * \return void
		 */
		virtual void initialize(std::vector<std::string> params) = 0;

		/**
		 * \brief functionality for completing an auction
		 * 
		 * This describes what the node should consider when it is first activated (i.e. when it starts an auction). Behavior nodes should request any resources that they need to begin working,
		 * 	and task nodes should spread activation to their children and propogate their choice up afterwards
		 * 
		 * \param container pointer to the node class which owns the node type in order to give use of the public methods.
		 * 
		 * \return returns the result part of the Activation action in order to function with the Node class.
		 */
		virtual std::shared_ptr<dhtt_msgs::action::Activation::Result> auction_callback( Node* container ) = 0;

		/**
		 * \brief functionality for performing work
		 * 
		 * This describes what a node should do when it's request for resources is approved and it can begin working. Behaviors should perform some action and then return the success of that action
		 * 	and tasks should propogate the message down and wait for the response (or perform some other action first in special cases). For an example of how to pass resources the dHTT plugins 
		 * 	actions should suffice.
		 *  
		 * \param container pointer to the node class which owns the node type in order to give use of the public methods.
		 * 
		 * \return returns the result part of the Activation action in order to function with the Node class. For behaviors this should contain the resources that need to be released.
		 */
		virtual std::shared_ptr<dhtt_msgs::action::Activation::Result> work_callback( Node* container ) = 0;

		/**
		 * \brief method to parse the input parameters for the NodeType
		 * 
		 * generally should take the parameters from the description file and parse them in some way. This could include just setting flags, getting topic names for information retrieval, etc.
		 * 
		 * \param params list of params taken from the description file (usually in key, value pairs by convention)
		 * 
		 * \return void
		 */
		virtual void parse_params(std::vector<std::string> params) = 0;

		/**
		 * \brief calculates the activation potential of the node locally
		 * 
		 * This looks at the calculation that this node locally uses, meaning some function of the childrens' activation potential for task nodes and some function (like distance) for behavior nodes.
		 * 
		 * \return activation potential value (0 -> 1 by convention) 
		 */
		virtual double get_perceived_efficiency() = 0;
		
		/**
		 * \brief check for if the node is finished running
		 * 
		 * just uses the logic of the node to find out if it is done running already. For behaviors this returns true once the work is complete, and for tasks this is complete when the logic dictates
		 * 	(AND is done when all children are done, etc.). 
		 * 
		 * \return true if the node is done. false otherwise.
		 */
		virtual bool is_done() {return true;};

		/**
		 * \brief whether or not this node can have children
		 *  
		 * Generally, behavior nodes cannot have children so in that case they should not be allowed to, however, task nodes can so all nodes can choose to override this function to set different
		 * 	constraints on a nodes ability to have children (e.g. throttling the number of children to a max number).
		 * 
		 * \return true if a child can be added. false otherwise.
		 */
		virtual bool can_add_child() {return this->children_allowed;};

		goal_t goal_type = PERSISTENT;

		std::vector<dhtt_msgs::msg::Resource> necessary_resources;

		std::vector<std::string> params;
		std::vector<std::string> preconditions;

	protected:

		bool children_allowed;
	};

}

#endif //NODE_TYPE_HPP