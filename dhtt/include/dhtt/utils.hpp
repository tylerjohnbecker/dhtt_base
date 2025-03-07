
#ifndef DHTT_UTILS_HPP_
#define DHTT_UTILS_HPP_

#include <unordered_map>
#include <vector>
#include <set>
#include <string>

#include "rclcpp/rclcpp.hpp"

#include "dhtt_msgs/action/condition.hpp"

#include "dhtt_msgs/msg/pair.hpp"

namespace dhtt_utils
{
	typedef struct PredCon
	{
		int logical_operator;

		std::vector<dhtt_msgs::msg::Pair> predicates;
		std::vector<std::shared_ptr<struct PredCon>> conjunctions;
	} PredicateConjunction;

	enum logical_operators 
	{
		LOGICAL_AND = 1,
		LOGICAL_OR = 2,
		LOGICAL_OTHER = 3
	};

	/**
	 * \brief helper for find_partial_predicate
	 * 
	 * This is the same as predicate_equals except that the negation member of the Pair message is not checked
	 * 
	 * \return True if predicates share key and value, false otherwise
	 */
	bool predicate_partial_equals(dhtt_msgs::msg::Pair lhs, dhtt_msgs::msg::Pair rhs);

	/**
	 * \brief equality function for predicates
	 * 
	 * \return True if predicates are equal, False otherwise
	 */
	bool predicate_equals(const dhtt_msgs::msg::Pair& lhs, const dhtt_msgs::msg::Pair& rhs);

	/**
	 * \brief comparison function for standard library sets
	 */
	bool compare_predicates(const dhtt_msgs::msg::Pair& lhs, const dhtt_msgs::msg::Pair& rhs);

	/**
	 * \brief Creates a deep copy of a predicate
	 * 
	 * \param msg predicate to copy
	 * 
	 * \return deep copy of predicate
	 */
	dhtt_msgs::msg::Pair predicate_copy(const dhtt_msgs::msg::Pair& msg);

	/**
	 * \brief Creates a deep copy of a conjunction of predicates
	 * 
	 * Utilizes the recursive definition of a predicate conjunction, and maintains all properties of the initial msg (including order).
	 * 
	 * \param msg predicate conjunction to copy
	 * 
	 * \return deep copy of predicate conjunction
	 */
	std::shared_ptr<PredicateConjunction> conjunction_copy(const PredicateConjunction& msg);

	std::string to_string(std::shared_ptr<PredicateConjunction> pred);

	std::string to_string(const PredicateConjunction& pred);

	PredicateConjunction convert_to_struct(std::string to_convert);

	/**
	 * \brief removes any duplicate predicates from the given predicate conjunction
	 * 
	 * Duplicate predicates with the same logical operator represent only themselves ( a and a = a ) therefore removing them is free and should save
	 * 	some work when processing the predicates as it will keep the lists shorter. Hopefully it is less work to maintain this property then it is
	 * 	to just leave duplicates in. Either way this is cleaner for the user when inspecting predicates.
	 * 
	 * \param msg where the duplicates will be removed
	 * 
	 * \return void
	 */
	void remove_predicate_duplicates(PredicateConjunction& msg);

	/**
	 * \brief removes duplicates of the same information changing in the temporall reverse order
	 * 
	 * Useful for instance in a THEN node where a single predicate's status may be changing. This ensures that only the last state is remembered as a
	 * 	postcondition of that behavior
	 * 
	 * \param msg where the duplicates will be removed
	 * 
	 * \return void
	 */
	void remove_predicate_partial_duplicates(PredicateConjunction& msg);

	/**
	 * \brief check if the execution of given postconditions would violate the given preconditions
	 * 
	 * This check essentially set's all values of the given postconditions to true and then assumes all others are true as well. So any negations not
	 * 	in the postconditions list will evaluate to false etc. and whatever is given in the postconditions list is considered true (negations will also
	 * 	evaluate to true). Therefore, we will just evaluate the whole boolean statement recursively to generate and return the negation of truth value.
	 * 
	 * \param postconditions Predicate Conjunction of the postcondition list
	 * \param preconditions Predicate Conjunction of the preconditions list
	 * 
	 * \return True if the postconditions would violate the preconditions (on their own), false otherwise. 
	 */
	bool violates_predicates(const PredicateConjunction& postconditions, const PredicateConjunction& preconditions);

	/**
	 * \brief checks given predicate list for internal contradictions
	 * 
	 * An internal contradiction in this context involves for instance having the following predicate relationship ( p /\ !p ) or ( p \/ !p ). If a 
	 * 	predicate is meant to realize both possible values we consider that a contradiction for now. However, logically this will not cover every 
	 * 	possible contradiction that could occur in predicate relationships as testing if a predicate conjunction is logical is NP-Hard (SAT).
	 * 
	 * \param to_check Predicate Conjunction to check for contradictions
	 * 
	 * \return True if a contradiction is found, false otherwise.
	 */
	bool contains_contradiction (const PredicateConjunction& to_check);

	/**
	 * \brief flattens given predicate conjunction as much as possible
	 * 
	 * Predicates can only be flattened if they share the same logical operator otherwise distributive properties have to be used and the list gets 
	 * 	more confusing. So those are the only ones that are flattened. We are not considering cases where there are multiple levels of predicate 
	 * 	conjunctions with the same logical operator. In those cases, run this function multiple times.
	 * 
	 * \param msg to flatten
	 * 
	 * \return deep copy of the message but flattened as much as possible
	 */
	PredicateConjunction flatten_predicates(const PredicateConjunction& msg);

	/**
	 * \brief combines two predicate conjunctions
	 * 
	 * This function does not check for duplicate predicates after appending to the to list. If that is the desired behavior then explicitly run
	 * 	remove_predicate_duplicates after this function.
	 * 
	 * \param to will have the other parameter appended to it
	 * \param from will be appended to to
	 * 
	 * \return void
	 */
	void append_predicate_conjunction(PredicateConjunction& to, const PredicateConjunction from);

	/**
	 * \brief combines two given lists of predicates
	 * 
	 * \param l_pred first Predicate Conjunctions
	 * \param r_pred second Predicate Conjunctions
	 * 
	 * \return flattened list of predicates combined into a Predicate Conjunctions
	 */
	PredicateConjunction combine_predicates(const PredicateConjunction& l_pred, const PredicateConjunction& r_pred);

	/**
	 * \brief recursively negate the predicates of the given predicate conjunction
	 * 
	 * This negation also follows de'morgans law so it will flip the and to an or and vice versa at each statement.
	 * 
	 * \param to_negate Predicate Conjunction which is to be negated
	 * 
	 * \return inverse copy of to_negate
	 */
	std::shared_ptr<dhtt_utils::PredicateConjunction> negate_predicates(const PredicateConjunction& to_negate);

	/**
	 * \brief finds a given key value pair in the given vector of predicates
	 * 
	 * \return iterator pointing to found predicate, or to_search.end() if not found
	 */
	std::vector<dhtt_msgs::msg::Pair>::iterator find_partial_predicate(std::vector<dhtt_msgs::msg::Pair> to_search, dhtt_msgs::msg::Pair to_find);

	/**
	 * \brief finds a given predicate in a given vector of predicates
	 * 
	 * \return iterator pointing to found predicate, or to_search.end() if not found
	 */
	std::vector<dhtt_msgs::msg::Pair>::iterator find_predicate(std::vector<dhtt_msgs::msg::Pair>& to_search, dhtt_msgs::msg::Pair to_find);

	/**
	 * \brief Removes partial temporal dependencies between the given list of postconditions and the given preconditions
	 * 
	 * Searches the list of preconditions for the given postconditions. If any are found then they are not included in the returned set of preconditions.
	 * 	Currently only works on preconditions that are and'ed together because if they are or'ed instead then there is no partial dependency. For that 
	 * 	same reason this check is not recursive.
	 * 	
	 * \param post vector of predicate conjunctions representing the sum total of the postconditions to check for
	 * \param pre predicate conjunction of preconditions to check against
	 * 
	 * \return new predicate conjunction of only the preconditions which were not found in the postconditions
	 */
	PredicateConjunction remove_partial_dependencies(const std::vector<PredicateConjunction> post, const PredicateConjunction& pre);
}

#endif // DHTT_UTILS_HPP_