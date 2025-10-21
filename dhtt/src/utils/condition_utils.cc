#include "dhtt/utils/condition_utils.hpp"

namespace dhtt_utils
{
	bool predicate_partial_equals(dhtt_msgs::msg::Pair lhs, dhtt_msgs::msg::Pair rhs) 
	{
		return not strcmp(lhs.key.c_str(), rhs.key.c_str()) and not strcmp(lhs.value.c_str(), rhs.value.c_str());
	}

	bool predicate_equals(const dhtt_msgs::msg::Pair& lhs, const dhtt_msgs::msg::Pair& rhs) 
	{
		return not strcmp(lhs.key.c_str(), rhs.key.c_str()) and not strcmp(lhs.value.c_str(), rhs.value.c_str()) and lhs.negate == rhs.negate;
	}

	bool compare_predicates(const dhtt_msgs::msg::Pair& lhs, const dhtt_msgs::msg::Pair& rhs) 
	{
		return not predicate_equals(lhs, rhs);
	}

	dhtt_msgs::msg::Pair predicate_copy(const dhtt_msgs::msg::Pair& msg)
	{
		dhtt_msgs::msg::Pair to_ret;

		to_ret.key = msg.key;
		to_ret.value = msg.value;

		to_ret.negate = msg.negate;

		return to_ret;
	}

	std::shared_ptr<PredicateConjunction> conjunction_copy(const PredicateConjunction& msg)
	{
		std::shared_ptr<PredicateConjunction> to_ret = std::make_shared<PredicateConjunction>();

		to_ret->logical_operator = msg.logical_operator;

		for ( auto iter = msg.predicates.begin() ; iter != msg.predicates.end() ; iter++ )
			to_ret->predicates.push_back(predicate_copy(*iter));

		// need to recursively copy here 
		for ( auto iter = msg.conjunctions.begin() ; iter != msg.conjunctions.end() ; iter++ )
			to_ret->conjunctions.push_back(conjunction_copy(**iter));

		return to_ret;
	}

	std::string to_string(const dhtt_msgs::msg::Pair& pred)
	{
			std::string to_ret = std::string( ( pred.negate ) ? "!" : " " );

			return "(" + to_ret + pred.key + ": " + pred.value + ")";
	}

	std::string to_string(std::shared_ptr<PredicateConjunction> pred) 
	{
		// essentially strings should look like () for each predicate, [] for each depth, ! is not, and then key: val

		auto output_pred = [](dhtt_msgs::msg::Pair to_convert) 
		{
			std::string to_ret = std::string( ( to_convert.negate ) ? "!" : " " );

			return "(" + to_ret + to_convert.key + ": " + to_convert.value + ")";
		};

		std::function<std::string(const PredicateConjunction pred)> to_string_helper = [&](const PredicateConjunction current)
		{
			std::string to_ret;

			std::string op = ( current.logical_operator == LOGICAL_AND ) ? "^" : "v";

			for ( auto iter = current.predicates.begin() ; iter != current.predicates.end() ; iter++ )
			{
				to_ret += output_pred(*iter);

				if ( not ( iter == ( current.predicates.end() - 1 ) ) )
					to_ret += op;
			}

			for ( auto iter = current.conjunctions.begin() ; iter != current.conjunctions.end() ; iter++ )
				to_ret += ( ( current.predicates.size() == 0 ) ? "" : op ) + "[" +  to_string_helper(**iter) + "]";

			return to_ret;
		};

		return to_string_helper(*pred);
	}

	std::string to_string(const PredicateConjunction& pred)
	{
		std::shared_ptr<PredicateConjunction> tmp = std::make_shared<PredicateConjunction>();
		*tmp = pred;

		return to_string(tmp);
	}

	PredicateConjunction convert_to_struct(std::string to_convert)
	{
		// recurse as if we are already given ( input ) without the parentheses 
		auto create_pair = [](std::string pair_string)
		{
			dhtt_msgs::msg::Pair to_ret;

			to_ret.type = (int) (pair_string[0] - '0');
			to_ret.negate = ( pair_string[1] == '!' ) ? true : false;

			std::string key_val = pair_string.substr(2);

			std::size_t colon = key_val.find(":");

			to_ret.key = key_val.substr(0, colon);
			to_ret.value = key_val.substr(colon + 2);

			return to_ret;
		};

		// recurse as if we are already given [ input ] without the brackets
		std::function<PredicateConjunction(std::string)> convert_helper = [&](std::string current_level)
		{
			PredicateConjunction to_ret;
			int scope_start;
			std::string looking_for("-");
			
			bool in_scope = false;
			bool in_pred = false;
			bool set = false;
				
			// first find the logical operator
			for ( int i = 0 ; i < (int) current_level.size() ; i++ )
			{
				if ( current_level[i] == '[' )
					in_scope = true;
				else if ( current_level[i] == ']' and in_scope )
					in_scope = false;
				if ( current_level[i] == '(' )
					in_pred = true;
				else if ( current_level[i] == ')' and in_pred )
					in_pred = false;
				else if ( current_level[i] == '^' and not in_scope and not in_pred )
				{
					to_ret.logical_operator = LOGICAL_AND;
					set = true;
					break;
				}
				else if ( current_level[i] == 'v' and not in_scope and not in_pred )
				{
					to_ret.logical_operator = LOGICAL_OR;
					set = true;
					break;
				}
			}

			// for conditions of one predicate default to AND
			if ( not set )
				to_ret.logical_operator = LOGICAL_OTHER;

			for ( int i = 0 ; i < (int) current_level.size() ; i++ )
			{
				switch(current_level[i])
				{
				case '(':
					if ( not strcmp(looking_for.c_str(), "-" ) )
					{
						scope_start = i;
						looking_for = ")";
					}
					break;
				case ')':
					if ( not strcmp( looking_for.c_str(), ")" ) )
					{
						dhtt_msgs::msg::Pair n_pair = create_pair( current_level.substr(scope_start, i - scope_start) );

						to_ret.predicates.push_back(n_pair);
						looking_for = "-";
					}
					break;
				case '[':
					if ( not strcmp(looking_for.c_str(), "-" ) )
					{
						scope_start = i;
						looking_for = "]";
					}
					break;
				case ']':
					if ( not strcmp( looking_for.c_str(), "]" ) )
					{
						std::shared_ptr<PredicateConjunction> n_conj = std::make_shared<PredicateConjunction>();
						*n_conj = convert_helper( current_level.substr(scope_start, i - scope_start) );

						to_ret.conjunctions.push_back(n_conj);
						looking_for = "-";
					}
					break;
				}
			}

			return to_ret;
		};

		return convert_helper(to_convert);
	}

	struct PairComparator 
	{
		bool operator()(const dhtt_msgs::msg::Pair& lhs, const dhtt_msgs::msg::Pair& rhs) const
		{
			return not predicate_equals(lhs, rhs); 
		}
	};

	struct PairPartialComparator 
	{
		bool operator()(const dhtt_msgs::msg::Pair& lhs, const dhtt_msgs::msg::Pair& rhs) const
		{
			if ( predicate_partial_equals(lhs, rhs) )
				return false;

			return to_string(lhs) < to_string(rhs); 
		}
	};

	void remove_predicate_duplicates(PredicateConjunction& msg)
	{
		// make a set to remove duplicates
		std::set<dhtt_msgs::msg::Pair, PairComparator> duplicate_remover;

		for ( auto iter = msg.predicates.begin() ; iter != msg.predicates.end() ; iter++ )
			duplicate_remover.insert( (*iter) );

		msg.predicates.clear();
		
		// copy back to the vector
		msg.predicates.assign( duplicate_remover.begin(), duplicate_remover.end() );
	}

	// this assumes that the order of the predicates is temporal but that's not guaranteed anywhere ( in fact it's constantly violated )
	void remove_predicate_partial_duplicates(PredicateConjunction& msg)
	{
		// make a set to remove duplicates
		std::set<dhtt_msgs::msg::Pair, PairPartialComparator> duplicate_remover;

		std::function<void(PredicateConjunction&)> rec_remover = [&](PredicateConjunction& cur_msg)
		{
			std::vector<dhtt_msgs::msg::Pair> to_append;

			for ( auto iter = cur_msg.conjunctions.begin() ; iter != cur_msg.conjunctions.end() ; iter++ )
				rec_remover(**iter);

			for ( auto iter = cur_msg.predicates.rbegin() ; iter != cur_msg.predicates.rend() ; iter++ )
			{
				int initial = (int) duplicate_remover.size();

				duplicate_remover.insert( (*iter) );

				if ( (int) duplicate_remover.size() > initial )
					to_append.push_back(*iter);
			}

			cur_msg.predicates.clear();
			
			// copy back to the vector
			cur_msg.predicates.assign( to_append.begin(), to_append.end() );
		};

		rec_remover(msg);

		msg = flatten_predicates(msg);
	}

	bool violates_predicates(const PredicateConjunction& postconditions, const PredicateConjunction& preconditions) 
	{
		auto AND = [](bool a, bool b){ return a and b; };
		auto OR = [](bool a, bool b){ return a or b; };

		// this might be useful enough to make a full function
		std::function<void(std::list<std::map<std::string, bool>>&, const PredicateConjunction&)> build_ground_truth = 
			[&build_ground_truth](std::list<std::map<std::string, bool>>& ground_truth_tables, const PredicateConjunction& postconditions)
		{
			if ( postconditions.logical_operator == LOGICAL_AND or postconditions.logical_operator == LOGICAL_OTHER )
			{
				// loop through predicates
				for ( auto iter = postconditions.predicates.begin() ; iter != postconditions.predicates.end() ; iter++ )
				{
					dhtt_msgs::msg::Pair cp = predicate_copy(*iter);
					cp.negate = false;

					ground_truth_tables.front()[to_string(cp)] = (*iter).negate;
				}

				// then recurse through conjunctions
				for ( auto iter = postconditions.conjunctions.begin() ; iter != postconditions.conjunctions.end() ; iter++ )
					build_ground_truth(ground_truth_tables, **iter);
			}
			else // this call is essentially distributing the or across the equation (presumably very memory intensive)
			{
				// first make a deep copy of the 0th map
				std::map<std::string, bool> copy = ground_truth_tables.front();

				// then delete
				ground_truth_tables.erase(ground_truth_tables.begin());

				// then for each potential predicate insert a new table at 0
				for ( auto iter = postconditions.predicates.begin() ; iter != postconditions.predicates.end() ; iter++ )
				{
					std::map<std::string, bool> tmp = copy;

					dhtt_msgs::msg::Pair cp = predicate_copy(*iter);
					cp.negate = false;

					copy[to_string(cp)] = (*iter).negate;

					ground_truth_tables.insert(ground_truth_tables.begin(), tmp);
				}

				// for each conjunction insert a new table at 0 and recurse
				for ( auto iter = postconditions.conjunctions.begin() ; iter != postconditions.conjunctions.end(); iter++ )
				{					
					std::map<std::string, bool> tmp = copy;
					ground_truth_tables.insert(ground_truth_tables.begin(), tmp);

					build_ground_truth(ground_truth_tables, **iter);
				}
			}
		};

		std::function<bool(const std::map<std::string, bool>&, const PredicateConjunction&)> evaluate_conjunction = 
			[&] (const std::map<std::string, bool>& ground_truth, const PredicateConjunction& conditions) 
		{
			auto op = (conditions.logical_operator == LOGICAL_AND)? AND : OR;

			bool satisfied = (conditions.logical_operator == LOGICAL_AND)? true : false;

			// first check all the predicates
			for ( auto iter = conditions.predicates.begin() ; iter != conditions.predicates.end() ; iter++ )
			{
				dhtt_msgs::msg::Pair cp = predicate_copy(*iter);
				cp.negate = false;

				bool exists = true;
				bool pred = false;

				// try to find the normal value
				if ( ground_truth.find(to_string(cp)) != ground_truth.end() )
					pred = ground_truth.at(to_string(cp));
				else
					exists = false;

				cp.negate = not cp.negate;

				// now try the negated version to check if it is contradicted
				// if ( ground_truth.find(cp) != ground_truth.end() )
				// 	std::cout << "as;lkjf" << std::endl; 

				if ( not exists )
					satisfied = op(satisfied, true);
				else if ( pred == (*iter).negate )
					satisfied = op(satisfied, true);
				else // the negation of the predicate exists in ground truth and therefore the predicate is false 
					satisfied = op(satisfied, false);

				// early exit
				if ( ( conditions.logical_operator == LOGICAL_AND or conditions.logical_operator == LOGICAL_OTHER ) and not satisfied )
					return false;
				else if ( conditions.logical_operator == LOGICAL_OR and satisfied )
					return true;
			}

			// next recursively check the predicate conjunctions
			for ( auto iter = conditions.conjunctions.begin() ; iter != conditions.conjunctions.end() ; iter++ )
			{
				satisfied = op(satisfied, evaluate_conjunction( ground_truth, **iter ) );

				if ( ( conditions.logical_operator == LOGICAL_AND or conditions.logical_operator == LOGICAL_OTHER ) and not satisfied )
					return false;
				else if ( conditions.logical_operator == LOGICAL_OR and satisfied )
					return true;
			}

			return true;
		};

		// first build the dictionary of ground truth
		std::list<std::map<std::string, bool>> ground_truth_tables;
		ground_truth_tables.push_back(std::map<std::string, bool>());

		build_ground_truth(ground_truth_tables, postconditions);

		// check the truth value recursively against the dictionary and return
		bool to_ret = false;

		for ( auto iter = ground_truth_tables.begin() ; iter != ground_truth_tables.end() ; iter++ )
		{
			to_ret = evaluate_conjunction(*iter, preconditions);

			if (to_ret)
				return false;
		}

		return true;
	}

	bool contains_contradiction (const PredicateConjunction& to_check)
	{
		std::function<bool(const dhtt_msgs::msg::Pair&, const dhtt_msgs::msg::Pair&)> comp = 
			[](const dhtt_msgs::msg::Pair& lhs, const dhtt_msgs::msg::Pair& rhs) { return predicate_equals(lhs, rhs); };// wrapped in a lambda to please cpp
		std::map<dhtt_msgs::msg::Pair, bool, PairComparator> truth_table;
		bool to_ret = false;

		(void) to_check;

		// if ( current.logical_operator == LOGICAL_AND )
		// {
		// 	// check for contradiction in the predicates


		// 	// now check in the conjunctions

		// }

		return to_ret;
	}

	PredicateConjunction flatten_predicates(const PredicateConjunction& msg)
	{
		std::shared_ptr<dhtt_utils::PredicateConjunction> to_ret = conjunction_copy(msg);

		// these will be stored in order so they must be deleted in reverse order
		std::vector<std::vector<std::shared_ptr<PredicateConjunction>>::iterator> to_delete;
		std::vector< std::shared_ptr<PredicateConjunction> > to_append;

		// deep copying belowing so that a pointer isn't kept and erasing doesn't entirely delete things
		for ( auto iter = to_ret->conjunctions.begin() ; iter != to_ret->conjunctions.end() ; iter++ )
		{
			bool only = (int) ( (**iter).predicates.size() + (**iter).conjunctions.size() ) == 1;

			if ( to_ret->logical_operator == (**iter).logical_operator or (**iter).logical_operator == LOGICAL_OTHER or only )
			{
				// deep copy over the predicates
				for ( auto pred_iter = (**iter).predicates.begin() ; pred_iter != (**iter).predicates.end() ; pred_iter++ )
					to_ret->predicates.push_back( predicate_copy( (*pred_iter) ) );

				// now make sure to deep copy of the predicates of iter, and to append iter to the to_delete pile
				for ( auto conj_iter = (**iter).conjunctions.begin() ; conj_iter != (**iter).conjunctions.end() ; conj_iter++ )
					to_append.push_back(  conjunction_copy( (**conj_iter) ) );

				to_delete.push_back(iter);
			}
		}

		// now clean up 
		for ( auto rev_iter = to_delete.rbegin() ; rev_iter != to_delete.rend() ; rev_iter++ )
			to_ret->conjunctions.erase( (*rev_iter) );

		// and copy over the lingering conjunctions (not necessary to deep copy)
		to_ret->conjunctions.insert(to_ret->conjunctions.end(), to_append.begin(), to_append.end());

		return *to_ret;
	}

	void append_predicate_conjunction(PredicateConjunction& to, const PredicateConjunction from)
	{
		// I can just add it to the list of predicate conjunctions and then flatten the list (to ensure that they have the same logical operator)
		std::shared_ptr<dhtt_utils::PredicateConjunction> tmp = conjunction_copy(from);

		to.conjunctions.push_back(tmp);

		to = flatten_predicates(to);
	}

	PredicateConjunction combine_predicates(const PredicateConjunction& l_pred, const PredicateConjunction& r_pred)
	{
		std::shared_ptr<dhtt_utils::PredicateConjunction> combined = conjunction_copy( l_pred );

		// we just arbitrarily choose one to be the outer conjunction ( always the left hand side here )
		if ( l_pred.logical_operator != r_pred.logical_operator )
			combined->conjunctions.push_back( conjunction_copy( r_pred ) );
		else
		{
			for ( auto iter = r_pred.predicates.begin() ; iter != r_pred.predicates.end() ; iter++ )
				combined->predicates.push_back( predicate_copy( *iter ) );

			for ( auto iter = r_pred.conjunctions.begin() ; iter != r_pred.conjunctions.end() ; iter++ )
				combined->conjunctions.push_back( conjunction_copy( **iter ) );
		}

		return *combined; 
	}

	std::shared_ptr<PredicateConjunction> negate_predicates(const PredicateConjunction& to_negate)
	{
		std::shared_ptr<PredicateConjunction> to_ret = std::make_shared<PredicateConjunction>();
		to_ret->predicates = std::vector<dhtt_msgs::msg::Pair>();
		to_ret->conjunctions = std::vector<std::shared_ptr<PredicateConjunction>>();
		to_ret->logical_operator = to_negate.logical_operator;

		// negate all the predicates
		for ( auto iter = to_negate.predicates.begin() ; iter != to_negate.predicates.end() ; iter++ )
		{
			dhtt_msgs::msg::Pair cp = predicate_copy(*iter);
			cp.negate = not cp.negate;

			to_ret->predicates.push_back(cp);
		}

		// negate the conjunctions recursively
		for ( auto iter = to_negate.conjunctions.begin() ; iter != to_negate.conjunctions.end() ; iter++ )
		{
			std::shared_ptr<PredicateConjunction> tmp = negate_predicates(**iter);

			to_ret->conjunctions.push_back(tmp);
		}

		return to_ret;
	}

	std::vector<dhtt_msgs::msg::Pair>::iterator find_partial_predicate(std::vector<dhtt_msgs::msg::Pair> to_search, dhtt_msgs::msg::Pair to_find)
	{
		auto unary_predicate_equals = [&](dhtt_msgs::msg::Pair to_check) { return predicate_partial_equals(to_check, to_find); };

		return std::find_if(to_search.begin(), to_search.end(), unary_predicate_equals);
	}

	std::vector<dhtt_msgs::msg::Pair>::iterator find_predicate(std::vector<dhtt_msgs::msg::Pair>& to_search, dhtt_msgs::msg::Pair to_find)
	{
		auto unary_predicate_equals = [&](dhtt_msgs::msg::Pair to_check) { return predicate_equals(to_check, to_find); };

		return std::find_if(to_search.begin(), to_search.end(), unary_predicate_equals);
	}

	PredicateConjunction remove_partial_dependencies(const std::vector<PredicateConjunction> post, const PredicateConjunction& pre)
	{
		std::shared_ptr<dhtt_utils::PredicateConjunction> to_ret = conjunction_copy(pre);

		auto post_iter = post.begin();

		while ( not to_ret->predicates.empty() and post_iter != post.end() )
		{
			for ( auto inner_iter = (*post_iter).predicates.begin() ; inner_iter != (*post_iter).predicates.end() ; inner_iter++ )
			{
				auto to_ret_iter = find_predicate( to_ret->predicates, (*inner_iter) );

				if ( to_ret_iter != to_ret->predicates.end() )
				{
					to_ret->predicates.erase(to_ret_iter);
				}
			}

			post_iter++;
		}

		return *to_ret;
	}
}