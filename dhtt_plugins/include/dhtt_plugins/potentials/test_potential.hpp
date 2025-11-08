#ifndef TEST_POTENTIAL_HPP_
#define TEST_POTENTIAL_HPP_

#include "dhtt/tree/node.hpp"
#include "dhtt/tree/potential_type.hpp"

namespace dhtt_plugins
{
/**
 * \brief dummy activation potential plugin for testing
 *
 * Scales activation potential so that early nodes added have low potential
 */
class TestPotential : public dhtt::PotentialType
{
  public:
	virtual ~TestPotential() = default;
	TestPotential();

	/**
	 * \brief returns a dummy activation potential such that nodes added first have lower potential
	 */
	double compute_activation_potential(dhtt::Node *container) override;

  private:
	static inline size_t nodes_added = 0;
	size_t my_node_counter;
};
} // namespace dhtt_plugins

#endif // TEST_POTENTIAL_HPP_