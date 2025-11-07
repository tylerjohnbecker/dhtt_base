#include "dhtt_plugins/potentials/test_potential.hpp"

namespace dhtt_plugins
{
TestPotential::TestPotential()
{
	++TestPotential::nodes_added;
	this->my_node_counter = TestPotential::nodes_added;
}

double TestPotential::compute_activation_potential(dhtt::Node *container)
{
	container; // unused
	return static_cast<double>(this->my_node_counter) /
		   static_cast<double>(TestPotential::nodes_added);
}
} // namespace dhtt_plugins