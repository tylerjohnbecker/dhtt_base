#include "dhtt_plugins/potentials/efficiency_potential.hpp"

namespace dhtt_plugins
{
    double EfficiencyPotential::compute_activation_potential(dhtt::Node* container)
    {
        return container->get_logic()->get_perceived_efficiency(container);
    }
}