#include "dhtt_plugins/potentials/resource_potential.hpp"

namespace dhtt_plugins
{
    double ResourcePotential::compute_activation_potential(dhtt::Node* container)
    {
        // find number of total locked resources 
        auto resources = container->get_resource_state();
        int total_num = (int) resources.size();
        int total_locked = 0;
        
        for ( auto resource : resources )
            if ( resource.locked )
                total_locked++;

        // check the proportion of locked resources against the threshold
        double proportion = total_locked / total_num;

        // if above the threshold return our proportion of the resources
        if ( proportion >= RESOURCE_USAGE_THRESHOLD )
        {
            // return our proportion of the total locked resources
            return container->get_subtree_resources() / total_locked;
        }

        // else return the perceived efficiency
        return container->get_logic()->get_perceived_efficiency(container);
    }
}