#ifndef RESOURCE_POTENTIAL_HPP_
#define RESOURCE_POTENTIAL_HPP_ 

#include "dhtt/tree/potential_type.hpp"
#include "dhtt/tree/node.hpp"

#define RESOURCE_USAGE_THRESHOLD .6

namespace dhtt_plugins
{
    /**
     * \brief monitors and changes activation potential according to the number of resources used in the tree
     * 
     * When resource usage is below a threshold value the activation potential is just returned from the node logic, however,
     *  when the threshold is exceeded the number of resources used in the subtree is given so that the dHTT prioritizes freeing 
     *  up resources.
     */
    class ResourcePotential : public dhtt::PotentialType
    {
    public:
        /**
         * \brief returns the perceived efficiency from the node logic plugin or the number of resources
         *  owned by the subtree over the total number of resources
         */
        double compute_activation_potential(dhtt::Node* container) override; 
    };
}

#endif // RESOURCE_POTENTIAL_HPP_