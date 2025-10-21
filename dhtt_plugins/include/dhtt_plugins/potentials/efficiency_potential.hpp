#ifndef EFFICIENCY_POTENTIAL_HPP_
#define EFFICIENCY_POTENTIAL_HPP_

#include "dhtt/tree/potential_type.hpp"
#include "dhtt/tree/node.hpp"

namespace dhtt_plugins
{
    /**
     * \brief Returns the perceived efficiency from the logic of the Node
     * 
     * This class is meant to represent the original computation of activation potential for dHTT's where only
     *  the perceived efficiency of a node is taken into account when computing activation potential.
     */
    class EfficiencyPotential : public dhtt::PotentialType
    {
    public:
        /**
         * \brief returns the perceived efficiency from the node logic plugin
         */
        double compute_activation_potential(dhtt::Node* container) override; 
    };
}

#endif // EFFICIENCY_POTENTIAL_HPP_