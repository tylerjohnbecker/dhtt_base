#ifndef POTENTIAL_TYPE_HPP_
#define POTENTIAL_TYPE_HPP_

namespace dhtt
{
    class Node;

    /**
     * \brief a lightweight plugin for computing the activation potential of a node
     * 
     * This class is meant to enable the designer to change the computation of the activation potential without dealing with any of the code
     *  related to the node logic.
     */
    class PotentialType
    {
    public:
        /**
         * \brief computes the activation potential of the node
         * 
         * \param container dhtt::Node which contains this plugin for help with computations
         * 
         * \return double of activation potential between 0 - 1
         */
        virtual double compute_activation_potential(Node* container) = 0;
    protected:
    private:
    };
}

#endif // POTENTIAL_TYPE_HPP_