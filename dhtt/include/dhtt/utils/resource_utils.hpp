#ifndef RESOURCE_UTILS_HPP_
#define RESOURCE_UTILS_HPP_

#include "dhtt_msgs/msg/resources.hpp"

#include <vector>
#include <any>
#include <cassert>

namespace dhtt_utils 
{
    typedef struct VectorResources
    {
        std::vector<std::string> names;
        std::vector<long int> types;
        std::vector<long int> channels;
        std::vector<bool> locks;
        std::vector<long int> owners;

        VectorResources() {};

        VectorResources(std::vector<std::string> a, std::vector<long int> b, std::vector<long int> c, std::vector<bool> d, std::vector<long int> e)
        {
            names = a;
            types = b;
            channels = c;
            locks = d;
            owners = e;
        };

        VectorResources(std::initializer_list<std::any> tup) 
        {
            assert( (int) tup.size() == 5 );

            names = std::any_cast<std::vector<std::string>>(*(tup.begin()));
            types = std::any_cast<std::vector<long int>>(*(tup.begin() + 1));
            channels = std::any_cast<std::vector<long int>>(*(tup.begin() + 2));
            locks = std::any_cast<std::vector<bool>>(*(tup.begin() + 3));
            owners = std::any_cast<std::vector<long int>>(*(tup.begin() + 4));
        };
    } VectorResources;
    
    /**
     * \brief creates a dhtt_msgs::msg::Resources object from vectors containing the necessary information
     * 
     * Vectors should all be the same size and the information for a single resource should correspond to a single index across all of them.
     * 
     * \param data VectorResources struct containing all of the information to transform
     * 
     * \return msg which is equivalent to the data
     */
    dhtt_msgs::msg::Resources vector_to_msg( VectorResources data );

    /**
     * \brief overload of vector_to_msg which takes each vector individually as a parameter
     * 
     * \param names vector of strings 
     * \param types vector of types represented as integers
     * \param channels vector of resource channels
     * \param locks vector of booleans representing locked or unlocked
     * \param owners vector of each resource's number of owners 
     * 
     * \return msg representation of above information
     */
    dhtt_msgs::msg::Resources vector_to_msg( std::vector<std::string> names, std::vector<long int> types, std::vector<long int> channels, std::vector<bool> locked, std::vector<long int> owners );

    /**
     * \brief creates a VectorResources object from a given vector of dhtt_msgs::msg::Resource msgs
     * 
     * This transformation is useful for pushing resources to the param server. Will put default values for resources with empty values.
     * 
     * \param msgs object to transform for list representation
     * 
     * \return VectorResources struct with equivalent information to msg
     */
    VectorResources msg_to_vector( std::vector<dhtt_msgs::msg::Resource> msgs );

    /**
     * \brief creates a VectorResources object from a given dhtt_msgs::msg::Resources msg
     * 
     * This is an overload of msg_to_vector.
     * 
     * \param msg object to transform for list representation
     * 
     * \return VectorResources struct with equivalent information to msg
     */
    VectorResources msg_to_vector( dhtt_msgs::msg::Resources msg );
}

#endif // RESOURCE_UTILS_HPP_