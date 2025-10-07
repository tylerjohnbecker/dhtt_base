#include "dhtt/utils/resource_utils.hpp"

namespace dhtt_utils
{
    dhtt_msgs::msg::Resources vector_to_msg( VectorResources data )
    {
        dhtt_msgs::msg::Resources to_ret;

        for ( int i = 0 ; i < (int) data.names.size() ; i++ )
        {
            dhtt_msgs::msg::Resource n_resource;

            n_resource.name = data.names[i];
            n_resource.type = data.types[i];
            n_resource.channel = data.channels[i];
            n_resource.locked = data.locks[i];
            n_resource.owners = data.owners[i];

            to_ret.resource_state.push_back(n_resource);
        }

        return to_ret;
    }

    dhtt_msgs::msg::Resources vector_to_msg( std::vector<std::string> names, std::vector<long int> types, std::vector<long int> channels, std::vector<bool> locked, std::vector<long int> owners )
    {
        return vector_to_msg({names, types, channels, locked, owners});
    }

    VectorResources msg_to_vector( std::vector<dhtt_msgs::msg::Resource> msgs )
    {
        VectorResources to_ret;

        for ( auto iter : msgs )
        {
            to_ret.names.push_back(iter.name);
            to_ret.types.push_back(iter.type);
            to_ret.channels.push_back(iter.channel);
            to_ret.locks.push_back(iter.locked);
            to_ret.owners.push_back(iter.owners);
        }
        
        return to_ret;
    }

    VectorResources msg_to_vector( dhtt_msgs::msg::Resources msg )
    {
        return msg_to_vector(msg.resource_state);
    }
}