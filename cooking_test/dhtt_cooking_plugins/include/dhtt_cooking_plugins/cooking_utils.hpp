#ifndef COOKING_UTILS_HPP_
#define COOKING_UTILS_HPP_

#include <map>
#include <cstring>

#include "dhtt_msgs/msg/resource.hpp"

#include "dhtt_cooking_msgs/msg/cooking_types.hpp"
#include "dhtt_cooking_msgs/msg/cooking_object.hpp"


namespace dhtt_cooking_utils
{
    /**
     * \brief a translator from the string params we use for input to the int assigned from the msg file
     * 
     * \param param string parameter inputted from the class
     * 
     * \return int8_t assigned integer for the given param or -1 if the given param wasn't found
     */
    int8_t param_to_msg_val(std::string param);

    /**
     * \brief returns the name of the cooking object in the resource list
     * 
     * \param obj cooking object to translate into a name
     * 
     * \return internal name for obj (including consideration for it's conditions)
     */
    std::string get_resource_name(dhtt_cooking_msgs::msg::CookingObject obj);

    /**
     * \brief takes a resource name and transforms it to a CookingTypes type
     * 
     * \param name taken in the format (paramType_worldid)
     * 
     * \return msg type of paramType in name
     */
    int8_t resource_name_to_type(std::string name);

    /**
     * \brief checks if the given type is one of the known dynamic objects
     * 
     * \return True if the object is dynamic, false otherwise
     */
    bool is_dynamic_obj(int8_t type);

    /**
     * \brief overload of is_dynamic_obj for CookingObject msgs
     * 
     * \return True if the object is dynamic, false otherwise
     */
    bool is_dynamic_obj(dhtt_cooking_msgs::msg::CookingObject obj);

    /**
     * \brief checks if the given type is one of the known static objects
     * 
     * \return True if the object is static, false otherwise
     */
    bool is_static_obj(int8_t type);

    /**
     * \brief overload of is_static_obj for CookingObject msgs
     * 
     * \return True if the object is static, false otherwise
     */
    bool is_static_obj(dhtt_cooking_msgs::msg::CookingObject obj);

    /**
     * \brief checks if given type is one of the CookingObject types
     * 
     * \return True if type is a CookingObject type, false otherwise
     */
    bool is_cooking_obj(int8_t type);

    /**
     * \brief returns true if the given list contains a resource of the given type
     * 
     * \param type the type to search for in the list
     * \param to_search the list of resources to search
     * 
     * \return true if type is in to_search, false otherwise
     */
    bool has_type(int8_t type, std::vector<dhtt_msgs::msg::Resource> to_search);

    /**
     * \brief returns true if the given static object has no object on top of it
     * 
     * \param obj object being tested
     * \param ground_truth map of the environment objects by name
     * 
     * \return true if object is free, false otherwise
     */
    bool is_free(dhtt_msgs::msg::Resource& obj, std::map<std::string, dhtt_cooking_msgs::msg::CookingObject> ground_truth);

    /**
     * \brief check if a given type is a tool which cooks something over time
     * 
     * \param type type to check
     * 
     * \return true if after executing given type takes time, false otherwise
     */
    bool is_cooking_tool(int8_t type);

    /**
     * \brief check if a given cooking object is a tool which cooks something over time
     * 
     * \param obj object to check
     * 
     * \return true if after executing given type takes time, false otherwise
     */
    bool is_cooking_tool(dhtt_cooking_msgs::msg::CookingObject obj);

}

#endif // COOKING_UTILS_HPP_