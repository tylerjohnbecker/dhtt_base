#include "dhtt_cooking_plugins/cooking_utils.hpp"

namespace dhtt_cooking_utils
{
    int8_t param_to_msg_val(std::string param)
    {
        if ( param == "Counter" )
            return dhtt_cooking_msgs::msg::CookingTypes::COUNTER;
        if ( param == "Cutboard" )
            return dhtt_cooking_msgs::msg::CookingTypes::CUT_BOARD;
        if ( param == "Toaster" )
            return dhtt_cooking_msgs::msg::CookingTypes::TOASTER;
        if ( param == "Pot" )
            return dhtt_cooking_msgs::msg::CookingTypes::POT;
        if ( param == "Blender" )
            return dhtt_cooking_msgs::msg::CookingTypes::BLENDER;
        if ( param == "Pan" )
            return dhtt_cooking_msgs::msg::CookingTypes::PAN;
        if ( param == "PlateDispenser" )
            return dhtt_cooking_msgs::msg::CookingTypes::PLATE_DISPENSER;
        if ( param == "BreadDispenser" )
            return dhtt_cooking_msgs::msg::CookingTypes::BREAD_DISPENSER;
        if ( param == "PastaDispenser" )
            return dhtt_cooking_msgs::msg::CookingTypes::PASTA_DISPENSER;
        if ( param == "IceDispenser" )
            return dhtt_cooking_msgs::msg::CookingTypes::ICE_DISPENSER;
        if ( param == "OnionDispenser" )
            return dhtt_cooking_msgs::msg::CookingTypes::ONION_DISPENSER;
        if ( param == "EggDispenser" )
            return dhtt_cooking_msgs::msg::CookingTypes::EGG_DISPENSER;
        if ( param == "BananaDispenser" )
            return dhtt_cooking_msgs::msg::CookingTypes::BANANA_DISPENSER;
        if ( param == "StrawberryDispenser" )
            return dhtt_cooking_msgs::msg::CookingTypes::STRAWBERRY_DISPENSER;
        if ( param == "TomatoDispenser" )
            return dhtt_cooking_msgs::msg::CookingTypes::TOMATO_DISPENSER;
        if ( param == "LettuceDispenser" )
            return dhtt_cooking_msgs::msg::CookingTypes::LETTUCE_DISPENSER;
        if ( param == "AbsorbingDeliversquare" )
            return dhtt_cooking_msgs::msg::CookingTypes::ABSORBING_DELIVER_SQUARE;
        if ( param == "Plate" )
            return dhtt_cooking_msgs::msg::CookingTypes::PLATE;
        if ( param == "Bread" )
            return dhtt_cooking_msgs::msg::CookingTypes::BREAD;
        if ( param == "Pasta" )
            return dhtt_cooking_msgs::msg::CookingTypes::PASTA;
        if ( param == "Ice" )
            return dhtt_cooking_msgs::msg::CookingTypes::ICE;
        if ( param == "Onion" )
            return dhtt_cooking_msgs::msg::CookingTypes::ONION;
        if ( param == "Egg" )
            return dhtt_cooking_msgs::msg::CookingTypes::EGG;
        if ( param == "Banana" )
            return dhtt_cooking_msgs::msg::CookingTypes::BANANA;
        if ( param == "Strawberry" )
            return dhtt_cooking_msgs::msg::CookingTypes::STRAWBERRY;
        if ( param == "Tomato" )
            return dhtt_cooking_msgs::msg::CookingTypes::TOMATO;
        if ( param == "Lettuce" )
            return dhtt_cooking_msgs::msg::CookingTypes::LETTUCE;
        if ( param == "BreadChopped" )
            return dhtt_cooking_msgs::msg::CookingTypes::BREAD_CHOPPED;
        if ( param == "BreadToasted" )
            return dhtt_cooking_msgs::msg::CookingTypes::BREAD_TOASTED;
        if ( param == "PastaCooked" )
            return dhtt_cooking_msgs::msg::CookingTypes::PASTA_COOKED;
        if ( param == "OnionChopped" )
            return dhtt_cooking_msgs::msg::CookingTypes::ONION_CHOPPED;
        if ( param == "EggFried" )
            return dhtt_cooking_msgs::msg::CookingTypes::EGG_FRIED;
        if ( param == "BananaChopped" )
            return dhtt_cooking_msgs::msg::CookingTypes::BANANA_CHOPPED;
        if ( param == "TomatoChopped" )
            return dhtt_cooking_msgs::msg::CookingTypes::TOMATO_CHOPPED;
        if ( param == "TomatoFried" )
            return dhtt_cooking_msgs::msg::CookingTypes::TOMATO_FRIED;
        if ( param == "LettuceChopped" )
            return dhtt_cooking_msgs::msg::CookingTypes::LETTUCE_CHOPPED;
        if ( param == "Smoothie" )
            return dhtt_cooking_msgs::msg::CookingTypes::SMOOTHIE;

        return -1;
    }

    std::string get_resource_name(dhtt_cooking_msgs::msg::CookingObject obj)
    {
        std::string relevant_condition = "";

        for ( auto cond : obj.physical_state )
        {
            if ( not strcmp ( cond.c_str(), "Chopped" ) )
            {
                relevant_condition = cond;
            }
            else if ( not strcmp (cond.c_str(), "Toasted" ) or
                        not strcmp (cond.c_str(), "Cooked" ) or
                        not strcmp (cond.c_str(), "Fried" ) ) 
            {
                relevant_condition = cond;
                break;
            }
        }

        return obj.object_type + relevant_condition + "_" + std::to_string(obj.world_id);
    }

    int8_t resource_name_to_type(std::string name)
    {
        // isolate the paramType from paramType_worldId
        std::string param_type = name.substr(0, name.find("_"));

        // return the param_to_msg_val_output
        return param_to_msg_val(param_type);
    }

    bool is_dynamic_obj(int8_t type)
    {
        if ( type >= dhtt_cooking_msgs::msg::CookingTypes::PLATE &&
                type <= dhtt_cooking_msgs::msg::CookingTypes::SMOOTHIE )
            return true;

        return false;
    }

    bool is_dynamic_obj(dhtt_cooking_msgs::msg::CookingObject obj)
    {
        return is_dynamic_obj(param_to_msg_val(obj.object_type));
    }
    
    bool is_static_obj(int8_t type)
    {
        if ( type >= dhtt_cooking_msgs::msg::CookingTypes::COUNTER &&
                type <= dhtt_cooking_msgs::msg::CookingTypes::ABSORBING_DELIVER_SQUARE )
            return true;

        return false;
    }

    bool is_static_obj(dhtt_cooking_msgs::msg::CookingObject obj)
    {
        return is_static_obj(param_to_msg_val(obj.object_type));
    }

    bool is_cooking_obj(int8_t type)
    {
        return is_dynamic_obj(type) or is_static_obj(type);
    }

    bool has_type(int8_t type, std::vector<dhtt_msgs::msg::Resource> to_search)
    {
        auto find_type = [&type](dhtt_msgs::msg::Resource check) { return check.type == type; };

        return std::find_if(to_search.begin(), to_search.end(), find_type) != to_search.end();
    }

    bool is_free(dhtt_msgs::msg::Resource& obj, std::map<std::string, dhtt_cooking_msgs::msg::CookingObject> ground_truth)
    {
        auto same_location = [](dhtt_cooking_msgs::msg::CookingObject& lhs, dhtt_cooking_msgs::msg::CookingObject& rhs)
        {
            return lhs.location.x == rhs.location.x and lhs.location.y == rhs.location.y;
        };

        auto obj_gt = ground_truth[obj.name]; // throws exception if object not in map (intended)

        for ( auto pair : ground_truth )
        {
            if ( not strcmp(obj.name.c_str(), pair.first.c_str()) )
                continue;

            if ( same_location(obj_gt, pair.second) )
                return false;
        }

        return true;
    }

    bool is_cooking_tool(int8_t type)
    {
        return type >= dhtt_cooking_msgs::msg::CookingTypes::TOASTER and type <= dhtt_cooking_msgs::msg::CookingTypes::PAN;
    }

    bool is_cooking_tool(dhtt_cooking_msgs::msg::CookingObject obj)
    {
        return is_cooking_tool(param_to_msg_val(obj.object_type));
    }
}