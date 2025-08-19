#ifndef DHTT_COOKING_PLUGINS__VISIBILITY_CONTROL_H_
#define DHTT_COOKING_PLUGINS__VISIBILITY_CONTROL_H_

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define DHTT_COOKING_PLUGINS_EXPORT __attribute__ ((dllexport))
    #define DHTT_COOKING_PLUGINS_IMPORT __attribute__ ((dllimport))
  #else
    #define DHTT_COOKING_PLUGINS_EXPORT __declspec(dllexport)
    #define DHTT_COOKING_PLUGINS_IMPORT __declspec(dllimport)
  #endif
  #ifdef DHTT_COOKING_PLUGINS_BUILDING_LIBRARY
    #define DHTT_COOKING_PLUGINS_PUBLIC DHTT_COOKING_PLUGINS_EXPORT
  #else
    #define DHTT_COOKING_PLUGINS_PUBLIC DHTT_COOKING_PLUGINS_IMPORT
  #endif
  #define DHTT_COOKING_PLUGINS_PUBLIC_TYPE DHTT_COOKING_PLUGINS_PUBLIC
  #define DHTT_COOKING_PLUGINS_LOCAL
#else
  #define DHTT_COOKING_PLUGINS_EXPORT __attribute__ ((visibility("default")))
  #define DHTT_COOKING_PLUGINS_IMPORT
  #if __GNUC__ >= 4
    #define DHTT_COOKING_PLUGINS_PUBLIC __attribute__ ((visibility("default")))
    #define DHTT_COOKING_PLUGINS_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define DHTT_COOKING_PLUGINS_PUBLIC
    #define DHTT_COOKING_PLUGINS_LOCAL
  #endif
  #define DHTT_COOKING_PLUGINS_PUBLIC_TYPE
#endif

#endif  // DHTT_COOKING_PLUGINS__VISIBILITY_CONTROL_H_
