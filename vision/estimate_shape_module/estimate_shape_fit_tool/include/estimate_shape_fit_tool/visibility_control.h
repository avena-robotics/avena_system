#ifndef FIT_TOOL__VISIBILITY_CONTROL_H_
#define FIT_TOOL__VISIBILITY_CONTROL_H_

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define FIT_TOOL_EXPORT __attribute__ ((dllexport))
    #define FIT_TOOL_IMPORT __attribute__ ((dllimport))
  #else
    #define FIT_TOOL_EXPORT __declspec(dllexport)
    #define FIT_TOOL_IMPORT __declspec(dllimport)
  #endif
  #ifdef FIT_TOOL_BUILDING_LIBRARY
    #define FIT_TOOL_PUBLIC FIT_TOOL_EXPORT
  #else
    #define FIT_TOOL_PUBLIC FIT_TOOL_IMPORT
  #endif
  #define FIT_TOOL_PUBLIC_TYPE FIT_TOOL_PUBLIC
  #define FIT_TOOL_LOCAL
#else
  #define FIT_TOOL_EXPORT __attribute__ ((visibility("default")))
  #define FIT_TOOL_IMPORT
  #if __GNUC__ >= 4
    #define FIT_TOOL_PUBLIC __attribute__ ((visibility("default")))
    #define FIT_TOOL_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define FIT_TOOL_PUBLIC
    #define FIT_TOOL_LOCAL
  #endif
  #define FIT_TOOL_PUBLIC_TYPE
#endif

#endif  // FIT_TOOL__VISIBILITY_CONTROL_H_
