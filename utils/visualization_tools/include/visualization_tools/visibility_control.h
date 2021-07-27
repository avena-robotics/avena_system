#ifndef DEBUGGING_TOOLS__VISIBILITY_CONTROL_H_
#define DEBUGGING_TOOLS__VISIBILITY_CONTROL_H_

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define DEBUGGING_TOOLS_EXPORT __attribute__ ((dllexport))
    #define DEBUGGING_TOOLS_IMPORT __attribute__ ((dllimport))
  #else
    #define DEBUGGING_TOOLS_EXPORT __declspec(dllexport)
    #define DEBUGGING_TOOLS_IMPORT __declspec(dllimport)
  #endif
  #ifdef DEBUGGING_TOOLS_BUILDING_LIBRARY
    #define DEBUGGING_TOOLS_PUBLIC DEBUGGING_TOOLS_EXPORT
  #else
    #define DEBUGGING_TOOLS_PUBLIC DEBUGGING_TOOLS_IMPORT
  #endif
  #define DEBUGGING_TOOLS_PUBLIC_TYPE DEBUGGING_TOOLS_PUBLIC
  #define DEBUGGING_TOOLS_LOCAL
#else
  #define DEBUGGING_TOOLS_EXPORT __attribute__ ((visibility("default")))
  #define DEBUGGING_TOOLS_IMPORT
  #if __GNUC__ >= 4
    #define DEBUGGING_TOOLS_PUBLIC __attribute__ ((visibility("default")))
    #define DEBUGGING_TOOLS_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define DEBUGGING_TOOLS_PUBLIC
    #define DEBUGGING_TOOLS_LOCAL
  #endif
  #define DEBUGGING_TOOLS_PUBLIC_TYPE
#endif

#endif  // DEBUGGING_TOOLS__VISIBILITY_CONTROL_H_
