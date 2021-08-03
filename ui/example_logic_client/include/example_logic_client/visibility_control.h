#ifndef CLI__VISIBILITY_CONTROL_H_
#define CLI__VISIBILITY_CONTROL_H_

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define CLI_EXPORT __attribute__ ((dllexport))
    #define CLI_IMPORT __attribute__ ((dllimport))
  #else
    #define CLI_EXPORT __declspec(dllexport)
    #define CLI_IMPORT __declspec(dllimport)
  #endif
  #ifdef CLI_BUILDING_LIBRARY
    #define CLI_PUBLIC CLI_EXPORT
  #else
    #define CLI_PUBLIC CLI_IMPORT
  #endif
  #define CLI_PUBLIC_TYPE CLI_PUBLIC
  #define CLI_LOCAL
#else
  #define CLI_EXPORT __attribute__ ((visibility("default")))
  #define CLI_IMPORT
  #if __GNUC__ >= 4
    #define CLI_PUBLIC __attribute__ ((visibility("default")))
    #define CLI_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define CLI_PUBLIC
    #define CLI_LOCAL
  #endif
  #define CLI_PUBLIC_TYPE
#endif

#endif  // CLI__VISIBILITY_CONTROL_H_
