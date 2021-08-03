#ifndef GENERATE_PATH__VISIBILITY_CONTROL_H_
#define GENERATE_PATH__VISIBILITY_CONTROL_H_

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define GENERATE_PATH_EXPORT __attribute__ ((dllexport))
    #define GENERATE_PATH_IMPORT __attribute__ ((dllimport))
  #else
    #define GENERATE_PATH_EXPORT __declspec(dllexport)
    #define GENERATE_PATH_IMPORT __declspec(dllimport)
  #endif
  #ifdef GENERATE_PATH_BUILDING_LIBRARY
    #define GENERATE_PATH_PUBLIC GENERATE_PATH_EXPORT
  #else
    #define GENERATE_PATH_PUBLIC GENERATE_PATH_IMPORT
  #endif
  #define GENERATE_PATH_PUBLIC_TYPE GENERATE_PATH_PUBLIC
  #define GENERATE_PATH_LOCAL
#else
  #define GENERATE_PATH_EXPORT __attribute__ ((visibility("default")))
  #define GENERATE_PATH_IMPORT
  #if __GNUC__ >= 4
    #define GENERATE_PATH_PUBLIC __attribute__ ((visibility("default")))
    #define GENERATE_PATH_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define GENERATE_PATH_PUBLIC
    #define GENERATE_PATH_LOCAL
  #endif
  #define GENERATE_PATH_PUBLIC_TYPE
#endif

#endif  // GENERATE_PATH__VISIBILITY_CONTROL_H_
