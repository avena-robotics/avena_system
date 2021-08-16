#ifndef FIT_KNIFE__VISIBILITY_CONTROL_H_
#define FIT_KNIFE__VISIBILITY_CONTROL_H_

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define FIT_KNIFE_EXPORT __attribute__ ((dllexport))
    #define FIT_KNIFE_IMPORT __attribute__ ((dllimport))
  #else
    #define FIT_KNIFE_EXPORT __declspec(dllexport)
    #define FIT_KNIFE_IMPORT __declspec(dllimport)
  #endif
  #ifdef FIT_KNIFE_BUILDING_LIBRARY
    #define FIT_KNIFE_PUBLIC FIT_KNIFE_EXPORT
  #else
    #define FIT_KNIFE_PUBLIC FIT_KNIFE_IMPORT
  #endif
  #define FIT_KNIFE_PUBLIC_TYPE FIT_KNIFE_PUBLIC
  #define FIT_KNIFE_LOCAL
#else
  #define FIT_KNIFE_EXPORT __attribute__ ((visibility("default")))
  #define FIT_KNIFE_IMPORT
  #if __GNUC__ >= 4
    #define FIT_KNIFE_PUBLIC __attribute__ ((visibility("default")))
    #define FIT_KNIFE_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define FIT_KNIFE_PUBLIC
    #define FIT_KNIFE_LOCAL
  #endif
  #define FIT_KNIFE_PUBLIC_TYPE
#endif

#endif  // FIT_KNIFE__VISIBILITY_CONTROL_H_