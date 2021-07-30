#ifndef FIT_SPHERE__VISIBILITY_CONTROL_H_
#define FIT_SPHERE__VISIBILITY_CONTROL_H_

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define FIT_SPHERE_EXPORT __attribute__ ((dllexport))
    #define FIT_SPHERE_IMPORT __attribute__ ((dllimport))
  #else
    #define FIT_SPHERE_EXPORT __declspec(dllexport)
    #define FIT_SPHERE_IMPORT __declspec(dllimport)
  #endif
  #ifdef FIT_SPHERE_BUILDING_LIBRARY
    #define FIT_SPHERE_PUBLIC FIT_SPHERE_EXPORT
  #else
    #define FIT_SPHERE_PUBLIC FIT_SPHERE_IMPORT
  #endif
  #define FIT_SPHERE_PUBLIC_TYPE FIT_SPHERE_PUBLIC
  #define FIT_SPHERE_LOCAL
#else
  #define FIT_SPHERE_EXPORT __attribute__ ((visibility("default")))
  #define FIT_SPHERE_IMPORT
  #if __GNUC__ >= 4
    #define FIT_SPHERE_PUBLIC __attribute__ ((visibility("default")))
    #define FIT_SPHERE_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define FIT_SPHERE_PUBLIC
    #define FIT_SPHERE_LOCAL
  #endif
  #define FIT_SPHERE_PUBLIC_TYPE
#endif

#endif  // FIT_SPHERE__VISIBILITY_CONTROL_H_
