#ifndef FIT_CYLINDER__VISIBILITY_CONTROL_H_
#define FIT_CYLINDER__VISIBILITY_CONTROL_H_

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define FIT_CYLINDER_EXPORT __attribute__ ((dllexport))
    #define FIT_CYLINDER_IMPORT __attribute__ ((dllimport))
  #else
    #define FIT_CYLINDER_EXPORT __declspec(dllexport)
    #define FIT_CYLINDER_IMPORT __declspec(dllimport)
  #endif
  #ifdef FIT_CYLINDER_BUILDING_LIBRARY
    #define FIT_CYLINDER_PUBLIC FIT_CYLINDER_EXPORT
  #else
    #define FIT_CYLINDER_PUBLIC FIT_CYLINDER_IMPORT
  #endif
  #define FIT_CYLINDER_PUBLIC_TYPE FIT_CYLINDER_PUBLIC
  #define FIT_CYLINDER_LOCAL
#else
  #define FIT_CYLINDER_EXPORT __attribute__ ((visibility("default")))
  #define FIT_CYLINDER_IMPORT
  #if __GNUC__ >= 4
    #define FIT_CYLINDER_PUBLIC __attribute__ ((visibility("default")))
    #define FIT_CYLINDER_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define FIT_CYLINDER_PUBLIC
    #define FIT_CYLINDER_LOCAL
  #endif
  #define FIT_CYLINDER_PUBLIC_TYPE
#endif

#endif  // FIT_CYLINDER__VISIBILITY_CONTROL_H_
