#ifndef FIT_BOX__VISIBILITY_CONTROL_H_
#define FIT_BOX__VISIBILITY_CONTROL_H_

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define FIT_BOX_EXPORT __attribute__ ((dllexport))
    #define FIT_BOX_IMPORT __attribute__ ((dllimport))
  #else
    #define FIT_BOX_EXPORT __declspec(dllexport)
    #define FIT_BOX_IMPORT __declspec(dllimport)
  #endif
  #ifdef FIT_BOX_BUILDING_LIBRARY
    #define FIT_BOX_PUBLIC FIT_BOX_EXPORT
  #else
    #define FIT_BOX_PUBLIC FIT_BOX_IMPORT
  #endif
  #define FIT_BOX_PUBLIC_TYPE FIT_BOX_PUBLIC
  #define FIT_BOX_LOCAL
#else
  #define FIT_BOX_EXPORT __attribute__ ((visibility("default")))
  #define FIT_BOX_IMPORT
  #if __GNUC__ >= 4
    #define FIT_BOX_PUBLIC __attribute__ ((visibility("default")))
    #define FIT_BOX_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define FIT_BOX_PUBLIC
    #define FIT_BOX_LOCAL
  #endif
  #define FIT_BOX_PUBLIC_TYPE
#endif

#endif  // FIT_BOX__VISIBILITY_CONTROL_H_
