#ifndef BULLET_CLIENT__VISIBILITY_CONTROL_H_
#define BULLET_CLIENT__VISIBILITY_CONTROL_H_

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define BULLET_CLIENT_EXPORT __attribute__ ((dllexport))
    #define BULLET_CLIENT_IMPORT __attribute__ ((dllimport))
  #else
    #define BULLET_CLIENT_EXPORT __declspec(dllexport)
    #define BULLET_CLIENT_IMPORT __declspec(dllimport)
  #endif
  #ifdef BULLET_CLIENT_BUILDING_LIBRARY
    #define BULLET_CLIENT_PUBLIC BULLET_CLIENT_EXPORT
  #else
    #define BULLET_CLIENT_PUBLIC BULLET_CLIENT_IMPORT
  #endif
  #define BULLET_CLIENT_PUBLIC_TYPE BULLET_CLIENT_PUBLIC
  #define BULLET_CLIENT_LOCAL
#else
  #define BULLET_CLIENT_EXPORT __attribute__ ((visibility("default")))
  #define BULLET_CLIENT_IMPORT
  #if __GNUC__ >= 4
    #define BULLET_CLIENT_PUBLIC __attribute__ ((visibility("default")))
    #define BULLET_CLIENT_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define BULLET_CLIENT_PUBLIC
    #define BULLET_CLIENT_LOCAL
  #endif
  #define BULLET_CLIENT_PUBLIC_TYPE
#endif

#endif  // BULLET_CLIENT__VISIBILITY_CONTROL_H_
