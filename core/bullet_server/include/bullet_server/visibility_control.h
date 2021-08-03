#ifndef BULLET_SERVER__VISIBILITY_CONTROL_H_
#define BULLET_SERVER__VISIBILITY_CONTROL_H_

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define BULLET_SERVER_EXPORT __attribute__ ((dllexport))
    #define BULLET_SERVER_IMPORT __attribute__ ((dllimport))
  #else
    #define BULLET_SERVER_EXPORT __declspec(dllexport)
    #define BULLET_SERVER_IMPORT __declspec(dllimport)
  #endif
  #ifdef BULLET_SERVER_BUILDING_LIBRARY
    #define BULLET_SERVER_PUBLIC BULLET_SERVER_EXPORT
  #else
    #define BULLET_SERVER_PUBLIC BULLET_SERVER_IMPORT
  #endif
  #define BULLET_SERVER_PUBLIC_TYPE BULLET_SERVER_PUBLIC
  #define BULLET_SERVER_LOCAL
#else
  #define BULLET_SERVER_EXPORT __attribute__ ((visibility("default")))
  #define BULLET_SERVER_IMPORT
  #if __GNUC__ >= 4
    #define BULLET_SERVER_PUBLIC __attribute__ ((visibility("default")))
    #define BULLET_SERVER_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define BULLET_SERVER_PUBLIC
    #define BULLET_SERVER_LOCAL
  #endif
  #define BULLET_SERVER_PUBLIC_TYPE
#endif

#endif  // BULLET_SERVER__VISIBILITY_CONTROL_H_
