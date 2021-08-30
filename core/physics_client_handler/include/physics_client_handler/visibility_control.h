#ifndef PHYSICS_CLIENT_HANDLER__VISIBILITY_CONTROL_H_
#define PHYSICS_CLIENT_HANDLER__VISIBILITY_CONTROL_H_

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define PHYSICS_CLIENT_HANDLER_EXPORT __attribute__ ((dllexport))
    #define PHYSICS_CLIENT_HANDLER_IMPORT __attribute__ ((dllimport))
  #else
    #define PHYSICS_CLIENT_HANDLER_EXPORT __declspec(dllexport)
    #define PHYSICS_CLIENT_HANDLER_IMPORT __declspec(dllimport)
  #endif
  #ifdef PHYSICS_CLIENT_HANDLER_BUILDING_LIBRARY
    #define PHYSICS_CLIENT_HANDLER_PUBLIC PHYSICS_CLIENT_HANDLER_EXPORT
  #else
    #define PHYSICS_CLIENT_HANDLER_PUBLIC PHYSICS_CLIENT_HANDLER_IMPORT
  #endif
  #define PHYSICS_CLIENT_HANDLER_PUBLIC_TYPE PHYSICS_CLIENT_HANDLER_PUBLIC
  #define PHYSICS_CLIENT_HANDLER_LOCAL
#else
  #define PHYSICS_CLIENT_HANDLER_EXPORT __attribute__ ((visibility("default")))
  #define PHYSICS_CLIENT_HANDLER_IMPORT
  #if __GNUC__ >= 4
    #define PHYSICS_CLIENT_HANDLER_PUBLIC __attribute__ ((visibility("default")))
    #define PHYSICS_CLIENT_HANDLER_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define PHYSICS_CLIENT_HANDLER_PUBLIC
    #define PHYSICS_CLIENT_HANDLER_LOCAL
  #endif
  #define PHYSICS_CLIENT_HANDLER_PUBLIC_TYPE
#endif

#endif  // PHYSICS_CLIENT_HANDLER__VISIBILITY_CONTROL_H_
