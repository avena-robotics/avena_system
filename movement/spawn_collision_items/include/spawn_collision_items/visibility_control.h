#ifndef SPAWN_COLLISION_ITEMS__VISIBILITY_CONTROL_H_
#define SPAWN_COLLISION_ITEMS__VISIBILITY_CONTROL_H_

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define SPAWN_COLLISION_ITEMS_EXPORT __attribute__ ((dllexport))
    #define SPAWN_COLLISION_ITEMS_IMPORT __attribute__ ((dllimport))
  #else
    #define SPAWN_COLLISION_ITEMS_EXPORT __declspec(dllexport)
    #define SPAWN_COLLISION_ITEMS_IMPORT __declspec(dllimport)
  #endif
  #ifdef SPAWN_COLLISION_ITEMS_BUILDING_LIBRARY
    #define SPAWN_COLLISION_ITEMS_PUBLIC SPAWN_COLLISION_ITEMS_EXPORT
  #else
    #define SPAWN_COLLISION_ITEMS_PUBLIC SPAWN_COLLISION_ITEMS_IMPORT
  #endif
  #define SPAWN_COLLISION_ITEMS_PUBLIC_TYPE SPAWN_COLLISION_ITEMS_PUBLIC
  #define SPAWN_COLLISION_ITEMS_LOCAL
#else
  #define SPAWN_COLLISION_ITEMS_EXPORT __attribute__ ((visibility("default")))
  #define SPAWN_COLLISION_ITEMS_IMPORT
  #if __GNUC__ >= 4
    #define SPAWN_COLLISION_ITEMS_PUBLIC __attribute__ ((visibility("default")))
    #define SPAWN_COLLISION_ITEMS_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define SPAWN_COLLISION_ITEMS_PUBLIC
    #define SPAWN_COLLISION_ITEMS_LOCAL
  #endif
  #define SPAWN_COLLISION_ITEMS_PUBLIC_TYPE
#endif

#endif  // SPAWN_COLLISION_ITEMS__VISIBILITY_CONTROL_H_
