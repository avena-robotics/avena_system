#ifndef DUMMY_ARM_CONTROLLER__VISIBILITY_CONTROL_H_
#define DUMMY_ARM_CONTROLLER__VISIBILITY_CONTROL_H_

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define DUMMY_ARM_CONTROLLER_EXPORT __attribute__ ((dllexport))
    #define DUMMY_ARM_CONTROLLER_IMPORT __attribute__ ((dllimport))
  #else
    #define DUMMY_ARM_CONTROLLER_EXPORT __declspec(dllexport)
    #define DUMMY_ARM_CONTROLLER_IMPORT __declspec(dllimport)
  #endif
  #ifdef DUMMY_ARM_CONTROLLER_BUILDING_LIBRARY
    #define DUMMY_ARM_CONTROLLER_PUBLIC DUMMY_ARM_CONTROLLER_EXPORT
  #else
    #define DUMMY_ARM_CONTROLLER_PUBLIC DUMMY_ARM_CONTROLLER_IMPORT
  #endif
  #define DUMMY_ARM_CONTROLLER_PUBLIC_TYPE DUMMY_ARM_CONTROLLER_PUBLIC
  #define DUMMY_ARM_CONTROLLER_LOCAL
#else
  #define DUMMY_ARM_CONTROLLER_EXPORT __attribute__ ((visibility("default")))
  #define DUMMY_ARM_CONTROLLER_IMPORT
  #if __GNUC__ >= 4
    #define DUMMY_ARM_CONTROLLER_PUBLIC __attribute__ ((visibility("default")))
    #define DUMMY_ARM_CONTROLLER_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define DUMMY_ARM_CONTROLLER_PUBLIC
    #define DUMMY_ARM_CONTROLLER_LOCAL
  #endif
  #define DUMMY_ARM_CONTROLLER_PUBLIC_TYPE
#endif

#endif  // DUMMY_ARM_CONTROLLER__VISIBILITY_CONTROL_H_
