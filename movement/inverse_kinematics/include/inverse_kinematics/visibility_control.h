#ifndef INVERSE_KINEMATICS__VISIBILITY_CONTROL_H_
#define INVERSE_KINEMATICS__VISIBILITY_CONTROL_H_

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define INVERSE_KINEMATICS_EXPORT __attribute__ ((dllexport))
    #define INVERSE_KINEMATICS_IMPORT __attribute__ ((dllimport))
  #else
    #define INVERSE_KINEMATICS_EXPORT __declspec(dllexport)
    #define INVERSE_KINEMATICS_IMPORT __declspec(dllimport)
  #endif
  #ifdef INVERSE_KINEMATICS_BUILDING_LIBRARY
    #define INVERSE_KINEMATICS_PUBLIC INVERSE_KINEMATICS_EXPORT
  #else
    #define INVERSE_KINEMATICS_PUBLIC INVERSE_KINEMATICS_IMPORT
  #endif
  #define INVERSE_KINEMATICS_PUBLIC_TYPE INVERSE_KINEMATICS_PUBLIC
  #define INVERSE_KINEMATICS_LOCAL
#else
  #define INVERSE_KINEMATICS_EXPORT __attribute__ ((visibility("default")))
  #define INVERSE_KINEMATICS_IMPORT
  #if __GNUC__ >= 4
    #define INVERSE_KINEMATICS_PUBLIC __attribute__ ((visibility("default")))
    #define INVERSE_KINEMATICS_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define INVERSE_KINEMATICS_PUBLIC
    #define INVERSE_KINEMATICS_LOCAL
  #endif
  #define INVERSE_KINEMATICS_PUBLIC_TYPE
#endif

#endif  // INVERSE_KINEMATICS__VISIBILITY_CONTROL_H_
