#ifndef GENERATE_TRAJECTORY_VISIBILITY_CONTROL_H_
#define GENERATE_TRAJECTORY_VISIBILITY_CONTROL_H_

#ifdef __cplusplus
extern "C"
{
#endif

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define GENERATE_TRAJECTORY_EXPORT __attribute__ ((dllexport))
    #define GENERATE_TRAJECTORY_IMPORT __attribute__ ((dllimport))
  #else
    #define GENERATE_TRAJECTORY_EXPORT __declspec(dllexport)
    #define GENERATE_TRAJECTORY_IMPORT __declspec(dllimport)
  #endif
  #ifdef GENERATE_TRAJECTORY_BUILDING_DLL
    #define GENERATE_TRAJECTORY_PUBLIC GENERATE_TRAJECTORY_EXPORT
  #else
    #define GENERATE_TRAJECTORY_PUBLIC GENERATE_TRAJECTORY_IMPORT
  #endif
  #define GENERATE_TRAJECTORY_PUBLIC_TYPE GENERATE_TRAJECTORY_PUBLIC
  #define GENERATE_TRAJECTORY_LOCAL
#else
  #define GENERATE_TRAJECTORY_EXPORT __attribute__ ((visibility("default")))
  #define GENERATE_TRAJECTORY_IMPORT
  #if __GNUC__ >= 4
    #define GENERATE_TRAJECTORY_PUBLIC __attribute__ ((visibility("default")))
    #define GENERATE_TRAJECTORY_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define GENERATE_TRAJECTORY_PUBLIC
    #define GENERATE_TRAJECTORY_LOCAL
  #endif
  #define GENERATE_TRAJECTORY_PUBLIC_TYPE
#endif

#ifdef __cplusplus
}
#endif

#endif  // GENERATE_TRAJECTORY_VISIBILITY_CONTROL_H_
