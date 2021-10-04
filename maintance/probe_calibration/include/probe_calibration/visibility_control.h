#ifndef PROBE_CALIBRATION__VISIBILITY_CONTROL_H_
#define PROBE_CALIBRATION__VISIBILITY_CONTROL_H_

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define PROBE_CALIBRATION_EXPORT __attribute__ ((dllexport))
    #define PROBE_CALIBRATION_IMPORT __attribute__ ((dllimport))
  #else
    #define PROBE_CALIBRATION_EXPORT __declspec(dllexport)
    #define PROBE_CALIBRATION_IMPORT __declspec(dllimport)
  #endif
  #ifdef PROBE_CALIBRATION_BUILDING_LIBRARY
    #define PROBE_CALIBRATION_PUBLIC PROBE_CALIBRATION_EXPORT
  #else
    #define PROBE_CALIBRATION_PUBLIC PROBE_CALIBRATION_IMPORT
  #endif
  #define PROBE_CALIBRATION_PUBLIC_TYPE PROBE_CALIBRATION_PUBLIC
  #define PROBE_CALIBRATION_LOCAL
#else
  #define PROBE_CALIBRATION_EXPORT __attribute__ ((visibility("default")))
  #define PROBE_CALIBRATION_IMPORT
  #if __GNUC__ >= 4
    #define PROBE_CALIBRATION_PUBLIC __attribute__ ((visibility("default")))
    #define PROBE_CALIBRATION_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define PROBE_CALIBRATION_PUBLIC
    #define PROBE_CALIBRATION_LOCAL
  #endif
  #define PROBE_CALIBRATION_PUBLIC_TYPE
#endif

#endif  // PROBE_CALIBRATION__VISIBILITY_CONTROL_H_
