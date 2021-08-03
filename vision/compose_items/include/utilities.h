#ifndef VS_UTILITIES_H
#define VS_UTILITIES_H

#include <cmath>
#include <chrono>
#include <utility>

typedef std::chrono::system_clock::time_point TimeVar;

#define duration(a) std::chrono::duration_cast<std::chrono::milliseconds>(a).count()
#define timeNow() std::chrono::high_resolution_clock::now()

/**
 *
 * @tparam F -> template function type
 * @tparam Args -> template arguments type
 * @param func -> pointer to function that will be measured
 * @return high resolution timestamp from start to end of execution
 */
template<typename F>
double funcTime(F func){
    TimeVar t1=timeNow();
    func();
    return duration(timeNow()-t1);
}

#endif //VS_UTILITIES_H
