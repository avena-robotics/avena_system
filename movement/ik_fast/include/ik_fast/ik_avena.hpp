/// autogenerated analytical inverse kinematics code from ikfast program part of OpenRAVE
/// \author Rosen Diankov
///
/// Licensed under the Apache License, Version 2.0 (the "License");
/// you may not use this file except in compliance with the License.
/// You may obtain a copy of the License at
///     http://www.apache.org/licenses/LICENSE-2.0
///
/// Unless required by applicable law or agreed to in writing, software
/// distributed under the License is distributed on an "AS IS" BASIS,
/// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
/// See the License for the specific language governing permissions and
/// limitations under the License.
///
/// ikfast version 0x1000004b generated on 2021-08-25 14:28:29.768267
/// Generated using solver transform6d
/// To compile with gcc:
///     gcc -lstdc++ ik.cpp
/// To compile without any main function as a shared object (might need -llapack):
///     gcc -fPIC -lstdc++ -DIKFAST_NO_MAIN -DIKFAST_CLIBRARY -shared -Wl,-soname,libik.so -o libik.so ik.cpp
#ifndef IK_FAST__IK_AVENA_HPP_
#define IK_FAST__IK_AVENA_HPP_

#define IKFAST_HAS_LIBRARY
#include "ikfast.h" // found inside share/openrave-X.Y/python/ikfast.h
using namespace ikfast;

#define IKFAST_NAMESPACE_AVENA ik_avena

// check if the included ikfast version matches what this file was compiled with
#define IKFAST_COMPILE_ASSERT(x) extern int __dummy[(int)x]
IKFAST_COMPILE_ASSERT(IKFAST_VERSION == 0x1000004b);

#include <cmath>
#include <vector>
#include <limits>
#include <algorithm>
#include <complex>

#ifndef IKFAST_ASSERT
#include <stdexcept>
#include <sstream>
#include <iostream>

#ifdef _MSC_VER
#ifndef __PRETTY_FUNCTION__
#define __PRETTY_FUNCTION__ __FUNCDNAME__
#endif
#endif

#ifndef __PRETTY_FUNCTION__
#define __PRETTY_FUNCTION__ __func__
#endif

#define IKFAST_ASSERT(b)                                                                                                                     \
    {                                                                                                                                        \
        if (!(b))                                                                                                                            \
        {                                                                                                                                    \
            std::stringstream ss;                                                                                                            \
            ss << "ikfast exception: " << __FILE__ << ":" << __LINE__ << ": " << __PRETTY_FUNCTION__ << ": Assertion '" << #b << "' failed"; \
            throw std::runtime_error(ss.str());                                                                                              \
        }                                                                                                                                    \
    }

#endif

#if defined(_MSC_VER)
#define IKFAST_ALIGNED16(x) __declspec(align(16)) x
#else
#define IKFAST_ALIGNED16(x) x __attribute((aligned(16)))
#endif

#define IK2PI ((IkReal)6.28318530717959)
#define IKPI ((IkReal)3.14159265358979)
#define IKPI_2 ((IkReal)1.57079632679490)

#ifdef _MSC_VER
#ifndef isnan
#define isnan _isnan
#endif
#ifndef isinf
#define isinf _isinf
#endif
//#ifndef isfinite
//#define isfinite _isfinite
//#endif
#endif // _MSC_VER

// lapack routines
extern "C"
{
    void dgetrf_(const int *m, const int *n, double *a, const int *lda, int *ipiv, int *info);
    void zgetrf_(const int *m, const int *n, std::complex<double> *a, const int *lda, int *ipiv, int *info);
    void dgetri_(const int *n, const double *a, const int *lda, int *ipiv, double *work, const int *lwork, int *info);
    void dgesv_(const int *n, const int *nrhs, double *a, const int *lda, int *ipiv, double *b, const int *ldb, int *info);
    void dgetrs_(const char *trans, const int *n, const int *nrhs, double *a, const int *lda, int *ipiv, double *b, const int *ldb, int *info);
    void dgeev_(const char *jobvl, const char *jobvr, const int *n, double *a, const int *lda, double *wr, double *wi, double *vl, const int *ldvl, double *vr, const int *ldvr, double *work, const int *lwork, int *info);
}

using namespace std; // necessary to get std math routines

#ifdef IKFAST_NAMESPACE_AVENA
namespace IKFAST_NAMESPACE_AVENA
{
#endif

    inline float IKabs(float f)
    {
        return fabsf(f);
    }
    inline double IKabs(double f) { return fabs(f); }

    inline float IKsqr(float f) { return f * f; }
    inline double IKsqr(double f) { return f * f; }

    inline float IKlog(float f) { return logf(f); }
    inline double IKlog(double f) { return log(f); }

// allows asin and acos to exceed 1. has to be smaller than thresholds used for branch conds and evaluation
#ifndef IKFAST_SINCOS_THRESH
#define IKFAST_SINCOS_THRESH ((IkReal)1e-7)
#endif

// used to check input to atan2 for degenerate cases. has to be smaller than thresholds used for branch conds and evaluation
#ifndef IKFAST_ATAN2_MAGTHRESH
#define IKFAST_ATAN2_MAGTHRESH ((IkReal)1e-7)
#endif

// minimum distance of separate solutions
#ifndef IKFAST_SOLUTION_THRESH
#define IKFAST_SOLUTION_THRESH ((IkReal)1e-6)
#endif

// there are checkpoints in ikfast that are evaluated to make sure they are 0. This threshold speicfies by how much they can deviate
#ifndef IKFAST_EVALCOND_THRESH
#define IKFAST_EVALCOND_THRESH ((IkReal)0.03) // 5D IK has some crazy degenerate cases, but can rely on jacobian refinment to make better, just need good starting point
#endif

    inline float IKasin(float f)
    {
        IKFAST_ASSERT(f > -1 - IKFAST_SINCOS_THRESH && f < 1 + IKFAST_SINCOS_THRESH); // any more error implies something is wrong with the solver
        if (f <= -1)
            return float(-IKPI_2);
        else if (f >= 1)
            return float(IKPI_2);
        return asinf(f);
    }
    inline double IKasin(double f)
    {
        IKFAST_ASSERT(f > -1 - IKFAST_SINCOS_THRESH && f < 1 + IKFAST_SINCOS_THRESH); // any more error implies something is wrong with the solver
        if (f <= -1)
            return -IKPI_2;
        else if (f >= 1)
            return IKPI_2;
        return asin(f);
    }

    // return positive value in [0,y)
    inline float IKfmod(float x, float y)
    {
        while (x < 0)
        {
            x += y;
        }
        return fmodf(x, y);
    }

    // return positive value in [0,y)
    inline double IKfmod(double x, double y)
    {
        while (x < 0)
        {
            x += y;
        }
        return fmod(x, y);
    }

    inline float IKacos(float f)
    {
        IKFAST_ASSERT(f > -1 - IKFAST_SINCOS_THRESH && f < 1 + IKFAST_SINCOS_THRESH); // any more error implies something is wrong with the solver
        if (f <= -1)
            return float(IKPI);
        else if (f >= 1)
            return float(0);
        return acosf(f);
    }
    inline double IKacos(double f)
    {
        IKFAST_ASSERT(f > -1 - IKFAST_SINCOS_THRESH && f < 1 + IKFAST_SINCOS_THRESH); // any more error implies something is wrong with the solver
        if (f <= -1)
            return IKPI;
        else if (f >= 1)
            return 0;
        return acos(f);
    }
    inline float IKsin(float f) { return sinf(f); }
    inline double IKsin(double f) { return sin(f); }
    inline float IKcos(float f) { return cosf(f); }
    inline double IKcos(double f) { return cos(f); }
    inline float IKtan(float f) { return tanf(f); }
    inline double IKtan(double f) { return tan(f); }
    inline float IKsqrt(float f)
    {
        if (f <= 0.0f)
            return 0.0f;
        return sqrtf(f);
    }
    inline double IKsqrt(double f)
    {
        if (f <= 0.0)
            return 0.0;
        return sqrt(f);
    }
    inline float IKatan2Simple(float fy, float fx)
    {
        return atan2f(fy, fx);
    }
    inline float IKatan2(float fy, float fx)
    {
        if (isnan(fy))
        {
            IKFAST_ASSERT(!isnan(fx)); // if both are nan, probably wrong value will be returned
            return float(IKPI_2);
        }
        else if (isnan(fx))
        {
            return 0;
        }
        return atan2f(fy, fx);
    }
    inline double IKatan2Simple(double fy, double fx)
    {
        return atan2(fy, fx);
    }
    inline double IKatan2(double fy, double fx)
    {
        if (isnan(fy))
        {
            IKFAST_ASSERT(!isnan(fx)); // if both are nan, probably wrong value will be returned
            return IKPI_2;
        }
        else if (isnan(fx))
        {
            return 0;
        }
        return atan2(fy, fx);
    }

    template <typename T>
    struct CheckValue
    {
        T value;
        bool valid;
    };

    template <typename T>
    inline CheckValue<T> IKatan2WithCheck(T fy, T fx, T epsilon)
    {
        CheckValue<T> ret;
        ret.valid = false;
        ret.value = 0;
        if (!isnan(fy) && !isnan(fx))
        {
            if (IKabs(fy) >= IKFAST_ATAN2_MAGTHRESH || IKabs(fx) > IKFAST_ATAN2_MAGTHRESH)
            {
                ret.value = IKatan2Simple(fy, fx);
                ret.valid = true;
            }
        }
        return ret;
    }

    inline float IKsign(float f)
    {
        if (f > 0)
        {
            return float(1);
        }
        else if (f < 0)
        {
            return float(-1);
        }
        return 0;
    }

    inline double IKsign(double f)
    {
        if (f > 0)
        {
            return 1.0;
        }
        else if (f < 0)
        {
            return -1.0;
        }
        return 0;
    }

    template <typename T>
    inline CheckValue<T> IKPowWithIntegerCheck(T f, int n)
    {
        CheckValue<T> ret;
        ret.valid = true;
        if (n == 0)
        {
            ret.value = 1.0;
            return ret;
        }
        else if (n == 1)
        {
            ret.value = f;
            return ret;
        }
        else if (n < 0)
        {
            if (f == 0)
            {
                ret.valid = false;
                ret.value = (T)1.0e30;
                return ret;
            }
            if (n == -1)
            {
                ret.value = T(1.0) / f;
                return ret;
            }
        }

        int num = n > 0 ? n : -n;
        if (num == 2)
        {
            ret.value = f * f;
        }
        else if (num == 3)
        {
            ret.value = f * f * f;
        }
        else
        {
            ret.value = 1.0;
            while (num > 0)
            {
                if (num & 1)
                {
                    ret.value *= f;
                }
                num >>= 1;
                f *= f;
            }
        }

        if (n < 0)
        {
            ret.value = T(1.0) / ret.value;
        }
        return ret;
    }

    template <typename T>
    struct ComplexLess
    {
        bool operator()(const complex<T> &lhs, const complex<T> &rhs) const
        {
            if (real(lhs) < real(rhs))
            {
                return true;
            }
            if (real(lhs) > real(rhs))
            {
                return false;
            }
            return imag(lhs) < imag(rhs);
        }
    };

    /// solves the forward kinematics equations.
    /// \param pfree is an array specifying the free joints of the chain.
    IKFAST_API void ComputeFk(const IkReal *j, IkReal *eetrans, IkReal *eerot);
    IKFAST_API int GetNumFreeParameters();
    IKFAST_API const int *GetFreeIndices();
    IKFAST_API int GetNumJoints();
    IKFAST_API int GetIkRealSize();
    IKFAST_API int GetIkType();

    class IKSolver
    {
    public:
        IkReal j0, cj0, sj0, htj0, j0mul, j1, cj1, sj1, htj1, j1mul, j2, cj2, sj2, htj2, j2mul, j3, cj3, sj3, htj3, j3mul, j4, cj4, sj4, htj4, j4mul, j5, cj5, sj5, htj5, j5mul, new_r00, r00, rxp0_0, new_r01, r01, rxp0_1, new_r02, r02, rxp0_2, new_r10, r10, rxp1_0, new_r11, r11, rxp1_1, new_r12, r12, rxp1_2, new_r20, r20, rxp2_0, new_r21, r21, rxp2_1, new_r22, r22, rxp2_2, new_px, px, npx, new_py, py, npy, new_pz, pz, npz, pp;
        unsigned char _ij0[2], _nj0, _ij1[2], _nj1, _ij2[2], _nj2, _ij3[2], _nj3, _ij4[2], _nj4, _ij5[2], _nj5;

        IkReal j100, cj100, sj100;
        unsigned char _ij100[2], _nj100;
        bool ComputeIk(const IkReal *eetrans, const IkReal *eerot, const IkReal *pfree, IkSolutionListBase<IkReal> &solutions);
        static inline void polyroots3(IkReal rawcoeffs[3 + 1], IkReal rawroots[3], int &numroots)
        {
            using std::complex;
            if (rawcoeffs[0] == 0)
            {
                // solve with one reduced degree
                polyroots2(&rawcoeffs[1], &rawroots[0], numroots);
                return;
            }
            IKFAST_ASSERT(rawcoeffs[0] != 0);
            const IkReal tol = 128.0 * std::numeric_limits<IkReal>::epsilon();
            const IkReal tolsqrt = sqrt(std::numeric_limits<IkReal>::epsilon());
            complex<IkReal> coeffs[3];
            const int maxsteps = 110;
            for (int i = 0; i < 3; ++i)
            {
                coeffs[i] = complex<IkReal>(rawcoeffs[i + 1] / rawcoeffs[0]);
            }
            complex<IkReal> roots[3];
            IkReal err[3];
            roots[0] = complex<IkReal>(1, 0);
            roots[1] = complex<IkReal>(0.4, 0.9); // any complex number not a root of unity works
            err[0] = 1.0;
            err[1] = 1.0;
            for (int i = 2; i < 3; ++i)
            {
                roots[i] = roots[i - 1] * roots[1];
                err[i] = 1.0;
            }
            for (int step = 0; step < maxsteps; ++step)
            {
                bool changed = false;
                for (int i = 0; i < 3; ++i)
                {
                    if (err[i] >= tol)
                    {
                        changed = true;
                        // evaluate
                        complex<IkReal> x = roots[i] + coeffs[0];
                        for (int j = 1; j < 3; ++j)
                        {
                            x = roots[i] * x + coeffs[j];
                        }
                        for (int j = 0; j < 3; ++j)
                        {
                            if (i != j)
                            {
                                if (roots[i] != roots[j])
                                {
                                    x /= (roots[i] - roots[j]);
                                }
                            }
                        }
                        roots[i] -= x;
                        err[i] = abs(x);
                    }
                }
                if (!changed)
                {
                    break;
                }
            }

            // sort roots hoping that it solution indices become more robust to slight change in coeffs
            std::sort(roots, roots + 3, ComplexLess<IkReal>());

            numroots = 0;
            bool visited[3] = {false};
            for (int i = 0; i < 3; ++i)
            {
                if (!visited[i])
                {
                    // might be a multiple root, in which case it will have more error than the other roots
                    // find any neighboring roots, and take the average
                    complex<IkReal> newroot = roots[i];
                    int n = 1;
                    for (int j = i + 1; j < 3; ++j)
                    {
                        // care about error in real much more than imaginary
                        if (abs(real(roots[i]) - real(roots[j])) < tolsqrt && (abs(imag(roots[i]) - imag(roots[j])) < 0.002 || abs(imag(roots[i]) + imag(roots[j])) < 0.002) && abs(imag(roots[i])) < 0.002)
                        {
                            newroot += roots[j];
                            n += 1;
                            visited[j] = true;
                        }
                    }
                    if (n > 1)
                    {
                        newroot /= n;
                    }
                    // there are still cases where even the mean is not accurate enough, until a better multi-root algorithm is used, need to use the sqrt
                    if (IKabs(imag(newroot)) < tolsqrt)
                    {
                        rawroots[numroots++] = real(newroot);
                    }
                }
            }
        }
        static inline void polyroots2(IkReal rawcoeffs[2 + 1], IkReal rawroots[2], int &numroots)
        {
            IkReal det = rawcoeffs[1] * rawcoeffs[1] - 4 * rawcoeffs[0] * rawcoeffs[2];
            if (det < 0)
            {
                numroots = 0;
            }
            else if (det == 0)
            {
                rawroots[0] = -0.5 * rawcoeffs[1] / rawcoeffs[0];
                numroots = 1;
            }
            else
            {
                det = IKsqrt(det);
                rawroots[0] = (-rawcoeffs[1] + det) / (2 * rawcoeffs[0]);
                rawroots[1] = (-rawcoeffs[1] - det) / (2 * rawcoeffs[0]); //rawcoeffs[2]/(rawcoeffs[0]*rawroots[0]);
                numroots = 2;
            }
        }
        static inline void polyroots5(IkReal rawcoeffs[5 + 1], IkReal rawroots[5], int &numroots)
        {
            using std::complex;
            if (rawcoeffs[0] == 0)
            {
                // solve with one reduced degree
                polyroots4(&rawcoeffs[1], &rawroots[0], numroots);
                return;
            }
            IKFAST_ASSERT(rawcoeffs[0] != 0);
            const IkReal tol = 128.0 * std::numeric_limits<IkReal>::epsilon();
            const IkReal tolsqrt = sqrt(std::numeric_limits<IkReal>::epsilon());
            complex<IkReal> coeffs[5];
            const int maxsteps = 110;
            for (int i = 0; i < 5; ++i)
            {
                coeffs[i] = complex<IkReal>(rawcoeffs[i + 1] / rawcoeffs[0]);
            }
            complex<IkReal> roots[5];
            IkReal err[5];
            roots[0] = complex<IkReal>(1, 0);
            roots[1] = complex<IkReal>(0.4, 0.9); // any complex number not a root of unity works
            err[0] = 1.0;
            err[1] = 1.0;
            for (int i = 2; i < 5; ++i)
            {
                roots[i] = roots[i - 1] * roots[1];
                err[i] = 1.0;
            }
            for (int step = 0; step < maxsteps; ++step)
            {
                bool changed = false;
                for (int i = 0; i < 5; ++i)
                {
                    if (err[i] >= tol)
                    {
                        changed = true;
                        // evaluate
                        complex<IkReal> x = roots[i] + coeffs[0];
                        for (int j = 1; j < 5; ++j)
                        {
                            x = roots[i] * x + coeffs[j];
                        }
                        for (int j = 0; j < 5; ++j)
                        {
                            if (i != j)
                            {
                                if (roots[i] != roots[j])
                                {
                                    x /= (roots[i] - roots[j]);
                                }
                            }
                        }
                        roots[i] -= x;
                        err[i] = abs(x);
                    }
                }
                if (!changed)
                {
                    break;
                }
            }

            // sort roots hoping that it solution indices become more robust to slight change in coeffs
            std::sort(roots, roots + 5, ComplexLess<IkReal>());

            numroots = 0;
            bool visited[5] = {false};
            for (int i = 0; i < 5; ++i)
            {
                if (!visited[i])
                {
                    // might be a multiple root, in which case it will have more error than the other roots
                    // find any neighboring roots, and take the average
                    complex<IkReal> newroot = roots[i];
                    int n = 1;
                    for (int j = i + 1; j < 5; ++j)
                    {
                        // care about error in real much more than imaginary
                        if (abs(real(roots[i]) - real(roots[j])) < tolsqrt && (abs(imag(roots[i]) - imag(roots[j])) < 0.002 || abs(imag(roots[i]) + imag(roots[j])) < 0.002) && abs(imag(roots[i])) < 0.002)
                        {
                            newroot += roots[j];
                            n += 1;
                            visited[j] = true;
                        }
                    }
                    if (n > 1)
                    {
                        newroot /= n;
                    }
                    // there are still cases where even the mean is not accurate enough, until a better multi-root algorithm is used, need to use the sqrt
                    if (IKabs(imag(newroot)) < tolsqrt)
                    {
                        rawroots[numroots++] = real(newroot);
                    }
                }
            }
        }
        static inline void polyroots4(IkReal rawcoeffs[4 + 1], IkReal rawroots[4], int &numroots)
        {
            using std::complex;
            if (rawcoeffs[0] == 0)
            {
                // solve with one reduced degree
                polyroots3(&rawcoeffs[1], &rawroots[0], numroots);
                return;
            }
            IKFAST_ASSERT(rawcoeffs[0] != 0);
            const IkReal tol = 128.0 * std::numeric_limits<IkReal>::epsilon();
            const IkReal tolsqrt = sqrt(std::numeric_limits<IkReal>::epsilon());
            complex<IkReal> coeffs[4];
            const int maxsteps = 110;
            for (int i = 0; i < 4; ++i)
            {
                coeffs[i] = complex<IkReal>(rawcoeffs[i + 1] / rawcoeffs[0]);
            }
            complex<IkReal> roots[4];
            IkReal err[4];
            roots[0] = complex<IkReal>(1, 0);
            roots[1] = complex<IkReal>(0.4, 0.9); // any complex number not a root of unity works
            err[0] = 1.0;
            err[1] = 1.0;
            for (int i = 2; i < 4; ++i)
            {
                roots[i] = roots[i - 1] * roots[1];
                err[i] = 1.0;
            }
            for (int step = 0; step < maxsteps; ++step)
            {
                bool changed = false;
                for (int i = 0; i < 4; ++i)
                {
                    if (err[i] >= tol)
                    {
                        changed = true;
                        // evaluate
                        complex<IkReal> x = roots[i] + coeffs[0];
                        for (int j = 1; j < 4; ++j)
                        {
                            x = roots[i] * x + coeffs[j];
                        }
                        for (int j = 0; j < 4; ++j)
                        {
                            if (i != j)
                            {
                                if (roots[i] != roots[j])
                                {
                                    x /= (roots[i] - roots[j]);
                                }
                            }
                        }
                        roots[i] -= x;
                        err[i] = abs(x);
                    }
                }
                if (!changed)
                {
                    break;
                }
            }

            // sort roots hoping that it solution indices become more robust to slight change in coeffs
            std::sort(roots, roots + 4, ComplexLess<IkReal>());

            numroots = 0;
            bool visited[4] = {false};
            for (int i = 0; i < 4; ++i)
            {
                if (!visited[i])
                {
                    // might be a multiple root, in which case it will have more error than the other roots
                    // find any neighboring roots, and take the average
                    complex<IkReal> newroot = roots[i];
                    int n = 1;
                    for (int j = i + 1; j < 4; ++j)
                    {
                        // care about error in real much more than imaginary
                        if (abs(real(roots[i]) - real(roots[j])) < tolsqrt && (abs(imag(roots[i]) - imag(roots[j])) < 0.002 || abs(imag(roots[i]) + imag(roots[j])) < 0.002) && abs(imag(roots[i])) < 0.002)
                        {
                            newroot += roots[j];
                            n += 1;
                            visited[j] = true;
                        }
                    }
                    if (n > 1)
                    {
                        newroot /= n;
                    }
                    // there are still cases where even the mean is not accurate enough, until a better multi-root algorithm is used, need to use the sqrt
                    if (IKabs(imag(newroot)) < tolsqrt)
                    {
                        rawroots[numroots++] = real(newroot);
                    }
                }
            }
        }
        static inline void polyroots7(IkReal rawcoeffs[7 + 1], IkReal rawroots[7], int &numroots)
        {
            using std::complex;
            if (rawcoeffs[0] == 0)
            {
                // solve with one reduced degree
                polyroots6(&rawcoeffs[1], &rawroots[0], numroots);
                return;
            }
            IKFAST_ASSERT(rawcoeffs[0] != 0);
            const IkReal tol = 128.0 * std::numeric_limits<IkReal>::epsilon();
            const IkReal tolsqrt = sqrt(std::numeric_limits<IkReal>::epsilon());
            complex<IkReal> coeffs[7];
            const int maxsteps = 110;
            for (int i = 0; i < 7; ++i)
            {
                coeffs[i] = complex<IkReal>(rawcoeffs[i + 1] / rawcoeffs[0]);
            }
            complex<IkReal> roots[7];
            IkReal err[7];
            roots[0] = complex<IkReal>(1, 0);
            roots[1] = complex<IkReal>(0.4, 0.9); // any complex number not a root of unity works
            err[0] = 1.0;
            err[1] = 1.0;
            for (int i = 2; i < 7; ++i)
            {
                roots[i] = roots[i - 1] * roots[1];
                err[i] = 1.0;
            }
            for (int step = 0; step < maxsteps; ++step)
            {
                bool changed = false;
                for (int i = 0; i < 7; ++i)
                {
                    if (err[i] >= tol)
                    {
                        changed = true;
                        // evaluate
                        complex<IkReal> x = roots[i] + coeffs[0];
                        for (int j = 1; j < 7; ++j)
                        {
                            x = roots[i] * x + coeffs[j];
                        }
                        for (int j = 0; j < 7; ++j)
                        {
                            if (i != j)
                            {
                                if (roots[i] != roots[j])
                                {
                                    x /= (roots[i] - roots[j]);
                                }
                            }
                        }
                        roots[i] -= x;
                        err[i] = abs(x);
                    }
                }
                if (!changed)
                {
                    break;
                }
            }

            // sort roots hoping that it solution indices become more robust to slight change in coeffs
            std::sort(roots, roots + 7, ComplexLess<IkReal>());

            numroots = 0;
            bool visited[7] = {false};
            for (int i = 0; i < 7; ++i)
            {
                if (!visited[i])
                {
                    // might be a multiple root, in which case it will have more error than the other roots
                    // find any neighboring roots, and take the average
                    complex<IkReal> newroot = roots[i];
                    int n = 1;
                    for (int j = i + 1; j < 7; ++j)
                    {
                        // care about error in real much more than imaginary
                        if (abs(real(roots[i]) - real(roots[j])) < tolsqrt && (abs(imag(roots[i]) - imag(roots[j])) < 0.002 || abs(imag(roots[i]) + imag(roots[j])) < 0.002) && abs(imag(roots[i])) < 0.002)
                        {
                            newroot += roots[j];
                            n += 1;
                            visited[j] = true;
                        }
                    }
                    if (n > 1)
                    {
                        newroot /= n;
                    }
                    // there are still cases where even the mean is not accurate enough, until a better multi-root algorithm is used, need to use the sqrt
                    if (IKabs(imag(newroot)) < tolsqrt)
                    {
                        rawroots[numroots++] = real(newroot);
                    }
                }
            }
        }
        static inline void polyroots6(IkReal rawcoeffs[6 + 1], IkReal rawroots[6], int &numroots)
        {
            using std::complex;
            if (rawcoeffs[0] == 0)
            {
                // solve with one reduced degree
                polyroots5(&rawcoeffs[1], &rawroots[0], numroots);
                return;
            }
            IKFAST_ASSERT(rawcoeffs[0] != 0);
            const IkReal tol = 128.0 * std::numeric_limits<IkReal>::epsilon();
            const IkReal tolsqrt = sqrt(std::numeric_limits<IkReal>::epsilon());
            complex<IkReal> coeffs[6];
            const int maxsteps = 110;
            for (int i = 0; i < 6; ++i)
            {
                coeffs[i] = complex<IkReal>(rawcoeffs[i + 1] / rawcoeffs[0]);
            }
            complex<IkReal> roots[6];
            IkReal err[6];
            roots[0] = complex<IkReal>(1, 0);
            roots[1] = complex<IkReal>(0.4, 0.9); // any complex number not a root of unity works
            err[0] = 1.0;
            err[1] = 1.0;
            for (int i = 2; i < 6; ++i)
            {
                roots[i] = roots[i - 1] * roots[1];
                err[i] = 1.0;
            }
            for (int step = 0; step < maxsteps; ++step)
            {
                bool changed = false;
                for (int i = 0; i < 6; ++i)
                {
                    if (err[i] >= tol)
                    {
                        changed = true;
                        // evaluate
                        complex<IkReal> x = roots[i] + coeffs[0];
                        for (int j = 1; j < 6; ++j)
                        {
                            x = roots[i] * x + coeffs[j];
                        }
                        for (int j = 0; j < 6; ++j)
                        {
                            if (i != j)
                            {
                                if (roots[i] != roots[j])
                                {
                                    x /= (roots[i] - roots[j]);
                                }
                            }
                        }
                        roots[i] -= x;
                        err[i] = abs(x);
                    }
                }
                if (!changed)
                {
                    break;
                }
            }

            // sort roots hoping that it solution indices become more robust to slight change in coeffs
            std::sort(roots, roots + 6, ComplexLess<IkReal>());

            numroots = 0;
            bool visited[6] = {false};
            for (int i = 0; i < 6; ++i)
            {
                if (!visited[i])
                {
                    // might be a multiple root, in which case it will have more error than the other roots
                    // find any neighboring roots, and take the average
                    complex<IkReal> newroot = roots[i];
                    int n = 1;
                    for (int j = i + 1; j < 6; ++j)
                    {
                        // care about error in real much more than imaginary
                        if (abs(real(roots[i]) - real(roots[j])) < tolsqrt && (abs(imag(roots[i]) - imag(roots[j])) < 0.002 || abs(imag(roots[i]) + imag(roots[j])) < 0.002) && abs(imag(roots[i])) < 0.002)
                        {
                            newroot += roots[j];
                            n += 1;
                            visited[j] = true;
                        }
                    }
                    if (n > 1)
                    {
                        newroot /= n;
                    }
                    // there are still cases where even the mean is not accurate enough, until a better multi-root algorithm is used, need to use the sqrt
                    if (IKabs(imag(newroot)) < tolsqrt)
                    {
                        rawroots[numroots++] = real(newroot);
                    }
                }
            }
        }
        static inline void polyroots8(IkReal rawcoeffs[8 + 1], IkReal rawroots[8], int &numroots)
        {
            using std::complex;
            if (rawcoeffs[0] == 0)
            {
                // solve with one reduced degree
                polyroots7(&rawcoeffs[1], &rawroots[0], numroots);
                return;
            }
            IKFAST_ASSERT(rawcoeffs[0] != 0);
            const IkReal tol = 128.0 * std::numeric_limits<IkReal>::epsilon();
            const IkReal tolsqrt = sqrt(std::numeric_limits<IkReal>::epsilon());
            complex<IkReal> coeffs[8];
            const int maxsteps = 110;
            for (int i = 0; i < 8; ++i)
            {
                coeffs[i] = complex<IkReal>(rawcoeffs[i + 1] / rawcoeffs[0]);
            }
            complex<IkReal> roots[8];
            IkReal err[8];
            roots[0] = complex<IkReal>(1, 0);
            roots[1] = complex<IkReal>(0.4, 0.9); // any complex number not a root of unity works
            err[0] = 1.0;
            err[1] = 1.0;
            for (int i = 2; i < 8; ++i)
            {
                roots[i] = roots[i - 1] * roots[1];
                err[i] = 1.0;
            }
            for (int step = 0; step < maxsteps; ++step)
            {
                bool changed = false;
                for (int i = 0; i < 8; ++i)
                {
                    if (err[i] >= tol)
                    {
                        changed = true;
                        // evaluate
                        complex<IkReal> x = roots[i] + coeffs[0];
                        for (int j = 1; j < 8; ++j)
                        {
                            x = roots[i] * x + coeffs[j];
                        }
                        for (int j = 0; j < 8; ++j)
                        {
                            if (i != j)
                            {
                                if (roots[i] != roots[j])
                                {
                                    x /= (roots[i] - roots[j]);
                                }
                            }
                        }
                        roots[i] -= x;
                        err[i] = abs(x);
                    }
                }
                if (!changed)
                {
                    break;
                }
            }

            // sort roots hoping that it solution indices become more robust to slight change in coeffs
            std::sort(roots, roots + 8, ComplexLess<IkReal>());

            numroots = 0;
            bool visited[8] = {false};
            for (int i = 0; i < 8; ++i)
            {
                if (!visited[i])
                {
                    // might be a multiple root, in which case it will have more error than the other roots
                    // find any neighboring roots, and take the average
                    complex<IkReal> newroot = roots[i];
                    int n = 1;
                    for (int j = i + 1; j < 8; ++j)
                    {
                        // care about error in real much more than imaginary
                        if (abs(real(roots[i]) - real(roots[j])) < tolsqrt && (abs(imag(roots[i]) - imag(roots[j])) < 0.002 || abs(imag(roots[i]) + imag(roots[j])) < 0.002) && abs(imag(roots[i])) < 0.002)
                        {
                            newroot += roots[j];
                            n += 1;
                            visited[j] = true;
                        }
                    }
                    if (n > 1)
                    {
                        newroot /= n;
                    }
                    // there are still cases where even the mean is not accurate enough, until a better multi-root algorithm is used, need to use the sqrt
                    if (IKabs(imag(newroot)) < tolsqrt)
                    {
                        rawroots[numroots++] = real(newroot);
                    }
                }
            }
        }
        inline void innerfn(IkSolutionListBase<IkReal> &solutions);
    };

    /// solves the inverse kinematics equations.
    /// \param pfree is an array specifying the free joints of the chain.
    IKFAST_API bool ComputeIk(const IkReal *eetrans, const IkReal *eerot, const IkReal *pfree, IkSolutionListBase<IkReal> &solutions);

    IKFAST_API bool ComputeIk2(const IkReal *eetrans, const IkReal *eerot, const IkReal *pfree, IkSolutionListBase<IkReal> &solutions, void *pOpenRAVEManip);

    IKFAST_API const char *GetKinematicsHash();

    IKFAST_API const char *GetIkFastVersion();

#ifdef IKFAST_NAMESPACE_AVENA
} // end namespace
#endif

#endif // IK_FAST__IK_AVENA_HPP_
