#include "ik_fast/ik_avena.hpp"

namespace ik_avena
{
    bool IKSolver::ComputeIk(const IkReal *eetrans, const IkReal *eerot, const IkReal *pfree, IkSolutionListBase<IkReal> &solutions)
    {
        j0 = numeric_limits<IkReal>::quiet_NaN();
        _ij0[0] = -1;
        _ij0[1] = -1;
        _nj0 = -1;
        j1 = numeric_limits<IkReal>::quiet_NaN();
        _ij1[0] = -1;
        _ij1[1] = -1;
        _nj1 = -1;
        j2 = numeric_limits<IkReal>::quiet_NaN();
        _ij2[0] = -1;
        _ij2[1] = -1;
        _nj2 = -1;
        j3 = numeric_limits<IkReal>::quiet_NaN();
        _ij3[0] = -1;
        _ij3[1] = -1;
        _nj3 = -1;
        j4 = numeric_limits<IkReal>::quiet_NaN();
        _ij4[0] = -1;
        _ij4[1] = -1;
        _nj4 = -1;
        j5 = numeric_limits<IkReal>::quiet_NaN();
        _ij5[0] = -1;
        _ij5[1] = -1;
        _nj5 = -1;
        for (int dummyiter = 0; dummyiter < 1; ++dummyiter)
        {
            solutions.Clear();
            r00 = eerot[0 * 3 + 0];
            r01 = eerot[0 * 3 + 1];
            r02 = eerot[0 * 3 + 2];
            r10 = eerot[1 * 3 + 0];
            r11 = eerot[1 * 3 + 1];
            r12 = eerot[1 * 3 + 2];
            r20 = eerot[2 * 3 + 0];
            r21 = eerot[2 * 3 + 1];
            r22 = eerot[2 * 3 + 2];
            px = eetrans[0];
            py = eetrans[1];
            pz = eetrans[2];

            new_r00 = r00;
            new_r01 = r01;
            new_r02 = r02;
            new_px = ((((-0.0676) * r02)) + px);
            new_r10 = r10;
            new_r11 = r11;
            new_r12 = r12;
            new_py = (py + (((-0.0676) * r12)));
            new_r20 = r20;
            new_r21 = r21;
            new_r22 = r22;
            new_pz = ((-0.1393) + pz + (((-0.0676) * r22)));
            r00 = new_r00;
            r01 = new_r01;
            r02 = new_r02;
            r10 = new_r10;
            r11 = new_r11;
            r12 = new_r12;
            r20 = new_r20;
            r21 = new_r21;
            r22 = new_r22;
            px = new_px;
            py = new_py;
            pz = new_pz;
            IkReal x60 = ((1.0) * px);
            IkReal x61 = ((1.0) * pz);
            IkReal x62 = ((1.0) * py);
            pp = ((px * px) + (py * py) + (pz * pz));
            npx = (((px * r00)) + ((py * r10)) + ((pz * r20)));
            npy = (((px * r01)) + ((py * r11)) + ((pz * r21)));
            npz = (((px * r02)) + ((py * r12)) + ((pz * r22)));
            rxp0_0 = ((((-1.0) * r20 * x62)) + ((pz * r10)));
            rxp0_1 = (((px * r20)) + (((-1.0) * r00 * x61)));
            rxp0_2 = ((((-1.0) * r10 * x60)) + ((py * r00)));
            rxp1_0 = ((((-1.0) * r21 * x62)) + ((pz * r11)));
            rxp1_1 = (((px * r21)) + (((-1.0) * r01 * x61)));
            rxp1_2 = ((((-1.0) * r11 * x60)) + ((py * r01)));
            rxp2_0 = (((pz * r12)) + (((-1.0) * r22 * x62)));
            rxp2_1 = (((px * r22)) + (((-1.0) * r02 * x61)));
            rxp2_2 = ((((-1.0) * r12 * x60)) + ((py * r02)));
            IkReal op[8 + 1], zeror[8];
            int numroots;
            IkReal x63 = ((-0.113399998709592) + npz);
            IkReal x64 = ((-0.113399998709592) + (((-1.0) * npz)));
            IkReal gconst0 = x63;
            IkReal gconst1 = x64;
            IkReal gconst2 = x63;
            IkReal gconst3 = x64;
            IkReal gconst4 = x63;
            IkReal gconst5 = x64;
            IkReal gconst6 = x63;
            IkReal gconst7 = x64;
            IkReal x65 = r22 * r22;
            IkReal x66 = r21 * r21;
            IkReal x67 = npx * npx;
            IkReal x68 = r20 * r20;
            IkReal x69 = npy * npy;
            IkReal x70 = ((4.0) * npy);
            IkReal x71 = ((1.0) * gconst4);
            IkReal x72 = ((2.0) * gconst1);
            IkReal x73 = ((1.0) * gconst1);
            IkReal x74 = (r20 * r22);
            IkReal x75 = ((2.0) * gconst5);
            IkReal x76 = ((1.0) * gconst0);
            IkReal x77 = (gconst5 * gconst6);
            IkReal x78 = ((16.0) * npx);
            IkReal x79 = (r21 * r22);
            IkReal x80 = ((1.0) * gconst5);
            IkReal x81 = ((8.0) * gconst5);
            IkReal x82 = ((2.0) * gconst4);
            IkReal x83 = ((8.0) * npy);
            IkReal x84 = (r20 * r21);
            IkReal x85 = ((8.0) * gconst1);
            IkReal x86 = ((4.0) * gconst1);
            IkReal x87 = (gconst1 * gconst6);
            IkReal x88 = (gconst2 * gconst5);
            IkReal x89 = ((2.0) * gconst0);
            IkReal x90 = ((2.1904e-12) * x65);
            IkReal x91 = ((5.476e-13) * x65);
            IkReal x92 = (npy * x65);
            IkReal x93 = (gconst2 * x79);
            IkReal x94 = (gconst3 * x65);
            IkReal x95 = (gconst2 * x65);
            IkReal x96 = ((2.0) * x65);
            IkReal x97 = (gconst7 * x65);
            IkReal x98 = (gconst2 * x66);
            IkReal x99 = (gconst6 * x65);
            IkReal x100 = ((16.0) * gconst1 * gconst2);
            IkReal x101 = (npx * x65);
            IkReal x102 = ((16.0) * x68);
            IkReal x103 = (gconst6 * x66);
            IkReal x104 = ((1.48e-6) * gconst5 * x79);
            IkReal x105 = ((1.48e-6) * gconst6 * x79);
            IkReal x106 = ((1.48e-6) * gconst1 * x79);
            IkReal x107 = ((1.48e-6) * x93);
            IkReal x108 = ((2.96e-6) * gconst6 * x74);
            IkReal x109 = ((1.776e-5) * x101);
            IkReal x110 = ((2.96e-6) * gconst5 * x74);
            IkReal x111 = ((5.92e-6) * x92);
            IkReal x112 = ((2.96e-6) * gconst2 * x74);
            IkReal x113 = ((2.96e-6) * gconst1 * x74);
            IkReal x114 = ((5.92e-6) * x101);
            IkReal x115 = ((2.96e-6) * x92);
            IkReal x116 = ((8.0) * npx * x79);
            IkReal x117 = (x65 * x69);
            IkReal x118 = (x65 * x67);
            IkReal x119 = (gconst6 * x70 * x79);
            IkReal x120 = (gconst5 * x70 * x79);
            IkReal x121 = (x78 * x92);
            IkReal x122 = ((16.0) * x77 * x84);
            IkReal x123 = (gconst5 * x74 * x78);
            IkReal x124 = (x70 * x93);
            IkReal x125 = (x71 * x99);
            IkReal x126 = ((4.0) * x66 * x77);
            IkReal x127 = ((1.0) * x65 * x77);
            IkReal x128 = (x80 * x97);
            IkReal x129 = (x71 * x97);
            IkReal x130 = (gconst1 * x70 * x79);
            IkReal x131 = (gconst6 * x74 * x78);
            IkReal x132 = (npx * x79 * x81);
            IkReal x133 = (npy * x74 * x81);
            IkReal x134 = (gconst6 * x74 * x83);
            IkReal x135 = (gconst6 * x116);
            IkReal x136 = ((16.0) * x84 * x87);
            IkReal x137 = ((16.0) * x84 * x88);
            IkReal x138 = (gconst1 * x74 * x78);
            IkReal x139 = ((16.0) * x118);
            IkReal x140 = (x76 * x97);
            IkReal x141 = (x80 * x95);
            IkReal x142 = (x73 * x97);
            IkReal x143 = (x71 * x95);
            IkReal x144 = ((4.0) * x66 * x88);
            IkReal x145 = (gconst2 * x74 * x78);
            IkReal x146 = (x103 * x86);
            IkReal x147 = (x73 * x99);
            IkReal x148 = (x80 * x94);
            IkReal x149 = (x71 * x94);
            IkReal x150 = (x76 * x99);
            IkReal x151 = (npx * x79 * x85);
            IkReal x152 = (gconst1 * x74 * x83);
            IkReal x153 = ((8.0) * npx * x93);
            IkReal x154 = (gconst2 * x74 * x83);
            IkReal x155 = (x100 * x84);
            IkReal x156 = ((4.0) * x117);
            IkReal x157 = (x73 * x95);
            IkReal x158 = (x76 * x94);
            IkReal x159 = (x76 * x95);
            IkReal x160 = (x73 * x94);
            IkReal x161 = (x86 * x98);
            IkReal x162 = (x156 + x91);
            IkReal x163 = (x139 + x90);
            IkReal x164 = (x124 + x105);
            IkReal x165 = (x119 + x106);
            IkReal x166 = (x135 + x134);
            IkReal x167 = (x137 + x136);
            IkReal x168 = (x154 + x153);
            IkReal x169 = (x131 + x120 + x107);
            IkReal x170 = (x145 + x130 + x104);
            IkReal x171 = (x133 + x132 + x121);
            IkReal x172 = (x152 + x151 + x121);
            IkReal x173 = (x126 + x127 + x125 + x128 + x129);
            IkReal x174 = (x160 + x161 + x157 + x159 + x158);
            IkReal x175 = (x150 + x140 + x141 + x142 + x143 + x144 + x146 + x147 + x148 + x149);
            op[0] = ((((-1.0) * x173)) + (((-1.0) * x105)) + (((-1.0) * x119)) + x162 + x120 + x115 + x104);
            op[1] = ((((-1.0) * x171)) + (((-1.0) * x114)) + (((-1.0) * x110)) + x166 + x122 + x108);
            op[2] = ((((-1.0) * x82 * x99)) + (((-1.0) * x82 * x97)) + (((-1.0) * x169)) + (((-1.0) * x164)) + (((-1.0) * x175)) + x163 + x165 + x130 + x123 + x111 + x104 + (((8.0) * x66 * x77)) + (((-1.0) * x75 * x99)) + (((-1.0) * x75 * x97)) + (((-1.0) * x102 * x77)));
            op[3] = ((((-1.0) * x172)) + (((5.92e-6) * gconst6 * x74)) + (((-1.0) * x109)) + (((-1.0) * x113)) + (((-1.0) * x122)) + x168 + x167 + x112 + (((-5.92e-6) * gconst5 * x74)));
            op[4] = ((((-1.0) * x82 * x94)) + (((-1.0) * x82 * x95)) + (((-1.0) * x169)) + (((-1.0) * x102 * x88)) + (((-1.0) * x102 * x87)) + ((x81 * x98)) + ((x103 * x85)) + (((-1.0) * x170)) + (((-1.0) * x174)) + (((-1.0) * x173)) + (((-1.0) * x72 * x99)) + (((-1.0) * x72 * x97)) + x164 + x165 + x138 + x123 + (((-8.0) * x117)) + (((3.2856e-12) * x65)) + (((-1.0) * x75 * x94)) + (((-1.0) * x75 * x95)) + (((32.0) * x118)) + (((-1.0) * x89 * x97)) + (((-1.0) * x89 * x99)));
            op[5] = ((((5.92e-6) * gconst2 * x74)) + (((-1.0) * x167)) + (((-1.0) * x166)) + (((-1.0) * x109)) + (((-5.92e-6) * gconst1 * x74)) + (((-1.0) * x110)) + x171 + x155 + x108);
            op[6] = ((((-1.0) * x165)) + (((-1.0) * x170)) + (((-1.0) * x175)) + (((-1.0) * x72 * x95)) + (((-1.0) * x72 * x94)) + (((-1.0) * x111)) + ((x85 * x98)) + x163 + x164 + x138 + x120 + x107 + (((-1.0) * x100 * x68)) + (((-1.0) * x89 * x95)) + (((-1.0) * x89 * x94)));
            op[7] = ((((-1.0) * x155)) + (((-1.0) * x168)) + (((-1.0) * x113)) + (((-1.0) * x114)) + x172 + x112);
            op[8] = ((((-1.0) * x174)) + (((-1.0) * x106)) + (((-1.0) * x115)) + (((-1.0) * x124)) + x162 + x130 + x107);
            polyroots8(op, zeror, numroots);
            IkReal j5array[8], cj5array[8], sj5array[8], tempj5array[1];
            int numsolutions = 0;
            for (int ij5 = 0; ij5 < numroots; ++ij5)
            {
                IkReal htj5 = zeror[ij5];
                tempj5array[0] = ((2.0) * (atan(htj5)));
                for (int kj5 = 0; kj5 < 1; ++kj5)
                {
                    j5array[numsolutions] = tempj5array[kj5];
                    if (j5array[numsolutions] > IKPI)
                    {
                        j5array[numsolutions] -= IK2PI;
                    }
                    else if (j5array[numsolutions] < -IKPI)
                    {
                        j5array[numsolutions] += IK2PI;
                    }
                    sj5array[numsolutions] = IKsin(j5array[numsolutions]);
                    cj5array[numsolutions] = IKcos(j5array[numsolutions]);
                    numsolutions++;
                }
            }
            bool j5valid[8] = {true, true, true, true, true, true, true, true};
            _nj5 = 8;
            for (int ij5 = 0; ij5 < numsolutions; ++ij5)
            {
                if (!j5valid[ij5])
                {
                    continue;
                }
                j5 = j5array[ij5];
                cj5 = cj5array[ij5];
                sj5 = sj5array[ij5];
                htj5 = IKtan(j5 / 2);

                _ij5[0] = ij5;
                _ij5[1] = -1;
                for (int iij5 = ij5 + 1; iij5 < numsolutions; ++iij5)
                {
                    if (j5valid[iij5] && IKabs(cj5array[ij5] - cj5array[iij5]) < IKFAST_SOLUTION_THRESH && IKabs(sj5array[ij5] - sj5array[iij5]) < IKFAST_SOLUTION_THRESH)
                    {
                        j5valid[iij5] = false;
                        _ij5[1] = iij5;
                        break;
                    }
                }
                {
                    IkReal j4eval[3];
                    IkReal x176 = (r20 * sj5);
                    IkReal x177 = ((2702702.7027027) * npz);
                    IkReal x178 = ((1.0) * npz);
                    IkReal x179 = (cj5 * r21);
                    IkReal x180 = (npx * r22 * sj5);
                    IkReal x181 = (cj5 * npy * r22);
                    j4eval[0] = ((((-1.0) * x177 * x179)) + (((-1.0) * r22)) + (((-1.0) * x176 * x177)) + (((2702702.7027027) * x180)) + (((2702702.7027027) * x181)));
                    j4eval[1] = ((((-2.24999997439666) * x176)) + (((-2.24999997439666) * x179)));
                    j4eval[2] = IKsign((x180 + x181 + (((-1.0) * x176 * x178)) + (((-1.0) * x178 * x179)) + (((-3.7e-7) * r22))));
                    if (IKabs(j4eval[0]) < 0.0000010000000000 || IKabs(j4eval[1]) < 0.0000010000000000 || IKabs(j4eval[2]) < 0.0000010000000000)
                    {
                        {
                            IkReal evalcond[1];
                            bool bgotonextstatement = true;
                            do
                            {
                                IkReal x182 = r22 * r22;
                                IkReal x183 = npz * npz;
                                IkReal x184 = ((2702702.7027027) * r22);
                                IkReal x185 = ((2702702.7027027) * npz);
                                IkReal x186 = ((7304601899196.49) * x183);
                                IkReal x187 = ((7304601899196.49) * x182);
                                IkReal x188 = (((npy * x184)) + (((-1.0) * r21 * x185)));
                                IkReal x189 = (((npx * x184)) + (((-1.0) * r20 * x185)));
                                IkReal x190 = (((x187 * (npy * npy))) + x186 + ((x187 * (npx * npx))) + (((-14609203798393.0) * npz * pz * r22)) + ((x182 * x186)));
                                if ((x190) < -0.00001)
                                    continue;
                                IkReal x191 = IKabs(IKsqrt(x190));
                                IkReal x197 = x190;
                                if (IKabs(x197) == 0)
                                {
                                    continue;
                                }
                                IkReal x192 = pow(x197, -0.5);
                                CheckValue<IkReal> x198 = IKPowWithIntegerCheck(x191, -1);
                                if (!x198.valid)
                                {
                                    continue;
                                }
                                IkReal x193 = x198.value;
                                if ((((1.0) + (((-1.0) * x182 * (x193 * x193))))) < -0.00001)
                                    continue;
                                IkReal x194 = IKsqrt(((1.0) + (((-1.0) * x182 * (x193 * x193)))));
                                IkReal x195 = (r22 * x192 * x193);
                                IkReal x196 = (x192 * x194);
                                if ((((x189 * x189) + (x188 * x188))) < -0.00001)
                                    continue;
                                CheckValue<IkReal> x199 = IKPowWithIntegerCheck(IKabs(IKsqrt(((x189 * x189) + (x188 * x188)))), -1);
                                if (!x199.valid)
                                {
                                    continue;
                                }
                                if (((r22 * (x199.value))) < -1 - IKFAST_SINCOS_THRESH || ((r22 * (x199.value))) > 1 + IKFAST_SINCOS_THRESH)
                                    continue;
                                CheckValue<IkReal> x200 = IKatan2WithCheck(IkReal(x188), IkReal(x189), IKFAST_ATAN2_MAGTHRESH);
                                if (!x200.valid)
                                {
                                    continue;
                                }
                                IkReal gconst24 = ((IKasin((r22 * (x199.value)))) + (((-1.0) * (x200.value))));
                                IkReal gconst25 = ((((-1.0) * x188 * x196)) + ((x189 * x195)));
                                IkReal gconst26 = (((x188 * x195)) + ((x189 * x196)));
                                IkReal x201 = ((2702702.7027027) * r22);
                                IkReal x202 = ((2702702.7027027) * npz);
                                IkReal x203 = x189;
                                IkReal x204 = x188;
                                CheckValue<IkReal> x205 = IKatan2WithCheck(IkReal(x204), IkReal(x203), IKFAST_ATAN2_MAGTHRESH);
                                if (!x205.valid)
                                {
                                    continue;
                                }
                                if ((((x203 * x203) + (x204 * x204))) < -0.00001)
                                    continue;
                                CheckValue<IkReal> x206 = IKPowWithIntegerCheck(IKabs(IKsqrt(((x203 * x203) + (x204 * x204)))), -1);
                                if (!x206.valid)
                                {
                                    continue;
                                }
                                if (((r22 * (x206.value))) < -1 - IKFAST_SINCOS_THRESH || ((r22 * (x206.value))) > 1 + IKFAST_SINCOS_THRESH)
                                    continue;
                                evalcond[0] = ((-3.14159265358979) + (IKfmod(((3.14159265358979) + (IKabs(((x205.value) + (((-1.0) * (IKasin((r22 * (x206.value)))))) + j5)))), 6.28318530717959)));
                                if (IKabs(evalcond[0]) < 0.0000050000000000)
                                {
                                    bgotonextstatement = false;
                                    {
                                        IkReal j4array[1], cj4array[1], sj4array[1];
                                        bool j4valid[1] = {false};
                                        _nj4 = 1;
                                        IkReal x207 = (gconst26 * r21);
                                        IkReal x208 = (gconst25 * r20);
                                        IkReal x209 = ((1.0) * npz);
                                        CheckValue<IkReal> x210 = IKatan2WithCheck(IkReal(((-0.113399998709592) * r22)), IkReal(((((-0.113399998709592) * x208)) + (((-0.113399998709592) * x207)))), IKFAST_ATAN2_MAGTHRESH);
                                        if (!x210.valid)
                                        {
                                            continue;
                                        }
                                        CheckValue<IkReal> x211 = IKPowWithIntegerCheck(IKsign((((gconst25 * npx * r22)) + (((-1.0) * x208 * x209)) + (((-1.0) * x207 * x209)) + ((gconst26 * npy * r22)) + (((-3.7e-7) * r22)))), -1);
                                        if (!x211.valid)
                                        {
                                            continue;
                                        }
                                        j4array[0] = ((-1.5707963267949) + (x210.value) + (((1.5707963267949) * (x211.value))));
                                        sj4array[0] = IKsin(j4array[0]);
                                        cj4array[0] = IKcos(j4array[0]);
                                        if (j4array[0] > IKPI)
                                        {
                                            j4array[0] -= IK2PI;
                                        }
                                        else if (j4array[0] < -IKPI)
                                        {
                                            j4array[0] += IK2PI;
                                        }
                                        j4valid[0] = true;
                                        for (int ij4 = 0; ij4 < 1; ++ij4)
                                        {
                                            if (!j4valid[ij4])
                                            {
                                                continue;
                                            }
                                            _ij4[0] = ij4;
                                            _ij4[1] = -1;
                                            for (int iij4 = ij4 + 1; iij4 < 1; ++iij4)
                                            {
                                                if (j4valid[iij4] && IKabs(cj4array[ij4] - cj4array[iij4]) < IKFAST_SOLUTION_THRESH && IKabs(sj4array[ij4] - sj4array[iij4]) < IKFAST_SOLUTION_THRESH)
                                                {
                                                    j4valid[iij4] = false;
                                                    _ij4[1] = iij4;
                                                    break;
                                                }
                                            }
                                            j4 = j4array[ij4];
                                            cj4 = cj4array[ij4];
                                            sj4 = sj4array[ij4];
                                            {
                                                IkReal evalcond[2];
                                                IkReal x212 = IKsin(j4);
                                                IkReal x213 = IKcos(j4);
                                                IkReal x214 = ((1.0) * gconst25 * x212);
                                                IkReal x215 = ((1.0) * gconst26 * x212);
                                                evalcond[0] = ((((-1.0) * r20 * x214)) + ((r22 * x213)) + (((-1.0) * r21 * x215)));
                                                evalcond[1] = ((-0.113399998709592) + (((-1.0) * npy * x215)) + (((3.7e-7) * x212)) + (((-1.0) * npx * x214)) + ((npz * x213)));
                                                if (IKabs(evalcond[0]) > IKFAST_EVALCOND_THRESH || IKabs(evalcond[1]) > IKFAST_EVALCOND_THRESH)
                                                {
                                                    continue;
                                                }
                                            }

                                            {
                                                IkReal j0eval[1];
                                                IkReal x216 = ((1.01958072531191) * sj4);
                                                j0eval[0] = ((-1.0) + (((-1.01958072531191) * cj4 * r02)) + ((r00 * sj5 * x216)) + ((cj5 * r01 * x216)));
                                                if (IKabs(j0eval[0]) < 0.0000010000000000)
                                                {
                                                    {
                                                        IkReal j0eval[1];
                                                        IkReal x217 = ((5.12715705179976) * sj4);
                                                        j0eval[0] = ((-1.0) + (((5.12715705179976) * cj4 * r12)) + (((-1.0) * cj5 * r11 * x217)) + (((-1.0) * r10 * sj5 * x217)));
                                                        if (IKabs(j0eval[0]) < 0.0000010000000000)
                                                        {
                                                            {
                                                                IkReal j0eval[1];
                                                                IkReal x218 = (sj4 * sj5);
                                                                IkReal x219 = (cj5 * sj4);
                                                                j0eval[0] = ((((5.02869162246205) * r10 * x218)) + (((5.02869162246205) * r11 * x219)) + (((-1.0) * cj4 * r02)) + (((-5.02869162246205) * cj4 * r12)) + ((r01 * x219)) + ((r00 * x218)));
                                                                if (IKabs(j0eval[0]) < 0.0000010000000000)
                                                                {
                                                                    continue; // no branches [j0]
                                                                }
                                                                else
                                                                {
                                                                    {
                                                                        IkReal j0array[2], cj0array[2], sj0array[2];
                                                                        bool j0valid[2] = {false};
                                                                        _nj0 = 2;
                                                                        IkReal x220 = cj4 * cj4;
                                                                        IkReal x221 = r00 * r00;
                                                                        IkReal x222 = cj5 * cj5;
                                                                        IkReal x223 = r10 * r10;
                                                                        IkReal x224 = r11 * r11;
                                                                        IkReal x225 = r01 * r01;
                                                                        IkReal x226 = ((0.195039861251953) * sj4);
                                                                        IkReal x227 = (r00 * sj5);
                                                                        IkReal x228 = (cj5 * r01);
                                                                        IkReal x229 = ((0.382588364824742) * r12);
                                                                        IkReal x230 = (cj4 * sj4);
                                                                        IkReal x231 = (r10 * sj5);
                                                                        IkReal x232 = ((1.92391890504564) * r12);
                                                                        IkReal x233 = (cj5 * r11);
                                                                        IkReal x234 = ((0.382588364824742) * r02);
                                                                        IkReal x235 = ((0.980795316323859) * sj4);
                                                                        IkReal x236 = ((0.0760810949543624) * r02);
                                                                        IkReal x237 = ((0.382588364824742) * r00 * r10);
                                                                        IkReal x238 = ((0.961959452522819) * x223);
                                                                        IkReal x239 = ((0.0380405474771812) * x221);
                                                                        IkReal x240 = ((0.0380405474771812) * x225);
                                                                        IkReal x241 = ((0.961959452522819) * x224);
                                                                        IkReal x242 = ((0.382588364824742) * r01 * r11);
                                                                        IkReal x243 = ((0.382588364824742) * x220);
                                                                        IkReal x244 = (x220 * x222);
                                                                        CheckValue<IkReal> x248 = IKPowWithIntegerCheck(((((-0.195039861251953) * cj4 * r02)) + ((x233 * x235)) + (((-0.980795316323859) * cj4 * r12)) + ((x226 * x228)) + ((x226 * x227)) + ((x231 * x235))), -1);
                                                                        if (!x248.valid)
                                                                        {
                                                                            continue;
                                                                        }
                                                                        IkReal x245 = x248.value;
                                                                        if ((((1.0) + (((-0.382588364824742) * x228 * x231)) + ((x228 * x230 * x236)) + (((-0.0380405474771812) * x220 * (r02 * r02))) + ((x240 * x244)) + ((x228 * x229 * x230)) + (((-0.382588364824742) * x227 * x233)) + (((0.0760810949543624) * x220 * x227 * x228)) + (((-0.0760810949543624) * x227 * x228)) + ((x227 * x233 * x243)) + (((-1.0) * x222 * x241)) + (((-1.0) * x222 * x240)) + (((-1.0) * x222 * x242)) + (((-1.0) * x237 * x244)) + ((x227 * x229 * x230)) + (((-1.0) * x239)) + (((-1.0) * x238)) + (((-1.0) * x237)) + ((x222 * x237)) + ((x222 * x239)) + ((x222 * x238)) + ((x230 * x232 * x233)) + ((x227 * x230 * x236)) + (((-1.92391890504564) * x231 * x233)) + ((x241 * x244)) + (((-1.0) * x238 * x244)) + ((x242 * x244)) + (((-0.961959452522819) * x220 * (r12 * r12))) + ((x230 * x233 * x234)) + (((1.92391890504564) * x220 * x231 * x233)) + (((-1.0) * r02 * x220 * x229)) + (((-1.0) * x239 * x244)) + ((x220 * x237)) + ((x220 * x239)) + ((x220 * x238)) + ((x228 * x231 * x243)) + ((x230 * x231 * x232)) + ((x230 * x231 * x234)))) < -0.00001)
                                                                            continue;
                                                                        IkReal x246 = IKsqrt(((1.0) + (((-0.382588364824742) * x228 * x231)) + ((x228 * x230 * x236)) + (((-0.0380405474771812) * x220 * (r02 * r02))) + ((x240 * x244)) + ((x228 * x229 * x230)) + (((-0.382588364824742) * x227 * x233)) + (((0.0760810949543624) * x220 * x227 * x228)) + (((-0.0760810949543624) * x227 * x228)) + ((x227 * x233 * x243)) + (((-1.0) * x222 * x241)) + (((-1.0) * x222 * x240)) + (((-1.0) * x222 * x242)) + (((-1.0) * x237 * x244)) + ((x227 * x229 * x230)) + (((-1.0) * x239)) + (((-1.0) * x238)) + (((-1.0) * x237)) + ((x222 * x237)) + ((x222 * x239)) + ((x222 * x238)) + ((x230 * x232 * x233)) + ((x227 * x230 * x236)) + (((-1.92391890504564) * x231 * x233)) + ((x241 * x244)) + (((-1.0) * x238 * x244)) + ((x242 * x244)) + (((-0.961959452522819) * x220 * (r12 * r12))) + ((x230 * x233 * x234)) + (((1.92391890504564) * x220 * x231 * x233)) + (((-1.0) * r02 * x220 * x229)) + (((-1.0) * x239 * x244)) + ((x220 * x237)) + ((x220 * x239)) + ((x220 * x238)) + ((x228 * x231 * x243)) + ((x230 * x231 * x232)) + ((x230 * x231 * x234))));
                                                                        IkReal x247 = (x245 * x246);
                                                                        j0array[0] = ((2.0) * (atan(((((1.0) * x247)) + (((-1.0) * x245))))));
                                                                        sj0array[0] = IKsin(j0array[0]);
                                                                        cj0array[0] = IKcos(j0array[0]);
                                                                        j0array[1] = ((-2.0) * (atan((x247 + x245))));
                                                                        sj0array[1] = IKsin(j0array[1]);
                                                                        cj0array[1] = IKcos(j0array[1]);
                                                                        if (j0array[0] > IKPI)
                                                                        {
                                                                            j0array[0] -= IK2PI;
                                                                        }
                                                                        else if (j0array[0] < -IKPI)
                                                                        {
                                                                            j0array[0] += IK2PI;
                                                                        }
                                                                        j0valid[0] = true;
                                                                        if (j0array[1] > IKPI)
                                                                        {
                                                                            j0array[1] -= IK2PI;
                                                                        }
                                                                        else if (j0array[1] < -IKPI)
                                                                        {
                                                                            j0array[1] += IK2PI;
                                                                        }
                                                                        j0valid[1] = true;
                                                                        for (int ij0 = 0; ij0 < 2; ++ij0)
                                                                        {
                                                                            if (!j0valid[ij0])
                                                                            {
                                                                                continue;
                                                                            }
                                                                            _ij0[0] = ij0;
                                                                            _ij0[1] = -1;
                                                                            for (int iij0 = ij0 + 1; iij0 < 2; ++iij0)
                                                                            {
                                                                                if (j0valid[iij0] && IKabs(cj0array[ij0] - cj0array[iij0]) < IKFAST_SOLUTION_THRESH && IKabs(sj0array[ij0] - sj0array[iij0]) < IKFAST_SOLUTION_THRESH)
                                                                                {
                                                                                    j0valid[iij0] = false;
                                                                                    _ij0[1] = iij0;
                                                                                    break;
                                                                                }
                                                                            }
                                                                            j0 = j0array[ij0];
                                                                            cj0 = cj0array[ij0];
                                                                            sj0 = sj0array[ij0];

                                                                            innerfn(solutions);
                                                                        }
                                                                    }
                                                                }
                                                            }
                                                        }
                                                        else
                                                        {
                                                            {
                                                                IkReal j0array[1], cj0array[1], sj0array[1];
                                                                bool j0valid[1] = {false};
                                                                _nj0 = 1;
                                                                IkReal x1247 = (cj5 * sj4);
                                                                IkReal x1248 = (sj4 * sj5);
                                                                CheckValue<IkReal> x1251 = IKPowWithIntegerCheck(((-0.195039861251953) + (((-1.0) * r11 * x1247)) + (((-1.0) * r10 * x1248)) + ((cj4 * r12))), -1);
                                                                if (!x1251.valid)
                                                                {
                                                                    continue;
                                                                }
                                                                IkReal x1249 = x1251.value;
                                                                IkReal x1250 = (sj4 * x1249);
                                                                CheckValue<IkReal> x1252 = IKPowWithIntegerCheck(((-0.195039861251953) + (((-1.0) * r10 * x1248)) + (((-1.0) * cj5 * r11 * sj4)) + ((cj4 * r12))), -1);
                                                                if (!x1252.valid)
                                                                {
                                                                    continue;
                                                                }
                                                                CheckValue<IkReal> x1253 = IKPowWithIntegerCheck(((-0.195039861251953) + (((-1.0) * r11 * x1247)) + (((-1.0) * r10 * sj4 * sj5)) + ((cj4 * r12))), -1);
                                                                if (!x1253.valid)
                                                                {
                                                                    continue;
                                                                }
                                                                j0array[0] = ((2.0) * (atan(((((-1.0) * cj4 * r02 * x1249)) + ((r01 * x1247 * (x1252.value))) + (((0.980795316323859) * x1249)) + ((r00 * x1248 * (x1253.value)))))));
                                                                sj0array[0] = IKsin(j0array[0]);
                                                                cj0array[0] = IKcos(j0array[0]);
                                                                if (j0array[0] > IKPI)
                                                                {
                                                                    j0array[0] -= IK2PI;
                                                                }
                                                                else if (j0array[0] < -IKPI)
                                                                {
                                                                    j0array[0] += IK2PI;
                                                                }
                                                                j0valid[0] = true;
                                                                for (int ij0 = 0; ij0 < 1; ++ij0)
                                                                {
                                                                    if (!j0valid[ij0])
                                                                    {
                                                                        continue;
                                                                    }
                                                                    _ij0[0] = ij0;
                                                                    _ij0[1] = -1;
                                                                    for (int iij0 = ij0 + 1; iij0 < 1; ++iij0)
                                                                    {
                                                                        if (j0valid[iij0] && IKabs(cj0array[ij0] - cj0array[iij0]) < IKFAST_SOLUTION_THRESH && IKabs(sj0array[ij0] - sj0array[iij0]) < IKFAST_SOLUTION_THRESH)
                                                                        {
                                                                            j0valid[iij0] = false;
                                                                            _ij0[1] = iij0;
                                                                            break;
                                                                        }
                                                                    }
                                                                    j0 = j0array[ij0];
                                                                    cj0 = cj0array[ij0];
                                                                    sj0 = sj0array[ij0];

                                                                    innerfn(solutions);
                                                                }
                                                            }
                                                        }
                                                    }
                                                }
                                                else
                                                {
                                                    {
                                                        IkReal j0array[1], cj0array[1], sj0array[1];
                                                        bool j0valid[1] = {false};
                                                        _nj0 = 1;
                                                        IkReal x1254 = ((1.0) * cj4);
                                                        IkReal x1255 = (sj4 * sj5);
                                                        IkReal x1256 = (cj5 * sj4);
                                                        CheckValue<IkReal> x1258 = IKPowWithIntegerCheck(((-0.980795316323859) + (((-1.0) * r02 * x1254)) + ((r00 * x1255)) + ((r01 * x1256))), -1);
                                                        if (!x1258.valid)
                                                        {
                                                            continue;
                                                        }
                                                        IkReal x1257 = x1258.value;
                                                        CheckValue<IkReal> x1259 = IKPowWithIntegerCheck(((-0.980795316323859) + ((cj5 * r01 * sj4)) + (((-1.0) * r02 * x1254)) + ((r00 * x1255))), -1);
                                                        if (!x1259.valid)
                                                        {
                                                            continue;
                                                        }
                                                        CheckValue<IkReal> x1260 = IKPowWithIntegerCheck(((-0.980795316323859) + ((r00 * sj4 * sj5)) + (((-1.0) * r02 * x1254)) + ((r01 * x1256))), -1);
                                                        if (!x1260.valid)
                                                        {
                                                            continue;
                                                        }
                                                        CheckValue<IkReal> x1261 = IKPowWithIntegerCheck(((-0.980795316323859) + (((-1.0) * cj4 * r02)) + ((r00 * x1255)) + ((r01 * x1256))), -1);
                                                        if (!x1261.valid)
                                                        {
                                                            continue;
                                                        }
                                                        j0array[0] = ((2.0) * (atan(((((-0.195039861251953) * x1257)) + ((r11 * x1256 * (x1259.value))) + ((r10 * x1255 * (x1260.value))) + (((-1.0) * r12 * x1254 * (x1261.value)))))));
                                                        sj0array[0] = IKsin(j0array[0]);
                                                        cj0array[0] = IKcos(j0array[0]);
                                                        if (j0array[0] > IKPI)
                                                        {
                                                            j0array[0] -= IK2PI;
                                                        }
                                                        else if (j0array[0] < -IKPI)
                                                        {
                                                            j0array[0] += IK2PI;
                                                        }
                                                        j0valid[0] = true;
                                                        for (int ij0 = 0; ij0 < 1; ++ij0)
                                                        {
                                                            if (!j0valid[ij0])
                                                            {
                                                                continue;
                                                            }
                                                            _ij0[0] = ij0;
                                                            _ij0[1] = -1;
                                                            for (int iij0 = ij0 + 1; iij0 < 1; ++iij0)
                                                            {
                                                                if (j0valid[iij0] && IKabs(cj0array[ij0] - cj0array[iij0]) < IKFAST_SOLUTION_THRESH && IKabs(sj0array[ij0] - sj0array[iij0]) < IKFAST_SOLUTION_THRESH)
                                                                {
                                                                    j0valid[iij0] = false;
                                                                    _ij0[1] = iij0;
                                                                    break;
                                                                }
                                                            }
                                                            j0 = j0array[ij0];
                                                            cj0 = cj0array[ij0];
                                                            sj0 = sj0array[ij0];

                                                            innerfn(solutions);
                                                        }
                                                    }
                                                }
                                            }
                                        }
                                    }
                                }
                            } while (0);
                            if (bgotonextstatement)
                            {
                                bool bgotonextstatement = true;
                                do
                                {
                                    IkReal x1262 = r22 * r22;
                                    IkReal x1263 = npz * npz;
                                    IkReal x1264 = ((2702702.7027027) * r22);
                                    IkReal x1265 = ((2702702.7027027) * npz);
                                    IkReal x1266 = ((7304601899196.49) * x1263);
                                    IkReal x1267 = ((7304601899196.49) * x1262);
                                    IkReal x1268 = (((npx * x1264)) + (((-1.0) * r20 * x1265)));
                                    IkReal x1269 = ((((-1.0) * r21 * x1265)) + ((npy * x1264)));
                                    IkReal x1270 = (x1266 + ((x1262 * x1266)) + (((-14609203798393.0) * npz * pz * r22)) + ((x1267 * (npy * npy))) + ((x1267 * (npx * npx))));
                                    if ((x1270) < -0.00001)
                                        continue;
                                    IkReal x1271 = IKabs(IKsqrt(x1270));
                                    IkReal x1279 = x1270;
                                    if (IKabs(x1279) == 0)
                                    {
                                        continue;
                                    }
                                    IkReal x1272 = pow(x1279, -0.5);
                                    IkReal x1273 = ((1.0) * x1272);
                                    CheckValue<IkReal> x1280 = IKPowWithIntegerCheck(x1271, -1);
                                    if (!x1280.valid)
                                    {
                                        continue;
                                    }
                                    IkReal x1274 = x1280.value;
                                    IkReal x1275 = (r22 * x1274);
                                    CheckValue<IkReal> x1281 = IKPowWithIntegerCheck(x1271, -2);
                                    if (!x1281.valid)
                                    {
                                        continue;
                                    }
                                    IkReal x1276 = x1281.value;
                                    IkReal x1277 = (x1262 * x1276);
                                    IkReal x1278 = (x1269 * x1273);
                                    if ((((x1268 * x1268) + (x1269 * x1269))) < -0.00001)
                                        continue;
                                    CheckValue<IkReal> x1282 = IKPowWithIntegerCheck(IKabs(IKsqrt(((x1268 * x1268) + (x1269 * x1269)))), -1);
                                    if (!x1282.valid)
                                    {
                                        continue;
                                    }
                                    if (((r22 * (x1282.value))) < -1 - IKFAST_SINCOS_THRESH || ((r22 * (x1282.value))) > 1 + IKFAST_SINCOS_THRESH)
                                        continue;
                                    CheckValue<IkReal> x1283 = IKatan2WithCheck(IkReal(x1269), IkReal(x1268), IKFAST_ATAN2_MAGTHRESH);
                                    if (!x1283.valid)
                                    {
                                        continue;
                                    }
                                    IkReal gconst27 = ((3.14159265358979) + (((-1.0) * (IKasin((r22 * (x1282.value)))))) + (((-1.0) * (x1283.value))));
                                    if ((((1.0) + (((-1.0) * x1277)))) < -0.00001)
                                        continue;
                                    IkReal gconst28 = (((x1268 * x1273 * x1275)) + ((x1278 * (IKsqrt(((1.0) + (((-1.0) * x1277))))))));
                                    if ((((1.0) + (((-1.0) * x1277)))) < -0.00001)
                                        continue;
                                    IkReal gconst29 = (((x1275 * x1278)) + (((-1.0) * x1268 * x1273 * (IKsqrt(((1.0) + (((-1.0) * x1277))))))));
                                    IkReal x1284 = r22 * r22;
                                    IkReal x1285 = npz * npz;
                                    IkReal x1286 = j5;
                                    IkReal x1287 = (npz * r21);
                                    IkReal x1288 = ((2702702.7027027) * r22);
                                    IkReal x1289 = ((14609203798393.0) * r22);
                                    IkReal x1290 = (npz * r20);
                                    IkReal x1291 = ((7304601899196.49) * x1284);
                                    IkReal x1292 = ((7304601899196.49) * x1285);
                                    CheckValue<IkReal> x1297 = IKatan2WithCheck(IkReal(((((-2702702.7027027) * x1287)) + ((npy * x1288)))), IkReal(((((-2702702.7027027) * x1290)) + ((npx * x1288)))), IKFAST_ATAN2_MAGTHRESH);
                                    if (!x1297.valid)
                                    {
                                        continue;
                                    }
                                    IkReal x1293 = x1297.value;
                                    IkReal x1294 = x1293;
                                    if ((((((-1.0) * npy * x1287 * x1289)) + ((x1291 * (npy * npy))) + ((x1292 * (r21 * r21))) + ((x1291 * (npx * npx))) + ((x1292 * (r20 * r20))) + (((-1.0) * npx * x1289 * x1290)))) < -0.00001)
                                        continue;
                                    CheckValue<IkReal> x1298 = IKPowWithIntegerCheck(IKabs(IKsqrt(((((-1.0) * npy * x1287 * x1289)) + ((x1291 * (npy * npy))) + ((x1292 * (r21 * r21))) + ((x1291 * (npx * npx))) + ((x1292 * (r20 * r20))) + (((-1.0) * npx * x1289 * x1290))))), -1);
                                    if (!x1298.valid)
                                    {
                                        continue;
                                    }
                                    if (((r22 * (x1298.value))) < -1 - IKFAST_SINCOS_THRESH || ((r22 * (x1298.value))) > 1 + IKFAST_SINCOS_THRESH)
                                        continue;
                                    IkReal x1295 = IKasin((r22 * (x1298.value)));
                                    IkReal x1296 = x1295;
                                    if ((((9.86960440108936) + (((-3.14159265358979) * x1286)) + ((j5 * x1296)) + ((j5 * x1294)) + ((x1286 * x1295)) + ((x1286 * x1293)) + ((x1295 * x1296)) + ((x1294 * x1295)) + ((x1293 * x1296)) + ((x1293 * x1294)) + (((-3.14159265358979) * x1296)) + (((-3.14159265358979) * x1295)) + (((-3.14159265358979) * x1294)) + (((-3.14159265358979) * x1293)) + ((j5 * x1286)) + (((-3.14159265358979) * j5)))) < -0.00001)
                                        continue;
                                    evalcond[0] = ((-3.14159265358979) + (IKfmod(((3.14159265358979) + (IKsqrt(((9.86960440108936) + (((-3.14159265358979) * x1286)) + ((j5 * x1296)) + ((j5 * x1294)) + ((x1286 * x1295)) + ((x1286 * x1293)) + ((x1295 * x1296)) + ((x1294 * x1295)) + ((x1293 * x1296)) + ((x1293 * x1294)) + (((-3.14159265358979) * x1296)) + (((-3.14159265358979) * x1295)) + (((-3.14159265358979) * x1294)) + (((-3.14159265358979) * x1293)) + ((j5 * x1286)) + (((-3.14159265358979) * j5)))))), 6.28318530717959)));
                                    if (IKabs(evalcond[0]) < 0.0000050000000000)
                                    {
                                        bgotonextstatement = false;
                                        {
                                            IkReal j4array[1], cj4array[1], sj4array[1];
                                            bool j4valid[1] = {false};
                                            _nj4 = 1;
                                            IkReal x1299 = (gconst28 * r20);
                                            IkReal x1300 = ((1.0) * npz);
                                            IkReal x1301 = (gconst29 * r21);
                                            CheckValue<IkReal> x1302 = IKPowWithIntegerCheck(IKsign(((((-1.0) * x1299 * x1300)) + (((-1.0) * x1300 * x1301)) + ((gconst28 * npx * r22)) + ((gconst29 * npy * r22)) + (((-3.7e-7) * r22)))), -1);
                                            if (!x1302.valid)
                                            {
                                                continue;
                                            }
                                            CheckValue<IkReal> x1303 = IKatan2WithCheck(IkReal(((-0.113399998709592) * r22)), IkReal(((((-0.113399998709592) * x1301)) + (((-0.113399998709592) * x1299)))), IKFAST_ATAN2_MAGTHRESH);
                                            if (!x1303.valid)
                                            {
                                                continue;
                                            }
                                            j4array[0] = ((-1.5707963267949) + (((1.5707963267949) * (x1302.value))) + (x1303.value));
                                            sj4array[0] = IKsin(j4array[0]);
                                            cj4array[0] = IKcos(j4array[0]);
                                            if (j4array[0] > IKPI)
                                            {
                                                j4array[0] -= IK2PI;
                                            }
                                            else if (j4array[0] < -IKPI)
                                            {
                                                j4array[0] += IK2PI;
                                            }
                                            j4valid[0] = true;
                                            for (int ij4 = 0; ij4 < 1; ++ij4)
                                            {
                                                if (!j4valid[ij4])
                                                {
                                                    continue;
                                                }
                                                _ij4[0] = ij4;
                                                _ij4[1] = -1;
                                                for (int iij4 = ij4 + 1; iij4 < 1; ++iij4)
                                                {
                                                    if (j4valid[iij4] && IKabs(cj4array[ij4] - cj4array[iij4]) < IKFAST_SOLUTION_THRESH && IKabs(sj4array[ij4] - sj4array[iij4]) < IKFAST_SOLUTION_THRESH)
                                                    {
                                                        j4valid[iij4] = false;
                                                        _ij4[1] = iij4;
                                                        break;
                                                    }
                                                }
                                                j4 = j4array[ij4];
                                                cj4 = cj4array[ij4];
                                                sj4 = sj4array[ij4];
                                                {
                                                    IkReal evalcond[2];
                                                    IkReal x1304 = IKsin(j4);
                                                    IkReal x1305 = IKcos(j4);
                                                    IkReal x1306 = ((1.0) * x1304);
                                                    evalcond[0] = (((r22 * x1305)) + (((-1.0) * gconst28 * r20 * x1306)) + (((-1.0) * gconst29 * r21 * x1306)));
                                                    evalcond[1] = ((-0.113399998709592) + (((3.7e-7) * x1304)) + (((-1.0) * gconst28 * npx * x1306)) + (((-1.0) * gconst29 * npy * x1306)) + ((npz * x1305)));
                                                    if (IKabs(evalcond[0]) > IKFAST_EVALCOND_THRESH || IKabs(evalcond[1]) > IKFAST_EVALCOND_THRESH)
                                                    {
                                                        continue;
                                                    }
                                                }

                                                {
                                                    IkReal j0eval[1];
                                                    IkReal x1307 = ((1.01958072531191) * sj4);
                                                    j0eval[0] = ((-1.0) + (((-1.01958072531191) * cj4 * r02)) + ((r00 * sj5 * x1307)) + ((cj5 * r01 * x1307)));
                                                    if (IKabs(j0eval[0]) < 0.0000010000000000)
                                                    {
                                                        {
                                                            IkReal j0eval[1];
                                                            IkReal x1308 = ((5.12715705179976) * sj4);
                                                            j0eval[0] = ((-1.0) + (((5.12715705179976) * cj4 * r12)) + (((-1.0) * cj5 * r11 * x1308)) + (((-1.0) * r10 * sj5 * x1308)));
                                                            if (IKabs(j0eval[0]) < 0.0000010000000000)
                                                            {
                                                                {
                                                                    IkReal j0eval[1];
                                                                    IkReal x1309 = (sj4 * sj5);
                                                                    IkReal x1310 = (cj5 * sj4);
                                                                    j0eval[0] = ((((-1.0) * cj4 * r02)) + ((r01 * x1310)) + (((5.02869162246205) * r10 * x1309)) + (((-5.02869162246205) * cj4 * r12)) + ((r00 * x1309)) + (((5.02869162246205) * r11 * x1310)));
                                                                    if (IKabs(j0eval[0]) < 0.0000010000000000)
                                                                    {
                                                                        continue; // no branches [j0]
                                                                    }
                                                                    else
                                                                    {
                                                                        {
                                                                            IkReal j0array[2], cj0array[2], sj0array[2];
                                                                            bool j0valid[2] = {false};
                                                                            _nj0 = 2;
                                                                            IkReal x1311 = cj4 * cj4;
                                                                            IkReal x1312 = r00 * r00;
                                                                            IkReal x1313 = cj5 * cj5;
                                                                            IkReal x1314 = r10 * r10;
                                                                            IkReal x1315 = r11 * r11;
                                                                            IkReal x1316 = r01 * r01;
                                                                            IkReal x1317 = ((0.195039861251953) * sj4);
                                                                            IkReal x1318 = (r00 * sj5);
                                                                            IkReal x1319 = (cj5 * r01);
                                                                            IkReal x1320 = ((0.382588364824742) * r12);
                                                                            IkReal x1321 = (cj4 * sj4);
                                                                            IkReal x1322 = (r10 * sj5);
                                                                            IkReal x1323 = ((1.92391890504564) * r12);
                                                                            IkReal x1324 = (cj5 * r11);
                                                                            IkReal x1325 = ((0.382588364824742) * r02);
                                                                            IkReal x1326 = ((0.980795316323859) * sj4);
                                                                            IkReal x1327 = ((0.0760810949543624) * r02);
                                                                            IkReal x1328 = ((0.382588364824742) * r00 * r10);
                                                                            IkReal x1329 = ((0.961959452522819) * x1314);
                                                                            IkReal x1330 = ((0.0380405474771812) * x1312);
                                                                            IkReal x1331 = ((0.0380405474771812) * x1316);
                                                                            IkReal x1332 = ((0.961959452522819) * x1315);
                                                                            IkReal x1333 = ((0.382588364824742) * r01 * r11);
                                                                            IkReal x1334 = ((0.382588364824742) * x1311);
                                                                            IkReal x1335 = (x1311 * x1313);
                                                                            CheckValue<IkReal> x1339 = IKPowWithIntegerCheck((((x1317 * x1319)) + ((x1317 * x1318)) + (((-0.195039861251953) * cj4 * r02)) + ((x1322 * x1326)) + (((-0.980795316323859) * cj4 * r12)) + ((x1324 * x1326))), -1);
                                                                            if (!x1339.valid)
                                                                            {
                                                                                continue;
                                                                            }
                                                                            IkReal x1336 = x1339.value;
                                                                            if ((((1.0) + ((x1313 * x1329)) + ((x1313 * x1328)) + ((x1318 * x1321 * x1327)) + ((x1318 * x1324 * x1334)) + (((-1.92391890504564) * x1322 * x1324)) + ((x1311 * x1330)) + ((x1331 * x1335)) + (((-0.0760810949543624) * x1318 * x1319)) + (((-1.0) * x1328 * x1335)) + (((-1.0) * x1329)) + (((-1.0) * x1328)) + ((x1318 * x1320 * x1321)) + ((x1319 * x1322 * x1334)) + (((-1.0) * x1330 * x1335)) + ((x1319 * x1320 * x1321)) + (((-0.382588364824742) * x1318 * x1324)) + (((-1.0) * x1329 * x1335)) + (((-1.0) * r02 * x1311 * x1320)) + (((-0.382588364824742) * x1319 * x1322)) + (((-0.961959452522819) * x1311 * (r12 * r12))) + ((x1319 * x1321 * x1327)) + (((-1.0) * x1313 * x1331)) + (((-1.0) * x1313 * x1332)) + (((-1.0) * x1313 * x1333)) + ((x1321 * x1323 * x1324)) + (((-0.0380405474771812) * x1311 * (r02 * r02))) + (((-1.0) * x1330)) + (((0.0760810949543624) * x1311 * x1318 * x1319)) + ((x1321 * x1322 * x1323)) + ((x1321 * x1322 * x1325)) + ((x1333 * x1335)) + ((x1332 * x1335)) + ((x1313 * x1330)) + (((1.92391890504564) * x1311 * x1322 * x1324)) + ((x1321 * x1324 * x1325)) + ((x1311 * x1328)) + ((x1311 * x1329)))) < -0.00001)
                                                                                continue;
                                                                            IkReal x1337 = IKsqrt(((1.0) + ((x1313 * x1329)) + ((x1313 * x1328)) + ((x1318 * x1321 * x1327)) + ((x1318 * x1324 * x1334)) + (((-1.92391890504564) * x1322 * x1324)) + ((x1311 * x1330)) + ((x1331 * x1335)) + (((-0.0760810949543624) * x1318 * x1319)) + (((-1.0) * x1328 * x1335)) + (((-1.0) * x1329)) + (((-1.0) * x1328)) + ((x1318 * x1320 * x1321)) + ((x1319 * x1322 * x1334)) + (((-1.0) * x1330 * x1335)) + ((x1319 * x1320 * x1321)) + (((-0.382588364824742) * x1318 * x1324)) + (((-1.0) * x1329 * x1335)) + (((-1.0) * r02 * x1311 * x1320)) + (((-0.382588364824742) * x1319 * x1322)) + (((-0.961959452522819) * x1311 * (r12 * r12))) + ((x1319 * x1321 * x1327)) + (((-1.0) * x1313 * x1331)) + (((-1.0) * x1313 * x1332)) + (((-1.0) * x1313 * x1333)) + ((x1321 * x1323 * x1324)) + (((-0.0380405474771812) * x1311 * (r02 * r02))) + (((-1.0) * x1330)) + (((0.0760810949543624) * x1311 * x1318 * x1319)) + ((x1321 * x1322 * x1323)) + ((x1321 * x1322 * x1325)) + ((x1333 * x1335)) + ((x1332 * x1335)) + ((x1313 * x1330)) + (((1.92391890504564) * x1311 * x1322 * x1324)) + ((x1321 * x1324 * x1325)) + ((x1311 * x1328)) + ((x1311 * x1329))));
                                                                            IkReal x1338 = (x1336 * x1337);
                                                                            j0array[0] = ((-2.0) * (atan((x1336 + (((-1.0) * x1338))))));
                                                                            sj0array[0] = IKsin(j0array[0]);
                                                                            cj0array[0] = IKcos(j0array[0]);
                                                                            j0array[1] = ((-2.0) * (atan((x1338 + x1336))));
                                                                            sj0array[1] = IKsin(j0array[1]);
                                                                            cj0array[1] = IKcos(j0array[1]);
                                                                            if (j0array[0] > IKPI)
                                                                            {
                                                                                j0array[0] -= IK2PI;
                                                                            }
                                                                            else if (j0array[0] < -IKPI)
                                                                            {
                                                                                j0array[0] += IK2PI;
                                                                            }
                                                                            j0valid[0] = true;
                                                                            if (j0array[1] > IKPI)
                                                                            {
                                                                                j0array[1] -= IK2PI;
                                                                            }
                                                                            else if (j0array[1] < -IKPI)
                                                                            {
                                                                                j0array[1] += IK2PI;
                                                                            }
                                                                            j0valid[1] = true;
                                                                            for (int ij0 = 0; ij0 < 2; ++ij0)
                                                                            {
                                                                                if (!j0valid[ij0])
                                                                                {
                                                                                    continue;
                                                                                }
                                                                                _ij0[0] = ij0;
                                                                                _ij0[1] = -1;
                                                                                for (int iij0 = ij0 + 1; iij0 < 2; ++iij0)
                                                                                {
                                                                                    if (j0valid[iij0] && IKabs(cj0array[ij0] - cj0array[iij0]) < IKFAST_SOLUTION_THRESH && IKabs(sj0array[ij0] - sj0array[iij0]) < IKFAST_SOLUTION_THRESH)
                                                                                    {
                                                                                        j0valid[iij0] = false;
                                                                                        _ij0[1] = iij0;
                                                                                        break;
                                                                                    }
                                                                                }
                                                                                j0 = j0array[ij0];
                                                                                cj0 = cj0array[ij0];
                                                                                sj0 = sj0array[ij0];

                                                                                innerfn(solutions);
                                                                            }
                                                                        }
                                                                    }
                                                                }
                                                            }
                                                            else
                                                            {
                                                                {
                                                                    IkReal j0array[1], cj0array[1], sj0array[1];
                                                                    bool j0valid[1] = {false};
                                                                    _nj0 = 1;
                                                                    IkReal x1340 = (cj5 * sj4);
                                                                    IkReal x1341 = (sj4 * sj5);
                                                                    CheckValue<IkReal> x1344 = IKPowWithIntegerCheck(((-0.195039861251953) + (((-1.0) * r10 * x1341)) + (((-1.0) * r11 * x1340)) + ((cj4 * r12))), -1);
                                                                    if (!x1344.valid)
                                                                    {
                                                                        continue;
                                                                    }
                                                                    IkReal x1342 = x1344.value;
                                                                    IkReal x1343 = (sj4 * x1342);
                                                                    CheckValue<IkReal> x1345 = IKPowWithIntegerCheck(((-0.195039861251953) + (((-1.0) * r10 * sj4 * sj5)) + (((-1.0) * r11 * x1340)) + ((cj4 * r12))), -1);
                                                                    if (!x1345.valid)
                                                                    {
                                                                        continue;
                                                                    }
                                                                    CheckValue<IkReal> x1346 = IKPowWithIntegerCheck(((-0.195039861251953) + (((-1.0) * r10 * x1341)) + (((-1.0) * cj5 * r11 * sj4)) + ((cj4 * r12))), -1);
                                                                    if (!x1346.valid)
                                                                    {
                                                                        continue;
                                                                    }
                                                                    j0array[0] = ((2.0) * (atan(((((0.980795316323859) * x1342)) + ((r00 * x1341 * (x1345.value))) + (((-1.0) * cj4 * r02 * x1342)) + ((r01 * x1340 * (x1346.value)))))));
                                                                    sj0array[0] = IKsin(j0array[0]);
                                                                    cj0array[0] = IKcos(j0array[0]);
                                                                    if (j0array[0] > IKPI)
                                                                    {
                                                                        j0array[0] -= IK2PI;
                                                                    }
                                                                    else if (j0array[0] < -IKPI)
                                                                    {
                                                                        j0array[0] += IK2PI;
                                                                    }
                                                                    j0valid[0] = true;
                                                                    for (int ij0 = 0; ij0 < 1; ++ij0)
                                                                    {
                                                                        if (!j0valid[ij0])
                                                                        {
                                                                            continue;
                                                                        }
                                                                        _ij0[0] = ij0;
                                                                        _ij0[1] = -1;
                                                                        for (int iij0 = ij0 + 1; iij0 < 1; ++iij0)
                                                                        {
                                                                            if (j0valid[iij0] && IKabs(cj0array[ij0] - cj0array[iij0]) < IKFAST_SOLUTION_THRESH && IKabs(sj0array[ij0] - sj0array[iij0]) < IKFAST_SOLUTION_THRESH)
                                                                            {
                                                                                j0valid[iij0] = false;
                                                                                _ij0[1] = iij0;
                                                                                break;
                                                                            }
                                                                        }
                                                                        j0 = j0array[ij0];
                                                                        cj0 = cj0array[ij0];
                                                                        sj0 = sj0array[ij0];

                                                                        innerfn(solutions);
                                                                    }
                                                                }
                                                            }
                                                        }
                                                    }
                                                    else
                                                    {
                                                        {
                                                            IkReal j0array[1], cj0array[1], sj0array[1];
                                                            bool j0valid[1] = {false};
                                                            _nj0 = 1;
                                                            IkReal x1347 = ((1.0) * cj4);
                                                            IkReal x1348 = (sj4 * sj5);
                                                            IkReal x1349 = (cj5 * sj4);
                                                            CheckValue<IkReal> x1351 = IKPowWithIntegerCheck(((-0.980795316323859) + (((-1.0) * r02 * x1347)) + ((r00 * x1348)) + ((r01 * x1349))), -1);
                                                            if (!x1351.valid)
                                                            {
                                                                continue;
                                                            }
                                                            IkReal x1350 = x1351.value;
                                                            CheckValue<IkReal> x1352 = IKPowWithIntegerCheck(((-0.980795316323859) + (((-1.0) * cj4 * r02)) + ((r00 * x1348)) + ((r01 * x1349))), -1);
                                                            if (!x1352.valid)
                                                            {
                                                                continue;
                                                            }
                                                            CheckValue<IkReal> x1353 = IKPowWithIntegerCheck(((-0.980795316323859) + ((r00 * sj4 * sj5)) + (((-1.0) * r02 * x1347)) + ((r01 * x1349))), -1);
                                                            if (!x1353.valid)
                                                            {
                                                                continue;
                                                            }
                                                            CheckValue<IkReal> x1354 = IKPowWithIntegerCheck(((-0.980795316323859) + (((-1.0) * r02 * x1347)) + ((cj5 * r01 * sj4)) + ((r00 * x1348))), -1);
                                                            if (!x1354.valid)
                                                            {
                                                                continue;
                                                            }
                                                            j0array[0] = ((2.0) * (atan(((((-1.0) * r12 * x1347 * (x1352.value))) + (((-0.195039861251953) * x1350)) + ((r10 * x1348 * (x1353.value))) + ((r11 * x1349 * (x1354.value)))))));
                                                            sj0array[0] = IKsin(j0array[0]);
                                                            cj0array[0] = IKcos(j0array[0]);
                                                            if (j0array[0] > IKPI)
                                                            {
                                                                j0array[0] -= IK2PI;
                                                            }
                                                            else if (j0array[0] < -IKPI)
                                                            {
                                                                j0array[0] += IK2PI;
                                                            }
                                                            j0valid[0] = true;
                                                            for (int ij0 = 0; ij0 < 1; ++ij0)
                                                            {
                                                                if (!j0valid[ij0])
                                                                {
                                                                    continue;
                                                                }
                                                                _ij0[0] = ij0;
                                                                _ij0[1] = -1;
                                                                for (int iij0 = ij0 + 1; iij0 < 1; ++iij0)
                                                                {
                                                                    if (j0valid[iij0] && IKabs(cj0array[ij0] - cj0array[iij0]) < IKFAST_SOLUTION_THRESH && IKabs(sj0array[ij0] - sj0array[iij0]) < IKFAST_SOLUTION_THRESH)
                                                                    {
                                                                        j0valid[iij0] = false;
                                                                        _ij0[1] = iij0;
                                                                        break;
                                                                    }
                                                                }
                                                                j0 = j0array[ij0];
                                                                cj0 = cj0array[ij0];
                                                                sj0 = sj0array[ij0];

                                                                innerfn(solutions);
                                                            }
                                                        }
                                                    }
                                                }
                                            }
                                        }
                                    }
                                } while (0);
                                if (bgotonextstatement)
                                {
                                    bool bgotonextstatement = true;
                                    do
                                    {
                                        evalcond[0] = ((IKabs(r21)) + (IKabs(r20)));
                                        if (IKabs(evalcond[0]) < 0.0000050000000000)
                                        {
                                            bgotonextstatement = false;
                                            {
                                                IkReal j4eval[2];
                                                IkReal x1355 = ((-1.0) * pz);
                                                r20 = 0;
                                                r21 = 0;
                                                r02 = 0;
                                                r12 = 0;
                                                npx = (((px * r00)) + ((py * r10)));
                                                npy = (((px * r01)) + ((py * r11)));
                                                npz = (pz * r22);
                                                rxp0_0 = (pz * r10);
                                                rxp0_1 = (r00 * x1355);
                                                rxp1_0 = (pz * r11);
                                                rxp1_1 = (r01 * x1355);
                                                rxp2_0 = ((-1.0) * py * r22);
                                                rxp2_1 = (px * r22);
                                                rxp2_2 = 0;
                                                IkReal x1356 = py * py;
                                                IkReal x1357 = sj5 * sj5;
                                                IkReal x1358 = cj5 * cj5;
                                                IkReal x1359 = px * px;
                                                IkReal x1360 = (px * r01);
                                                IkReal x1361 = ((1.0) * cj5);
                                                IkReal x1362 = ((14609203798393.0) * r11);
                                                IkReal x1363 = (px * r00);
                                                IkReal x1364 = ((5405405.4054054) * sj5);
                                                IkReal x1365 = ((14609203798393.0) * py);
                                                IkReal x1366 = (py * sj5);
                                                IkReal x1367 = ((5405405.4054054) * cj5);
                                                IkReal x1368 = (py * r11);
                                                IkReal x1369 = ((7304601899196.49) * x1356);
                                                IkReal x1370 = (cj5 * r10 * sj5);
                                                IkReal x1371 = ((7304601899196.49) * x1359);
                                                j4eval[0] = ((1.0) + ((py * x1358 * x1360 * x1362)) + ((x1360 * x1365 * x1370)) + ((x1358 * x1369 * (r11 * r11))) + ((x1357 * x1369 * (r10 * r10))) + ((x1357 * x1371 * (r00 * r00))) + ((cj5 * x1362 * x1363 * x1366)) + (((7304601899196.49) * (pz * pz) * (r22 * r22))) + (((-1.0) * py * r10 * x1364)) + (((14609203798393.0) * cj5 * r00 * r01 * sj5 * x1359)) + ((x1356 * x1362 * x1370)) + ((r10 * x1357 * x1363 * x1365)) + ((x1358 * x1371 * (r01 * r01))) + (((-1.0) * x1363 * x1364)) + (((-1.0) * x1360 * x1367)) + (((-1.0) * x1367 * x1368)));
                                                j4eval[1] = ((IKabs(((3.7e-7) + (((-1.0) * r10 * x1366)) + (((-1.0) * sj5 * x1363)) + (((-1.0) * x1361 * x1368)) + (((-1.0) * x1360 * x1361))))) + (IKabs((pz * r22))));
                                                if (IKabs(j4eval[0]) < 0.0000010000000000 || IKabs(j4eval[1]) < 0.0000010000000000)
                                                {
                                                    continue; // no branches [j4]
                                                }
                                                else
                                                {
                                                    {
                                                        IkReal j4array[2], cj4array[2], sj4array[2];
                                                        bool j4valid[2] = {false};
                                                        _nj4 = 2;
                                                        IkReal x1372 = ((1.0) * cj5);
                                                        IkReal x1373 = ((1.0) * sj5);
                                                        IkReal x1374 = ((3.7e-7) + (((-1.0) * px * r00 * x1373)) + (((-1.0) * px * r01 * x1372)) + (((-1.0) * py * r10 * x1373)) + (((-1.0) * py * r11 * x1372)));
                                                        CheckValue<IkReal> x1377 = IKatan2WithCheck(IkReal((pz * r22)), IkReal(x1374), IKFAST_ATAN2_MAGTHRESH);
                                                        if (!x1377.valid)
                                                        {
                                                            continue;
                                                        }
                                                        IkReal x1375 = ((1.0) * (x1377.value));
                                                        if ((((x1374 * x1374) + (((pz * pz) * (r22 * r22))))) < -0.00001)
                                                            continue;
                                                        CheckValue<IkReal> x1378 = IKPowWithIntegerCheck(IKabs(IKsqrt(((x1374 * x1374) + (((pz * pz) * (r22 * r22)))))), -1);
                                                        if (!x1378.valid)
                                                        {
                                                            continue;
                                                        }
                                                        if ((((0.113399998709592) * (x1378.value))) < -1 - IKFAST_SINCOS_THRESH || (((0.113399998709592) * (x1378.value))) > 1 + IKFAST_SINCOS_THRESH)
                                                            continue;
                                                        IkReal x1376 = IKasin(((0.113399998709592) * (x1378.value)));
                                                        j4array[0] = (x1376 + (((-1.0) * x1375)));
                                                        sj4array[0] = IKsin(j4array[0]);
                                                        cj4array[0] = IKcos(j4array[0]);
                                                        j4array[1] = ((3.14159265358979) + (((-1.0) * x1376)) + (((-1.0) * x1375)));
                                                        sj4array[1] = IKsin(j4array[1]);
                                                        cj4array[1] = IKcos(j4array[1]);
                                                        if (j4array[0] > IKPI)
                                                        {
                                                            j4array[0] -= IK2PI;
                                                        }
                                                        else if (j4array[0] < -IKPI)
                                                        {
                                                            j4array[0] += IK2PI;
                                                        }
                                                        j4valid[0] = true;
                                                        if (j4array[1] > IKPI)
                                                        {
                                                            j4array[1] -= IK2PI;
                                                        }
                                                        else if (j4array[1] < -IKPI)
                                                        {
                                                            j4array[1] += IK2PI;
                                                        }
                                                        j4valid[1] = true;
                                                        for (int ij4 = 0; ij4 < 2; ++ij4)
                                                        {
                                                            if (!j4valid[ij4])
                                                            {
                                                                continue;
                                                            }
                                                            _ij4[0] = ij4;
                                                            _ij4[1] = -1;
                                                            for (int iij4 = ij4 + 1; iij4 < 2; ++iij4)
                                                            {
                                                                if (j4valid[iij4] && IKabs(cj4array[ij4] - cj4array[iij4]) < IKFAST_SOLUTION_THRESH && IKabs(sj4array[ij4] - sj4array[iij4]) < IKFAST_SOLUTION_THRESH)
                                                                {
                                                                    j4valid[iij4] = false;
                                                                    _ij4[1] = iij4;
                                                                    break;
                                                                }
                                                            }
                                                            j4 = j4array[ij4];
                                                            cj4 = cj4array[ij4];
                                                            sj4 = sj4array[ij4];
                                                            {
                                                                IkReal evalcond[1];
                                                                evalcond[0] = (r22 * (IKcos(j4)));
                                                                if (IKabs(evalcond[0]) > IKFAST_EVALCOND_THRESH)
                                                                {
                                                                    continue;
                                                                }
                                                            }

                                                            {
                                                                IkReal j0eval[1];
                                                                IkReal x1379 = ((1.01958072531191) * sj4);
                                                                j0eval[0] = ((-1.0) + (((-1.01958072531191) * cj4 * r02)) + ((r00 * sj5 * x1379)) + ((cj5 * r01 * x1379)));
                                                                if (IKabs(j0eval[0]) < 0.0000010000000000)
                                                                {
                                                                    {
                                                                        IkReal j0eval[1];
                                                                        IkReal x1380 = ((5.12715705179976) * sj4);
                                                                        j0eval[0] = ((-1.0) + (((5.12715705179976) * cj4 * r12)) + (((-1.0) * cj5 * r11 * x1380)) + (((-1.0) * r10 * sj5 * x1380)));
                                                                        if (IKabs(j0eval[0]) < 0.0000010000000000)
                                                                        {
                                                                            {
                                                                                IkReal j0eval[1];
                                                                                IkReal x1381 = (sj4 * sj5);
                                                                                IkReal x1382 = (cj5 * sj4);
                                                                                j0eval[0] = ((((-1.0) * cj4 * r02)) + (((5.02869162246205) * r11 * x1382)) + (((5.02869162246205) * r10 * x1381)) + (((-5.02869162246205) * cj4 * r12)) + ((r00 * x1381)) + ((r01 * x1382)));
                                                                                if (IKabs(j0eval[0]) < 0.0000010000000000)
                                                                                {
                                                                                    continue; // no branches [j0]
                                                                                }
                                                                                else
                                                                                {
                                                                                    {
                                                                                        IkReal j0array[2], cj0array[2], sj0array[2];
                                                                                        bool j0valid[2] = {false};
                                                                                        _nj0 = 2;
                                                                                        IkReal x1383 = cj4 * cj4;
                                                                                        IkReal x1384 = r00 * r00;
                                                                                        IkReal x1385 = cj5 * cj5;
                                                                                        IkReal x1386 = r10 * r10;
                                                                                        IkReal x1387 = r11 * r11;
                                                                                        IkReal x1388 = r01 * r01;
                                                                                        IkReal x1389 = ((0.195039861251953) * sj4);
                                                                                        IkReal x1390 = (r00 * sj5);
                                                                                        IkReal x1391 = (cj5 * r01);
                                                                                        IkReal x1392 = ((0.382588364824742) * r12);
                                                                                        IkReal x1393 = (cj4 * sj4);
                                                                                        IkReal x1394 = (r10 * sj5);
                                                                                        IkReal x1395 = ((1.92391890504564) * r12);
                                                                                        IkReal x1396 = (cj5 * r11);
                                                                                        IkReal x1397 = ((0.382588364824742) * r02);
                                                                                        IkReal x1398 = ((0.980795316323859) * sj4);
                                                                                        IkReal x1399 = ((0.0760810949543624) * r02);
                                                                                        IkReal x1400 = ((0.382588364824742) * r00 * r10);
                                                                                        IkReal x1401 = ((0.961959452522819) * x1386);
                                                                                        IkReal x1402 = ((0.0380405474771812) * x1384);
                                                                                        IkReal x1403 = ((0.0380405474771812) * x1388);
                                                                                        IkReal x1404 = ((0.961959452522819) * x1387);
                                                                                        IkReal x1405 = ((0.382588364824742) * r01 * r11);
                                                                                        IkReal x1406 = ((0.382588364824742) * x1383);
                                                                                        IkReal x1407 = (x1383 * x1385);
                                                                                        CheckValue<IkReal> x1411 = IKPowWithIntegerCheck(((((-0.195039861251953) * cj4 * r02)) + ((x1394 * x1398)) + (((-0.980795316323859) * cj4 * r12)) + ((x1396 * x1398)) + ((x1389 * x1390)) + ((x1389 * x1391))), -1);
                                                                                        if (!x1411.valid)
                                                                                        {
                                                                                            continue;
                                                                                        }
                                                                                        IkReal x1408 = x1411.value;
                                                                                        if ((((1.0) + ((x1391 * x1392 * x1393)) + (((-0.961959452522819) * x1383 * (r12 * r12))) + ((x1385 * x1400)) + ((x1385 * x1401)) + ((x1385 * x1402)) + (((-1.92391890504564) * x1394 * x1396)) + ((x1391 * x1393 * x1399)) + (((0.0760810949543624) * x1383 * x1390 * x1391)) + ((x1393 * x1394 * x1395)) + ((x1393 * x1394 * x1397)) + (((1.92391890504564) * x1383 * x1394 * x1396)) + ((x1393 * x1395 * x1396)) + ((x1391 * x1394 * x1406)) + (((-1.0) * x1400)) + (((-1.0) * x1401)) + (((-1.0) * x1402)) + ((x1393 * x1396 * x1397)) + (((-0.382588364824742) * x1391 * x1394)) + ((x1390 * x1393 * x1399)) + ((x1405 * x1407)) + (((-1.0) * x1402 * x1407)) + ((x1403 * x1407)) + (((-0.0380405474771812) * x1383 * (r02 * r02))) + ((x1404 * x1407)) + ((x1390 * x1392 * x1393)) + (((-1.0) * x1401 * x1407)) + (((-1.0) * x1400 * x1407)) + (((-1.0) * r02 * x1383 * x1392)) + (((-1.0) * x1385 * x1404)) + (((-1.0) * x1385 * x1405)) + (((-1.0) * x1385 * x1403)) + ((x1390 * x1396 * x1406)) + (((-0.0760810949543624) * x1390 * x1391)) + ((x1383 * x1400)) + ((x1383 * x1402)) + ((x1383 * x1401)) + (((-0.382588364824742) * x1390 * x1396)))) < -0.00001)
                                                                                            continue;
                                                                                        IkReal x1409 = IKsqrt(((1.0) + ((x1391 * x1392 * x1393)) + (((-0.961959452522819) * x1383 * (r12 * r12))) + ((x1385 * x1400)) + ((x1385 * x1401)) + ((x1385 * x1402)) + (((-1.92391890504564) * x1394 * x1396)) + ((x1391 * x1393 * x1399)) + (((0.0760810949543624) * x1383 * x1390 * x1391)) + ((x1393 * x1394 * x1395)) + ((x1393 * x1394 * x1397)) + (((1.92391890504564) * x1383 * x1394 * x1396)) + ((x1393 * x1395 * x1396)) + ((x1391 * x1394 * x1406)) + (((-1.0) * x1400)) + (((-1.0) * x1401)) + (((-1.0) * x1402)) + ((x1393 * x1396 * x1397)) + (((-0.382588364824742) * x1391 * x1394)) + ((x1390 * x1393 * x1399)) + ((x1405 * x1407)) + (((-1.0) * x1402 * x1407)) + ((x1403 * x1407)) + (((-0.0380405474771812) * x1383 * (r02 * r02))) + ((x1404 * x1407)) + ((x1390 * x1392 * x1393)) + (((-1.0) * x1401 * x1407)) + (((-1.0) * x1400 * x1407)) + (((-1.0) * r02 * x1383 * x1392)) + (((-1.0) * x1385 * x1404)) + (((-1.0) * x1385 * x1405)) + (((-1.0) * x1385 * x1403)) + ((x1390 * x1396 * x1406)) + (((-0.0760810949543624) * x1390 * x1391)) + ((x1383 * x1400)) + ((x1383 * x1402)) + ((x1383 * x1401)) + (((-0.382588364824742) * x1390 * x1396))));
                                                                                        IkReal x1410 = (x1408 * x1409);
                                                                                        j0array[0] = ((2.0) * (atan(((((1.0) * x1410)) + (((-1.0) * x1408))))));
                                                                                        sj0array[0] = IKsin(j0array[0]);
                                                                                        cj0array[0] = IKcos(j0array[0]);
                                                                                        j0array[1] = ((-2.0) * (atan((x1410 + x1408))));
                                                                                        sj0array[1] = IKsin(j0array[1]);
                                                                                        cj0array[1] = IKcos(j0array[1]);
                                                                                        if (j0array[0] > IKPI)
                                                                                        {
                                                                                            j0array[0] -= IK2PI;
                                                                                        }
                                                                                        else if (j0array[0] < -IKPI)
                                                                                        {
                                                                                            j0array[0] += IK2PI;
                                                                                        }
                                                                                        j0valid[0] = true;
                                                                                        if (j0array[1] > IKPI)
                                                                                        {
                                                                                            j0array[1] -= IK2PI;
                                                                                        }
                                                                                        else if (j0array[1] < -IKPI)
                                                                                        {
                                                                                            j0array[1] += IK2PI;
                                                                                        }
                                                                                        j0valid[1] = true;
                                                                                        for (int ij0 = 0; ij0 < 2; ++ij0)
                                                                                        {
                                                                                            if (!j0valid[ij0])
                                                                                            {
                                                                                                continue;
                                                                                            }
                                                                                            _ij0[0] = ij0;
                                                                                            _ij0[1] = -1;
                                                                                            for (int iij0 = ij0 + 1; iij0 < 2; ++iij0)
                                                                                            {
                                                                                                if (j0valid[iij0] && IKabs(cj0array[ij0] - cj0array[iij0]) < IKFAST_SOLUTION_THRESH && IKabs(sj0array[ij0] - sj0array[iij0]) < IKFAST_SOLUTION_THRESH)
                                                                                                {
                                                                                                    j0valid[iij0] = false;
                                                                                                    _ij0[1] = iij0;
                                                                                                    break;
                                                                                                }
                                                                                            }
                                                                                            j0 = j0array[ij0];
                                                                                            cj0 = cj0array[ij0];
                                                                                            sj0 = sj0array[ij0];

                                                                                            innerfn(solutions);
                                                                                        }
                                                                                    }
                                                                                }
                                                                            }
                                                                        }
                                                                        else
                                                                        {
                                                                            {
                                                                                IkReal j0array[1], cj0array[1], sj0array[1];
                                                                                bool j0valid[1] = {false};
                                                                                _nj0 = 1;
                                                                                IkReal x1412 = (cj5 * sj4);
                                                                                IkReal x1413 = (sj4 * sj5);
                                                                                CheckValue<IkReal> x1416 = IKPowWithIntegerCheck(((-0.195039861251953) + (((-1.0) * r10 * x1413)) + (((-1.0) * r11 * x1412)) + ((cj4 * r12))), -1);
                                                                                if (!x1416.valid)
                                                                                {
                                                                                    continue;
                                                                                }
                                                                                IkReal x1414 = x1416.value;
                                                                                IkReal x1415 = (sj4 * x1414);
                                                                                CheckValue<IkReal> x1417 = IKPowWithIntegerCheck(((-0.195039861251953) + (((-1.0) * r10 * x1413)) + (((-1.0) * cj5 * r11 * sj4)) + ((cj4 * r12))), -1);
                                                                                if (!x1417.valid)
                                                                                {
                                                                                    continue;
                                                                                }
                                                                                CheckValue<IkReal> x1418 = IKPowWithIntegerCheck(((-0.195039861251953) + (((-1.0) * r10 * sj4 * sj5)) + (((-1.0) * r11 * x1412)) + ((cj4 * r12))), -1);
                                                                                if (!x1418.valid)
                                                                                {
                                                                                    continue;
                                                                                }
                                                                                j0array[0] = ((2.0) * (atan(((((-1.0) * cj4 * r02 * x1414)) + (((0.980795316323859) * x1414)) + ((r01 * x1412 * (x1417.value))) + ((r00 * x1413 * (x1418.value)))))));
                                                                                sj0array[0] = IKsin(j0array[0]);
                                                                                cj0array[0] = IKcos(j0array[0]);
                                                                                if (j0array[0] > IKPI)
                                                                                {
                                                                                    j0array[0] -= IK2PI;
                                                                                }
                                                                                else if (j0array[0] < -IKPI)
                                                                                {
                                                                                    j0array[0] += IK2PI;
                                                                                }
                                                                                j0valid[0] = true;
                                                                                for (int ij0 = 0; ij0 < 1; ++ij0)
                                                                                {
                                                                                    if (!j0valid[ij0])
                                                                                    {
                                                                                        continue;
                                                                                    }
                                                                                    _ij0[0] = ij0;
                                                                                    _ij0[1] = -1;
                                                                                    for (int iij0 = ij0 + 1; iij0 < 1; ++iij0)
                                                                                    {
                                                                                        if (j0valid[iij0] && IKabs(cj0array[ij0] - cj0array[iij0]) < IKFAST_SOLUTION_THRESH && IKabs(sj0array[ij0] - sj0array[iij0]) < IKFAST_SOLUTION_THRESH)
                                                                                        {
                                                                                            j0valid[iij0] = false;
                                                                                            _ij0[1] = iij0;
                                                                                            break;
                                                                                        }
                                                                                    }
                                                                                    j0 = j0array[ij0];
                                                                                    cj0 = cj0array[ij0];
                                                                                    sj0 = sj0array[ij0];

                                                                                    innerfn(solutions);
                                                                                }
                                                                            }
                                                                        }
                                                                    }
                                                                }
                                                                else
                                                                {
                                                                    {
                                                                        IkReal j0array[1], cj0array[1], sj0array[1];
                                                                        bool j0valid[1] = {false};
                                                                        _nj0 = 1;
                                                                        IkReal x1419 = ((1.0) * cj4);
                                                                        IkReal x1420 = (sj4 * sj5);
                                                                        IkReal x1421 = (cj5 * sj4);
                                                                        CheckValue<IkReal> x1423 = IKPowWithIntegerCheck(((-0.980795316323859) + ((r01 * x1421)) + ((r00 * x1420)) + (((-1.0) * r02 * x1419))), -1);
                                                                        if (!x1423.valid)
                                                                        {
                                                                            continue;
                                                                        }
                                                                        IkReal x1422 = x1423.value;
                                                                        CheckValue<IkReal> x1424 = IKPowWithIntegerCheck(((-0.980795316323859) + ((r01 * x1421)) + ((r00 * sj4 * sj5)) + (((-1.0) * r02 * x1419))), -1);
                                                                        if (!x1424.valid)
                                                                        {
                                                                            continue;
                                                                        }
                                                                        CheckValue<IkReal> x1425 = IKPowWithIntegerCheck(((-0.980795316323859) + (((-1.0) * cj4 * r02)) + ((r01 * x1421)) + ((r00 * x1420))), -1);
                                                                        if (!x1425.valid)
                                                                        {
                                                                            continue;
                                                                        }
                                                                        CheckValue<IkReal> x1426 = IKPowWithIntegerCheck(((-0.980795316323859) + ((r00 * x1420)) + ((cj5 * r01 * sj4)) + (((-1.0) * r02 * x1419))), -1);
                                                                        if (!x1426.valid)
                                                                        {
                                                                            continue;
                                                                        }
                                                                        j0array[0] = ((-2.0) * (atan(((((-1.0) * r10 * x1420 * (x1424.value))) + ((r12 * x1419 * (x1425.value))) + (((0.195039861251953) * x1422)) + (((-1.0) * r11 * x1421 * (x1426.value)))))));
                                                                        sj0array[0] = IKsin(j0array[0]);
                                                                        cj0array[0] = IKcos(j0array[0]);
                                                                        if (j0array[0] > IKPI)
                                                                        {
                                                                            j0array[0] -= IK2PI;
                                                                        }
                                                                        else if (j0array[0] < -IKPI)
                                                                        {
                                                                            j0array[0] += IK2PI;
                                                                        }
                                                                        j0valid[0] = true;
                                                                        for (int ij0 = 0; ij0 < 1; ++ij0)
                                                                        {
                                                                            if (!j0valid[ij0])
                                                                            {
                                                                                continue;
                                                                            }
                                                                            _ij0[0] = ij0;
                                                                            _ij0[1] = -1;
                                                                            for (int iij0 = ij0 + 1; iij0 < 1; ++iij0)
                                                                            {
                                                                                if (j0valid[iij0] && IKabs(cj0array[ij0] - cj0array[iij0]) < IKFAST_SOLUTION_THRESH && IKabs(sj0array[ij0] - sj0array[iij0]) < IKFAST_SOLUTION_THRESH)
                                                                                {
                                                                                    j0valid[iij0] = false;
                                                                                    _ij0[1] = iij0;
                                                                                    break;
                                                                                }
                                                                            }
                                                                            j0 = j0array[ij0];
                                                                            cj0 = cj0array[ij0];
                                                                            sj0 = sj0array[ij0];

                                                                            innerfn(solutions);
                                                                        }
                                                                    }
                                                                }
                                                            }
                                                        }
                                                    }
                                                }
                                            }
                                        }
                                    } while (0);
                                    if (bgotonextstatement)
                                    {
                                        bool bgotonextstatement = true;
                                        do
                                        {
                                            if (1)
                                            {
                                                bgotonextstatement = false;
                                                continue; // branch miss [j4]
                                            }
                                        } while (0);
                                        if (bgotonextstatement)
                                        {
                                        }
                                    }
                                }
                            }
                        }
                    }
                    else
                    {
                        {
                            IkReal j4array[1], cj4array[1], sj4array[1];
                            bool j4valid[1] = {false};
                            _nj4 = 1;
                            IkReal x1427 = (r20 * sj5);
                            IkReal x1428 = (cj5 * r21);
                            IkReal x1429 = ((1.0) * npz);
                            CheckValue<IkReal> x1430 = IKPowWithIntegerCheck(IKsign(((((-1.0) * x1427 * x1429)) + ((npx * r22 * sj5)) + ((cj5 * npy * r22)) + (((-3.7e-7) * r22)) + (((-1.0) * x1428 * x1429)))), -1);
                            if (!x1430.valid)
                            {
                                continue;
                            }
                            CheckValue<IkReal> x1431 = IKatan2WithCheck(IkReal(((-0.113399998709592) * r22)), IkReal(((((-0.113399998709592) * x1428)) + (((-0.113399998709592) * x1427)))), IKFAST_ATAN2_MAGTHRESH);
                            if (!x1431.valid)
                            {
                                continue;
                            }
                            j4array[0] = ((-1.5707963267949) + (((1.5707963267949) * (x1430.value))) + (x1431.value));
                            sj4array[0] = IKsin(j4array[0]);
                            cj4array[0] = IKcos(j4array[0]);
                            if (j4array[0] > IKPI)
                            {
                                j4array[0] -= IK2PI;
                            }
                            else if (j4array[0] < -IKPI)
                            {
                                j4array[0] += IK2PI;
                            }
                            j4valid[0] = true;
                            for (int ij4 = 0; ij4 < 1; ++ij4)
                            {
                                if (!j4valid[ij4])
                                {
                                    continue;
                                }
                                _ij4[0] = ij4;
                                _ij4[1] = -1;
                                for (int iij4 = ij4 + 1; iij4 < 1; ++iij4)
                                {
                                    if (j4valid[iij4] && IKabs(cj4array[ij4] - cj4array[iij4]) < IKFAST_SOLUTION_THRESH && IKabs(sj4array[ij4] - sj4array[iij4]) < IKFAST_SOLUTION_THRESH)
                                    {
                                        j4valid[iij4] = false;
                                        _ij4[1] = iij4;
                                        break;
                                    }
                                }
                                j4 = j4array[ij4];
                                cj4 = cj4array[ij4];
                                sj4 = sj4array[ij4];
                                {
                                    IkReal evalcond[2];
                                    IkReal x1432 = IKsin(j4);
                                    IkReal x1433 = IKcos(j4);
                                    IkReal x1434 = ((1.0) * x1432);
                                    evalcond[0] = ((((-1.0) * cj5 * r21 * x1434)) + (((-1.0) * r20 * sj5 * x1434)) + ((r22 * x1433)));
                                    evalcond[1] = ((-0.113399998709592) + (((-1.0) * npx * sj5 * x1434)) + (((-1.0) * cj5 * npy * x1434)) + ((npz * x1433)) + (((3.7e-7) * x1432)));
                                    if (IKabs(evalcond[0]) > IKFAST_EVALCOND_THRESH || IKabs(evalcond[1]) > IKFAST_EVALCOND_THRESH)
                                    {
                                        continue;
                                    }
                                }

                                {
                                    IkReal j0eval[1];
                                    IkReal x1435 = ((1.01958072531191) * sj4);
                                    j0eval[0] = ((-1.0) + (((-1.01958072531191) * cj4 * r02)) + ((r00 * sj5 * x1435)) + ((cj5 * r01 * x1435)));
                                    if (IKabs(j0eval[0]) < 0.0000010000000000)
                                    {
                                        {
                                            IkReal j0eval[1];
                                            IkReal x1436 = ((5.12715705179976) * sj4);
                                            j0eval[0] = ((-1.0) + (((5.12715705179976) * cj4 * r12)) + (((-1.0) * r10 * sj5 * x1436)) + (((-1.0) * cj5 * r11 * x1436)));
                                            if (IKabs(j0eval[0]) < 0.0000010000000000)
                                            {
                                                {
                                                    IkReal j0eval[1];
                                                    IkReal x1437 = (sj4 * sj5);
                                                    IkReal x1438 = (cj5 * sj4);
                                                    j0eval[0] = ((((-1.0) * cj4 * r02)) + (((5.02869162246205) * r10 * x1437)) + (((5.02869162246205) * r11 * x1438)) + ((r01 * x1438)) + ((r00 * x1437)) + (((-5.02869162246205) * cj4 * r12)));
                                                    if (IKabs(j0eval[0]) < 0.0000010000000000)
                                                    {
                                                        continue; // no branches [j0]
                                                    }
                                                    else
                                                    {
                                                        {
                                                            IkReal j0array[2], cj0array[2], sj0array[2];
                                                            bool j0valid[2] = {false};
                                                            _nj0 = 2;
                                                            IkReal x1439 = cj4 * cj4;
                                                            IkReal x1440 = r00 * r00;
                                                            IkReal x1441 = cj5 * cj5;
                                                            IkReal x1442 = r10 * r10;
                                                            IkReal x1443 = r11 * r11;
                                                            IkReal x1444 = r01 * r01;
                                                            IkReal x1445 = ((0.195039861251953) * sj4);
                                                            IkReal x1446 = (r00 * sj5);
                                                            IkReal x1447 = (cj5 * r01);
                                                            IkReal x1448 = ((0.382588364824742) * r12);
                                                            IkReal x1449 = (cj4 * sj4);
                                                            IkReal x1450 = (r10 * sj5);
                                                            IkReal x1451 = ((1.92391890504564) * r12);
                                                            IkReal x1452 = (cj5 * r11);
                                                            IkReal x1453 = ((0.382588364824742) * r02);
                                                            IkReal x1454 = ((0.980795316323859) * sj4);
                                                            IkReal x1455 = ((0.0760810949543624) * r02);
                                                            IkReal x1456 = ((0.382588364824742) * r00 * r10);
                                                            IkReal x1457 = ((0.961959452522819) * x1442);
                                                            IkReal x1458 = ((0.0380405474771812) * x1440);
                                                            IkReal x1459 = ((0.0380405474771812) * x1444);
                                                            IkReal x1460 = ((0.961959452522819) * x1443);
                                                            IkReal x1461 = ((0.382588364824742) * r01 * r11);
                                                            IkReal x1462 = ((0.382588364824742) * x1439);
                                                            IkReal x1463 = (x1439 * x1441);
                                                            CheckValue<IkReal> x1467 = IKPowWithIntegerCheck((((x1445 * x1447)) + ((x1445 * x1446)) + (((-0.195039861251953) * cj4 * r02)) + ((x1450 * x1454)) + (((-0.980795316323859) * cj4 * r12)) + ((x1452 * x1454))), -1);
                                                            if (!x1467.valid)
                                                            {
                                                                continue;
                                                            }
                                                            IkReal x1464 = x1467.value;
                                                            if ((((1.0) + ((x1446 * x1452 * x1462)) + (((-1.0) * x1441 * x1459)) + ((x1446 * x1449 * x1455)) + (((-1.0) * x1458 * x1463)) + (((-0.961959452522819) * x1439 * (r12 * r12))) + ((x1446 * x1448 * x1449)) + ((x1447 * x1449 * x1455)) + ((x1449 * x1452 * x1453)) + (((-1.0) * x1457)) + (((-1.0) * x1456)) + (((-1.0) * x1458)) + (((-1.0) * x1456 * x1463)) + (((-0.382588364824742) * x1446 * x1452)) + ((x1459 * x1463)) + ((x1461 * x1463)) + (((1.92391890504564) * x1439 * x1450 * x1452)) + ((x1439 * x1458)) + ((x1439 * x1457)) + ((x1439 * x1456)) + (((-0.382588364824742) * x1447 * x1450)) + ((x1460 * x1463)) + (((-1.0) * x1457 * x1463)) + (((-1.92391890504564) * x1450 * x1452)) + (((-0.0380405474771812) * x1439 * (r02 * r02))) + (((-1.0) * x1441 * x1461)) + (((-1.0) * x1441 * x1460)) + ((x1449 * x1451 * x1452)) + ((x1449 * x1450 * x1453)) + ((x1449 * x1450 * x1451)) + ((x1447 * x1450 * x1462)) + (((0.0760810949543624) * x1439 * x1446 * x1447)) + (((-0.0760810949543624) * x1446 * x1447)) + ((x1441 * x1456)) + ((x1441 * x1457)) + ((x1441 * x1458)) + (((-1.0) * r02 * x1439 * x1448)) + ((x1447 * x1448 * x1449)))) < -0.00001)
                                                                continue;
                                                            IkReal x1465 = IKsqrt(((1.0) + ((x1446 * x1452 * x1462)) + (((-1.0) * x1441 * x1459)) + ((x1446 * x1449 * x1455)) + (((-1.0) * x1458 * x1463)) + (((-0.961959452522819) * x1439 * (r12 * r12))) + ((x1446 * x1448 * x1449)) + ((x1447 * x1449 * x1455)) + ((x1449 * x1452 * x1453)) + (((-1.0) * x1457)) + (((-1.0) * x1456)) + (((-1.0) * x1458)) + (((-1.0) * x1456 * x1463)) + (((-0.382588364824742) * x1446 * x1452)) + ((x1459 * x1463)) + ((x1461 * x1463)) + (((1.92391890504564) * x1439 * x1450 * x1452)) + ((x1439 * x1458)) + ((x1439 * x1457)) + ((x1439 * x1456)) + (((-0.382588364824742) * x1447 * x1450)) + ((x1460 * x1463)) + (((-1.0) * x1457 * x1463)) + (((-1.92391890504564) * x1450 * x1452)) + (((-0.0380405474771812) * x1439 * (r02 * r02))) + (((-1.0) * x1441 * x1461)) + (((-1.0) * x1441 * x1460)) + ((x1449 * x1451 * x1452)) + ((x1449 * x1450 * x1453)) + ((x1449 * x1450 * x1451)) + ((x1447 * x1450 * x1462)) + (((0.0760810949543624) * x1439 * x1446 * x1447)) + (((-0.0760810949543624) * x1446 * x1447)) + ((x1441 * x1456)) + ((x1441 * x1457)) + ((x1441 * x1458)) + (((-1.0) * r02 * x1439 * x1448)) + ((x1447 * x1448 * x1449))));
                                                            IkReal x1466 = (x1464 * x1465);
                                                            j0array[0] = ((-2.0) * (atan((x1464 + (((-1.0) * x1466))))));
                                                            sj0array[0] = IKsin(j0array[0]);
                                                            cj0array[0] = IKcos(j0array[0]);
                                                            j0array[1] = ((-2.0) * (atan((x1464 + x1466))));
                                                            sj0array[1] = IKsin(j0array[1]);
                                                            cj0array[1] = IKcos(j0array[1]);
                                                            if (j0array[0] > IKPI)
                                                            {
                                                                j0array[0] -= IK2PI;
                                                            }
                                                            else if (j0array[0] < -IKPI)
                                                            {
                                                                j0array[0] += IK2PI;
                                                            }
                                                            j0valid[0] = true;
                                                            if (j0array[1] > IKPI)
                                                            {
                                                                j0array[1] -= IK2PI;
                                                            }
                                                            else if (j0array[1] < -IKPI)
                                                            {
                                                                j0array[1] += IK2PI;
                                                            }
                                                            j0valid[1] = true;
                                                            for (int ij0 = 0; ij0 < 2; ++ij0)
                                                            {
                                                                if (!j0valid[ij0])
                                                                {
                                                                    continue;
                                                                }
                                                                _ij0[0] = ij0;
                                                                _ij0[1] = -1;
                                                                for (int iij0 = ij0 + 1; iij0 < 2; ++iij0)
                                                                {
                                                                    if (j0valid[iij0] && IKabs(cj0array[ij0] - cj0array[iij0]) < IKFAST_SOLUTION_THRESH && IKabs(sj0array[ij0] - sj0array[iij0]) < IKFAST_SOLUTION_THRESH)
                                                                    {
                                                                        j0valid[iij0] = false;
                                                                        _ij0[1] = iij0;
                                                                        break;
                                                                    }
                                                                }
                                                                j0 = j0array[ij0];
                                                                cj0 = cj0array[ij0];
                                                                sj0 = sj0array[ij0];

                                                                innerfn(solutions);
                                                            }
                                                        }
                                                    }
                                                }
                                            }
                                            else
                                            {
                                                {
                                                    IkReal j0array[1], cj0array[1], sj0array[1];
                                                    bool j0valid[1] = {false};
                                                    _nj0 = 1;
                                                    IkReal x1468 = (cj5 * sj4);
                                                    IkReal x1469 = (sj4 * sj5);
                                                    CheckValue<IkReal> x1472 = IKPowWithIntegerCheck(((-0.195039861251953) + (((-1.0) * r11 * x1468)) + (((-1.0) * r10 * x1469)) + ((cj4 * r12))), -1);
                                                    if (!x1472.valid)
                                                    {
                                                        continue;
                                                    }
                                                    IkReal x1470 = x1472.value;
                                                    IkReal x1471 = (sj4 * x1470);
                                                    CheckValue<IkReal> x1473 = IKPowWithIntegerCheck(((-0.195039861251953) + (((-1.0) * r11 * x1468)) + (((-1.0) * r10 * sj4 * sj5)) + ((cj4 * r12))), -1);
                                                    if (!x1473.valid)
                                                    {
                                                        continue;
                                                    }
                                                    CheckValue<IkReal> x1474 = IKPowWithIntegerCheck(((-0.195039861251953) + (((-1.0) * r10 * x1469)) + (((-1.0) * cj5 * r11 * sj4)) + ((cj4 * r12))), -1);
                                                    if (!x1474.valid)
                                                    {
                                                        continue;
                                                    }
                                                    j0array[0] = ((2.0) * (atan((((r00 * x1469 * (x1473.value))) + (((0.980795316323859) * x1470)) + (((-1.0) * cj4 * r02 * x1470)) + ((r01 * x1468 * (x1474.value)))))));
                                                    sj0array[0] = IKsin(j0array[0]);
                                                    cj0array[0] = IKcos(j0array[0]);
                                                    if (j0array[0] > IKPI)
                                                    {
                                                        j0array[0] -= IK2PI;
                                                    }
                                                    else if (j0array[0] < -IKPI)
                                                    {
                                                        j0array[0] += IK2PI;
                                                    }
                                                    j0valid[0] = true;
                                                    for (int ij0 = 0; ij0 < 1; ++ij0)
                                                    {
                                                        if (!j0valid[ij0])
                                                        {
                                                            continue;
                                                        }
                                                        _ij0[0] = ij0;
                                                        _ij0[1] = -1;
                                                        for (int iij0 = ij0 + 1; iij0 < 1; ++iij0)
                                                        {
                                                            if (j0valid[iij0] && IKabs(cj0array[ij0] - cj0array[iij0]) < IKFAST_SOLUTION_THRESH && IKabs(sj0array[ij0] - sj0array[iij0]) < IKFAST_SOLUTION_THRESH)
                                                            {
                                                                j0valid[iij0] = false;
                                                                _ij0[1] = iij0;
                                                                break;
                                                            }
                                                        }
                                                        j0 = j0array[ij0];
                                                        cj0 = cj0array[ij0];
                                                        sj0 = sj0array[ij0];

                                                        innerfn(solutions);
                                                    }
                                                }
                                            }
                                        }
                                    }
                                    else
                                    {
                                        {
                                            IkReal j0array[1], cj0array[1], sj0array[1];
                                            bool j0valid[1] = {false};
                                            _nj0 = 1;
                                            IkReal x1475 = ((1.0) * cj4);
                                            IkReal x1476 = (sj4 * sj5);
                                            IkReal x1477 = (cj5 * sj4);
                                            CheckValue<IkReal> x1479 = IKPowWithIntegerCheck(((-0.980795316323859) + (((-1.0) * r02 * x1475)) + ((r01 * x1477)) + ((r00 * x1476))), -1);
                                            if (!x1479.valid)
                                            {
                                                continue;
                                            }
                                            IkReal x1478 = x1479.value;
                                            CheckValue<IkReal> x1480 = IKPowWithIntegerCheck(((-0.980795316323859) + ((cj5 * r01 * sj4)) + (((-1.0) * r02 * x1475)) + ((r00 * x1476))), -1);
                                            if (!x1480.valid)
                                            {
                                                continue;
                                            }
                                            CheckValue<IkReal> x1481 = IKPowWithIntegerCheck(((-0.980795316323859) + ((r00 * sj4 * sj5)) + (((-1.0) * r02 * x1475)) + ((r01 * x1477))), -1);
                                            if (!x1481.valid)
                                            {
                                                continue;
                                            }
                                            CheckValue<IkReal> x1482 = IKPowWithIntegerCheck(((-0.980795316323859) + (((-1.0) * cj4 * r02)) + ((r01 * x1477)) + ((r00 * x1476))), -1);
                                            if (!x1482.valid)
                                            {
                                                continue;
                                            }
                                            j0array[0] = ((2.0) * (atan((((r11 * x1477 * (x1480.value))) + (((-0.195039861251953) * x1478)) + ((r10 * x1476 * (x1481.value))) + (((-1.0) * r12 * x1475 * (x1482.value)))))));
                                            sj0array[0] = IKsin(j0array[0]);
                                            cj0array[0] = IKcos(j0array[0]);
                                            if (j0array[0] > IKPI)
                                            {
                                                j0array[0] -= IK2PI;
                                            }
                                            else if (j0array[0] < -IKPI)
                                            {
                                                j0array[0] += IK2PI;
                                            }
                                            j0valid[0] = true;
                                            for (int ij0 = 0; ij0 < 1; ++ij0)
                                            {
                                                if (!j0valid[ij0])
                                                {
                                                    continue;
                                                }
                                                _ij0[0] = ij0;
                                                _ij0[1] = -1;
                                                for (int iij0 = ij0 + 1; iij0 < 1; ++iij0)
                                                {
                                                    if (j0valid[iij0] && IKabs(cj0array[ij0] - cj0array[iij0]) < IKFAST_SOLUTION_THRESH && IKabs(sj0array[ij0] - sj0array[iij0]) < IKFAST_SOLUTION_THRESH)
                                                    {
                                                        j0valid[iij0] = false;
                                                        _ij0[1] = iij0;
                                                        break;
                                                    }
                                                }
                                                j0 = j0array[ij0];
                                                cj0 = cj0array[ij0];
                                                sj0 = sj0array[ij0];

                                                innerfn(solutions);
                                            }
                                        }
                                    }
                                }
                            }
                        }
                    }
                }
            }
        }
        return solutions.GetNumSolutions() > 0;
    }

    void IKSolver::innerfn(IkSolutionListBase<IkReal> &solutions)
    {
        for (int fniter = 0; fniter < 1; ++fniter)
        {
            IkReal op[8 + 1], zeror[8];
            int numroots;
            IkReal x249 = ((0.886638965956769) * sj0);
            IkReal x250 = ((0.176316034571766) * cj0);
            IkReal x251 = ((3.7e-7) * cj5);
            IkReal x252 = ((0.1135) * sj5);
            IkReal x253 = ((3.7e-7) * sj5);
            IkReal x254 = ((0.1135) * cj5);
            IkReal x255 = (r21 * x252);
            IkReal x256 = (r21 * x251);
            IkReal x257 = (r20 * x253);
            IkReal x258 = (x250 + (((-1.0) * x249)));
            IkReal x259 = ((((1.80270179140325) * sj0)) + (((-0.35848326498109) * cj0)));
            IkReal x260 = ((((0.0294238594897158) * sj0)) + (((-0.0058511958375586) * cj0)));
            IkReal x261 = (x249 + (((-1.0) * x250)));
            IkReal x262 = (((r20 * x254)) + (((1.0) * pz)));
            IkReal x263 = (x255 + x256 + x257);
            IkReal x264 = ((0.919) + x263 + (((-1.0) * x262)));
            IkReal x265 = ((0.015) + x263 + (((-1.0) * x262)));
            IkReal x266 = ((-0.919) + x263 + (((-1.0) * x262)));
            IkReal x267 = ((-0.015) + x263 + (((-1.0) * x262)));
            IkReal x268 = ((((0.0221176390070984) * sj0)) + ((r00 * x253)) + ((r01 * x252)) + ((r01 * x251)) + (((0.111222163942722) * cj0)) + (((-1.0) * r00 * x254)) + (((-1.0) * px)));
            IkReal gconst36 = x264;
            IkReal gconst37 = x265;
            IkReal gconst38 = x268;
            IkReal gconst39 = x258;
            IkReal gconst40 = x268;
            IkReal gconst41 = x264;
            IkReal gconst42 = x265;
            IkReal gconst43 = x268;
            IkReal gconst44 = x258;
            IkReal gconst45 = x268;
            IkReal gconst46 = x259;
            IkReal gconst47 = x260;
            IkReal gconst48 = x259;
            IkReal gconst49 = x260;
            IkReal gconst50 = x266;
            IkReal gconst51 = x267;
            IkReal gconst52 = x268;
            IkReal gconst53 = x261;
            IkReal gconst54 = x268;
            IkReal gconst55 = x266;
            IkReal gconst56 = x267;
            IkReal gconst57 = x268;
            IkReal gconst58 = x261;
            IkReal gconst59 = x268;
            IkReal x269 = (gconst40 * gconst41);
            IkReal x270 = ((1.808) * gconst58);
            IkReal x271 = (gconst48 * gconst56);
            IkReal x272 = ((1.0) * gconst51);
            IkReal x273 = (gconst41 * gconst52);
            IkReal x274 = (gconst37 * gconst59);
            IkReal x275 = (gconst47 * gconst50);
            IkReal x276 = (gconst49 * gconst51);
            IkReal x277 = ((1.808) * gconst48);
            IkReal x278 = (gconst51 * gconst53);
            IkReal x279 = (gconst37 * gconst45);
            IkReal x280 = (gconst38 * gconst55);
            IkReal x281 = (gconst41 * gconst46);
            IkReal x282 = ((1.0) * gconst49);
            IkReal x283 = ((3.268864) * gconst54);
            IkReal x284 = ((1.808) * gconst44);
            IkReal x285 = (gconst37 * gconst39);
            IkReal x286 = (gconst56 * gconst57);
            IkReal x287 = (gconst36 * gconst47);
            IkReal x288 = (gconst38 * gconst41);
            IkReal x289 = (gconst39 * gconst58);
            IkReal x290 = (gconst51 * gconst59);
            IkReal x291 = (gconst42 * gconst57);
            IkReal x292 = (gconst40 * gconst50);
            IkReal x293 = (gconst40 * gconst55);
            IkReal x294 = (gconst41 * gconst54);
            IkReal x295 = ((1.808) * gconst43);
            IkReal x296 = (gconst37 * gconst53);
            IkReal x297 = ((1.0) * gconst59);
            IkReal x298 = (gconst43 * gconst56);
            IkReal x299 = ((3.268864) * gconst43);
            IkReal x300 = (gconst36 * gconst40);
            IkReal x301 = (gconst36 * gconst54);
            IkReal x302 = (gconst44 * gconst55);
            IkReal x303 = (gconst42 * gconst43);
            IkReal x304 = ((1.808) * gconst57);
            IkReal x305 = (gconst45 * gconst51);
            IkReal x306 = ((1.0) * gconst45);
            IkReal x307 = (gconst39 * gconst51);
            IkReal x308 = (gconst52 * gconst55);
            IkReal x309 = (gconst41 * gconst47);
            IkReal x310 = (gconst42 * gconst48);
            IkReal x311 = ((3.268864) * gconst47);
            IkReal x312 = (gconst54 * gconst55);
            IkReal x313 = (gconst41 * gconst58);
            IkReal x314 = (gconst47 * gconst55);
            IkReal x315 = (gconst41 * gconst44);
            IkReal x316 = (gconst46 * gconst55);
            IkReal x317 = (gconst37 * gconst49);
            IkReal x318 = (gconst50 * gconst54);
            IkReal x319 = ((3.268864) * gconst40);
            IkReal x320 = ((1.0) * gconst37 * gconst52);
            IkReal x321 = (gconst42 * x318);
            IkReal x322 = ((1.0) * gconst37 * gconst38);
            IkReal x323 = ((1.0) * gconst37 * gconst46);
            IkReal x324 = ((1.0) * gconst55 * gconst58);
            op[0] = (((x290 * x308)) + ((x286 * x318)) + (((-1.0) * gconst52 * x272 * x286)) + (((-1.0) * gconst50 * x297 * x312)) + (((-1.0) * gconst53 * gconst55 * gconst58 * x272)));
            op[1] = ((((-1.0) * gconst46 * x272 * x286)) + ((x271 * x318)) + (((-1.0) * gconst55 * x275 * x297)) + ((x278 * x304)) + ((x275 * x286)) + (((-1.0) * gconst50 * x282 * x312)) + ((x290 * x316)) + (((-1.0) * gconst52 * x271 * x272)) + ((x276 * x308)) + ((x270 * x312)));
            op[2] = (((x276 * x316)) + (((-1.0) * gconst50 * x294 * x297)) + ((x273 * x290)) + ((x271 * x275)) + (((-1.0) * gconst53 * x272 * x302)) + ((x280 * x290)) + ((x286 * x301)) + (((-1.0) * gconst55 * x297 * x301)) + (((-1.0) * x286 * x320)) + ((x274 * x308)) + ((x298 * x318)) + ((x286 * x292)) + (((-1.0) * gconst55 * x292 * x297)) + (((-1.0) * gconst53 * x272 * x313)) + (((-1.0) * gconst50 * x306 * x312)) + ((x277 * x278)) + ((x305 * x308)) + ((x291 * x318)) + (((-1.0) * x296 * x324)) + (((-1.0) * gconst55 * x275 * x282)) + (((-1.0) * gconst57 * x283)) + (((-1.0) * gconst46 * x271 * x272)) + (((-1.0) * gconst55 * x272 * x289)) + (((-1.0) * gconst52 * x272 * x291)) + (((-1.0) * gconst52 * x272 * x298)) + ((x270 * x314)) + (((-1.0) * gconst38 * x272 * x286)));
            op[3] = ((((-1.0) * gconst52 * x272 * x310)) + ((x273 * x276)) + ((x305 * x316)) + ((x271 * x292)) + ((x310 * x318)) + ((x271 * x301)) + ((x286 * x287)) + (((-1.0) * gconst55 * x282 * x301)) + ((x275 * x291)) + ((x275 * x298)) + (((-1.0) * gconst48 * x283)) + (((-1.0) * x271 * x320)) + ((x281 * x290)) + (((-1.0) * x286 * x323)) + ((x308 * x317)) + (((-1.0) * gconst50 * x282 * x294)) + ((x270 * x293)) + ((x270 * x294)) + ((x284 * x312)) + (((-1.0) * gconst38 * x271 * x272)) + ((x274 * x316)) + ((x278 * x295)) + ((x296 * x304)) + (((-1.0) * gconst57 * x311)) + ((x304 * x307)) + (((-1.0) * gconst46 * x272 * x291)) + (((-1.0) * gconst46 * x272 * x298)) + ((x276 * x280)) + (((-1.0) * gconst41 * x275 * x297)) + (((-1.0) * gconst55 * x282 * x292)) + (((-1.0) * gconst55 * x275 * x306)) + (((-1.0) * gconst55 * x287 * x297)));
            op[4] = ((((-1.0) * x296 * x302)) + ((x273 * x274)) + (((-1.0) * gconst55 * x292 * x306)) + ((x291 * x292)) + (((-1.0) * x298 * x320)) + ((x291 * x301)) + (((-1.0) * x296 * x313)) + (((-1.0) * gconst39 * x272 * x302)) + ((x286 * x300)) + (((-1.0) * gconst36 * x294 * x297)) + ((x275 * x310)) + (((-1.0) * gconst38 * x272 * x291)) + (((-1.0) * gconst38 * x272 * x298)) + ((x277 * x296)) + (((-1.0) * x271 * x323)) + ((x279 * x308)) + ((x274 * x280)) + (((-1.0) * x286 * x322)) + (((-1.0) * gconst36 * x293 * x297)) + ((x280 * x305)) + ((x271 * x287)) + (((-1.0) * gconst46 * x272 * x310)) + ((x284 * x314)) + (((-1.0) * gconst41 * x272 * x289)) + (((-1.0) * x285 * x324)) + (((-1.0) * gconst43 * x283)) + (((-1.0) * gconst41 * x275 * x282)) + (((-1.0) * gconst55 * x282 * x287)) + ((x298 * x301)) + (((-1.0) * gconst55 * x301 * x306)) + ((x270 * x309)) + (((-1.0) * gconst53 * x272 * x315)) + (((-1.0) * gconst57 * x319)) + (((-1.0) * gconst50 * x269 * x297)) + ((x277 * x307)) + ((x303 * x318)) + (((-1.0) * gconst50 * x294 * x306)) + (((-1.0) * gconst48 * x311)) + (((-1.0) * gconst52 * x272 * x303)) + ((x276 * x281)) + ((x292 * x298)) + (((-1.0) * x291 * x320)) + ((x316 * x317)) + ((x288 * x290)) + ((x273 * x305)));
            op[5] = ((((-1.0) * gconst46 * x272 * x303)) + ((x275 * x303)) + ((x279 * x316)) + ((x271 * x300)) + (((-1.0) * x298 * x323)) + (((-1.0) * gconst50 * x269 * x282)) + ((x285 * x304)) + (((-1.0) * gconst41 * x287 * x297)) + (((-1.0) * gconst38 * x272 * x310)) + ((x273 * x317)) + (((-1.0) * x271 * x322)) + ((x301 * x310)) + ((x274 * x281)) + (((-1.0) * gconst55 * x287 * x306)) + (((-1.0) * gconst41 * x275 * x306)) + ((x287 * x291)) + ((x287 * x298)) + ((x284 * x293)) + ((x284 * x294)) + ((x281 * x305)) + ((x280 * x317)) + ((x295 * x307)) + ((x295 * x296)) + (((-1.0) * gconst47 * x299)) + (((-1.0) * x310 * x320)) + (((-1.0) * gconst48 * x319)) + ((x269 * x270)) + (((-1.0) * gconst36 * x282 * x293)) + (((-1.0) * gconst36 * x282 * x294)) + ((x292 * x310)) + ((x276 * x288)) + (((-1.0) * x291 * x323)));
            op[6] = (((x301 * x303)) + ((x273 * x279)) + (((-1.0) * x303 * x320)) + (((-1.0) * gconst40 * x299)) + (((-1.0) * x298 * x322)) + ((x279 * x280)) + ((x291 * x300)) + (((-1.0) * x296 * x315)) + (((-1.0) * gconst50 * x269 * x306)) + (((-1.0) * gconst39 * x272 * x315)) + ((x292 * x303)) + ((x274 * x288)) + (((-1.0) * gconst36 * x269 * x297)) + ((x284 * x309)) + (((-1.0) * gconst41 * x282 * x287)) + (((-1.0) * gconst38 * x272 * x303)) + (((-1.0) * x285 * x313)) + (((-1.0) * gconst36 * x294 * x306)) + ((x298 * x300)) + (((-1.0) * gconst36 * x293 * x306)) + ((x281 * x317)) + ((x288 * x305)) + (((-1.0) * x285 * x302)) + ((x287 * x310)) + (((-1.0) * x310 * x323)) + ((x277 * x285)) + (((-1.0) * x291 * x322)));
            op[7] = ((((-1.0) * x303 * x323)) + ((x279 * x281)) + ((x269 * x284)) + ((x300 * x310)) + ((x287 * x303)) + ((x285 * x295)) + (((-1.0) * gconst36 * x269 * x282)) + (((-1.0) * x310 * x322)) + (((-1.0) * gconst41 * x287 * x306)) + ((x288 * x317)));
            op[8] = ((((-1.0) * x303 * x322)) + ((x279 * x288)) + (((-1.0) * gconst36 * x269 * x306)) + (((-1.0) * x285 * x315)) + ((x300 * x303)));
            polyroots8(op, zeror, numroots);
            IkReal j1array[8], cj1array[8], sj1array[8], tempj1array[1];
            int numsolutions = 0;
            for (int ij1 = 0; ij1 < numroots; ++ij1)
            {
                IkReal htj1 = zeror[ij1];
                tempj1array[0] = ((2.0) * (atan(htj1)));
                for (int kj1 = 0; kj1 < 1; ++kj1)
                {
                    j1array[numsolutions] = tempj1array[kj1];
                    if (j1array[numsolutions] > IKPI)
                    {
                        j1array[numsolutions] -= IK2PI;
                    }
                    else if (j1array[numsolutions] < -IKPI)
                    {
                        j1array[numsolutions] += IK2PI;
                    }
                    sj1array[numsolutions] = IKsin(j1array[numsolutions]);
                    cj1array[numsolutions] = IKcos(j1array[numsolutions]);
                    numsolutions++;
                }
            }
            bool j1valid[8] = {true, true, true, true, true, true, true, true};
            _nj1 = 8;
            for (int ij1 = 0; ij1 < numsolutions; ++ij1)
            {
                if (!j1valid[ij1])
                {
                    continue;
                }
                j1 = j1array[ij1];
                cj1 = cj1array[ij1];
                sj1 = sj1array[ij1];
                htj1 = IKtan(j1 / 2);

                _ij1[0] = ij1;
                _ij1[1] = -1;
                for (int iij1 = ij1 + 1; iij1 < numsolutions; ++iij1)
                {
                    if (j1valid[iij1] && IKabs(cj1array[ij1] - cj1array[iij1]) < IKFAST_SOLUTION_THRESH && IKabs(sj1array[ij1] - sj1array[iij1]) < IKFAST_SOLUTION_THRESH)
                    {
                        j1valid[iij1] = false;
                        _ij1[1] = iij1;
                        break;
                    }
                }
                {
                    IkReal j2eval[2];
                    j2eval[0] = (cj0 + (((-5.02869162246205) * sj0)));
                    j2eval[1] = IKsign(((((9.96185595330477) * cj0)) + (((-50.0951015765574) * sj0))));
                    if (IKabs(j2eval[0]) < 0.0000010000000000 || IKabs(j2eval[1]) < 0.0000010000000000)
                    {
                        {
                            IkReal j2eval[2];
                            j2eval[0] = (sj0 + (((5.02869162246205) * cj0)));
                            j2eval[1] = IKsign(((((50.0951015765574) * cj0)) + (((9.96185595330477) * sj0))));
                            if (IKabs(j2eval[0]) < 0.0000010000000000 || IKabs(j2eval[1]) < 0.0000010000000000)
                            {
                                {
                                    IkReal evalcond[1];
                                    bool bgotonextstatement = true;
                                    do
                                    {
                                        evalcond[0] = ((-3.14159265358979) + (IKfmod(((3.14159265358979) + (IKabs(((1.37449823503129) + j0)))), 6.28318530717959)));
                                        if (IKabs(evalcond[0]) < 0.0000050000000000)
                                        {
                                            bgotonextstatement = false;
                                            {
                                                IkReal j2array[1], cj2array[1], sj2array[1];
                                                bool j2valid[1] = {false};
                                                _nj2 = 1;
                                                IkReal x325 = (cj5 * sj1);
                                                IkReal x326 = ((8.1858407079646e-7) * r21);
                                                IkReal x327 = ((2.21238937309433) * px);
                                                IkReal x328 = ((0.251106193846207) * r00);
                                                IkReal x329 = ((2.21238938053097) * pz);
                                                IkReal x330 = (cj1 * cj5);
                                                IkReal x331 = ((0.251106194690265) * r20);
                                                IkReal x332 = (cj1 * sj5);
                                                IkReal x333 = ((8.18584068044904e-7) * r00);
                                                IkReal x334 = ((8.18584068044904e-7) * r01);
                                                IkReal x335 = ((8.1858407079646e-7) * r20);
                                                IkReal x336 = (sj1 * sj5);
                                                IkReal x337 = ((0.251106194690265) * r21 * sj5);
                                                IkReal x338 = ((0.251106193846207) * r01 * sj5);
                                                if (IKabs((((cj1 * x327)) + (((2.68901162727221e-7) * cj1)) + (((-1.0) * x325 * x326)) + (((-1.0) * x330 * x334)) + ((sj1 * x329)) + ((x328 * x330)) + (((-1.0) * x332 * x333)) + (((-0.251106194690265) * r21 * x336)) + ((x325 * x331)) + (((-0.251106193846207) * r01 * x332)) + (((-1.0) * x335 * x336)))) < IKFAST_ATAN2_MAGTHRESH && IKabs(((-1.03318584070796) + (((-1.0) * sj1 * x327)) + ((cj1 * x329)) + ((x330 * x331)) + (((-1.0) * x325 * x328)) + (((-2.68901162727221e-7) * sj1)) + ((x333 * x336)) + (((-1.0) * x332 * x335)) + (((-0.251106194690265) * r21 * x332)) + ((x325 * x334)) + (((0.251106193846207) * r01 * x336)) + (((-1.0) * x326 * x330)))) < IKFAST_ATAN2_MAGTHRESH && IKabs(IKsqr((((cj1 * x327)) + (((2.68901162727221e-7) * cj1)) + (((-1.0) * x325 * x326)) + (((-1.0) * x330 * x334)) + ((sj1 * x329)) + ((x328 * x330)) + (((-1.0) * x332 * x333)) + (((-0.251106194690265) * r21 * x336)) + ((x325 * x331)) + (((-0.251106193846207) * r01 * x332)) + (((-1.0) * x335 * x336)))) + IKsqr(((-1.03318584070796) + (((-1.0) * sj1 * x327)) + ((cj1 * x329)) + ((x330 * x331)) + (((-1.0) * x325 * x328)) + (((-2.68901162727221e-7) * sj1)) + ((x333 * x336)) + (((-1.0) * x332 * x335)) + (((-0.251106194690265) * r21 * x332)) + ((x325 * x334)) + (((0.251106193846207) * r01 * x336)) + (((-1.0) * x326 * x330)))) - 1) <= IKFAST_SINCOS_THRESH)
                                                    continue;
                                                j2array[0] = IKatan2((((cj1 * x327)) + (((2.68901162727221e-7) * cj1)) + (((-1.0) * x325 * x326)) + (((-1.0) * x330 * x334)) + ((sj1 * x329)) + ((x328 * x330)) + (((-1.0) * x332 * x333)) + (((-0.251106194690265) * r21 * x336)) + ((x325 * x331)) + (((-0.251106193846207) * r01 * x332)) + (((-1.0) * x335 * x336))), ((-1.03318584070796) + (((-1.0) * sj1 * x327)) + ((cj1 * x329)) + ((x330 * x331)) + (((-1.0) * x325 * x328)) + (((-2.68901162727221e-7) * sj1)) + ((x333 * x336)) + (((-1.0) * x332 * x335)) + (((-0.251106194690265) * r21 * x332)) + ((x325 * x334)) + (((0.251106193846207) * r01 * x336)) + (((-1.0) * x326 * x330))));
                                                sj2array[0] = IKsin(j2array[0]);
                                                cj2array[0] = IKcos(j2array[0]);
                                                if (j2array[0] > IKPI)
                                                {
                                                    j2array[0] -= IK2PI;
                                                }
                                                else if (j2array[0] < -IKPI)
                                                {
                                                    j2array[0] += IK2PI;
                                                }
                                                j2valid[0] = true;
                                                for (int ij2 = 0; ij2 < 1; ++ij2)
                                                {
                                                    if (!j2valid[ij2])
                                                    {
                                                        continue;
                                                    }
                                                    _ij2[0] = ij2;
                                                    _ij2[1] = -1;
                                                    for (int iij2 = ij2 + 1; iij2 < 1; ++iij2)
                                                    {
                                                        if (j2valid[iij2] && IKabs(cj2array[ij2] - cj2array[iij2]) < IKFAST_SOLUTION_THRESH && IKabs(sj2array[ij2] - sj2array[iij2]) < IKFAST_SOLUTION_THRESH)
                                                        {
                                                            j2valid[iij2] = false;
                                                            _ij2[1] = iij2;
                                                            break;
                                                        }
                                                    }
                                                    j2 = j2array[ij2];
                                                    cj2 = cj2array[ij2];
                                                    sj2 = sj2array[ij2];
                                                    {
                                                        IkReal evalcond[3];
                                                        IkReal x339 = IKsin(j2);
                                                        IkReal x340 = IKcos(j2);
                                                        IkReal x341 = ((3.7e-7) * sj5);
                                                        IkReal x342 = ((3.7e-7) * cj5);
                                                        IkReal x343 = ((0.1135) * cj5);
                                                        IkReal x344 = ((0.1135) * sj5);
                                                        IkReal x345 = (cj1 * x339);
                                                        IkReal x346 = (sj1 * x340);
                                                        evalcond[0] = ((((-1.0) * r20 * x343)) + ((r21 * x344)) + ((r21 * x342)) + (((0.467) * cj1)) + (((0.452) * sj1 * x339)) + (((-1.0) * pz)) + ((r20 * x341)) + (((0.452) * cj1 * x340)));
                                                        evalcond[1] = ((-1.21543325961255e-7) + ((r01 * x344)) + ((r01 * x342)) + ((r00 * x341)) + (((-1.0) * px)) + (((0.452000001519335) * x345)) + (((-0.452000001519335) * x346)) + (((-1.0) * r00 * x343)) + (((-0.467000001569756) * sj1)));
                                                        evalcond[2] = ((-0.11339999909077) + (((8.79096604904732e-10) * x346)) + (((-1.0) * py)) + ((r11 * x342)) + ((r11 * x344)) + ((r10 * x341)) + (((-8.79096604904732e-10) * x345)) + (((-1.0) * r10 * x343)) + (((9.08270164802013e-10) * sj1)));
                                                        if (IKabs(evalcond[0]) > IKFAST_EVALCOND_THRESH || IKabs(evalcond[1]) > IKFAST_EVALCOND_THRESH || IKabs(evalcond[2]) > IKFAST_EVALCOND_THRESH)
                                                        {
                                                            continue;
                                                        }
                                                    }

                                                    {
                                                        IkReal j3eval[2];
                                                        sj0 = -0.98079532;
                                                        cj0 = 0.19503986;
                                                        j0 = -1.37449824;
                                                        IkReal x347 = (((cj5 * r11 * sj4)) + ((r10 * sj4 * sj5)) + (((-1.0) * cj4 * r12)));
                                                        j3eval[0] = x347;
                                                        j3eval[1] = IKsign(x347);
                                                        if (IKabs(j3eval[0]) < 0.0000010000000000 || IKabs(j3eval[1]) < 0.0000010000000000)
                                                        {
                                                            {
                                                                IkReal j3eval[2];
                                                                sj0 = -0.98079532;
                                                                cj0 = 0.19503986;
                                                                j0 = -1.37449824;
                                                                IkReal x348 = ((1.0) * sj4);
                                                                IkReal x349 = ((((-1.0) * cj5 * r01 * x348)) + (((-1.0) * r00 * sj5 * x348)) + ((cj4 * r02)));
                                                                j3eval[0] = x349;
                                                                j3eval[1] = IKsign(x349);
                                                                if (IKabs(j3eval[0]) < 0.0000010000000000 || IKabs(j3eval[1]) < 0.0000010000000000)
                                                                {
                                                                    {
                                                                        IkReal j3eval[2];
                                                                        sj0 = -0.98079532;
                                                                        cj0 = 0.19503986;
                                                                        j0 = -1.37449824;
                                                                        IkReal x350 = cj4 * cj4;
                                                                        IkReal x351 = cj5 * cj5;
                                                                        IkReal x352 = r22 * r22;
                                                                        IkReal x353 = r21 * r21;
                                                                        IkReal x354 = r20 * r20;
                                                                        IkReal x355 = (r20 * sj5);
                                                                        IkReal x356 = (cj5 * r21);
                                                                        IkReal x357 = ((1.0) * x353);
                                                                        IkReal x358 = ((1.0) * x354);
                                                                        IkReal x359 = (x350 * x351);
                                                                        IkReal x360 = ((2.0) * cj4 * r22 * sj4);
                                                                        IkReal x361 = ((((2.0) * x355 * x356)) + (((-1.0) * x357 * x359)) + (((-1.0) * x357)) + (((-1.0) * x355 * x360)) + (((-1.0) * x356 * x360)) + (((-2.0) * x350 * x355 * x356)) + ((x351 * x353)) + (((-1.0) * x352)) + ((x354 * x359)) + ((x350 * x352)) + (((-1.0) * x351 * x358)) + (((-1.0) * x350 * x358)));
                                                                        j3eval[0] = x361;
                                                                        j3eval[1] = IKsign(x361);
                                                                        if (IKabs(j3eval[0]) < 0.0000010000000000 || IKabs(j3eval[1]) < 0.0000010000000000)
                                                                        {
                                                                            continue; // no branches [j3]
                                                                        }
                                                                        else
                                                                        {
                                                                            {
                                                                                IkReal j3array[1], cj3array[1], sj3array[1];
                                                                                bool j3valid[1] = {false};
                                                                                _nj3 = 1;
                                                                                IkReal x362 = cj4 * cj4;
                                                                                IkReal x363 = cj5 * cj5;
                                                                                IkReal x364 = r22 * r22;
                                                                                IkReal x365 = r21 * r21;
                                                                                IkReal x366 = r20 * r20;
                                                                                IkReal x367 = ((1.0) * sj1);
                                                                                IkReal x368 = (r22 * sj4);
                                                                                IkReal x369 = (sj2 * sj5);
                                                                                IkReal x370 = (cj4 * r20);
                                                                                IkReal x371 = (cj5 * sj2);
                                                                                IkReal x372 = (r21 * sj5);
                                                                                IkReal x373 = (cj4 * r21);
                                                                                IkReal x374 = ((2.0) * cj5);
                                                                                IkReal x375 = (cj2 * cj5 * r20);
                                                                                IkReal x376 = ((1.0) * x365);
                                                                                IkReal x377 = ((1.0) * cj1 * cj2);
                                                                                IkReal x378 = ((1.0) * x366);
                                                                                IkReal x379 = (x362 * x363);
                                                                                CheckValue<IkReal> x380 = IKatan2WithCheck(IkReal(((((-1.0) * x368 * x377)) + (((-1.0) * sj2 * x367 * x368)) + (((-1.0) * x367 * x371 * x373)) + ((cj2 * sj1 * x372)) + (((-1.0) * cj5 * x373 * x377)) + (((-1.0) * x367 * x369 * x370)) + (((-1.0) * x367 * x375)) + (((-1.0) * sj5 * x370 * x377)) + (((-1.0) * cj1 * r21 * x369)) + ((cj1 * r20 * x371)))), IkReal(((((-1.0) * x372 * x377)) + ((cj1 * x371 * x373)) + ((r20 * sj1 * x371)) + ((cj1 * x369 * x370)) + ((cj1 * x375)) + (((-1.0) * cj2 * x367 * x368)) + ((cj1 * sj2 * x368)) + (((-1.0) * r21 * x367 * x369)) + (((-1.0) * cj2 * sj5 * x367 * x370)) + (((-1.0) * cj2 * cj5 * x367 * x373)))), IKFAST_ATAN2_MAGTHRESH);
                                                                                if (!x380.valid)
                                                                                {
                                                                                    continue;
                                                                                }
                                                                                CheckValue<IkReal> x381 = IKPowWithIntegerCheck(IKsign(((((-1.0) * x362 * x378)) + (((-1.0) * x363 * x378)) + (((-1.0) * x364)) + (((-2.0) * sj5 * x368 * x370)) + (((-1.0) * x368 * x373 * x374)) + (((-1.0) * x376)) + ((x366 * x379)) + (((-1.0) * x376 * x379)) + ((x363 * x365)) + ((x362 * x364)) + ((r20 * x372 * x374)) + (((-1.0) * r20 * x362 * x372 * x374)))), -1);
                                                                                if (!x381.valid)
                                                                                {
                                                                                    continue;
                                                                                }
                                                                                j3array[0] = ((-1.5707963267949) + (x380.value) + (((1.5707963267949) * (x381.value))));
                                                                                sj3array[0] = IKsin(j3array[0]);
                                                                                cj3array[0] = IKcos(j3array[0]);
                                                                                if (j3array[0] > IKPI)
                                                                                {
                                                                                    j3array[0] -= IK2PI;
                                                                                }
                                                                                else if (j3array[0] < -IKPI)
                                                                                {
                                                                                    j3array[0] += IK2PI;
                                                                                }
                                                                                j3valid[0] = true;
                                                                                for (int ij3 = 0; ij3 < 1; ++ij3)
                                                                                {
                                                                                    if (!j3valid[ij3])
                                                                                    {
                                                                                        continue;
                                                                                    }
                                                                                    _ij3[0] = ij3;
                                                                                    _ij3[1] = -1;
                                                                                    for (int iij3 = ij3 + 1; iij3 < 1; ++iij3)
                                                                                    {
                                                                                        if (j3valid[iij3] && IKabs(cj3array[ij3] - cj3array[iij3]) < IKFAST_SOLUTION_THRESH && IKabs(sj3array[ij3] - sj3array[iij3]) < IKFAST_SOLUTION_THRESH)
                                                                                        {
                                                                                            j3valid[iij3] = false;
                                                                                            _ij3[1] = iij3;
                                                                                            break;
                                                                                        }
                                                                                    }
                                                                                    j3 = j3array[ij3];
                                                                                    cj3 = cj3array[ij3];
                                                                                    sj3 = sj3array[ij3];
                                                                                    {
                                                                                        IkReal evalcond[6];
                                                                                        IkReal x382 = IKcos(j3);
                                                                                        IkReal x383 = IKsin(j3);
                                                                                        IkReal x384 = (r00 * sj5);
                                                                                        IkReal x385 = ((1.94490399315206e-9) * cj1);
                                                                                        IkReal x386 = (sj1 * sj2);
                                                                                        IkReal x387 = (cj2 * sj1);
                                                                                        IkReal x388 = ((1.0) * r11);
                                                                                        IkReal x389 = ((1.00000000336136) * cj1);
                                                                                        IkReal x390 = (cj5 * x382);
                                                                                        IkReal x391 = (sj4 * x382);
                                                                                        IkReal x392 = (sj5 * x382);
                                                                                        IkReal x393 = ((1.0) * x383);
                                                                                        IkReal x394 = (cj5 * x383);
                                                                                        IkReal x395 = (cj4 * x393);
                                                                                        evalcond[0] = (((r20 * x394)) + ((cj1 * sj2)) + (((-1.0) * r21 * sj5 * x393)) + (((-1.0) * x387)) + ((r22 * x391)) + ((cj4 * r20 * x392)) + ((cj4 * r21 * x390)));
                                                                                        evalcond[1] = (((r20 * x390)) + (((-1.0) * r20 * sj5 * x395)) + (((-1.0) * r22 * sj4 * x393)) + x386 + (((-1.0) * r21 * x392)) + ((cj1 * cj2)) + (((-1.0) * cj5 * r21 * x395)));
                                                                                        evalcond[2] = ((((-1.0) * cj2 * x389)) + (((-1.0) * r01 * sj5 * x393)) + ((cj4 * x382 * x384)) + ((r00 * x394)) + (((-1.00000000336136) * x386)) + ((cj4 * r01 * x390)) + ((r02 * x391)));
                                                                                        evalcond[3] = (((r10 * x394)) + (((-1.0) * sj5 * x383 * x388)) + (((1.94490399315206e-9) * x386)) + ((cj4 * r11 * x390)) + ((cj2 * x385)) + ((cj4 * r10 * x392)) + ((r12 * x391)));
                                                                                        evalcond[4] = ((((-1.0) * x384 * x395)) + (((-1.0) * r01 * x392)) + (((-1.0) * r02 * sj4 * x393)) + ((r00 * x390)) + (((-1.00000000336136) * x387)) + (((-1.0) * cj5 * r01 * x395)) + ((sj2 * x389)));
                                                                                        evalcond[5] = ((((-1.0) * r12 * sj4 * x393)) + ((r10 * x390)) + (((1.94490399315206e-9) * x387)) + (((-1.0) * cj4 * x388 * x394)) + (((-1.0) * r10 * sj5 * x395)) + (((-1.0) * x388 * x392)) + (((-1.0) * sj2 * x385)));
                                                                                        if (IKabs(evalcond[0]) > IKFAST_EVALCOND_THRESH || IKabs(evalcond[1]) > IKFAST_EVALCOND_THRESH || IKabs(evalcond[2]) > IKFAST_EVALCOND_THRESH || IKabs(evalcond[3]) > IKFAST_EVALCOND_THRESH || IKabs(evalcond[4]) > IKFAST_EVALCOND_THRESH || IKabs(evalcond[5]) > IKFAST_EVALCOND_THRESH)
                                                                                        {
                                                                                            continue;
                                                                                        }
                                                                                    }

                                                                                    {
                                                                                        std::vector<IkSingleDOFSolutionBase<IkReal>> vinfos(6);
                                                                                        vinfos[0].jointtype = 1;
                                                                                        vinfos[0].foffset = j0;
                                                                                        vinfos[0].indices[0] = _ij0[0];
                                                                                        vinfos[0].indices[1] = _ij0[1];
                                                                                        vinfos[0].maxsolutions = _nj0;
                                                                                        vinfos[1].jointtype = 1;
                                                                                        vinfos[1].foffset = j1;
                                                                                        vinfos[1].indices[0] = _ij1[0];
                                                                                        vinfos[1].indices[1] = _ij1[1];
                                                                                        vinfos[1].maxsolutions = _nj1;
                                                                                        vinfos[2].jointtype = 1;
                                                                                        vinfos[2].foffset = j2;
                                                                                        vinfos[2].indices[0] = _ij2[0];
                                                                                        vinfos[2].indices[1] = _ij2[1];
                                                                                        vinfos[2].maxsolutions = _nj2;
                                                                                        vinfos[3].jointtype = 1;
                                                                                        vinfos[3].foffset = j3;
                                                                                        vinfos[3].indices[0] = _ij3[0];
                                                                                        vinfos[3].indices[1] = _ij3[1];
                                                                                        vinfos[3].maxsolutions = _nj3;
                                                                                        vinfos[4].jointtype = 1;
                                                                                        vinfos[4].foffset = j4;
                                                                                        vinfos[4].indices[0] = _ij4[0];
                                                                                        vinfos[4].indices[1] = _ij4[1];
                                                                                        vinfos[4].maxsolutions = _nj4;
                                                                                        vinfos[5].jointtype = 1;
                                                                                        vinfos[5].foffset = j5;
                                                                                        vinfos[5].indices[0] = _ij5[0];
                                                                                        vinfos[5].indices[1] = _ij5[1];
                                                                                        vinfos[5].maxsolutions = _nj5;
                                                                                        std::vector<int> vfree(0);
                                                                                        solutions.AddSolution(vinfos, vfree);
                                                                                    }
                                                                                }
                                                                            }
                                                                        }
                                                                    }
                                                                }
                                                                else
                                                                {
                                                                    {
                                                                        IkReal j3array[1], cj3array[1], sj3array[1];
                                                                        bool j3valid[1] = {false};
                                                                        _nj3 = 1;
                                                                        IkReal x396 = (cj5 * sj2);
                                                                        IkReal x397 = (cj1 * cj4);
                                                                        IkReal x398 = ((1.94490399315206e-9) * r20);
                                                                        IkReal x399 = (sj2 * sj4);
                                                                        IkReal x400 = ((1.0) * sj4);
                                                                        IkReal x401 = (cj2 * cj5);
                                                                        IkReal x402 = ((1.94490399315206e-9) * sj1);
                                                                        IkReal x403 = (cj2 * sj1);
                                                                        IkReal x404 = ((1.0) * r10);
                                                                        IkReal x405 = (cj4 * sj5);
                                                                        IkReal x406 = (sj2 * sj5);
                                                                        IkReal x407 = ((1.0) * r11);
                                                                        IkReal x408 = ((1.94490399315206e-9) * cj1 * cj2 * sj5);
                                                                        CheckValue<IkReal> x409 = IKatan2WithCheck(IkReal((((r10 * x397 * x406)) + (((-1.94490399315206e-9) * cj1 * cj2 * r22 * sj4)) + (((-1.0) * cj4 * sj1 * x401 * x407)) + (((-1.94490399315206e-9) * r21 * x397 * x401)) + (((-1.0) * sj1 * sj2 * x398 * x405)) + ((r11 * x396 * x397)) + (((-1.0) * cj4 * r21 * x396 * x402)) + (((-1.0) * r12 * x400 * x403)) + (((-1.0) * r22 * x399 * x402)) + (((-1.0) * cj2 * sj5 * x397 * x398)) + ((cj1 * r12 * x399)) + (((-1.0) * x403 * x404 * x405)))), IkReal((((r10 * sj1 * x401)) + ((cj1 * r11 * x406)) + (((-1.0) * r21 * x408)) + (((-1.0) * cj1 * x396 * x404)) + ((sj1 * x396 * x398)) + (((-1.0) * r21 * x402 * x406)) + ((cj1 * x398 * x401)) + (((-1.0) * sj5 * x403 * x407)))), IKFAST_ATAN2_MAGTHRESH);
                                                                        if (!x409.valid)
                                                                        {
                                                                            continue;
                                                                        }
                                                                        CheckValue<IkReal> x410 = IKPowWithIntegerCheck(IKsign(((((-1.0) * cj5 * r01 * x400)) + (((-1.0) * r00 * sj5 * x400)) + ((cj4 * r02)))), -1);
                                                                        if (!x410.valid)
                                                                        {
                                                                            continue;
                                                                        }
                                                                        j3array[0] = ((-1.5707963267949) + (x409.value) + (((1.5707963267949) * (x410.value))));
                                                                        sj3array[0] = IKsin(j3array[0]);
                                                                        cj3array[0] = IKcos(j3array[0]);
                                                                        if (j3array[0] > IKPI)
                                                                        {
                                                                            j3array[0] -= IK2PI;
                                                                        }
                                                                        else if (j3array[0] < -IKPI)
                                                                        {
                                                                            j3array[0] += IK2PI;
                                                                        }
                                                                        j3valid[0] = true;
                                                                        for (int ij3 = 0; ij3 < 1; ++ij3)
                                                                        {
                                                                            if (!j3valid[ij3])
                                                                            {
                                                                                continue;
                                                                            }
                                                                            _ij3[0] = ij3;
                                                                            _ij3[1] = -1;
                                                                            for (int iij3 = ij3 + 1; iij3 < 1; ++iij3)
                                                                            {
                                                                                if (j3valid[iij3] && IKabs(cj3array[ij3] - cj3array[iij3]) < IKFAST_SOLUTION_THRESH && IKabs(sj3array[ij3] - sj3array[iij3]) < IKFAST_SOLUTION_THRESH)
                                                                                {
                                                                                    j3valid[iij3] = false;
                                                                                    _ij3[1] = iij3;
                                                                                    break;
                                                                                }
                                                                            }
                                                                            j3 = j3array[ij3];
                                                                            cj3 = cj3array[ij3];
                                                                            sj3 = sj3array[ij3];
                                                                            {
                                                                                IkReal evalcond[6];
                                                                                IkReal x411 = IKcos(j3);
                                                                                IkReal x412 = IKsin(j3);
                                                                                IkReal x413 = (r00 * sj5);
                                                                                IkReal x414 = ((1.94490399315206e-9) * cj1);
                                                                                IkReal x415 = (sj1 * sj2);
                                                                                IkReal x416 = (cj2 * sj1);
                                                                                IkReal x417 = ((1.0) * r11);
                                                                                IkReal x418 = ((1.00000000336136) * cj1);
                                                                                IkReal x419 = (cj5 * x411);
                                                                                IkReal x420 = (sj4 * x411);
                                                                                IkReal x421 = (sj5 * x411);
                                                                                IkReal x422 = ((1.0) * x412);
                                                                                IkReal x423 = (cj5 * x412);
                                                                                IkReal x424 = (cj4 * x422);
                                                                                evalcond[0] = (((r20 * x423)) + ((cj4 * r21 * x419)) + ((cj1 * sj2)) + ((cj4 * r20 * x421)) + (((-1.0) * r21 * sj5 * x422)) + ((r22 * x420)) + (((-1.0) * x416)));
                                                                                evalcond[1] = ((((-1.0) * r21 * x421)) + (((-1.0) * cj5 * r21 * x424)) + (((-1.0) * r22 * sj4 * x422)) + x415 + ((r20 * x419)) + (((-1.0) * r20 * sj5 * x424)) + ((cj1 * cj2)));
                                                                                evalcond[2] = ((((-1.0) * cj2 * x418)) + (((-1.0) * r01 * sj5 * x422)) + ((r02 * x420)) + ((r00 * x423)) + ((cj4 * r01 * x419)) + (((-1.00000000336136) * x415)) + ((cj4 * x411 * x413)));
                                                                                evalcond[3] = ((((-1.0) * sj5 * x412 * x417)) + ((cj4 * r11 * x419)) + (((1.94490399315206e-9) * x415)) + ((r10 * x423)) + ((cj2 * x414)) + ((r12 * x420)) + ((cj4 * r10 * x421)));
                                                                                evalcond[4] = ((((-1.0) * cj5 * r01 * x424)) + ((sj2 * x418)) + (((-1.0) * r01 * x421)) + ((r00 * x419)) + (((-1.00000000336136) * x416)) + (((-1.0) * r02 * sj4 * x422)) + (((-1.0) * x413 * x424)));
                                                                                evalcond[5] = ((((1.94490399315206e-9) * x416)) + (((-1.0) * r12 * sj4 * x422)) + (((-1.0) * r10 * sj5 * x424)) + (((-1.0) * x417 * x421)) + ((r10 * x419)) + (((-1.0) * cj4 * x417 * x423)) + (((-1.0) * sj2 * x414)));
                                                                                if (IKabs(evalcond[0]) > IKFAST_EVALCOND_THRESH || IKabs(evalcond[1]) > IKFAST_EVALCOND_THRESH || IKabs(evalcond[2]) > IKFAST_EVALCOND_THRESH || IKabs(evalcond[3]) > IKFAST_EVALCOND_THRESH || IKabs(evalcond[4]) > IKFAST_EVALCOND_THRESH || IKabs(evalcond[5]) > IKFAST_EVALCOND_THRESH)
                                                                                {
                                                                                    continue;
                                                                                }
                                                                            }

                                                                            {
                                                                                std::vector<IkSingleDOFSolutionBase<IkReal>> vinfos(6);
                                                                                vinfos[0].jointtype = 1;
                                                                                vinfos[0].foffset = j0;
                                                                                vinfos[0].indices[0] = _ij0[0];
                                                                                vinfos[0].indices[1] = _ij0[1];
                                                                                vinfos[0].maxsolutions = _nj0;
                                                                                vinfos[1].jointtype = 1;
                                                                                vinfos[1].foffset = j1;
                                                                                vinfos[1].indices[0] = _ij1[0];
                                                                                vinfos[1].indices[1] = _ij1[1];
                                                                                vinfos[1].maxsolutions = _nj1;
                                                                                vinfos[2].jointtype = 1;
                                                                                vinfos[2].foffset = j2;
                                                                                vinfos[2].indices[0] = _ij2[0];
                                                                                vinfos[2].indices[1] = _ij2[1];
                                                                                vinfos[2].maxsolutions = _nj2;
                                                                                vinfos[3].jointtype = 1;
                                                                                vinfos[3].foffset = j3;
                                                                                vinfos[3].indices[0] = _ij3[0];
                                                                                vinfos[3].indices[1] = _ij3[1];
                                                                                vinfos[3].maxsolutions = _nj3;
                                                                                vinfos[4].jointtype = 1;
                                                                                vinfos[4].foffset = j4;
                                                                                vinfos[4].indices[0] = _ij4[0];
                                                                                vinfos[4].indices[1] = _ij4[1];
                                                                                vinfos[4].maxsolutions = _nj4;
                                                                                vinfos[5].jointtype = 1;
                                                                                vinfos[5].foffset = j5;
                                                                                vinfos[5].indices[0] = _ij5[0];
                                                                                vinfos[5].indices[1] = _ij5[1];
                                                                                vinfos[5].maxsolutions = _nj5;
                                                                                std::vector<int> vfree(0);
                                                                                solutions.AddSolution(vinfos, vfree);
                                                                            }
                                                                        }
                                                                    }
                                                                }
                                                            }
                                                        }
                                                        else
                                                        {
                                                            {
                                                                IkReal j3array[1], cj3array[1], sj3array[1];
                                                                bool j3valid[1] = {false};
                                                                _nj3 = 1;
                                                                IkReal x425 = (sj2 * sj5);
                                                                IkReal x426 = ((1.0) * r01);
                                                                IkReal x427 = (cj2 * sj5);
                                                                IkReal x428 = ((1.00000000336136) * sj1);
                                                                IkReal x429 = (r02 * sj4);
                                                                IkReal x430 = (cj1 * cj2);
                                                                IkReal x431 = (cj4 * r00);
                                                                IkReal x432 = (cj5 * sj2);
                                                                IkReal x433 = (cj4 * cj5);
                                                                IkReal x434 = (r22 * sj4);
                                                                IkReal x435 = ((1.00000000336136) * cj1);
                                                                IkReal x436 = (cj4 * x435);
                                                                CheckValue<IkReal> x437 = IKatan2WithCheck(IkReal(((((-1.0) * sj1 * x425 * x426)) + (((-1.0) * r21 * x427 * x428)) + (((-1.0) * cj1 * x426 * x427)) + ((r21 * x425 * x435)) + ((cj2 * cj5 * r20 * x428)) + ((cj5 * r00 * x430)) + ((r00 * sj1 * x432)) + (((-1.0) * r20 * x432 * x435)))), IkReal((((x429 * x430)) + ((cj4 * r20 * x427 * x428)) + ((cj2 * x428 * x434)) + ((cj2 * r21 * x428 * x433)) + (((-1.0) * sj2 * x434 * x435)) + (((-1.0) * r20 * x425 * x436)) + ((sj1 * sj2 * x429)) + ((sj1 * x425 * x431)) + ((cj1 * x427 * x431)) + ((r01 * x430 * x433)) + (((-1.0) * r21 * x432 * x436)) + ((cj4 * r01 * sj1 * x432)))), IKFAST_ATAN2_MAGTHRESH);
                                                                if (!x437.valid)
                                                                {
                                                                    continue;
                                                                }
                                                                CheckValue<IkReal> x438 = IKPowWithIntegerCheck(IKsign((((cj5 * r11 * sj4)) + ((r10 * sj4 * sj5)) + (((-1.0) * cj4 * r12)))), -1);
                                                                if (!x438.valid)
                                                                {
                                                                    continue;
                                                                }
                                                                j3array[0] = ((-1.5707963267949) + (x437.value) + (((1.5707963267949) * (x438.value))));
                                                                sj3array[0] = IKsin(j3array[0]);
                                                                cj3array[0] = IKcos(j3array[0]);
                                                                if (j3array[0] > IKPI)
                                                                {
                                                                    j3array[0] -= IK2PI;
                                                                }
                                                                else if (j3array[0] < -IKPI)
                                                                {
                                                                    j3array[0] += IK2PI;
                                                                }
                                                                j3valid[0] = true;
                                                                for (int ij3 = 0; ij3 < 1; ++ij3)
                                                                {
                                                                    if (!j3valid[ij3])
                                                                    {
                                                                        continue;
                                                                    }
                                                                    _ij3[0] = ij3;
                                                                    _ij3[1] = -1;
                                                                    for (int iij3 = ij3 + 1; iij3 < 1; ++iij3)
                                                                    {
                                                                        if (j3valid[iij3] && IKabs(cj3array[ij3] - cj3array[iij3]) < IKFAST_SOLUTION_THRESH && IKabs(sj3array[ij3] - sj3array[iij3]) < IKFAST_SOLUTION_THRESH)
                                                                        {
                                                                            j3valid[iij3] = false;
                                                                            _ij3[1] = iij3;
                                                                            break;
                                                                        }
                                                                    }
                                                                    j3 = j3array[ij3];
                                                                    cj3 = cj3array[ij3];
                                                                    sj3 = sj3array[ij3];
                                                                    {
                                                                        IkReal evalcond[6];
                                                                        IkReal x439 = IKcos(j3);
                                                                        IkReal x440 = IKsin(j3);
                                                                        IkReal x441 = (r00 * sj5);
                                                                        IkReal x442 = ((1.94490399315206e-9) * cj1);
                                                                        IkReal x443 = (sj1 * sj2);
                                                                        IkReal x444 = (cj2 * sj1);
                                                                        IkReal x445 = ((1.0) * r11);
                                                                        IkReal x446 = ((1.00000000336136) * cj1);
                                                                        IkReal x447 = (cj5 * x439);
                                                                        IkReal x448 = (sj4 * x439);
                                                                        IkReal x449 = (sj5 * x439);
                                                                        IkReal x450 = ((1.0) * x440);
                                                                        IkReal x451 = (cj5 * x440);
                                                                        IkReal x452 = (cj4 * x450);
                                                                        evalcond[0] = (((cj4 * r20 * x449)) + (((-1.0) * x444)) + ((cj1 * sj2)) + ((cj4 * r21 * x447)) + ((r20 * x451)) + ((r22 * x448)) + (((-1.0) * r21 * sj5 * x450)));
                                                                        evalcond[1] = ((((-1.0) * cj5 * r21 * x452)) + (((-1.0) * r20 * sj5 * x452)) + (((-1.0) * r22 * sj4 * x450)) + x443 + ((cj1 * cj2)) + ((r20 * x447)) + (((-1.0) * r21 * x449)));
                                                                        evalcond[2] = ((((-1.0) * cj2 * x446)) + ((r02 * x448)) + ((cj4 * x439 * x441)) + ((r00 * x451)) + ((cj4 * r01 * x447)) + (((-1.00000000336136) * x443)) + (((-1.0) * r01 * sj5 * x450)));
                                                                        evalcond[3] = (((cj4 * r10 * x449)) + (((1.94490399315206e-9) * x443)) + ((cj2 * x442)) + (((-1.0) * sj5 * x440 * x445)) + ((cj4 * r11 * x447)) + ((r10 * x451)) + ((r12 * x448)));
                                                                        evalcond[4] = ((((-1.0) * r02 * sj4 * x450)) + (((-1.0) * r01 * x449)) + (((-1.0) * cj5 * r01 * x452)) + ((sj2 * x446)) + ((r00 * x447)) + (((-1.00000000336136) * x444)) + (((-1.0) * x441 * x452)));
                                                                        evalcond[5] = ((((-1.0) * sj2 * x442)) + (((1.94490399315206e-9) * x444)) + (((-1.0) * x445 * x449)) + (((-1.0) * r10 * sj5 * x452)) + (((-1.0) * cj4 * x445 * x451)) + ((r10 * x447)) + (((-1.0) * r12 * sj4 * x450)));
                                                                        if (IKabs(evalcond[0]) > IKFAST_EVALCOND_THRESH || IKabs(evalcond[1]) > IKFAST_EVALCOND_THRESH || IKabs(evalcond[2]) > IKFAST_EVALCOND_THRESH || IKabs(evalcond[3]) > IKFAST_EVALCOND_THRESH || IKabs(evalcond[4]) > IKFAST_EVALCOND_THRESH || IKabs(evalcond[5]) > IKFAST_EVALCOND_THRESH)
                                                                        {
                                                                            continue;
                                                                        }
                                                                    }

                                                                    {
                                                                        std::vector<IkSingleDOFSolutionBase<IkReal>> vinfos(6);
                                                                        vinfos[0].jointtype = 1;
                                                                        vinfos[0].foffset = j0;
                                                                        vinfos[0].indices[0] = _ij0[0];
                                                                        vinfos[0].indices[1] = _ij0[1];
                                                                        vinfos[0].maxsolutions = _nj0;
                                                                        vinfos[1].jointtype = 1;
                                                                        vinfos[1].foffset = j1;
                                                                        vinfos[1].indices[0] = _ij1[0];
                                                                        vinfos[1].indices[1] = _ij1[1];
                                                                        vinfos[1].maxsolutions = _nj1;
                                                                        vinfos[2].jointtype = 1;
                                                                        vinfos[2].foffset = j2;
                                                                        vinfos[2].indices[0] = _ij2[0];
                                                                        vinfos[2].indices[1] = _ij2[1];
                                                                        vinfos[2].maxsolutions = _nj2;
                                                                        vinfos[3].jointtype = 1;
                                                                        vinfos[3].foffset = j3;
                                                                        vinfos[3].indices[0] = _ij3[0];
                                                                        vinfos[3].indices[1] = _ij3[1];
                                                                        vinfos[3].maxsolutions = _nj3;
                                                                        vinfos[4].jointtype = 1;
                                                                        vinfos[4].foffset = j4;
                                                                        vinfos[4].indices[0] = _ij4[0];
                                                                        vinfos[4].indices[1] = _ij4[1];
                                                                        vinfos[4].maxsolutions = _nj4;
                                                                        vinfos[5].jointtype = 1;
                                                                        vinfos[5].foffset = j5;
                                                                        vinfos[5].indices[0] = _ij5[0];
                                                                        vinfos[5].indices[1] = _ij5[1];
                                                                        vinfos[5].maxsolutions = _nj5;
                                                                        std::vector<int> vfree(0);
                                                                        solutions.AddSolution(vinfos, vfree);
                                                                    }
                                                                }
                                                            }
                                                        }
                                                    }
                                                }
                                            }
                                        }
                                    } while (0);
                                    if (bgotonextstatement)
                                    {
                                        bool bgotonextstatement = true;
                                        do
                                        {
                                            evalcond[0] = ((-3.14159265358979) + (IKfmod(((3.14159265358979) + (IKabs(((-1.76709441855851) + j0)))), 6.28318530717959)));
                                            if (IKabs(evalcond[0]) < 0.0000050000000000)
                                            {
                                                bgotonextstatement = false;
                                                {
                                                    IkReal j2array[1], cj2array[1], sj2array[1];
                                                    bool j2valid[1] = {false};
                                                    _nj2 = 1;
                                                    IkReal x453 = (cj5 * sj1);
                                                    IkReal x454 = ((8.1858407079646e-7) * r21);
                                                    IkReal x455 = ((2.21238937309433) * px);
                                                    IkReal x456 = ((0.251106193846207) * r00);
                                                    IkReal x457 = ((2.21238938053097) * pz);
                                                    IkReal x458 = (cj1 * cj5);
                                                    IkReal x459 = ((0.251106194690265) * r20);
                                                    IkReal x460 = (cj1 * sj5);
                                                    IkReal x461 = ((8.18584068044904e-7) * r00);
                                                    IkReal x462 = ((8.18584068044904e-7) * r01);
                                                    IkReal x463 = ((8.1858407079646e-7) * r20);
                                                    IkReal x464 = (sj1 * sj5);
                                                    IkReal x465 = ((0.251106194690265) * r21 * sj5);
                                                    IkReal x466 = ((0.251106193846207) * r01 * sj5);
                                                    if (IKabs(((((2.68901162727221e-7) * cj1)) + (((-1.0) * x463 * x464)) + (((0.251106193846207) * r01 * x460)) + ((x453 * x459)) + (((-0.251106194690265) * r21 * x464)) + (((-1.0) * x456 * x458)) + (((-1.0) * cj1 * x455)) + ((sj1 * x457)) + (((-1.0) * x453 * x454)) + ((x460 * x461)) + ((x458 * x462)))) < IKFAST_ATAN2_MAGTHRESH && IKabs(((-1.03318584070796) + (((-0.251106193846207) * r01 * x464)) + ((x453 * x456)) + ((cj1 * x457)) + (((-0.251106194690265) * r21 * x460)) + (((-1.0) * x460 * x463)) + (((-1.0) * x454 * x458)) + (((-2.68901162727221e-7) * sj1)) + (((-1.0) * x453 * x462)) + ((sj1 * x455)) + (((-1.0) * x461 * x464)) + ((x458 * x459)))) < IKFAST_ATAN2_MAGTHRESH && IKabs(IKsqr(((((2.68901162727221e-7) * cj1)) + (((-1.0) * x463 * x464)) + (((0.251106193846207) * r01 * x460)) + ((x453 * x459)) + (((-0.251106194690265) * r21 * x464)) + (((-1.0) * x456 * x458)) + (((-1.0) * cj1 * x455)) + ((sj1 * x457)) + (((-1.0) * x453 * x454)) + ((x460 * x461)) + ((x458 * x462)))) + IKsqr(((-1.03318584070796) + (((-0.251106193846207) * r01 * x464)) + ((x453 * x456)) + ((cj1 * x457)) + (((-0.251106194690265) * r21 * x460)) + (((-1.0) * x460 * x463)) + (((-1.0) * x454 * x458)) + (((-2.68901162727221e-7) * sj1)) + (((-1.0) * x453 * x462)) + ((sj1 * x455)) + (((-1.0) * x461 * x464)) + ((x458 * x459)))) - 1) <= IKFAST_SINCOS_THRESH)
                                                        continue;
                                                    j2array[0] = IKatan2(((((2.68901162727221e-7) * cj1)) + (((-1.0) * x463 * x464)) + (((0.251106193846207) * r01 * x460)) + ((x453 * x459)) + (((-0.251106194690265) * r21 * x464)) + (((-1.0) * x456 * x458)) + (((-1.0) * cj1 * x455)) + ((sj1 * x457)) + (((-1.0) * x453 * x454)) + ((x460 * x461)) + ((x458 * x462))), ((-1.03318584070796) + (((-0.251106193846207) * r01 * x464)) + ((x453 * x456)) + ((cj1 * x457)) + (((-0.251106194690265) * r21 * x460)) + (((-1.0) * x460 * x463)) + (((-1.0) * x454 * x458)) + (((-2.68901162727221e-7) * sj1)) + (((-1.0) * x453 * x462)) + ((sj1 * x455)) + (((-1.0) * x461 * x464)) + ((x458 * x459))));
                                                    sj2array[0] = IKsin(j2array[0]);
                                                    cj2array[0] = IKcos(j2array[0]);
                                                    if (j2array[0] > IKPI)
                                                    {
                                                        j2array[0] -= IK2PI;
                                                    }
                                                    else if (j2array[0] < -IKPI)
                                                    {
                                                        j2array[0] += IK2PI;
                                                    }
                                                    j2valid[0] = true;
                                                    for (int ij2 = 0; ij2 < 1; ++ij2)
                                                    {
                                                        if (!j2valid[ij2])
                                                        {
                                                            continue;
                                                        }
                                                        _ij2[0] = ij2;
                                                        _ij2[1] = -1;
                                                        for (int iij2 = ij2 + 1; iij2 < 1; ++iij2)
                                                        {
                                                            if (j2valid[iij2] && IKabs(cj2array[ij2] - cj2array[iij2]) < IKFAST_SOLUTION_THRESH && IKabs(sj2array[ij2] - sj2array[iij2]) < IKFAST_SOLUTION_THRESH)
                                                            {
                                                                j2valid[iij2] = false;
                                                                _ij2[1] = iij2;
                                                                break;
                                                            }
                                                        }
                                                        j2 = j2array[ij2];
                                                        cj2 = cj2array[ij2];
                                                        sj2 = sj2array[ij2];
                                                        {
                                                            IkReal evalcond[3];
                                                            IkReal x467 = IKcos(j2);
                                                            IkReal x468 = IKsin(j2);
                                                            IkReal x469 = ((3.7e-7) * cj5);
                                                            IkReal x470 = ((3.7e-7) * sj5);
                                                            IkReal x471 = ((0.1135) * cj5);
                                                            IkReal x472 = ((0.1135) * sj5);
                                                            IkReal x473 = (cj1 * x468);
                                                            IkReal x474 = (sj1 * x467);
                                                            evalcond[0] = (((r21 * x472)) + (((0.452) * sj1 * x468)) + ((r20 * x470)) + (((0.467) * cj1)) + (((0.452) * cj1 * x467)) + (((-1.0) * pz)) + ((r21 * x469)) + (((-1.0) * r20 * x471)));
                                                            evalcond[1] = ((1.21543325961255e-7) + (((-0.452000001519335) * x473)) + (((-1.0) * r00 * x471)) + (((0.452000001519335) * x474)) + (((-1.0) * px)) + (((0.467000001569756) * sj1)) + ((r01 * x469)) + ((r00 * x470)) + ((r01 * x472)));
                                                            evalcond[2] = ((0.11339999909077) + ((r10 * x470)) + (((8.79096604904732e-10) * x473)) + (((-9.08270164802013e-10) * sj1)) + ((r11 * x472)) + (((-1.0) * py)) + (((-8.79096604904732e-10) * x474)) + (((-1.0) * r10 * x471)) + ((r11 * x469)));
                                                            if (IKabs(evalcond[0]) > IKFAST_EVALCOND_THRESH || IKabs(evalcond[1]) > IKFAST_EVALCOND_THRESH || IKabs(evalcond[2]) > IKFAST_EVALCOND_THRESH)
                                                            {
                                                                continue;
                                                            }
                                                        }

                                                        {
                                                            IkReal j3eval[2];
                                                            sj0 = 0.98079532;
                                                            cj0 = -0.19503986;
                                                            j0 = 1.76709441189614;
                                                            IkReal x475 = (((cj5 * r11 * sj4)) + ((r10 * sj4 * sj5)) + (((-1.0) * cj4 * r12)));
                                                            j3eval[0] = x475;
                                                            j3eval[1] = IKsign(x475);
                                                            if (IKabs(j3eval[0]) < 0.0000010000000000 || IKabs(j3eval[1]) < 0.0000010000000000)
                                                            {
                                                                {
                                                                    IkReal j3eval[2];
                                                                    sj0 = 0.98079532;
                                                                    cj0 = -0.19503986;
                                                                    j0 = 1.76709441189614;
                                                                    IkReal x476 = ((1.0) * sj4);
                                                                    IkReal x477 = ((((-1.0) * r00 * sj5 * x476)) + (((-1.0) * cj5 * r01 * x476)) + ((cj4 * r02)));
                                                                    j3eval[0] = x477;
                                                                    j3eval[1] = IKsign(x477);
                                                                    if (IKabs(j3eval[0]) < 0.0000010000000000 || IKabs(j3eval[1]) < 0.0000010000000000)
                                                                    {
                                                                        {
                                                                            IkReal j3eval[2];
                                                                            sj0 = 0.98079532;
                                                                            cj0 = -0.19503986;
                                                                            j0 = 1.76709441189614;
                                                                            IkReal x478 = cj4 * cj4;
                                                                            IkReal x479 = cj5 * cj5;
                                                                            IkReal x480 = r22 * r22;
                                                                            IkReal x481 = r21 * r21;
                                                                            IkReal x482 = r20 * r20;
                                                                            IkReal x483 = (r20 * sj5);
                                                                            IkReal x484 = (cj5 * r21);
                                                                            IkReal x485 = ((1.0) * x481);
                                                                            IkReal x486 = ((1.0) * x482);
                                                                            IkReal x487 = (x478 * x479);
                                                                            IkReal x488 = ((2.0) * cj4 * r22 * sj4);
                                                                            IkReal x489 = ((((-1.0) * x484 * x488)) + (((-2.0) * x478 * x483 * x484)) + (((-1.0) * x483 * x488)) + (((-1.0) * x485)) + (((-1.0) * x480)) + ((x478 * x480)) + (((2.0) * x483 * x484)) + (((-1.0) * x479 * x486)) + ((x482 * x487)) + ((x479 * x481)) + (((-1.0) * x478 * x486)) + (((-1.0) * x485 * x487)));
                                                                            j3eval[0] = x489;
                                                                            j3eval[1] = IKsign(x489);
                                                                            if (IKabs(j3eval[0]) < 0.0000010000000000 || IKabs(j3eval[1]) < 0.0000010000000000)
                                                                            {
                                                                                continue; // no branches [j3]
                                                                            }
                                                                            else
                                                                            {
                                                                                {
                                                                                    IkReal j3array[1], cj3array[1], sj3array[1];
                                                                                    bool j3valid[1] = {false};
                                                                                    _nj3 = 1;
                                                                                    IkReal x490 = cj4 * cj4;
                                                                                    IkReal x491 = cj5 * cj5;
                                                                                    IkReal x492 = r22 * r22;
                                                                                    IkReal x493 = r21 * r21;
                                                                                    IkReal x494 = r20 * r20;
                                                                                    IkReal x495 = ((1.0) * sj1);
                                                                                    IkReal x496 = (r22 * sj4);
                                                                                    IkReal x497 = (sj2 * sj5);
                                                                                    IkReal x498 = (cj4 * r20);
                                                                                    IkReal x499 = (cj5 * sj2);
                                                                                    IkReal x500 = (r21 * sj5);
                                                                                    IkReal x501 = (cj4 * r21);
                                                                                    IkReal x502 = ((2.0) * cj5);
                                                                                    IkReal x503 = (cj2 * cj5 * r20);
                                                                                    IkReal x504 = ((1.0) * x493);
                                                                                    IkReal x505 = ((1.0) * cj1 * cj2);
                                                                                    IkReal x506 = ((1.0) * x494);
                                                                                    IkReal x507 = (x490 * x491);
                                                                                    CheckValue<IkReal> x508 = IKPowWithIntegerCheck(IKsign((((x494 * x507)) + ((x491 * x493)) + (((-1.0) * x504)) + ((x490 * x492)) + (((-1.0) * x491 * x506)) + (((-1.0) * x492)) + (((-1.0) * x490 * x506)) + (((-2.0) * sj5 * x496 * x498)) + ((r20 * x500 * x502)) + (((-1.0) * x496 * x501 * x502)) + (((-1.0) * x504 * x507)) + (((-1.0) * r20 * x490 * x500 * x502)))), -1);
                                                                                    if (!x508.valid)
                                                                                    {
                                                                                        continue;
                                                                                    }
                                                                                    CheckValue<IkReal> x509 = IKatan2WithCheck(IkReal((((cj2 * sj1 * x500)) + (((-1.0) * x495 * x499 * x501)) + (((-1.0) * cj5 * x501 * x505)) + (((-1.0) * x496 * x505)) + (((-1.0) * x495 * x503)) + (((-1.0) * sj2 * x495 * x496)) + (((-1.0) * cj1 * r21 * x497)) + ((cj1 * r20 * x499)) + (((-1.0) * sj5 * x498 * x505)) + (((-1.0) * x495 * x497 * x498)))), IkReal(((((-1.0) * cj2 * sj5 * x495 * x498)) + (((-1.0) * r21 * x495 * x497)) + ((cj1 * x503)) + ((r20 * sj1 * x499)) + (((-1.0) * cj2 * x495 * x496)) + ((cj1 * sj2 * x496)) + ((cj1 * x499 * x501)) + ((cj1 * x497 * x498)) + (((-1.0) * cj2 * cj5 * x495 * x501)) + (((-1.0) * x500 * x505)))), IKFAST_ATAN2_MAGTHRESH);
                                                                                    if (!x509.valid)
                                                                                    {
                                                                                        continue;
                                                                                    }
                                                                                    j3array[0] = ((-1.5707963267949) + (((1.5707963267949) * (x508.value))) + (x509.value));
                                                                                    sj3array[0] = IKsin(j3array[0]);
                                                                                    cj3array[0] = IKcos(j3array[0]);
                                                                                    if (j3array[0] > IKPI)
                                                                                    {
                                                                                        j3array[0] -= IK2PI;
                                                                                    }
                                                                                    else if (j3array[0] < -IKPI)
                                                                                    {
                                                                                        j3array[0] += IK2PI;
                                                                                    }
                                                                                    j3valid[0] = true;
                                                                                    for (int ij3 = 0; ij3 < 1; ++ij3)
                                                                                    {
                                                                                        if (!j3valid[ij3])
                                                                                        {
                                                                                            continue;
                                                                                        }
                                                                                        _ij3[0] = ij3;
                                                                                        _ij3[1] = -1;
                                                                                        for (int iij3 = ij3 + 1; iij3 < 1; ++iij3)
                                                                                        {
                                                                                            if (j3valid[iij3] && IKabs(cj3array[ij3] - cj3array[iij3]) < IKFAST_SOLUTION_THRESH && IKabs(sj3array[ij3] - sj3array[iij3]) < IKFAST_SOLUTION_THRESH)
                                                                                            {
                                                                                                j3valid[iij3] = false;
                                                                                                _ij3[1] = iij3;
                                                                                                break;
                                                                                            }
                                                                                        }
                                                                                        j3 = j3array[ij3];
                                                                                        cj3 = cj3array[ij3];
                                                                                        sj3 = sj3array[ij3];
                                                                                        {
                                                                                            IkReal evalcond[6];
                                                                                            IkReal x510 = IKcos(j3);
                                                                                            IkReal x511 = IKsin(j3);
                                                                                            IkReal x512 = (r00 * sj5);
                                                                                            IkReal x513 = ((1.94490399315206e-9) * cj1);
                                                                                            IkReal x514 = (sj1 * sj2);
                                                                                            IkReal x515 = (cj2 * sj1);
                                                                                            IkReal x516 = ((1.0) * r11);
                                                                                            IkReal x517 = (cj1 * sj2);
                                                                                            IkReal x518 = (cj1 * cj2);
                                                                                            IkReal x519 = (cj5 * x510);
                                                                                            IkReal x520 = (sj4 * x510);
                                                                                            IkReal x521 = (sj5 * x510);
                                                                                            IkReal x522 = (cj5 * x511);
                                                                                            IkReal x523 = ((1.0) * x511);
                                                                                            IkReal x524 = (cj4 * x523);
                                                                                            evalcond[0] = ((((-1.0) * r21 * sj5 * x523)) + ((r20 * x522)) + (((-1.0) * x515)) + ((r22 * x520)) + ((cj4 * r20 * x521)) + x517 + ((cj4 * r21 * x519)));
                                                                                            evalcond[1] = ((((-1.0) * cj4 * r21 * x522)) + (((-1.0) * r21 * x521)) + (((-1.0) * r20 * sj5 * x524)) + ((r20 * x519)) + x518 + x514 + (((-1.0) * r22 * sj4 * x523)));
                                                                                            evalcond[2] = ((((-1.0) * r01 * sj5 * x523)) + ((r02 * x520)) + (((1.00000000336136) * x518)) + (((1.00000000336136) * x514)) + ((cj4 * x510 * x512)) + ((cj4 * r01 * x519)) + ((r00 * x522)));
                                                                                            evalcond[3] = (((cj4 * r11 * x519)) + ((r12 * x520)) + ((cj4 * r10 * x521)) + (((-1.0) * cj2 * x513)) + (((-1.0) * sj5 * x511 * x516)) + ((r10 * x522)) + (((-1.94490399315206e-9) * x514)));
                                                                                            evalcond[4] = ((((1.00000000336136) * x515)) + (((-1.0) * cj4 * r01 * x522)) + (((-1.0) * x512 * x524)) + (((-1.00000000336136) * x517)) + (((-1.0) * r02 * sj4 * x523)) + (((-1.0) * r01 * x521)) + ((r00 * x519)));
                                                                                            evalcond[5] = (((r10 * x519)) + (((-1.0) * x516 * x521)) + (((-1.94490399315206e-9) * x515)) + ((sj2 * x513)) + (((-1.0) * cj4 * x516 * x522)) + (((-1.0) * r10 * sj5 * x524)) + (((-1.0) * r12 * sj4 * x523)));
                                                                                            if (IKabs(evalcond[0]) > IKFAST_EVALCOND_THRESH || IKabs(evalcond[1]) > IKFAST_EVALCOND_THRESH || IKabs(evalcond[2]) > IKFAST_EVALCOND_THRESH || IKabs(evalcond[3]) > IKFAST_EVALCOND_THRESH || IKabs(evalcond[4]) > IKFAST_EVALCOND_THRESH || IKabs(evalcond[5]) > IKFAST_EVALCOND_THRESH)
                                                                                            {
                                                                                                continue;
                                                                                            }
                                                                                        }

                                                                                        {
                                                                                            std::vector<IkSingleDOFSolutionBase<IkReal>> vinfos(6);
                                                                                            vinfos[0].jointtype = 1;
                                                                                            vinfos[0].foffset = j0;
                                                                                            vinfos[0].indices[0] = _ij0[0];
                                                                                            vinfos[0].indices[1] = _ij0[1];
                                                                                            vinfos[0].maxsolutions = _nj0;
                                                                                            vinfos[1].jointtype = 1;
                                                                                            vinfos[1].foffset = j1;
                                                                                            vinfos[1].indices[0] = _ij1[0];
                                                                                            vinfos[1].indices[1] = _ij1[1];
                                                                                            vinfos[1].maxsolutions = _nj1;
                                                                                            vinfos[2].jointtype = 1;
                                                                                            vinfos[2].foffset = j2;
                                                                                            vinfos[2].indices[0] = _ij2[0];
                                                                                            vinfos[2].indices[1] = _ij2[1];
                                                                                            vinfos[2].maxsolutions = _nj2;
                                                                                            vinfos[3].jointtype = 1;
                                                                                            vinfos[3].foffset = j3;
                                                                                            vinfos[3].indices[0] = _ij3[0];
                                                                                            vinfos[3].indices[1] = _ij3[1];
                                                                                            vinfos[3].maxsolutions = _nj3;
                                                                                            vinfos[4].jointtype = 1;
                                                                                            vinfos[4].foffset = j4;
                                                                                            vinfos[4].indices[0] = _ij4[0];
                                                                                            vinfos[4].indices[1] = _ij4[1];
                                                                                            vinfos[4].maxsolutions = _nj4;
                                                                                            vinfos[5].jointtype = 1;
                                                                                            vinfos[5].foffset = j5;
                                                                                            vinfos[5].indices[0] = _ij5[0];
                                                                                            vinfos[5].indices[1] = _ij5[1];
                                                                                            vinfos[5].maxsolutions = _nj5;
                                                                                            std::vector<int> vfree(0);
                                                                                            solutions.AddSolution(vinfos, vfree);
                                                                                        }
                                                                                    }
                                                                                }
                                                                            }
                                                                        }
                                                                    }
                                                                    else
                                                                    {
                                                                        {
                                                                            IkReal j3array[1], cj3array[1], sj3array[1];
                                                                            bool j3valid[1] = {false};
                                                                            _nj3 = 1;
                                                                            IkReal x525 = (cj5 * sj2);
                                                                            IkReal x526 = (cj1 * cj4);
                                                                            IkReal x527 = ((1.94490399315206e-9) * r20);
                                                                            IkReal x528 = (sj2 * sj4);
                                                                            IkReal x529 = ((1.0) * sj4);
                                                                            IkReal x530 = (cj2 * cj5);
                                                                            IkReal x531 = ((1.94490399315206e-9) * sj1);
                                                                            IkReal x532 = (cj2 * sj1);
                                                                            IkReal x533 = ((1.0) * r10);
                                                                            IkReal x534 = (cj4 * sj5);
                                                                            IkReal x535 = (sj2 * sj5);
                                                                            IkReal x536 = ((1.0) * r11);
                                                                            IkReal x537 = ((1.94490399315206e-9) * cj1 * cj2 * sj5);
                                                                            CheckValue<IkReal> x538 = IKatan2WithCheck(IkReal((((cj4 * r21 * x525 * x531)) + ((r22 * x528 * x531)) + (((1.94490399315206e-9) * r21 * x526 * x530)) + ((r11 * x525 * x526)) + ((r10 * x526 * x535)) + ((sj1 * sj2 * x527 * x534)) + (((-1.0) * x532 * x533 * x534)) + (((-1.0) * r12 * x529 * x532)) + ((cj1 * r12 * x528)) + (((1.94490399315206e-9) * cj1 * cj2 * r22 * sj4)) + (((-1.0) * cj4 * sj1 * x530 * x536)) + ((cj2 * sj5 * x526 * x527)))), IkReal((((r21 * x531 * x535)) + ((r10 * sj1 * x530)) + (((-1.0) * cj1 * x525 * x533)) + (((-1.0) * cj1 * x527 * x530)) + ((cj1 * r11 * x535)) + (((-1.0) * sj1 * x525 * x527)) + ((r21 * x537)) + (((-1.0) * sj5 * x532 * x536)))), IKFAST_ATAN2_MAGTHRESH);
                                                                            if (!x538.valid)
                                                                            {
                                                                                continue;
                                                                            }
                                                                            CheckValue<IkReal> x539 = IKPowWithIntegerCheck(IKsign(((((-1.0) * cj5 * r01 * x529)) + (((-1.0) * r00 * sj5 * x529)) + ((cj4 * r02)))), -1);
                                                                            if (!x539.valid)
                                                                            {
                                                                                continue;
                                                                            }
                                                                            j3array[0] = ((-1.5707963267949) + (x538.value) + (((1.5707963267949) * (x539.value))));
                                                                            sj3array[0] = IKsin(j3array[0]);
                                                                            cj3array[0] = IKcos(j3array[0]);
                                                                            if (j3array[0] > IKPI)
                                                                            {
                                                                                j3array[0] -= IK2PI;
                                                                            }
                                                                            else if (j3array[0] < -IKPI)
                                                                            {
                                                                                j3array[0] += IK2PI;
                                                                            }
                                                                            j3valid[0] = true;
                                                                            for (int ij3 = 0; ij3 < 1; ++ij3)
                                                                            {
                                                                                if (!j3valid[ij3])
                                                                                {
                                                                                    continue;
                                                                                }
                                                                                _ij3[0] = ij3;
                                                                                _ij3[1] = -1;
                                                                                for (int iij3 = ij3 + 1; iij3 < 1; ++iij3)
                                                                                {
                                                                                    if (j3valid[iij3] && IKabs(cj3array[ij3] - cj3array[iij3]) < IKFAST_SOLUTION_THRESH && IKabs(sj3array[ij3] - sj3array[iij3]) < IKFAST_SOLUTION_THRESH)
                                                                                    {
                                                                                        j3valid[iij3] = false;
                                                                                        _ij3[1] = iij3;
                                                                                        break;
                                                                                    }
                                                                                }
                                                                                j3 = j3array[ij3];
                                                                                cj3 = cj3array[ij3];
                                                                                sj3 = sj3array[ij3];
                                                                                {
                                                                                    IkReal evalcond[6];
                                                                                    IkReal x540 = IKcos(j3);
                                                                                    IkReal x541 = IKsin(j3);
                                                                                    IkReal x542 = (r00 * sj5);
                                                                                    IkReal x543 = ((1.94490399315206e-9) * cj1);
                                                                                    IkReal x544 = (sj1 * sj2);
                                                                                    IkReal x545 = (cj2 * sj1);
                                                                                    IkReal x546 = ((1.0) * r11);
                                                                                    IkReal x547 = (cj1 * sj2);
                                                                                    IkReal x548 = (cj1 * cj2);
                                                                                    IkReal x549 = (cj5 * x540);
                                                                                    IkReal x550 = (sj4 * x540);
                                                                                    IkReal x551 = (sj5 * x540);
                                                                                    IkReal x552 = (cj5 * x541);
                                                                                    IkReal x553 = ((1.0) * x541);
                                                                                    IkReal x554 = (cj4 * x553);
                                                                                    evalcond[0] = ((((-1.0) * x545)) + (((-1.0) * r21 * sj5 * x553)) + ((r20 * x552)) + x547 + ((r22 * x550)) + ((cj4 * r21 * x549)) + ((cj4 * r20 * x551)));
                                                                                    evalcond[1] = ((((-1.0) * cj4 * r21 * x552)) + ((r20 * x549)) + (((-1.0) * r20 * sj5 * x554)) + (((-1.0) * r22 * sj4 * x553)) + (((-1.0) * r21 * x551)) + x544 + x548);
                                                                                    evalcond[2] = ((((-1.0) * r01 * sj5 * x553)) + ((r02 * x550)) + ((cj4 * r01 * x549)) + ((cj4 * x540 * x542)) + ((r00 * x552)) + (((1.00000000336136) * x544)) + (((1.00000000336136) * x548)));
                                                                                    evalcond[3] = ((((-1.94490399315206e-9) * x544)) + ((cj4 * r10 * x551)) + ((r12 * x550)) + (((-1.0) * cj2 * x543)) + ((cj4 * r11 * x549)) + ((r10 * x552)) + (((-1.0) * sj5 * x541 * x546)));
                                                                                    evalcond[4] = ((((-1.0) * cj4 * r01 * x552)) + (((-1.0) * r02 * sj4 * x553)) + (((-1.00000000336136) * x547)) + (((1.00000000336136) * x545)) + (((-1.0) * x542 * x554)) + ((r00 * x549)) + (((-1.0) * r01 * x551)));
                                                                                    evalcond[5] = ((((-1.94490399315206e-9) * x545)) + (((-1.0) * cj4 * x546 * x552)) + ((sj2 * x543)) + ((r10 * x549)) + (((-1.0) * r10 * sj5 * x554)) + (((-1.0) * r12 * sj4 * x553)) + (((-1.0) * x546 * x551)));
                                                                                    if (IKabs(evalcond[0]) > IKFAST_EVALCOND_THRESH || IKabs(evalcond[1]) > IKFAST_EVALCOND_THRESH || IKabs(evalcond[2]) > IKFAST_EVALCOND_THRESH || IKabs(evalcond[3]) > IKFAST_EVALCOND_THRESH || IKabs(evalcond[4]) > IKFAST_EVALCOND_THRESH || IKabs(evalcond[5]) > IKFAST_EVALCOND_THRESH)
                                                                                    {
                                                                                        continue;
                                                                                    }
                                                                                }

                                                                                {
                                                                                    std::vector<IkSingleDOFSolutionBase<IkReal>> vinfos(6);
                                                                                    vinfos[0].jointtype = 1;
                                                                                    vinfos[0].foffset = j0;
                                                                                    vinfos[0].indices[0] = _ij0[0];
                                                                                    vinfos[0].indices[1] = _ij0[1];
                                                                                    vinfos[0].maxsolutions = _nj0;
                                                                                    vinfos[1].jointtype = 1;
                                                                                    vinfos[1].foffset = j1;
                                                                                    vinfos[1].indices[0] = _ij1[0];
                                                                                    vinfos[1].indices[1] = _ij1[1];
                                                                                    vinfos[1].maxsolutions = _nj1;
                                                                                    vinfos[2].jointtype = 1;
                                                                                    vinfos[2].foffset = j2;
                                                                                    vinfos[2].indices[0] = _ij2[0];
                                                                                    vinfos[2].indices[1] = _ij2[1];
                                                                                    vinfos[2].maxsolutions = _nj2;
                                                                                    vinfos[3].jointtype = 1;
                                                                                    vinfos[3].foffset = j3;
                                                                                    vinfos[3].indices[0] = _ij3[0];
                                                                                    vinfos[3].indices[1] = _ij3[1];
                                                                                    vinfos[3].maxsolutions = _nj3;
                                                                                    vinfos[4].jointtype = 1;
                                                                                    vinfos[4].foffset = j4;
                                                                                    vinfos[4].indices[0] = _ij4[0];
                                                                                    vinfos[4].indices[1] = _ij4[1];
                                                                                    vinfos[4].maxsolutions = _nj4;
                                                                                    vinfos[5].jointtype = 1;
                                                                                    vinfos[5].foffset = j5;
                                                                                    vinfos[5].indices[0] = _ij5[0];
                                                                                    vinfos[5].indices[1] = _ij5[1];
                                                                                    vinfos[5].maxsolutions = _nj5;
                                                                                    std::vector<int> vfree(0);
                                                                                    solutions.AddSolution(vinfos, vfree);
                                                                                }
                                                                            }
                                                                        }
                                                                    }
                                                                }
                                                            }
                                                            else
                                                            {
                                                                {
                                                                    IkReal j3array[1], cj3array[1], sj3array[1];
                                                                    bool j3valid[1] = {false};
                                                                    _nj3 = 1;
                                                                    IkReal x555 = (sj2 * sj5);
                                                                    IkReal x556 = ((1.00000000336136) * cj4);
                                                                    IkReal x557 = (cj1 * r20);
                                                                    IkReal x558 = ((1.0) * r01);
                                                                    IkReal x559 = (r02 * sj4);
                                                                    IkReal x560 = (cj1 * cj2);
                                                                    IkReal x561 = (cj4 * r00);
                                                                    IkReal x562 = (sj1 * sj2);
                                                                    IkReal x563 = (cj1 * sj2);
                                                                    IkReal x564 = (cj2 * sj1);
                                                                    IkReal x565 = ((1.00000000336136) * cj5);
                                                                    IkReal x566 = (cj5 * r00);
                                                                    IkReal x567 = (r21 * x564);
                                                                    IkReal x568 = (cj4 * cj5 * r01);
                                                                    IkReal x569 = ((1.00000000336136) * r22 * sj4);
                                                                    CheckValue<IkReal> x570 = IKPowWithIntegerCheck(IKsign((((cj5 * r11 * sj4)) + ((r10 * sj4 * sj5)) + (((-1.0) * cj4 * r12)))), -1);
                                                                    if (!x570.valid)
                                                                    {
                                                                        continue;
                                                                    }
                                                                    CheckValue<IkReal> x571 = IKatan2WithCheck(IkReal(((((1.00000000336136) * sj5 * x567)) + (((-1.0) * r20 * x564 * x565)) + ((sj2 * x557 * x565)) + ((x562 * x566)) + (((-1.00000000336136) * cj1 * r21 * x555)) + ((x560 * x566)) + (((-1.0) * sj1 * x555 * x558)) + (((-1.0) * sj5 * x558 * x560)))), IkReal(((((-1.0) * x564 * x569)) + ((sj5 * x560 * x561)) + ((x562 * x568)) + ((x563 * x569)) + ((x555 * x556 * x557)) + ((sj1 * x555 * x561)) + ((x560 * x568)) + (((-1.0) * r20 * sj5 * x556 * x564)) + ((cj5 * r21 * x556 * x563)) + (((-1.0) * cj5 * x556 * x567)) + ((x559 * x562)) + ((x559 * x560)))), IKFAST_ATAN2_MAGTHRESH);
                                                                    if (!x571.valid)
                                                                    {
                                                                        continue;
                                                                    }
                                                                    j3array[0] = ((-1.5707963267949) + (((1.5707963267949) * (x570.value))) + (x571.value));
                                                                    sj3array[0] = IKsin(j3array[0]);
                                                                    cj3array[0] = IKcos(j3array[0]);
                                                                    if (j3array[0] > IKPI)
                                                                    {
                                                                        j3array[0] -= IK2PI;
                                                                    }
                                                                    else if (j3array[0] < -IKPI)
                                                                    {
                                                                        j3array[0] += IK2PI;
                                                                    }
                                                                    j3valid[0] = true;
                                                                    for (int ij3 = 0; ij3 < 1; ++ij3)
                                                                    {
                                                                        if (!j3valid[ij3])
                                                                        {
                                                                            continue;
                                                                        }
                                                                        _ij3[0] = ij3;
                                                                        _ij3[1] = -1;
                                                                        for (int iij3 = ij3 + 1; iij3 < 1; ++iij3)
                                                                        {
                                                                            if (j3valid[iij3] && IKabs(cj3array[ij3] - cj3array[iij3]) < IKFAST_SOLUTION_THRESH && IKabs(sj3array[ij3] - sj3array[iij3]) < IKFAST_SOLUTION_THRESH)
                                                                            {
                                                                                j3valid[iij3] = false;
                                                                                _ij3[1] = iij3;
                                                                                break;
                                                                            }
                                                                        }
                                                                        j3 = j3array[ij3];
                                                                        cj3 = cj3array[ij3];
                                                                        sj3 = sj3array[ij3];
                                                                        {
                                                                            IkReal evalcond[6];
                                                                            IkReal x572 = IKcos(j3);
                                                                            IkReal x573 = IKsin(j3);
                                                                            IkReal x574 = (r00 * sj5);
                                                                            IkReal x575 = ((1.94490399315206e-9) * cj1);
                                                                            IkReal x576 = (sj1 * sj2);
                                                                            IkReal x577 = (cj2 * sj1);
                                                                            IkReal x578 = ((1.0) * r11);
                                                                            IkReal x579 = (cj1 * sj2);
                                                                            IkReal x580 = (cj1 * cj2);
                                                                            IkReal x581 = (cj5 * x572);
                                                                            IkReal x582 = (sj4 * x572);
                                                                            IkReal x583 = (sj5 * x572);
                                                                            IkReal x584 = (cj5 * x573);
                                                                            IkReal x585 = ((1.0) * x573);
                                                                            IkReal x586 = (cj4 * x585);
                                                                            evalcond[0] = (((r20 * x584)) + ((cj4 * r20 * x583)) + (((-1.0) * x577)) + x579 + ((cj4 * r21 * x581)) + ((r22 * x582)) + (((-1.0) * r21 * sj5 * x585)));
                                                                            evalcond[1] = (((r20 * x581)) + (((-1.0) * r22 * sj4 * x585)) + (((-1.0) * r21 * x583)) + (((-1.0) * cj4 * r21 * x584)) + x576 + x580 + (((-1.0) * r20 * sj5 * x586)));
                                                                            evalcond[2] = ((((1.00000000336136) * x576)) + ((r02 * x582)) + ((cj4 * x572 * x574)) + (((1.00000000336136) * x580)) + ((r00 * x584)) + (((-1.0) * r01 * sj5 * x585)) + ((cj4 * r01 * x581)));
                                                                            evalcond[3] = ((((-1.0) * cj2 * x575)) + ((r12 * x582)) + (((-1.0) * sj5 * x573 * x578)) + ((r10 * x584)) + ((cj4 * r11 * x581)) + ((cj4 * r10 * x583)) + (((-1.94490399315206e-9) * x576)));
                                                                            evalcond[4] = ((((1.00000000336136) * x577)) + (((-1.0) * r02 * sj4 * x585)) + (((-1.0) * cj4 * r01 * x584)) + (((-1.0) * x574 * x586)) + ((r00 * x581)) + (((-1.00000000336136) * x579)) + (((-1.0) * r01 * x583)));
                                                                            evalcond[5] = ((((-1.0) * x578 * x583)) + ((r10 * x581)) + (((-1.0) * r12 * sj4 * x585)) + (((-1.0) * cj4 * x578 * x584)) + ((sj2 * x575)) + (((-1.0) * r10 * sj5 * x586)) + (((-1.94490399315206e-9) * x577)));
                                                                            if (IKabs(evalcond[0]) > IKFAST_EVALCOND_THRESH || IKabs(evalcond[1]) > IKFAST_EVALCOND_THRESH || IKabs(evalcond[2]) > IKFAST_EVALCOND_THRESH || IKabs(evalcond[3]) > IKFAST_EVALCOND_THRESH || IKabs(evalcond[4]) > IKFAST_EVALCOND_THRESH || IKabs(evalcond[5]) > IKFAST_EVALCOND_THRESH)
                                                                            {
                                                                                continue;
                                                                            }
                                                                        }

                                                                        {
                                                                            std::vector<IkSingleDOFSolutionBase<IkReal>> vinfos(6);
                                                                            vinfos[0].jointtype = 1;
                                                                            vinfos[0].foffset = j0;
                                                                            vinfos[0].indices[0] = _ij0[0];
                                                                            vinfos[0].indices[1] = _ij0[1];
                                                                            vinfos[0].maxsolutions = _nj0;
                                                                            vinfos[1].jointtype = 1;
                                                                            vinfos[1].foffset = j1;
                                                                            vinfos[1].indices[0] = _ij1[0];
                                                                            vinfos[1].indices[1] = _ij1[1];
                                                                            vinfos[1].maxsolutions = _nj1;
                                                                            vinfos[2].jointtype = 1;
                                                                            vinfos[2].foffset = j2;
                                                                            vinfos[2].indices[0] = _ij2[0];
                                                                            vinfos[2].indices[1] = _ij2[1];
                                                                            vinfos[2].maxsolutions = _nj2;
                                                                            vinfos[3].jointtype = 1;
                                                                            vinfos[3].foffset = j3;
                                                                            vinfos[3].indices[0] = _ij3[0];
                                                                            vinfos[3].indices[1] = _ij3[1];
                                                                            vinfos[3].maxsolutions = _nj3;
                                                                            vinfos[4].jointtype = 1;
                                                                            vinfos[4].foffset = j4;
                                                                            vinfos[4].indices[0] = _ij4[0];
                                                                            vinfos[4].indices[1] = _ij4[1];
                                                                            vinfos[4].maxsolutions = _nj4;
                                                                            vinfos[5].jointtype = 1;
                                                                            vinfos[5].foffset = j5;
                                                                            vinfos[5].indices[0] = _ij5[0];
                                                                            vinfos[5].indices[1] = _ij5[1];
                                                                            vinfos[5].maxsolutions = _nj5;
                                                                            std::vector<int> vfree(0);
                                                                            solutions.AddSolution(vinfos, vfree);
                                                                        }
                                                                    }
                                                                }
                                                            }
                                                        }
                                                    }
                                                }
                                            }
                                        } while (0);
                                        if (bgotonextstatement)
                                        {
                                            bool bgotonextstatement = true;
                                            do
                                            {
                                                evalcond[0] = ((-3.14159265358979) + (IKfmod(((3.14159265358979) + (IKabs(((2.94529456182618) + j0)))), 6.28318530717959)));
                                                if (IKabs(evalcond[0]) < 0.0000050000000000)
                                                {
                                                    bgotonextstatement = false;
                                                    {
                                                        IkReal j2array[1], cj2array[1], sj2array[1];
                                                        bool j2valid[1] = {false};
                                                        _nj2 = 1;
                                                        IkReal x587 = (cj5 * sj1);
                                                        IkReal x588 = ((8.1858407079646e-7) * r21);
                                                        IkReal x589 = (cj1 * cj5);
                                                        IkReal x590 = ((420.886621488087) * r01);
                                                        IkReal x591 = ((2.21238938053097) * pz);
                                                        IkReal x592 = ((1137531409.42726) * px);
                                                        IkReal x593 = ((0.251106194690265) * r20);
                                                        IkReal x594 = (cj1 * sj5);
                                                        IkReal x595 = ((129109814.969994) * r01);
                                                        IkReal x596 = ((129109814.969994) * r00);
                                                        IkReal x597 = ((8.1858407079646e-7) * r20);
                                                        IkReal x598 = (sj1 * sj5);
                                                        IkReal x599 = ((420.886621488087) * r00);
                                                        IkReal x600 = ((0.251106194690265) * r21 * sj5);
                                                        if (IKabs((((x594 * x595)) + ((x594 * x599)) + (((-1.0) * x597 * x598)) + (((-1.0) * x589 * x596)) + (((-0.251106194690265) * r21 * x598)) + (((-1.0) * x587 * x588)) + (((-1.0) * cj1 * x592)) + ((sj1 * x591)) + (((-128996060.794774) * cj1)) + ((x589 * x590)) + ((x587 * x593)))) < IKFAST_ATAN2_MAGTHRESH && IKabs(((-1.03318584070796) + (((-1.0) * x598 * x599)) + (((-1.0) * x587 * x590)) + (((-0.251106194690265) * r21 * x594)) + (((-1.0) * x588 * x589)) + ((cj1 * x591)) + (((-1.0) * x594 * x597)) + (((-1.0) * x595 * x598)) + (((128996060.794774) * sj1)) + ((sj1 * x592)) + ((x589 * x593)) + ((x587 * x596)))) < IKFAST_ATAN2_MAGTHRESH && IKabs(IKsqr((((x594 * x595)) + ((x594 * x599)) + (((-1.0) * x597 * x598)) + (((-1.0) * x589 * x596)) + (((-0.251106194690265) * r21 * x598)) + (((-1.0) * x587 * x588)) + (((-1.0) * cj1 * x592)) + ((sj1 * x591)) + (((-128996060.794774) * cj1)) + ((x589 * x590)) + ((x587 * x593)))) + IKsqr(((-1.03318584070796) + (((-1.0) * x598 * x599)) + (((-1.0) * x587 * x590)) + (((-0.251106194690265) * r21 * x594)) + (((-1.0) * x588 * x589)) + ((cj1 * x591)) + (((-1.0) * x594 * x597)) + (((-1.0) * x595 * x598)) + (((128996060.794774) * sj1)) + ((sj1 * x592)) + ((x589 * x593)) + ((x587 * x596)))) - 1) <= IKFAST_SINCOS_THRESH)
                                                            continue;
                                                        j2array[0] = IKatan2((((x594 * x595)) + ((x594 * x599)) + (((-1.0) * x597 * x598)) + (((-1.0) * x589 * x596)) + (((-0.251106194690265) * r21 * x598)) + (((-1.0) * x587 * x588)) + (((-1.0) * cj1 * x592)) + ((sj1 * x591)) + (((-128996060.794774) * cj1)) + ((x589 * x590)) + ((x587 * x593))), ((-1.03318584070796) + (((-1.0) * x598 * x599)) + (((-1.0) * x587 * x590)) + (((-0.251106194690265) * r21 * x594)) + (((-1.0) * x588 * x589)) + ((cj1 * x591)) + (((-1.0) * x594 * x597)) + (((-1.0) * x595 * x598)) + (((128996060.794774) * sj1)) + ((sj1 * x592)) + ((x589 * x593)) + ((x587 * x596))));
                                                        sj2array[0] = IKsin(j2array[0]);
                                                        cj2array[0] = IKcos(j2array[0]);
                                                        if (j2array[0] > IKPI)
                                                        {
                                                            j2array[0] -= IK2PI;
                                                        }
                                                        else if (j2array[0] < -IKPI)
                                                        {
                                                            j2array[0] += IK2PI;
                                                        }
                                                        j2valid[0] = true;
                                                        for (int ij2 = 0; ij2 < 1; ++ij2)
                                                        {
                                                            if (!j2valid[ij2])
                                                            {
                                                                continue;
                                                            }
                                                            _ij2[0] = ij2;
                                                            _ij2[1] = -1;
                                                            for (int iij2 = ij2 + 1; iij2 < 1; ++iij2)
                                                            {
                                                                if (j2valid[iij2] && IKabs(cj2array[ij2] - cj2array[iij2]) < IKFAST_SOLUTION_THRESH && IKabs(sj2array[ij2] - sj2array[iij2]) < IKFAST_SOLUTION_THRESH)
                                                                {
                                                                    j2valid[iij2] = false;
                                                                    _ij2[1] = iij2;
                                                                    break;
                                                                }
                                                            }
                                                            j2 = j2array[ij2];
                                                            cj2 = cj2array[ij2];
                                                            sj2 = sj2array[ij2];
                                                            {
                                                                IkReal evalcond[3];
                                                                IkReal x601 = IKcos(j2);
                                                                IkReal x602 = IKsin(j2);
                                                                IkReal x603 = ((3.7e-7) * cj5);
                                                                IkReal x604 = ((0.1135) * sj5);
                                                                IkReal x605 = ((3.7e-7) * sj5);
                                                                IkReal x606 = ((0.1135) * cj5);
                                                                IkReal x607 = (cj1 * x602);
                                                                IkReal x608 = (sj1 * x601);
                                                                evalcond[0] = ((((-1.0) * r20 * x606)) + (((0.452) * sj1 * x602)) + (((0.452) * cj1 * x601)) + ((r20 * x605)) + (((0.467) * cj1)) + ((r21 * x604)) + ((r21 * x603)) + (((-1.0) * pz)));
                                                                evalcond[1] = ((-0.11339999909077) + ((r00 * x605)) + (((-1.0) * r00 * x606)) + (((8.79096604904732e-10) * x608)) + (((-1.0) * px)) + (((-8.79096604904732e-10) * x607)) + ((r01 * x604)) + ((r01 * x603)) + (((9.08270164802013e-10) * sj1)));
                                                                evalcond[2] = ((1.21543325961255e-7) + (((-1.0) * r10 * x606)) + (((0.452000001519335) * x608)) + (((-0.452000001519335) * x607)) + (((-1.0) * py)) + ((r11 * x603)) + ((r11 * x604)) + (((0.467000001569756) * sj1)) + ((r10 * x605)));
                                                                if (IKabs(evalcond[0]) > IKFAST_EVALCOND_THRESH || IKabs(evalcond[1]) > IKFAST_EVALCOND_THRESH || IKabs(evalcond[2]) > IKFAST_EVALCOND_THRESH)
                                                                {
                                                                    continue;
                                                                }
                                                            }

                                                            {
                                                                IkReal j3eval[2];
                                                                sj0 = -0.19503986;
                                                                cj0 = -0.98079532;
                                                                j0 = -2.94529456;
                                                                IkReal x609 = (((cj5 * r11 * sj4)) + ((r10 * sj4 * sj5)) + (((-1.0) * cj4 * r12)));
                                                                j3eval[0] = x609;
                                                                j3eval[1] = IKsign(x609);
                                                                if (IKabs(j3eval[0]) < 0.0000010000000000 || IKabs(j3eval[1]) < 0.0000010000000000)
                                                                {
                                                                    {
                                                                        IkReal j3eval[2];
                                                                        sj0 = -0.19503986;
                                                                        cj0 = -0.98079532;
                                                                        j0 = -2.94529456;
                                                                        IkReal x610 = ((1.0) * sj4);
                                                                        IkReal x611 = ((((-1.0) * cj5 * r01 * x610)) + (((-1.0) * r00 * sj5 * x610)) + ((cj4 * r02)));
                                                                        j3eval[0] = x611;
                                                                        j3eval[1] = IKsign(x611);
                                                                        if (IKabs(j3eval[0]) < 0.0000010000000000 || IKabs(j3eval[1]) < 0.0000010000000000)
                                                                        {
                                                                            {
                                                                                IkReal j3eval[2];
                                                                                sj0 = -0.19503986;
                                                                                cj0 = -0.98079532;
                                                                                j0 = -2.94529456;
                                                                                IkReal x612 = cj4 * cj4;
                                                                                IkReal x613 = cj5 * cj5;
                                                                                IkReal x614 = r22 * r22;
                                                                                IkReal x615 = r21 * r21;
                                                                                IkReal x616 = r20 * r20;
                                                                                IkReal x617 = (r20 * sj5);
                                                                                IkReal x618 = (cj5 * r21);
                                                                                IkReal x619 = ((1.0) * x615);
                                                                                IkReal x620 = ((1.0) * x616);
                                                                                IkReal x621 = (x612 * x613);
                                                                                IkReal x622 = ((2.0) * cj4 * r22 * sj4);
                                                                                IkReal x623 = (((x616 * x621)) + (((-2.0) * x612 * x617 * x618)) + ((x612 * x614)) + (((-1.0) * x614)) + (((-1.0) * x617 * x622)) + ((x613 * x615)) + (((-1.0) * x619)) + (((-1.0) * x612 * x620)) + (((-1.0) * x619 * x621)) + (((-1.0) * x618 * x622)) + (((2.0) * x617 * x618)) + (((-1.0) * x613 * x620)));
                                                                                j3eval[0] = x623;
                                                                                j3eval[1] = IKsign(x623);
                                                                                if (IKabs(j3eval[0]) < 0.0000010000000000 || IKabs(j3eval[1]) < 0.0000010000000000)
                                                                                {
                                                                                    continue; // no branches [j3]
                                                                                }
                                                                                else
                                                                                {
                                                                                    {
                                                                                        IkReal j3array[1], cj3array[1], sj3array[1];
                                                                                        bool j3valid[1] = {false};
                                                                                        _nj3 = 1;
                                                                                        IkReal x624 = cj4 * cj4;
                                                                                        IkReal x625 = cj5 * cj5;
                                                                                        IkReal x626 = r22 * r22;
                                                                                        IkReal x627 = r21 * r21;
                                                                                        IkReal x628 = r20 * r20;
                                                                                        IkReal x629 = ((1.0) * sj1);
                                                                                        IkReal x630 = (r22 * sj4);
                                                                                        IkReal x631 = (sj2 * sj5);
                                                                                        IkReal x632 = (cj4 * r20);
                                                                                        IkReal x633 = (cj5 * sj2);
                                                                                        IkReal x634 = (r21 * sj5);
                                                                                        IkReal x635 = (cj4 * r21);
                                                                                        IkReal x636 = ((2.0) * cj5);
                                                                                        IkReal x637 = (cj2 * cj5 * r20);
                                                                                        IkReal x638 = ((1.0) * x627);
                                                                                        IkReal x639 = ((1.0) * cj1 * cj2);
                                                                                        IkReal x640 = ((1.0) * x628);
                                                                                        IkReal x641 = (x624 * x625);
                                                                                        CheckValue<IkReal> x642 = IKPowWithIntegerCheck(IKsign((((x628 * x641)) + (((-1.0) * x625 * x640)) + ((x625 * x627)) + (((-1.0) * x638)) + (((-2.0) * sj5 * x630 * x632)) + (((-1.0) * r20 * x624 * x634 * x636)) + (((-1.0) * x624 * x640)) + (((-1.0) * x630 * x635 * x636)) + ((x624 * x626)) + (((-1.0) * x626)) + ((r20 * x634 * x636)) + (((-1.0) * x638 * x641)))), -1);
                                                                                        if (!x642.valid)
                                                                                        {
                                                                                            continue;
                                                                                        }
                                                                                        CheckValue<IkReal> x643 = IKatan2WithCheck(IkReal(((((-1.0) * x629 * x637)) + (((-1.0) * x630 * x639)) + (((-1.0) * x629 * x631 * x632)) + ((cj2 * sj1 * x634)) + (((-1.0) * cj5 * x635 * x639)) + (((-1.0) * cj1 * r21 * x631)) + (((-1.0) * sj5 * x632 * x639)) + (((-1.0) * sj2 * x629 * x630)) + ((cj1 * r20 * x633)) + (((-1.0) * x629 * x633 * x635)))), IkReal((((cj1 * x631 * x632)) + ((cj1 * x633 * x635)) + (((-1.0) * cj2 * sj5 * x629 * x632)) + (((-1.0) * x634 * x639)) + (((-1.0) * cj2 * cj5 * x629 * x635)) + (((-1.0) * cj2 * x629 * x630)) + ((cj1 * sj2 * x630)) + ((r20 * sj1 * x633)) + (((-1.0) * r21 * x629 * x631)) + ((cj1 * x637)))), IKFAST_ATAN2_MAGTHRESH);
                                                                                        if (!x643.valid)
                                                                                        {
                                                                                            continue;
                                                                                        }
                                                                                        j3array[0] = ((-1.5707963267949) + (((1.5707963267949) * (x642.value))) + (x643.value));
                                                                                        sj3array[0] = IKsin(j3array[0]);
                                                                                        cj3array[0] = IKcos(j3array[0]);
                                                                                        if (j3array[0] > IKPI)
                                                                                        {
                                                                                            j3array[0] -= IK2PI;
                                                                                        }
                                                                                        else if (j3array[0] < -IKPI)
                                                                                        {
                                                                                            j3array[0] += IK2PI;
                                                                                        }
                                                                                        j3valid[0] = true;
                                                                                        for (int ij3 = 0; ij3 < 1; ++ij3)
                                                                                        {
                                                                                            if (!j3valid[ij3])
                                                                                            {
                                                                                                continue;
                                                                                            }
                                                                                            _ij3[0] = ij3;
                                                                                            _ij3[1] = -1;
                                                                                            for (int iij3 = ij3 + 1; iij3 < 1; ++iij3)
                                                                                            {
                                                                                                if (j3valid[iij3] && IKabs(cj3array[ij3] - cj3array[iij3]) < IKFAST_SOLUTION_THRESH && IKabs(sj3array[ij3] - sj3array[iij3]) < IKFAST_SOLUTION_THRESH)
                                                                                                {
                                                                                                    j3valid[iij3] = false;
                                                                                                    _ij3[1] = iij3;
                                                                                                    break;
                                                                                                }
                                                                                            }
                                                                                            j3 = j3array[ij3];
                                                                                            cj3 = cj3array[ij3];
                                                                                            sj3 = sj3array[ij3];
                                                                                            {
                                                                                                IkReal evalcond[6];
                                                                                                IkReal x644 = IKcos(j3);
                                                                                                IkReal x645 = IKsin(j3);
                                                                                                IkReal x646 = ((1.0) * r11);
                                                                                                IkReal x647 = ((1.94490399315206e-9) * sj2);
                                                                                                IkReal x648 = (sj1 * sj2);
                                                                                                IkReal x649 = (cj2 * sj1);
                                                                                                IkReal x650 = (cj4 * r01);
                                                                                                IkReal x651 = (cj1 * sj2);
                                                                                                IkReal x652 = (cj1 * cj2);
                                                                                                IkReal x653 = ((1.0) * r21);
                                                                                                IkReal x654 = (cj5 * x644);
                                                                                                IkReal x655 = (sj4 * x644);
                                                                                                IkReal x656 = (cj5 * x645);
                                                                                                IkReal x657 = (sj5 * x644);
                                                                                                IkReal x658 = ((1.0) * x645);
                                                                                                IkReal x659 = (cj4 * sj5 * x658);
                                                                                                evalcond[0] = ((((-1.0) * x649)) + ((r22 * x655)) + ((cj4 * r20 * x657)) + (((-1.0) * sj5 * x645 * x653)) + ((cj4 * r21 * x654)) + x651 + ((r20 * x656)));
                                                                                                evalcond[1] = ((((-1.0) * cj4 * x653 * x656)) + x648 + x652 + (((-1.0) * r20 * x659)) + (((-1.0) * r22 * sj4 * x658)) + (((-1.0) * x653 * x657)) + ((r20 * x654)));
                                                                                                evalcond[2] = (((cj4 * r00 * x657)) + ((sj1 * x647)) + ((r02 * x655)) + (((-1.0) * r01 * sj5 * x658)) + (((1.94490399315206e-9) * x652)) + ((x650 * x654)) + ((r00 * x656)));
                                                                                                evalcond[3] = (((cj4 * r11 * x654)) + (((1.00000000336136) * x652)) + ((r10 * x656)) + (((1.00000000336136) * x648)) + (((-1.0) * sj5 * x645 * x646)) + ((cj4 * r10 * x657)) + ((r12 * x655)));
                                                                                                evalcond[4] = ((((-1.0) * cj1 * x647)) + (((-1.0) * x650 * x656)) + (((-1.0) * r00 * x659)) + (((-1.0) * r01 * x657)) + (((-1.0) * r02 * sj4 * x658)) + ((r00 * x654)) + (((1.94490399315206e-9) * x649)));
                                                                                                evalcond[5] = ((((-1.0) * r12 * sj4 * x658)) + (((-1.0) * x646 * x657)) + (((-1.00000000336136) * x651)) + (((-1.0) * r10 * x659)) + (((-1.0) * cj4 * x646 * x656)) + ((r10 * x654)) + (((1.00000000336136) * x649)));
                                                                                                if (IKabs(evalcond[0]) > IKFAST_EVALCOND_THRESH || IKabs(evalcond[1]) > IKFAST_EVALCOND_THRESH || IKabs(evalcond[2]) > IKFAST_EVALCOND_THRESH || IKabs(evalcond[3]) > IKFAST_EVALCOND_THRESH || IKabs(evalcond[4]) > IKFAST_EVALCOND_THRESH || IKabs(evalcond[5]) > IKFAST_EVALCOND_THRESH)
                                                                                                {
                                                                                                    continue;
                                                                                                }
                                                                                            }

                                                                                            {
                                                                                                std::vector<IkSingleDOFSolutionBase<IkReal>> vinfos(6);
                                                                                                vinfos[0].jointtype = 1;
                                                                                                vinfos[0].foffset = j0;
                                                                                                vinfos[0].indices[0] = _ij0[0];
                                                                                                vinfos[0].indices[1] = _ij0[1];
                                                                                                vinfos[0].maxsolutions = _nj0;
                                                                                                vinfos[1].jointtype = 1;
                                                                                                vinfos[1].foffset = j1;
                                                                                                vinfos[1].indices[0] = _ij1[0];
                                                                                                vinfos[1].indices[1] = _ij1[1];
                                                                                                vinfos[1].maxsolutions = _nj1;
                                                                                                vinfos[2].jointtype = 1;
                                                                                                vinfos[2].foffset = j2;
                                                                                                vinfos[2].indices[0] = _ij2[0];
                                                                                                vinfos[2].indices[1] = _ij2[1];
                                                                                                vinfos[2].maxsolutions = _nj2;
                                                                                                vinfos[3].jointtype = 1;
                                                                                                vinfos[3].foffset = j3;
                                                                                                vinfos[3].indices[0] = _ij3[0];
                                                                                                vinfos[3].indices[1] = _ij3[1];
                                                                                                vinfos[3].maxsolutions = _nj3;
                                                                                                vinfos[4].jointtype = 1;
                                                                                                vinfos[4].foffset = j4;
                                                                                                vinfos[4].indices[0] = _ij4[0];
                                                                                                vinfos[4].indices[1] = _ij4[1];
                                                                                                vinfos[4].maxsolutions = _nj4;
                                                                                                vinfos[5].jointtype = 1;
                                                                                                vinfos[5].foffset = j5;
                                                                                                vinfos[5].indices[0] = _ij5[0];
                                                                                                vinfos[5].indices[1] = _ij5[1];
                                                                                                vinfos[5].maxsolutions = _nj5;
                                                                                                std::vector<int> vfree(0);
                                                                                                solutions.AddSolution(vinfos, vfree);
                                                                                            }
                                                                                        }
                                                                                    }
                                                                                }
                                                                            }
                                                                        }
                                                                        else
                                                                        {
                                                                            {
                                                                                IkReal j3array[1], cj3array[1], sj3array[1];
                                                                                bool j3valid[1] = {false};
                                                                                _nj3 = 1;
                                                                                IkReal x660 = (cj1 * cj5);
                                                                                IkReal x661 = ((1.00000000336136) * r20);
                                                                                IkReal x662 = ((1.0) * sj4);
                                                                                IkReal x663 = (cj4 * sj2);
                                                                                IkReal x664 = (sj1 * sj5);
                                                                                IkReal x665 = ((1.00000000336136) * r21);
                                                                                IkReal x666 = (cj5 * sj1);
                                                                                IkReal x667 = (sj2 * sj4);
                                                                                IkReal x668 = (cj2 * r10);
                                                                                IkReal x669 = ((1.0) * cj4);
                                                                                IkReal x670 = ((1.00000000336136) * r22);
                                                                                IkReal x671 = (cj1 * cj2);
                                                                                IkReal x672 = (cj1 * sj5);
                                                                                IkReal x673 = (cj2 * r11);
                                                                                CheckValue<IkReal> x674 = IKPowWithIntegerCheck(IKsign(((((-1.0) * cj5 * r01 * x662)) + (((-1.0) * r00 * sj5 * x662)) + ((cj4 * r02)))), -1);
                                                                                if (!x674.valid)
                                                                                {
                                                                                    continue;
                                                                                }
                                                                                CheckValue<IkReal> x675 = IKatan2WithCheck(IkReal(((((-1.0) * x661 * x663 * x664)) + (((-1.0) * cj4 * sj5 * x661 * x671)) + (((-1.0) * sj4 * x670 * x671)) + (((-1.0) * sj1 * x667 * x670)) + ((r10 * x663 * x672)) + (((-1.0) * x664 * x668 * x669)) + ((cj1 * r12 * x667)) + (((-1.0) * x666 * x669 * x673)) + (((-1.0) * cj2 * r12 * sj1 * x662)) + ((r11 * x660 * x663)) + (((-1.0) * cj2 * cj4 * x660 * x665)) + (((-1.0) * x663 * x665 * x666)))), IkReal((((x666 * x668)) + ((sj2 * x661 * x666)) + ((cj2 * x660 * x661)) + (((-1.0) * sj2 * x664 * x665)) + (((-1.0) * x664 * x673)) + (((-1.0) * r10 * sj2 * x660)) + (((-1.0) * sj5 * x665 * x671)) + ((r11 * sj2 * x672)))), IKFAST_ATAN2_MAGTHRESH);
                                                                                if (!x675.valid)
                                                                                {
                                                                                    continue;
                                                                                }
                                                                                j3array[0] = ((-1.5707963267949) + (((1.5707963267949) * (x674.value))) + (x675.value));
                                                                                sj3array[0] = IKsin(j3array[0]);
                                                                                cj3array[0] = IKcos(j3array[0]);
                                                                                if (j3array[0] > IKPI)
                                                                                {
                                                                                    j3array[0] -= IK2PI;
                                                                                }
                                                                                else if (j3array[0] < -IKPI)
                                                                                {
                                                                                    j3array[0] += IK2PI;
                                                                                }
                                                                                j3valid[0] = true;
                                                                                for (int ij3 = 0; ij3 < 1; ++ij3)
                                                                                {
                                                                                    if (!j3valid[ij3])
                                                                                    {
                                                                                        continue;
                                                                                    }
                                                                                    _ij3[0] = ij3;
                                                                                    _ij3[1] = -1;
                                                                                    for (int iij3 = ij3 + 1; iij3 < 1; ++iij3)
                                                                                    {
                                                                                        if (j3valid[iij3] && IKabs(cj3array[ij3] - cj3array[iij3]) < IKFAST_SOLUTION_THRESH && IKabs(sj3array[ij3] - sj3array[iij3]) < IKFAST_SOLUTION_THRESH)
                                                                                        {
                                                                                            j3valid[iij3] = false;
                                                                                            _ij3[1] = iij3;
                                                                                            break;
                                                                                        }
                                                                                    }
                                                                                    j3 = j3array[ij3];
                                                                                    cj3 = cj3array[ij3];
                                                                                    sj3 = sj3array[ij3];
                                                                                    {
                                                                                        IkReal evalcond[6];
                                                                                        IkReal x676 = IKcos(j3);
                                                                                        IkReal x677 = IKsin(j3);
                                                                                        IkReal x678 = ((1.0) * r11);
                                                                                        IkReal x679 = ((1.94490399315206e-9) * sj2);
                                                                                        IkReal x680 = (sj1 * sj2);
                                                                                        IkReal x681 = (cj2 * sj1);
                                                                                        IkReal x682 = (cj4 * r01);
                                                                                        IkReal x683 = (cj1 * sj2);
                                                                                        IkReal x684 = (cj1 * cj2);
                                                                                        IkReal x685 = ((1.0) * r21);
                                                                                        IkReal x686 = (cj5 * x676);
                                                                                        IkReal x687 = (sj4 * x676);
                                                                                        IkReal x688 = (cj5 * x677);
                                                                                        IkReal x689 = (sj5 * x676);
                                                                                        IkReal x690 = ((1.0) * x677);
                                                                                        IkReal x691 = (cj4 * sj5 * x690);
                                                                                        evalcond[0] = (((cj4 * r21 * x686)) + ((r22 * x687)) + x683 + (((-1.0) * sj5 * x677 * x685)) + ((r20 * x688)) + (((-1.0) * x681)) + ((cj4 * r20 * x689)));
                                                                                        evalcond[1] = ((((-1.0) * x685 * x689)) + x680 + x684 + (((-1.0) * cj4 * x685 * x688)) + ((r20 * x686)) + (((-1.0) * r20 * x691)) + (((-1.0) * r22 * sj4 * x690)));
                                                                                        evalcond[2] = ((((-1.0) * r01 * sj5 * x690)) + (((1.94490399315206e-9) * x684)) + ((x682 * x686)) + ((cj4 * r00 * x689)) + ((r00 * x688)) + ((sj1 * x679)) + ((r02 * x687)));
                                                                                        evalcond[3] = ((((1.00000000336136) * x684)) + (((1.00000000336136) * x680)) + (((-1.0) * sj5 * x677 * x678)) + ((r10 * x688)) + ((cj4 * r10 * x689)) + ((cj4 * r11 * x686)) + ((r12 * x687)));
                                                                                        evalcond[4] = ((((-1.0) * r00 * x691)) + (((1.94490399315206e-9) * x681)) + (((-1.0) * cj1 * x679)) + (((-1.0) * r01 * x689)) + (((-1.0) * r02 * sj4 * x690)) + ((r00 * x686)) + (((-1.0) * x682 * x688)));
                                                                                        evalcond[5] = ((((1.00000000336136) * x681)) + (((-1.0) * x678 * x689)) + (((-1.0) * r12 * sj4 * x690)) + ((r10 * x686)) + (((-1.0) * r10 * x691)) + (((-1.0) * cj4 * x678 * x688)) + (((-1.00000000336136) * x683)));
                                                                                        if (IKabs(evalcond[0]) > IKFAST_EVALCOND_THRESH || IKabs(evalcond[1]) > IKFAST_EVALCOND_THRESH || IKabs(evalcond[2]) > IKFAST_EVALCOND_THRESH || IKabs(evalcond[3]) > IKFAST_EVALCOND_THRESH || IKabs(evalcond[4]) > IKFAST_EVALCOND_THRESH || IKabs(evalcond[5]) > IKFAST_EVALCOND_THRESH)
                                                                                        {
                                                                                            continue;
                                                                                        }
                                                                                    }

                                                                                    {
                                                                                        std::vector<IkSingleDOFSolutionBase<IkReal>> vinfos(6);
                                                                                        vinfos[0].jointtype = 1;
                                                                                        vinfos[0].foffset = j0;
                                                                                        vinfos[0].indices[0] = _ij0[0];
                                                                                        vinfos[0].indices[1] = _ij0[1];
                                                                                        vinfos[0].maxsolutions = _nj0;
                                                                                        vinfos[1].jointtype = 1;
                                                                                        vinfos[1].foffset = j1;
                                                                                        vinfos[1].indices[0] = _ij1[0];
                                                                                        vinfos[1].indices[1] = _ij1[1];
                                                                                        vinfos[1].maxsolutions = _nj1;
                                                                                        vinfos[2].jointtype = 1;
                                                                                        vinfos[2].foffset = j2;
                                                                                        vinfos[2].indices[0] = _ij2[0];
                                                                                        vinfos[2].indices[1] = _ij2[1];
                                                                                        vinfos[2].maxsolutions = _nj2;
                                                                                        vinfos[3].jointtype = 1;
                                                                                        vinfos[3].foffset = j3;
                                                                                        vinfos[3].indices[0] = _ij3[0];
                                                                                        vinfos[3].indices[1] = _ij3[1];
                                                                                        vinfos[3].maxsolutions = _nj3;
                                                                                        vinfos[4].jointtype = 1;
                                                                                        vinfos[4].foffset = j4;
                                                                                        vinfos[4].indices[0] = _ij4[0];
                                                                                        vinfos[4].indices[1] = _ij4[1];
                                                                                        vinfos[4].maxsolutions = _nj4;
                                                                                        vinfos[5].jointtype = 1;
                                                                                        vinfos[5].foffset = j5;
                                                                                        vinfos[5].indices[0] = _ij5[0];
                                                                                        vinfos[5].indices[1] = _ij5[1];
                                                                                        vinfos[5].maxsolutions = _nj5;
                                                                                        std::vector<int> vfree(0);
                                                                                        solutions.AddSolution(vinfos, vfree);
                                                                                    }
                                                                                }
                                                                            }
                                                                        }
                                                                    }
                                                                }
                                                                else
                                                                {
                                                                    {
                                                                        IkReal j3array[1], cj3array[1], sj3array[1];
                                                                        bool j3valid[1] = {false};
                                                                        _nj3 = 1;
                                                                        IkReal x692 = (sj1 * sj5);
                                                                        IkReal x693 = ((1.0) * r01);
                                                                        IkReal x694 = (cj1 * sj2);
                                                                        IkReal x695 = (cj2 * sj1);
                                                                        IkReal x696 = (r02 * sj4);
                                                                        IkReal x697 = (cj1 * cj2);
                                                                        IkReal x698 = ((1.94490399315206e-9) * r20);
                                                                        IkReal x699 = (sj1 * sj2);
                                                                        IkReal x700 = (cj4 * r00);
                                                                        IkReal x701 = ((1.94490399315206e-9) * r21);
                                                                        IkReal x702 = (cj5 * r00);
                                                                        IkReal x703 = ((1.94490399315206e-9) * r22 * sj4);
                                                                        IkReal x704 = (cj4 * cj5 * r01);
                                                                        IkReal x705 = (cj4 * cj5 * x701);
                                                                        CheckValue<IkReal> x706 = IKatan2WithCheck(IkReal((((cj5 * x694 * x698)) + (((-1.0) * sj5 * x694 * x701)) + (((-1.0) * cj5 * x695 * x698)) + ((x697 * x702)) + ((cj2 * x692 * x701)) + (((-1.0) * sj2 * x692 * x693)) + (((-1.0) * sj5 * x693 * x697)) + ((x699 * x702)))), IkReal((((x696 * x697)) + ((x696 * x699)) + (((-1.0) * x695 * x705)) + (((-1.0) * x695 * x703)) + ((x694 * x705)) + ((x694 * x703)) + ((sj5 * x697 * x700)) + ((x697 * x704)) + (((-1.0) * cj2 * cj4 * x692 * x698)) + ((cj4 * sj5 * x694 * x698)) + ((sj2 * x692 * x700)) + ((x699 * x704)))), IKFAST_ATAN2_MAGTHRESH);
                                                                        if (!x706.valid)
                                                                        {
                                                                            continue;
                                                                        }
                                                                        CheckValue<IkReal> x707 = IKPowWithIntegerCheck(IKsign((((cj5 * r11 * sj4)) + ((r10 * sj4 * sj5)) + (((-1.0) * cj4 * r12)))), -1);
                                                                        if (!x707.valid)
                                                                        {
                                                                            continue;
                                                                        }
                                                                        j3array[0] = ((-1.5707963267949) + (x706.value) + (((1.5707963267949) * (x707.value))));
                                                                        sj3array[0] = IKsin(j3array[0]);
                                                                        cj3array[0] = IKcos(j3array[0]);
                                                                        if (j3array[0] > IKPI)
                                                                        {
                                                                            j3array[0] -= IK2PI;
                                                                        }
                                                                        else if (j3array[0] < -IKPI)
                                                                        {
                                                                            j3array[0] += IK2PI;
                                                                        }
                                                                        j3valid[0] = true;
                                                                        for (int ij3 = 0; ij3 < 1; ++ij3)
                                                                        {
                                                                            if (!j3valid[ij3])
                                                                            {
                                                                                continue;
                                                                            }
                                                                            _ij3[0] = ij3;
                                                                            _ij3[1] = -1;
                                                                            for (int iij3 = ij3 + 1; iij3 < 1; ++iij3)
                                                                            {
                                                                                if (j3valid[iij3] && IKabs(cj3array[ij3] - cj3array[iij3]) < IKFAST_SOLUTION_THRESH && IKabs(sj3array[ij3] - sj3array[iij3]) < IKFAST_SOLUTION_THRESH)
                                                                                {
                                                                                    j3valid[iij3] = false;
                                                                                    _ij3[1] = iij3;
                                                                                    break;
                                                                                }
                                                                            }
                                                                            j3 = j3array[ij3];
                                                                            cj3 = cj3array[ij3];
                                                                            sj3 = sj3array[ij3];
                                                                            {
                                                                                IkReal evalcond[6];
                                                                                IkReal x708 = IKcos(j3);
                                                                                IkReal x709 = IKsin(j3);
                                                                                IkReal x710 = ((1.0) * r11);
                                                                                IkReal x711 = ((1.94490399315206e-9) * sj2);
                                                                                IkReal x712 = (sj1 * sj2);
                                                                                IkReal x713 = (cj2 * sj1);
                                                                                IkReal x714 = (cj4 * r01);
                                                                                IkReal x715 = (cj1 * sj2);
                                                                                IkReal x716 = (cj1 * cj2);
                                                                                IkReal x717 = ((1.0) * r21);
                                                                                IkReal x718 = (cj5 * x708);
                                                                                IkReal x719 = (sj4 * x708);
                                                                                IkReal x720 = (cj5 * x709);
                                                                                IkReal x721 = (sj5 * x708);
                                                                                IkReal x722 = ((1.0) * x709);
                                                                                IkReal x723 = (cj4 * sj5 * x722);
                                                                                evalcond[0] = (((r22 * x719)) + ((r20 * x720)) + (((-1.0) * x713)) + (((-1.0) * sj5 * x709 * x717)) + x715 + ((cj4 * r20 * x721)) + ((cj4 * r21 * x718)));
                                                                                evalcond[1] = ((((-1.0) * x717 * x721)) + ((r20 * x718)) + x712 + x716 + (((-1.0) * r20 * x723)) + (((-1.0) * cj4 * x717 * x720)) + (((-1.0) * r22 * sj4 * x722)));
                                                                                evalcond[2] = (((x714 * x718)) + (((1.94490399315206e-9) * x716)) + ((r00 * x720)) + ((sj1 * x711)) + ((r02 * x719)) + (((-1.0) * r01 * sj5 * x722)) + ((cj4 * r00 * x721)));
                                                                                evalcond[3] = (((r10 * x720)) + ((cj4 * r10 * x721)) + ((cj4 * r11 * x718)) + (((-1.0) * sj5 * x709 * x710)) + (((1.00000000336136) * x712)) + (((1.00000000336136) * x716)) + ((r12 * x719)));
                                                                                evalcond[4] = ((((-1.0) * r00 * x723)) + (((1.94490399315206e-9) * x713)) + ((r00 * x718)) + (((-1.0) * r01 * x721)) + (((-1.0) * cj1 * x711)) + (((-1.0) * r02 * sj4 * x722)) + (((-1.0) * x714 * x720)));
                                                                                evalcond[5] = ((((-1.0) * r12 * sj4 * x722)) + (((-1.0) * x710 * x721)) + (((-1.00000000336136) * x715)) + (((-1.0) * cj4 * x710 * x720)) + (((1.00000000336136) * x713)) + (((-1.0) * r10 * x723)) + ((r10 * x718)));
                                                                                if (IKabs(evalcond[0]) > IKFAST_EVALCOND_THRESH || IKabs(evalcond[1]) > IKFAST_EVALCOND_THRESH || IKabs(evalcond[2]) > IKFAST_EVALCOND_THRESH || IKabs(evalcond[3]) > IKFAST_EVALCOND_THRESH || IKabs(evalcond[4]) > IKFAST_EVALCOND_THRESH || IKabs(evalcond[5]) > IKFAST_EVALCOND_THRESH)
                                                                                {
                                                                                    continue;
                                                                                }
                                                                            }

                                                                            {
                                                                                std::vector<IkSingleDOFSolutionBase<IkReal>> vinfos(6);
                                                                                vinfos[0].jointtype = 1;
                                                                                vinfos[0].foffset = j0;
                                                                                vinfos[0].indices[0] = _ij0[0];
                                                                                vinfos[0].indices[1] = _ij0[1];
                                                                                vinfos[0].maxsolutions = _nj0;
                                                                                vinfos[1].jointtype = 1;
                                                                                vinfos[1].foffset = j1;
                                                                                vinfos[1].indices[0] = _ij1[0];
                                                                                vinfos[1].indices[1] = _ij1[1];
                                                                                vinfos[1].maxsolutions = _nj1;
                                                                                vinfos[2].jointtype = 1;
                                                                                vinfos[2].foffset = j2;
                                                                                vinfos[2].indices[0] = _ij2[0];
                                                                                vinfos[2].indices[1] = _ij2[1];
                                                                                vinfos[2].maxsolutions = _nj2;
                                                                                vinfos[3].jointtype = 1;
                                                                                vinfos[3].foffset = j3;
                                                                                vinfos[3].indices[0] = _ij3[0];
                                                                                vinfos[3].indices[1] = _ij3[1];
                                                                                vinfos[3].maxsolutions = _nj3;
                                                                                vinfos[4].jointtype = 1;
                                                                                vinfos[4].foffset = j4;
                                                                                vinfos[4].indices[0] = _ij4[0];
                                                                                vinfos[4].indices[1] = _ij4[1];
                                                                                vinfos[4].maxsolutions = _nj4;
                                                                                vinfos[5].jointtype = 1;
                                                                                vinfos[5].foffset = j5;
                                                                                vinfos[5].indices[0] = _ij5[0];
                                                                                vinfos[5].indices[1] = _ij5[1];
                                                                                vinfos[5].maxsolutions = _nj5;
                                                                                std::vector<int> vfree(0);
                                                                                solutions.AddSolution(vinfos, vfree);
                                                                            }
                                                                        }
                                                                    }
                                                                }
                                                            }
                                                        }
                                                    }
                                                }
                                            } while (0);
                                            if (bgotonextstatement)
                                            {
                                                bool bgotonextstatement = true;
                                                do
                                                {
                                                    evalcond[0] = ((-3.14159265358979) + (IKfmod(((3.14159265358979) + (IKabs(((-0.196298091763612) + j0)))), 6.28318530717959)));
                                                    if (IKabs(evalcond[0]) < 0.0000050000000000)
                                                    {
                                                        bgotonextstatement = false;
                                                        {
                                                            IkReal j2array[1], cj2array[1], sj2array[1];
                                                            bool j2valid[1] = {false};
                                                            _nj2 = 1;
                                                            IkReal x724 = (cj5 * sj1);
                                                            IkReal x725 = ((8.1858407079646e-7) * r21);
                                                            IkReal x726 = (cj1 * cj5);
                                                            IkReal x727 = ((420.886621488087) * r01);
                                                            IkReal x728 = ((2.21238938053097) * pz);
                                                            IkReal x729 = ((1137531409.42726) * px);
                                                            IkReal x730 = ((0.251106194690265) * r20);
                                                            IkReal x731 = (cj1 * sj5);
                                                            IkReal x732 = ((129109814.969994) * r01);
                                                            IkReal x733 = ((129109814.969994) * r00);
                                                            IkReal x734 = ((8.1858407079646e-7) * r20);
                                                            IkReal x735 = (sj1 * sj5);
                                                            IkReal x736 = ((420.886621488087) * r00);
                                                            IkReal x737 = ((0.251106194690265) * r21 * sj5);
                                                            if (IKabs((((cj1 * x729)) + (((-1.0) * x724 * x725)) + (((-1.0) * x731 * x732)) + (((-1.0) * x731 * x736)) + ((x726 * x733)) + (((-1.0) * x734 * x735)) + (((-0.251106194690265) * r21 * x735)) + ((x724 * x730)) + (((-128996060.794774) * cj1)) + (((-1.0) * x726 * x727)) + ((sj1 * x728)))) < IKFAST_ATAN2_MAGTHRESH && IKabs(((-1.03318584070796) + ((cj1 * x728)) + (((-1.0) * x731 * x734)) + ((x735 * x736)) + (((-1.0) * x724 * x733)) + ((x726 * x730)) + (((-1.0) * x725 * x726)) + (((-1.0) * sj1 * x729)) + (((128996060.794774) * sj1)) + (((-0.251106194690265) * r21 * x731)) + ((x724 * x727)) + ((x732 * x735)))) < IKFAST_ATAN2_MAGTHRESH && IKabs(IKsqr((((cj1 * x729)) + (((-1.0) * x724 * x725)) + (((-1.0) * x731 * x732)) + (((-1.0) * x731 * x736)) + ((x726 * x733)) + (((-1.0) * x734 * x735)) + (((-0.251106194690265) * r21 * x735)) + ((x724 * x730)) + (((-128996060.794774) * cj1)) + (((-1.0) * x726 * x727)) + ((sj1 * x728)))) + IKsqr(((-1.03318584070796) + ((cj1 * x728)) + (((-1.0) * x731 * x734)) + ((x735 * x736)) + (((-1.0) * x724 * x733)) + ((x726 * x730)) + (((-1.0) * x725 * x726)) + (((-1.0) * sj1 * x729)) + (((128996060.794774) * sj1)) + (((-0.251106194690265) * r21 * x731)) + ((x724 * x727)) + ((x732 * x735)))) - 1) <= IKFAST_SINCOS_THRESH)
                                                                continue;
                                                            j2array[0] = IKatan2((((cj1 * x729)) + (((-1.0) * x724 * x725)) + (((-1.0) * x731 * x732)) + (((-1.0) * x731 * x736)) + ((x726 * x733)) + (((-1.0) * x734 * x735)) + (((-0.251106194690265) * r21 * x735)) + ((x724 * x730)) + (((-128996060.794774) * cj1)) + (((-1.0) * x726 * x727)) + ((sj1 * x728))), ((-1.03318584070796) + ((cj1 * x728)) + (((-1.0) * x731 * x734)) + ((x735 * x736)) + (((-1.0) * x724 * x733)) + ((x726 * x730)) + (((-1.0) * x725 * x726)) + (((-1.0) * sj1 * x729)) + (((128996060.794774) * sj1)) + (((-0.251106194690265) * r21 * x731)) + ((x724 * x727)) + ((x732 * x735))));
                                                            sj2array[0] = IKsin(j2array[0]);
                                                            cj2array[0] = IKcos(j2array[0]);
                                                            if (j2array[0] > IKPI)
                                                            {
                                                                j2array[0] -= IK2PI;
                                                            }
                                                            else if (j2array[0] < -IKPI)
                                                            {
                                                                j2array[0] += IK2PI;
                                                            }
                                                            j2valid[0] = true;
                                                            for (int ij2 = 0; ij2 < 1; ++ij2)
                                                            {
                                                                if (!j2valid[ij2])
                                                                {
                                                                    continue;
                                                                }
                                                                _ij2[0] = ij2;
                                                                _ij2[1] = -1;
                                                                for (int iij2 = ij2 + 1; iij2 < 1; ++iij2)
                                                                {
                                                                    if (j2valid[iij2] && IKabs(cj2array[ij2] - cj2array[iij2]) < IKFAST_SOLUTION_THRESH && IKabs(sj2array[ij2] - sj2array[iij2]) < IKFAST_SOLUTION_THRESH)
                                                                    {
                                                                        j2valid[iij2] = false;
                                                                        _ij2[1] = iij2;
                                                                        break;
                                                                    }
                                                                }
                                                                j2 = j2array[ij2];
                                                                cj2 = cj2array[ij2];
                                                                sj2 = sj2array[ij2];
                                                                {
                                                                    IkReal evalcond[3];
                                                                    IkReal x738 = IKsin(j2);
                                                                    IkReal x739 = IKcos(j2);
                                                                    IkReal x740 = ((3.7e-7) * cj5);
                                                                    IkReal x741 = ((0.1135) * sj5);
                                                                    IkReal x742 = ((3.7e-7) * sj5);
                                                                    IkReal x743 = ((0.1135) * cj5);
                                                                    IkReal x744 = (cj1 * x738);
                                                                    IkReal x745 = (sj1 * x739);
                                                                    evalcond[0] = ((((-1.0) * r20 * x743)) + (((0.467) * cj1)) + (((0.452) * cj1 * x739)) + (((-1.0) * pz)) + (((0.452) * sj1 * x738)) + ((r21 * x741)) + ((r21 * x740)) + ((r20 * x742)));
                                                                    evalcond[1] = ((0.11339999909077) + (((8.79096604904732e-10) * x744)) + (((-9.08270164802013e-10) * sj1)) + (((-1.0) * px)) + (((-8.79096604904732e-10) * x745)) + (((-1.0) * r00 * x743)) + ((r00 * x742)) + ((r01 * x740)) + ((r01 * x741)));
                                                                    evalcond[2] = ((-1.21543325961255e-7) + ((r10 * x742)) + (((0.452000001519335) * x744)) + (((-1.0) * py)) + ((r11 * x740)) + ((r11 * x741)) + (((-1.0) * r10 * x743)) + (((-0.452000001519335) * x745)) + (((-0.467000001569756) * sj1)));
                                                                    if (IKabs(evalcond[0]) > IKFAST_EVALCOND_THRESH || IKabs(evalcond[1]) > IKFAST_EVALCOND_THRESH || IKabs(evalcond[2]) > IKFAST_EVALCOND_THRESH)
                                                                    {
                                                                        continue;
                                                                    }
                                                                }

                                                                {
                                                                    IkReal j3eval[2];
                                                                    sj0 = 0.19503986;
                                                                    cj0 = 0.98079532;
                                                                    j0 = 0.19629809;
                                                                    IkReal x746 = (((cj5 * r11 * sj4)) + ((r10 * sj4 * sj5)) + (((-1.0) * cj4 * r12)));
                                                                    j3eval[0] = x746;
                                                                    j3eval[1] = IKsign(x746);
                                                                    if (IKabs(j3eval[0]) < 0.0000010000000000 || IKabs(j3eval[1]) < 0.0000010000000000)
                                                                    {
                                                                        {
                                                                            IkReal j3eval[2];
                                                                            sj0 = 0.19503986;
                                                                            cj0 = 0.98079532;
                                                                            j0 = 0.19629809;
                                                                            IkReal x747 = ((1.0) * sj4);
                                                                            IkReal x748 = ((((-1.0) * r00 * sj5 * x747)) + (((-1.0) * cj5 * r01 * x747)) + ((cj4 * r02)));
                                                                            j3eval[0] = x748;
                                                                            j3eval[1] = IKsign(x748);
                                                                            if (IKabs(j3eval[0]) < 0.0000010000000000 || IKabs(j3eval[1]) < 0.0000010000000000)
                                                                            {
                                                                                {
                                                                                    IkReal j3eval[2];
                                                                                    sj0 = 0.19503986;
                                                                                    cj0 = 0.98079532;
                                                                                    j0 = 0.19629809;
                                                                                    IkReal x749 = cj4 * cj4;
                                                                                    IkReal x750 = cj5 * cj5;
                                                                                    IkReal x751 = r22 * r22;
                                                                                    IkReal x752 = r21 * r21;
                                                                                    IkReal x753 = r20 * r20;
                                                                                    IkReal x754 = (r20 * sj5);
                                                                                    IkReal x755 = (cj5 * r21);
                                                                                    IkReal x756 = ((1.0) * x752);
                                                                                    IkReal x757 = ((1.0) * x753);
                                                                                    IkReal x758 = (x749 * x750);
                                                                                    IkReal x759 = ((2.0) * cj4 * r22 * sj4);
                                                                                    IkReal x760 = ((((-1.0) * x750 * x757)) + (((2.0) * x754 * x755)) + (((-1.0) * x754 * x759)) + (((-2.0) * x749 * x754 * x755)) + (((-1.0) * x755 * x759)) + (((-1.0) * x751)) + (((-1.0) * x756 * x758)) + (((-1.0) * x749 * x757)) + ((x753 * x758)) + ((x749 * x751)) + ((x750 * x752)) + (((-1.0) * x756)));
                                                                                    j3eval[0] = x760;
                                                                                    j3eval[1] = IKsign(x760);
                                                                                    if (IKabs(j3eval[0]) < 0.0000010000000000 || IKabs(j3eval[1]) < 0.0000010000000000)
                                                                                    {
                                                                                        continue; // no branches [j3]
                                                                                    }
                                                                                    else
                                                                                    {
                                                                                        {
                                                                                            IkReal j3array[1], cj3array[1], sj3array[1];
                                                                                            bool j3valid[1] = {false};
                                                                                            _nj3 = 1;
                                                                                            IkReal x761 = cj4 * cj4;
                                                                                            IkReal x762 = cj5 * cj5;
                                                                                            IkReal x763 = r22 * r22;
                                                                                            IkReal x764 = r21 * r21;
                                                                                            IkReal x765 = r20 * r20;
                                                                                            IkReal x766 = ((1.0) * sj1);
                                                                                            IkReal x767 = (r22 * sj4);
                                                                                            IkReal x768 = (sj2 * sj5);
                                                                                            IkReal x769 = (cj4 * r20);
                                                                                            IkReal x770 = (cj5 * sj2);
                                                                                            IkReal x771 = (r21 * sj5);
                                                                                            IkReal x772 = (cj4 * r21);
                                                                                            IkReal x773 = ((2.0) * cj5);
                                                                                            IkReal x774 = (cj2 * cj5 * r20);
                                                                                            IkReal x775 = ((1.0) * x764);
                                                                                            IkReal x776 = ((1.0) * cj1 * cj2);
                                                                                            IkReal x777 = ((1.0) * x765);
                                                                                            IkReal x778 = (x761 * x762);
                                                                                            CheckValue<IkReal> x779 = IKatan2WithCheck(IkReal(((((-1.0) * x766 * x770 * x772)) + (((-1.0) * x766 * x768 * x769)) + (((-1.0) * cj5 * x772 * x776)) + (((-1.0) * cj1 * r21 * x768)) + (((-1.0) * x767 * x776)) + ((cj1 * r20 * x770)) + (((-1.0) * x766 * x774)) + ((cj2 * sj1 * x771)) + (((-1.0) * sj2 * x766 * x767)) + (((-1.0) * sj5 * x769 * x776)))), IkReal((((cj1 * x768 * x769)) + ((cj1 * x774)) + (((-1.0) * r21 * x766 * x768)) + (((-1.0) * cj2 * sj5 * x766 * x769)) + (((-1.0) * x771 * x776)) + (((-1.0) * cj2 * x766 * x767)) + ((cj1 * sj2 * x767)) + (((-1.0) * cj2 * cj5 * x766 * x772)) + ((r20 * sj1 * x770)) + ((cj1 * x770 * x772)))), IKFAST_ATAN2_MAGTHRESH);
                                                                                            if (!x779.valid)
                                                                                            {
                                                                                                continue;
                                                                                            }
                                                                                            CheckValue<IkReal> x780 = IKPowWithIntegerCheck(IKsign(((((-2.0) * sj5 * x767 * x769)) + (((-1.0) * x767 * x772 * x773)) + (((-1.0) * r20 * x761 * x771 * x773)) + ((x761 * x763)) + ((x762 * x764)) + ((x765 * x778)) + (((-1.0) * x763)) + (((-1.0) * x775 * x778)) + (((-1.0) * x775)) + (((-1.0) * x762 * x777)) + ((r20 * x771 * x773)) + (((-1.0) * x761 * x777)))), -1);
                                                                                            if (!x780.valid)
                                                                                            {
                                                                                                continue;
                                                                                            }
                                                                                            j3array[0] = ((-1.5707963267949) + (x779.value) + (((1.5707963267949) * (x780.value))));
                                                                                            sj3array[0] = IKsin(j3array[0]);
                                                                                            cj3array[0] = IKcos(j3array[0]);
                                                                                            if (j3array[0] > IKPI)
                                                                                            {
                                                                                                j3array[0] -= IK2PI;
                                                                                            }
                                                                                            else if (j3array[0] < -IKPI)
                                                                                            {
                                                                                                j3array[0] += IK2PI;
                                                                                            }
                                                                                            j3valid[0] = true;
                                                                                            for (int ij3 = 0; ij3 < 1; ++ij3)
                                                                                            {
                                                                                                if (!j3valid[ij3])
                                                                                                {
                                                                                                    continue;
                                                                                                }
                                                                                                _ij3[0] = ij3;
                                                                                                _ij3[1] = -1;
                                                                                                for (int iij3 = ij3 + 1; iij3 < 1; ++iij3)
                                                                                                {
                                                                                                    if (j3valid[iij3] && IKabs(cj3array[ij3] - cj3array[iij3]) < IKFAST_SOLUTION_THRESH && IKabs(sj3array[ij3] - sj3array[iij3]) < IKFAST_SOLUTION_THRESH)
                                                                                                    {
                                                                                                        j3valid[iij3] = false;
                                                                                                        _ij3[1] = iij3;
                                                                                                        break;
                                                                                                    }
                                                                                                }
                                                                                                j3 = j3array[ij3];
                                                                                                cj3 = cj3array[ij3];
                                                                                                sj3 = sj3array[ij3];
                                                                                                {
                                                                                                    IkReal evalcond[6];
                                                                                                    IkReal x781 = IKcos(j3);
                                                                                                    IkReal x782 = IKsin(j3);
                                                                                                    IkReal x783 = ((1.0) * r11);
                                                                                                    IkReal x784 = ((1.94490399315206e-9) * sj2);
                                                                                                    IkReal x785 = (sj1 * sj2);
                                                                                                    IkReal x786 = (cj2 * sj1);
                                                                                                    IkReal x787 = (cj4 * r01);
                                                                                                    IkReal x788 = (cj1 * sj2);
                                                                                                    IkReal x789 = (cj1 * cj2);
                                                                                                    IkReal x790 = ((1.0) * r21);
                                                                                                    IkReal x791 = (cj5 * x781);
                                                                                                    IkReal x792 = (sj4 * x781);
                                                                                                    IkReal x793 = (cj5 * x782);
                                                                                                    IkReal x794 = (sj5 * x781);
                                                                                                    IkReal x795 = ((1.0) * x782);
                                                                                                    IkReal x796 = (cj4 * sj5 * x795);
                                                                                                    evalcond[0] = (((r20 * x793)) + ((cj4 * r20 * x794)) + ((cj4 * r21 * x791)) + (((-1.0) * sj5 * x782 * x790)) + ((r22 * x792)) + x788 + (((-1.0) * x786)));
                                                                                                    evalcond[1] = (((r20 * x791)) + (((-1.0) * r20 * x796)) + (((-1.0) * x790 * x794)) + (((-1.0) * r22 * sj4 * x795)) + (((-1.0) * cj4 * x790 * x793)) + x785 + x789);
                                                                                                    evalcond[2] = (((r00 * x793)) + (((-1.94490399315206e-9) * x789)) + ((cj4 * r00 * x794)) + (((-1.0) * r01 * sj5 * x795)) + ((x787 * x791)) + ((r02 * x792)) + (((-1.0) * sj1 * x784)));
                                                                                                    evalcond[3] = (((r10 * x793)) + ((cj4 * r10 * x794)) + (((-1.0) * sj5 * x782 * x783)) + (((-1.00000000336136) * x785)) + (((-1.00000000336136) * x789)) + ((r12 * x792)) + ((cj4 * r11 * x791)));
                                                                                                    evalcond[4] = (((r00 * x791)) + (((-1.0) * r01 * x794)) + (((-1.94490399315206e-9) * x786)) + (((-1.0) * r00 * x796)) + ((cj1 * x784)) + (((-1.0) * x787 * x793)) + (((-1.0) * r02 * sj4 * x795)));
                                                                                                    evalcond[5] = ((((-1.0) * cj4 * x783 * x793)) + (((-1.0) * x783 * x794)) + ((r10 * x791)) + (((-1.0) * r12 * sj4 * x795)) + (((1.00000000336136) * x788)) + (((-1.0) * r10 * x796)) + (((-1.00000000336136) * x786)));
                                                                                                    if (IKabs(evalcond[0]) > IKFAST_EVALCOND_THRESH || IKabs(evalcond[1]) > IKFAST_EVALCOND_THRESH || IKabs(evalcond[2]) > IKFAST_EVALCOND_THRESH || IKabs(evalcond[3]) > IKFAST_EVALCOND_THRESH || IKabs(evalcond[4]) > IKFAST_EVALCOND_THRESH || IKabs(evalcond[5]) > IKFAST_EVALCOND_THRESH)
                                                                                                    {
                                                                                                        continue;
                                                                                                    }
                                                                                                }

                                                                                                {
                                                                                                    std::vector<IkSingleDOFSolutionBase<IkReal>> vinfos(6);
                                                                                                    vinfos[0].jointtype = 1;
                                                                                                    vinfos[0].foffset = j0;
                                                                                                    vinfos[0].indices[0] = _ij0[0];
                                                                                                    vinfos[0].indices[1] = _ij0[1];
                                                                                                    vinfos[0].maxsolutions = _nj0;
                                                                                                    vinfos[1].jointtype = 1;
                                                                                                    vinfos[1].foffset = j1;
                                                                                                    vinfos[1].indices[0] = _ij1[0];
                                                                                                    vinfos[1].indices[1] = _ij1[1];
                                                                                                    vinfos[1].maxsolutions = _nj1;
                                                                                                    vinfos[2].jointtype = 1;
                                                                                                    vinfos[2].foffset = j2;
                                                                                                    vinfos[2].indices[0] = _ij2[0];
                                                                                                    vinfos[2].indices[1] = _ij2[1];
                                                                                                    vinfos[2].maxsolutions = _nj2;
                                                                                                    vinfos[3].jointtype = 1;
                                                                                                    vinfos[3].foffset = j3;
                                                                                                    vinfos[3].indices[0] = _ij3[0];
                                                                                                    vinfos[3].indices[1] = _ij3[1];
                                                                                                    vinfos[3].maxsolutions = _nj3;
                                                                                                    vinfos[4].jointtype = 1;
                                                                                                    vinfos[4].foffset = j4;
                                                                                                    vinfos[4].indices[0] = _ij4[0];
                                                                                                    vinfos[4].indices[1] = _ij4[1];
                                                                                                    vinfos[4].maxsolutions = _nj4;
                                                                                                    vinfos[5].jointtype = 1;
                                                                                                    vinfos[5].foffset = j5;
                                                                                                    vinfos[5].indices[0] = _ij5[0];
                                                                                                    vinfos[5].indices[1] = _ij5[1];
                                                                                                    vinfos[5].maxsolutions = _nj5;
                                                                                                    std::vector<int> vfree(0);
                                                                                                    solutions.AddSolution(vinfos, vfree);
                                                                                                }
                                                                                            }
                                                                                        }
                                                                                    }
                                                                                }
                                                                            }
                                                                            else
                                                                            {
                                                                                {
                                                                                    IkReal j3array[1], cj3array[1], sj3array[1];
                                                                                    bool j3valid[1] = {false};
                                                                                    _nj3 = 1;
                                                                                    IkReal x797 = (cj1 * cj5);
                                                                                    IkReal x798 = ((1.00000000336136) * r20);
                                                                                    IkReal x799 = ((1.0) * sj4);
                                                                                    IkReal x800 = (cj4 * sj2);
                                                                                    IkReal x801 = (sj1 * sj5);
                                                                                    IkReal x802 = ((1.00000000336136) * r21);
                                                                                    IkReal x803 = (cj5 * sj1);
                                                                                    IkReal x804 = (sj2 * sj4);
                                                                                    IkReal x805 = (cj2 * r10);
                                                                                    IkReal x806 = ((1.0) * cj4);
                                                                                    IkReal x807 = ((1.00000000336136) * r22);
                                                                                    IkReal x808 = (cj1 * cj2);
                                                                                    IkReal x809 = (cj1 * sj5);
                                                                                    IkReal x810 = (cj2 * r11);
                                                                                    CheckValue<IkReal> x811 = IKatan2WithCheck(IkReal((((sj1 * x804 * x807)) + (((-1.0) * cj2 * r12 * sj1 * x799)) + ((cj1 * r12 * x804)) + ((x798 * x800 * x801)) + ((sj4 * x807 * x808)) + ((r10 * x800 * x809)) + ((cj2 * cj4 * x797 * x802)) + (((-1.0) * x803 * x806 * x810)) + ((r11 * x797 * x800)) + ((cj4 * sj5 * x798 * x808)) + ((x800 * x802 * x803)) + (((-1.0) * x801 * x805 * x806)))), IkReal(((((-1.0) * r10 * sj2 * x797)) + ((r11 * sj2 * x809)) + (((-1.0) * sj2 * x798 * x803)) + ((x803 * x805)) + ((sj5 * x802 * x808)) + ((sj2 * x801 * x802)) + (((-1.0) * cj2 * x797 * x798)) + (((-1.0) * x801 * x810)))), IKFAST_ATAN2_MAGTHRESH);
                                                                                    if (!x811.valid)
                                                                                    {
                                                                                        continue;
                                                                                    }
                                                                                    CheckValue<IkReal> x812 = IKPowWithIntegerCheck(IKsign(((((-1.0) * cj5 * r01 * x799)) + (((-1.0) * r00 * sj5 * x799)) + ((cj4 * r02)))), -1);
                                                                                    if (!x812.valid)
                                                                                    {
                                                                                        continue;
                                                                                    }
                                                                                    j3array[0] = ((-1.5707963267949) + (x811.value) + (((1.5707963267949) * (x812.value))));
                                                                                    sj3array[0] = IKsin(j3array[0]);
                                                                                    cj3array[0] = IKcos(j3array[0]);
                                                                                    if (j3array[0] > IKPI)
                                                                                    {
                                                                                        j3array[0] -= IK2PI;
                                                                                    }
                                                                                    else if (j3array[0] < -IKPI)
                                                                                    {
                                                                                        j3array[0] += IK2PI;
                                                                                    }
                                                                                    j3valid[0] = true;
                                                                                    for (int ij3 = 0; ij3 < 1; ++ij3)
                                                                                    {
                                                                                        if (!j3valid[ij3])
                                                                                        {
                                                                                            continue;
                                                                                        }
                                                                                        _ij3[0] = ij3;
                                                                                        _ij3[1] = -1;
                                                                                        for (int iij3 = ij3 + 1; iij3 < 1; ++iij3)
                                                                                        {
                                                                                            if (j3valid[iij3] && IKabs(cj3array[ij3] - cj3array[iij3]) < IKFAST_SOLUTION_THRESH && IKabs(sj3array[ij3] - sj3array[iij3]) < IKFAST_SOLUTION_THRESH)
                                                                                            {
                                                                                                j3valid[iij3] = false;
                                                                                                _ij3[1] = iij3;
                                                                                                break;
                                                                                            }
                                                                                        }
                                                                                        j3 = j3array[ij3];
                                                                                        cj3 = cj3array[ij3];
                                                                                        sj3 = sj3array[ij3];
                                                                                        {
                                                                                            IkReal evalcond[6];
                                                                                            IkReal x813 = IKcos(j3);
                                                                                            IkReal x814 = IKsin(j3);
                                                                                            IkReal x815 = ((1.0) * r11);
                                                                                            IkReal x816 = ((1.94490399315206e-9) * sj2);
                                                                                            IkReal x817 = (sj1 * sj2);
                                                                                            IkReal x818 = (cj2 * sj1);
                                                                                            IkReal x819 = (cj4 * r01);
                                                                                            IkReal x820 = (cj1 * sj2);
                                                                                            IkReal x821 = (cj1 * cj2);
                                                                                            IkReal x822 = ((1.0) * r21);
                                                                                            IkReal x823 = (cj5 * x813);
                                                                                            IkReal x824 = (sj4 * x813);
                                                                                            IkReal x825 = (cj5 * x814);
                                                                                            IkReal x826 = (sj5 * x813);
                                                                                            IkReal x827 = ((1.0) * x814);
                                                                                            IkReal x828 = (cj4 * sj5 * x827);
                                                                                            evalcond[0] = (((r20 * x825)) + (((-1.0) * x818)) + (((-1.0) * sj5 * x814 * x822)) + ((r22 * x824)) + ((cj4 * r20 * x826)) + ((cj4 * r21 * x823)) + x820);
                                                                                            evalcond[1] = ((((-1.0) * r20 * x828)) + (((-1.0) * r22 * sj4 * x827)) + (((-1.0) * x822 * x826)) + ((r20 * x823)) + (((-1.0) * cj4 * x822 * x825)) + x817 + x821);
                                                                                            evalcond[2] = ((((-1.0) * r01 * sj5 * x827)) + ((x819 * x823)) + ((r00 * x825)) + ((cj4 * r00 * x826)) + (((-1.94490399315206e-9) * x821)) + (((-1.0) * sj1 * x816)) + ((r02 * x824)));
                                                                                            evalcond[3] = (((r12 * x824)) + (((-1.00000000336136) * x817)) + ((cj4 * r11 * x823)) + (((-1.00000000336136) * x821)) + ((r10 * x825)) + (((-1.0) * sj5 * x814 * x815)) + ((cj4 * r10 * x826)));
                                                                                            evalcond[4] = ((((-1.0) * r01 * x826)) + ((r00 * x823)) + (((-1.94490399315206e-9) * x818)) + (((-1.0) * x819 * x825)) + ((cj1 * x816)) + (((-1.0) * r00 * x828)) + (((-1.0) * r02 * sj4 * x827)));
                                                                                            evalcond[5] = ((((-1.00000000336136) * x818)) + (((-1.0) * cj4 * x815 * x825)) + (((-1.0) * x815 * x826)) + ((r10 * x823)) + (((-1.0) * r12 * sj4 * x827)) + (((-1.0) * r10 * x828)) + (((1.00000000336136) * x820)));
                                                                                            if (IKabs(evalcond[0]) > IKFAST_EVALCOND_THRESH || IKabs(evalcond[1]) > IKFAST_EVALCOND_THRESH || IKabs(evalcond[2]) > IKFAST_EVALCOND_THRESH || IKabs(evalcond[3]) > IKFAST_EVALCOND_THRESH || IKabs(evalcond[4]) > IKFAST_EVALCOND_THRESH || IKabs(evalcond[5]) > IKFAST_EVALCOND_THRESH)
                                                                                            {
                                                                                                continue;
                                                                                            }
                                                                                        }

                                                                                        {
                                                                                            std::vector<IkSingleDOFSolutionBase<IkReal>> vinfos(6);
                                                                                            vinfos[0].jointtype = 1;
                                                                                            vinfos[0].foffset = j0;
                                                                                            vinfos[0].indices[0] = _ij0[0];
                                                                                            vinfos[0].indices[1] = _ij0[1];
                                                                                            vinfos[0].maxsolutions = _nj0;
                                                                                            vinfos[1].jointtype = 1;
                                                                                            vinfos[1].foffset = j1;
                                                                                            vinfos[1].indices[0] = _ij1[0];
                                                                                            vinfos[1].indices[1] = _ij1[1];
                                                                                            vinfos[1].maxsolutions = _nj1;
                                                                                            vinfos[2].jointtype = 1;
                                                                                            vinfos[2].foffset = j2;
                                                                                            vinfos[2].indices[0] = _ij2[0];
                                                                                            vinfos[2].indices[1] = _ij2[1];
                                                                                            vinfos[2].maxsolutions = _nj2;
                                                                                            vinfos[3].jointtype = 1;
                                                                                            vinfos[3].foffset = j3;
                                                                                            vinfos[3].indices[0] = _ij3[0];
                                                                                            vinfos[3].indices[1] = _ij3[1];
                                                                                            vinfos[3].maxsolutions = _nj3;
                                                                                            vinfos[4].jointtype = 1;
                                                                                            vinfos[4].foffset = j4;
                                                                                            vinfos[4].indices[0] = _ij4[0];
                                                                                            vinfos[4].indices[1] = _ij4[1];
                                                                                            vinfos[4].maxsolutions = _nj4;
                                                                                            vinfos[5].jointtype = 1;
                                                                                            vinfos[5].foffset = j5;
                                                                                            vinfos[5].indices[0] = _ij5[0];
                                                                                            vinfos[5].indices[1] = _ij5[1];
                                                                                            vinfos[5].maxsolutions = _nj5;
                                                                                            std::vector<int> vfree(0);
                                                                                            solutions.AddSolution(vinfos, vfree);
                                                                                        }
                                                                                    }
                                                                                }
                                                                            }
                                                                        }
                                                                    }
                                                                    else
                                                                    {
                                                                        {
                                                                            IkReal j3array[1], cj3array[1], sj3array[1];
                                                                            bool j3valid[1] = {false};
                                                                            _nj3 = 1;
                                                                            IkReal x829 = (sj1 * sj5);
                                                                            IkReal x830 = ((1.0) * r01);
                                                                            IkReal x831 = (cj2 * sj1);
                                                                            IkReal x832 = (cj1 * sj2);
                                                                            IkReal x833 = (r02 * sj4);
                                                                            IkReal x834 = (cj1 * cj2);
                                                                            IkReal x835 = ((1.94490399315206e-9) * r20);
                                                                            IkReal x836 = (sj1 * sj2);
                                                                            IkReal x837 = (cj4 * r00);
                                                                            IkReal x838 = (cj5 * r00);
                                                                            IkReal x839 = ((1.94490399315206e-9) * r21);
                                                                            IkReal x840 = ((1.94490399315206e-9) * r22 * sj4);
                                                                            IkReal x841 = (cj4 * cj5 * r01);
                                                                            IkReal x842 = (cj4 * cj5 * x839);
                                                                            CheckValue<IkReal> x843 = IKatan2WithCheck(IkReal(((((-1.0) * sj2 * x829 * x830)) + (((-1.0) * cj5 * x832 * x835)) + ((sj5 * x832 * x839)) + (((-1.0) * sj5 * x830 * x834)) + ((x836 * x838)) + (((-1.0) * cj2 * x829 * x839)) + ((x834 * x838)) + ((cj5 * x831 * x835)))), IkReal((((sj2 * x829 * x837)) + ((sj5 * x834 * x837)) + ((x833 * x836)) + ((x833 * x834)) + ((x836 * x841)) + ((x834 * x841)) + ((cj2 * cj4 * x829 * x835)) + (((-1.0) * cj4 * sj5 * x832 * x835)) + (((-1.0) * x832 * x840)) + (((-1.0) * x832 * x842)) + ((x831 * x842)) + ((x831 * x840)))), IKFAST_ATAN2_MAGTHRESH);
                                                                            if (!x843.valid)
                                                                            {
                                                                                continue;
                                                                            }
                                                                            CheckValue<IkReal> x844 = IKPowWithIntegerCheck(IKsign((((cj5 * r11 * sj4)) + ((r10 * sj4 * sj5)) + (((-1.0) * cj4 * r12)))), -1);
                                                                            if (!x844.valid)
                                                                            {
                                                                                continue;
                                                                            }
                                                                            j3array[0] = ((-1.5707963267949) + (x843.value) + (((1.5707963267949) * (x844.value))));
                                                                            sj3array[0] = IKsin(j3array[0]);
                                                                            cj3array[0] = IKcos(j3array[0]);
                                                                            if (j3array[0] > IKPI)
                                                                            {
                                                                                j3array[0] -= IK2PI;
                                                                            }
                                                                            else if (j3array[0] < -IKPI)
                                                                            {
                                                                                j3array[0] += IK2PI;
                                                                            }
                                                                            j3valid[0] = true;
                                                                            for (int ij3 = 0; ij3 < 1; ++ij3)
                                                                            {
                                                                                if (!j3valid[ij3])
                                                                                {
                                                                                    continue;
                                                                                }
                                                                                _ij3[0] = ij3;
                                                                                _ij3[1] = -1;
                                                                                for (int iij3 = ij3 + 1; iij3 < 1; ++iij3)
                                                                                {
                                                                                    if (j3valid[iij3] && IKabs(cj3array[ij3] - cj3array[iij3]) < IKFAST_SOLUTION_THRESH && IKabs(sj3array[ij3] - sj3array[iij3]) < IKFAST_SOLUTION_THRESH)
                                                                                    {
                                                                                        j3valid[iij3] = false;
                                                                                        _ij3[1] = iij3;
                                                                                        break;
                                                                                    }
                                                                                }
                                                                                j3 = j3array[ij3];
                                                                                cj3 = cj3array[ij3];
                                                                                sj3 = sj3array[ij3];
                                                                                {
                                                                                    IkReal evalcond[6];
                                                                                    IkReal x845 = IKcos(j3);
                                                                                    IkReal x846 = IKsin(j3);
                                                                                    IkReal x847 = ((1.0) * r11);
                                                                                    IkReal x848 = ((1.94490399315206e-9) * sj2);
                                                                                    IkReal x849 = (sj1 * sj2);
                                                                                    IkReal x850 = (cj2 * sj1);
                                                                                    IkReal x851 = (cj4 * r01);
                                                                                    IkReal x852 = (cj1 * sj2);
                                                                                    IkReal x853 = (cj1 * cj2);
                                                                                    IkReal x854 = ((1.0) * r21);
                                                                                    IkReal x855 = (cj5 * x845);
                                                                                    IkReal x856 = (sj4 * x845);
                                                                                    IkReal x857 = (cj5 * x846);
                                                                                    IkReal x858 = (sj5 * x845);
                                                                                    IkReal x859 = ((1.0) * x846);
                                                                                    IkReal x860 = (cj4 * sj5 * x859);
                                                                                    evalcond[0] = ((((-1.0) * x850)) + ((cj4 * r21 * x855)) + ((cj4 * r20 * x858)) + ((r22 * x856)) + (((-1.0) * sj5 * x846 * x854)) + x852 + ((r20 * x857)));
                                                                                    evalcond[1] = ((((-1.0) * cj4 * x854 * x857)) + (((-1.0) * r22 * sj4 * x859)) + (((-1.0) * r20 * x860)) + x853 + x849 + (((-1.0) * x854 * x858)) + ((r20 * x855)));
                                                                                    evalcond[2] = (((r00 * x857)) + ((cj4 * r00 * x858)) + ((x851 * x855)) + (((-1.0) * r01 * sj5 * x859)) + ((r02 * x856)) + (((-1.94490399315206e-9) * x853)) + (((-1.0) * sj1 * x848)));
                                                                                    evalcond[3] = (((r10 * x857)) + ((r12 * x856)) + (((-1.00000000336136) * x849)) + ((cj4 * r10 * x858)) + ((cj4 * r11 * x855)) + (((-1.00000000336136) * x853)) + (((-1.0) * sj5 * x846 * x847)));
                                                                                    evalcond[4] = (((cj1 * x848)) + ((r00 * x855)) + (((-1.0) * r00 * x860)) + (((-1.0) * r01 * x858)) + (((-1.0) * x851 * x857)) + (((-1.94490399315206e-9) * x850)) + (((-1.0) * r02 * sj4 * x859)));
                                                                                    evalcond[5] = ((((-1.0) * r10 * x860)) + ((r10 * x855)) + (((-1.0) * cj4 * x847 * x857)) + (((-1.0) * x847 * x858)) + (((1.00000000336136) * x852)) + (((-1.00000000336136) * x850)) + (((-1.0) * r12 * sj4 * x859)));
                                                                                    if (IKabs(evalcond[0]) > IKFAST_EVALCOND_THRESH || IKabs(evalcond[1]) > IKFAST_EVALCOND_THRESH || IKabs(evalcond[2]) > IKFAST_EVALCOND_THRESH || IKabs(evalcond[3]) > IKFAST_EVALCOND_THRESH || IKabs(evalcond[4]) > IKFAST_EVALCOND_THRESH || IKabs(evalcond[5]) > IKFAST_EVALCOND_THRESH)
                                                                                    {
                                                                                        continue;
                                                                                    }
                                                                                }

                                                                                {
                                                                                    std::vector<IkSingleDOFSolutionBase<IkReal>> vinfos(6);
                                                                                    vinfos[0].jointtype = 1;
                                                                                    vinfos[0].foffset = j0;
                                                                                    vinfos[0].indices[0] = _ij0[0];
                                                                                    vinfos[0].indices[1] = _ij0[1];
                                                                                    vinfos[0].maxsolutions = _nj0;
                                                                                    vinfos[1].jointtype = 1;
                                                                                    vinfos[1].foffset = j1;
                                                                                    vinfos[1].indices[0] = _ij1[0];
                                                                                    vinfos[1].indices[1] = _ij1[1];
                                                                                    vinfos[1].maxsolutions = _nj1;
                                                                                    vinfos[2].jointtype = 1;
                                                                                    vinfos[2].foffset = j2;
                                                                                    vinfos[2].indices[0] = _ij2[0];
                                                                                    vinfos[2].indices[1] = _ij2[1];
                                                                                    vinfos[2].maxsolutions = _nj2;
                                                                                    vinfos[3].jointtype = 1;
                                                                                    vinfos[3].foffset = j3;
                                                                                    vinfos[3].indices[0] = _ij3[0];
                                                                                    vinfos[3].indices[1] = _ij3[1];
                                                                                    vinfos[3].maxsolutions = _nj3;
                                                                                    vinfos[4].jointtype = 1;
                                                                                    vinfos[4].foffset = j4;
                                                                                    vinfos[4].indices[0] = _ij4[0];
                                                                                    vinfos[4].indices[1] = _ij4[1];
                                                                                    vinfos[4].maxsolutions = _nj4;
                                                                                    vinfos[5].jointtype = 1;
                                                                                    vinfos[5].foffset = j5;
                                                                                    vinfos[5].indices[0] = _ij5[0];
                                                                                    vinfos[5].indices[1] = _ij5[1];
                                                                                    vinfos[5].maxsolutions = _nj5;
                                                                                    std::vector<int> vfree(0);
                                                                                    solutions.AddSolution(vinfos, vfree);
                                                                                }
                                                                            }
                                                                        }
                                                                    }
                                                                }
                                                            }
                                                        }
                                                    }
                                                } while (0);
                                                if (bgotonextstatement)
                                                {
                                                    bool bgotonextstatement = true;
                                                    do
                                                    {
                                                        if (1)
                                                        {
                                                            bgotonextstatement = false;
                                                            continue; // branch miss [j2, j3]
                                                        }
                                                    } while (0);
                                                    if (bgotonextstatement)
                                                    {
                                                    }
                                                }
                                            }
                                        }
                                    }
                                }
                            }
                            else
                            {
                                {
                                    IkReal j2array[1], cj2array[1], sj2array[1];
                                    bool j2valid[1] = {false};
                                    _nj2 = 1;
                                    IkReal x861 = ((2.50148374048693) * cj5);
                                    IkReal x862 = ((8.15461659894417e-6) * sj5);
                                    IkReal x863 = (cj0 * r20);
                                    IkReal x864 = ((12.5791903295117) * sj1);
                                    IkReal x865 = (sj1 * sj5);
                                    IkReal x866 = ((4.181e-5) * r10);
                                    IkReal x867 = (cj1 * cj5);
                                    IkReal x868 = ((4.181e-5) * r11);
                                    IkReal x869 = (cj1 * sj5);
                                    IkReal x870 = (r21 * sj0);
                                    IkReal x871 = (sj0 * sj1);
                                    IkReal x872 = ((12.8255) * r11);
                                    IkReal x873 = ((12.8255) * r10);
                                    IkReal x874 = ((22.0395043214707) * pz);
                                    IkReal x875 = (cj1 * sj0);
                                    IkReal x876 = (cj0 * cj1);
                                    IkReal x877 = (cj0 * r21);
                                    IkReal x878 = ((113.0) * py);
                                    IkReal x879 = (cj5 * sj1);
                                    IkReal x880 = ((110.829870744596) * pz);
                                    IkReal x881 = (cj0 * sj1);
                                    IkReal x882 = (r20 * x875);
                                    CheckValue<IkReal> x883 = IKatan2WithCheck(IkReal((((cj5 * x863 * x864)) + (((-4.10070521755005e-5) * x877 * x879)) + ((x867 * x873)) + ((x871 * x874)) + (((-8.15461659894417e-6) * x870 * x879)) + ((r20 * x861 * x871)) + ((x880 * x881)) + (((-4.10070521755005e-5) * x863 * x865)) + (((-1.0) * x866 * x869)) + (((-1.0) * x869 * x872)) + (((-12.5681045255276) * x875)) + (((-2.50148374048693) * x865 * x870)) + ((cj1 * x878)) + (((2.49929320780212) * x876)) + (((-1.0) * x867 * x868)) + (((-1.0) * sj5 * x864 * x877)) + (((-1.0) * r20 * x862 * x871)))), IkReal(((((-51.7575496377264) * cj0)) + ((x868 * x879)) + ((x874 * x875)) + (((-4.10070521755005e-5) * x863 * x869)) + (((-8.15461659894417e-6) * x867 * x870)) + (((-2.50148374048693) * x869 * x870)) + ((x876 * x880)) + (((12.5681045255276) * x871)) + (((12.5791903295117) * x863 * x867)) + (((-2.49929320780212) * x881)) + (((-1.0) * sj1 * x878)) + (((-1.0) * x873 * x879)) + (((-4.10070521755005e-5) * x867 * x877)) + (((-12.5791903295117) * x869 * x877)) + ((x861 * x882)) + (((-10.2924485181268) * sj0)) + ((x865 * x866)) + ((x865 * x872)) + (((-1.0) * x862 * x882)))), IKFAST_ATAN2_MAGTHRESH);
                                    if (!x883.valid)
                                    {
                                        continue;
                                    }
                                    CheckValue<IkReal> x884 = IKPowWithIntegerCheck(IKsign(((((50.0951015765574) * cj0)) + (((9.96185595330477) * sj0)))), -1);
                                    if (!x884.valid)
                                    {
                                        continue;
                                    }
                                    j2array[0] = ((-1.5707963267949) + (x883.value) + (((1.5707963267949) * (x884.value))));
                                    sj2array[0] = IKsin(j2array[0]);
                                    cj2array[0] = IKcos(j2array[0]);
                                    if (j2array[0] > IKPI)
                                    {
                                        j2array[0] -= IK2PI;
                                    }
                                    else if (j2array[0] < -IKPI)
                                    {
                                        j2array[0] += IK2PI;
                                    }
                                    j2valid[0] = true;
                                    for (int ij2 = 0; ij2 < 1; ++ij2)
                                    {
                                        if (!j2valid[ij2])
                                        {
                                            continue;
                                        }
                                        _ij2[0] = ij2;
                                        _ij2[1] = -1;
                                        for (int iij2 = ij2 + 1; iij2 < 1; ++iij2)
                                        {
                                            if (j2valid[iij2] && IKabs(cj2array[ij2] - cj2array[iij2]) < IKFAST_SOLUTION_THRESH && IKabs(sj2array[ij2] - sj2array[iij2]) < IKFAST_SOLUTION_THRESH)
                                            {
                                                j2valid[iij2] = false;
                                                _ij2[1] = iij2;
                                                break;
                                            }
                                        }
                                        j2 = j2array[ij2];
                                        cj2 = cj2array[ij2];
                                        sj2 = sj2array[ij2];
                                        {
                                            IkReal evalcond[3];
                                            IkReal x885 = IKcos(j2);
                                            IkReal x886 = IKsin(j2);
                                            IkReal x887 = ((3.7e-7) * sj5);
                                            IkReal x888 = (cj0 * sj1);
                                            IkReal x889 = ((3.7e-7) * cj5);
                                            IkReal x890 = ((0.1135) * cj5);
                                            IkReal x891 = ((0.1135) * sj5);
                                            IkReal x892 = (sj0 * sj1);
                                            IkReal x893 = ((0.0881580172858829) * x885);
                                            IkReal x894 = ((0.443319482978384) * x885);
                                            IkReal x895 = ((0.443319482978384) * cj1 * x886);
                                            IkReal x896 = ((0.0881580172858829) * cj1 * x886);
                                            evalcond[0] = ((((0.467) * cj1)) + ((r20 * x887)) + (((0.452) * sj1 * x886)) + (((-1.0) * pz)) + ((r21 * x891)) + ((r21 * x889)) + (((0.452) * cj1 * x885)) + (((-1.0) * r20 * x890)));
                                            evalcond[1] = ((((0.458031412723242) * x892)) + ((cj0 * x896)) + (((0.0221176390070984) * sj0)) + ((r00 * x887)) + ((r01 * x891)) + (((0.111222163942722) * cj0)) + ((r01 * x889)) + (((-1.0) * sj0 * x895)) + (((-1.0) * px)) + ((x892 * x894)) + (((-1.0) * x888 * x893)) + (((-0.0910836152046622) * x888)) + (((-1.0) * r00 * x890)));
                                            evalcond[2] = ((((-1.0) * x892 * x893)) + ((r11 * x889)) + ((sj0 * x896)) + ((r11 * x891)) + ((cj0 * x895)) + (((-0.458031412723242) * x888)) + ((r10 * x887)) + (((0.111222163942722) * sj0)) + (((-1.0) * py)) + (((-1.0) * r10 * x890)) + (((-0.0221176390070984) * cj0)) + (((-1.0) * x888 * x894)) + (((-0.0910836152046622) * x892)));
                                            if (IKabs(evalcond[0]) > IKFAST_EVALCOND_THRESH || IKabs(evalcond[1]) > IKFAST_EVALCOND_THRESH || IKabs(evalcond[2]) > IKFAST_EVALCOND_THRESH)
                                            {
                                                continue;
                                            }
                                        }

                                        {
                                            IkReal j3eval[2];
                                            IkReal x897 = cj4 * cj4;
                                            IkReal x898 = cj5 * cj5;
                                            IkReal x899 = r22 * r22;
                                            IkReal x900 = r21 * r21;
                                            IkReal x901 = r20 * r20;
                                            IkReal x902 = (r20 * sj5);
                                            IkReal x903 = (cj5 * r21);
                                            IkReal x904 = ((1.0) * x900);
                                            IkReal x905 = ((1.0) * x901);
                                            IkReal x906 = (x897 * x898);
                                            IkReal x907 = ((2.0) * cj4 * r22 * sj4);
                                            IkReal x908 = ((((-1.0) * x903 * x907)) + (((2.0) * x902 * x903)) + (((-1.0) * x897 * x905)) + (((-1.0) * x902 * x907)) + ((x901 * x906)) + (((-1.0) * x904)) + (((-1.0) * x898 * x905)) + (((-1.0) * x899)) + ((x897 * x899)) + (((-2.0) * x897 * x902 * x903)) + (((-1.0) * x904 * x906)) + ((x898 * x900)));
                                            j3eval[0] = x908;
                                            j3eval[1] = IKsign(x908);
                                            if (IKabs(j3eval[0]) < 0.0000010000000000 || IKabs(j3eval[1]) < 0.0000010000000000)
                                            {
                                                {
                                                    IkReal j3eval[2];
                                                    IkReal x909 = (((cj5 * r11 * sj4)) + ((r10 * sj4 * sj5)) + (((-1.0) * cj4 * r12)));
                                                    j3eval[0] = x909;
                                                    j3eval[1] = IKsign(x909);
                                                    if (IKabs(j3eval[0]) < 0.0000010000000000 || IKabs(j3eval[1]) < 0.0000010000000000)
                                                    {
                                                        {
                                                            IkReal j3eval[2];
                                                            IkReal x910 = ((1.0) * sj4);
                                                            IkReal x911 = ((((-1.0) * cj5 * r01 * x910)) + (((-1.0) * r00 * sj5 * x910)) + ((cj4 * r02)));
                                                            j3eval[0] = x911;
                                                            j3eval[1] = IKsign(x911);
                                                            if (IKabs(j3eval[0]) < 0.0000010000000000 || IKabs(j3eval[1]) < 0.0000010000000000)
                                                            {
                                                                continue; // no branches [j3]
                                                            }
                                                            else
                                                            {
                                                                {
                                                                    IkReal j3array[1], cj3array[1], sj3array[1];
                                                                    bool j3valid[1] = {false};
                                                                    _nj3 = 1;
                                                                    IkReal x912 = ((0.195039861251953) * sj0);
                                                                    IkReal x913 = (cj1 * cj2);
                                                                    IkReal x914 = (cj5 * r20);
                                                                    IkReal x915 = (r22 * sj4);
                                                                    IkReal x916 = (cj1 * sj2);
                                                                    IkReal x917 = (cj4 * cj5);
                                                                    IkReal x918 = (r12 * sj4);
                                                                    IkReal x919 = ((0.980795316323859) * cj0);
                                                                    IkReal x920 = ((1.0) * sj5);
                                                                    IkReal x921 = (sj1 * sj2);
                                                                    IkReal x922 = (cj2 * sj1);
                                                                    IkReal x923 = (r21 * sj5);
                                                                    IkReal x924 = ((1.0) * cj5);
                                                                    IkReal x925 = (cj4 * r10);
                                                                    IkReal x926 = (r21 * x921);
                                                                    IkReal x927 = (cj4 * r20 * sj5);
                                                                    CheckValue<IkReal> x928 = IKatan2WithCheck(IkReal((((x913 * x915 * x919)) + ((r21 * x912 * x913 * x917)) + (((-1.0) * x920 * x922 * x925)) + (((-1.0) * r11 * x917 * x922)) + ((x912 * x915 * x921)) + (((-1.0) * x918 * x922)) + ((x913 * x919 * x927)) + ((x915 * x919 * x921)) + ((x912 * x913 * x915)) + ((r11 * x916 * x917)) + ((x919 * x921 * x927)) + ((x912 * x917 * x926)) + ((x917 * x919 * x926)) + ((x916 * x918)) + ((sj5 * x916 * x925)) + ((r21 * x913 * x917 * x919)) + ((x912 * x921 * x927)) + ((x912 * x913 * x927)))), IkReal((((r11 * sj5 * x916)) + (((-1.0) * x912 * x914 * x921)) + ((x913 * x919 * x923)) + (((-1.0) * x912 * x913 * x914)) + (((-1.0) * x914 * x919 * x921)) + ((x919 * x921 * x923)) + ((cj5 * r10 * x922)) + ((x912 * x921 * x923)) + ((x912 * x913 * x923)) + (((-1.0) * x913 * x914 * x919)) + (((-1.0) * r10 * x916 * x924)) + (((-1.0) * r11 * x920 * x922)))), IKFAST_ATAN2_MAGTHRESH);
                                                                    if (!x928.valid)
                                                                    {
                                                                        continue;
                                                                    }
                                                                    CheckValue<IkReal> x929 = IKPowWithIntegerCheck(IKsign(((((-1.0) * r01 * sj4 * x924)) + (((-1.0) * r00 * sj4 * x920)) + ((cj4 * r02)))), -1);
                                                                    if (!x929.valid)
                                                                    {
                                                                        continue;
                                                                    }
                                                                    j3array[0] = ((-1.5707963267949) + (x928.value) + (((1.5707963267949) * (x929.value))));
                                                                    sj3array[0] = IKsin(j3array[0]);
                                                                    cj3array[0] = IKcos(j3array[0]);
                                                                    if (j3array[0] > IKPI)
                                                                    {
                                                                        j3array[0] -= IK2PI;
                                                                    }
                                                                    else if (j3array[0] < -IKPI)
                                                                    {
                                                                        j3array[0] += IK2PI;
                                                                    }
                                                                    j3valid[0] = true;
                                                                    for (int ij3 = 0; ij3 < 1; ++ij3)
                                                                    {
                                                                        if (!j3valid[ij3])
                                                                        {
                                                                            continue;
                                                                        }
                                                                        _ij3[0] = ij3;
                                                                        _ij3[1] = -1;
                                                                        for (int iij3 = ij3 + 1; iij3 < 1; ++iij3)
                                                                        {
                                                                            if (j3valid[iij3] && IKabs(cj3array[ij3] - cj3array[iij3]) < IKFAST_SOLUTION_THRESH && IKabs(sj3array[ij3] - sj3array[iij3]) < IKFAST_SOLUTION_THRESH)
                                                                            {
                                                                                j3valid[iij3] = false;
                                                                                _ij3[1] = iij3;
                                                                                break;
                                                                            }
                                                                        }
                                                                        j3 = j3array[ij3];
                                                                        cj3 = cj3array[ij3];
                                                                        sj3 = sj3array[ij3];
                                                                        {
                                                                            IkReal evalcond[6];
                                                                            IkReal x930 = IKcos(j3);
                                                                            IkReal x931 = IKsin(j3);
                                                                            IkReal x932 = ((0.980795316323859) * cj0);
                                                                            IkReal x933 = (sj1 * sj2);
                                                                            IkReal x934 = (cj1 * cj2);
                                                                            IkReal x935 = ((0.980795316323859) * sj0);
                                                                            IkReal x936 = (r00 * sj5);
                                                                            IkReal x937 = (cj5 * r11);
                                                                            IkReal x938 = (r10 * sj5);
                                                                            IkReal x939 = (r02 * sj4);
                                                                            IkReal x940 = (cj5 * r01);
                                                                            IkReal x941 = (r12 * sj4);
                                                                            IkReal x942 = (cj2 * sj1);
                                                                            IkReal x943 = ((0.195039861251953) * sj0);
                                                                            IkReal x944 = (r20 * sj5);
                                                                            IkReal x945 = (r22 * sj4);
                                                                            IkReal x946 = (r21 * sj5);
                                                                            IkReal x947 = (r01 * sj5);
                                                                            IkReal x948 = ((0.195039861251953) * cj0);
                                                                            IkReal x949 = (cj5 * r21);
                                                                            IkReal x950 = (cj1 * sj2);
                                                                            IkReal x951 = ((1.0) * r11 * sj5);
                                                                            IkReal x952 = (cj5 * x931);
                                                                            IkReal x953 = ((1.0) * x931);
                                                                            IkReal x954 = (cj4 * x930);
                                                                            IkReal x955 = (cj5 * x930);
                                                                            IkReal x956 = ((1.0) * x930);
                                                                            IkReal x957 = (cj4 * x953);
                                                                            evalcond[0] = (((x944 * x954)) + ((x949 * x954)) + ((x930 * x945)) + (((-1.0) * x942)) + (((-1.0) * x946 * x953)) + ((r20 * x952)) + x950);
                                                                            evalcond[1] = ((((-1.0) * x944 * x957)) + (((-1.0) * x946 * x956)) + ((r20 * x955)) + (((-1.0) * x949 * x957)) + (((-1.0) * x945 * x953)) + x934 + x933);
                                                                            evalcond[2] = (((x940 * x954)) + (((-1.0) * x933 * x948)) + ((x930 * x939)) + (((-1.0) * x947 * x953)) + ((r00 * x952)) + ((x934 * x935)) + ((x936 * x954)) + ((x933 * x935)) + (((-1.0) * x934 * x948)));
                                                                            evalcond[3] = (((x930 * x941)) + (((-1.0) * x933 * x943)) + (((-1.0) * x932 * x933)) + (((-1.0) * x932 * x934)) + ((x938 * x954)) + (((-1.0) * x931 * x951)) + ((r10 * x952)) + ((x937 * x954)) + (((-1.0) * x934 * x943)));
                                                                            evalcond[4] = ((((-1.0) * x940 * x957)) + (((-1.0) * x942 * x948)) + ((x948 * x950)) + (((-1.0) * x939 * x953)) + (((-1.0) * x947 * x956)) + ((r00 * x955)) + (((-1.0) * x935 * x950)) + ((x935 * x942)) + (((-1.0) * x936 * x957)));
                                                                            evalcond[5] = (((x932 * x950)) + (((-1.0) * x938 * x957)) + (((-1.0) * x942 * x943)) + ((x943 * x950)) + (((-1.0) * x941 * x953)) + (((-1.0) * x932 * x942)) + (((-1.0) * x930 * x951)) + (((-1.0) * x937 * x957)) + ((r10 * x955)));
                                                                            if (IKabs(evalcond[0]) > IKFAST_EVALCOND_THRESH || IKabs(evalcond[1]) > IKFAST_EVALCOND_THRESH || IKabs(evalcond[2]) > IKFAST_EVALCOND_THRESH || IKabs(evalcond[3]) > IKFAST_EVALCOND_THRESH || IKabs(evalcond[4]) > IKFAST_EVALCOND_THRESH || IKabs(evalcond[5]) > IKFAST_EVALCOND_THRESH)
                                                                            {
                                                                                continue;
                                                                            }
                                                                        }

                                                                        {
                                                                            std::vector<IkSingleDOFSolutionBase<IkReal>> vinfos(6);
                                                                            vinfos[0].jointtype = 1;
                                                                            vinfos[0].foffset = j0;
                                                                            vinfos[0].indices[0] = _ij0[0];
                                                                            vinfos[0].indices[1] = _ij0[1];
                                                                            vinfos[0].maxsolutions = _nj0;
                                                                            vinfos[1].jointtype = 1;
                                                                            vinfos[1].foffset = j1;
                                                                            vinfos[1].indices[0] = _ij1[0];
                                                                            vinfos[1].indices[1] = _ij1[1];
                                                                            vinfos[1].maxsolutions = _nj1;
                                                                            vinfos[2].jointtype = 1;
                                                                            vinfos[2].foffset = j2;
                                                                            vinfos[2].indices[0] = _ij2[0];
                                                                            vinfos[2].indices[1] = _ij2[1];
                                                                            vinfos[2].maxsolutions = _nj2;
                                                                            vinfos[3].jointtype = 1;
                                                                            vinfos[3].foffset = j3;
                                                                            vinfos[3].indices[0] = _ij3[0];
                                                                            vinfos[3].indices[1] = _ij3[1];
                                                                            vinfos[3].maxsolutions = _nj3;
                                                                            vinfos[4].jointtype = 1;
                                                                            vinfos[4].foffset = j4;
                                                                            vinfos[4].indices[0] = _ij4[0];
                                                                            vinfos[4].indices[1] = _ij4[1];
                                                                            vinfos[4].maxsolutions = _nj4;
                                                                            vinfos[5].jointtype = 1;
                                                                            vinfos[5].foffset = j5;
                                                                            vinfos[5].indices[0] = _ij5[0];
                                                                            vinfos[5].indices[1] = _ij5[1];
                                                                            vinfos[5].maxsolutions = _nj5;
                                                                            std::vector<int> vfree(0);
                                                                            solutions.AddSolution(vinfos, vfree);
                                                                        }
                                                                    }
                                                                }
                                                            }
                                                        }
                                                    }
                                                    else
                                                    {
                                                        {
                                                            IkReal j3array[1], cj3array[1], sj3array[1];
                                                            bool j3valid[1] = {false};
                                                            _nj3 = 1;
                                                            IkReal x958 = (cj1 * sj2);
                                                            IkReal x959 = (cj4 * cj5);
                                                            IkReal x960 = (sj1 * sj5);
                                                            IkReal x961 = ((1.0) * r01);
                                                            IkReal x962 = ((0.980795316323859) * sj0);
                                                            IkReal x963 = (r22 * sj4);
                                                            IkReal x964 = (r02 * sj4);
                                                            IkReal x965 = (cj1 * cj2);
                                                            IkReal x966 = (cj2 * r21);
                                                            IkReal x967 = (sj1 * sj2);
                                                            IkReal x968 = ((0.195039861251953) * cj0);
                                                            IkReal x969 = (cj5 * r20);
                                                            IkReal x970 = (cj4 * r00);
                                                            IkReal x971 = (cj2 * sj1);
                                                            IkReal x972 = (cj5 * r00);
                                                            IkReal x973 = (r21 * x968);
                                                            IkReal x974 = (cj2 * cj4 * r20);
                                                            IkReal x975 = (cj4 * r20 * sj5);
                                                            CheckValue<IkReal> x976 = IKatan2WithCheck(IkReal((((x958 * x962 * x969)) + ((sj5 * x958 * x973)) + (((-1.0) * x960 * x966 * x968)) + ((x960 * x962 * x966)) + (((-1.0) * sj5 * x961 * x965)) + ((x965 * x972)) + (((-1.0) * x962 * x969 * x971)) + ((x967 * x972)) + (((-1.0) * sj2 * x960 * x961)) + (((-1.0) * r21 * sj5 * x958 * x962)) + ((x968 * x969 * x971)) + (((-1.0) * x958 * x968 * x969)))), IkReal(((((-1.0) * x958 * x968 * x975)) + ((x958 * x962 * x963)) + ((sj5 * x965 * x970)) + ((x958 * x962 * x975)) + (((-1.0) * x958 * x963 * x968)) + ((sj2 * x960 * x970)) + ((r01 * x959 * x965)) + ((r01 * x959 * x967)) + ((x963 * x968 * x971)) + (((-1.0) * x958 * x959 * x973)) + (((-1.0) * x960 * x962 * x974)) + ((x964 * x967)) + ((x964 * x965)) + (((-1.0) * x962 * x963 * x971)) + ((sj1 * x959 * x966 * x968)) + (((-1.0) * sj1 * x959 * x962 * x966)) + ((x960 * x968 * x974)) + ((r21 * x958 * x959 * x962)))), IKFAST_ATAN2_MAGTHRESH);
                                                            if (!x976.valid)
                                                            {
                                                                continue;
                                                            }
                                                            CheckValue<IkReal> x977 = IKPowWithIntegerCheck(IKsign((((cj5 * r11 * sj4)) + ((r10 * sj4 * sj5)) + (((-1.0) * cj4 * r12)))), -1);
                                                            if (!x977.valid)
                                                            {
                                                                continue;
                                                            }
                                                            j3array[0] = ((-1.5707963267949) + (x976.value) + (((1.5707963267949) * (x977.value))));
                                                            sj3array[0] = IKsin(j3array[0]);
                                                            cj3array[0] = IKcos(j3array[0]);
                                                            if (j3array[0] > IKPI)
                                                            {
                                                                j3array[0] -= IK2PI;
                                                            }
                                                            else if (j3array[0] < -IKPI)
                                                            {
                                                                j3array[0] += IK2PI;
                                                            }
                                                            j3valid[0] = true;
                                                            for (int ij3 = 0; ij3 < 1; ++ij3)
                                                            {
                                                                if (!j3valid[ij3])
                                                                {
                                                                    continue;
                                                                }
                                                                _ij3[0] = ij3;
                                                                _ij3[1] = -1;
                                                                for (int iij3 = ij3 + 1; iij3 < 1; ++iij3)
                                                                {
                                                                    if (j3valid[iij3] && IKabs(cj3array[ij3] - cj3array[iij3]) < IKFAST_SOLUTION_THRESH && IKabs(sj3array[ij3] - sj3array[iij3]) < IKFAST_SOLUTION_THRESH)
                                                                    {
                                                                        j3valid[iij3] = false;
                                                                        _ij3[1] = iij3;
                                                                        break;
                                                                    }
                                                                }
                                                                j3 = j3array[ij3];
                                                                cj3 = cj3array[ij3];
                                                                sj3 = sj3array[ij3];
                                                                {
                                                                    IkReal evalcond[6];
                                                                    IkReal x978 = IKcos(j3);
                                                                    IkReal x979 = IKsin(j3);
                                                                    IkReal x980 = ((0.980795316323859) * cj0);
                                                                    IkReal x981 = (sj1 * sj2);
                                                                    IkReal x982 = (cj1 * cj2);
                                                                    IkReal x983 = ((0.980795316323859) * sj0);
                                                                    IkReal x984 = (r00 * sj5);
                                                                    IkReal x985 = (cj5 * r11);
                                                                    IkReal x986 = (r10 * sj5);
                                                                    IkReal x987 = (r02 * sj4);
                                                                    IkReal x988 = (cj5 * r01);
                                                                    IkReal x989 = (r12 * sj4);
                                                                    IkReal x990 = (cj2 * sj1);
                                                                    IkReal x991 = ((0.195039861251953) * sj0);
                                                                    IkReal x992 = (r20 * sj5);
                                                                    IkReal x993 = (r22 * sj4);
                                                                    IkReal x994 = (r21 * sj5);
                                                                    IkReal x995 = (r01 * sj5);
                                                                    IkReal x996 = ((0.195039861251953) * cj0);
                                                                    IkReal x997 = (cj5 * r21);
                                                                    IkReal x998 = (cj1 * sj2);
                                                                    IkReal x999 = ((1.0) * r11 * sj5);
                                                                    IkReal x1000 = (cj5 * x979);
                                                                    IkReal x1001 = ((1.0) * x979);
                                                                    IkReal x1002 = (cj4 * x978);
                                                                    IkReal x1003 = (cj5 * x978);
                                                                    IkReal x1004 = ((1.0) * x978);
                                                                    IkReal x1005 = (cj4 * x1001);
                                                                    evalcond[0] = ((((-1.0) * x990)) + ((x978 * x993)) + (((-1.0) * x1001 * x994)) + ((x1002 * x997)) + ((x1002 * x992)) + ((r20 * x1000)) + x998);
                                                                    evalcond[1] = ((((-1.0) * x1001 * x993)) + ((r20 * x1003)) + (((-1.0) * x1004 * x994)) + (((-1.0) * x1005 * x997)) + (((-1.0) * x1005 * x992)) + x981 + x982);
                                                                    evalcond[2] = (((x981 * x983)) + ((x1002 * x988)) + ((x1002 * x984)) + (((-1.0) * x982 * x996)) + ((x982 * x983)) + (((-1.0) * x1001 * x995)) + (((-1.0) * x981 * x996)) + ((x978 * x987)) + ((r00 * x1000)));
                                                                    evalcond[3] = (((x1002 * x986)) + ((x1002 * x985)) + (((-1.0) * x982 * x991)) + (((-1.0) * x980 * x982)) + (((-1.0) * x980 * x981)) + (((-1.0) * x979 * x999)) + (((-1.0) * x981 * x991)) + ((r10 * x1000)) + ((x978 * x989)));
                                                                    evalcond[4] = (((x983 * x990)) + (((-1.0) * x990 * x996)) + (((-1.0) * x1005 * x984)) + (((-1.0) * x1005 * x988)) + (((-1.0) * x1001 * x987)) + (((-1.0) * x983 * x998)) + (((-1.0) * x1004 * x995)) + ((x996 * x998)) + ((r00 * x1003)));
                                                                    evalcond[5] = (((x991 * x998)) + (((-1.0) * x990 * x991)) + (((-1.0) * x978 * x999)) + (((-1.0) * x1005 * x985)) + (((-1.0) * x1005 * x986)) + ((x980 * x998)) + (((-1.0) * x1001 * x989)) + (((-1.0) * x980 * x990)) + ((r10 * x1003)));
                                                                    if (IKabs(evalcond[0]) > IKFAST_EVALCOND_THRESH || IKabs(evalcond[1]) > IKFAST_EVALCOND_THRESH || IKabs(evalcond[2]) > IKFAST_EVALCOND_THRESH || IKabs(evalcond[3]) > IKFAST_EVALCOND_THRESH || IKabs(evalcond[4]) > IKFAST_EVALCOND_THRESH || IKabs(evalcond[5]) > IKFAST_EVALCOND_THRESH)
                                                                    {
                                                                        continue;
                                                                    }
                                                                }

                                                                {
                                                                    std::vector<IkSingleDOFSolutionBase<IkReal>> vinfos(6);
                                                                    vinfos[0].jointtype = 1;
                                                                    vinfos[0].foffset = j0;
                                                                    vinfos[0].indices[0] = _ij0[0];
                                                                    vinfos[0].indices[1] = _ij0[1];
                                                                    vinfos[0].maxsolutions = _nj0;
                                                                    vinfos[1].jointtype = 1;
                                                                    vinfos[1].foffset = j1;
                                                                    vinfos[1].indices[0] = _ij1[0];
                                                                    vinfos[1].indices[1] = _ij1[1];
                                                                    vinfos[1].maxsolutions = _nj1;
                                                                    vinfos[2].jointtype = 1;
                                                                    vinfos[2].foffset = j2;
                                                                    vinfos[2].indices[0] = _ij2[0];
                                                                    vinfos[2].indices[1] = _ij2[1];
                                                                    vinfos[2].maxsolutions = _nj2;
                                                                    vinfos[3].jointtype = 1;
                                                                    vinfos[3].foffset = j3;
                                                                    vinfos[3].indices[0] = _ij3[0];
                                                                    vinfos[3].indices[1] = _ij3[1];
                                                                    vinfos[3].maxsolutions = _nj3;
                                                                    vinfos[4].jointtype = 1;
                                                                    vinfos[4].foffset = j4;
                                                                    vinfos[4].indices[0] = _ij4[0];
                                                                    vinfos[4].indices[1] = _ij4[1];
                                                                    vinfos[4].maxsolutions = _nj4;
                                                                    vinfos[5].jointtype = 1;
                                                                    vinfos[5].foffset = j5;
                                                                    vinfos[5].indices[0] = _ij5[0];
                                                                    vinfos[5].indices[1] = _ij5[1];
                                                                    vinfos[5].maxsolutions = _nj5;
                                                                    std::vector<int> vfree(0);
                                                                    solutions.AddSolution(vinfos, vfree);
                                                                }
                                                            }
                                                        }
                                                    }
                                                }
                                            }
                                            else
                                            {
                                                {
                                                    IkReal j3array[1], cj3array[1], sj3array[1];
                                                    bool j3valid[1] = {false};
                                                    _nj3 = 1;
                                                    IkReal x1006 = cj4 * cj4;
                                                    IkReal x1007 = cj5 * cj5;
                                                    IkReal x1008 = r22 * r22;
                                                    IkReal x1009 = r21 * r21;
                                                    IkReal x1010 = r20 * r20;
                                                    IkReal x1011 = ((1.0) * sj1);
                                                    IkReal x1012 = (r22 * sj4);
                                                    IkReal x1013 = (sj2 * sj5);
                                                    IkReal x1014 = (cj4 * r20);
                                                    IkReal x1015 = (cj5 * sj2);
                                                    IkReal x1016 = (r21 * sj5);
                                                    IkReal x1017 = (cj4 * r21);
                                                    IkReal x1018 = ((2.0) * cj5);
                                                    IkReal x1019 = (cj2 * cj5 * r20);
                                                    IkReal x1020 = ((1.0) * x1009);
                                                    IkReal x1021 = ((1.0) * cj1 * cj2);
                                                    IkReal x1022 = ((1.0) * x1010);
                                                    IkReal x1023 = (x1006 * x1007);
                                                    CheckValue<IkReal> x1024 = IKPowWithIntegerCheck(IKsign((((x1010 * x1023)) + (((-1.0) * x1008)) + (((-2.0) * sj5 * x1012 * x1014)) + (((-1.0) * x1020)) + (((-1.0) * x1020 * x1023)) + (((-1.0) * r20 * x1006 * x1016 * x1018)) + ((r20 * x1016 * x1018)) + ((x1006 * x1008)) + ((x1007 * x1009)) + (((-1.0) * x1007 * x1022)) + (((-1.0) * x1012 * x1017 * x1018)) + (((-1.0) * x1006 * x1022)))), -1);
                                                    if (!x1024.valid)
                                                    {
                                                        continue;
                                                    }
                                                    CheckValue<IkReal> x1025 = IKatan2WithCheck(IkReal((((cj1 * r20 * x1015)) + (((-1.0) * x1011 * x1019)) + (((-1.0) * sj5 * x1014 * x1021)) + (((-1.0) * cj5 * x1017 * x1021)) + (((-1.0) * sj2 * x1011 * x1012)) + (((-1.0) * cj1 * r21 * x1013)) + ((cj2 * sj1 * x1016)) + (((-1.0) * x1011 * x1013 * x1014)) + (((-1.0) * x1011 * x1015 * x1017)) + (((-1.0) * x1012 * x1021)))), IkReal(((((-1.0) * cj2 * x1011 * x1012)) + (((-1.0) * r21 * x1011 * x1013)) + ((cj1 * x1013 * x1014)) + (((-1.0) * cj2 * sj5 * x1011 * x1014)) + (((-1.0) * x1016 * x1021)) + ((cj1 * x1019)) + (((-1.0) * cj2 * cj5 * x1011 * x1017)) + ((cj1 * x1015 * x1017)) + ((r20 * sj1 * x1015)) + ((cj1 * sj2 * x1012)))), IKFAST_ATAN2_MAGTHRESH);
                                                    if (!x1025.valid)
                                                    {
                                                        continue;
                                                    }
                                                    j3array[0] = ((-1.5707963267949) + (((1.5707963267949) * (x1024.value))) + (x1025.value));
                                                    sj3array[0] = IKsin(j3array[0]);
                                                    cj3array[0] = IKcos(j3array[0]);
                                                    if (j3array[0] > IKPI)
                                                    {
                                                        j3array[0] -= IK2PI;
                                                    }
                                                    else if (j3array[0] < -IKPI)
                                                    {
                                                        j3array[0] += IK2PI;
                                                    }
                                                    j3valid[0] = true;
                                                    for (int ij3 = 0; ij3 < 1; ++ij3)
                                                    {
                                                        if (!j3valid[ij3])
                                                        {
                                                            continue;
                                                        }
                                                        _ij3[0] = ij3;
                                                        _ij3[1] = -1;
                                                        for (int iij3 = ij3 + 1; iij3 < 1; ++iij3)
                                                        {
                                                            if (j3valid[iij3] && IKabs(cj3array[ij3] - cj3array[iij3]) < IKFAST_SOLUTION_THRESH && IKabs(sj3array[ij3] - sj3array[iij3]) < IKFAST_SOLUTION_THRESH)
                                                            {
                                                                j3valid[iij3] = false;
                                                                _ij3[1] = iij3;
                                                                break;
                                                            }
                                                        }
                                                        j3 = j3array[ij3];
                                                        cj3 = cj3array[ij3];
                                                        sj3 = sj3array[ij3];
                                                        {
                                                            IkReal evalcond[6];
                                                            IkReal x1026 = IKcos(j3);
                                                            IkReal x1027 = IKsin(j3);
                                                            IkReal x1028 = ((0.980795316323859) * cj0);
                                                            IkReal x1029 = (sj1 * sj2);
                                                            IkReal x1030 = (cj1 * cj2);
                                                            IkReal x1031 = ((0.980795316323859) * sj0);
                                                            IkReal x1032 = (r00 * sj5);
                                                            IkReal x1033 = (cj5 * r11);
                                                            IkReal x1034 = (r10 * sj5);
                                                            IkReal x1035 = (r02 * sj4);
                                                            IkReal x1036 = (cj5 * r01);
                                                            IkReal x1037 = (r12 * sj4);
                                                            IkReal x1038 = (cj2 * sj1);
                                                            IkReal x1039 = ((0.195039861251953) * sj0);
                                                            IkReal x1040 = (r20 * sj5);
                                                            IkReal x1041 = (r22 * sj4);
                                                            IkReal x1042 = (r21 * sj5);
                                                            IkReal x1043 = (r01 * sj5);
                                                            IkReal x1044 = ((0.195039861251953) * cj0);
                                                            IkReal x1045 = (cj5 * r21);
                                                            IkReal x1046 = (cj1 * sj2);
                                                            IkReal x1047 = ((1.0) * r11 * sj5);
                                                            IkReal x1048 = (cj5 * x1027);
                                                            IkReal x1049 = ((1.0) * x1027);
                                                            IkReal x1050 = (cj4 * x1026);
                                                            IkReal x1051 = (cj5 * x1026);
                                                            IkReal x1052 = ((1.0) * x1026);
                                                            IkReal x1053 = (cj4 * x1049);
                                                            evalcond[0] = (((x1026 * x1041)) + x1046 + (((-1.0) * x1038)) + ((x1040 * x1050)) + ((r20 * x1048)) + (((-1.0) * x1042 * x1049)) + ((x1045 * x1050)));
                                                            evalcond[1] = (x1030 + x1029 + (((-1.0) * x1045 * x1053)) + (((-1.0) * x1041 * x1049)) + (((-1.0) * x1042 * x1052)) + ((r20 * x1051)) + (((-1.0) * x1040 * x1053)));
                                                            evalcond[2] = (((x1030 * x1031)) + ((x1026 * x1035)) + (((-1.0) * x1029 * x1044)) + (((-1.0) * x1043 * x1049)) + ((x1029 * x1031)) + ((r00 * x1048)) + ((x1036 * x1050)) + ((x1032 * x1050)) + (((-1.0) * x1030 * x1044)));
                                                            evalcond[3] = ((((-1.0) * x1028 * x1029)) + (((-1.0) * x1028 * x1030)) + ((x1026 * x1037)) + ((x1034 * x1050)) + (((-1.0) * x1029 * x1039)) + (((-1.0) * x1027 * x1047)) + ((r10 * x1048)) + (((-1.0) * x1030 * x1039)) + ((x1033 * x1050)));
                                                            evalcond[4] = (((x1031 * x1038)) + (((-1.0) * x1031 * x1046)) + (((-1.0) * x1038 * x1044)) + (((-1.0) * x1043 * x1052)) + ((r00 * x1051)) + (((-1.0) * x1035 * x1049)) + (((-1.0) * x1036 * x1053)) + (((-1.0) * x1032 * x1053)) + ((x1044 * x1046)));
                                                            evalcond[5] = ((((-1.0) * x1033 * x1053)) + (((-1.0) * x1028 * x1038)) + (((-1.0) * x1038 * x1039)) + ((x1039 * x1046)) + (((-1.0) * x1037 * x1049)) + ((r10 * x1051)) + ((x1028 * x1046)) + (((-1.0) * x1034 * x1053)) + (((-1.0) * x1026 * x1047)));
                                                            if (IKabs(evalcond[0]) > IKFAST_EVALCOND_THRESH || IKabs(evalcond[1]) > IKFAST_EVALCOND_THRESH || IKabs(evalcond[2]) > IKFAST_EVALCOND_THRESH || IKabs(evalcond[3]) > IKFAST_EVALCOND_THRESH || IKabs(evalcond[4]) > IKFAST_EVALCOND_THRESH || IKabs(evalcond[5]) > IKFAST_EVALCOND_THRESH)
                                                            {
                                                                continue;
                                                            }
                                                        }

                                                        {
                                                            std::vector<IkSingleDOFSolutionBase<IkReal>> vinfos(6);
                                                            vinfos[0].jointtype = 1;
                                                            vinfos[0].foffset = j0;
                                                            vinfos[0].indices[0] = _ij0[0];
                                                            vinfos[0].indices[1] = _ij0[1];
                                                            vinfos[0].maxsolutions = _nj0;
                                                            vinfos[1].jointtype = 1;
                                                            vinfos[1].foffset = j1;
                                                            vinfos[1].indices[0] = _ij1[0];
                                                            vinfos[1].indices[1] = _ij1[1];
                                                            vinfos[1].maxsolutions = _nj1;
                                                            vinfos[2].jointtype = 1;
                                                            vinfos[2].foffset = j2;
                                                            vinfos[2].indices[0] = _ij2[0];
                                                            vinfos[2].indices[1] = _ij2[1];
                                                            vinfos[2].maxsolutions = _nj2;
                                                            vinfos[3].jointtype = 1;
                                                            vinfos[3].foffset = j3;
                                                            vinfos[3].indices[0] = _ij3[0];
                                                            vinfos[3].indices[1] = _ij3[1];
                                                            vinfos[3].maxsolutions = _nj3;
                                                            vinfos[4].jointtype = 1;
                                                            vinfos[4].foffset = j4;
                                                            vinfos[4].indices[0] = _ij4[0];
                                                            vinfos[4].indices[1] = _ij4[1];
                                                            vinfos[4].maxsolutions = _nj4;
                                                            vinfos[5].jointtype = 1;
                                                            vinfos[5].foffset = j5;
                                                            vinfos[5].indices[0] = _ij5[0];
                                                            vinfos[5].indices[1] = _ij5[1];
                                                            vinfos[5].maxsolutions = _nj5;
                                                            std::vector<int> vfree(0);
                                                            solutions.AddSolution(vinfos, vfree);
                                                        }
                                                    }
                                                }
                                            }
                                        }
                                    }
                                }
                            }
                        }
                    }
                    else
                    {
                        {
                            IkReal j2array[1], cj2array[1], sj2array[1];
                            bool j2valid[1] = {false};
                            _nj2 = 1;
                            IkReal x1054 = ((8.15461659894417e-6) * r20);
                            IkReal x1055 = ((2.50148374048693) * r21);
                            IkReal x1056 = (cj1 * r01);
                            IkReal x1057 = ((4.181e-5) * cj5);
                            IkReal x1058 = ((12.8255) * sj5);
                            IkReal x1059 = (cj1 * r00);
                            IkReal x1060 = ((4.181e-5) * sj5);
                            IkReal x1061 = ((12.8255) * cj5);
                            IkReal x1062 = (cj0 * cj1);
                            IkReal x1063 = (cj1 * sj0);
                            IkReal x1064 = (cj5 * r20);
                            IkReal x1065 = ((110.829870744596) * pz);
                            IkReal x1066 = (sj0 * sj1);
                            IkReal x1067 = (r01 * sj1);
                            IkReal x1068 = (cj5 * r21);
                            IkReal x1069 = (cj0 * sj1);
                            IkReal x1070 = (r00 * sj1);
                            IkReal x1071 = ((113.0) * px);
                            IkReal x1072 = (sj5 * x1069);
                            IkReal x1073 = ((22.0395043214707) * cj0 * pz);
                            IkReal x1074 = ((12.5791903295117) * r21 * sj5);
                            IkReal x1075 = ((4.10070521755005e-5) * r20 * sj5);
                            CheckValue<IkReal> x1076 = IKatan2WithCheck(IkReal(((((2.50148374048693) * x1064 * x1069)) + (((-1.0) * x1059 * x1060)) + (((-12.5681045255276) * x1062)) + (((-8.15461659894417e-6) * x1068 * x1069)) + ((x1066 * x1074)) + ((x1066 * x1075)) + ((cj1 * x1071)) + (((-1.0) * x1054 * x1072)) + (((22.0395043214707) * pz * x1069)) + (((-1.0) * x1056 * x1057)) + (((-1.0) * x1056 * x1058)) + (((-1.0) * x1055 * x1072)) + (((4.10070521755005e-5) * x1066 * x1068)) + (((-12.5791903295117) * x1064 * x1066)) + ((x1059 * x1061)) + (((-2.49929320780212) * x1063)) + (((-1.0) * x1065 * x1066)))), IkReal(((((-1.0) * x1063 * x1065)) + (((12.5681045255276) * x1069)) + (((-1.0) * sj5 * x1054 * x1062)) + ((x1060 * x1070)) + (((2.50148374048693) * x1062 * x1064)) + (((-1.0) * sj1 * x1071)) + (((-1.0) * x1061 * x1070)) + ((x1057 * x1067)) + (((-12.5791903295117) * x1063 * x1064)) + (((4.10070521755005e-5) * x1063 * x1068)) + ((x1063 * x1074)) + ((x1063 * x1075)) + (((22.0395043214707) * pz * x1062)) + (((2.49929320780212) * x1066)) + (((-10.2924485181268) * cj0)) + (((-8.15461659894417e-6) * x1062 * x1068)) + ((x1058 * x1067)) + (((51.7575496377264) * sj0)) + (((-1.0) * sj5 * x1055 * x1062)))), IKFAST_ATAN2_MAGTHRESH);
                            if (!x1076.valid)
                            {
                                continue;
                            }
                            CheckValue<IkReal> x1077 = IKPowWithIntegerCheck(IKsign(((((9.96185595330477) * cj0)) + (((-50.0951015765574) * sj0)))), -1);
                            if (!x1077.valid)
                            {
                                continue;
                            }
                            j2array[0] = ((-1.5707963267949) + (x1076.value) + (((1.5707963267949) * (x1077.value))));
                            sj2array[0] = IKsin(j2array[0]);
                            cj2array[0] = IKcos(j2array[0]);
                            if (j2array[0] > IKPI)
                            {
                                j2array[0] -= IK2PI;
                            }
                            else if (j2array[0] < -IKPI)
                            {
                                j2array[0] += IK2PI;
                            }
                            j2valid[0] = true;
                            for (int ij2 = 0; ij2 < 1; ++ij2)
                            {
                                if (!j2valid[ij2])
                                {
                                    continue;
                                }
                                _ij2[0] = ij2;
                                _ij2[1] = -1;
                                for (int iij2 = ij2 + 1; iij2 < 1; ++iij2)
                                {
                                    if (j2valid[iij2] && IKabs(cj2array[ij2] - cj2array[iij2]) < IKFAST_SOLUTION_THRESH && IKabs(sj2array[ij2] - sj2array[iij2]) < IKFAST_SOLUTION_THRESH)
                                    {
                                        j2valid[iij2] = false;
                                        _ij2[1] = iij2;
                                        break;
                                    }
                                }
                                j2 = j2array[ij2];
                                cj2 = cj2array[ij2];
                                sj2 = sj2array[ij2];
                                {
                                    IkReal evalcond[3];
                                    IkReal x1078 = IKcos(j2);
                                    IkReal x1079 = IKsin(j2);
                                    IkReal x1080 = ((3.7e-7) * sj5);
                                    IkReal x1081 = (cj0 * sj1);
                                    IkReal x1082 = ((3.7e-7) * cj5);
                                    IkReal x1083 = ((0.1135) * cj5);
                                    IkReal x1084 = ((0.1135) * sj5);
                                    IkReal x1085 = (sj0 * sj1);
                                    IkReal x1086 = ((0.0881580172858829) * x1078);
                                    IkReal x1087 = ((0.443319482978384) * x1078);
                                    IkReal x1088 = ((0.443319482978384) * cj1 * x1079);
                                    IkReal x1089 = ((0.0881580172858829) * cj1 * x1079);
                                    evalcond[0] = ((((0.452) * sj1 * x1079)) + (((0.467) * cj1)) + ((r20 * x1080)) + (((-1.0) * pz)) + (((0.452) * cj1 * x1078)) + (((-1.0) * r20 * x1083)) + ((r21 * x1082)) + ((r21 * x1084)));
                                    evalcond[1] = ((((-0.0910836152046622) * x1081)) + (((0.458031412723242) * x1085)) + (((0.0221176390070984) * sj0)) + ((r00 * x1080)) + (((0.111222163942722) * cj0)) + ((r01 * x1082)) + ((r01 * x1084)) + (((-1.0) * x1081 * x1086)) + (((-1.0) * r00 * x1083)) + (((-1.0) * px)) + (((-1.0) * sj0 * x1088)) + ((x1085 * x1087)) + ((cj0 * x1089)));
                                    evalcond[2] = ((((-0.0910836152046622) * x1085)) + (((-1.0) * r10 * x1083)) + (((-0.458031412723242) * x1081)) + ((r10 * x1080)) + ((r11 * x1084)) + ((r11 * x1082)) + (((-1.0) * x1081 * x1087)) + (((0.111222163942722) * sj0)) + (((-1.0) * py)) + (((-0.0221176390070984) * cj0)) + (((-1.0) * x1085 * x1086)) + ((sj0 * x1089)) + ((cj0 * x1088)));
                                    if (IKabs(evalcond[0]) > IKFAST_EVALCOND_THRESH || IKabs(evalcond[1]) > IKFAST_EVALCOND_THRESH || IKabs(evalcond[2]) > IKFAST_EVALCOND_THRESH)
                                    {
                                        continue;
                                    }
                                }

                                {
                                    IkReal j3eval[2];
                                    IkReal x1090 = cj4 * cj4;
                                    IkReal x1091 = cj5 * cj5;
                                    IkReal x1092 = r22 * r22;
                                    IkReal x1093 = r21 * r21;
                                    IkReal x1094 = r20 * r20;
                                    IkReal x1095 = (r20 * sj5);
                                    IkReal x1096 = (cj5 * r21);
                                    IkReal x1097 = ((1.0) * x1093);
                                    IkReal x1098 = ((1.0) * x1094);
                                    IkReal x1099 = (x1090 * x1091);
                                    IkReal x1100 = ((2.0) * cj4 * r22 * sj4);
                                    IkReal x1101 = ((((2.0) * x1095 * x1096)) + (((-1.0) * x1091 * x1098)) + (((-1.0) * x1090 * x1098)) + (((-1.0) * x1097 * x1099)) + (((-1.0) * x1097)) + ((x1094 * x1099)) + (((-1.0) * x1092)) + (((-2.0) * x1090 * x1095 * x1096)) + (((-1.0) * x1095 * x1100)) + ((x1091 * x1093)) + ((x1090 * x1092)) + (((-1.0) * x1096 * x1100)));
                                    j3eval[0] = x1101;
                                    j3eval[1] = IKsign(x1101);
                                    if (IKabs(j3eval[0]) < 0.0000010000000000 || IKabs(j3eval[1]) < 0.0000010000000000)
                                    {
                                        {
                                            IkReal j3eval[2];
                                            IkReal x1102 = (((cj5 * r11 * sj4)) + ((r10 * sj4 * sj5)) + (((-1.0) * cj4 * r12)));
                                            j3eval[0] = x1102;
                                            j3eval[1] = IKsign(x1102);
                                            if (IKabs(j3eval[0]) < 0.0000010000000000 || IKabs(j3eval[1]) < 0.0000010000000000)
                                            {
                                                {
                                                    IkReal j3eval[2];
                                                    IkReal x1103 = ((1.0) * sj4);
                                                    IkReal x1104 = ((((-1.0) * r00 * sj5 * x1103)) + (((-1.0) * cj5 * r01 * x1103)) + ((cj4 * r02)));
                                                    j3eval[0] = x1104;
                                                    j3eval[1] = IKsign(x1104);
                                                    if (IKabs(j3eval[0]) < 0.0000010000000000 || IKabs(j3eval[1]) < 0.0000010000000000)
                                                    {
                                                        continue; // no branches [j3]
                                                    }
                                                    else
                                                    {
                                                        {
                                                            IkReal j3array[1], cj3array[1], sj3array[1];
                                                            bool j3valid[1] = {false};
                                                            _nj3 = 1;
                                                            IkReal x1105 = ((0.195039861251953) * sj0);
                                                            IkReal x1106 = (cj1 * cj2);
                                                            IkReal x1107 = (cj5 * r20);
                                                            IkReal x1108 = (r22 * sj4);
                                                            IkReal x1109 = (cj1 * sj2);
                                                            IkReal x1110 = (cj4 * cj5);
                                                            IkReal x1111 = (r12 * sj4);
                                                            IkReal x1112 = ((0.980795316323859) * cj0);
                                                            IkReal x1113 = ((1.0) * sj5);
                                                            IkReal x1114 = (sj1 * sj2);
                                                            IkReal x1115 = (cj2 * sj1);
                                                            IkReal x1116 = (r21 * sj5);
                                                            IkReal x1117 = ((1.0) * cj5);
                                                            IkReal x1118 = (cj4 * r10);
                                                            IkReal x1119 = (r21 * x1114);
                                                            IkReal x1120 = (cj4 * r20 * sj5);
                                                            CheckValue<IkReal> x1121 = IKatan2WithCheck(IkReal((((x1109 * x1111)) + ((x1105 * x1108 * x1114)) + ((r21 * x1105 * x1106 * x1110)) + (((-1.0) * r11 * x1110 * x1115)) + ((x1108 * x1112 * x1114)) + ((x1105 * x1114 * x1120)) + (((-1.0) * x1111 * x1115)) + ((x1105 * x1110 * x1119)) + ((r21 * x1106 * x1110 * x1112)) + ((x1105 * x1106 * x1108)) + ((x1110 * x1112 * x1119)) + ((r11 * x1109 * x1110)) + (((-1.0) * x1113 * x1115 * x1118)) + ((x1105 * x1106 * x1120)) + ((x1112 * x1114 * x1120)) + ((sj5 * x1109 * x1118)) + ((x1106 * x1112 * x1120)) + ((x1106 * x1108 * x1112)))), IkReal(((((-1.0) * x1106 * x1107 * x1112)) + (((-1.0) * r11 * x1113 * x1115)) + ((cj5 * r10 * x1115)) + (((-1.0) * r10 * x1109 * x1117)) + ((r11 * sj5 * x1109)) + (((-1.0) * x1107 * x1112 * x1114)) + ((x1105 * x1106 * x1116)) + ((x1106 * x1112 * x1116)) + (((-1.0) * x1105 * x1106 * x1107)) + ((x1112 * x1114 * x1116)) + (((-1.0) * x1105 * x1107 * x1114)) + ((x1105 * x1114 * x1116)))), IKFAST_ATAN2_MAGTHRESH);
                                                            if (!x1121.valid)
                                                            {
                                                                continue;
                                                            }
                                                            CheckValue<IkReal> x1122 = IKPowWithIntegerCheck(IKsign(((((-1.0) * r00 * sj4 * x1113)) + (((-1.0) * r01 * sj4 * x1117)) + ((cj4 * r02)))), -1);
                                                            if (!x1122.valid)
                                                            {
                                                                continue;
                                                            }
                                                            j3array[0] = ((-1.5707963267949) + (x1121.value) + (((1.5707963267949) * (x1122.value))));
                                                            sj3array[0] = IKsin(j3array[0]);
                                                            cj3array[0] = IKcos(j3array[0]);
                                                            if (j3array[0] > IKPI)
                                                            {
                                                                j3array[0] -= IK2PI;
                                                            }
                                                            else if (j3array[0] < -IKPI)
                                                            {
                                                                j3array[0] += IK2PI;
                                                            }
                                                            j3valid[0] = true;
                                                            for (int ij3 = 0; ij3 < 1; ++ij3)
                                                            {
                                                                if (!j3valid[ij3])
                                                                {
                                                                    continue;
                                                                }
                                                                _ij3[0] = ij3;
                                                                _ij3[1] = -1;
                                                                for (int iij3 = ij3 + 1; iij3 < 1; ++iij3)
                                                                {
                                                                    if (j3valid[iij3] && IKabs(cj3array[ij3] - cj3array[iij3]) < IKFAST_SOLUTION_THRESH && IKabs(sj3array[ij3] - sj3array[iij3]) < IKFAST_SOLUTION_THRESH)
                                                                    {
                                                                        j3valid[iij3] = false;
                                                                        _ij3[1] = iij3;
                                                                        break;
                                                                    }
                                                                }
                                                                j3 = j3array[ij3];
                                                                cj3 = cj3array[ij3];
                                                                sj3 = sj3array[ij3];
                                                                {
                                                                    IkReal evalcond[6];
                                                                    IkReal x1123 = IKcos(j3);
                                                                    IkReal x1124 = IKsin(j3);
                                                                    IkReal x1125 = ((0.980795316323859) * cj0);
                                                                    IkReal x1126 = (sj1 * sj2);
                                                                    IkReal x1127 = (cj1 * cj2);
                                                                    IkReal x1128 = ((0.980795316323859) * sj0);
                                                                    IkReal x1129 = (r00 * sj5);
                                                                    IkReal x1130 = (cj5 * r11);
                                                                    IkReal x1131 = (r10 * sj5);
                                                                    IkReal x1132 = (r02 * sj4);
                                                                    IkReal x1133 = (cj5 * r01);
                                                                    IkReal x1134 = (r12 * sj4);
                                                                    IkReal x1135 = (cj2 * sj1);
                                                                    IkReal x1136 = ((0.195039861251953) * sj0);
                                                                    IkReal x1137 = (r20 * sj5);
                                                                    IkReal x1138 = (r22 * sj4);
                                                                    IkReal x1139 = (r21 * sj5);
                                                                    IkReal x1140 = (r01 * sj5);
                                                                    IkReal x1141 = ((0.195039861251953) * cj0);
                                                                    IkReal x1142 = (cj5 * r21);
                                                                    IkReal x1143 = (cj1 * sj2);
                                                                    IkReal x1144 = ((1.0) * r11 * sj5);
                                                                    IkReal x1145 = (cj5 * x1124);
                                                                    IkReal x1146 = ((1.0) * x1124);
                                                                    IkReal x1147 = (cj4 * x1123);
                                                                    IkReal x1148 = (cj5 * x1123);
                                                                    IkReal x1149 = ((1.0) * x1123);
                                                                    IkReal x1150 = (cj4 * x1146);
                                                                    evalcond[0] = (((x1137 * x1147)) + x1143 + ((x1142 * x1147)) + ((r20 * x1145)) + (((-1.0) * x1135)) + ((x1123 * x1138)) + (((-1.0) * x1139 * x1146)));
                                                                    evalcond[1] = (x1126 + x1127 + ((r20 * x1148)) + (((-1.0) * x1139 * x1149)) + (((-1.0) * x1137 * x1150)) + (((-1.0) * x1142 * x1150)) + (((-1.0) * x1138 * x1146)));
                                                                    evalcond[2] = ((((-1.0) * x1140 * x1146)) + ((x1127 * x1128)) + ((x1129 * x1147)) + ((x1133 * x1147)) + (((-1.0) * x1127 * x1141)) + (((-1.0) * x1126 * x1141)) + ((x1123 * x1132)) + ((r00 * x1145)) + ((x1126 * x1128)));
                                                                    evalcond[3] = ((((-1.0) * x1127 * x1136)) + ((x1130 * x1147)) + (((-1.0) * x1126 * x1136)) + ((x1131 * x1147)) + (((-1.0) * x1124 * x1144)) + ((x1123 * x1134)) + (((-1.0) * x1125 * x1127)) + (((-1.0) * x1125 * x1126)) + ((r10 * x1145)));
                                                                    evalcond[4] = ((((-1.0) * x1135 * x1141)) + (((-1.0) * x1140 * x1149)) + (((-1.0) * x1128 * x1143)) + (((-1.0) * x1129 * x1150)) + ((x1141 * x1143)) + (((-1.0) * x1132 * x1146)) + ((x1128 * x1135)) + (((-1.0) * x1133 * x1150)) + ((r00 * x1148)));
                                                                    evalcond[5] = ((((-1.0) * x1123 * x1144)) + ((x1125 * x1143)) + (((-1.0) * x1134 * x1146)) + (((-1.0) * x1130 * x1150)) + (((-1.0) * x1131 * x1150)) + (((-1.0) * x1135 * x1136)) + ((r10 * x1148)) + (((-1.0) * x1125 * x1135)) + ((x1136 * x1143)));
                                                                    if (IKabs(evalcond[0]) > IKFAST_EVALCOND_THRESH || IKabs(evalcond[1]) > IKFAST_EVALCOND_THRESH || IKabs(evalcond[2]) > IKFAST_EVALCOND_THRESH || IKabs(evalcond[3]) > IKFAST_EVALCOND_THRESH || IKabs(evalcond[4]) > IKFAST_EVALCOND_THRESH || IKabs(evalcond[5]) > IKFAST_EVALCOND_THRESH)
                                                                    {
                                                                        continue;
                                                                    }
                                                                }

                                                                {
                                                                    std::vector<IkSingleDOFSolutionBase<IkReal>> vinfos(6);
                                                                    vinfos[0].jointtype = 1;
                                                                    vinfos[0].foffset = j0;
                                                                    vinfos[0].indices[0] = _ij0[0];
                                                                    vinfos[0].indices[1] = _ij0[1];
                                                                    vinfos[0].maxsolutions = _nj0;
                                                                    vinfos[1].jointtype = 1;
                                                                    vinfos[1].foffset = j1;
                                                                    vinfos[1].indices[0] = _ij1[0];
                                                                    vinfos[1].indices[1] = _ij1[1];
                                                                    vinfos[1].maxsolutions = _nj1;
                                                                    vinfos[2].jointtype = 1;
                                                                    vinfos[2].foffset = j2;
                                                                    vinfos[2].indices[0] = _ij2[0];
                                                                    vinfos[2].indices[1] = _ij2[1];
                                                                    vinfos[2].maxsolutions = _nj2;
                                                                    vinfos[3].jointtype = 1;
                                                                    vinfos[3].foffset = j3;
                                                                    vinfos[3].indices[0] = _ij3[0];
                                                                    vinfos[3].indices[1] = _ij3[1];
                                                                    vinfos[3].maxsolutions = _nj3;
                                                                    vinfos[4].jointtype = 1;
                                                                    vinfos[4].foffset = j4;
                                                                    vinfos[4].indices[0] = _ij4[0];
                                                                    vinfos[4].indices[1] = _ij4[1];
                                                                    vinfos[4].maxsolutions = _nj4;
                                                                    vinfos[5].jointtype = 1;
                                                                    vinfos[5].foffset = j5;
                                                                    vinfos[5].indices[0] = _ij5[0];
                                                                    vinfos[5].indices[1] = _ij5[1];
                                                                    vinfos[5].maxsolutions = _nj5;
                                                                    std::vector<int> vfree(0);
                                                                    solutions.AddSolution(vinfos, vfree);
                                                                }
                                                            }
                                                        }
                                                    }
                                                }
                                            }
                                            else
                                            {
                                                {
                                                    IkReal j3array[1], cj3array[1], sj3array[1];
                                                    bool j3valid[1] = {false};
                                                    _nj3 = 1;
                                                    IkReal x1151 = (cj1 * sj2);
                                                    IkReal x1152 = (cj4 * cj5);
                                                    IkReal x1153 = (sj1 * sj5);
                                                    IkReal x1154 = ((1.0) * r01);
                                                    IkReal x1155 = ((0.980795316323859) * sj0);
                                                    IkReal x1156 = (r22 * sj4);
                                                    IkReal x1157 = (r02 * sj4);
                                                    IkReal x1158 = (cj1 * cj2);
                                                    IkReal x1159 = (cj2 * r21);
                                                    IkReal x1160 = (sj1 * sj2);
                                                    IkReal x1161 = ((0.195039861251953) * cj0);
                                                    IkReal x1162 = (cj5 * r20);
                                                    IkReal x1163 = (cj4 * r00);
                                                    IkReal x1164 = (cj2 * sj1);
                                                    IkReal x1165 = (cj5 * r00);
                                                    IkReal x1166 = (r21 * x1161);
                                                    IkReal x1167 = (cj2 * cj4 * r20);
                                                    IkReal x1168 = (cj4 * r20 * sj5);
                                                    CheckValue<IkReal> x1169 = IKPowWithIntegerCheck(IKsign((((cj5 * r11 * sj4)) + ((r10 * sj4 * sj5)) + (((-1.0) * cj4 * r12)))), -1);
                                                    if (!x1169.valid)
                                                    {
                                                        continue;
                                                    }
                                                    CheckValue<IkReal> x1170 = IKatan2WithCheck(IkReal(((((-1.0) * x1153 * x1159 * x1161)) + (((-1.0) * r21 * sj5 * x1151 * x1155)) + (((-1.0) * sj2 * x1153 * x1154)) + ((x1161 * x1162 * x1164)) + ((x1153 * x1155 * x1159)) + (((-1.0) * x1155 * x1162 * x1164)) + ((sj5 * x1151 * x1166)) + ((x1151 * x1155 * x1162)) + ((x1158 * x1165)) + (((-1.0) * x1151 * x1161 * x1162)) + (((-1.0) * sj5 * x1154 * x1158)) + ((x1160 * x1165)))), IkReal((((r01 * x1152 * x1158)) + ((r01 * x1152 * x1160)) + (((-1.0) * x1153 * x1155 * x1167)) + ((x1151 * x1155 * x1156)) + ((x1157 * x1160)) + ((sj1 * x1152 * x1159 * x1161)) + ((x1153 * x1161 * x1167)) + ((sj2 * x1153 * x1163)) + (((-1.0) * sj1 * x1152 * x1155 * x1159)) + ((sj5 * x1158 * x1163)) + (((-1.0) * x1151 * x1156 * x1161)) + ((x1156 * x1161 * x1164)) + ((x1151 * x1155 * x1168)) + (((-1.0) * x1151 * x1161 * x1168)) + (((-1.0) * x1155 * x1156 * x1164)) + ((x1157 * x1158)) + (((-1.0) * x1151 * x1152 * x1166)) + ((r21 * x1151 * x1152 * x1155)))), IKFAST_ATAN2_MAGTHRESH);
                                                    if (!x1170.valid)
                                                    {
                                                        continue;
                                                    }
                                                    j3array[0] = ((-1.5707963267949) + (((1.5707963267949) * (x1169.value))) + (x1170.value));
                                                    sj3array[0] = IKsin(j3array[0]);
                                                    cj3array[0] = IKcos(j3array[0]);
                                                    if (j3array[0] > IKPI)
                                                    {
                                                        j3array[0] -= IK2PI;
                                                    }
                                                    else if (j3array[0] < -IKPI)
                                                    {
                                                        j3array[0] += IK2PI;
                                                    }
                                                    j3valid[0] = true;
                                                    for (int ij3 = 0; ij3 < 1; ++ij3)
                                                    {
                                                        if (!j3valid[ij3])
                                                        {
                                                            continue;
                                                        }
                                                        _ij3[0] = ij3;
                                                        _ij3[1] = -1;
                                                        for (int iij3 = ij3 + 1; iij3 < 1; ++iij3)
                                                        {
                                                            if (j3valid[iij3] && IKabs(cj3array[ij3] - cj3array[iij3]) < IKFAST_SOLUTION_THRESH && IKabs(sj3array[ij3] - sj3array[iij3]) < IKFAST_SOLUTION_THRESH)
                                                            {
                                                                j3valid[iij3] = false;
                                                                _ij3[1] = iij3;
                                                                break;
                                                            }
                                                        }
                                                        j3 = j3array[ij3];
                                                        cj3 = cj3array[ij3];
                                                        sj3 = sj3array[ij3];
                                                        {
                                                            IkReal evalcond[6];
                                                            IkReal x1171 = IKcos(j3);
                                                            IkReal x1172 = IKsin(j3);
                                                            IkReal x1173 = ((0.980795316323859) * cj0);
                                                            IkReal x1174 = (sj1 * sj2);
                                                            IkReal x1175 = (cj1 * cj2);
                                                            IkReal x1176 = ((0.980795316323859) * sj0);
                                                            IkReal x1177 = (r00 * sj5);
                                                            IkReal x1178 = (cj5 * r11);
                                                            IkReal x1179 = (r10 * sj5);
                                                            IkReal x1180 = (r02 * sj4);
                                                            IkReal x1181 = (cj5 * r01);
                                                            IkReal x1182 = (r12 * sj4);
                                                            IkReal x1183 = (cj2 * sj1);
                                                            IkReal x1184 = ((0.195039861251953) * sj0);
                                                            IkReal x1185 = (r20 * sj5);
                                                            IkReal x1186 = (r22 * sj4);
                                                            IkReal x1187 = (r21 * sj5);
                                                            IkReal x1188 = (r01 * sj5);
                                                            IkReal x1189 = ((0.195039861251953) * cj0);
                                                            IkReal x1190 = (cj5 * r21);
                                                            IkReal x1191 = (cj1 * sj2);
                                                            IkReal x1192 = ((1.0) * r11 * sj5);
                                                            IkReal x1193 = (cj5 * x1172);
                                                            IkReal x1194 = ((1.0) * x1172);
                                                            IkReal x1195 = (cj4 * x1171);
                                                            IkReal x1196 = (cj5 * x1171);
                                                            IkReal x1197 = ((1.0) * x1171);
                                                            IkReal x1198 = (cj4 * x1194);
                                                            evalcond[0] = (x1191 + ((x1185 * x1195)) + (((-1.0) * x1187 * x1194)) + ((x1171 * x1186)) + ((r20 * x1193)) + (((-1.0) * x1183)) + ((x1190 * x1195)));
                                                            evalcond[1] = (x1175 + x1174 + (((-1.0) * x1190 * x1198)) + (((-1.0) * x1186 * x1194)) + (((-1.0) * x1187 * x1197)) + (((-1.0) * x1185 * x1198)) + ((r20 * x1196)));
                                                            evalcond[2] = ((((-1.0) * x1175 * x1189)) + ((x1174 * x1176)) + ((r00 * x1193)) + (((-1.0) * x1188 * x1194)) + ((x1175 * x1176)) + ((x1177 * x1195)) + ((x1171 * x1180)) + ((x1181 * x1195)) + (((-1.0) * x1174 * x1189)));
                                                            evalcond[3] = ((((-1.0) * x1175 * x1184)) + ((x1178 * x1195)) + (((-1.0) * x1173 * x1174)) + (((-1.0) * x1173 * x1175)) + ((r10 * x1193)) + ((x1179 * x1195)) + (((-1.0) * x1172 * x1192)) + ((x1171 * x1182)) + (((-1.0) * x1174 * x1184)));
                                                            evalcond[4] = ((((-1.0) * x1177 * x1198)) + (((-1.0) * x1181 * x1198)) + ((r00 * x1196)) + (((-1.0) * x1188 * x1197)) + ((x1176 * x1183)) + ((x1189 * x1191)) + (((-1.0) * x1180 * x1194)) + (((-1.0) * x1183 * x1189)) + (((-1.0) * x1176 * x1191)));
                                                            evalcond[5] = (((x1184 * x1191)) + ((r10 * x1196)) + ((x1173 * x1191)) + (((-1.0) * x1179 * x1198)) + (((-1.0) * x1178 * x1198)) + (((-1.0) * x1182 * x1194)) + (((-1.0) * x1173 * x1183)) + (((-1.0) * x1183 * x1184)) + (((-1.0) * x1171 * x1192)));
                                                            if (IKabs(evalcond[0]) > IKFAST_EVALCOND_THRESH || IKabs(evalcond[1]) > IKFAST_EVALCOND_THRESH || IKabs(evalcond[2]) > IKFAST_EVALCOND_THRESH || IKabs(evalcond[3]) > IKFAST_EVALCOND_THRESH || IKabs(evalcond[4]) > IKFAST_EVALCOND_THRESH || IKabs(evalcond[5]) > IKFAST_EVALCOND_THRESH)
                                                            {
                                                                continue;
                                                            }
                                                        }

                                                        {
                                                            std::vector<IkSingleDOFSolutionBase<IkReal>> vinfos(6);
                                                            vinfos[0].jointtype = 1;
                                                            vinfos[0].foffset = j0;
                                                            vinfos[0].indices[0] = _ij0[0];
                                                            vinfos[0].indices[1] = _ij0[1];
                                                            vinfos[0].maxsolutions = _nj0;
                                                            vinfos[1].jointtype = 1;
                                                            vinfos[1].foffset = j1;
                                                            vinfos[1].indices[0] = _ij1[0];
                                                            vinfos[1].indices[1] = _ij1[1];
                                                            vinfos[1].maxsolutions = _nj1;
                                                            vinfos[2].jointtype = 1;
                                                            vinfos[2].foffset = j2;
                                                            vinfos[2].indices[0] = _ij2[0];
                                                            vinfos[2].indices[1] = _ij2[1];
                                                            vinfos[2].maxsolutions = _nj2;
                                                            vinfos[3].jointtype = 1;
                                                            vinfos[3].foffset = j3;
                                                            vinfos[3].indices[0] = _ij3[0];
                                                            vinfos[3].indices[1] = _ij3[1];
                                                            vinfos[3].maxsolutions = _nj3;
                                                            vinfos[4].jointtype = 1;
                                                            vinfos[4].foffset = j4;
                                                            vinfos[4].indices[0] = _ij4[0];
                                                            vinfos[4].indices[1] = _ij4[1];
                                                            vinfos[4].maxsolutions = _nj4;
                                                            vinfos[5].jointtype = 1;
                                                            vinfos[5].foffset = j5;
                                                            vinfos[5].indices[0] = _ij5[0];
                                                            vinfos[5].indices[1] = _ij5[1];
                                                            vinfos[5].maxsolutions = _nj5;
                                                            std::vector<int> vfree(0);
                                                            solutions.AddSolution(vinfos, vfree);
                                                        }
                                                    }
                                                }
                                            }
                                        }
                                    }
                                    else
                                    {
                                        {
                                            IkReal j3array[1], cj3array[1], sj3array[1];
                                            bool j3valid[1] = {false};
                                            _nj3 = 1;
                                            IkReal x1199 = cj4 * cj4;
                                            IkReal x1200 = cj5 * cj5;
                                            IkReal x1201 = r22 * r22;
                                            IkReal x1202 = r21 * r21;
                                            IkReal x1203 = r20 * r20;
                                            IkReal x1204 = ((1.0) * sj1);
                                            IkReal x1205 = (r22 * sj4);
                                            IkReal x1206 = (sj2 * sj5);
                                            IkReal x1207 = (cj4 * r20);
                                            IkReal x1208 = (cj5 * sj2);
                                            IkReal x1209 = (r21 * sj5);
                                            IkReal x1210 = (cj4 * r21);
                                            IkReal x1211 = ((2.0) * cj5);
                                            IkReal x1212 = (cj2 * cj5 * r20);
                                            IkReal x1213 = ((1.0) * x1202);
                                            IkReal x1214 = ((1.0) * cj1 * cj2);
                                            IkReal x1215 = ((1.0) * x1203);
                                            IkReal x1216 = (x1199 * x1200);
                                            CheckValue<IkReal> x1217 = IKPowWithIntegerCheck(IKsign(((((-1.0) * x1200 * x1215)) + ((x1199 * x1201)) + ((x1200 * x1202)) + (((-1.0) * x1213 * x1216)) + (((-1.0) * x1201)) + (((-1.0) * x1199 * x1215)) + ((x1203 * x1216)) + (((-1.0) * x1213)) + (((-1.0) * r20 * x1199 * x1209 * x1211)) + ((r20 * x1209 * x1211)) + (((-2.0) * sj5 * x1205 * x1207)) + (((-1.0) * x1205 * x1210 * x1211)))), -1);
                                            if (!x1217.valid)
                                            {
                                                continue;
                                            }
                                            CheckValue<IkReal> x1218 = IKatan2WithCheck(IkReal(((((-1.0) * x1204 * x1212)) + (((-1.0) * x1204 * x1206 * x1207)) + (((-1.0) * cj5 * x1210 * x1214)) + ((cj1 * r20 * x1208)) + (((-1.0) * x1204 * x1208 * x1210)) + (((-1.0) * x1205 * x1214)) + (((-1.0) * sj5 * x1207 * x1214)) + (((-1.0) * sj2 * x1204 * x1205)) + (((-1.0) * cj1 * r21 * x1206)) + ((cj2 * sj1 * x1209)))), IkReal(((((-1.0) * x1209 * x1214)) + ((cj1 * x1208 * x1210)) + (((-1.0) * r21 * x1204 * x1206)) + ((cj1 * x1206 * x1207)) + (((-1.0) * cj2 * sj5 * x1204 * x1207)) + (((-1.0) * cj2 * cj5 * x1204 * x1210)) + (((-1.0) * cj2 * x1204 * x1205)) + ((cj1 * sj2 * x1205)) + ((r20 * sj1 * x1208)) + ((cj1 * x1212)))), IKFAST_ATAN2_MAGTHRESH);
                                            if (!x1218.valid)
                                            {
                                                continue;
                                            }
                                            j3array[0] = ((-1.5707963267949) + (((1.5707963267949) * (x1217.value))) + (x1218.value));
                                            sj3array[0] = IKsin(j3array[0]);
                                            cj3array[0] = IKcos(j3array[0]);
                                            if (j3array[0] > IKPI)
                                            {
                                                j3array[0] -= IK2PI;
                                            }
                                            else if (j3array[0] < -IKPI)
                                            {
                                                j3array[0] += IK2PI;
                                            }
                                            j3valid[0] = true;
                                            for (int ij3 = 0; ij3 < 1; ++ij3)
                                            {
                                                if (!j3valid[ij3])
                                                {
                                                    continue;
                                                }
                                                _ij3[0] = ij3;
                                                _ij3[1] = -1;
                                                for (int iij3 = ij3 + 1; iij3 < 1; ++iij3)
                                                {
                                                    if (j3valid[iij3] && IKabs(cj3array[ij3] - cj3array[iij3]) < IKFAST_SOLUTION_THRESH && IKabs(sj3array[ij3] - sj3array[iij3]) < IKFAST_SOLUTION_THRESH)
                                                    {
                                                        j3valid[iij3] = false;
                                                        _ij3[1] = iij3;
                                                        break;
                                                    }
                                                }
                                                j3 = j3array[ij3];
                                                cj3 = cj3array[ij3];
                                                sj3 = sj3array[ij3];
                                                {
                                                    IkReal evalcond[6];
                                                    IkReal x1219 = IKcos(j3);
                                                    IkReal x1220 = IKsin(j3);
                                                    IkReal x1221 = ((0.980795316323859) * cj0);
                                                    IkReal x1222 = (sj1 * sj2);
                                                    IkReal x1223 = (cj1 * cj2);
                                                    IkReal x1224 = ((0.980795316323859) * sj0);
                                                    IkReal x1225 = (r00 * sj5);
                                                    IkReal x1226 = (cj5 * r11);
                                                    IkReal x1227 = (r10 * sj5);
                                                    IkReal x1228 = (r02 * sj4);
                                                    IkReal x1229 = (cj5 * r01);
                                                    IkReal x1230 = (r12 * sj4);
                                                    IkReal x1231 = (cj2 * sj1);
                                                    IkReal x1232 = ((0.195039861251953) * sj0);
                                                    IkReal x1233 = (r20 * sj5);
                                                    IkReal x1234 = (r22 * sj4);
                                                    IkReal x1235 = (r21 * sj5);
                                                    IkReal x1236 = (r01 * sj5);
                                                    IkReal x1237 = ((0.195039861251953) * cj0);
                                                    IkReal x1238 = (cj5 * r21);
                                                    IkReal x1239 = (cj1 * sj2);
                                                    IkReal x1240 = ((1.0) * r11 * sj5);
                                                    IkReal x1241 = (cj5 * x1220);
                                                    IkReal x1242 = ((1.0) * x1220);
                                                    IkReal x1243 = (cj4 * x1219);
                                                    IkReal x1244 = (cj5 * x1219);
                                                    IkReal x1245 = ((1.0) * x1219);
                                                    IkReal x1246 = (cj4 * x1242);
                                                    evalcond[0] = ((((-1.0) * x1231)) + x1239 + ((x1219 * x1234)) + ((r20 * x1241)) + ((x1233 * x1243)) + (((-1.0) * x1235 * x1242)) + ((x1238 * x1243)));
                                                    evalcond[1] = (x1223 + x1222 + ((r20 * x1244)) + (((-1.0) * x1233 * x1246)) + (((-1.0) * x1235 * x1245)) + (((-1.0) * x1238 * x1246)) + (((-1.0) * x1234 * x1242)));
                                                    evalcond[2] = ((((-1.0) * x1223 * x1237)) + ((x1225 * x1243)) + ((x1229 * x1243)) + ((x1223 * x1224)) + ((x1219 * x1228)) + (((-1.0) * x1236 * x1242)) + (((-1.0) * x1222 * x1237)) + ((r00 * x1241)) + ((x1222 * x1224)));
                                                    evalcond[3] = ((((-1.0) * x1223 * x1232)) + ((x1219 * x1230)) + (((-1.0) * x1220 * x1240)) + (((-1.0) * x1222 * x1232)) + (((-1.0) * x1221 * x1222)) + (((-1.0) * x1221 * x1223)) + ((x1226 * x1243)) + ((x1227 * x1243)) + ((r10 * x1241)));
                                                    evalcond[4] = ((((-1.0) * x1225 * x1246)) + (((-1.0) * x1224 * x1239)) + (((-1.0) * x1228 * x1242)) + (((-1.0) * x1236 * x1245)) + (((-1.0) * x1229 * x1246)) + ((x1224 * x1231)) + ((r00 * x1244)) + ((x1237 * x1239)) + (((-1.0) * x1231 * x1237)));
                                                    evalcond[5] = ((((-1.0) * x1226 * x1246)) + (((-1.0) * x1221 * x1231)) + (((-1.0) * x1227 * x1246)) + (((-1.0) * x1219 * x1240)) + (((-1.0) * x1230 * x1242)) + ((x1221 * x1239)) + ((x1232 * x1239)) + ((r10 * x1244)) + (((-1.0) * x1231 * x1232)));
                                                    if (IKabs(evalcond[0]) > IKFAST_EVALCOND_THRESH || IKabs(evalcond[1]) > IKFAST_EVALCOND_THRESH || IKabs(evalcond[2]) > IKFAST_EVALCOND_THRESH || IKabs(evalcond[3]) > IKFAST_EVALCOND_THRESH || IKabs(evalcond[4]) > IKFAST_EVALCOND_THRESH || IKabs(evalcond[5]) > IKFAST_EVALCOND_THRESH)
                                                    {
                                                        continue;
                                                    }
                                                }

                                                {
                                                    std::vector<IkSingleDOFSolutionBase<IkReal>> vinfos(6);
                                                    vinfos[0].jointtype = 1;
                                                    vinfos[0].foffset = j0;
                                                    vinfos[0].indices[0] = _ij0[0];
                                                    vinfos[0].indices[1] = _ij0[1];
                                                    vinfos[0].maxsolutions = _nj0;
                                                    vinfos[1].jointtype = 1;
                                                    vinfos[1].foffset = j1;
                                                    vinfos[1].indices[0] = _ij1[0];
                                                    vinfos[1].indices[1] = _ij1[1];
                                                    vinfos[1].maxsolutions = _nj1;
                                                    vinfos[2].jointtype = 1;
                                                    vinfos[2].foffset = j2;
                                                    vinfos[2].indices[0] = _ij2[0];
                                                    vinfos[2].indices[1] = _ij2[1];
                                                    vinfos[2].maxsolutions = _nj2;
                                                    vinfos[3].jointtype = 1;
                                                    vinfos[3].foffset = j3;
                                                    vinfos[3].indices[0] = _ij3[0];
                                                    vinfos[3].indices[1] = _ij3[1];
                                                    vinfos[3].maxsolutions = _nj3;
                                                    vinfos[4].jointtype = 1;
                                                    vinfos[4].foffset = j4;
                                                    vinfos[4].indices[0] = _ij4[0];
                                                    vinfos[4].indices[1] = _ij4[1];
                                                    vinfos[4].maxsolutions = _nj4;
                                                    vinfos[5].jointtype = 1;
                                                    vinfos[5].foffset = j5;
                                                    vinfos[5].indices[0] = _ij5[0];
                                                    vinfos[5].indices[1] = _ij5[1];
                                                    vinfos[5].maxsolutions = _nj5;
                                                    std::vector<int> vfree(0);
                                                    solutions.AddSolution(vinfos, vfree);
                                                }
                                            }
                                        }
                                    }
                                }
                            }
                        }
                    }
                }
            }
        }
    }

    /// solves the forward kinematics equations.
    /// \param pfree is an array specifying the free joints of the chain.
    IKFAST_API void ComputeFk(const IkReal *j, IkReal *eetrans, IkReal *eerot)
    {
        IkReal x0, x1, x2, x3, x4, x5, x6, x7, x8, x9, x10, x11, x12, x13, x14, x15, x16, x17, x18, x19, x20, x21, x22, x23, x24, x25, x26, x27, x28, x29, x30, x31, x32, x33, x34, x35, x36, x37, x38, x39, x40, x41, x42, x43, x44, x45, x46, x47, x48, x49, x50, x51, x52, x53, x54, x55, x56, x57, x58, x59;
        x0 = IKsin(j[0]);
        x1 = IKcos(j[0]);
        x2 = IKcos(j[1]);
        x3 = IKsin(j[2]);
        x4 = IKcos(j[2]);
        x5 = IKsin(j[1]);
        x6 = IKsin(j[3]);
        x7 = IKcos(j[3]);
        x8 = IKcos(j[5]);
        x9 = IKsin(j[5]);
        x10 = IKcos(j[4]);
        x11 = IKsin(j[4]);
        x12 = ((0.195039861251953) * x1);
        x13 = ((0.980795316323859) * x0);
        x14 = ((0.443319482978384) * x0);
        x15 = ((0.0881580172858829) * x1);
        x16 = ((3.7e-7) * x6);
        x17 = ((1.0) * x6);
        x18 = ((0.1135) * x3);
        x19 = ((1.0) * x7);
        x20 = ((3.7e-7) * x7);
        x21 = ((-1.0) * x4);
        x22 = (x2 * x4);
        x23 = ((-1.0) * x6);
        x24 = (x2 * x3);
        x25 = ((-1.0) * x7);
        x26 = ((-0.0676) * x7);
        x27 = (x4 * x5);
        x28 = (x3 * x5);
        x29 = ((-0.0676) * x6);
        x30 = ((((-1.0) * x12)) + x13);
        x31 = ((((-1.0) * x13)) + x12);
        x32 = ((1.0) * x27);
        x33 = ((((0.195039861251953) * x0)) + (((0.980795316323859) * x1)));
        x34 = ((-1.0) * x33);
        x35 = ((((0.0881580172858829) * x0)) + (((0.443319482978384) * x1)));
        x36 = ((0.1135) * x30);
        x37 = (x11 * x31);
        x38 = (x30 * x5);
        x39 = ((0.1135) * x34);
        x40 = ((((-1.0) * x32)) + x24);
        x41 = (x34 * x5);
        x42 = ((((-1.0) * x24)) + x32);
        x43 = (x24 * x31);
        x44 = ((((-1.0) * x22)) + (((-1.0) * x28)));
        x45 = (x44 * x6);
        x46 = (((x28 * x30)) + ((x22 * x30)));
        x47 = (((x28 * x34)) + ((x22 * x34)));
        x48 = ((((-1.0) * x43)) + ((x21 * x38)));
        x49 = (((x2 * x21 * x30)) + (((-1.0) * x28 * x30)));
        x50 = ((((-1.0) * x24 * x33)) + (((-1.0) * x32 * x34)));
        x51 = (x47 * x7);
        x52 = (((x2 * x21 * x34)) + (((-1.0) * x28 * x34)));
        x53 = (((x21 * x41)) + (((-1.0) * x24 * x33)));
        x54 = (x48 * x6);
        x55 = ((((-1.0) * x17 * x44)) + (((-1.0) * x19 * x40)));
        x56 = (x50 * x6);
        x57 = ((((-1.0) * x17 * x48)) + (((-1.0) * x19 * x46)));
        x58 = ((((-1.0) * x19 * x47)) + (((-1.0) * x17 * x50)));
        x59 = (((x11 * x34)) + ((x10 * x57)));
        eerot[0] = (((x8 * ((((x49 * x6)) + ((x48 * x7)))))) + ((x59 * x9)));
        eerot[1] = (((x9 * ((((x23 * x49)) + ((x25 * x48)))))) + ((x59 * x8)));
        eerot[2] = (((x10 * x33)) + ((x11 * x57)));
        eetrans[0] = (((x7 * ((((x27 * x36)) + ((x18 * x2 * x31)))))) + ((x5 * (((((0.458031412723242) * x0)) + (((-0.0910836152046622) * x1)))))) + ((x27 * (((((-1.0) * x15)) + x14)))) + ((x10 * (((((-3.7e-7) * x46 * x7)) + (((-3.7e-7) * x54)))))) + ((x6 * ((((x18 * x38)) + ((x22 * x36)))))) + (((0.0221176390070984) * x0)) + ((x10 * (((((0.0663017633834929) * x1)) + (((0.013184694620632) * x0)))))) + ((x24 * (((((-1.0) * x14)) + x15)))) + (((0.111222163942722) * x1)) + ((x11 * (((((-3.62894267039828e-7) * x1)) + (((-7.21647486632227e-8) * x0)))))) + ((x11 * ((((x29 * x48)) + ((x26 * x46)))))));
        eerot[3] = (((x8 * ((((x50 * x7)) + ((x52 * x6)))))) + ((x9 * ((((x10 * x58)) + x37)))));
        eerot[4] = (((x9 * (((((-1.0) * x19 * x50)) + (((-1.0) * x17 * x52)))))) + ((x8 * ((x37 + ((x10 * ((((x25 * x47)) + ((x23 * x50)))))))))));
        eerot[5] = (((x10 * x30)) + ((x11 * x58)));
        eetrans[1] = ((((-1.0) * x27 * x35)) + ((x11 * ((((x26 * x47)) + ((x29 * x53)))))) + (((-0.0221176390070984) * x1)) + ((x11 * (((((-3.62894267039828e-7) * x0)) + (((7.21647486632227e-8) * x1)))))) + ((x7 * ((((x27 * x39)) + ((x18 * x2 * x33)))))) + ((x6 * ((((x18 * x41)) + ((x22 * x39)))))) + ((x10 * (((((0.0663017633834929) * x0)) + (((-0.013184694620632) * x1)))))) + (((0.111222163942722) * x0)) + ((x24 * x35)) + ((x5 * (((((-0.458031412723242) * x1)) + (((-0.0910836152046622) * x0)))))) + ((x10 * (((((-1.0) * x20 * x47)) + (((-1.0) * x16 * x53)))))));
        eerot[6] = (((x10 * x55 * x9)) + ((x8 * ((((x42 * x6)) + ((x44 * x7)))))));
        eerot[7] = (((x9 * (((((-1.0) * x17 * x42)) + (((-1.0) * x19 * x44)))))) + ((x10 * x8 * ((((x23 * x44)) + ((x25 * x40)))))));
        eerot[8] = (x11 * x55);
        eetrans[2] = ((0.1393) + ((x10 * (((((-1.0) * x20 * x40)) + (((-1.0) * x16 * x44)))))) + ((x11 * ((((x29 * x44)) + ((x26 * x40)))))) + ((x6 * ((((x18 * x2)) + (((-0.1135) * x27)))))) + (((0.467) * x2)) + ((x7 * ((((x18 * x5)) + (((0.1135) * x22)))))) + (((0.452) * x22)) + (((0.452) * x28)));
    }

    IKFAST_API int GetNumFreeParameters() { return 0; }
    IKFAST_API const int *GetFreeIndices() { return NULL; }
    IKFAST_API int GetNumJoints() { return 6; }

    IKFAST_API int GetIkRealSize() { return sizeof(IkReal); }

    IKFAST_API int GetIkType() { return 0x67000001; }

    IKFAST_API bool ComputeIk(const IkReal *eetrans, const IkReal *eerot, const IkReal *pfree, IkSolutionListBase<IkReal> &solutions)
    {
        IKSolver solver;
        return solver.ComputeIk(eetrans, eerot, pfree, solutions);
    }

    IKFAST_API bool ComputeIk2(const IkReal *eetrans, const IkReal *eerot, const IkReal *pfree, IkSolutionListBase<IkReal> &solutions, void *pOpenRAVEManip)
    {
        IKSolver solver;
        return solver.ComputeIk(eetrans, eerot, pfree, solutions);
    }

    IKFAST_API const char *GetKinematicsHash() { return "<robot:GenericRobot - avena (1763be162c49ebf4d9f1a868e1d941f5)>"; }

    IKFAST_API const char *GetIkFastVersion() { return "0x1000004b"; }
} // namespace ik_avena
