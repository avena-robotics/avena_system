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
                new_pz = ((-0.1125) + pz + (((-0.0676) * r22)));
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
                IkReal x61 = ((1.0) * px);
                IkReal x62 = ((1.0) * pz);
                IkReal x63 = ((1.0) * py);
                pp = ((px * px) + (py * py) + (pz * pz));
                npx = (((px * r00)) + ((py * r10)) + ((pz * r20)));
                npy = (((px * r01)) + ((py * r11)) + ((pz * r21)));
                npz = (((px * r02)) + ((py * r12)) + ((pz * r22)));
                rxp0_0 = ((((-1.0) * r20 * x63)) + ((pz * r10)));
                rxp0_1 = (((px * r20)) + (((-1.0) * r00 * x62)));
                rxp0_2 = ((((-1.0) * r10 * x61)) + ((py * r00)));
                rxp1_0 = ((((-1.0) * r21 * x63)) + ((pz * r11)));
                rxp1_1 = (((px * r21)) + (((-1.0) * r01 * x62)));
                rxp1_2 = ((((-1.0) * r11 * x61)) + ((py * r01)));
                rxp2_0 = (((pz * r12)) + (((-1.0) * r22 * x63)));
                rxp2_1 = (((px * r22)) + (((-1.0) * r02 * x62)));
                rxp2_2 = ((((-1.0) * r12 * x61)) + ((py * r02)));
                IkReal op[8 + 1], zeror[8];
                int numroots;
                IkReal x64 = ((-0.105400003330381) + npz);
                IkReal x65 = ((-0.105400003330381) + (((-1.0) * npz)));
                IkReal gconst0 = x64;
                IkReal gconst1 = x65;
                IkReal gconst2 = x64;
                IkReal gconst3 = x65;
                IkReal gconst4 = x64;
                IkReal gconst5 = x65;
                IkReal gconst6 = x64;
                IkReal gconst7 = x65;
                IkReal x66 = r22 * r22;
                IkReal x67 = r21 * r21;
                IkReal x68 = npx * npx;
                IkReal x69 = r20 * r20;
                IkReal x70 = npy * npy;
                IkReal x71 = ((4.0) * npy);
                IkReal x72 = (gconst3 * gconst4);
                IkReal x73 = (gconst1 * gconst2);
                IkReal x74 = (gconst4 * gconst7);
                IkReal x75 = (gconst2 * gconst5);
                IkReal x76 = (gconst0 * gconst2);
                IkReal x77 = (gconst5 * gconst6);
                IkReal x78 = (gconst1 * gconst7);
                IkReal x79 = (gconst5 * gconst7);
                IkReal x80 = (gconst4 * gconst6);
                IkReal x81 = (r21 * r22);
                IkReal x82 = (gconst1 * gconst6);
                IkReal x83 = (gconst1 * gconst3);
                IkReal x84 = (gconst3 * gconst5);
                IkReal x85 = ((8.0) * npy);
                IkReal x86 = (gconst0 * gconst6);
                IkReal x87 = (gconst2 * gconst4);
                IkReal x88 = (gconst0 * gconst7);
                IkReal x89 = (r20 * r22);
                IkReal x90 = (gconst0 * gconst3);
                IkReal x91 = ((2.56e-14) * x66);
                IkReal x92 = ((6.4e-15) * x66);
                IkReal x93 = (gconst2 * x81);
                IkReal x94 = ((1.0) * x66);
                IkReal x95 = (npx * x66);
                IkReal x96 = ((2.0) * x66);
                IkReal x97 = ((8.0) * x67);
                IkReal x98 = ((1.6e-7) * x81);
                IkReal x99 = ((16.0) * x69);
                IkReal x100 = (gconst5 * x89);
                IkReal x101 = ((16.0) * r20 * r21);
                IkReal x102 = ((4.0) * x67);
                IkReal x103 = (npy * x66);
                IkReal x104 = (gconst6 * x98);
                IkReal x105 = (gconst5 * x98);
                IkReal x106 = ((1.6e-7) * x93);
                IkReal x107 = (gconst1 * x98);
                IkReal x108 = ((3.2e-7) * x100);
                IkReal x109 = ((1.92e-6) * x95);
                IkReal x110 = ((3.2e-7) * gconst6 * x89);
                IkReal x111 = ((6.4e-7) * x103);
                IkReal x112 = ((6.4e-7) * x95);
                IkReal x113 = ((3.2e-7) * gconst1 * x89);
                IkReal x114 = ((3.2e-7) * gconst2 * x89);
                IkReal x115 = ((3.2e-7) * x103);
                IkReal x116 = ((8.0) * npx * x81);
                IkReal x117 = (x66 * x70);
                IkReal x118 = ((16.0) * npx * x89);
                IkReal x119 = (x66 * x68);
                IkReal x120 = (gconst6 * x71 * x81);
                IkReal x121 = (gconst5 * x71 * x81);
                IkReal x122 = ((16.0) * npy * x95);
                IkReal x123 = (x101 * x77);
                IkReal x124 = ((16.0) * npx * x100);
                IkReal x125 = (x71 * x93);
                IkReal x126 = (x80 * x94);
                IkReal x127 = (x102 * x77);
                IkReal x128 = (x77 * x94);
                IkReal x129 = (x79 * x94);
                IkReal x130 = (x74 * x94);
                IkReal x131 = (gconst1 * x71 * x81);
                IkReal x132 = (gconst6 * x118);
                IkReal x133 = (gconst5 * x116);
                IkReal x134 = (x100 * x85);
                IkReal x135 = (gconst6 * x85 * x89);
                IkReal x136 = (gconst6 * x116);
                IkReal x137 = (x101 * x82);
                IkReal x138 = (x101 * x75);
                IkReal x139 = (gconst1 * x118);
                IkReal x140 = ((16.0) * x119);
                IkReal x141 = (x88 * x94);
                IkReal x142 = (x75 * x94);
                IkReal x143 = (x78 * x94);
                IkReal x144 = (x87 * x94);
                IkReal x145 = (x102 * x75);
                IkReal x146 = (gconst2 * x118);
                IkReal x147 = (x102 * x82);
                IkReal x148 = (x82 * x94);
                IkReal x149 = (x84 * x94);
                IkReal x150 = (x72 * x94);
                IkReal x151 = (x86 * x94);
                IkReal x152 = (gconst1 * x116);
                IkReal x153 = (gconst1 * x85 * x89);
                IkReal x154 = ((8.0) * npx * x93);
                IkReal x155 = (gconst2 * x85 * x89);
                IkReal x156 = (x101 * x73);
                IkReal x157 = ((4.0) * x117);
                IkReal x158 = (x73 * x94);
                IkReal x159 = (x90 * x94);
                IkReal x160 = (x76 * x94);
                IkReal x161 = (x83 * x94);
                IkReal x162 = (x102 * x73);
                IkReal x163 = (x157 + x92);
                IkReal x164 = (x140 + x91);
                IkReal x165 = (x121 + x104);
                IkReal x166 = (x120 + x106);
                IkReal x167 = (x131 + x104);
                IkReal x168 = (x125 + x105);
                IkReal x169 = (x131 + x107);
                IkReal x170 = (x135 + x136);
                IkReal x171 = (x137 + x138);
                IkReal x172 = (x155 + x154);
                IkReal x173 = (x121 + x111 + x107);
                IkReal x174 = (x134 + x133 + x122);
                IkReal x175 = (x153 + x152 + x122);
                IkReal x176 = (x162 + x160 + x161 + x159 + x158);
                IkReal x177 = (x130 + x126 + x127 + x128 + x129);
                IkReal x178 = (x151 + x150 + x141 + x142 + x143 + x144 + x145 + x147 + x148 + x149);
                op[0] = ((((-1.0) * x177)) + (((-1.0) * x105)) + (((-1.0) * x115)) + (((-1.0) * x120)) + x163 + x165);
                op[1] = ((((-1.0) * x174)) + (((-1.0) * x110)) + x170 + x123 + x112 + x108);
                op[2] = ((((-1.0) * x168)) + (((-1.0) * x178)) + (((-1.0) * x173)) + (((-1.0) * x80 * x96)) + (((-1.0) * x77 * x96)) + (((-1.0) * x77 * x99)) + x166 + x167 + x164 + x124 + (((-1.0) * x132)) + (((-1.0) * x74 * x96)) + (((-1.0) * x79 * x96)) + ((x77 * x97)));
                op[3] = ((((-6.4e-7) * gconst6 * x89)) + (((-1.0) * x175)) + (((-1.0) * x114)) + (((-1.0) * x123)) + x171 + x172 + x113 + x109 + (((6.4e-7) * x100)));
                op[4] = ((((-1.0) * x146)) + (((-1.0) * x82 * x99)) + (((-1.0) * x82 * x96)) + ((x82 * x97)) + (((-1.0) * x169)) + (((-1.0) * x165)) + (((-1.0) * x176)) + (((-1.0) * x177)) + (((-1.0) * x72 * x96)) + x168 + x166 + x139 + x124 + (((-1.0) * x78 * x96)) + (((-1.0) * x87 * x96)) + (((-1.0) * x132)) + (((-1.0) * x86 * x96)) + ((x75 * x97)) + (((-8.0) * x117)) + (((3.84e-14) * x66)) + (((-1.0) * x84 * x96)) + (((-1.0) * x75 * x99)) + (((-1.0) * x75 * x96)) + (((-1.0) * x88 * x96)) + (((32.0) * x119)));
                op[5] = ((((6.4e-7) * gconst1 * x89)) + (((-1.0) * x170)) + (((-1.0) * x171)) + (((-6.4e-7) * gconst2 * x89)) + (((-1.0) * x110)) + x174 + x156 + x108 + x109);
                op[6] = ((((-1.0) * x146)) + (((-1.0) * x167)) + (((-1.0) * x166)) + (((-1.0) * x178)) + (((-1.0) * x73 * x99)) + (((-1.0) * x73 * x96)) + (((-1.0) * x83 * x96)) + (((-1.0) * x90 * x96)) + ((x73 * x97)) + x173 + x168 + x164 + x139 + (((-1.0) * x76 * x96)));
                op[7] = ((((-1.0) * x156)) + (((-1.0) * x172)) + (((-1.0) * x114)) + x175 + x113 + x112);
                op[8] = ((((-1.0) * x176)) + (((-1.0) * x106)) + (((-1.0) * x125)) + x169 + x163 + x115);
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
                        IkReal x179 = (cj5 * r21);
                        IkReal x180 = (r20 * sj5);
                        IkReal x181 = ((1.0) * npz);
                        IkReal x182 = ((((-1.0) * x180 * x181)) + ((npx * r22 * sj5)) + (((4.0e-8) * r22)) + (((-1.0) * x179 * x181)) + ((cj5 * npy * r22)));
                        j4eval[0] = x182;
                        j4eval[1] = ((((-2.54589380025075) * x179)) + (((-2.54589380025075) * x180)));
                        j4eval[2] = IKsign(x182);
                        if (IKabs(j4eval[0]) < 0.0000010000000000 || IKabs(j4eval[1]) < 0.0000010000000000 || IKabs(j4eval[2]) < 0.0000010000000000)
                        {
                            {
                                IkReal evalcond[1];
                                bool bgotonextstatement = true;
                                do
                                {
                                    IkReal x183 = r22 * r22;
                                    IkReal x184 = npz * npz;
                                    IkReal x185 = ((1.0) * npz);
                                    IkReal x186 = (((npx * r22)) + (((-1.0) * r20 * x185)));
                                    IkReal x187 = (((npy * r22)) + (((-1.0) * r21 * x185)));
                                    IkReal x188 = ((((-2.0) * npz * pz * r22)) + x184 + ((x183 * (npx * npx))) + ((x183 * (npy * npy))) + ((x183 * x184)));
                                    if ((x188) < -0.00001)
                                        continue;
                                    IkReal x189 = IKabs(IKsqrt(x188));
                                    IkReal x195 = x188;
                                    if (IKabs(x195) == 0)
                                    {
                                        continue;
                                    }
                                    IkReal x190 = pow(x195, -0.5);
                                    CheckValue<IkReal> x196 = IKPowWithIntegerCheck(x189, -1);
                                    if (!x196.valid)
                                    {
                                        continue;
                                    }
                                    IkReal x191 = x196.value;
                                    if ((((1.0) + (((-1.6e-15) * x183 * (x191 * x191))))) < -0.00001)
                                        continue;
                                    IkReal x192 = IKsqrt(((1.0) + (((-1.6e-15) * x183 * (x191 * x191)))));
                                    IkReal x193 = ((4.0e-8) * r22 * x190 * x191);
                                    IkReal x194 = (x190 * x192);
                                    if ((((x187 * x187) + (x186 * x186))) < -0.00001)
                                        continue;
                                    CheckValue<IkReal> x197 = IKPowWithIntegerCheck(IKabs(IKsqrt(((x187 * x187) + (x186 * x186)))), -1);
                                    if (!x197.valid)
                                    {
                                        continue;
                                    }
                                    if ((((4.0e-8) * r22 * (x197.value))) < -1 - IKFAST_SINCOS_THRESH || (((4.0e-8) * r22 * (x197.value))) > 1 + IKFAST_SINCOS_THRESH)
                                        continue;
                                    CheckValue<IkReal> x198 = IKatan2WithCheck(IkReal(x187), IkReal(x186), IKFAST_ATAN2_MAGTHRESH);
                                    if (!x198.valid)
                                    {
                                        continue;
                                    }
                                    IkReal gconst24 = ((((-1.0) * (IKasin(((4.0e-8) * r22 * (x197.value)))))) + (((-1.0) * (x198.value))));
                                    IkReal gconst25 = ((((-1.0) * x186 * x193)) + (((-1.0) * x187 * x194)));
                                    IkReal gconst26 = (((x186 * x194)) + (((-1.0) * x187 * x193)));
                                    IkReal x199 = r22 * r22;
                                    IkReal x200 = npz * npz;
                                    IkReal x201 = j5;
                                    IkReal x202 = (npx * r22);
                                    IkReal x203 = (npz * r21);
                                    IkReal x204 = (npy * r22);
                                    IkReal x205 = (npz * r20);
                                    CheckValue<IkReal> x210 = IKatan2WithCheck(IkReal((x204 + (((-1.0) * x203)))), IkReal((x202 + (((-1.0) * x205)))), IKFAST_ATAN2_MAGTHRESH);
                                    if (!x210.valid)
                                    {
                                        continue;
                                    }
                                    IkReal x206 = x210.value;
                                    IkReal x207 = x206;
                                    if (((((x199 * (npy * npy))) + ((x200 * (r20 * r20))) + (((-2.0) * x202 * x205)) + ((x199 * (npx * npx))) + ((x200 * (r21 * r21))) + (((-2.0) * x203 * x204)))) < -0.00001)
                                        continue;
                                    CheckValue<IkReal> x211 = IKPowWithIntegerCheck(IKabs(IKsqrt((((x199 * (npy * npy))) + ((x200 * (r20 * r20))) + (((-2.0) * x202 * x205)) + ((x199 * (npx * npx))) + ((x200 * (r21 * r21))) + (((-2.0) * x203 * x204))))), -1);
                                    if (!x211.valid)
                                    {
                                        continue;
                                    }
                                    if ((((4.0e-8) * r22 * (x211.value))) < -1 - IKFAST_SINCOS_THRESH || (((4.0e-8) * r22 * (x211.value))) > 1 + IKFAST_SINCOS_THRESH)
                                        continue;
                                    IkReal x208 = IKasin(((4.0e-8) * r22 * (x211.value)));
                                    IkReal x209 = x208;
                                    if (((((j5 * x207)) + ((j5 * x201)) + ((j5 * x209)) + ((x201 * x208)) + ((x201 * x206)) + ((x208 * x209)) + ((x206 * x209)) + ((x206 * x207)) + ((x207 * x208)))) < -0.00001)
                                        continue;
                                    evalcond[0] = ((-3.14159265358979) + (IKfmod(((3.14159265358979) + (IKsqrt((((j5 * x207)) + ((j5 * x201)) + ((j5 * x209)) + ((x201 * x208)) + ((x201 * x206)) + ((x208 * x209)) + ((x206 * x209)) + ((x206 * x207)) + ((x207 * x208)))))), 6.28318530717959)));
                                    if (IKabs(evalcond[0]) < 0.0000050000000000)
                                    {
                                        bgotonextstatement = false;
                                        {
                                            IkReal j4array[1], cj4array[1], sj4array[1];
                                            bool j4valid[1] = {false};
                                            _nj4 = 1;
                                            IkReal x212 = (gconst26 * r21);
                                            IkReal x213 = ((1.0) * npz);
                                            IkReal x214 = (gconst25 * r20);
                                            CheckValue<IkReal> x215 = IKPowWithIntegerCheck(IKsign((((gconst25 * npx * r22)) + (((4.0e-8) * r22)) + (((-1.0) * x213 * x214)) + ((gconst26 * npy * r22)) + (((-1.0) * x212 * x213)))), -1);
                                            if (!x215.valid)
                                            {
                                                continue;
                                            }
                                            CheckValue<IkReal> x216 = IKatan2WithCheck(IkReal(((-0.105400003330381) * r22)), IkReal(((((-0.105400003330381) * x214)) + (((-0.105400003330381) * x212)))), IKFAST_ATAN2_MAGTHRESH);
                                            if (!x216.valid)
                                            {
                                                continue;
                                            }
                                            j4array[0] = ((-1.5707963267949) + (((1.5707963267949) * (x215.value))) + (x216.value));
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
                                                    IkReal x217 = IKsin(j4);
                                                    IkReal x218 = IKcos(j4);
                                                    IkReal x219 = ((1.0) * gconst25 * x217);
                                                    IkReal x220 = ((1.0) * gconst26 * x217);
                                                    evalcond[0] = ((((-1.0) * r20 * x219)) + (((-1.0) * r21 * x220)) + ((r22 * x218)));
                                                    evalcond[1] = ((-0.105400003330381) + (((-4.0e-8) * x217)) + (((-1.0) * npx * x219)) + ((npz * x218)) + (((-1.0) * npy * x220)));
                                                    if (IKabs(evalcond[0]) > IKFAST_EVALCOND_THRESH || IKabs(evalcond[1]) > IKFAST_EVALCOND_THRESH)
                                                    {
                                                        continue;
                                                    }
                                                }

                                                {
                                                    IkReal j0eval[1];
                                                    IkReal x221 = ((1.01958072531191) * sj4);
                                                    j0eval[0] = ((-1.0) + ((r00 * sj5 * x221)) + (((-1.01958072531191) * cj4 * r02)) + ((cj5 * r01 * x221)));
                                                    if (IKabs(j0eval[0]) < 0.0000010000000000)
                                                    {
                                                        {
                                                            IkReal j0eval[1];
                                                            IkReal x222 = ((5.12715705179976) * sj4);
                                                            j0eval[0] = ((-1.0) + (((5.12715705179976) * cj4 * r12)) + (((-1.0) * r10 * sj5 * x222)) + (((-1.0) * cj5 * r11 * x222)));
                                                            if (IKabs(j0eval[0]) < 0.0000010000000000)
                                                            {
                                                                {
                                                                    IkReal j0eval[1];
                                                                    IkReal x223 = (sj4 * sj5);
                                                                    IkReal x224 = (cj5 * sj4);
                                                                    j0eval[0] = ((((-1.0) * cj4 * r02)) + (((5.02869162246205) * r11 * x224)) + ((r01 * x224)) + ((r00 * x223)) + (((5.02869162246205) * r10 * x223)) + (((-5.02869162246205) * cj4 * r12)));
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
                                                                            IkReal x225 = cj4 * cj4;
                                                                            IkReal x226 = r00 * r00;
                                                                            IkReal x227 = cj5 * cj5;
                                                                            IkReal x228 = r10 * r10;
                                                                            IkReal x229 = r11 * r11;
                                                                            IkReal x230 = r01 * r01;
                                                                            IkReal x231 = ((0.195039861251953) * sj4);
                                                                            IkReal x232 = (r00 * sj5);
                                                                            IkReal x233 = (cj5 * r01);
                                                                            IkReal x234 = ((0.382588364824742) * r12);
                                                                            IkReal x235 = (cj4 * sj4);
                                                                            IkReal x236 = (r10 * sj5);
                                                                            IkReal x237 = ((1.92391890504564) * r12);
                                                                            IkReal x238 = (cj5 * r11);
                                                                            IkReal x239 = ((0.382588364824742) * r02);
                                                                            IkReal x240 = ((0.980795316323859) * sj4);
                                                                            IkReal x241 = ((0.0760810949543624) * r02);
                                                                            IkReal x242 = ((0.382588364824742) * r00 * r10);
                                                                            IkReal x243 = ((0.961959452522819) * x228);
                                                                            IkReal x244 = ((0.0380405474771812) * x226);
                                                                            IkReal x245 = ((0.0380405474771812) * x230);
                                                                            IkReal x246 = ((0.961959452522819) * x229);
                                                                            IkReal x247 = ((0.382588364824742) * r01 * r11);
                                                                            IkReal x248 = ((0.382588364824742) * x225);
                                                                            IkReal x249 = (x225 * x227);
                                                                            CheckValue<IkReal> x253 = IKPowWithIntegerCheck(((((-0.195039861251953) * cj4 * r02)) + ((x238 * x240)) + ((x236 * x240)) + (((-0.980795316323859) * cj4 * r12)) + ((x231 * x233)) + ((x231 * x232))), -1);
                                                                            if (!x253.valid)
                                                                            {
                                                                                continue;
                                                                            }
                                                                            IkReal x250 = x253.value;
                                                                            if ((((1.0) + (((-0.0760810949543624) * x232 * x233)) + (((-0.961959452522819) * x225 * (r12 * r12))) + (((-1.0) * x242 * x249)) + (((-0.382588364824742) * x232 * x238)) + ((x235 * x236 * x239)) + ((x235 * x236 * x237)) + ((x232 * x234 * x235)) + ((x232 * x235 * x241)) + (((1.92391890504564) * x225 * x236 * x238)) + ((x235 * x238 * x239)) + (((-0.382588364824742) * x233 * x236)) + ((x233 * x235 * x241)) + ((x227 * x244)) + ((x227 * x242)) + ((x227 * x243)) + (((-1.0) * x243 * x249)) + ((x235 * x237 * x238)) + (((0.0760810949543624) * x225 * x232 * x233)) + (((-1.0) * r02 * x225 * x234)) + ((x233 * x236 * x248)) + ((x233 * x234 * x235)) + (((-1.92391890504564) * x236 * x238)) + (((-1.0) * x242)) + (((-1.0) * x243)) + (((-1.0) * x244)) + ((x225 * x244)) + ((x225 * x242)) + ((x225 * x243)) + ((x245 * x249)) + (((-1.0) * x227 * x246)) + (((-1.0) * x227 * x247)) + (((-1.0) * x227 * x245)) + ((x246 * x249)) + ((x247 * x249)) + ((x232 * x238 * x248)) + (((-0.0380405474771812) * x225 * (r02 * r02))) + (((-1.0) * x244 * x249)))) < -0.00001)
                                                                                continue;
                                                                            IkReal x251 = IKsqrt(((1.0) + (((-0.0760810949543624) * x232 * x233)) + (((-0.961959452522819) * x225 * (r12 * r12))) + (((-1.0) * x242 * x249)) + (((-0.382588364824742) * x232 * x238)) + ((x235 * x236 * x239)) + ((x235 * x236 * x237)) + ((x232 * x234 * x235)) + ((x232 * x235 * x241)) + (((1.92391890504564) * x225 * x236 * x238)) + ((x235 * x238 * x239)) + (((-0.382588364824742) * x233 * x236)) + ((x233 * x235 * x241)) + ((x227 * x244)) + ((x227 * x242)) + ((x227 * x243)) + (((-1.0) * x243 * x249)) + ((x235 * x237 * x238)) + (((0.0760810949543624) * x225 * x232 * x233)) + (((-1.0) * r02 * x225 * x234)) + ((x233 * x236 * x248)) + ((x233 * x234 * x235)) + (((-1.92391890504564) * x236 * x238)) + (((-1.0) * x242)) + (((-1.0) * x243)) + (((-1.0) * x244)) + ((x225 * x244)) + ((x225 * x242)) + ((x225 * x243)) + ((x245 * x249)) + (((-1.0) * x227 * x246)) + (((-1.0) * x227 * x247)) + (((-1.0) * x227 * x245)) + ((x246 * x249)) + ((x247 * x249)) + ((x232 * x238 * x248)) + (((-0.0380405474771812) * x225 * (r02 * r02))) + (((-1.0) * x244 * x249))));
                                                                            IkReal x252 = (x250 * x251);
                                                                            j0array[0] = ((-2.0) * (atan((x250 + (((-1.0) * x252))))));
                                                                            sj0array[0] = IKsin(j0array[0]);
                                                                            cj0array[0] = IKcos(j0array[0]);
                                                                            j0array[1] = ((-2.0) * (atan((x250 + x252))));
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
                                                                    IkReal x1260 = (cj5 * sj4);
                                                                    IkReal x1261 = (sj4 * sj5);
                                                                    CheckValue<IkReal> x1264 = IKPowWithIntegerCheck(((-0.195039861251953) + (((-1.0) * r10 * x1261)) + (((-1.0) * r11 * x1260)) + ((cj4 * r12))), -1);
                                                                    if (!x1264.valid)
                                                                    {
                                                                        continue;
                                                                    }
                                                                    IkReal x1262 = x1264.value;
                                                                    IkReal x1263 = (sj4 * x1262);
                                                                    CheckValue<IkReal> x1265 = IKPowWithIntegerCheck(((-0.195039861251953) + (((-1.0) * r10 * x1261)) + (((-1.0) * cj5 * r11 * sj4)) + ((cj4 * r12))), -1);
                                                                    if (!x1265.valid)
                                                                    {
                                                                        continue;
                                                                    }
                                                                    CheckValue<IkReal> x1266 = IKPowWithIntegerCheck(((-0.195039861251953) + (((-1.0) * r10 * sj4 * sj5)) + (((-1.0) * r11 * x1260)) + ((cj4 * r12))), -1);
                                                                    if (!x1266.valid)
                                                                    {
                                                                        continue;
                                                                    }
                                                                    j0array[0] = ((2.0) * (atan(((((-1.0) * cj4 * r02 * x1262)) + ((r01 * x1260 * (x1265.value))) + ((r00 * x1261 * (x1266.value))) + (((0.980795316323859) * x1262))))));
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
                                                            IkReal x1267 = ((1.0) * cj4);
                                                            IkReal x1268 = (sj4 * sj5);
                                                            IkReal x1269 = (cj5 * sj4);
                                                            CheckValue<IkReal> x1271 = IKPowWithIntegerCheck(((-0.980795316323859) + (((-1.0) * r02 * x1267)) + ((r00 * x1268)) + ((r01 * x1269))), -1);
                                                            if (!x1271.valid)
                                                            {
                                                                continue;
                                                            }
                                                            IkReal x1270 = x1271.value;
                                                            CheckValue<IkReal> x1272 = IKPowWithIntegerCheck(((-0.980795316323859) + (((-1.0) * cj4 * r02)) + ((r00 * x1268)) + ((r01 * x1269))), -1);
                                                            if (!x1272.valid)
                                                            {
                                                                continue;
                                                            }
                                                            CheckValue<IkReal> x1273 = IKPowWithIntegerCheck(((-0.980795316323859) + ((r00 * sj4 * sj5)) + (((-1.0) * r02 * x1267)) + ((r01 * x1269))), -1);
                                                            if (!x1273.valid)
                                                            {
                                                                continue;
                                                            }
                                                            CheckValue<IkReal> x1274 = IKPowWithIntegerCheck(((-0.980795316323859) + ((cj5 * r01 * sj4)) + (((-1.0) * r02 * x1267)) + ((r00 * x1268))), -1);
                                                            if (!x1274.valid)
                                                            {
                                                                continue;
                                                            }
                                                            j0array[0] = ((-2.0) * (atan(((((0.195039861251953) * x1270)) + ((r12 * x1267 * (x1272.value))) + (((-1.0) * r10 * x1268 * (x1273.value))) + (((-1.0) * r11 * x1269 * (x1274.value)))))));
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
                                        IkReal x1275 = r22 * r22;
                                        IkReal x1276 = npz * npz;
                                        IkReal x1277 = ((4.0e-8) * r22);
                                        IkReal x1278 = ((1.0) * npz);
                                        IkReal x1279 = (((npy * r22)) + (((-1.0) * r21 * x1278)));
                                        IkReal x1280 = (((npx * r22)) + (((-1.0) * r20 * x1278)));
                                        IkReal x1281 = ((((-2.0) * npz * pz * r22)) + x1276 + ((x1275 * x1276)) + ((x1275 * (npx * npx))) + ((x1275 * (npy * npy))));
                                        if ((x1281) < -0.00001)
                                            continue;
                                        IkReal x1282 = IKabs(IKsqrt(x1281));
                                        IkReal x1290 = x1281;
                                        if (IKabs(x1290) == 0)
                                        {
                                            continue;
                                        }
                                        IkReal x1283 = pow(x1290, -0.5);
                                        IkReal x1284 = ((1.0) * x1283);
                                        CheckValue<IkReal> x1291 = IKPowWithIntegerCheck(x1282, -1);
                                        if (!x1291.valid)
                                        {
                                            continue;
                                        }
                                        IkReal x1285 = x1291.value;
                                        CheckValue<IkReal> x1292 = IKPowWithIntegerCheck(x1282, -2);
                                        if (!x1292.valid)
                                        {
                                            continue;
                                        }
                                        IkReal x1286 = x1292.value;
                                        IkReal x1287 = (x1280 * x1283);
                                        IkReal x1288 = (x1275 * x1286);
                                        IkReal x1289 = (x1283 * x1285);
                                        CheckValue<IkReal> x1293 = IKatan2WithCheck(IkReal(x1279), IkReal(x1280), IKFAST_ATAN2_MAGTHRESH);
                                        if (!x1293.valid)
                                        {
                                            continue;
                                        }
                                        if ((((x1280 * x1280) + (x1279 * x1279))) < -0.00001)
                                            continue;
                                        CheckValue<IkReal> x1294 = IKPowWithIntegerCheck(IKabs(IKsqrt(((x1280 * x1280) + (x1279 * x1279)))), -1);
                                        if (!x1294.valid)
                                        {
                                            continue;
                                        }
                                        if (((x1277 * (x1294.value))) < -1 - IKFAST_SINCOS_THRESH || ((x1277 * (x1294.value))) > 1 + IKFAST_SINCOS_THRESH)
                                            continue;
                                        IkReal gconst27 = ((3.14159265358979) + (((-1.0) * (x1293.value))) + (IKasin((x1277 * (x1294.value)))));
                                        if ((((1.0) + (((-1.6e-15) * x1288)))) < -0.00001)
                                            continue;
                                        IkReal gconst28 = (((x1279 * x1284 * (IKsqrt(((1.0) + (((-1.6e-15) * x1288))))))) + (((-1.0) * x1277 * x1285 * x1287)));
                                        if ((((1.0) + (((-1.6e-15) * x1288)))) < -0.00001)
                                            continue;
                                        IkReal gconst29 = ((((-1.0) * x1277 * x1279 * x1289)) + (((-1.0) * x1280 * x1284 * (IKsqrt(((1.0) + (((-1.6e-15) * x1288))))))));
                                        IkReal x1295 = ((1.0) * npz);
                                        IkReal x1296 = x1280;
                                        IkReal x1297 = x1279;
                                        if ((((x1297 * x1297) + (x1296 * x1296))) < -0.00001)
                                            continue;
                                        CheckValue<IkReal> x1298 = IKPowWithIntegerCheck(IKabs(IKsqrt(((x1297 * x1297) + (x1296 * x1296)))), -1);
                                        if (!x1298.valid)
                                        {
                                            continue;
                                        }
                                        if ((((4.0e-8) * r22 * (x1298.value))) < -1 - IKFAST_SINCOS_THRESH || (((4.0e-8) * r22 * (x1298.value))) > 1 + IKFAST_SINCOS_THRESH)
                                            continue;
                                        CheckValue<IkReal> x1299 = IKatan2WithCheck(IkReal(x1297), IkReal(x1296), IKFAST_ATAN2_MAGTHRESH);
                                        if (!x1299.valid)
                                        {
                                            continue;
                                        }
                                        evalcond[0] = ((-3.14159265358979) + (IKfmod(((3.14159265358979) + (IKabs(((-3.14159265358979) + (((-1.0) * (IKasin(((4.0e-8) * r22 * (x1298.value)))))) + (x1299.value) + j5)))), 6.28318530717959)));
                                        if (IKabs(evalcond[0]) < 0.0000050000000000)
                                        {
                                            bgotonextstatement = false;
                                            {
                                                IkReal j4array[1], cj4array[1], sj4array[1];
                                                bool j4valid[1] = {false};
                                                _nj4 = 1;
                                                IkReal x1300 = (gconst29 * r21);
                                                IkReal x1301 = ((1.0) * npz);
                                                IkReal x1302 = (gconst28 * r20);
                                                CheckValue<IkReal> x1303 = IKPowWithIntegerCheck(IKsign(((((-1.0) * x1300 * x1301)) + ((gconst28 * npx * r22)) + (((4.0e-8) * r22)) + (((-1.0) * x1301 * x1302)) + ((gconst29 * npy * r22)))), -1);
                                                if (!x1303.valid)
                                                {
                                                    continue;
                                                }
                                                CheckValue<IkReal> x1304 = IKatan2WithCheck(IkReal(((-0.105400003330381) * r22)), IkReal(((((-0.105400003330381) * x1302)) + (((-0.105400003330381) * x1300)))), IKFAST_ATAN2_MAGTHRESH);
                                                if (!x1304.valid)
                                                {
                                                    continue;
                                                }
                                                j4array[0] = ((-1.5707963267949) + (((1.5707963267949) * (x1303.value))) + (x1304.value));
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
                                                        IkReal x1305 = IKsin(j4);
                                                        IkReal x1306 = IKcos(j4);
                                                        IkReal x1307 = ((1.0) * x1305);
                                                        evalcond[0] = (((r22 * x1306)) + (((-1.0) * gconst28 * r20 * x1307)) + (((-1.0) * gconst29 * r21 * x1307)));
                                                        evalcond[1] = ((-0.105400003330381) + (((-4.0e-8) * x1305)) + (((-1.0) * gconst28 * npx * x1307)) + (((-1.0) * gconst29 * npy * x1307)) + ((npz * x1306)));
                                                        if (IKabs(evalcond[0]) > IKFAST_EVALCOND_THRESH || IKabs(evalcond[1]) > IKFAST_EVALCOND_THRESH)
                                                        {
                                                            continue;
                                                        }
                                                    }

                                                    {
                                                        IkReal j0eval[1];
                                                        IkReal x1308 = ((1.01958072531191) * sj4);
                                                        j0eval[0] = ((-1.0) + (((-1.01958072531191) * cj4 * r02)) + ((r00 * sj5 * x1308)) + ((cj5 * r01 * x1308)));
                                                        if (IKabs(j0eval[0]) < 0.0000010000000000)
                                                        {
                                                            {
                                                                IkReal j0eval[1];
                                                                IkReal x1309 = ((5.12715705179976) * sj4);
                                                                j0eval[0] = ((-1.0) + (((5.12715705179976) * cj4 * r12)) + (((-1.0) * cj5 * r11 * x1309)) + (((-1.0) * r10 * sj5 * x1309)));
                                                                if (IKabs(j0eval[0]) < 0.0000010000000000)
                                                                {
                                                                    {
                                                                        IkReal j0eval[1];
                                                                        IkReal x1310 = (sj4 * sj5);
                                                                        IkReal x1311 = (cj5 * sj4);
                                                                        j0eval[0] = (((r00 * x1310)) + (((-1.0) * cj4 * r02)) + (((5.02869162246205) * r10 * x1310)) + ((r01 * x1311)) + (((-5.02869162246205) * cj4 * r12)) + (((5.02869162246205) * r11 * x1311)));
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
                                                                                IkReal x1312 = cj4 * cj4;
                                                                                IkReal x1313 = r00 * r00;
                                                                                IkReal x1314 = cj5 * cj5;
                                                                                IkReal x1315 = r10 * r10;
                                                                                IkReal x1316 = r11 * r11;
                                                                                IkReal x1317 = r01 * r01;
                                                                                IkReal x1318 = ((0.195039861251953) * sj4);
                                                                                IkReal x1319 = (r00 * sj5);
                                                                                IkReal x1320 = (cj5 * r01);
                                                                                IkReal x1321 = ((0.382588364824742) * r12);
                                                                                IkReal x1322 = (cj4 * sj4);
                                                                                IkReal x1323 = (r10 * sj5);
                                                                                IkReal x1324 = ((1.92391890504564) * r12);
                                                                                IkReal x1325 = (cj5 * r11);
                                                                                IkReal x1326 = ((0.382588364824742) * r02);
                                                                                IkReal x1327 = ((0.980795316323859) * sj4);
                                                                                IkReal x1328 = ((0.0760810949543624) * r02);
                                                                                IkReal x1329 = ((0.382588364824742) * r00 * r10);
                                                                                IkReal x1330 = ((0.961959452522819) * x1315);
                                                                                IkReal x1331 = ((0.0380405474771812) * x1313);
                                                                                IkReal x1332 = ((0.0380405474771812) * x1317);
                                                                                IkReal x1333 = ((0.961959452522819) * x1316);
                                                                                IkReal x1334 = ((0.382588364824742) * r01 * r11);
                                                                                IkReal x1335 = ((0.382588364824742) * x1312);
                                                                                IkReal x1336 = (x1312 * x1314);
                                                                                CheckValue<IkReal> x1340 = IKPowWithIntegerCheck((((x1318 * x1319)) + (((-0.195039861251953) * cj4 * r02)) + ((x1323 * x1327)) + (((-0.980795316323859) * cj4 * r12)) + ((x1325 * x1327)) + ((x1318 * x1320))), -1);
                                                                                if (!x1340.valid)
                                                                                {
                                                                                    continue;
                                                                                }
                                                                                IkReal x1337 = x1340.value;
                                                                                if ((((1.0) + ((x1312 * x1329)) + ((x1312 * x1330)) + ((x1312 * x1331)) + (((-0.0760810949543624) * x1319 * x1320)) + (((-1.92391890504564) * x1323 * x1325)) + ((x1322 * x1325 * x1326)) + ((x1314 * x1329)) + (((-1.0) * x1329)) + ((x1319 * x1322 * x1328)) + (((-0.382588364824742) * x1320 * x1323)) + (((-1.0) * x1330 * x1336)) + (((-1.0) * x1329 * x1336)) + (((0.0760810949543624) * x1312 * x1319 * x1320)) + (((-1.0) * x1331 * x1336)) + (((1.92391890504564) * x1312 * x1323 * x1325)) + (((-0.382588364824742) * x1319 * x1325)) + ((x1320 * x1323 * x1335)) + (((-1.0) * r02 * x1312 * x1321)) + (((-1.0) * x1314 * x1334)) + (((-1.0) * x1314 * x1333)) + (((-1.0) * x1314 * x1332)) + ((x1319 * x1321 * x1322)) + (((-0.0380405474771812) * x1312 * (r02 * r02))) + ((x1322 * x1323 * x1324)) + ((x1322 * x1323 * x1326)) + (((-0.961959452522819) * x1312 * (r12 * r12))) + ((x1320 * x1321 * x1322)) + ((x1320 * x1322 * x1328)) + (((-1.0) * x1331)) + (((-1.0) * x1330)) + ((x1333 * x1336)) + ((x1322 * x1324 * x1325)) + ((x1314 * x1330)) + ((x1314 * x1331)) + ((x1334 * x1336)) + ((x1332 * x1336)) + ((x1319 * x1325 * x1335)))) < -0.00001)
                                                                                    continue;
                                                                                IkReal x1338 = IKsqrt(((1.0) + ((x1312 * x1329)) + ((x1312 * x1330)) + ((x1312 * x1331)) + (((-0.0760810949543624) * x1319 * x1320)) + (((-1.92391890504564) * x1323 * x1325)) + ((x1322 * x1325 * x1326)) + ((x1314 * x1329)) + (((-1.0) * x1329)) + ((x1319 * x1322 * x1328)) + (((-0.382588364824742) * x1320 * x1323)) + (((-1.0) * x1330 * x1336)) + (((-1.0) * x1329 * x1336)) + (((0.0760810949543624) * x1312 * x1319 * x1320)) + (((-1.0) * x1331 * x1336)) + (((1.92391890504564) * x1312 * x1323 * x1325)) + (((-0.382588364824742) * x1319 * x1325)) + ((x1320 * x1323 * x1335)) + (((-1.0) * r02 * x1312 * x1321)) + (((-1.0) * x1314 * x1334)) + (((-1.0) * x1314 * x1333)) + (((-1.0) * x1314 * x1332)) + ((x1319 * x1321 * x1322)) + (((-0.0380405474771812) * x1312 * (r02 * r02))) + ((x1322 * x1323 * x1324)) + ((x1322 * x1323 * x1326)) + (((-0.961959452522819) * x1312 * (r12 * r12))) + ((x1320 * x1321 * x1322)) + ((x1320 * x1322 * x1328)) + (((-1.0) * x1331)) + (((-1.0) * x1330)) + ((x1333 * x1336)) + ((x1322 * x1324 * x1325)) + ((x1314 * x1330)) + ((x1314 * x1331)) + ((x1334 * x1336)) + ((x1332 * x1336)) + ((x1319 * x1325 * x1335))));
                                                                                IkReal x1339 = (x1337 * x1338);
                                                                                j0array[0] = ((2.0) * (atan(((((1.0) * x1339)) + (((-1.0) * x1337))))));
                                                                                sj0array[0] = IKsin(j0array[0]);
                                                                                cj0array[0] = IKcos(j0array[0]);
                                                                                j0array[1] = ((-2.0) * (atan((x1339 + x1337))));
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
                                                                        IkReal x1341 = (cj5 * sj4);
                                                                        IkReal x1342 = (sj4 * sj5);
                                                                        CheckValue<IkReal> x1345 = IKPowWithIntegerCheck(((-0.195039861251953) + (((-1.0) * r10 * x1342)) + (((-1.0) * r11 * x1341)) + ((cj4 * r12))), -1);
                                                                        if (!x1345.valid)
                                                                        {
                                                                            continue;
                                                                        }
                                                                        IkReal x1343 = x1345.value;
                                                                        IkReal x1344 = (sj4 * x1343);
                                                                        CheckValue<IkReal> x1346 = IKPowWithIntegerCheck(((-0.195039861251953) + (((-1.0) * r10 * sj4 * sj5)) + (((-1.0) * r11 * x1341)) + ((cj4 * r12))), -1);
                                                                        if (!x1346.valid)
                                                                        {
                                                                            continue;
                                                                        }
                                                                        CheckValue<IkReal> x1347 = IKPowWithIntegerCheck(((-0.195039861251953) + (((-1.0) * r10 * x1342)) + (((-1.0) * cj5 * r11 * sj4)) + ((cj4 * r12))), -1);
                                                                        if (!x1347.valid)
                                                                        {
                                                                            continue;
                                                                        }
                                                                        j0array[0] = ((2.0) * (atan(((((0.980795316323859) * x1343)) + ((r00 * x1342 * (x1346.value))) + (((-1.0) * cj4 * r02 * x1343)) + ((r01 * x1341 * (x1347.value)))))));
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
                                                                IkReal x1348 = ((1.0) * cj4);
                                                                IkReal x1349 = (sj4 * sj5);
                                                                IkReal x1350 = (cj5 * sj4);
                                                                CheckValue<IkReal> x1352 = IKPowWithIntegerCheck(((-0.980795316323859) + (((-1.0) * r02 * x1348)) + ((r00 * x1349)) + ((r01 * x1350))), -1);
                                                                if (!x1352.valid)
                                                                {
                                                                    continue;
                                                                }
                                                                IkReal x1351 = x1352.value;
                                                                CheckValue<IkReal> x1353 = IKPowWithIntegerCheck(((-0.980795316323859) + ((r00 * sj4 * sj5)) + (((-1.0) * r02 * x1348)) + ((r01 * x1350))), -1);
                                                                if (!x1353.valid)
                                                                {
                                                                    continue;
                                                                }
                                                                CheckValue<IkReal> x1354 = IKPowWithIntegerCheck(((-0.980795316323859) + (((-1.0) * cj4 * r02)) + ((r00 * x1349)) + ((r01 * x1350))), -1);
                                                                if (!x1354.valid)
                                                                {
                                                                    continue;
                                                                }
                                                                CheckValue<IkReal> x1355 = IKPowWithIntegerCheck(((-0.980795316323859) + (((-1.0) * r02 * x1348)) + ((cj5 * r01 * sj4)) + ((r00 * x1349))), -1);
                                                                if (!x1355.valid)
                                                                {
                                                                    continue;
                                                                }
                                                                j0array[0] = ((-2.0) * (atan(((((-1.0) * r10 * x1349 * (x1353.value))) + ((r12 * x1348 * (x1354.value))) + (((-1.0) * r11 * x1350 * (x1355.value))) + (((0.195039861251953) * x1351))))));
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
                                                    IkReal x1356 = ((-1.0) * pz);
                                                    r20 = 0;
                                                    r21 = 0;
                                                    r02 = 0;
                                                    r12 = 0;
                                                    npx = (((px * r00)) + ((py * r10)));
                                                    npy = (((px * r01)) + ((py * r11)));
                                                    npz = (pz * r22);
                                                    rxp0_0 = (pz * r10);
                                                    rxp0_1 = (r00 * x1356);
                                                    rxp1_0 = (pz * r11);
                                                    rxp1_1 = (r01 * x1356);
                                                    rxp2_0 = ((-1.0) * py * r22);
                                                    rxp2_1 = (px * r22);
                                                    rxp2_2 = 0;
                                                    IkReal x1357 = sj5 * sj5;
                                                    IkReal x1358 = cj5 * cj5;
                                                    IkReal x1359 = py * py;
                                                    IkReal x1360 = px * px;
                                                    IkReal x1361 = (cj5 * r01);
                                                    IkReal x1362 = ((1.25e+15) * sj5);
                                                    IkReal x1363 = ((50000000.0) * px);
                                                    IkReal x1364 = (py * r10);
                                                    IkReal x1365 = (px * r00);
                                                    IkReal x1366 = ((1.0) * sj5);
                                                    IkReal x1367 = ((625000000000000.0) * x1359);
                                                    IkReal x1368 = (cj5 * py * r11);
                                                    IkReal x1369 = ((625000000000000.0) * x1360);
                                                    j4eval[0] = ((IKabs((pz * r22))) + (IKabs(((-4.0e-8) + (((-1.0) * x1364 * x1366)) + (((-1.0) * x1368)) + (((-1.0) * px * x1361)) + (((-1.0) * x1365 * x1366))))));
                                                    j4eval[1] = ((1.0) + ((px * x1361 * x1362 * x1364)) + (((50000000.0) * sj5 * x1364)) + ((x1361 * x1363)) + ((x1358 * x1367 * (r11 * r11))) + (((1.25e+15) * px * py * r01 * r11 * x1358)) + (((625000000000000.0) * (pz * pz) * (r22 * r22))) + ((r00 * x1360 * x1361 * x1362)) + ((x1362 * x1365 * x1368)) + ((cj5 * r10 * r11 * x1359 * x1362)) + (((50000000.0) * x1368)) + ((x1358 * x1369 * (r01 * r01))) + ((r00 * sj5 * x1363)) + ((x1357 * x1369 * (r00 * r00))) + (((1.25e+15) * x1357 * x1364 * x1365)) + ((x1357 * x1367 * (r10 * r10))));
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
                                                            IkReal x1370 = ((1.0) * cj5);
                                                            IkReal x1371 = ((1.0) * sj5);
                                                            IkReal x1372 = ((-4.0e-8) + (((-1.0) * px * r00 * x1371)) + (((-1.0) * px * r01 * x1370)) + (((-1.0) * py * r10 * x1371)) + (((-1.0) * py * r11 * x1370)));
                                                            CheckValue<IkReal> x1375 = IKatan2WithCheck(IkReal((pz * r22)), IkReal(x1372), IKFAST_ATAN2_MAGTHRESH);
                                                            if (!x1375.valid)
                                                            {
                                                                continue;
                                                            }
                                                            IkReal x1373 = ((1.0) * (x1375.value));
                                                            if ((((x1372 * x1372) + (((pz * pz) * (r22 * r22))))) < -0.00001)
                                                                continue;
                                                            CheckValue<IkReal> x1376 = IKPowWithIntegerCheck(IKabs(IKsqrt(((x1372 * x1372) + (((pz * pz) * (r22 * r22)))))), -1);
                                                            if (!x1376.valid)
                                                            {
                                                                continue;
                                                            }
                                                            if ((((0.105400003330381) * (x1376.value))) < -1 - IKFAST_SINCOS_THRESH || (((0.105400003330381) * (x1376.value))) > 1 + IKFAST_SINCOS_THRESH)
                                                                continue;
                                                            IkReal x1374 = IKasin(((0.105400003330381) * (x1376.value)));
                                                            j4array[0] = (x1374 + (((-1.0) * x1373)));
                                                            sj4array[0] = IKsin(j4array[0]);
                                                            cj4array[0] = IKcos(j4array[0]);
                                                            j4array[1] = ((3.14159265358979) + (((-1.0) * x1374)) + (((-1.0) * x1373)));
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
                                                                    IkReal x1377 = ((1.01958072531191) * sj4);
                                                                    j0eval[0] = ((-1.0) + (((-1.01958072531191) * cj4 * r02)) + ((r00 * sj5 * x1377)) + ((cj5 * r01 * x1377)));
                                                                    if (IKabs(j0eval[0]) < 0.0000010000000000)
                                                                    {
                                                                        {
                                                                            IkReal j0eval[1];
                                                                            IkReal x1378 = ((5.12715705179976) * sj4);
                                                                            j0eval[0] = ((-1.0) + (((-1.0) * cj5 * r11 * x1378)) + (((5.12715705179976) * cj4 * r12)) + (((-1.0) * r10 * sj5 * x1378)));
                                                                            if (IKabs(j0eval[0]) < 0.0000010000000000)
                                                                            {
                                                                                {
                                                                                    IkReal j0eval[1];
                                                                                    IkReal x1379 = (sj4 * sj5);
                                                                                    IkReal x1380 = (cj5 * sj4);
                                                                                    j0eval[0] = ((((-1.0) * cj4 * r02)) + (((5.02869162246205) * r11 * x1380)) + (((5.02869162246205) * r10 * x1379)) + (((-5.02869162246205) * cj4 * r12)) + ((r01 * x1380)) + ((r00 * x1379)));
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
                                                                                            IkReal x1381 = cj4 * cj4;
                                                                                            IkReal x1382 = r00 * r00;
                                                                                            IkReal x1383 = cj5 * cj5;
                                                                                            IkReal x1384 = r10 * r10;
                                                                                            IkReal x1385 = r11 * r11;
                                                                                            IkReal x1386 = r01 * r01;
                                                                                            IkReal x1387 = ((0.195039861251953) * sj4);
                                                                                            IkReal x1388 = (r00 * sj5);
                                                                                            IkReal x1389 = (cj5 * r01);
                                                                                            IkReal x1390 = ((0.382588364824742) * r12);
                                                                                            IkReal x1391 = (cj4 * sj4);
                                                                                            IkReal x1392 = (r10 * sj5);
                                                                                            IkReal x1393 = ((1.92391890504564) * r12);
                                                                                            IkReal x1394 = (cj5 * r11);
                                                                                            IkReal x1395 = ((0.382588364824742) * r02);
                                                                                            IkReal x1396 = ((0.980795316323859) * sj4);
                                                                                            IkReal x1397 = ((0.0760810949543624) * r02);
                                                                                            IkReal x1398 = ((0.382588364824742) * r00 * r10);
                                                                                            IkReal x1399 = ((0.961959452522819) * x1384);
                                                                                            IkReal x1400 = ((0.0380405474771812) * x1382);
                                                                                            IkReal x1401 = ((0.0380405474771812) * x1386);
                                                                                            IkReal x1402 = ((0.961959452522819) * x1385);
                                                                                            IkReal x1403 = ((0.382588364824742) * r01 * r11);
                                                                                            IkReal x1404 = ((0.382588364824742) * x1381);
                                                                                            IkReal x1405 = (x1381 * x1383);
                                                                                            CheckValue<IkReal> x1409 = IKPowWithIntegerCheck(((((-0.195039861251953) * cj4 * r02)) + ((x1394 * x1396)) + (((-0.980795316323859) * cj4 * r12)) + ((x1387 * x1388)) + ((x1387 * x1389)) + ((x1392 * x1396))), -1);
                                                                                            if (!x1409.valid)
                                                                                            {
                                                                                                continue;
                                                                                            }
                                                                                            IkReal x1406 = x1409.value;
                                                                                            if ((((1.0) + (((1.92391890504564) * x1381 * x1392 * x1394)) + (((-0.382588364824742) * x1389 * x1392)) + ((x1391 * x1392 * x1393)) + ((x1391 * x1392 * x1395)) + ((x1388 * x1391 * x1397)) + ((x1391 * x1393 * x1394)) + (((-1.0) * x1399 * x1405)) + (((-0.382588364824742) * x1388 * x1394)) + ((x1391 * x1394 * x1395)) + (((-0.961959452522819) * x1381 * (r12 * r12))) + ((x1389 * x1390 * x1391)) + (((-1.0) * x1398 * x1405)) + (((-1.0) * x1400)) + (((-0.0380405474771812) * x1381 * (r02 * r02))) + ((x1388 * x1394 * x1404)) + (((-1.0) * x1383 * x1403)) + (((-1.0) * x1383 * x1401)) + (((-1.0) * x1383 * x1402)) + (((-0.0760810949543624) * x1388 * x1389)) + ((x1403 * x1405)) + (((0.0760810949543624) * x1381 * x1388 * x1389)) + ((x1389 * x1392 * x1404)) + (((-1.0) * x1400 * x1405)) + (((-1.0) * r02 * x1381 * x1390)) + (((-1.92391890504564) * x1392 * x1394)) + ((x1388 * x1390 * x1391)) + ((x1402 * x1405)) + ((x1381 * x1400)) + ((x1389 * x1391 * x1397)) + ((x1401 * x1405)) + ((x1383 * x1399)) + ((x1383 * x1398)) + ((x1383 * x1400)) + ((x1381 * x1399)) + ((x1381 * x1398)) + (((-1.0) * x1398)) + (((-1.0) * x1399)))) < -0.00001)
                                                                                                continue;
                                                                                            IkReal x1407 = IKsqrt(((1.0) + (((1.92391890504564) * x1381 * x1392 * x1394)) + (((-0.382588364824742) * x1389 * x1392)) + ((x1391 * x1392 * x1393)) + ((x1391 * x1392 * x1395)) + ((x1388 * x1391 * x1397)) + ((x1391 * x1393 * x1394)) + (((-1.0) * x1399 * x1405)) + (((-0.382588364824742) * x1388 * x1394)) + ((x1391 * x1394 * x1395)) + (((-0.961959452522819) * x1381 * (r12 * r12))) + ((x1389 * x1390 * x1391)) + (((-1.0) * x1398 * x1405)) + (((-1.0) * x1400)) + (((-0.0380405474771812) * x1381 * (r02 * r02))) + ((x1388 * x1394 * x1404)) + (((-1.0) * x1383 * x1403)) + (((-1.0) * x1383 * x1401)) + (((-1.0) * x1383 * x1402)) + (((-0.0760810949543624) * x1388 * x1389)) + ((x1403 * x1405)) + (((0.0760810949543624) * x1381 * x1388 * x1389)) + ((x1389 * x1392 * x1404)) + (((-1.0) * x1400 * x1405)) + (((-1.0) * r02 * x1381 * x1390)) + (((-1.92391890504564) * x1392 * x1394)) + ((x1388 * x1390 * x1391)) + ((x1402 * x1405)) + ((x1381 * x1400)) + ((x1389 * x1391 * x1397)) + ((x1401 * x1405)) + ((x1383 * x1399)) + ((x1383 * x1398)) + ((x1383 * x1400)) + ((x1381 * x1399)) + ((x1381 * x1398)) + (((-1.0) * x1398)) + (((-1.0) * x1399))));
                                                                                            IkReal x1408 = (x1406 * x1407);
                                                                                            j0array[0] = ((2.0) * (atan(((((1.0) * x1408)) + (((-1.0) * x1406))))));
                                                                                            sj0array[0] = IKsin(j0array[0]);
                                                                                            cj0array[0] = IKcos(j0array[0]);
                                                                                            j0array[1] = ((-2.0) * (atan((x1408 + x1406))));
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
                                                                                    IkReal x1410 = (cj5 * sj4);
                                                                                    IkReal x1411 = (sj4 * sj5);
                                                                                    CheckValue<IkReal> x1414 = IKPowWithIntegerCheck(((-0.195039861251953) + (((-1.0) * r10 * x1411)) + (((-1.0) * r11 * x1410)) + ((cj4 * r12))), -1);
                                                                                    if (!x1414.valid)
                                                                                    {
                                                                                        continue;
                                                                                    }
                                                                                    IkReal x1412 = x1414.value;
                                                                                    IkReal x1413 = (sj4 * x1412);
                                                                                    CheckValue<IkReal> x1415 = IKPowWithIntegerCheck(((-0.195039861251953) + (((-1.0) * r10 * x1411)) + (((-1.0) * cj5 * r11 * sj4)) + ((cj4 * r12))), -1);
                                                                                    if (!x1415.valid)
                                                                                    {
                                                                                        continue;
                                                                                    }
                                                                                    CheckValue<IkReal> x1416 = IKPowWithIntegerCheck(((-0.195039861251953) + (((-1.0) * r10 * sj4 * sj5)) + (((-1.0) * r11 * x1410)) + ((cj4 * r12))), -1);
                                                                                    if (!x1416.valid)
                                                                                    {
                                                                                        continue;
                                                                                    }
                                                                                    j0array[0] = ((2.0) * (atan((((r01 * x1410 * (x1415.value))) + (((-1.0) * cj4 * r02 * x1412)) + ((r00 * x1411 * (x1416.value))) + (((0.980795316323859) * x1412))))));
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
                                                                            IkReal x1417 = ((1.0) * cj4);
                                                                            IkReal x1418 = (sj4 * sj5);
                                                                            IkReal x1419 = (cj5 * sj4);
                                                                            CheckValue<IkReal> x1421 = IKPowWithIntegerCheck(((-0.980795316323859) + (((-1.0) * r02 * x1417)) + ((r00 * x1418)) + ((r01 * x1419))), -1);
                                                                            if (!x1421.valid)
                                                                            {
                                                                                continue;
                                                                            }
                                                                            IkReal x1420 = x1421.value;
                                                                            CheckValue<IkReal> x1422 = IKPowWithIntegerCheck(((-0.980795316323859) + ((r00 * sj4 * sj5)) + (((-1.0) * r02 * x1417)) + ((r01 * x1419))), -1);
                                                                            if (!x1422.valid)
                                                                            {
                                                                                continue;
                                                                            }
                                                                            CheckValue<IkReal> x1423 = IKPowWithIntegerCheck(((-0.980795316323859) + ((cj5 * r01 * sj4)) + (((-1.0) * r02 * x1417)) + ((r00 * x1418))), -1);
                                                                            if (!x1423.valid)
                                                                            {
                                                                                continue;
                                                                            }
                                                                            CheckValue<IkReal> x1424 = IKPowWithIntegerCheck(((-0.980795316323859) + (((-1.0) * cj4 * r02)) + ((r00 * x1418)) + ((r01 * x1419))), -1);
                                                                            if (!x1424.valid)
                                                                            {
                                                                                continue;
                                                                            }
                                                                            j0array[0] = ((2.0) * (atan((((r10 * x1418 * (x1422.value))) + ((r11 * x1419 * (x1423.value))) + (((-1.0) * r12 * x1417 * (x1424.value))) + (((-0.195039861251953) * x1420))))));
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
                                IkReal x1425 = (r20 * sj5);
                                IkReal x1426 = (cj5 * r21);
                                IkReal x1427 = ((1.0) * npz);
                                CheckValue<IkReal> x1428 = IKatan2WithCheck(IkReal(((-0.105400003330381) * r22)), IkReal(((((-0.105400003330381) * x1426)) + (((-0.105400003330381) * x1425)))), IKFAST_ATAN2_MAGTHRESH);
                                if (!x1428.valid)
                                {
                                    continue;
                                }
                                CheckValue<IkReal> x1429 = IKPowWithIntegerCheck(IKsign(((((-1.0) * x1426 * x1427)) + (((-1.0) * x1425 * x1427)) + ((npx * r22 * sj5)) + (((4.0e-8) * r22)) + ((cj5 * npy * r22)))), -1);
                                if (!x1429.valid)
                                {
                                    continue;
                                }
                                j4array[0] = ((-1.5707963267949) + (x1428.value) + (((1.5707963267949) * (x1429.value))));
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
                                        IkReal x1430 = IKsin(j4);
                                        IkReal x1431 = IKcos(j4);
                                        IkReal x1432 = ((1.0) * x1430);
                                        evalcond[0] = ((((-1.0) * cj5 * r21 * x1432)) + (((-1.0) * r20 * sj5 * x1432)) + ((r22 * x1431)));
                                        evalcond[1] = ((-0.105400003330381) + (((-4.0e-8) * x1430)) + (((-1.0) * npx * sj5 * x1432)) + (((-1.0) * cj5 * npy * x1432)) + ((npz * x1431)));
                                        if (IKabs(evalcond[0]) > IKFAST_EVALCOND_THRESH || IKabs(evalcond[1]) > IKFAST_EVALCOND_THRESH)
                                        {
                                            continue;
                                        }
                                    }

                                    {
                                        IkReal j0eval[1];
                                        IkReal x1433 = ((1.01958072531191) * sj4);
                                        j0eval[0] = ((-1.0) + (((-1.01958072531191) * cj4 * r02)) + ((r00 * sj5 * x1433)) + ((cj5 * r01 * x1433)));
                                        if (IKabs(j0eval[0]) < 0.0000010000000000)
                                        {
                                            {
                                                IkReal j0eval[1];
                                                IkReal x1434 = ((5.12715705179976) * sj4);
                                                j0eval[0] = ((-1.0) + (((5.12715705179976) * cj4 * r12)) + (((-1.0) * r10 * sj5 * x1434)) + (((-1.0) * cj5 * r11 * x1434)));
                                                if (IKabs(j0eval[0]) < 0.0000010000000000)
                                                {
                                                    {
                                                        IkReal j0eval[1];
                                                        IkReal x1435 = (sj4 * sj5);
                                                        IkReal x1436 = (cj5 * sj4);
                                                        j0eval[0] = ((((-1.0) * cj4 * r02)) + (((5.02869162246205) * r10 * x1435)) + (((5.02869162246205) * r11 * x1436)) + ((r01 * x1436)) + ((r00 * x1435)) + (((-5.02869162246205) * cj4 * r12)));
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
                                                                IkReal x1437 = cj4 * cj4;
                                                                IkReal x1438 = r00 * r00;
                                                                IkReal x1439 = cj5 * cj5;
                                                                IkReal x1440 = r10 * r10;
                                                                IkReal x1441 = r11 * r11;
                                                                IkReal x1442 = r01 * r01;
                                                                IkReal x1443 = ((0.195039861251953) * sj4);
                                                                IkReal x1444 = (r00 * sj5);
                                                                IkReal x1445 = (cj5 * r01);
                                                                IkReal x1446 = ((0.382588364824742) * r12);
                                                                IkReal x1447 = (cj4 * sj4);
                                                                IkReal x1448 = (r10 * sj5);
                                                                IkReal x1449 = ((1.92391890504564) * r12);
                                                                IkReal x1450 = (cj5 * r11);
                                                                IkReal x1451 = ((0.382588364824742) * r02);
                                                                IkReal x1452 = ((0.980795316323859) * sj4);
                                                                IkReal x1453 = ((0.0760810949543624) * r02);
                                                                IkReal x1454 = ((0.382588364824742) * r00 * r10);
                                                                IkReal x1455 = ((0.961959452522819) * x1440);
                                                                IkReal x1456 = ((0.0380405474771812) * x1438);
                                                                IkReal x1457 = ((0.0380405474771812) * x1442);
                                                                IkReal x1458 = ((0.961959452522819) * x1441);
                                                                IkReal x1459 = ((0.382588364824742) * r01 * r11);
                                                                IkReal x1460 = ((0.382588364824742) * x1437);
                                                                IkReal x1461 = (x1437 * x1439);
                                                                CheckValue<IkReal> x1465 = IKPowWithIntegerCheck(((((-0.195039861251953) * cj4 * r02)) + ((x1443 * x1444)) + ((x1443 * x1445)) + ((x1450 * x1452)) + ((x1448 * x1452)) + (((-0.980795316323859) * cj4 * r12))), -1);
                                                                if (!x1465.valid)
                                                                {
                                                                    continue;
                                                                }
                                                                IkReal x1462 = x1465.value;
                                                                if ((((1.0) + ((x1445 * x1446 * x1447)) + ((x1447 * x1448 * x1451)) + (((-0.382588364824742) * x1444 * x1450)) + ((x1445 * x1448 * x1460)) + ((x1447 * x1449 * x1450)) + (((-0.0760810949543624) * x1444 * x1445)) + (((-1.0) * x1456)) + (((-1.0) * x1455)) + (((-1.0) * x1454)) + ((x1444 * x1450 * x1460)) + (((-1.0) * x1456 * x1461)) + ((x1437 * x1455)) + ((x1437 * x1454)) + ((x1437 * x1456)) + (((-1.0) * x1454 * x1461)) + ((x1459 * x1461)) + ((x1439 * x1456)) + ((x1439 * x1454)) + ((x1439 * x1455)) + ((x1458 * x1461)) + (((-1.0) * x1455 * x1461)) + ((x1444 * x1446 * x1447)) + ((x1447 * x1450 * x1451)) + (((-0.961959452522819) * x1437 * (r12 * r12))) + (((1.92391890504564) * x1437 * x1448 * x1450)) + ((x1457 * x1461)) + (((-0.382588364824742) * x1445 * x1448)) + ((x1445 * x1447 * x1453)) + (((-1.0) * x1439 * x1457)) + (((-1.0) * x1439 * x1458)) + (((-1.0) * x1439 * x1459)) + (((-0.0380405474771812) * x1437 * (r02 * r02))) + (((-1.92391890504564) * x1448 * x1450)) + ((x1444 * x1447 * x1453)) + ((x1447 * x1448 * x1449)) + (((-1.0) * r02 * x1437 * x1446)) + (((0.0760810949543624) * x1437 * x1444 * x1445)))) < -0.00001)
                                                                    continue;
                                                                IkReal x1463 = IKsqrt(((1.0) + ((x1445 * x1446 * x1447)) + ((x1447 * x1448 * x1451)) + (((-0.382588364824742) * x1444 * x1450)) + ((x1445 * x1448 * x1460)) + ((x1447 * x1449 * x1450)) + (((-0.0760810949543624) * x1444 * x1445)) + (((-1.0) * x1456)) + (((-1.0) * x1455)) + (((-1.0) * x1454)) + ((x1444 * x1450 * x1460)) + (((-1.0) * x1456 * x1461)) + ((x1437 * x1455)) + ((x1437 * x1454)) + ((x1437 * x1456)) + (((-1.0) * x1454 * x1461)) + ((x1459 * x1461)) + ((x1439 * x1456)) + ((x1439 * x1454)) + ((x1439 * x1455)) + ((x1458 * x1461)) + (((-1.0) * x1455 * x1461)) + ((x1444 * x1446 * x1447)) + ((x1447 * x1450 * x1451)) + (((-0.961959452522819) * x1437 * (r12 * r12))) + (((1.92391890504564) * x1437 * x1448 * x1450)) + ((x1457 * x1461)) + (((-0.382588364824742) * x1445 * x1448)) + ((x1445 * x1447 * x1453)) + (((-1.0) * x1439 * x1457)) + (((-1.0) * x1439 * x1458)) + (((-1.0) * x1439 * x1459)) + (((-0.0380405474771812) * x1437 * (r02 * r02))) + (((-1.92391890504564) * x1448 * x1450)) + ((x1444 * x1447 * x1453)) + ((x1447 * x1448 * x1449)) + (((-1.0) * r02 * x1437 * x1446)) + (((0.0760810949543624) * x1437 * x1444 * x1445))));
                                                                IkReal x1464 = (x1462 * x1463);
                                                                j0array[0] = ((-2.0) * (atan((x1462 + (((-1.0) * x1464))))));
                                                                sj0array[0] = IKsin(j0array[0]);
                                                                cj0array[0] = IKcos(j0array[0]);
                                                                j0array[1] = ((-2.0) * (atan((x1464 + x1462))));
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
                                                        IkReal x1466 = (cj5 * sj4);
                                                        IkReal x1467 = (sj4 * sj5);
                                                        CheckValue<IkReal> x1470 = IKPowWithIntegerCheck(((-0.195039861251953) + (((-1.0) * r11 * x1466)) + (((-1.0) * r10 * x1467)) + ((cj4 * r12))), -1);
                                                        if (!x1470.valid)
                                                        {
                                                            continue;
                                                        }
                                                        IkReal x1468 = x1470.value;
                                                        IkReal x1469 = (sj4 * x1468);
                                                        CheckValue<IkReal> x1471 = IKPowWithIntegerCheck(((-0.195039861251953) + (((-1.0) * r10 * x1467)) + (((-1.0) * cj5 * r11 * sj4)) + ((cj4 * r12))), -1);
                                                        if (!x1471.valid)
                                                        {
                                                            continue;
                                                        }
                                                        CheckValue<IkReal> x1472 = IKPowWithIntegerCheck(((-0.195039861251953) + (((-1.0) * r11 * x1466)) + (((-1.0) * r10 * sj4 * sj5)) + ((cj4 * r12))), -1);
                                                        if (!x1472.valid)
                                                        {
                                                            continue;
                                                        }
                                                        j0array[0] = ((2.0) * (atan(((((-1.0) * cj4 * r02 * x1468)) + ((r01 * x1466 * (x1471.value))) + ((r00 * x1467 * (x1472.value))) + (((0.980795316323859) * x1468))))));
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
                                                IkReal x1473 = ((1.0) * cj4);
                                                IkReal x1474 = (sj4 * sj5);
                                                IkReal x1475 = (cj5 * sj4);
                                                CheckValue<IkReal> x1477 = IKPowWithIntegerCheck(((-0.980795316323859) + (((-1.0) * r02 * x1473)) + ((r01 * x1475)) + ((r00 * x1474))), -1);
                                                if (!x1477.valid)
                                                {
                                                    continue;
                                                }
                                                IkReal x1476 = x1477.value;
                                                CheckValue<IkReal> x1478 = IKPowWithIntegerCheck(((-0.980795316323859) + ((cj5 * r01 * sj4)) + (((-1.0) * r02 * x1473)) + ((r00 * x1474))), -1);
                                                if (!x1478.valid)
                                                {
                                                    continue;
                                                }
                                                CheckValue<IkReal> x1479 = IKPowWithIntegerCheck(((-0.980795316323859) + (((-1.0) * cj4 * r02)) + ((r01 * x1475)) + ((r00 * x1474))), -1);
                                                if (!x1479.valid)
                                                {
                                                    continue;
                                                }
                                                CheckValue<IkReal> x1480 = IKPowWithIntegerCheck(((-0.980795316323859) + ((r00 * sj4 * sj5)) + (((-1.0) * r02 * x1473)) + ((r01 * x1475))), -1);
                                                if (!x1480.valid)
                                                {
                                                    continue;
                                                }
                                                j0array[0] = ((-2.0) * (atan(((((0.195039861251953) * x1476)) + (((-1.0) * r11 * x1475 * (x1478.value))) + ((r12 * x1473 * (x1479.value))) + (((-1.0) * r10 * x1474 * (x1480.value)))))));
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
                IkReal x254 = ((0.886638965956769) * sj0);
                IkReal x255 = ((0.176316034571766) * cj0);
                IkReal x256 = ((4.0e-8) * cj5);
                IkReal x257 = ((0.1135) * sj5);
                IkReal x258 = ((0.1135) * cj5);
                IkReal x259 = ((4.0e-8) * sj5);
                IkReal x260 = (r21 * x257);
                IkReal x261 = (x255 + (((-1.0) * x254)));
                IkReal x262 = ((((1.77327793191354) * sj0)) + (((-0.352632069143532) * cj0)));
                IkReal x263 = (x254 + (((-1.0) * x255)));
                IkReal x264 = (((r21 * x256)) + ((r20 * x258)) + ((r20 * x259)) + (((1.0) * pz)));
                IkReal x265 = (x260 + (((-1.0) * x264)));
                IkReal x266 = ((0.904) + x260 + (((-1.0) * x264)));
                IkReal x267 = ((-0.904) + x260 + (((-1.0) * x264)));
                IkReal x268 = ((((0.103375806095808) * cj0)) + ((r01 * x257)) + (((0.0205573202558309) * sj0)) + (((-1.0) * r00 * x258)) + (((-1.0) * r00 * x259)) + (((-1.0) * px)) + (((-1.0) * r01 * x256)));
                IkReal gconst36 = x266;
                IkReal gconst37 = x265;
                IkReal gconst38 = x268;
                IkReal gconst39 = x261;
                IkReal gconst40 = x268;
                IkReal gconst41 = x266;
                IkReal gconst42 = x265;
                IkReal gconst43 = x268;
                IkReal gconst44 = x261;
                IkReal gconst45 = x268;
                IkReal gconst46 = x262;
                IkReal gconst47 = x262;
                IkReal gconst48 = x267;
                IkReal gconst49 = x265;
                IkReal gconst50 = x268;
                IkReal gconst51 = x263;
                IkReal gconst52 = x268;
                IkReal gconst53 = x267;
                IkReal gconst54 = x265;
                IkReal gconst55 = x268;
                IkReal gconst56 = x263;
                IkReal gconst57 = x268;
                IkReal x269 = (gconst37 * gconst53);
                IkReal x270 = (gconst45 * gconst50);
                IkReal x271 = (gconst46 * gconst57);
                IkReal x272 = (gconst42 * gconst55);
                IkReal x273 = (gconst36 * gconst52);
                IkReal x274 = ((1.0) * gconst49);
                IkReal x275 = (gconst39 * gconst56);
                IkReal x276 = (gconst38 * gconst47);
                IkReal x277 = (gconst43 * gconst54);
                IkReal x278 = (gconst38 * gconst57);
                IkReal x279 = (gconst45 * gconst53);
                IkReal x280 = (gconst36 * gconst40);
                IkReal x281 = (gconst38 * gconst49);
                IkReal x282 = (gconst41 * gconst45);
                IkReal x283 = ((1.0) * gconst37);
                IkReal x284 = (gconst46 * gconst49);
                IkReal x285 = (gconst49 * gconst53);
                IkReal x286 = (gconst41 * gconst44);
                IkReal x287 = (gconst42 * gconst50);
                IkReal x288 = ((3.268864) * gconst52);
                IkReal x289 = ((1.808) * gconst37);
                IkReal x290 = (gconst39 * gconst55);
                IkReal x291 = (gconst42 * gconst43);
                IkReal x292 = ((1.0) * gconst51);
                IkReal x293 = (gconst53 * gconst57);
                IkReal x294 = (gconst41 * gconst49);
                IkReal x295 = (gconst42 * gconst47);
                IkReal x296 = (gconst50 * gconst57);
                IkReal x297 = (gconst37 * gconst41);
                IkReal x298 = (gconst51 * gconst53);
                IkReal x299 = ((1.808) * gconst49);
                IkReal x300 = (gconst43 * gconst51);
                IkReal x301 = (gconst47 * gconst51);
                IkReal x302 = (gconst47 * gconst54);
                IkReal x303 = (gconst40 * gconst48);
                IkReal x304 = ((3.268864) * gconst40);
                IkReal x305 = (gconst51 * gconst55);
                IkReal x306 = (gconst54 * gconst55);
                IkReal x307 = (gconst39 * gconst43);
                IkReal x308 = (gconst48 * gconst52);
                IkReal x309 = ((1.808) * gconst52);
                IkReal x310 = (gconst44 * gconst53);
                IkReal x311 = ((1.808) * gconst40 * gconst41);
                IkReal x312 = (gconst48 * x306);
                IkReal x313 = ((1.808) * gconst39 * gconst47);
                IkReal x314 = ((1.0) * gconst41 * gconst57);
                IkReal x315 = ((1.0) * x308);
                IkReal x316 = (gconst41 * gconst51 * gconst56);
                IkReal x317 = ((1.808) * gconst53 * gconst56);
                op[0] = (((x285 * x296)) + (((-1.0) * x293 * x315)) + (((-1.0) * gconst50 * x274 * x306)) + ((x306 * x308)) + (((-1.0) * gconst56 * x274 * x298)));
                op[1] = (((gconst53 * gconst56 * x309)) + ((x271 * x285)) + ((x299 * x305)) + ((x302 * x308)) + (((-1.0) * gconst50 * x274 * x302)) + (((-1.0) * gconst46 * x274 * x306)));
                op[2] = (((x278 * x285)) + (((-1.0) * gconst55 * x288)) + (((-1.0) * gconst44 * x274 * x298)) + (((-1.0) * x273 * x293)) + (((-1.0) * gconst50 * x274 * x277)) + (((-1.0) * gconst50 * x272 * x274)) + ((x303 * x306)) + (((-1.0) * gconst53 * x274 * x275)) + ((x299 * x301)) + (((-1.0) * x308 * x314)) + (((-1.0) * x279 * x315)) + (((-1.0) * x293 * x303)) + ((x269 * x296)) + ((x294 * x296)) + ((x277 * x308)) + (((-1.0) * gconst46 * x274 * x302)) + (((-1.0) * gconst38 * x274 * x306)) + (((-1.0) * gconst50 * x283 * x306)) + ((x270 * x285)) + ((x272 * x308)) + ((x273 * x306)) + (((-1.0) * gconst56 * x269 * x292)) + (((-1.0) * x274 * x316)));
                op[3] = (((gconst41 * gconst56 * x309)) + ((x271 * x294)) + ((x290 * x299)) + ((x279 * x284)) + (((-1.0) * gconst47 * x274 * x287)) + (((-1.0) * gconst54 * x274 * x276)) + (((-1.0) * gconst47 * x288)) + (((-1.0) * gconst46 * x272 * x274)) + ((x299 * x300)) + ((x309 * x310)) + (((-1.0) * gconst46 * x283 * x306)) + ((x295 * x308)) + ((x302 * x303)) + ((x289 * x305)) + ((x269 * x271)) + (((-1.0) * gconst46 * x274 * x277)) + ((gconst40 * x317)) + (((-1.0) * gconst50 * x283 * x302)) + ((x273 * x302)));
                op[4] = ((((-1.0) * x273 * x279)) + ((x272 * x273)) + ((x273 * x277)) + (((-1.0) * gconst38 * x274 * x277)) + ((x279 * x281)) + ((x291 * x308)) + (((-1.0) * gconst50 * x272 * x283)) + (((-1.0) * x303 * x314)) + (((-1.0) * gconst55 * x304)) + (((-1.0) * gconst46 * x274 * x295)) + (((-1.0) * gconst38 * x283 * x306)) + ((gconst39 * gconst47 * x299)) + (((-1.0) * x282 * x315)) + ((x270 * x294)) + (((-1.0) * gconst43 * x274 * x287)) + ((x280 * x306)) + (((-1.0) * gconst43 * x288)) + (((-1.0) * gconst46 * x283 * x302)) + (((-1.0) * gconst51 * x274 * x286)) + (((-1.0) * x273 * x314)) + ((x278 * x294)) + (((-1.0) * x280 * x293)) + (((-1.0) * gconst41 * x274 * x275)) + ((x296 * x297)) + ((x289 * x301)) + (((-1.0) * gconst38 * x272 * x274)) + ((x277 * x303)) + (((-1.0) * x269 * x275)) + (((-1.0) * gconst44 * x269 * x292)) + ((x269 * x270)) + ((x269 * x278)) + (((-1.0) * x279 * x303)) + ((x272 * x303)) + (((-1.0) * gconst50 * x277 * x283)) + (((-1.0) * x283 * x316)) + (((-1.0) * gconst39 * x274 * x310)));
                op[5] = (((x271 * x297)) + ((x289 * x290)) + (((-1.0) * gconst46 * x277 * x283)) + ((x273 * x295)) + (((-1.0) * gconst54 * x276 * x283)) + (((-1.0) * gconst46 * x272 * x283)) + (((-1.0) * gconst42 * x274 * x276)) + (((-1.0) * gconst47 * x304)) + ((x286 * x309)) + (((-1.0) * gconst46 * x274 * x291)) + ((gconst56 * x311)) + ((x280 * x302)) + ((x299 * x307)) + ((x295 * x303)) + (((1.808) * gconst40 * x310)) + ((x289 * x300)) + ((gconst45 * gconst46 * x269)) + ((x282 * x284)) + (((-1.0) * gconst47 * x283 * x287)));
                op[6] = ((((-1.0) * gconst43 * x283 * x287)) + (((-1.0) * gconst46 * x283 * x295)) + (((-1.0) * gconst38 * x277 * x283)) + ((x273 * x291)) + ((x291 * x303)) + ((gconst38 * gconst45 * x269)) + (((-1.0) * gconst51 * x283 * x286)) + ((x270 * x297)) + (((-1.0) * gconst43 * x304)) + (((-1.0) * x279 * x280)) + (((-1.0) * gconst38 * x274 * x291)) + ((x272 * x280)) + (((-1.0) * gconst41 * x275 * x283)) + (((-1.0) * gconst39 * x274 * x286)) + ((x278 * x297)) + (((-1.0) * x280 * x314)) + (((-1.0) * x282 * x303)) + (((-1.0) * x273 * x282)) + ((x277 * x280)) + ((x281 * x282)) + (((-1.0) * gconst38 * x272 * x283)) + ((gconst39 * gconst47 * x289)) + (((-1.0) * gconst39 * gconst44 * x269)));
                op[7] = ((((-1.0) * gconst46 * x283 * x291)) + ((x280 * x295)) + (((-1.0) * gconst42 * x276 * x283)) + (((1.808) * gconst40 * x286)) + ((gconst37 * gconst46 * x282)) + ((x289 * x307)));
                op[8] = (((gconst37 * gconst38 * x282)) + (((-1.0) * x280 * x282)) + ((x280 * x291)) + (((-1.0) * gconst39 * x283 * x286)) + (((-1.0) * gconst38 * x283 * x291)));
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
                                                    IkReal x318 = ((2.21238937309433) * px);
                                                    IkReal x319 = ((0.251106193846207) * sj1);
                                                    IkReal x320 = (cj5 * r00);
                                                    IkReal x321 = ((2.21238938053097) * pz);
                                                    IkReal x322 = (r01 * sj5);
                                                    IkReal x323 = (cj1 * cj5);
                                                    IkReal x324 = ((0.251106194690265) * r20);
                                                    IkReal x325 = ((0.251106193846207) * cj1);
                                                    IkReal x326 = ((8.84955749237734e-8) * r01);
                                                    IkReal x327 = (cj5 * sj1);
                                                    IkReal x328 = (sj1 * sj5);
                                                    IkReal x329 = ((8.84955752212389e-8) * r20);
                                                    IkReal x330 = ((8.84955752212389e-8) * r21);
                                                    IkReal x331 = ((8.84955749237734e-8) * r00);
                                                    IkReal x332 = (cj1 * sj5);
                                                    IkReal x333 = ((0.251106194690265) * r21 * sj5);
                                                    if (IKabs((((x323 * x326)) + ((x328 * x329)) + ((x320 * x325)) + ((x331 * x332)) + ((cj1 * x318)) + (((-1.0) * x322 * x325)) + (((-0.251106194690265) * r21 * x328)) + ((x324 * x327)) + ((x327 * x330)) + ((sj1 * x321)) + (((2.67146783671308e-7) * cj1)))) < IKFAST_ATAN2_MAGTHRESH && IKabs(((-1.0) + ((cj1 * x321)) + ((x323 * x324)) + (((-1.0) * sj1 * x318)) + (((-1.0) * x326 * x327)) + (((-2.67146783671308e-7) * sj1)) + (((-1.0) * x328 * x331)) + (((-1.0) * x319 * x320)) + ((x329 * x332)) + ((x323 * x330)) + (((-0.251106194690265) * r21 * x332)) + ((x319 * x322)))) < IKFAST_ATAN2_MAGTHRESH && IKabs(IKsqr((((x323 * x326)) + ((x328 * x329)) + ((x320 * x325)) + ((x331 * x332)) + ((cj1 * x318)) + (((-1.0) * x322 * x325)) + (((-0.251106194690265) * r21 * x328)) + ((x324 * x327)) + ((x327 * x330)) + ((sj1 * x321)) + (((2.67146783671308e-7) * cj1)))) + IKsqr(((-1.0) + ((cj1 * x321)) + ((x323 * x324)) + (((-1.0) * sj1 * x318)) + (((-1.0) * x326 * x327)) + (((-2.67146783671308e-7) * sj1)) + (((-1.0) * x328 * x331)) + (((-1.0) * x319 * x320)) + ((x329 * x332)) + ((x323 * x330)) + (((-0.251106194690265) * r21 * x332)) + ((x319 * x322)))) - 1) <= IKFAST_SINCOS_THRESH)
                                                        continue;
                                                    j2array[0] = IKatan2((((x323 * x326)) + ((x328 * x329)) + ((x320 * x325)) + ((x331 * x332)) + ((cj1 * x318)) + (((-1.0) * x322 * x325)) + (((-0.251106194690265) * r21 * x328)) + ((x324 * x327)) + ((x327 * x330)) + ((sj1 * x321)) + (((2.67146783671308e-7) * cj1))), ((-1.0) + ((cj1 * x321)) + ((x323 * x324)) + (((-1.0) * sj1 * x318)) + (((-1.0) * x326 * x327)) + (((-2.67146783671308e-7) * sj1)) + (((-1.0) * x328 * x331)) + (((-1.0) * x319 * x320)) + ((x329 * x332)) + ((x323 * x330)) + (((-0.251106194690265) * r21 * x332)) + ((x319 * x322))));
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
                                                            IkReal x334 = IKsin(j2);
                                                            IkReal x335 = IKcos(j2);
                                                            IkReal x336 = ((4.0e-8) * cj5);
                                                            IkReal x337 = ((8.79096604904732e-10) * sj1);
                                                            IkReal x338 = ((0.1135) * cj5);
                                                            IkReal x339 = ((0.1135) * sj5);
                                                            IkReal x340 = ((0.452) * cj1);
                                                            IkReal x341 = ((4.0e-8) * sj5);
                                                            IkReal x342 = ((0.452000001519335) * sj1);
                                                            IkReal x343 = (cj1 * x334);
                                                            evalcond[0] = ((((-1.0) * r20 * x341)) + ((x335 * x340)) + ((r21 * x339)) + (((-1.0) * r21 * x336)) + (((-1.0) * r20 * x338)) + x340 + (((0.452) * sj1 * x334)) + (((-1.0) * pz)));
                                                            evalcond[1] = ((-1.20750346625317e-7) + ((r01 * x339)) + (((-1.0) * x335 * x342)) + (((-1.0) * r01 * x336)) + (((-1.0) * r00 * x338)) + (((-1.0) * px)) + (((0.452000001519335) * x343)) + (((-1.0) * r00 * x341)) + (((-1.0) * x342)));
                                                            evalcond[2] = ((-0.105400003684668) + ((r11 * x339)) + ((x335 * x337)) + x337 + (((-1.0) * py)) + (((-1.0) * r11 * x336)) + (((-8.79096604904732e-10) * x343)) + (((-1.0) * r10 * x341)) + (((-1.0) * r10 * x338)));
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
                                                            IkReal x344 = (((cj5 * r11 * sj4)) + ((r10 * sj4 * sj5)) + (((-1.0) * cj4 * r12)));
                                                            j3eval[0] = x344;
                                                            j3eval[1] = IKsign(x344);
                                                            if (IKabs(j3eval[0]) < 0.0000010000000000 || IKabs(j3eval[1]) < 0.0000010000000000)
                                                            {
                                                                {
                                                                    IkReal j3eval[2];
                                                                    sj0 = -0.98079532;
                                                                    cj0 = 0.19503986;
                                                                    j0 = -1.37449824;
                                                                    IkReal x345 = ((1.0) * sj4);
                                                                    IkReal x346 = ((((-1.0) * cj5 * r01 * x345)) + (((-1.0) * r00 * sj5 * x345)) + ((cj4 * r02)));
                                                                    j3eval[0] = x346;
                                                                    j3eval[1] = IKsign(x346);
                                                                    if (IKabs(j3eval[0]) < 0.0000010000000000 || IKabs(j3eval[1]) < 0.0000010000000000)
                                                                    {
                                                                        {
                                                                            IkReal j3eval[2];
                                                                            sj0 = -0.98079532;
                                                                            cj0 = 0.19503986;
                                                                            j0 = -1.37449824;
                                                                            IkReal x347 = cj4 * cj4;
                                                                            IkReal x348 = cj5 * cj5;
                                                                            IkReal x349 = r22 * r22;
                                                                            IkReal x350 = r21 * r21;
                                                                            IkReal x351 = r20 * r20;
                                                                            IkReal x352 = (r20 * sj5);
                                                                            IkReal x353 = (cj5 * r21);
                                                                            IkReal x354 = ((1.0) * x350);
                                                                            IkReal x355 = ((1.0) * x351);
                                                                            IkReal x356 = (x347 * x348);
                                                                            IkReal x357 = ((2.0) * cj4 * r22 * sj4);
                                                                            IkReal x358 = ((((-1.0) * x348 * x355)) + (((-1.0) * x354)) + ((x351 * x356)) + (((-2.0) * x347 * x352 * x353)) + ((x347 * x349)) + (((-1.0) * x347 * x355)) + ((x348 * x350)) + (((-1.0) * x353 * x357)) + (((-1.0) * x354 * x356)) + (((2.0) * x352 * x353)) + (((-1.0) * x352 * x357)) + (((-1.0) * x349)));
                                                                            j3eval[0] = x358;
                                                                            j3eval[1] = IKsign(x358);
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
                                                                                    IkReal x359 = cj4 * cj4;
                                                                                    IkReal x360 = cj5 * cj5;
                                                                                    IkReal x361 = r22 * r22;
                                                                                    IkReal x362 = r21 * r21;
                                                                                    IkReal x363 = r20 * r20;
                                                                                    IkReal x364 = ((1.0) * sj1);
                                                                                    IkReal x365 = (r22 * sj4);
                                                                                    IkReal x366 = (sj2 * sj5);
                                                                                    IkReal x367 = (cj4 * r20);
                                                                                    IkReal x368 = (cj5 * sj2);
                                                                                    IkReal x369 = (r21 * sj5);
                                                                                    IkReal x370 = (cj4 * r21);
                                                                                    IkReal x371 = ((2.0) * cj5);
                                                                                    IkReal x372 = (cj2 * cj5 * r20);
                                                                                    IkReal x373 = ((1.0) * x362);
                                                                                    IkReal x374 = ((1.0) * cj1 * cj2);
                                                                                    IkReal x375 = ((1.0) * x363);
                                                                                    IkReal x376 = (x359 * x360);
                                                                                    CheckValue<IkReal> x377 = IKatan2WithCheck(IkReal(((((-1.0) * x364 * x368 * x370)) + ((cj2 * sj1 * x369)) + (((-1.0) * sj2 * x364 * x365)) + ((cj1 * r20 * x368)) + (((-1.0) * x364 * x372)) + (((-1.0) * cj1 * r21 * x366)) + (((-1.0) * sj5 * x367 * x374)) + (((-1.0) * cj5 * x370 * x374)) + (((-1.0) * x364 * x366 * x367)) + (((-1.0) * x365 * x374)))), IkReal((((cj1 * x366 * x367)) + (((-1.0) * cj2 * cj5 * x364 * x370)) + (((-1.0) * cj2 * sj5 * x364 * x367)) + ((r20 * sj1 * x368)) + ((cj1 * x372)) + (((-1.0) * cj2 * x364 * x365)) + ((cj1 * x368 * x370)) + (((-1.0) * x369 * x374)) + ((cj1 * sj2 * x365)) + (((-1.0) * r21 * x364 * x366)))), IKFAST_ATAN2_MAGTHRESH);
                                                                                    if (!x377.valid)
                                                                                    {
                                                                                        continue;
                                                                                    }
                                                                                    CheckValue<IkReal> x378 = IKPowWithIntegerCheck(IKsign(((((-1.0) * x361)) + (((-2.0) * sj5 * x365 * x367)) + (((-1.0) * x360 * x375)) + (((-1.0) * r20 * x359 * x369 * x371)) + ((r20 * x369 * x371)) + (((-1.0) * x373)) + ((x359 * x361)) + (((-1.0) * x359 * x375)) + (((-1.0) * x365 * x370 * x371)) + (((-1.0) * x373 * x376)) + ((x360 * x362)) + ((x363 * x376)))), -1);
                                                                                    if (!x378.valid)
                                                                                    {
                                                                                        continue;
                                                                                    }
                                                                                    j3array[0] = ((-1.5707963267949) + (x377.value) + (((1.5707963267949) * (x378.value))));
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
                                                                                            IkReal x379 = IKcos(j3);
                                                                                            IkReal x380 = IKsin(j3);
                                                                                            IkReal x381 = (r00 * sj5);
                                                                                            IkReal x382 = ((1.94490399315206e-9) * cj1);
                                                                                            IkReal x383 = (sj1 * sj2);
                                                                                            IkReal x384 = (cj2 * sj1);
                                                                                            IkReal x385 = ((1.0) * r11);
                                                                                            IkReal x386 = ((1.00000000336136) * cj1);
                                                                                            IkReal x387 = (cj5 * x379);
                                                                                            IkReal x388 = (sj4 * x379);
                                                                                            IkReal x389 = (sj5 * x379);
                                                                                            IkReal x390 = ((1.0) * x380);
                                                                                            IkReal x391 = (cj5 * x380);
                                                                                            IkReal x392 = (cj4 * x390);
                                                                                            evalcond[0] = (((r20 * x391)) + ((r22 * x388)) + ((cj4 * r20 * x389)) + ((cj1 * sj2)) + ((cj4 * r21 * x387)) + (((-1.0) * r21 * sj5 * x390)) + (((-1.0) * x384)));
                                                                                            evalcond[1] = ((((-1.0) * r20 * sj5 * x392)) + (((-1.0) * r22 * sj4 * x390)) + x383 + ((r20 * x387)) + ((cj1 * cj2)) + (((-1.0) * cj5 * r21 * x392)) + (((-1.0) * r21 * x389)));
                                                                                            evalcond[2] = ((((-1.0) * cj2 * x386)) + (((-1.0) * r01 * sj5 * x390)) + ((cj4 * r01 * x387)) + ((r00 * x391)) + ((r02 * x388)) + ((cj4 * x379 * x381)) + (((-1.00000000336136) * x383)));
                                                                                            evalcond[3] = (((r10 * x391)) + (((-1.0) * sj5 * x380 * x385)) + (((1.94490399315206e-9) * x383)) + ((cj4 * r10 * x389)) + ((cj2 * x382)) + ((r12 * x388)) + ((cj4 * r11 * x387)));
                                                                                            evalcond[4] = ((((-1.0) * r02 * sj4 * x390)) + (((-1.00000000336136) * x384)) + (((-1.0) * cj5 * r01 * x392)) + (((-1.0) * x381 * x392)) + (((-1.0) * r01 * x389)) + ((r00 * x387)) + ((sj2 * x386)));
                                                                                            evalcond[5] = ((((-1.0) * r12 * sj4 * x390)) + (((1.94490399315206e-9) * x384)) + (((-1.0) * r10 * sj5 * x392)) + (((-1.0) * cj4 * x385 * x391)) + ((r10 * x387)) + (((-1.0) * sj2 * x382)) + (((-1.0) * x385 * x389)));
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
                                                                            IkReal x393 = (cj5 * sj2);
                                                                            IkReal x394 = (cj1 * cj4);
                                                                            IkReal x395 = ((1.94490399315206e-9) * r20);
                                                                            IkReal x396 = (sj2 * sj4);
                                                                            IkReal x397 = ((1.0) * sj4);
                                                                            IkReal x398 = (cj2 * cj5);
                                                                            IkReal x399 = ((1.94490399315206e-9) * sj1);
                                                                            IkReal x400 = (cj2 * sj1);
                                                                            IkReal x401 = ((1.0) * r10);
                                                                            IkReal x402 = (cj4 * sj5);
                                                                            IkReal x403 = (sj2 * sj5);
                                                                            IkReal x404 = ((1.0) * r11);
                                                                            IkReal x405 = ((1.94490399315206e-9) * cj1 * cj2 * sj5);
                                                                            CheckValue<IkReal> x406 = IKatan2WithCheck(IkReal(((((-1.94490399315206e-9) * cj1 * cj2 * r22 * sj4)) + (((-1.0) * cj4 * sj1 * x398 * x404)) + (((-1.0) * r22 * x396 * x399)) + (((-1.0) * x400 * x401 * x402)) + (((-1.0) * r12 * x397 * x400)) + (((-1.0) * cj4 * r21 * x393 * x399)) + (((-1.0) * cj2 * sj5 * x394 * x395)) + ((r10 * x394 * x403)) + ((r11 * x393 * x394)) + (((-1.0) * sj1 * sj2 * x395 * x402)) + (((-1.94490399315206e-9) * r21 * x394 * x398)) + ((cj1 * r12 * x396)))), IkReal((((cj1 * x395 * x398)) + ((cj1 * r11 * x403)) + (((-1.0) * r21 * x405)) + ((sj1 * x393 * x395)) + ((r10 * sj1 * x398)) + (((-1.0) * cj1 * x393 * x401)) + (((-1.0) * r21 * x399 * x403)) + (((-1.0) * sj5 * x400 * x404)))), IKFAST_ATAN2_MAGTHRESH);
                                                                            if (!x406.valid)
                                                                            {
                                                                                continue;
                                                                            }
                                                                            CheckValue<IkReal> x407 = IKPowWithIntegerCheck(IKsign(((((-1.0) * r00 * sj5 * x397)) + (((-1.0) * cj5 * r01 * x397)) + ((cj4 * r02)))), -1);
                                                                            if (!x407.valid)
                                                                            {
                                                                                continue;
                                                                            }
                                                                            j3array[0] = ((-1.5707963267949) + (x406.value) + (((1.5707963267949) * (x407.value))));
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
                                                                                    IkReal x408 = IKcos(j3);
                                                                                    IkReal x409 = IKsin(j3);
                                                                                    IkReal x410 = (r00 * sj5);
                                                                                    IkReal x411 = ((1.94490399315206e-9) * cj1);
                                                                                    IkReal x412 = (sj1 * sj2);
                                                                                    IkReal x413 = (cj2 * sj1);
                                                                                    IkReal x414 = ((1.0) * r11);
                                                                                    IkReal x415 = ((1.00000000336136) * cj1);
                                                                                    IkReal x416 = (cj5 * x408);
                                                                                    IkReal x417 = (sj4 * x408);
                                                                                    IkReal x418 = (sj5 * x408);
                                                                                    IkReal x419 = ((1.0) * x409);
                                                                                    IkReal x420 = (cj5 * x409);
                                                                                    IkReal x421 = (cj4 * x419);
                                                                                    evalcond[0] = (((cj4 * r20 * x418)) + ((r20 * x420)) + (((-1.0) * r21 * sj5 * x419)) + ((cj4 * r21 * x416)) + ((cj1 * sj2)) + ((r22 * x417)) + (((-1.0) * x413)));
                                                                                    evalcond[1] = ((((-1.0) * cj5 * r21 * x421)) + (((-1.0) * r22 * sj4 * x419)) + x412 + (((-1.0) * r21 * x418)) + ((r20 * x416)) + (((-1.0) * r20 * sj5 * x421)) + ((cj1 * cj2)));
                                                                                    evalcond[2] = ((((-1.0) * cj2 * x415)) + (((-1.0) * r01 * sj5 * x419)) + ((r00 * x420)) + ((cj4 * r01 * x416)) + ((cj4 * x408 * x410)) + (((-1.00000000336136) * x412)) + ((r02 * x417)));
                                                                                    evalcond[3] = (((cj4 * r11 * x416)) + ((r12 * x417)) + (((1.94490399315206e-9) * x412)) + ((cj4 * r10 * x418)) + (((-1.0) * sj5 * x409 * x414)) + ((r10 * x420)) + ((cj2 * x411)));
                                                                                    evalcond[4] = ((((-1.0) * cj5 * r01 * x421)) + (((-1.0) * r01 * x418)) + ((sj2 * x415)) + (((-1.0) * r02 * sj4 * x419)) + ((r00 * x416)) + (((-1.00000000336136) * x413)) + (((-1.0) * x410 * x421)));
                                                                                    evalcond[5] = ((((-1.0) * cj4 * x414 * x420)) + (((1.94490399315206e-9) * x413)) + (((-1.0) * r12 * sj4 * x419)) + (((-1.0) * r10 * sj5 * x421)) + ((r10 * x416)) + (((-1.0) * x414 * x418)) + (((-1.0) * sj2 * x411)));
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
                                                                    IkReal x422 = (sj2 * sj5);
                                                                    IkReal x423 = ((1.0) * r01);
                                                                    IkReal x424 = (cj2 * sj5);
                                                                    IkReal x425 = ((1.00000000336136) * sj1);
                                                                    IkReal x426 = (r02 * sj4);
                                                                    IkReal x427 = (cj1 * cj2);
                                                                    IkReal x428 = (cj4 * r00);
                                                                    IkReal x429 = (cj5 * sj2);
                                                                    IkReal x430 = (cj4 * cj5);
                                                                    IkReal x431 = (r22 * sj4);
                                                                    IkReal x432 = ((1.00000000336136) * cj1);
                                                                    IkReal x433 = (cj4 * x432);
                                                                    CheckValue<IkReal> x434 = IKPowWithIntegerCheck(IKsign((((cj5 * r11 * sj4)) + ((r10 * sj4 * sj5)) + (((-1.0) * cj4 * r12)))), -1);
                                                                    if (!x434.valid)
                                                                    {
                                                                        continue;
                                                                    }
                                                                    CheckValue<IkReal> x435 = IKatan2WithCheck(IkReal(((((-1.0) * sj1 * x422 * x423)) + (((-1.0) * r20 * x429 * x432)) + ((r00 * sj1 * x429)) + ((cj2 * cj5 * r20 * x425)) + (((-1.0) * r21 * x424 * x425)) + (((-1.0) * cj1 * x423 * x424)) + ((cj5 * r00 * x427)) + ((r21 * x422 * x432)))), IkReal((((x426 * x427)) + ((sj1 * x422 * x428)) + ((cj2 * x425 * x431)) + ((cj4 * r20 * x424 * x425)) + (((-1.0) * r21 * x429 * x433)) + ((cj1 * x424 * x428)) + ((cj2 * r21 * x425 * x430)) + ((sj1 * sj2 * x426)) + ((r01 * x427 * x430)) + (((-1.0) * r20 * x422 * x433)) + (((-1.0) * sj2 * x431 * x432)) + ((cj4 * r01 * sj1 * x429)))), IKFAST_ATAN2_MAGTHRESH);
                                                                    if (!x435.valid)
                                                                    {
                                                                        continue;
                                                                    }
                                                                    j3array[0] = ((-1.5707963267949) + (((1.5707963267949) * (x434.value))) + (x435.value));
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
                                                                            IkReal x436 = IKcos(j3);
                                                                            IkReal x437 = IKsin(j3);
                                                                            IkReal x438 = (r00 * sj5);
                                                                            IkReal x439 = ((1.94490399315206e-9) * cj1);
                                                                            IkReal x440 = (sj1 * sj2);
                                                                            IkReal x441 = (cj2 * sj1);
                                                                            IkReal x442 = ((1.0) * r11);
                                                                            IkReal x443 = ((1.00000000336136) * cj1);
                                                                            IkReal x444 = (cj5 * x436);
                                                                            IkReal x445 = (sj4 * x436);
                                                                            IkReal x446 = (sj5 * x436);
                                                                            IkReal x447 = ((1.0) * x437);
                                                                            IkReal x448 = (cj5 * x437);
                                                                            IkReal x449 = (cj4 * x447);
                                                                            evalcond[0] = (((cj4 * r20 * x446)) + (((-1.0) * x441)) + ((cj1 * sj2)) + ((cj4 * r21 * x444)) + (((-1.0) * r21 * sj5 * x447)) + ((r22 * x445)) + ((r20 * x448)));
                                                                            evalcond[1] = ((((-1.0) * cj5 * r21 * x449)) + (((-1.0) * r22 * sj4 * x447)) + x440 + ((cj1 * cj2)) + (((-1.0) * r20 * sj5 * x449)) + ((r20 * x444)) + (((-1.0) * r21 * x446)));
                                                                            evalcond[2] = ((((-1.0) * cj2 * x443)) + ((r02 * x445)) + (((-1.0) * r01 * sj5 * x447)) + ((r00 * x448)) + ((cj4 * r01 * x444)) + (((-1.00000000336136) * x440)) + ((cj4 * x436 * x438)));
                                                                            evalcond[3] = (((cj4 * r10 * x446)) + (((1.94490399315206e-9) * x440)) + (((-1.0) * sj5 * x437 * x442)) + ((cj2 * x439)) + ((cj4 * r11 * x444)) + ((r10 * x448)) + ((r12 * x445)));
                                                                            evalcond[4] = ((((-1.0) * x438 * x449)) + (((-1.0) * cj5 * r01 * x449)) + (((-1.0) * r01 * x446)) + ((sj2 * x443)) + (((-1.0) * r02 * sj4 * x447)) + ((r00 * x444)) + (((-1.00000000336136) * x441)));
                                                                            evalcond[5] = ((((-1.0) * cj4 * x442 * x448)) + (((1.94490399315206e-9) * x441)) + (((-1.0) * r12 * sj4 * x447)) + (((-1.0) * r10 * sj5 * x449)) + ((r10 * x444)) + (((-1.0) * sj2 * x439)) + (((-1.0) * x442 * x446)));
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
                                                        IkReal x450 = ((2.21238937309433) * px);
                                                        IkReal x451 = ((0.251106193846207) * sj1);
                                                        IkReal x452 = (cj5 * r00);
                                                        IkReal x453 = ((2.21238938053097) * pz);
                                                        IkReal x454 = (r01 * sj5);
                                                        IkReal x455 = (cj1 * cj5);
                                                        IkReal x456 = ((0.251106194690265) * r20);
                                                        IkReal x457 = ((0.251106193846207) * cj1);
                                                        IkReal x458 = ((8.84955749237734e-8) * r01);
                                                        IkReal x459 = (cj5 * sj1);
                                                        IkReal x460 = (sj1 * sj5);
                                                        IkReal x461 = ((8.84955752212389e-8) * r20);
                                                        IkReal x462 = ((8.84955752212389e-8) * r21);
                                                        IkReal x463 = ((8.84955749237734e-8) * r00);
                                                        IkReal x464 = (cj1 * sj5);
                                                        IkReal x465 = ((0.251106194690265) * r21 * sj5);
                                                        if (IKabs((((x454 * x457)) + (((-1.0) * x463 * x464)) + (((-0.251106194690265) * r21 * x460)) + (((-1.0) * cj1 * x450)) + ((sj1 * x453)) + (((-1.0) * x455 * x458)) + (((-1.0) * x452 * x457)) + ((x460 * x461)) + ((x459 * x462)) + (((2.67146783671308e-7) * cj1)) + ((x456 * x459)))) < IKFAST_ATAN2_MAGTHRESH && IKabs(((-1.0) + ((x455 * x462)) + ((x461 * x464)) + ((x455 * x456)) + ((cj1 * x453)) + (((-0.251106194690265) * r21 * x464)) + (((-2.67146783671308e-7) * sj1)) + ((sj1 * x450)) + ((x460 * x463)) + (((-1.0) * x451 * x454)) + ((x451 * x452)) + ((x458 * x459)))) < IKFAST_ATAN2_MAGTHRESH && IKabs(IKsqr((((x454 * x457)) + (((-1.0) * x463 * x464)) + (((-0.251106194690265) * r21 * x460)) + (((-1.0) * cj1 * x450)) + ((sj1 * x453)) + (((-1.0) * x455 * x458)) + (((-1.0) * x452 * x457)) + ((x460 * x461)) + ((x459 * x462)) + (((2.67146783671308e-7) * cj1)) + ((x456 * x459)))) + IKsqr(((-1.0) + ((x455 * x462)) + ((x461 * x464)) + ((x455 * x456)) + ((cj1 * x453)) + (((-0.251106194690265) * r21 * x464)) + (((-2.67146783671308e-7) * sj1)) + ((sj1 * x450)) + ((x460 * x463)) + (((-1.0) * x451 * x454)) + ((x451 * x452)) + ((x458 * x459)))) - 1) <= IKFAST_SINCOS_THRESH)
                                                            continue;
                                                        j2array[0] = IKatan2((((x454 * x457)) + (((-1.0) * x463 * x464)) + (((-0.251106194690265) * r21 * x460)) + (((-1.0) * cj1 * x450)) + ((sj1 * x453)) + (((-1.0) * x455 * x458)) + (((-1.0) * x452 * x457)) + ((x460 * x461)) + ((x459 * x462)) + (((2.67146783671308e-7) * cj1)) + ((x456 * x459))), ((-1.0) + ((x455 * x462)) + ((x461 * x464)) + ((x455 * x456)) + ((cj1 * x453)) + (((-0.251106194690265) * r21 * x464)) + (((-2.67146783671308e-7) * sj1)) + ((sj1 * x450)) + ((x460 * x463)) + (((-1.0) * x451 * x454)) + ((x451 * x452)) + ((x458 * x459))));
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
                                                                IkReal x466 = IKcos(j2);
                                                                IkReal x467 = IKsin(j2);
                                                                IkReal x468 = ((4.0e-8) * cj5);
                                                                IkReal x469 = ((8.79096604904732e-10) * sj1);
                                                                IkReal x470 = ((0.1135) * cj5);
                                                                IkReal x471 = ((0.1135) * sj5);
                                                                IkReal x472 = ((0.452) * cj1);
                                                                IkReal x473 = ((4.0e-8) * sj5);
                                                                IkReal x474 = ((0.452000001519335) * sj1);
                                                                IkReal x475 = (cj1 * x467);
                                                                evalcond[0] = (((r21 * x471)) + (((0.452) * sj1 * x467)) + ((x466 * x472)) + (((-1.0) * pz)) + x472 + (((-1.0) * r21 * x468)) + (((-1.0) * r20 * x473)) + (((-1.0) * r20 * x470)));
                                                                evalcond[1] = ((1.20750346625316e-7) + (((-0.452000001519335) * x475)) + (((-1.0) * r01 * x468)) + (((-1.0) * r00 * x470)) + (((-1.0) * r00 * x473)) + ((x466 * x474)) + (((-1.0) * px)) + x474 + ((r01 * x471)));
                                                                evalcond[2] = ((0.105400003684668) + (((-1.0) * x466 * x469)) + (((8.79096604904732e-10) * x475)) + (((-1.0) * x469)) + ((r11 * x471)) + (((-1.0) * py)) + (((-1.0) * r11 * x468)) + (((-1.0) * r10 * x473)) + (((-1.0) * r10 * x470)));
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
                                                                IkReal x476 = (((cj5 * r11 * sj4)) + ((r10 * sj4 * sj5)) + (((-1.0) * cj4 * r12)));
                                                                j3eval[0] = x476;
                                                                j3eval[1] = IKsign(x476);
                                                                if (IKabs(j3eval[0]) < 0.0000010000000000 || IKabs(j3eval[1]) < 0.0000010000000000)
                                                                {
                                                                    {
                                                                        IkReal j3eval[2];
                                                                        sj0 = 0.98079532;
                                                                        cj0 = -0.19503986;
                                                                        j0 = 1.76709441189614;
                                                                        IkReal x477 = ((1.0) * sj4);
                                                                        IkReal x478 = ((((-1.0) * r00 * sj5 * x477)) + (((-1.0) * cj5 * r01 * x477)) + ((cj4 * r02)));
                                                                        j3eval[0] = x478;
                                                                        j3eval[1] = IKsign(x478);
                                                                        if (IKabs(j3eval[0]) < 0.0000010000000000 || IKabs(j3eval[1]) < 0.0000010000000000)
                                                                        {
                                                                            {
                                                                                IkReal j3eval[2];
                                                                                sj0 = 0.98079532;
                                                                                cj0 = -0.19503986;
                                                                                j0 = 1.76709441189614;
                                                                                IkReal x479 = cj4 * cj4;
                                                                                IkReal x480 = cj5 * cj5;
                                                                                IkReal x481 = r22 * r22;
                                                                                IkReal x482 = r21 * r21;
                                                                                IkReal x483 = r20 * r20;
                                                                                IkReal x484 = (r20 * sj5);
                                                                                IkReal x485 = (cj5 * r21);
                                                                                IkReal x486 = ((1.0) * x482);
                                                                                IkReal x487 = ((1.0) * x483);
                                                                                IkReal x488 = (x479 * x480);
                                                                                IkReal x489 = ((2.0) * cj4 * r22 * sj4);
                                                                                IkReal x490 = ((((-1.0) * x484 * x489)) + (((-1.0) * x486 * x488)) + (((-2.0) * x479 * x484 * x485)) + (((-1.0) * x486)) + ((x483 * x488)) + (((-1.0) * x481)) + ((x480 * x482)) + (((-1.0) * x480 * x487)) + (((-1.0) * x479 * x487)) + ((x479 * x481)) + (((2.0) * x484 * x485)) + (((-1.0) * x485 * x489)));
                                                                                j3eval[0] = x490;
                                                                                j3eval[1] = IKsign(x490);
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
                                                                                        IkReal x491 = cj4 * cj4;
                                                                                        IkReal x492 = cj5 * cj5;
                                                                                        IkReal x493 = r22 * r22;
                                                                                        IkReal x494 = r21 * r21;
                                                                                        IkReal x495 = r20 * r20;
                                                                                        IkReal x496 = ((1.0) * sj1);
                                                                                        IkReal x497 = (r22 * sj4);
                                                                                        IkReal x498 = (sj2 * sj5);
                                                                                        IkReal x499 = (cj4 * r20);
                                                                                        IkReal x500 = (cj5 * sj2);
                                                                                        IkReal x501 = (r21 * sj5);
                                                                                        IkReal x502 = (cj4 * r21);
                                                                                        IkReal x503 = ((2.0) * cj5);
                                                                                        IkReal x504 = (cj2 * cj5 * r20);
                                                                                        IkReal x505 = ((1.0) * x494);
                                                                                        IkReal x506 = ((1.0) * cj1 * cj2);
                                                                                        IkReal x507 = ((1.0) * x495);
                                                                                        IkReal x508 = (x491 * x492);
                                                                                        CheckValue<IkReal> x509 = IKPowWithIntegerCheck(IKsign(((((-1.0) * x505 * x508)) + ((x491 * x493)) + (((-1.0) * x505)) + ((x495 * x508)) + (((-1.0) * x491 * x507)) + (((-1.0) * x493)) + (((-1.0) * x497 * x502 * x503)) + (((-1.0) * x492 * x507)) + (((-1.0) * r20 * x491 * x501 * x503)) + ((r20 * x501 * x503)) + ((x492 * x494)) + (((-2.0) * sj5 * x497 * x499)))), -1);
                                                                                        if (!x509.valid)
                                                                                        {
                                                                                            continue;
                                                                                        }
                                                                                        CheckValue<IkReal> x510 = IKatan2WithCheck(IkReal(((((-1.0) * sj5 * x499 * x506)) + ((cj2 * sj1 * x501)) + (((-1.0) * cj5 * x502 * x506)) + (((-1.0) * x496 * x500 * x502)) + (((-1.0) * x497 * x506)) + (((-1.0) * x496 * x504)) + ((cj1 * r20 * x500)) + (((-1.0) * cj1 * r21 * x498)) + (((-1.0) * sj2 * x496 * x497)) + (((-1.0) * x496 * x498 * x499)))), IkReal(((((-1.0) * r21 * x496 * x498)) + ((cj1 * x504)) + (((-1.0) * cj2 * x496 * x497)) + ((cj1 * x500 * x502)) + (((-1.0) * x501 * x506)) + ((cj1 * sj2 * x497)) + (((-1.0) * cj2 * cj5 * x496 * x502)) + (((-1.0) * cj2 * sj5 * x496 * x499)) + ((r20 * sj1 * x500)) + ((cj1 * x498 * x499)))), IKFAST_ATAN2_MAGTHRESH);
                                                                                        if (!x510.valid)
                                                                                        {
                                                                                            continue;
                                                                                        }
                                                                                        j3array[0] = ((-1.5707963267949) + (((1.5707963267949) * (x509.value))) + (x510.value));
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
                                                                                                IkReal x511 = IKcos(j3);
                                                                                                IkReal x512 = IKsin(j3);
                                                                                                IkReal x513 = (r00 * sj5);
                                                                                                IkReal x514 = ((1.94490399315206e-9) * cj1);
                                                                                                IkReal x515 = (sj1 * sj2);
                                                                                                IkReal x516 = (cj2 * sj1);
                                                                                                IkReal x517 = ((1.0) * r11);
                                                                                                IkReal x518 = (cj1 * sj2);
                                                                                                IkReal x519 = (cj1 * cj2);
                                                                                                IkReal x520 = (cj5 * x511);
                                                                                                IkReal x521 = (sj4 * x511);
                                                                                                IkReal x522 = (sj5 * x511);
                                                                                                IkReal x523 = (cj5 * x512);
                                                                                                IkReal x524 = ((1.0) * x512);
                                                                                                IkReal x525 = (cj4 * x524);
                                                                                                evalcond[0] = ((((-1.0) * r21 * sj5 * x524)) + ((r20 * x523)) + (((-1.0) * x516)) + ((r22 * x521)) + ((cj4 * r20 * x522)) + x518 + ((cj4 * r21 * x520)));
                                                                                                evalcond[1] = (((r20 * x520)) + (((-1.0) * cj4 * r21 * x523)) + (((-1.0) * r21 * x522)) + (((-1.0) * r20 * sj5 * x525)) + x519 + x515 + (((-1.0) * r22 * sj4 * x524)));
                                                                                                evalcond[2] = ((((-1.0) * r01 * sj5 * x524)) + ((r02 * x521)) + (((1.00000000336136) * x515)) + (((1.00000000336136) * x519)) + ((cj4 * r01 * x520)) + ((cj4 * x511 * x513)) + ((r00 * x523)));
                                                                                                evalcond[3] = (((r12 * x521)) + (((-1.0) * sj5 * x512 * x517)) + ((cj4 * r10 * x522)) + (((-1.0) * cj2 * x514)) + ((cj4 * r11 * x520)) + ((r10 * x523)) + (((-1.94490399315206e-9) * x515)));
                                                                                                evalcond[4] = ((((1.00000000336136) * x516)) + (((-1.0) * cj4 * r01 * x523)) + (((-1.00000000336136) * x518)) + (((-1.0) * x513 * x525)) + (((-1.0) * r02 * sj4 * x524)) + (((-1.0) * r01 * x522)) + ((r00 * x520)));
                                                                                                evalcond[5] = ((((-1.0) * x517 * x522)) + ((r10 * x520)) + (((-1.94490399315206e-9) * x516)) + ((sj2 * x514)) + (((-1.0) * cj4 * x517 * x523)) + (((-1.0) * r10 * sj5 * x525)) + (((-1.0) * r12 * sj4 * x524)));
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
                                                                                IkReal x526 = (cj5 * sj2);
                                                                                IkReal x527 = (cj1 * cj4);
                                                                                IkReal x528 = ((1.94490399315206e-9) * r20);
                                                                                IkReal x529 = (sj2 * sj4);
                                                                                IkReal x530 = ((1.0) * sj4);
                                                                                IkReal x531 = (cj2 * cj5);
                                                                                IkReal x532 = ((1.94490399315206e-9) * sj1);
                                                                                IkReal x533 = (cj2 * sj1);
                                                                                IkReal x534 = ((1.0) * r10);
                                                                                IkReal x535 = (cj4 * sj5);
                                                                                IkReal x536 = (sj2 * sj5);
                                                                                IkReal x537 = ((1.0) * r11);
                                                                                IkReal x538 = ((1.94490399315206e-9) * cj1 * cj2 * sj5);
                                                                                CheckValue<IkReal> x539 = IKPowWithIntegerCheck(IKsign(((((-1.0) * cj5 * r01 * x530)) + (((-1.0) * r00 * sj5 * x530)) + ((cj4 * r02)))), -1);
                                                                                if (!x539.valid)
                                                                                {
                                                                                    continue;
                                                                                }
                                                                                CheckValue<IkReal> x540 = IKatan2WithCheck(IkReal((((cj4 * r21 * x526 * x532)) + ((r10 * x527 * x536)) + ((cj1 * r12 * x529)) + ((r11 * x526 * x527)) + ((sj1 * sj2 * x528 * x535)) + (((1.94490399315206e-9) * cj1 * cj2 * r22 * sj4)) + (((-1.0) * x533 * x534 * x535)) + ((cj2 * sj5 * x527 * x528)) + (((-1.0) * cj4 * sj1 * x531 * x537)) + (((-1.0) * r12 * x530 * x533)) + (((1.94490399315206e-9) * r21 * x527 * x531)) + ((r22 * x529 * x532)))), IkReal(((((-1.0) * sj5 * x533 * x537)) + ((r10 * sj1 * x531)) + (((-1.0) * sj1 * x526 * x528)) + (((-1.0) * cj1 * x526 * x534)) + ((r21 * x532 * x536)) + (((-1.0) * cj1 * x528 * x531)) + ((cj1 * r11 * x536)) + ((r21 * x538)))), IKFAST_ATAN2_MAGTHRESH);
                                                                                if (!x540.valid)
                                                                                {
                                                                                    continue;
                                                                                }
                                                                                j3array[0] = ((-1.5707963267949) + (((1.5707963267949) * (x539.value))) + (x540.value));
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
                                                                                        IkReal x541 = IKcos(j3);
                                                                                        IkReal x542 = IKsin(j3);
                                                                                        IkReal x543 = (r00 * sj5);
                                                                                        IkReal x544 = ((1.94490399315206e-9) * cj1);
                                                                                        IkReal x545 = (sj1 * sj2);
                                                                                        IkReal x546 = (cj2 * sj1);
                                                                                        IkReal x547 = ((1.0) * r11);
                                                                                        IkReal x548 = (cj1 * sj2);
                                                                                        IkReal x549 = (cj1 * cj2);
                                                                                        IkReal x550 = (cj5 * x541);
                                                                                        IkReal x551 = (sj4 * x541);
                                                                                        IkReal x552 = (sj5 * x541);
                                                                                        IkReal x553 = (cj5 * x542);
                                                                                        IkReal x554 = ((1.0) * x542);
                                                                                        IkReal x555 = (cj4 * x554);
                                                                                        evalcond[0] = ((((-1.0) * x546)) + (((-1.0) * r21 * sj5 * x554)) + ((r20 * x553)) + ((cj4 * r21 * x550)) + x548 + ((r22 * x551)) + ((cj4 * r20 * x552)));
                                                                                        evalcond[1] = ((((-1.0) * cj4 * r21 * x553)) + (((-1.0) * r20 * sj5 * x555)) + ((r20 * x550)) + (((-1.0) * r22 * sj4 * x554)) + (((-1.0) * r21 * x552)) + x545 + x549);
                                                                                        evalcond[2] = (((cj4 * x541 * x543)) + (((-1.0) * r01 * sj5 * x554)) + ((r02 * x551)) + ((cj4 * r01 * x550)) + ((r00 * x553)) + (((1.00000000336136) * x545)) + (((1.00000000336136) * x549)));
                                                                                        evalcond[3] = ((((-1.94490399315206e-9) * x545)) + ((cj4 * r10 * x552)) + (((-1.0) * sj5 * x542 * x547)) + ((r12 * x551)) + (((-1.0) * cj2 * x544)) + ((r10 * x553)) + ((cj4 * r11 * x550)));
                                                                                        evalcond[4] = ((((-1.0) * cj4 * r01 * x553)) + (((-1.0) * r02 * sj4 * x554)) + (((-1.00000000336136) * x548)) + (((-1.0) * x543 * x555)) + ((r00 * x550)) + (((1.00000000336136) * x546)) + (((-1.0) * r01 * x552)));
                                                                                        evalcond[5] = ((((-1.94490399315206e-9) * x546)) + ((sj2 * x544)) + (((-1.0) * x547 * x552)) + ((r10 * x550)) + (((-1.0) * cj4 * x547 * x553)) + (((-1.0) * r10 * sj5 * x555)) + (((-1.0) * r12 * sj4 * x554)));
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
                                                                        IkReal x556 = (sj2 * sj5);
                                                                        IkReal x557 = ((1.00000000336136) * cj4);
                                                                        IkReal x558 = (cj1 * r20);
                                                                        IkReal x559 = ((1.0) * r01);
                                                                        IkReal x560 = (r02 * sj4);
                                                                        IkReal x561 = (cj1 * cj2);
                                                                        IkReal x562 = (cj4 * r00);
                                                                        IkReal x563 = (sj1 * sj2);
                                                                        IkReal x564 = (cj1 * sj2);
                                                                        IkReal x565 = (cj2 * sj1);
                                                                        IkReal x566 = ((1.00000000336136) * cj5);
                                                                        IkReal x567 = (cj5 * r00);
                                                                        IkReal x568 = (r21 * x565);
                                                                        IkReal x569 = (cj4 * cj5 * r01);
                                                                        IkReal x570 = ((1.00000000336136) * r22 * sj4);
                                                                        CheckValue<IkReal> x571 = IKatan2WithCheck(IkReal(((((1.00000000336136) * sj5 * x568)) + ((sj2 * x558 * x566)) + ((x563 * x567)) + ((x561 * x567)) + (((-1.00000000336136) * cj1 * r21 * x556)) + (((-1.0) * sj5 * x559 * x561)) + (((-1.0) * r20 * x565 * x566)) + (((-1.0) * sj1 * x556 * x559)))), IkReal((((sj1 * x556 * x562)) + ((sj5 * x561 * x562)) + (((-1.0) * cj5 * x557 * x568)) + ((x563 * x569)) + ((x561 * x569)) + (((-1.0) * x565 * x570)) + ((x560 * x563)) + ((x560 * x561)) + (((-1.0) * r20 * sj5 * x557 * x565)) + ((cj5 * r21 * x557 * x564)) + ((x564 * x570)) + ((x556 * x557 * x558)))), IKFAST_ATAN2_MAGTHRESH);
                                                                        if (!x571.valid)
                                                                        {
                                                                            continue;
                                                                        }
                                                                        CheckValue<IkReal> x572 = IKPowWithIntegerCheck(IKsign((((cj5 * r11 * sj4)) + ((r10 * sj4 * sj5)) + (((-1.0) * cj4 * r12)))), -1);
                                                                        if (!x572.valid)
                                                                        {
                                                                            continue;
                                                                        }
                                                                        j3array[0] = ((-1.5707963267949) + (x571.value) + (((1.5707963267949) * (x572.value))));
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
                                                                                IkReal x573 = IKcos(j3);
                                                                                IkReal x574 = IKsin(j3);
                                                                                IkReal x575 = (r00 * sj5);
                                                                                IkReal x576 = ((1.94490399315206e-9) * cj1);
                                                                                IkReal x577 = (sj1 * sj2);
                                                                                IkReal x578 = (cj2 * sj1);
                                                                                IkReal x579 = ((1.0) * r11);
                                                                                IkReal x580 = (cj1 * sj2);
                                                                                IkReal x581 = (cj1 * cj2);
                                                                                IkReal x582 = (cj5 * x573);
                                                                                IkReal x583 = (sj4 * x573);
                                                                                IkReal x584 = (sj5 * x573);
                                                                                IkReal x585 = (cj5 * x574);
                                                                                IkReal x586 = ((1.0) * x574);
                                                                                IkReal x587 = (cj4 * x586);
                                                                                evalcond[0] = (((r20 * x585)) + ((cj4 * r20 * x584)) + (((-1.0) * x578)) + x580 + ((cj4 * r21 * x582)) + ((r22 * x583)) + (((-1.0) * r21 * sj5 * x586)));
                                                                                evalcond[1] = (((r20 * x582)) + (((-1.0) * r22 * sj4 * x586)) + (((-1.0) * r21 * x584)) + (((-1.0) * cj4 * r21 * x585)) + x577 + x581 + (((-1.0) * r20 * sj5 * x587)));
                                                                                evalcond[2] = ((((1.00000000336136) * x577)) + ((r02 * x583)) + ((cj4 * x573 * x575)) + (((1.00000000336136) * x581)) + ((r00 * x585)) + (((-1.0) * r01 * sj5 * x586)) + ((cj4 * r01 * x582)));
                                                                                evalcond[3] = ((((-1.0) * cj2 * x576)) + ((r12 * x583)) + ((r10 * x585)) + ((cj4 * r11 * x582)) + ((cj4 * r10 * x584)) + (((-1.0) * sj5 * x574 * x579)) + (((-1.94490399315206e-9) * x577)));
                                                                                evalcond[4] = ((((1.00000000336136) * x578)) + (((-1.0) * r02 * sj4 * x586)) + (((-1.00000000336136) * x580)) + (((-1.0) * cj4 * r01 * x585)) + (((-1.0) * x575 * x587)) + ((r00 * x582)) + (((-1.0) * r01 * x584)));
                                                                                evalcond[5] = (((r10 * x582)) + (((-1.0) * x579 * x584)) + (((-1.0) * r12 * sj4 * x586)) + (((-1.0) * cj4 * x579 * x585)) + ((sj2 * x576)) + (((-1.0) * r10 * sj5 * x587)) + (((-1.94490399315206e-9) * x578)));
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
                                                            IkReal x588 = (cj1 * sj5);
                                                            IkReal x589 = ((0.251106194690265) * r21);
                                                            IkReal x590 = ((45.5012563770905) * r00);
                                                            IkReal x591 = ((2.21238938053097) * pz);
                                                            IkReal x592 = (sj1 * sj5);
                                                            IkReal x593 = ((1137531409.42726) * px);
                                                            IkReal x594 = (cj1 * cj5);
                                                            IkReal x595 = ((0.251106194690265) * r20);
                                                            IkReal x596 = ((129109814.969994) * r01);
                                                            IkReal x597 = ((129109814.969994) * r00);
                                                            IkReal x598 = ((8.84955752212389e-8) * r20);
                                                            IkReal x599 = ((8.84955752212389e-8) * r21);
                                                            IkReal x600 = ((45.5012563770905) * r01);
                                                            IkReal x601 = (cj5 * sj1);
                                                            if (IKabs(((((-1.0) * x588 * x590)) + ((x595 * x601)) + (((-1.0) * x589 * x592)) + (((-1.0) * cj1 * x593)) + (((-1.0) * x594 * x597)) + ((sj1 * x591)) + ((x599 * x601)) + (((-119895814.745059) * cj1)) + (((-1.0) * x594 * x600)) + ((x588 * x596)) + ((x592 * x598)))) < IKFAST_ATAN2_MAGTHRESH && IKabs(((-1.0) + ((x594 * x595)) + ((x594 * x599)) + (((-1.0) * x588 * x589)) + ((cj1 * x591)) + ((x600 * x601)) + ((x597 * x601)) + (((-1.0) * x592 * x596)) + ((x590 * x592)) + ((sj1 * x593)) + ((x588 * x598)) + (((119895814.745059) * sj1)))) < IKFAST_ATAN2_MAGTHRESH && IKabs(IKsqr(((((-1.0) * x588 * x590)) + ((x595 * x601)) + (((-1.0) * x589 * x592)) + (((-1.0) * cj1 * x593)) + (((-1.0) * x594 * x597)) + ((sj1 * x591)) + ((x599 * x601)) + (((-119895814.745059) * cj1)) + (((-1.0) * x594 * x600)) + ((x588 * x596)) + ((x592 * x598)))) + IKsqr(((-1.0) + ((x594 * x595)) + ((x594 * x599)) + (((-1.0) * x588 * x589)) + ((cj1 * x591)) + ((x600 * x601)) + ((x597 * x601)) + (((-1.0) * x592 * x596)) + ((x590 * x592)) + ((sj1 * x593)) + ((x588 * x598)) + (((119895814.745059) * sj1)))) - 1) <= IKFAST_SINCOS_THRESH)
                                                                continue;
                                                            j2array[0] = IKatan2(((((-1.0) * x588 * x590)) + ((x595 * x601)) + (((-1.0) * x589 * x592)) + (((-1.0) * cj1 * x593)) + (((-1.0) * x594 * x597)) + ((sj1 * x591)) + ((x599 * x601)) + (((-119895814.745059) * cj1)) + (((-1.0) * x594 * x600)) + ((x588 * x596)) + ((x592 * x598))), ((-1.0) + ((x594 * x595)) + ((x594 * x599)) + (((-1.0) * x588 * x589)) + ((cj1 * x591)) + ((x600 * x601)) + ((x597 * x601)) + (((-1.0) * x592 * x596)) + ((x590 * x592)) + ((sj1 * x593)) + ((x588 * x598)) + (((119895814.745059) * sj1))));
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
                                                                    IkReal x602 = IKcos(j2);
                                                                    IkReal x603 = IKsin(j2);
                                                                    IkReal x604 = ((4.0e-8) * cj5);
                                                                    IkReal x605 = ((8.79096604904732e-10) * sj1);
                                                                    IkReal x606 = ((0.1135) * cj5);
                                                                    IkReal x607 = ((0.1135) * sj5);
                                                                    IkReal x608 = ((0.452) * cj1);
                                                                    IkReal x609 = ((4.0e-8) * sj5);
                                                                    IkReal x610 = ((0.452000001519335) * sj1);
                                                                    IkReal x611 = (cj1 * x603);
                                                                    evalcond[0] = ((((-1.0) * r20 * x609)) + (((-1.0) * r20 * x606)) + (((0.452) * sj1 * x603)) + ((r21 * x607)) + (((-1.0) * pz)) + x608 + ((x602 * x608)) + (((-1.0) * r21 * x604)));
                                                                    evalcond[1] = ((-0.105400003684668) + (((-1.0) * r01 * x604)) + (((-1.0) * r00 * x606)) + (((-1.0) * r00 * x609)) + (((-8.79096604904732e-10) * x611)) + (((-1.0) * px)) + x605 + ((x602 * x605)) + ((r01 * x607)));
                                                                    evalcond[2] = ((1.20750346625316e-7) + (((-1.0) * r10 * x606)) + (((-1.0) * r10 * x609)) + (((-0.452000001519335) * x611)) + (((-1.0) * py)) + (((-1.0) * r11 * x604)) + ((r11 * x607)) + x610 + ((x602 * x610)));
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
                                                                    IkReal x612 = (((cj5 * r11 * sj4)) + ((r10 * sj4 * sj5)) + (((-1.0) * cj4 * r12)));
                                                                    j3eval[0] = x612;
                                                                    j3eval[1] = IKsign(x612);
                                                                    if (IKabs(j3eval[0]) < 0.0000010000000000 || IKabs(j3eval[1]) < 0.0000010000000000)
                                                                    {
                                                                        {
                                                                            IkReal j3eval[2];
                                                                            sj0 = -0.19503986;
                                                                            cj0 = -0.98079532;
                                                                            j0 = -2.94529456;
                                                                            IkReal x613 = ((1.0) * sj4);
                                                                            IkReal x614 = ((((-1.0) * cj5 * r01 * x613)) + (((-1.0) * r00 * sj5 * x613)) + ((cj4 * r02)));
                                                                            j3eval[0] = x614;
                                                                            j3eval[1] = IKsign(x614);
                                                                            if (IKabs(j3eval[0]) < 0.0000010000000000 || IKabs(j3eval[1]) < 0.0000010000000000)
                                                                            {
                                                                                {
                                                                                    IkReal j3eval[2];
                                                                                    sj0 = -0.19503986;
                                                                                    cj0 = -0.98079532;
                                                                                    j0 = -2.94529456;
                                                                                    IkReal x615 = cj4 * cj4;
                                                                                    IkReal x616 = cj5 * cj5;
                                                                                    IkReal x617 = r22 * r22;
                                                                                    IkReal x618 = r21 * r21;
                                                                                    IkReal x619 = r20 * r20;
                                                                                    IkReal x620 = (r20 * sj5);
                                                                                    IkReal x621 = (cj5 * r21);
                                                                                    IkReal x622 = ((1.0) * x618);
                                                                                    IkReal x623 = ((1.0) * x619);
                                                                                    IkReal x624 = (x615 * x616);
                                                                                    IkReal x625 = ((2.0) * cj4 * r22 * sj4);
                                                                                    IkReal x626 = ((((-1.0) * x621 * x625)) + (((-2.0) * x615 * x620 * x621)) + ((x619 * x624)) + (((-1.0) * x622 * x624)) + (((-1.0) * x622)) + (((-1.0) * x620 * x625)) + ((x616 * x618)) + (((-1.0) * x617)) + (((2.0) * x620 * x621)) + ((x615 * x617)) + (((-1.0) * x616 * x623)) + (((-1.0) * x615 * x623)));
                                                                                    j3eval[0] = x626;
                                                                                    j3eval[1] = IKsign(x626);
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
                                                                                            IkReal x627 = cj4 * cj4;
                                                                                            IkReal x628 = cj5 * cj5;
                                                                                            IkReal x629 = r22 * r22;
                                                                                            IkReal x630 = r21 * r21;
                                                                                            IkReal x631 = r20 * r20;
                                                                                            IkReal x632 = ((1.0) * sj1);
                                                                                            IkReal x633 = (r22 * sj4);
                                                                                            IkReal x634 = (sj2 * sj5);
                                                                                            IkReal x635 = (cj4 * r20);
                                                                                            IkReal x636 = (cj5 * sj2);
                                                                                            IkReal x637 = (r21 * sj5);
                                                                                            IkReal x638 = (cj4 * r21);
                                                                                            IkReal x639 = ((2.0) * cj5);
                                                                                            IkReal x640 = (cj2 * cj5 * r20);
                                                                                            IkReal x641 = ((1.0) * x630);
                                                                                            IkReal x642 = ((1.0) * cj1 * cj2);
                                                                                            IkReal x643 = ((1.0) * x631);
                                                                                            IkReal x644 = (x627 * x628);
                                                                                            CheckValue<IkReal> x645 = IKatan2WithCheck(IkReal((((cj2 * sj1 * x637)) + (((-1.0) * x633 * x642)) + (((-1.0) * x632 * x636 * x638)) + (((-1.0) * sj5 * x635 * x642)) + (((-1.0) * x632 * x634 * x635)) + (((-1.0) * cj1 * r21 * x634)) + (((-1.0) * x632 * x640)) + (((-1.0) * cj5 * x638 * x642)) + (((-1.0) * sj2 * x632 * x633)) + ((cj1 * r20 * x636)))), IkReal(((((-1.0) * cj2 * sj5 * x632 * x635)) + (((-1.0) * r21 * x632 * x634)) + ((cj1 * x640)) + ((cj1 * sj2 * x633)) + (((-1.0) * cj2 * cj5 * x632 * x638)) + ((r20 * sj1 * x636)) + (((-1.0) * x637 * x642)) + ((cj1 * x634 * x635)) + ((cj1 * x636 * x638)) + (((-1.0) * cj2 * x632 * x633)))), IKFAST_ATAN2_MAGTHRESH);
                                                                                            if (!x645.valid)
                                                                                            {
                                                                                                continue;
                                                                                            }
                                                                                            CheckValue<IkReal> x646 = IKPowWithIntegerCheck(IKsign((((r20 * x637 * x639)) + (((-1.0) * r20 * x627 * x637 * x639)) + (((-2.0) * sj5 * x633 * x635)) + (((-1.0) * x627 * x643)) + ((x628 * x630)) + (((-1.0) * x641 * x644)) + (((-1.0) * x633 * x638 * x639)) + (((-1.0) * x641)) + ((x631 * x644)) + (((-1.0) * x629)) + (((-1.0) * x628 * x643)) + ((x627 * x629)))), -1);
                                                                                            if (!x646.valid)
                                                                                            {
                                                                                                continue;
                                                                                            }
                                                                                            j3array[0] = ((-1.5707963267949) + (x645.value) + (((1.5707963267949) * (x646.value))));
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
                                                                                                    IkReal x647 = IKcos(j3);
                                                                                                    IkReal x648 = IKsin(j3);
                                                                                                    IkReal x649 = ((1.0) * r11);
                                                                                                    IkReal x650 = ((1.94490399315206e-9) * sj2);
                                                                                                    IkReal x651 = (sj1 * sj2);
                                                                                                    IkReal x652 = (cj2 * sj1);
                                                                                                    IkReal x653 = (cj4 * r01);
                                                                                                    IkReal x654 = (cj1 * sj2);
                                                                                                    IkReal x655 = (cj1 * cj2);
                                                                                                    IkReal x656 = ((1.0) * r21);
                                                                                                    IkReal x657 = (cj5 * x647);
                                                                                                    IkReal x658 = (sj4 * x647);
                                                                                                    IkReal x659 = (cj5 * x648);
                                                                                                    IkReal x660 = (sj5 * x647);
                                                                                                    IkReal x661 = ((1.0) * x648);
                                                                                                    IkReal x662 = (cj4 * sj5 * x661);
                                                                                                    evalcond[0] = ((((-1.0) * x652)) + (((-1.0) * sj5 * x648 * x656)) + ((r22 * x658)) + ((cj4 * r20 * x660)) + ((cj4 * r21 * x657)) + x654 + ((r20 * x659)));
                                                                                                    evalcond[1] = ((((-1.0) * r20 * x662)) + (((-1.0) * x656 * x660)) + (((-1.0) * cj4 * x656 * x659)) + x651 + x655 + (((-1.0) * r22 * sj4 * x661)) + ((r20 * x657)));
                                                                                                    evalcond[2] = (((sj1 * x650)) + ((r02 * x658)) + ((x653 * x657)) + ((cj4 * r00 * x660)) + (((-1.0) * r01 * sj5 * x661)) + (((1.94490399315206e-9) * x655)) + ((r00 * x659)));
                                                                                                    evalcond[3] = (((cj4 * r11 * x657)) + ((cj4 * r10 * x660)) + (((-1.0) * sj5 * x648 * x649)) + (((1.00000000336136) * x651)) + (((1.00000000336136) * x655)) + ((r10 * x659)) + ((r12 * x658)));
                                                                                                    evalcond[4] = ((((-1.0) * r02 * sj4 * x661)) + (((-1.0) * x653 * x659)) + (((-1.0) * cj1 * x650)) + (((-1.0) * r00 * x662)) + (((-1.0) * r01 * x660)) + (((1.94490399315206e-9) * x652)) + ((r00 * x657)));
                                                                                                    evalcond[5] = ((((-1.0) * x649 * x660)) + (((-1.0) * r10 * x662)) + (((-1.0) * r12 * sj4 * x661)) + (((-1.00000000336136) * x654)) + (((-1.0) * cj4 * x649 * x659)) + (((1.00000000336136) * x652)) + ((r10 * x657)));
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
                                                                                    IkReal x663 = (cj1 * cj5);
                                                                                    IkReal x664 = ((1.00000000336136) * r20);
                                                                                    IkReal x665 = ((1.0) * sj4);
                                                                                    IkReal x666 = (cj4 * sj2);
                                                                                    IkReal x667 = (sj1 * sj5);
                                                                                    IkReal x668 = ((1.00000000336136) * r21);
                                                                                    IkReal x669 = (cj5 * sj1);
                                                                                    IkReal x670 = (sj2 * sj4);
                                                                                    IkReal x671 = (cj2 * r10);
                                                                                    IkReal x672 = ((1.0) * cj4);
                                                                                    IkReal x673 = ((1.00000000336136) * r22);
                                                                                    IkReal x674 = (cj1 * cj2);
                                                                                    IkReal x675 = (cj1 * sj5);
                                                                                    IkReal x676 = (cj2 * r11);
                                                                                    CheckValue<IkReal> x677 = IKPowWithIntegerCheck(IKsign(((((-1.0) * cj5 * r01 * x665)) + (((-1.0) * r00 * sj5 * x665)) + ((cj4 * r02)))), -1);
                                                                                    if (!x677.valid)
                                                                                    {
                                                                                        continue;
                                                                                    }
                                                                                    CheckValue<IkReal> x678 = IKatan2WithCheck(IkReal(((((-1.0) * sj1 * x670 * x673)) + (((-1.0) * cj2 * cj4 * x663 * x668)) + (((-1.0) * sj4 * x673 * x674)) + ((r10 * x666 * x675)) + ((cj1 * r12 * x670)) + (((-1.0) * x666 * x668 * x669)) + (((-1.0) * cj4 * sj5 * x664 * x674)) + (((-1.0) * x669 * x672 * x676)) + (((-1.0) * x667 * x671 * x672)) + (((-1.0) * cj2 * r12 * sj1 * x665)) + ((r11 * x663 * x666)) + (((-1.0) * x664 * x666 * x667)))), IkReal((((sj2 * x664 * x669)) + ((cj2 * x663 * x664)) + (((-1.0) * sj2 * x667 * x668)) + (((-1.0) * x667 * x676)) + (((-1.0) * sj5 * x668 * x674)) + (((-1.0) * r10 * sj2 * x663)) + ((x669 * x671)) + ((r11 * sj2 * x675)))), IKFAST_ATAN2_MAGTHRESH);
                                                                                    if (!x678.valid)
                                                                                    {
                                                                                        continue;
                                                                                    }
                                                                                    j3array[0] = ((-1.5707963267949) + (((1.5707963267949) * (x677.value))) + (x678.value));
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
                                                                                            IkReal x679 = IKcos(j3);
                                                                                            IkReal x680 = IKsin(j3);
                                                                                            IkReal x681 = ((1.0) * r11);
                                                                                            IkReal x682 = ((1.94490399315206e-9) * sj2);
                                                                                            IkReal x683 = (sj1 * sj2);
                                                                                            IkReal x684 = (cj2 * sj1);
                                                                                            IkReal x685 = (cj4 * r01);
                                                                                            IkReal x686 = (cj1 * sj2);
                                                                                            IkReal x687 = (cj1 * cj2);
                                                                                            IkReal x688 = ((1.0) * r21);
                                                                                            IkReal x689 = (cj5 * x679);
                                                                                            IkReal x690 = (sj4 * x679);
                                                                                            IkReal x691 = (cj5 * x680);
                                                                                            IkReal x692 = (sj5 * x679);
                                                                                            IkReal x693 = ((1.0) * x680);
                                                                                            IkReal x694 = (cj4 * sj5 * x693);
                                                                                            evalcond[0] = ((((-1.0) * sj5 * x680 * x688)) + ((cj4 * r21 * x689)) + ((r20 * x691)) + ((cj4 * r20 * x692)) + ((r22 * x690)) + x686 + (((-1.0) * x684)));
                                                                                            evalcond[1] = ((((-1.0) * x688 * x692)) + x683 + x687 + ((r20 * x689)) + (((-1.0) * r20 * x694)) + (((-1.0) * r22 * sj4 * x693)) + (((-1.0) * cj4 * x688 * x691)));
                                                                                            evalcond[2] = ((((-1.0) * r01 * sj5 * x693)) + (((1.94490399315206e-9) * x687)) + ((x685 * x689)) + ((r02 * x690)) + ((sj1 * x682)) + ((r00 * x691)) + ((cj4 * r00 * x692)));
                                                                                            evalcond[3] = ((((-1.0) * sj5 * x680 * x681)) + ((cj4 * r10 * x692)) + (((1.00000000336136) * x683)) + (((1.00000000336136) * x687)) + ((r12 * x690)) + ((r10 * x691)) + ((cj4 * r11 * x689)));
                                                                                            evalcond[4] = ((((-1.0) * r00 * x694)) + (((1.94490399315206e-9) * x684)) + (((-1.0) * cj1 * x682)) + (((-1.0) * x685 * x691)) + (((-1.0) * r02 * sj4 * x693)) + ((r00 * x689)) + (((-1.0) * r01 * x692)));
                                                                                            evalcond[5] = ((((1.00000000336136) * x684)) + (((-1.0) * r12 * sj4 * x693)) + ((r10 * x689)) + (((-1.0) * r10 * x694)) + (((-1.0) * cj4 * x681 * x691)) + (((-1.0) * x681 * x692)) + (((-1.00000000336136) * x686)));
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
                                                                            IkReal x695 = (sj1 * sj5);
                                                                            IkReal x696 = ((1.0) * r01);
                                                                            IkReal x697 = (cj1 * sj2);
                                                                            IkReal x698 = (cj2 * sj1);
                                                                            IkReal x699 = (r02 * sj4);
                                                                            IkReal x700 = (cj1 * cj2);
                                                                            IkReal x701 = ((1.94490399315206e-9) * r20);
                                                                            IkReal x702 = (sj1 * sj2);
                                                                            IkReal x703 = (cj4 * r00);
                                                                            IkReal x704 = ((1.94490399315206e-9) * r21);
                                                                            IkReal x705 = (cj5 * r00);
                                                                            IkReal x706 = ((1.94490399315206e-9) * r22 * sj4);
                                                                            IkReal x707 = (cj4 * cj5 * r01);
                                                                            IkReal x708 = (cj4 * cj5 * x704);
                                                                            CheckValue<IkReal> x709 = IKPowWithIntegerCheck(IKsign((((cj5 * r11 * sj4)) + ((r10 * sj4 * sj5)) + (((-1.0) * cj4 * r12)))), -1);
                                                                            if (!x709.valid)
                                                                            {
                                                                                continue;
                                                                            }
                                                                            CheckValue<IkReal> x710 = IKatan2WithCheck(IkReal(((((-1.0) * cj5 * x698 * x701)) + ((x702 * x705)) + ((x700 * x705)) + ((cj2 * x695 * x704)) + (((-1.0) * sj5 * x696 * x700)) + (((-1.0) * sj2 * x695 * x696)) + ((cj5 * x697 * x701)) + (((-1.0) * sj5 * x697 * x704)))), IkReal((((x702 * x707)) + ((cj4 * sj5 * x697 * x701)) + ((x700 * x707)) + (((-1.0) * cj2 * cj4 * x695 * x701)) + ((x697 * x708)) + ((x697 * x706)) + (((-1.0) * x698 * x708)) + (((-1.0) * x698 * x706)) + ((sj5 * x700 * x703)) + ((sj2 * x695 * x703)) + ((x699 * x700)) + ((x699 * x702)))), IKFAST_ATAN2_MAGTHRESH);
                                                                            if (!x710.valid)
                                                                            {
                                                                                continue;
                                                                            }
                                                                            j3array[0] = ((-1.5707963267949) + (((1.5707963267949) * (x709.value))) + (x710.value));
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
                                                                                    IkReal x711 = IKcos(j3);
                                                                                    IkReal x712 = IKsin(j3);
                                                                                    IkReal x713 = ((1.0) * r11);
                                                                                    IkReal x714 = ((1.94490399315206e-9) * sj2);
                                                                                    IkReal x715 = (sj1 * sj2);
                                                                                    IkReal x716 = (cj2 * sj1);
                                                                                    IkReal x717 = (cj4 * r01);
                                                                                    IkReal x718 = (cj1 * sj2);
                                                                                    IkReal x719 = (cj1 * cj2);
                                                                                    IkReal x720 = ((1.0) * r21);
                                                                                    IkReal x721 = (cj5 * x711);
                                                                                    IkReal x722 = (sj4 * x711);
                                                                                    IkReal x723 = (cj5 * x712);
                                                                                    IkReal x724 = (sj5 * x711);
                                                                                    IkReal x725 = ((1.0) * x712);
                                                                                    IkReal x726 = (cj4 * sj5 * x725);
                                                                                    evalcond[0] = (((cj4 * r21 * x721)) + ((r20 * x723)) + (((-1.0) * x716)) + (((-1.0) * sj5 * x712 * x720)) + x718 + ((cj4 * r20 * x724)) + ((r22 * x722)));
                                                                                    evalcond[1] = (((r20 * x721)) + (((-1.0) * x720 * x724)) + (((-1.0) * cj4 * x720 * x723)) + x719 + x715 + (((-1.0) * r20 * x726)) + (((-1.0) * r22 * sj4 * x725)));
                                                                                    evalcond[2] = ((((1.94490399315206e-9) * x719)) + ((x717 * x721)) + ((r00 * x723)) + ((r02 * x722)) + ((sj1 * x714)) + (((-1.0) * r01 * sj5 * x725)) + ((cj4 * r00 * x724)));
                                                                                    evalcond[3] = (((r10 * x723)) + ((cj4 * r10 * x724)) + ((cj4 * r11 * x721)) + (((-1.0) * sj5 * x712 * x713)) + (((1.00000000336136) * x719)) + (((1.00000000336136) * x715)) + ((r12 * x722)));
                                                                                    evalcond[4] = ((((-1.0) * r00 * x726)) + (((1.94490399315206e-9) * x716)) + (((-1.0) * r01 * x724)) + ((r00 * x721)) + (((-1.0) * cj1 * x714)) + (((-1.0) * r02 * sj4 * x725)) + (((-1.0) * x717 * x723)));
                                                                                    evalcond[5] = ((((-1.0) * x713 * x724)) + (((-1.0) * r12 * sj4 * x725)) + ((r10 * x721)) + (((-1.00000000336136) * x718)) + (((1.00000000336136) * x716)) + (((-1.0) * r10 * x726)) + (((-1.0) * cj4 * x713 * x723)));
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
                                                                IkReal x727 = (cj1 * sj5);
                                                                IkReal x728 = ((0.251106194690265) * r21);
                                                                IkReal x729 = ((45.5012563770905) * r00);
                                                                IkReal x730 = ((2.21238938053097) * pz);
                                                                IkReal x731 = (sj1 * sj5);
                                                                IkReal x732 = ((1137531409.42726) * px);
                                                                IkReal x733 = (cj1 * cj5);
                                                                IkReal x734 = ((0.251106194690265) * r20);
                                                                IkReal x735 = ((129109814.969994) * r01);
                                                                IkReal x736 = ((129109814.969994) * r00);
                                                                IkReal x737 = ((8.84955752212389e-8) * r20);
                                                                IkReal x738 = ((8.84955752212389e-8) * r21);
                                                                IkReal x739 = ((45.5012563770905) * r01);
                                                                IkReal x740 = (cj5 * sj1);
                                                                if (IKabs((((x734 * x740)) + ((x738 * x740)) + ((x731 * x737)) + (((-1.0) * x727 * x735)) + ((x727 * x729)) + ((x733 * x739)) + ((x733 * x736)) + (((-119895814.745059) * cj1)) + ((sj1 * x730)) + (((-1.0) * x728 * x731)) + ((cj1 * x732)))) < IKFAST_ATAN2_MAGTHRESH && IKabs(((-1.0) + (((-1.0) * x727 * x728)) + ((x731 * x735)) + ((x727 * x737)) + ((x733 * x738)) + ((x733 * x734)) + (((-1.0) * sj1 * x732)) + (((-1.0) * x739 * x740)) + (((-1.0) * x736 * x740)) + ((cj1 * x730)) + (((119895814.745059) * sj1)) + (((-1.0) * x729 * x731)))) < IKFAST_ATAN2_MAGTHRESH && IKabs(IKsqr((((x734 * x740)) + ((x738 * x740)) + ((x731 * x737)) + (((-1.0) * x727 * x735)) + ((x727 * x729)) + ((x733 * x739)) + ((x733 * x736)) + (((-119895814.745059) * cj1)) + ((sj1 * x730)) + (((-1.0) * x728 * x731)) + ((cj1 * x732)))) + IKsqr(((-1.0) + (((-1.0) * x727 * x728)) + ((x731 * x735)) + ((x727 * x737)) + ((x733 * x738)) + ((x733 * x734)) + (((-1.0) * sj1 * x732)) + (((-1.0) * x739 * x740)) + (((-1.0) * x736 * x740)) + ((cj1 * x730)) + (((119895814.745059) * sj1)) + (((-1.0) * x729 * x731)))) - 1) <= IKFAST_SINCOS_THRESH)
                                                                    continue;
                                                                j2array[0] = IKatan2((((x734 * x740)) + ((x738 * x740)) + ((x731 * x737)) + (((-1.0) * x727 * x735)) + ((x727 * x729)) + ((x733 * x739)) + ((x733 * x736)) + (((-119895814.745059) * cj1)) + ((sj1 * x730)) + (((-1.0) * x728 * x731)) + ((cj1 * x732))), ((-1.0) + (((-1.0) * x727 * x728)) + ((x731 * x735)) + ((x727 * x737)) + ((x733 * x738)) + ((x733 * x734)) + (((-1.0) * sj1 * x732)) + (((-1.0) * x739 * x740)) + (((-1.0) * x736 * x740)) + ((cj1 * x730)) + (((119895814.745059) * sj1)) + (((-1.0) * x729 * x731))));
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
                                                                        IkReal x741 = IKsin(j2);
                                                                        IkReal x742 = IKcos(j2);
                                                                        IkReal x743 = ((4.0e-8) * cj5);
                                                                        IkReal x744 = ((8.79096604904732e-10) * sj1);
                                                                        IkReal x745 = ((0.1135) * cj5);
                                                                        IkReal x746 = ((0.1135) * sj5);
                                                                        IkReal x747 = ((0.452) * cj1);
                                                                        IkReal x748 = ((4.0e-8) * sj5);
                                                                        IkReal x749 = ((0.452000001519335) * sj1);
                                                                        IkReal x750 = (cj1 * x741);
                                                                        evalcond[0] = ((((-1.0) * r20 * x748)) + (((-1.0) * r20 * x745)) + (((-1.0) * r21 * x743)) + ((x742 * x747)) + (((-1.0) * pz)) + ((r21 * x746)) + x747 + (((0.452) * sj1 * x741)));
                                                                        evalcond[1] = ((0.105400003684668) + (((-1.0) * x742 * x744)) + (((8.79096604904732e-10) * x750)) + (((-1.0) * px)) + (((-1.0) * r00 * x745)) + (((-1.0) * r00 * x748)) + (((-1.0) * r01 * x743)) + ((r01 * x746)) + (((-1.0) * x744)));
                                                                        evalcond[2] = ((-1.20750346625317e-7) + (((0.452000001519335) * x750)) + (((-1.0) * r11 * x743)) + (((-1.0) * x742 * x749)) + (((-1.0) * py)) + ((r11 * x746)) + (((-1.0) * r10 * x748)) + (((-1.0) * r10 * x745)) + (((-1.0) * x749)));
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
                                                                        IkReal x751 = (((cj5 * r11 * sj4)) + ((r10 * sj4 * sj5)) + (((-1.0) * cj4 * r12)));
                                                                        j3eval[0] = x751;
                                                                        j3eval[1] = IKsign(x751);
                                                                        if (IKabs(j3eval[0]) < 0.0000010000000000 || IKabs(j3eval[1]) < 0.0000010000000000)
                                                                        {
                                                                            {
                                                                                IkReal j3eval[2];
                                                                                sj0 = 0.19503986;
                                                                                cj0 = 0.98079532;
                                                                                j0 = 0.19629809;
                                                                                IkReal x752 = ((1.0) * sj4);
                                                                                IkReal x753 = ((((-1.0) * cj5 * r01 * x752)) + (((-1.0) * r00 * sj5 * x752)) + ((cj4 * r02)));
                                                                                j3eval[0] = x753;
                                                                                j3eval[1] = IKsign(x753);
                                                                                if (IKabs(j3eval[0]) < 0.0000010000000000 || IKabs(j3eval[1]) < 0.0000010000000000)
                                                                                {
                                                                                    {
                                                                                        IkReal j3eval[2];
                                                                                        sj0 = 0.19503986;
                                                                                        cj0 = 0.98079532;
                                                                                        j0 = 0.19629809;
                                                                                        IkReal x754 = cj4 * cj4;
                                                                                        IkReal x755 = cj5 * cj5;
                                                                                        IkReal x756 = r22 * r22;
                                                                                        IkReal x757 = r21 * r21;
                                                                                        IkReal x758 = r20 * r20;
                                                                                        IkReal x759 = (r20 * sj5);
                                                                                        IkReal x760 = (cj5 * r21);
                                                                                        IkReal x761 = ((1.0) * x757);
                                                                                        IkReal x762 = ((1.0) * x758);
                                                                                        IkReal x763 = (x754 * x755);
                                                                                        IkReal x764 = ((2.0) * cj4 * r22 * sj4);
                                                                                        IkReal x765 = ((((-1.0) * x761)) + (((-2.0) * x754 * x759 * x760)) + (((-1.0) * x755 * x762)) + ((x758 * x763)) + (((-1.0) * x754 * x762)) + (((2.0) * x759 * x760)) + (((-1.0) * x756)) + ((x755 * x757)) + (((-1.0) * x760 * x764)) + (((-1.0) * x761 * x763)) + (((-1.0) * x759 * x764)) + ((x754 * x756)));
                                                                                        j3eval[0] = x765;
                                                                                        j3eval[1] = IKsign(x765);
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
                                                                                                IkReal x766 = cj4 * cj4;
                                                                                                IkReal x767 = cj5 * cj5;
                                                                                                IkReal x768 = r22 * r22;
                                                                                                IkReal x769 = r21 * r21;
                                                                                                IkReal x770 = r20 * r20;
                                                                                                IkReal x771 = ((1.0) * sj1);
                                                                                                IkReal x772 = (r22 * sj4);
                                                                                                IkReal x773 = (sj2 * sj5);
                                                                                                IkReal x774 = (cj4 * r20);
                                                                                                IkReal x775 = (cj5 * sj2);
                                                                                                IkReal x776 = (r21 * sj5);
                                                                                                IkReal x777 = (cj4 * r21);
                                                                                                IkReal x778 = ((2.0) * cj5);
                                                                                                IkReal x779 = (cj2 * cj5 * r20);
                                                                                                IkReal x780 = ((1.0) * x769);
                                                                                                IkReal x781 = ((1.0) * cj1 * cj2);
                                                                                                IkReal x782 = ((1.0) * x770);
                                                                                                IkReal x783 = (x766 * x767);
                                                                                                CheckValue<IkReal> x784 = IKatan2WithCheck(IkReal(((((-1.0) * sj5 * x774 * x781)) + (((-1.0) * cj5 * x777 * x781)) + (((-1.0) * x772 * x781)) + (((-1.0) * cj1 * r21 * x773)) + ((cj1 * r20 * x775)) + (((-1.0) * x771 * x775 * x777)) + (((-1.0) * x771 * x779)) + ((cj2 * sj1 * x776)) + (((-1.0) * sj2 * x771 * x772)) + (((-1.0) * x771 * x773 * x774)))), IkReal(((((-1.0) * cj2 * x771 * x772)) + ((cj1 * sj2 * x772)) + ((cj1 * x775 * x777)) + ((cj1 * x779)) + ((cj1 * x773 * x774)) + (((-1.0) * cj2 * sj5 * x771 * x774)) + (((-1.0) * x776 * x781)) + (((-1.0) * cj2 * cj5 * x771 * x777)) + ((r20 * sj1 * x775)) + (((-1.0) * r21 * x771 * x773)))), IKFAST_ATAN2_MAGTHRESH);
                                                                                                if (!x784.valid)
                                                                                                {
                                                                                                    continue;
                                                                                                }
                                                                                                CheckValue<IkReal> x785 = IKPowWithIntegerCheck(IKsign((((r20 * x776 * x778)) + ((x770 * x783)) + (((-1.0) * x780)) + (((-1.0) * x772 * x777 * x778)) + (((-1.0) * x767 * x782)) + (((-2.0) * sj5 * x772 * x774)) + (((-1.0) * r20 * x766 * x776 * x778)) + (((-1.0) * x766 * x782)) + (((-1.0) * x768)) + ((x766 * x768)) + ((x767 * x769)) + (((-1.0) * x780 * x783)))), -1);
                                                                                                if (!x785.valid)
                                                                                                {
                                                                                                    continue;
                                                                                                }
                                                                                                j3array[0] = ((-1.5707963267949) + (x784.value) + (((1.5707963267949) * (x785.value))));
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
                                                                                                        IkReal x786 = IKcos(j3);
                                                                                                        IkReal x787 = IKsin(j3);
                                                                                                        IkReal x788 = ((1.0) * r11);
                                                                                                        IkReal x789 = ((1.94490399315206e-9) * sj2);
                                                                                                        IkReal x790 = (sj1 * sj2);
                                                                                                        IkReal x791 = (cj2 * sj1);
                                                                                                        IkReal x792 = (cj4 * r01);
                                                                                                        IkReal x793 = (cj1 * sj2);
                                                                                                        IkReal x794 = (cj1 * cj2);
                                                                                                        IkReal x795 = ((1.0) * r21);
                                                                                                        IkReal x796 = (cj5 * x786);
                                                                                                        IkReal x797 = (sj4 * x786);
                                                                                                        IkReal x798 = (cj5 * x787);
                                                                                                        IkReal x799 = (sj5 * x786);
                                                                                                        IkReal x800 = ((1.0) * x787);
                                                                                                        IkReal x801 = (cj4 * sj5 * x800);
                                                                                                        evalcond[0] = (((r20 * x798)) + (((-1.0) * sj5 * x787 * x795)) + ((cj4 * r20 * x799)) + ((cj4 * r21 * x796)) + ((r22 * x797)) + x793 + (((-1.0) * x791)));
                                                                                                        evalcond[1] = (((r20 * x796)) + (((-1.0) * x795 * x799)) + (((-1.0) * r22 * sj4 * x800)) + (((-1.0) * r20 * x801)) + x790 + x794 + (((-1.0) * cj4 * x795 * x798)));
                                                                                                        evalcond[2] = (((r00 * x798)) + (((-1.94490399315206e-9) * x794)) + ((cj4 * r00 * x799)) + ((r02 * x797)) + ((x792 * x796)) + (((-1.0) * r01 * sj5 * x800)) + (((-1.0) * sj1 * x789)));
                                                                                                        evalcond[3] = (((r10 * x798)) + (((-1.0) * sj5 * x787 * x788)) + ((cj4 * r10 * x799)) + (((-1.00000000336136) * x790)) + (((-1.00000000336136) * x794)) + ((r12 * x797)) + ((cj4 * r11 * x796)));
                                                                                                        evalcond[4] = (((r00 * x796)) + (((-1.0) * r01 * x799)) + (((-1.94490399315206e-9) * x791)) + ((cj1 * x789)) + (((-1.0) * r00 * x801)) + (((-1.0) * x792 * x798)) + (((-1.0) * r02 * sj4 * x800)));
                                                                                                        evalcond[5] = (((r10 * x796)) + (((-1.0) * x788 * x799)) + (((-1.0) * cj4 * x788 * x798)) + (((-1.00000000336136) * x791)) + (((1.00000000336136) * x793)) + (((-1.0) * r12 * sj4 * x800)) + (((-1.0) * r10 * x801)));
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
                                                                                        IkReal x802 = (cj1 * cj5);
                                                                                        IkReal x803 = ((1.00000000336136) * r20);
                                                                                        IkReal x804 = ((1.0) * sj4);
                                                                                        IkReal x805 = (cj4 * sj2);
                                                                                        IkReal x806 = (sj1 * sj5);
                                                                                        IkReal x807 = ((1.00000000336136) * r21);
                                                                                        IkReal x808 = (cj5 * sj1);
                                                                                        IkReal x809 = (sj2 * sj4);
                                                                                        IkReal x810 = (cj2 * r10);
                                                                                        IkReal x811 = ((1.0) * cj4);
                                                                                        IkReal x812 = ((1.00000000336136) * r22);
                                                                                        IkReal x813 = (cj1 * cj2);
                                                                                        IkReal x814 = (cj1 * sj5);
                                                                                        IkReal x815 = (cj2 * r11);
                                                                                        CheckValue<IkReal> x816 = IKPowWithIntegerCheck(IKsign(((((-1.0) * r00 * sj5 * x804)) + (((-1.0) * cj5 * r01 * x804)) + ((cj4 * r02)))), -1);
                                                                                        if (!x816.valid)
                                                                                        {
                                                                                            continue;
                                                                                        }
                                                                                        CheckValue<IkReal> x817 = IKatan2WithCheck(IkReal((((cj2 * cj4 * x802 * x807)) + ((sj1 * x809 * x812)) + (((-1.0) * x806 * x810 * x811)) + ((cj4 * sj5 * x803 * x813)) + (((-1.0) * x808 * x811 * x815)) + ((cj1 * r12 * x809)) + ((r10 * x805 * x814)) + ((x805 * x807 * x808)) + ((x803 * x805 * x806)) + ((r11 * x802 * x805)) + ((sj4 * x812 * x813)) + (((-1.0) * cj2 * r12 * sj1 * x804)))), IkReal(((((-1.0) * cj2 * x802 * x803)) + (((-1.0) * sj2 * x803 * x808)) + ((sj5 * x807 * x813)) + ((x808 * x810)) + ((sj2 * x806 * x807)) + (((-1.0) * r10 * sj2 * x802)) + (((-1.0) * x806 * x815)) + ((r11 * sj2 * x814)))), IKFAST_ATAN2_MAGTHRESH);
                                                                                        if (!x817.valid)
                                                                                        {
                                                                                            continue;
                                                                                        }
                                                                                        j3array[0] = ((-1.5707963267949) + (((1.5707963267949) * (x816.value))) + (x817.value));
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
                                                                                                IkReal x818 = IKcos(j3);
                                                                                                IkReal x819 = IKsin(j3);
                                                                                                IkReal x820 = ((1.0) * r11);
                                                                                                IkReal x821 = ((1.94490399315206e-9) * sj2);
                                                                                                IkReal x822 = (sj1 * sj2);
                                                                                                IkReal x823 = (cj2 * sj1);
                                                                                                IkReal x824 = (cj4 * r01);
                                                                                                IkReal x825 = (cj1 * sj2);
                                                                                                IkReal x826 = (cj1 * cj2);
                                                                                                IkReal x827 = ((1.0) * r21);
                                                                                                IkReal x828 = (cj5 * x818);
                                                                                                IkReal x829 = (sj4 * x818);
                                                                                                IkReal x830 = (cj5 * x819);
                                                                                                IkReal x831 = (sj5 * x818);
                                                                                                IkReal x832 = ((1.0) * x819);
                                                                                                IkReal x833 = (cj4 * sj5 * x832);
                                                                                                evalcond[0] = ((((-1.0) * sj5 * x819 * x827)) + ((r20 * x830)) + (((-1.0) * x823)) + ((r22 * x829)) + ((cj4 * r21 * x828)) + x825 + ((cj4 * r20 * x831)));
                                                                                                evalcond[1] = ((((-1.0) * cj4 * x827 * x830)) + ((r20 * x828)) + (((-1.0) * x827 * x831)) + (((-1.0) * r20 * x833)) + x822 + x826 + (((-1.0) * r22 * sj4 * x832)));
                                                                                                evalcond[2] = ((((-1.0) * r01 * sj5 * x832)) + ((x824 * x828)) + ((r00 * x830)) + (((-1.0) * sj1 * x821)) + (((-1.94490399315206e-9) * x826)) + ((r02 * x829)) + ((cj4 * r00 * x831)));
                                                                                                evalcond[3] = (((r12 * x829)) + ((cj4 * r10 * x831)) + (((-1.0) * sj5 * x819 * x820)) + ((cj4 * r11 * x828)) + (((-1.00000000336136) * x822)) + (((-1.00000000336136) * x826)) + ((r10 * x830)));
                                                                                                evalcond[4] = (((r00 * x828)) + (((-1.0) * r02 * sj4 * x832)) + (((-1.0) * r01 * x831)) + (((-1.0) * r00 * x833)) + (((-1.94490399315206e-9) * x823)) + ((cj1 * x821)) + (((-1.0) * x824 * x830)));
                                                                                                evalcond[5] = ((((-1.0) * cj4 * x820 * x830)) + (((-1.0) * x820 * x831)) + (((-1.0) * r10 * x833)) + (((-1.00000000336136) * x823)) + ((r10 * x828)) + (((-1.0) * r12 * sj4 * x832)) + (((1.00000000336136) * x825)));
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
                                                                                IkReal x834 = (sj1 * sj5);
                                                                                IkReal x835 = ((1.0) * r01);
                                                                                IkReal x836 = (cj2 * sj1);
                                                                                IkReal x837 = (cj1 * sj2);
                                                                                IkReal x838 = (r02 * sj4);
                                                                                IkReal x839 = (cj1 * cj2);
                                                                                IkReal x840 = ((1.94490399315206e-9) * r20);
                                                                                IkReal x841 = (sj1 * sj2);
                                                                                IkReal x842 = (cj4 * r00);
                                                                                IkReal x843 = (cj5 * r00);
                                                                                IkReal x844 = ((1.94490399315206e-9) * r21);
                                                                                IkReal x845 = ((1.94490399315206e-9) * r22 * sj4);
                                                                                IkReal x846 = (cj4 * cj5 * r01);
                                                                                IkReal x847 = (cj4 * cj5 * x844);
                                                                                CheckValue<IkReal> x848 = IKPowWithIntegerCheck(IKsign((((cj5 * r11 * sj4)) + ((r10 * sj4 * sj5)) + (((-1.0) * cj4 * r12)))), -1);
                                                                                if (!x848.valid)
                                                                                {
                                                                                    continue;
                                                                                }
                                                                                CheckValue<IkReal> x849 = IKatan2WithCheck(IkReal((((x839 * x843)) + ((sj5 * x837 * x844)) + (((-1.0) * cj5 * x837 * x840)) + ((cj5 * x836 * x840)) + (((-1.0) * cj2 * x834 * x844)) + (((-1.0) * sj2 * x834 * x835)) + (((-1.0) * sj5 * x835 * x839)) + ((x841 * x843)))), IkReal((((x838 * x841)) + ((x838 * x839)) + ((sj2 * x834 * x842)) + ((cj2 * cj4 * x834 * x840)) + ((x839 * x846)) + ((sj5 * x839 * x842)) + (((-1.0) * cj4 * sj5 * x837 * x840)) + (((-1.0) * x837 * x847)) + (((-1.0) * x837 * x845)) + ((x836 * x845)) + ((x836 * x847)) + ((x841 * x846)))), IKFAST_ATAN2_MAGTHRESH);
                                                                                if (!x849.valid)
                                                                                {
                                                                                    continue;
                                                                                }
                                                                                j3array[0] = ((-1.5707963267949) + (((1.5707963267949) * (x848.value))) + (x849.value));
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
                                                                                        IkReal x850 = IKcos(j3);
                                                                                        IkReal x851 = IKsin(j3);
                                                                                        IkReal x852 = ((1.0) * r11);
                                                                                        IkReal x853 = ((1.94490399315206e-9) * sj2);
                                                                                        IkReal x854 = (sj1 * sj2);
                                                                                        IkReal x855 = (cj2 * sj1);
                                                                                        IkReal x856 = (cj4 * r01);
                                                                                        IkReal x857 = (cj1 * sj2);
                                                                                        IkReal x858 = (cj1 * cj2);
                                                                                        IkReal x859 = ((1.0) * r21);
                                                                                        IkReal x860 = (cj5 * x850);
                                                                                        IkReal x861 = (sj4 * x850);
                                                                                        IkReal x862 = (cj5 * x851);
                                                                                        IkReal x863 = (sj5 * x850);
                                                                                        IkReal x864 = ((1.0) * x851);
                                                                                        IkReal x865 = (cj4 * sj5 * x864);
                                                                                        evalcond[0] = ((((-1.0) * x855)) + ((r22 * x861)) + ((cj4 * r20 * x863)) + ((cj4 * r21 * x860)) + (((-1.0) * sj5 * x851 * x859)) + x857 + ((r20 * x862)));
                                                                                        evalcond[1] = ((((-1.0) * cj4 * x859 * x862)) + (((-1.0) * x859 * x863)) + (((-1.0) * r20 * x865)) + x854 + x858 + (((-1.0) * r22 * sj4 * x864)) + ((r20 * x860)));
                                                                                        evalcond[2] = (((r00 * x862)) + (((-1.0) * r01 * sj5 * x864)) + ((cj4 * r00 * x863)) + ((x856 * x860)) + (((-1.0) * sj1 * x853)) + ((r02 * x861)) + (((-1.94490399315206e-9) * x858)));
                                                                                        evalcond[3] = (((cj4 * r10 * x863)) + ((r10 * x862)) + ((r12 * x861)) + (((-1.0) * sj5 * x851 * x852)) + ((cj4 * r11 * x860)) + (((-1.00000000336136) * x854)) + (((-1.00000000336136) * x858)));
                                                                                        evalcond[4] = ((((-1.0) * r01 * x863)) + ((r00 * x860)) + (((-1.0) * r00 * x865)) + ((cj1 * x853)) + (((-1.0) * x856 * x862)) + (((-1.0) * r02 * sj4 * x864)) + (((-1.94490399315206e-9) * x855)));
                                                                                        evalcond[5] = ((((-1.0) * r10 * x865)) + (((-1.0) * r12 * sj4 * x864)) + ((r10 * x860)) + (((-1.0) * cj4 * x852 * x862)) + (((-1.0) * x852 * x863)) + (((1.00000000336136) * x857)) + (((-1.00000000336136) * x855)));
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
                                        IkReal x866 = ((50.0951015765574) * cj0);
                                        IkReal x867 = ((9.96185595330477) * sj0);
                                        IkReal x868 = ((8.81580172858829e-7) * sj5);
                                        IkReal x869 = ((2.50148374048693) * cj5);
                                        IkReal x870 = (r20 * sj1);
                                        IkReal x871 = (cj1 * r21);
                                        IkReal x872 = (cj0 * cj5);
                                        IkReal x873 = (cj1 * cj5);
                                        IkReal x874 = ((12.8255) * r10);
                                        IkReal x875 = (sj0 * sj1);
                                        IkReal x876 = ((22.0395043214707) * pz);
                                        IkReal x877 = (cj1 * sj0);
                                        IkReal x878 = ((113.0) * py);
                                        IkReal x879 = (r21 * sj1);
                                        IkReal x880 = ((4.52e-6) * r11);
                                        IkReal x881 = (cj0 * sj1);
                                        IkReal x882 = ((110.829870744596) * pz);
                                        IkReal x883 = (cj1 * r20);
                                        IkReal x884 = ((8.81580172858829e-7) * cj5);
                                        IkReal x885 = (cj0 * cj1);
                                        IkReal x886 = (cj5 * sj1);
                                        IkReal x887 = (r20 * x877);
                                        IkReal x888 = ((2.50148374048693) * sj0 * sj5);
                                        IkReal x889 = ((12.8255) * r11 * sj5);
                                        IkReal x890 = ((12.5791903295117) * cj0 * sj5);
                                        IkReal x891 = ((4.43319482978384e-6) * cj0 * sj5);
                                        IkReal x892 = ((4.52e-6) * r10 * sj5);
                                        IkReal x893 = (x867 + x866);
                                        CheckValue<IkReal> x894 = IKPowWithIntegerCheck(IKsign(x893), -1);
                                        if (!x894.valid)
                                        {
                                            continue;
                                        }
                                        CheckValue<IkReal> x895 = IKatan2WithCheck(IkReal((((r21 * x875 * x884)) + ((x881 * x882)) + (((-1.0) * x879 * x890)) + (((-11.6814660888263) * x877)) + ((x873 * x874)) + ((sj0 * x868 * x870)) + (((-2.50148374048693) * r21 * sj5 * x875)) + (((2.32297718890889) * x885)) + ((cj1 * x878)) + ((x875 * x876)) + ((x870 * x891)) + ((sj0 * x869 * x870)) + ((cj1 * x892)) + (((12.5791903295117) * x870 * x872)) + (((-1.0) * cj1 * x889)) + (((4.43319482978384e-6) * x872 * x879)) + ((x873 * x880)))), IkReal(((((12.5791903295117) * x872 * x883)) + ((sj1 * x889)) + ((x876 * x877)) + ((x868 * x887)) + ((sj0 * x871 * x884)) + ((x883 * x891)) + ((x882 * x885)) + (((-1.0) * x893)) + (((-1.0) * sj1 * x878)) + (((-2.32297718890889) * x881)) + ((x869 * x887)) + (((11.6814660888263) * x875)) + (((-1.0) * x871 * x888)) + (((-1.0) * x871 * x890)) + (((-1.0) * sj1 * x892)) + (((-1.0) * x874 * x886)) + (((4.43319482978384e-6) * x871 * x872)) + (((-1.0) * x880 * x886)))), IKFAST_ATAN2_MAGTHRESH);
                                        if (!x895.valid)
                                        {
                                            continue;
                                        }
                                        j2array[0] = ((-1.5707963267949) + (((1.5707963267949) * (x894.value))) + (x895.value));
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
                                                IkReal x896 = IKcos(j2);
                                                IkReal x897 = IKsin(j2);
                                                IkReal x898 = ((4.0e-8) * cj5);
                                                IkReal x899 = (cj0 * sj1);
                                                IkReal x900 = ((0.1135) * cj5);
                                                IkReal x901 = ((0.1135) * sj5);
                                                IkReal x902 = ((0.452) * cj1);
                                                IkReal x903 = ((4.0e-8) * sj5);
                                                IkReal x904 = ((0.0881580172858829) * sj0 * sj1);
                                                IkReal x905 = ((0.443319482978384) * sj0 * sj1);
                                                IkReal x906 = ((0.443319482978384) * cj1 * x897);
                                                IkReal x907 = ((0.0881580172858829) * cj1 * x897);
                                                evalcond[0] = (((r21 * x901)) + (((-1.0) * r20 * x903)) + (((-1.0) * r20 * x900)) + (((0.452) * sj1 * x897)) + (((-1.0) * pz)) + ((x896 * x902)) + x902 + (((-1.0) * r21 * x898)));
                                                evalcond[1] = ((((0.103375806095808) * cj0)) + ((r01 * x901)) + (((-1.0) * sj0 * x906)) + (((0.0205573202558309) * sj0)) + (((-1.0) * r00 * x900)) + (((-1.0) * r00 * x903)) + (((-1.0) * px)) + (((-0.0881580172858829) * x899)) + ((x896 * x905)) + ((cj0 * x907)) + (((-0.0881580172858829) * x896 * x899)) + (((-1.0) * r01 * x898)) + x905);
                                                evalcond[2] = ((((-1.0) * x896 * x904)) + ((sj0 * x907)) + (((-0.443319482978384) * x899)) + ((r11 * x901)) + (((-1.0) * x904)) + (((-0.0205573202558309) * cj0)) + (((-1.0) * r11 * x898)) + (((0.103375806095808) * sj0)) + (((-0.443319482978384) * x896 * x899)) + (((-1.0) * py)) + ((cj0 * x906)) + (((-1.0) * r10 * x903)) + (((-1.0) * r10 * x900)));
                                                if (IKabs(evalcond[0]) > IKFAST_EVALCOND_THRESH || IKabs(evalcond[1]) > IKFAST_EVALCOND_THRESH || IKabs(evalcond[2]) > IKFAST_EVALCOND_THRESH)
                                                {
                                                    continue;
                                                }
                                            }

                                            {
                                                IkReal j3eval[2];
                                                IkReal x908 = cj4 * cj4;
                                                IkReal x909 = cj5 * cj5;
                                                IkReal x910 = r22 * r22;
                                                IkReal x911 = r21 * r21;
                                                IkReal x912 = r20 * r20;
                                                IkReal x913 = (r20 * sj5);
                                                IkReal x914 = (cj5 * r21);
                                                IkReal x915 = ((1.0) * x911);
                                                IkReal x916 = ((1.0) * x912);
                                                IkReal x917 = (x908 * x909);
                                                IkReal x918 = ((2.0) * cj4 * r22 * sj4);
                                                IkReal x919 = ((((-1.0) * x913 * x918)) + ((x909 * x911)) + (((-1.0) * x908 * x916)) + (((-1.0) * x915)) + (((2.0) * x913 * x914)) + (((-1.0) * x910)) + (((-1.0) * x914 * x918)) + ((x908 * x910)) + ((x912 * x917)) + (((-1.0) * x909 * x916)) + (((-2.0) * x908 * x913 * x914)) + (((-1.0) * x915 * x917)));
                                                j3eval[0] = x919;
                                                j3eval[1] = IKsign(x919);
                                                if (IKabs(j3eval[0]) < 0.0000010000000000 || IKabs(j3eval[1]) < 0.0000010000000000)
                                                {
                                                    {
                                                        IkReal j3eval[2];
                                                        IkReal x920 = (((cj5 * r11 * sj4)) + ((r10 * sj4 * sj5)) + (((-1.0) * cj4 * r12)));
                                                        j3eval[0] = x920;
                                                        j3eval[1] = IKsign(x920);
                                                        if (IKabs(j3eval[0]) < 0.0000010000000000 || IKabs(j3eval[1]) < 0.0000010000000000)
                                                        {
                                                            {
                                                                IkReal j3eval[2];
                                                                IkReal x921 = ((1.0) * sj4);
                                                                IkReal x922 = ((((-1.0) * r00 * sj5 * x921)) + (((-1.0) * cj5 * r01 * x921)) + ((cj4 * r02)));
                                                                j3eval[0] = x922;
                                                                j3eval[1] = IKsign(x922);
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
                                                                        IkReal x923 = ((0.195039861251953) * sj0);
                                                                        IkReal x924 = (cj1 * cj2);
                                                                        IkReal x925 = (cj5 * r20);
                                                                        IkReal x926 = (r22 * sj4);
                                                                        IkReal x927 = (cj1 * sj2);
                                                                        IkReal x928 = (cj4 * cj5);
                                                                        IkReal x929 = (r12 * sj4);
                                                                        IkReal x930 = ((0.980795316323859) * cj0);
                                                                        IkReal x931 = ((1.0) * sj5);
                                                                        IkReal x932 = (sj1 * sj2);
                                                                        IkReal x933 = (cj2 * sj1);
                                                                        IkReal x934 = (r21 * sj5);
                                                                        IkReal x935 = ((1.0) * cj5);
                                                                        IkReal x936 = (cj4 * r10);
                                                                        IkReal x937 = (r21 * x932);
                                                                        IkReal x938 = (cj4 * r20 * sj5);
                                                                        CheckValue<IkReal> x939 = IKPowWithIntegerCheck(IKsign(((((-1.0) * r01 * sj4 * x935)) + (((-1.0) * r00 * sj4 * x931)) + ((cj4 * r02)))), -1);
                                                                        if (!x939.valid)
                                                                        {
                                                                            continue;
                                                                        }
                                                                        CheckValue<IkReal> x940 = IKatan2WithCheck(IkReal((((x924 * x930 * x938)) + ((x923 * x928 * x937)) + ((x930 * x932 * x938)) + ((x926 * x930 * x932)) + ((x928 * x930 * x937)) + (((-1.0) * x931 * x933 * x936)) + ((x927 * x929)) + ((x924 * x926 * x930)) + ((x923 * x926 * x932)) + ((r21 * x923 * x924 * x928)) + ((x923 * x924 * x938)) + (((-1.0) * r11 * x928 * x933)) + ((x923 * x924 * x926)) + ((r11 * x927 * x928)) + ((r21 * x924 * x928 * x930)) + ((sj5 * x927 * x936)) + (((-1.0) * x929 * x933)) + ((x923 * x932 * x938)))), IkReal((((x924 * x930 * x934)) + (((-1.0) * x923 * x924 * x925)) + (((-1.0) * x923 * x925 * x932)) + ((x930 * x932 * x934)) + (((-1.0) * r10 * x927 * x935)) + (((-1.0) * x925 * x930 * x932)) + (((-1.0) * r11 * x931 * x933)) + ((r11 * sj5 * x927)) + ((x923 * x924 * x934)) + ((cj5 * r10 * x933)) + (((-1.0) * x924 * x925 * x930)) + ((x923 * x932 * x934)))), IKFAST_ATAN2_MAGTHRESH);
                                                                        if (!x940.valid)
                                                                        {
                                                                            continue;
                                                                        }
                                                                        j3array[0] = ((-1.5707963267949) + (((1.5707963267949) * (x939.value))) + (x940.value));
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
                                                                                IkReal x941 = IKcos(j3);
                                                                                IkReal x942 = IKsin(j3);
                                                                                IkReal x943 = ((0.980795316323859) * cj0);
                                                                                IkReal x944 = (sj1 * sj2);
                                                                                IkReal x945 = (cj1 * cj2);
                                                                                IkReal x946 = ((0.980795316323859) * sj0);
                                                                                IkReal x947 = (r00 * sj5);
                                                                                IkReal x948 = (cj5 * r11);
                                                                                IkReal x949 = (r10 * sj5);
                                                                                IkReal x950 = (r02 * sj4);
                                                                                IkReal x951 = (cj5 * r01);
                                                                                IkReal x952 = (r12 * sj4);
                                                                                IkReal x953 = (cj2 * sj1);
                                                                                IkReal x954 = ((0.195039861251953) * sj0);
                                                                                IkReal x955 = (r20 * sj5);
                                                                                IkReal x956 = (r22 * sj4);
                                                                                IkReal x957 = (r21 * sj5);
                                                                                IkReal x958 = (r01 * sj5);
                                                                                IkReal x959 = ((0.195039861251953) * cj0);
                                                                                IkReal x960 = (cj5 * r21);
                                                                                IkReal x961 = (cj1 * sj2);
                                                                                IkReal x962 = ((1.0) * r11 * sj5);
                                                                                IkReal x963 = (cj5 * x942);
                                                                                IkReal x964 = ((1.0) * x942);
                                                                                IkReal x965 = (cj4 * x941);
                                                                                IkReal x966 = (cj5 * x941);
                                                                                IkReal x967 = ((1.0) * x941);
                                                                                IkReal x968 = (cj4 * x964);
                                                                                evalcond[0] = ((((-1.0) * x953)) + ((x960 * x965)) + ((x941 * x956)) + (((-1.0) * x957 * x964)) + ((r20 * x963)) + x961 + ((x955 * x965)));
                                                                                evalcond[1] = ((((-1.0) * x960 * x968)) + (((-1.0) * x956 * x964)) + (((-1.0) * x957 * x967)) + ((r20 * x966)) + (((-1.0) * x955 * x968)) + x944 + x945);
                                                                                evalcond[2] = (((x951 * x965)) + ((x945 * x946)) + (((-1.0) * x944 * x959)) + ((x941 * x950)) + ((r00 * x963)) + (((-1.0) * x958 * x964)) + ((x947 * x965)) + ((x944 * x946)) + (((-1.0) * x945 * x959)));
                                                                                evalcond[3] = ((((-1.0) * x944 * x954)) + ((x949 * x965)) + ((x941 * x952)) + (((-1.0) * x942 * x962)) + ((x948 * x965)) + (((-1.0) * x943 * x944)) + (((-1.0) * x943 * x945)) + (((-1.0) * x945 * x954)) + ((r10 * x963)));
                                                                                evalcond[4] = (((r00 * x966)) + (((-1.0) * x958 * x967)) + (((-1.0) * x946 * x961)) + (((-1.0) * x950 * x964)) + (((-1.0) * x953 * x959)) + (((-1.0) * x947 * x968)) + ((x959 * x961)) + ((x946 * x953)) + (((-1.0) * x951 * x968)));
                                                                                evalcond[5] = (((x954 * x961)) + (((-1.0) * x943 * x953)) + ((x943 * x961)) + (((-1.0) * x941 * x962)) + (((-1.0) * x953 * x954)) + (((-1.0) * x952 * x964)) + (((-1.0) * x949 * x968)) + ((r10 * x966)) + (((-1.0) * x948 * x968)));
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
                                                                IkReal x969 = (cj1 * sj2);
                                                                IkReal x970 = (cj4 * cj5);
                                                                IkReal x971 = (sj1 * sj5);
                                                                IkReal x972 = ((1.0) * r01);
                                                                IkReal x973 = ((0.980795316323859) * sj0);
                                                                IkReal x974 = (r22 * sj4);
                                                                IkReal x975 = (r02 * sj4);
                                                                IkReal x976 = (cj1 * cj2);
                                                                IkReal x977 = (cj2 * r21);
                                                                IkReal x978 = (sj1 * sj2);
                                                                IkReal x979 = ((0.195039861251953) * cj0);
                                                                IkReal x980 = (cj5 * r20);
                                                                IkReal x981 = (cj4 * r00);
                                                                IkReal x982 = (cj2 * sj1);
                                                                IkReal x983 = (cj5 * r00);
                                                                IkReal x984 = (r21 * x979);
                                                                IkReal x985 = (cj2 * cj4 * r20);
                                                                IkReal x986 = (cj4 * r20 * sj5);
                                                                CheckValue<IkReal> x987 = IKatan2WithCheck(IkReal((((x979 * x980 * x982)) + ((sj5 * x969 * x984)) + (((-1.0) * x969 * x979 * x980)) + (((-1.0) * x971 * x977 * x979)) + (((-1.0) * r21 * sj5 * x969 * x973)) + ((x976 * x983)) + ((x971 * x973 * x977)) + (((-1.0) * x973 * x980 * x982)) + (((-1.0) * sj2 * x971 * x972)) + (((-1.0) * sj5 * x972 * x976)) + ((x969 * x973 * x980)) + ((x978 * x983)))), IkReal((((x969 * x973 * x974)) + ((x975 * x978)) + ((x975 * x976)) + (((-1.0) * x969 * x970 * x984)) + (((-1.0) * x969 * x979 * x986)) + ((x971 * x979 * x985)) + ((sj5 * x976 * x981)) + (((-1.0) * x973 * x974 * x982)) + (((-1.0) * x971 * x973 * x985)) + ((r21 * x969 * x970 * x973)) + ((r01 * x970 * x976)) + ((r01 * x970 * x978)) + ((x974 * x979 * x982)) + ((sj2 * x971 * x981)) + (((-1.0) * sj1 * x970 * x973 * x977)) + (((-1.0) * x969 * x974 * x979)) + ((x969 * x973 * x986)) + ((sj1 * x970 * x977 * x979)))), IKFAST_ATAN2_MAGTHRESH);
                                                                if (!x987.valid)
                                                                {
                                                                    continue;
                                                                }
                                                                CheckValue<IkReal> x988 = IKPowWithIntegerCheck(IKsign((((cj5 * r11 * sj4)) + ((r10 * sj4 * sj5)) + (((-1.0) * cj4 * r12)))), -1);
                                                                if (!x988.valid)
                                                                {
                                                                    continue;
                                                                }
                                                                j3array[0] = ((-1.5707963267949) + (x987.value) + (((1.5707963267949) * (x988.value))));
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
                                                                        IkReal x989 = IKcos(j3);
                                                                        IkReal x990 = IKsin(j3);
                                                                        IkReal x991 = ((0.980795316323859) * cj0);
                                                                        IkReal x992 = (sj1 * sj2);
                                                                        IkReal x993 = (cj1 * cj2);
                                                                        IkReal x994 = ((0.980795316323859) * sj0);
                                                                        IkReal x995 = (r00 * sj5);
                                                                        IkReal x996 = (cj5 * r11);
                                                                        IkReal x997 = (r10 * sj5);
                                                                        IkReal x998 = (r02 * sj4);
                                                                        IkReal x999 = (cj5 * r01);
                                                                        IkReal x1000 = (r12 * sj4);
                                                                        IkReal x1001 = (cj2 * sj1);
                                                                        IkReal x1002 = ((0.195039861251953) * sj0);
                                                                        IkReal x1003 = (r20 * sj5);
                                                                        IkReal x1004 = (r22 * sj4);
                                                                        IkReal x1005 = (r21 * sj5);
                                                                        IkReal x1006 = (r01 * sj5);
                                                                        IkReal x1007 = ((0.195039861251953) * cj0);
                                                                        IkReal x1008 = (cj5 * r21);
                                                                        IkReal x1009 = (cj1 * sj2);
                                                                        IkReal x1010 = ((1.0) * r11 * sj5);
                                                                        IkReal x1011 = (cj5 * x990);
                                                                        IkReal x1012 = ((1.0) * x990);
                                                                        IkReal x1013 = (cj4 * x989);
                                                                        IkReal x1014 = (cj5 * x989);
                                                                        IkReal x1015 = ((1.0) * x989);
                                                                        IkReal x1016 = (cj4 * x1012);
                                                                        evalcond[0] = ((((-1.0) * x1001)) + x1009 + ((x1004 * x989)) + ((x1003 * x1013)) + (((-1.0) * x1005 * x1012)) + ((r20 * x1011)) + ((x1008 * x1013)));
                                                                        evalcond[1] = ((((-1.0) * x1005 * x1015)) + (((-1.0) * x1004 * x1012)) + (((-1.0) * x1008 * x1016)) + (((-1.0) * x1003 * x1016)) + ((r20 * x1014)) + x993 + x992);
                                                                        evalcond[2] = (((x993 * x994)) + ((x989 * x998)) + ((x992 * x994)) + (((-1.0) * x1006 * x1012)) + (((-1.0) * x1007 * x992)) + (((-1.0) * x1007 * x993)) + ((x1013 * x995)) + ((x1013 * x999)) + ((r00 * x1011)));
                                                                        evalcond[3] = ((((-1.0) * x1002 * x993)) + (((-1.0) * x1002 * x992)) + (((-1.0) * x991 * x993)) + (((-1.0) * x991 * x992)) + (((-1.0) * x1010 * x990)) + ((x1013 * x996)) + ((x1013 * x997)) + ((r10 * x1011)) + ((x1000 * x989)));
                                                                        evalcond[4] = ((((-1.0) * x1016 * x995)) + (((-1.0) * x1016 * x999)) + (((-1.0) * x1009 * x994)) + (((-1.0) * x1012 * x998)) + (((-1.0) * x1006 * x1015)) + ((x1001 * x994)) + ((x1007 * x1009)) + (((-1.0) * x1001 * x1007)) + ((r00 * x1014)));
                                                                        evalcond[5] = ((((-1.0) * x1016 * x997)) + (((-1.0) * x1016 * x996)) + (((-1.0) * x1010 * x989)) + ((x1009 * x991)) + (((-1.0) * x1001 * x991)) + (((-1.0) * x1001 * x1002)) + ((x1002 * x1009)) + ((r10 * x1014)) + (((-1.0) * x1000 * x1012)));
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
                                                        IkReal x1017 = cj4 * cj4;
                                                        IkReal x1018 = cj5 * cj5;
                                                        IkReal x1019 = r22 * r22;
                                                        IkReal x1020 = r21 * r21;
                                                        IkReal x1021 = r20 * r20;
                                                        IkReal x1022 = ((1.0) * sj1);
                                                        IkReal x1023 = (r22 * sj4);
                                                        IkReal x1024 = (sj2 * sj5);
                                                        IkReal x1025 = (cj4 * r20);
                                                        IkReal x1026 = (cj5 * sj2);
                                                        IkReal x1027 = (r21 * sj5);
                                                        IkReal x1028 = (cj4 * r21);
                                                        IkReal x1029 = ((2.0) * cj5);
                                                        IkReal x1030 = (cj2 * cj5 * r20);
                                                        IkReal x1031 = ((1.0) * x1020);
                                                        IkReal x1032 = ((1.0) * cj1 * cj2);
                                                        IkReal x1033 = ((1.0) * x1021);
                                                        IkReal x1034 = (x1017 * x1018);
                                                        CheckValue<IkReal> x1035 = IKatan2WithCheck(IkReal(((((-1.0) * x1022 * x1030)) + (((-1.0) * x1022 * x1026 * x1028)) + (((-1.0) * x1023 * x1032)) + (((-1.0) * cj5 * x1028 * x1032)) + ((cj2 * sj1 * x1027)) + (((-1.0) * sj5 * x1025 * x1032)) + (((-1.0) * sj2 * x1022 * x1023)) + (((-1.0) * x1022 * x1024 * x1025)) + (((-1.0) * cj1 * r21 * x1024)) + ((cj1 * r20 * x1026)))), IkReal(((((-1.0) * x1027 * x1032)) + ((r20 * sj1 * x1026)) + ((cj1 * x1024 * x1025)) + (((-1.0) * r21 * x1022 * x1024)) + ((cj1 * sj2 * x1023)) + (((-1.0) * cj2 * sj5 * x1022 * x1025)) + (((-1.0) * cj2 * x1022 * x1023)) + (((-1.0) * cj2 * cj5 * x1022 * x1028)) + ((cj1 * x1026 * x1028)) + ((cj1 * x1030)))), IKFAST_ATAN2_MAGTHRESH);
                                                        if (!x1035.valid)
                                                        {
                                                            continue;
                                                        }
                                                        CheckValue<IkReal> x1036 = IKPowWithIntegerCheck(IKsign((((x1021 * x1034)) + (((-1.0) * x1031)) + ((x1017 * x1019)) + (((-1.0) * x1017 * x1033)) + (((-1.0) * x1023 * x1028 * x1029)) + (((-2.0) * sj5 * x1023 * x1025)) + ((x1018 * x1020)) + ((r20 * x1027 * x1029)) + (((-1.0) * x1031 * x1034)) + (((-1.0) * x1018 * x1033)) + (((-1.0) * r20 * x1017 * x1027 * x1029)) + (((-1.0) * x1019)))), -1);
                                                        if (!x1036.valid)
                                                        {
                                                            continue;
                                                        }
                                                        j3array[0] = ((-1.5707963267949) + (x1035.value) + (((1.5707963267949) * (x1036.value))));
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
                                                                IkReal x1037 = IKcos(j3);
                                                                IkReal x1038 = IKsin(j3);
                                                                IkReal x1039 = ((0.980795316323859) * cj0);
                                                                IkReal x1040 = (sj1 * sj2);
                                                                IkReal x1041 = (cj1 * cj2);
                                                                IkReal x1042 = ((0.980795316323859) * sj0);
                                                                IkReal x1043 = (r00 * sj5);
                                                                IkReal x1044 = (cj5 * r11);
                                                                IkReal x1045 = (r10 * sj5);
                                                                IkReal x1046 = (r02 * sj4);
                                                                IkReal x1047 = (cj5 * r01);
                                                                IkReal x1048 = (r12 * sj4);
                                                                IkReal x1049 = (cj2 * sj1);
                                                                IkReal x1050 = ((0.195039861251953) * sj0);
                                                                IkReal x1051 = (r20 * sj5);
                                                                IkReal x1052 = (r22 * sj4);
                                                                IkReal x1053 = (r21 * sj5);
                                                                IkReal x1054 = (r01 * sj5);
                                                                IkReal x1055 = ((0.195039861251953) * cj0);
                                                                IkReal x1056 = (cj5 * r21);
                                                                IkReal x1057 = (cj1 * sj2);
                                                                IkReal x1058 = ((1.0) * r11 * sj5);
                                                                IkReal x1059 = (cj5 * x1038);
                                                                IkReal x1060 = ((1.0) * x1038);
                                                                IkReal x1061 = (cj4 * x1037);
                                                                IkReal x1062 = (cj5 * x1037);
                                                                IkReal x1063 = ((1.0) * x1037);
                                                                IkReal x1064 = (cj4 * x1060);
                                                                evalcond[0] = (((x1056 * x1061)) + ((x1037 * x1052)) + x1057 + ((x1051 * x1061)) + ((r20 * x1059)) + (((-1.0) * x1049)) + (((-1.0) * x1053 * x1060)));
                                                                evalcond[1] = (x1041 + x1040 + ((r20 * x1062)) + (((-1.0) * x1056 * x1064)) + (((-1.0) * x1052 * x1060)) + (((-1.0) * x1051 * x1064)) + (((-1.0) * x1053 * x1063)));
                                                                evalcond[2] = (((x1043 * x1061)) + ((x1037 * x1046)) + ((x1040 * x1042)) + ((x1041 * x1042)) + (((-1.0) * x1041 * x1055)) + ((r00 * x1059)) + (((-1.0) * x1040 * x1055)) + (((-1.0) * x1054 * x1060)) + ((x1047 * x1061)));
                                                                evalcond[3] = (((x1044 * x1061)) + ((x1037 * x1048)) + (((-1.0) * x1038 * x1058)) + (((-1.0) * x1039 * x1040)) + (((-1.0) * x1039 * x1041)) + (((-1.0) * x1041 * x1050)) + ((r10 * x1059)) + (((-1.0) * x1040 * x1050)) + ((x1045 * x1061)));
                                                                evalcond[4] = ((((-1.0) * x1047 * x1064)) + ((x1042 * x1049)) + ((x1055 * x1057)) + (((-1.0) * x1043 * x1064)) + (((-1.0) * x1042 * x1057)) + (((-1.0) * x1054 * x1063)) + ((r00 * x1062)) + (((-1.0) * x1049 * x1055)) + (((-1.0) * x1046 * x1060)));
                                                                evalcond[5] = (((x1039 * x1057)) + (((-1.0) * x1044 * x1064)) + (((-1.0) * x1039 * x1049)) + ((x1050 * x1057)) + (((-1.0) * x1045 * x1064)) + (((-1.0) * x1048 * x1060)) + (((-1.0) * x1037 * x1058)) + ((r10 * x1062)) + (((-1.0) * x1049 * x1050)));
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
                                IkReal x1065 = ((50.0951015765574) * sj0);
                                IkReal x1066 = ((9.96185595330477) * cj0);
                                IkReal x1067 = ((8.81580172858829e-7) * r20);
                                IkReal x1068 = ((2.50148374048693) * r21);
                                IkReal x1069 = (cj5 * r21);
                                IkReal x1070 = (cj1 * sj5);
                                IkReal x1071 = ((12.8255) * r01);
                                IkReal x1072 = (r20 * sj5);
                                IkReal x1073 = ((2.32297718890889) * sj0);
                                IkReal x1074 = ((4.52e-6) * r00);
                                IkReal x1075 = (sj1 * sj5);
                                IkReal x1076 = (cj0 * cj1);
                                IkReal x1077 = (cj5 * r20);
                                IkReal x1078 = (cj1 * sj0);
                                IkReal x1079 = ((110.829870744596) * pz);
                                IkReal x1080 = ((12.5791903295117) * sj0);
                                IkReal x1081 = (cj0 * sj1);
                                IkReal x1082 = ((113.0) * px);
                                IkReal x1083 = (cj0 * x1075);
                                IkReal x1084 = ((22.0395043214707) * cj0 * pz);
                                IkReal x1085 = ((4.43319482978384e-6) * x1078);
                                IkReal x1086 = ((12.8255) * cj5 * r00);
                                IkReal x1087 = ((4.43319482978384e-6) * sj0 * sj1);
                                IkReal x1088 = ((4.52e-6) * cj5 * r01);
                                CheckValue<IkReal> x1089 = IKatan2WithCheck(IkReal(((((-1.0) * sj1 * x1077 * x1080)) + (((2.50148374048693) * x1077 * x1081)) + (((-1.0) * sj0 * sj1 * x1079)) + (((-11.6814660888263) * x1076)) + ((r21 * x1075 * x1080)) + ((x1070 * x1074)) + (((-1.0) * x1070 * x1071)) + ((cj1 * x1086)) + ((cj1 * x1082)) + ((cj1 * x1088)) + (((-1.0) * cj1 * x1073)) + (((-1.0) * x1068 * x1083)) + ((x1067 * x1083)) + (((22.0395043214707) * pz * x1081)) + (((-1.0) * x1069 * x1087)) + (((8.81580172858829e-7) * x1069 * x1081)) + (((-1.0) * x1072 * x1087)))), IkReal((((r21 * x1070 * x1080)) + (((-1.0) * sj1 * x1082)) + (((-1.0) * sj1 * x1086)) + (((-1.0) * sj1 * x1088)) + (((-1.0) * x1074 * x1075)) + x1065 + (((-1.0) * x1078 * x1079)) + (((8.81580172858829e-7) * x1069 * x1076)) + ((x1071 * x1075)) + (((2.50148374048693) * x1076 * x1077)) + ((cj0 * x1067 * x1070)) + (((11.6814660888263) * x1081)) + (((-1.0) * x1069 * x1085)) + (((-12.5791903295117) * x1077 * x1078)) + (((22.0395043214707) * pz * x1076)) + ((sj1 * x1073)) + (((-4.43319482978384e-6) * r20 * sj0 * x1070)) + (((-1.0) * cj0 * x1068 * x1070)) + (((-1.0) * x1066)))), IKFAST_ATAN2_MAGTHRESH);
                                if (!x1089.valid)
                                {
                                    continue;
                                }
                                CheckValue<IkReal> x1090 = IKPowWithIntegerCheck(IKsign((x1066 + (((-1.0) * x1065)))), -1);
                                if (!x1090.valid)
                                {
                                    continue;
                                }
                                j2array[0] = ((-1.5707963267949) + (x1089.value) + (((1.5707963267949) * (x1090.value))));
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
                                        IkReal x1091 = IKcos(j2);
                                        IkReal x1092 = IKsin(j2);
                                        IkReal x1093 = ((4.0e-8) * cj5);
                                        IkReal x1094 = (cj0 * sj1);
                                        IkReal x1095 = ((0.1135) * cj5);
                                        IkReal x1096 = ((0.1135) * sj5);
                                        IkReal x1097 = ((0.452) * cj1);
                                        IkReal x1098 = ((4.0e-8) * sj5);
                                        IkReal x1099 = ((0.0881580172858829) * sj0 * sj1);
                                        IkReal x1100 = ((0.443319482978384) * sj0 * sj1);
                                        IkReal x1101 = ((0.443319482978384) * cj1 * x1092);
                                        IkReal x1102 = ((0.0881580172858829) * cj1 * x1092);
                                        evalcond[0] = (x1097 + ((r21 * x1096)) + (((-1.0) * pz)) + (((-1.0) * r21 * x1093)) + ((x1091 * x1097)) + (((-1.0) * r20 * x1098)) + (((-1.0) * r20 * x1095)) + (((0.452) * sj1 * x1092)));
                                        evalcond[1] = (x1100 + (((-0.0881580172858829) * x1094)) + (((0.103375806095808) * cj0)) + ((r01 * x1096)) + (((0.0205573202558309) * sj0)) + ((x1091 * x1100)) + (((-1.0) * px)) + (((-0.0881580172858829) * x1091 * x1094)) + (((-1.0) * r01 * x1093)) + ((cj0 * x1102)) + (((-1.0) * sj0 * x1101)) + (((-1.0) * r00 * x1098)) + (((-1.0) * r00 * x1095)));
                                        evalcond[2] = ((((-0.443319482978384) * x1091 * x1094)) + (((-1.0) * x1091 * x1099)) + (((-1.0) * x1099)) + (((-0.0205573202558309) * cj0)) + (((0.103375806095808) * sj0)) + (((-1.0) * py)) + (((-1.0) * r11 * x1093)) + ((r11 * x1096)) + (((-1.0) * r10 * x1098)) + (((-1.0) * r10 * x1095)) + ((sj0 * x1102)) + (((-0.443319482978384) * x1094)) + ((cj0 * x1101)));
                                        if (IKabs(evalcond[0]) > IKFAST_EVALCOND_THRESH || IKabs(evalcond[1]) > IKFAST_EVALCOND_THRESH || IKabs(evalcond[2]) > IKFAST_EVALCOND_THRESH)
                                        {
                                            continue;
                                        }
                                    }

                                    {
                                        IkReal j3eval[2];
                                        IkReal x1103 = cj4 * cj4;
                                        IkReal x1104 = cj5 * cj5;
                                        IkReal x1105 = r22 * r22;
                                        IkReal x1106 = r21 * r21;
                                        IkReal x1107 = r20 * r20;
                                        IkReal x1108 = (r20 * sj5);
                                        IkReal x1109 = (cj5 * r21);
                                        IkReal x1110 = ((1.0) * x1106);
                                        IkReal x1111 = ((1.0) * x1107);
                                        IkReal x1112 = (x1103 * x1104);
                                        IkReal x1113 = ((2.0) * cj4 * r22 * sj4);
                                        IkReal x1114 = ((((-1.0) * x1103 * x1111)) + (((-1.0) * x1109 * x1113)) + (((2.0) * x1108 * x1109)) + (((-1.0) * x1105)) + ((x1104 * x1106)) + (((-1.0) * x1108 * x1113)) + (((-2.0) * x1103 * x1108 * x1109)) + (((-1.0) * x1104 * x1111)) + (((-1.0) * x1110 * x1112)) + ((x1103 * x1105)) + (((-1.0) * x1110)) + ((x1107 * x1112)));
                                        j3eval[0] = x1114;
                                        j3eval[1] = IKsign(x1114);
                                        if (IKabs(j3eval[0]) < 0.0000010000000000 || IKabs(j3eval[1]) < 0.0000010000000000)
                                        {
                                            {
                                                IkReal j3eval[2];
                                                IkReal x1115 = (((cj5 * r11 * sj4)) + ((r10 * sj4 * sj5)) + (((-1.0) * cj4 * r12)));
                                                j3eval[0] = x1115;
                                                j3eval[1] = IKsign(x1115);
                                                if (IKabs(j3eval[0]) < 0.0000010000000000 || IKabs(j3eval[1]) < 0.0000010000000000)
                                                {
                                                    {
                                                        IkReal j3eval[2];
                                                        IkReal x1116 = ((1.0) * sj4);
                                                        IkReal x1117 = ((((-1.0) * r00 * sj5 * x1116)) + (((-1.0) * cj5 * r01 * x1116)) + ((cj4 * r02)));
                                                        j3eval[0] = x1117;
                                                        j3eval[1] = IKsign(x1117);
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
                                                                IkReal x1118 = ((0.195039861251953) * sj0);
                                                                IkReal x1119 = (cj1 * cj2);
                                                                IkReal x1120 = (cj5 * r20);
                                                                IkReal x1121 = (r22 * sj4);
                                                                IkReal x1122 = (cj1 * sj2);
                                                                IkReal x1123 = (cj4 * cj5);
                                                                IkReal x1124 = (r12 * sj4);
                                                                IkReal x1125 = ((0.980795316323859) * cj0);
                                                                IkReal x1126 = ((1.0) * sj5);
                                                                IkReal x1127 = (sj1 * sj2);
                                                                IkReal x1128 = (cj2 * sj1);
                                                                IkReal x1129 = (r21 * sj5);
                                                                IkReal x1130 = ((1.0) * cj5);
                                                                IkReal x1131 = (cj4 * r10);
                                                                IkReal x1132 = (r21 * x1127);
                                                                IkReal x1133 = (cj4 * r20 * sj5);
                                                                CheckValue<IkReal> x1134 = IKPowWithIntegerCheck(IKsign(((((-1.0) * r01 * sj4 * x1130)) + (((-1.0) * r00 * sj4 * x1126)) + ((cj4 * r02)))), -1);
                                                                if (!x1134.valid)
                                                                {
                                                                    continue;
                                                                }
                                                                CheckValue<IkReal> x1135 = IKatan2WithCheck(IkReal((((r21 * x1118 * x1119 * x1123)) + ((x1118 * x1127 * x1133)) + (((-1.0) * x1124 * x1128)) + ((r11 * x1122 * x1123)) + ((x1119 * x1121 * x1125)) + (((-1.0) * x1126 * x1128 * x1131)) + ((x1119 * x1125 * x1133)) + ((x1125 * x1127 * x1133)) + ((x1118 * x1121 * x1127)) + ((x1118 * x1119 * x1121)) + ((r21 * x1119 * x1123 * x1125)) + ((x1118 * x1119 * x1133)) + (((-1.0) * r11 * x1123 * x1128)) + ((x1122 * x1124)) + ((x1121 * x1125 * x1127)) + ((sj5 * x1122 * x1131)) + ((x1118 * x1123 * x1132)) + ((x1123 * x1125 * x1132)))), IkReal((((x1118 * x1127 * x1129)) + ((cj5 * r10 * x1128)) + ((r11 * sj5 * x1122)) + (((-1.0) * x1118 * x1119 * x1120)) + ((x1119 * x1125 * x1129)) + ((x1125 * x1127 * x1129)) + (((-1.0) * r11 * x1126 * x1128)) + ((x1118 * x1119 * x1129)) + (((-1.0) * x1118 * x1120 * x1127)) + (((-1.0) * r10 * x1122 * x1130)) + (((-1.0) * x1119 * x1120 * x1125)) + (((-1.0) * x1120 * x1125 * x1127)))), IKFAST_ATAN2_MAGTHRESH);
                                                                if (!x1135.valid)
                                                                {
                                                                    continue;
                                                                }
                                                                j3array[0] = ((-1.5707963267949) + (((1.5707963267949) * (x1134.value))) + (x1135.value));
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
                                                                        IkReal x1136 = IKcos(j3);
                                                                        IkReal x1137 = IKsin(j3);
                                                                        IkReal x1138 = ((0.980795316323859) * cj0);
                                                                        IkReal x1139 = (sj1 * sj2);
                                                                        IkReal x1140 = (cj1 * cj2);
                                                                        IkReal x1141 = ((0.980795316323859) * sj0);
                                                                        IkReal x1142 = (r00 * sj5);
                                                                        IkReal x1143 = (cj5 * r11);
                                                                        IkReal x1144 = (r10 * sj5);
                                                                        IkReal x1145 = (r02 * sj4);
                                                                        IkReal x1146 = (cj5 * r01);
                                                                        IkReal x1147 = (r12 * sj4);
                                                                        IkReal x1148 = (cj2 * sj1);
                                                                        IkReal x1149 = ((0.195039861251953) * sj0);
                                                                        IkReal x1150 = (r20 * sj5);
                                                                        IkReal x1151 = (r22 * sj4);
                                                                        IkReal x1152 = (r21 * sj5);
                                                                        IkReal x1153 = (r01 * sj5);
                                                                        IkReal x1154 = ((0.195039861251953) * cj0);
                                                                        IkReal x1155 = (cj5 * r21);
                                                                        IkReal x1156 = (cj1 * sj2);
                                                                        IkReal x1157 = ((1.0) * r11 * sj5);
                                                                        IkReal x1158 = (cj5 * x1137);
                                                                        IkReal x1159 = ((1.0) * x1137);
                                                                        IkReal x1160 = (cj4 * x1136);
                                                                        IkReal x1161 = (cj5 * x1136);
                                                                        IkReal x1162 = ((1.0) * x1136);
                                                                        IkReal x1163 = (cj4 * x1159);
                                                                        evalcond[0] = (x1156 + ((x1136 * x1151)) + ((x1155 * x1160)) + (((-1.0) * x1148)) + ((r20 * x1158)) + (((-1.0) * x1152 * x1159)) + ((x1150 * x1160)));
                                                                        evalcond[1] = (x1140 + x1139 + ((r20 * x1161)) + (((-1.0) * x1152 * x1162)) + (((-1.0) * x1150 * x1163)) + (((-1.0) * x1155 * x1163)) + (((-1.0) * x1151 * x1159)));
                                                                        evalcond[2] = (((r00 * x1158)) + ((x1140 * x1141)) + (((-1.0) * x1140 * x1154)) + ((x1142 * x1160)) + (((-1.0) * x1139 * x1154)) + ((x1139 * x1141)) + ((x1146 * x1160)) + (((-1.0) * x1153 * x1159)) + ((x1136 * x1145)));
                                                                        evalcond[3] = ((((-1.0) * x1140 * x1149)) + ((x1144 * x1160)) + ((x1143 * x1160)) + (((-1.0) * x1138 * x1139)) + (((-1.0) * x1139 * x1149)) + (((-1.0) * x1137 * x1157)) + ((x1136 * x1147)) + (((-1.0) * x1138 * x1140)) + ((r10 * x1158)));
                                                                        evalcond[4] = ((((-1.0) * x1141 * x1156)) + ((r00 * x1161)) + ((x1141 * x1148)) + (((-1.0) * x1153 * x1162)) + (((-1.0) * x1142 * x1163)) + (((-1.0) * x1148 * x1154)) + (((-1.0) * x1145 * x1159)) + ((x1154 * x1156)) + (((-1.0) * x1146 * x1163)));
                                                                        evalcond[5] = (((x1149 * x1156)) + (((-1.0) * x1144 * x1163)) + ((r10 * x1161)) + (((-1.0) * x1147 * x1159)) + (((-1.0) * x1143 * x1163)) + (((-1.0) * x1148 * x1149)) + ((x1138 * x1156)) + (((-1.0) * x1136 * x1157)) + (((-1.0) * x1138 * x1148)));
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
                                                        IkReal x1164 = (cj1 * sj2);
                                                        IkReal x1165 = (cj4 * cj5);
                                                        IkReal x1166 = (sj1 * sj5);
                                                        IkReal x1167 = ((1.0) * r01);
                                                        IkReal x1168 = ((0.980795316323859) * sj0);
                                                        IkReal x1169 = (r22 * sj4);
                                                        IkReal x1170 = (r02 * sj4);
                                                        IkReal x1171 = (cj1 * cj2);
                                                        IkReal x1172 = (cj2 * r21);
                                                        IkReal x1173 = (sj1 * sj2);
                                                        IkReal x1174 = ((0.195039861251953) * cj0);
                                                        IkReal x1175 = (cj5 * r20);
                                                        IkReal x1176 = (cj4 * r00);
                                                        IkReal x1177 = (cj2 * sj1);
                                                        IkReal x1178 = (cj5 * r00);
                                                        IkReal x1179 = (r21 * x1174);
                                                        IkReal x1180 = (cj2 * cj4 * r20);
                                                        IkReal x1181 = (cj4 * r20 * sj5);
                                                        CheckValue<IkReal> x1182 = IKatan2WithCheck(IkReal(((((-1.0) * r21 * sj5 * x1164 * x1168)) + (((-1.0) * x1164 * x1174 * x1175)) + ((x1174 * x1175 * x1177)) + ((sj5 * x1164 * x1179)) + ((x1173 * x1178)) + ((x1166 * x1168 * x1172)) + (((-1.0) * x1166 * x1172 * x1174)) + (((-1.0) * sj2 * x1166 * x1167)) + (((-1.0) * sj5 * x1167 * x1171)) + ((x1164 * x1168 * x1175)) + (((-1.0) * x1168 * x1175 * x1177)) + ((x1171 * x1178)))), IkReal(((((-1.0) * x1166 * x1168 * x1180)) + (((-1.0) * x1164 * x1169 * x1174)) + ((x1169 * x1174 * x1177)) + ((sj5 * x1171 * x1176)) + (((-1.0) * x1164 * x1165 * x1179)) + (((-1.0) * sj1 * x1165 * x1168 * x1172)) + (((-1.0) * x1164 * x1174 * x1181)) + (((-1.0) * x1168 * x1169 * x1177)) + ((x1164 * x1168 * x1169)) + ((sj1 * x1165 * x1172 * x1174)) + ((x1164 * x1168 * x1181)) + ((sj2 * x1166 * x1176)) + ((r01 * x1165 * x1171)) + ((r01 * x1165 * x1173)) + ((x1170 * x1173)) + ((x1170 * x1171)) + ((r21 * x1164 * x1165 * x1168)) + ((x1166 * x1174 * x1180)))), IKFAST_ATAN2_MAGTHRESH);
                                                        if (!x1182.valid)
                                                        {
                                                            continue;
                                                        }
                                                        CheckValue<IkReal> x1183 = IKPowWithIntegerCheck(IKsign((((cj5 * r11 * sj4)) + ((r10 * sj4 * sj5)) + (((-1.0) * cj4 * r12)))), -1);
                                                        if (!x1183.valid)
                                                        {
                                                            continue;
                                                        }
                                                        j3array[0] = ((-1.5707963267949) + (x1182.value) + (((1.5707963267949) * (x1183.value))));
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
                                                                IkReal x1184 = IKcos(j3);
                                                                IkReal x1185 = IKsin(j3);
                                                                IkReal x1186 = ((0.980795316323859) * cj0);
                                                                IkReal x1187 = (sj1 * sj2);
                                                                IkReal x1188 = (cj1 * cj2);
                                                                IkReal x1189 = ((0.980795316323859) * sj0);
                                                                IkReal x1190 = (r00 * sj5);
                                                                IkReal x1191 = (cj5 * r11);
                                                                IkReal x1192 = (r10 * sj5);
                                                                IkReal x1193 = (r02 * sj4);
                                                                IkReal x1194 = (cj5 * r01);
                                                                IkReal x1195 = (r12 * sj4);
                                                                IkReal x1196 = (cj2 * sj1);
                                                                IkReal x1197 = ((0.195039861251953) * sj0);
                                                                IkReal x1198 = (r20 * sj5);
                                                                IkReal x1199 = (r22 * sj4);
                                                                IkReal x1200 = (r21 * sj5);
                                                                IkReal x1201 = (r01 * sj5);
                                                                IkReal x1202 = ((0.195039861251953) * cj0);
                                                                IkReal x1203 = (cj5 * r21);
                                                                IkReal x1204 = (cj1 * sj2);
                                                                IkReal x1205 = ((1.0) * r11 * sj5);
                                                                IkReal x1206 = (cj5 * x1185);
                                                                IkReal x1207 = ((1.0) * x1185);
                                                                IkReal x1208 = (cj4 * x1184);
                                                                IkReal x1209 = (cj5 * x1184);
                                                                IkReal x1210 = ((1.0) * x1184);
                                                                IkReal x1211 = (cj4 * x1207);
                                                                evalcond[0] = (((x1184 * x1199)) + x1204 + ((x1203 * x1208)) + (((-1.0) * x1200 * x1207)) + ((r20 * x1206)) + (((-1.0) * x1196)) + ((x1198 * x1208)));
                                                                evalcond[1] = ((((-1.0) * x1199 * x1207)) + x1188 + x1187 + (((-1.0) * x1200 * x1210)) + (((-1.0) * x1203 * x1211)) + (((-1.0) * x1198 * x1211)) + ((r20 * x1209)));
                                                                evalcond[2] = (((x1187 * x1189)) + ((x1184 * x1193)) + ((x1188 * x1189)) + ((x1190 * x1208)) + (((-1.0) * x1188 * x1202)) + ((x1194 * x1208)) + (((-1.0) * x1187 * x1202)) + (((-1.0) * x1201 * x1207)) + ((r00 * x1206)));
                                                                evalcond[3] = (((x1184 * x1195)) + ((r10 * x1206)) + ((x1192 * x1208)) + (((-1.0) * x1188 * x1197)) + (((-1.0) * x1186 * x1188)) + (((-1.0) * x1186 * x1187)) + ((x1191 * x1208)) + (((-1.0) * x1187 * x1197)) + (((-1.0) * x1185 * x1205)));
                                                                evalcond[4] = ((((-1.0) * x1196 * x1202)) + (((-1.0) * x1201 * x1210)) + (((-1.0) * x1189 * x1204)) + ((x1189 * x1196)) + ((x1202 * x1204)) + ((r00 * x1209)) + (((-1.0) * x1190 * x1211)) + (((-1.0) * x1194 * x1211)) + (((-1.0) * x1193 * x1207)));
                                                                evalcond[5] = ((((-1.0) * x1192 * x1211)) + ((x1186 * x1204)) + ((r10 * x1209)) + (((-1.0) * x1186 * x1196)) + (((-1.0) * x1184 * x1205)) + ((x1197 * x1204)) + (((-1.0) * x1196 * x1197)) + (((-1.0) * x1191 * x1211)) + (((-1.0) * x1195 * x1207)));
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
                                                IkReal x1212 = cj4 * cj4;
                                                IkReal x1213 = cj5 * cj5;
                                                IkReal x1214 = r22 * r22;
                                                IkReal x1215 = r21 * r21;
                                                IkReal x1216 = r20 * r20;
                                                IkReal x1217 = ((1.0) * sj1);
                                                IkReal x1218 = (r22 * sj4);
                                                IkReal x1219 = (sj2 * sj5);
                                                IkReal x1220 = (cj4 * r20);
                                                IkReal x1221 = (cj5 * sj2);
                                                IkReal x1222 = (r21 * sj5);
                                                IkReal x1223 = (cj4 * r21);
                                                IkReal x1224 = ((2.0) * cj5);
                                                IkReal x1225 = (cj2 * cj5 * r20);
                                                IkReal x1226 = ((1.0) * x1215);
                                                IkReal x1227 = ((1.0) * cj1 * cj2);
                                                IkReal x1228 = ((1.0) * x1216);
                                                IkReal x1229 = (x1212 * x1213);
                                                CheckValue<IkReal> x1230 = IKatan2WithCheck(IkReal((((cj1 * r20 * x1221)) + (((-1.0) * x1218 * x1227)) + (((-1.0) * cj1 * r21 * x1219)) + (((-1.0) * cj5 * x1223 * x1227)) + ((cj2 * sj1 * x1222)) + (((-1.0) * x1217 * x1221 * x1223)) + (((-1.0) * x1217 * x1225)) + (((-1.0) * sj2 * x1217 * x1218)) + (((-1.0) * x1217 * x1219 * x1220)) + (((-1.0) * sj5 * x1220 * x1227)))), IkReal(((((-1.0) * r21 * x1217 * x1219)) + (((-1.0) * cj2 * sj5 * x1217 * x1220)) + ((r20 * sj1 * x1221)) + (((-1.0) * cj2 * x1217 * x1218)) + (((-1.0) * x1222 * x1227)) + (((-1.0) * cj2 * cj5 * x1217 * x1223)) + ((cj1 * x1221 * x1223)) + ((cj1 * x1219 * x1220)) + ((cj1 * sj2 * x1218)) + ((cj1 * x1225)))), IKFAST_ATAN2_MAGTHRESH);
                                                if (!x1230.valid)
                                                {
                                                    continue;
                                                }
                                                CheckValue<IkReal> x1231 = IKPowWithIntegerCheck(IKsign(((((-1.0) * r20 * x1212 * x1222 * x1224)) + (((-1.0) * x1213 * x1228)) + ((x1212 * x1214)) + (((-1.0) * x1212 * x1228)) + ((r20 * x1222 * x1224)) + ((x1213 * x1215)) + (((-1.0) * x1226)) + ((x1216 * x1229)) + (((-1.0) * x1226 * x1229)) + (((-1.0) * x1218 * x1223 * x1224)) + (((-2.0) * sj5 * x1218 * x1220)) + (((-1.0) * x1214)))), -1);
                                                if (!x1231.valid)
                                                {
                                                    continue;
                                                }
                                                j3array[0] = ((-1.5707963267949) + (x1230.value) + (((1.5707963267949) * (x1231.value))));
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
                                                        IkReal x1232 = IKcos(j3);
                                                        IkReal x1233 = IKsin(j3);
                                                        IkReal x1234 = ((0.980795316323859) * cj0);
                                                        IkReal x1235 = (sj1 * sj2);
                                                        IkReal x1236 = (cj1 * cj2);
                                                        IkReal x1237 = ((0.980795316323859) * sj0);
                                                        IkReal x1238 = (r00 * sj5);
                                                        IkReal x1239 = (cj5 * r11);
                                                        IkReal x1240 = (r10 * sj5);
                                                        IkReal x1241 = (r02 * sj4);
                                                        IkReal x1242 = (cj5 * r01);
                                                        IkReal x1243 = (r12 * sj4);
                                                        IkReal x1244 = (cj2 * sj1);
                                                        IkReal x1245 = ((0.195039861251953) * sj0);
                                                        IkReal x1246 = (r20 * sj5);
                                                        IkReal x1247 = (r22 * sj4);
                                                        IkReal x1248 = (r21 * sj5);
                                                        IkReal x1249 = (r01 * sj5);
                                                        IkReal x1250 = ((0.195039861251953) * cj0);
                                                        IkReal x1251 = (cj5 * r21);
                                                        IkReal x1252 = (cj1 * sj2);
                                                        IkReal x1253 = ((1.0) * r11 * sj5);
                                                        IkReal x1254 = (cj5 * x1233);
                                                        IkReal x1255 = ((1.0) * x1233);
                                                        IkReal x1256 = (cj4 * x1232);
                                                        IkReal x1257 = (cj5 * x1232);
                                                        IkReal x1258 = ((1.0) * x1232);
                                                        IkReal x1259 = (cj4 * x1255);
                                                        evalcond[0] = (((x1246 * x1256)) + x1252 + (((-1.0) * x1244)) + (((-1.0) * x1248 * x1255)) + ((x1232 * x1247)) + ((x1251 * x1256)) + ((r20 * x1254)));
                                                        evalcond[1] = (x1235 + x1236 + (((-1.0) * x1248 * x1258)) + (((-1.0) * x1246 * x1259)) + (((-1.0) * x1247 * x1255)) + (((-1.0) * x1251 * x1259)) + ((r20 * x1257)));
                                                        evalcond[2] = ((((-1.0) * x1249 * x1255)) + ((x1235 * x1237)) + ((x1238 * x1256)) + ((x1232 * x1241)) + (((-1.0) * x1235 * x1250)) + (((-1.0) * x1236 * x1250)) + ((r00 * x1254)) + ((x1236 * x1237)) + ((x1242 * x1256)));
                                                        evalcond[3] = ((((-1.0) * x1233 * x1253)) + ((r10 * x1254)) + (((-1.0) * x1236 * x1245)) + (((-1.0) * x1235 * x1245)) + ((x1232 * x1243)) + ((x1239 * x1256)) + (((-1.0) * x1234 * x1236)) + (((-1.0) * x1234 * x1235)) + ((x1240 * x1256)));
                                                        evalcond[4] = ((((-1.0) * x1249 * x1258)) + (((-1.0) * x1238 * x1259)) + (((-1.0) * x1242 * x1259)) + (((-1.0) * x1237 * x1252)) + (((-1.0) * x1241 * x1255)) + ((x1250 * x1252)) + (((-1.0) * x1244 * x1250)) + ((r00 * x1257)) + ((x1237 * x1244)));
                                                        evalcond[5] = ((((-1.0) * x1244 * x1245)) + ((x1245 * x1252)) + ((r10 * x1257)) + (((-1.0) * x1239 * x1259)) + ((x1234 * x1252)) + (((-1.0) * x1232 * x1253)) + (((-1.0) * x1243 * x1255)) + (((-1.0) * x1240 * x1259)) + (((-1.0) * x1234 * x1244)));
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

    IKFAST_API void ComputeFk(const IkReal *j, IkReal *eetrans, IkReal *eerot)
    {
        IkReal x0, x1, x2, x3, x4, x5, x6, x7, x8, x9, x10, x11, x12, x13, x14, x15, x16, x17, x18, x19, x20, x21, x22, x23, x24, x25, x26, x27, x28, x29, x30, x31, x32, x33, x34, x35, x36, x37, x38, x39, x40, x41, x42, x43, x44, x45, x46, x47, x48, x49, x50, x51, x52, x53, x54, x55, x56, x57, x58, x59, x60;
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
        x16 = ((4.0e-8) * x7);
        x17 = ((1.0) * x6);
        x18 = ((0.1135) * x3);
        x19 = ((1.0) * x7);
        x20 = ((4.0e-8) * x6);
        x21 = ((-1.0) * x4);
        x22 = (x2 * x4);
        x23 = ((-1.0) * x6);
        x24 = (x2 * x3);
        x25 = ((-1.0) * x7);
        x26 = (x4 * x5);
        x27 = ((-0.0676) * x7);
        x28 = (x3 * x5);
        x29 = ((-0.0676) * x6);
        x30 = ((((-1.0) * x12)) + x13);
        x31 = ((((-1.0) * x13)) + x12);
        x32 = ((((-1.0) * x15)) + x14);
        x33 = ((1.0) * x26);
        x34 = ((((0.195039861251953) * x0)) + (((0.980795316323859) * x1)));
        x35 = ((-1.0) * x34);
        x36 = ((((0.0881580172858829) * x0)) + (((0.443319482978384) * x1)));
        x37 = ((-1.0) * x36);
        x38 = ((0.1135) * x30);
        x39 = (x11 * x31);
        x40 = (x30 * x5);
        x41 = ((0.1135) * x35);
        x42 = ((((-1.0) * x33)) + x24);
        x43 = (x35 * x5);
        x44 = ((((-1.0) * x24)) + x33);
        x45 = (x24 * x31);
        x46 = ((((-1.0) * x22)) + (((-1.0) * x28)));
        x47 = (x42 * x7);
        x48 = (((x28 * x30)) + ((x22 * x30)));
        x49 = (((x28 * x35)) + ((x22 * x35)));
        x50 = (x48 * x7);
        x51 = ((((-1.0) * x45)) + ((x21 * x40)));
        x52 = (((x2 * x21 * x30)) + (((-1.0) * x28 * x30)));
        x53 = ((((-1.0) * x24 * x34)) + (((-1.0) * x33 * x35)));
        x54 = (((x2 * x21 * x35)) + (((-1.0) * x28 * x35)));
        x55 = (((x21 * x43)) + (((-1.0) * x24 * x34)));
        x56 = ((((-1.0) * x17 * x46)) + (((-1.0) * x19 * x42)));
        x57 = (x53 * x6);
        x58 = ((((-1.0) * x19 * x48)) + (((-1.0) * x17 * x51)));
        x59 = ((((-1.0) * x19 * x49)) + (((-1.0) * x17 * x53)));
        x60 = (((x11 * x35)) + ((x10 * x58)));
        eerot[0] = (((x60 * x9)) + ((x8 * ((((x51 * x7)) + ((x52 * x6)))))));
        eerot[1] = (((x9 * ((((x25 * x51)) + ((x23 * x52)))))) + ((x60 * x8)));
        eerot[2] = (((x10 * x34)) + ((x11 * x58)));
        eetrans[0] = (((x7 * ((((x26 * x38)) + ((x18 * x2 * x31)))))) + ((x11 * ((((x27 * x48)) + ((x29 * x51)))))) + ((x32 * x5)) + ((x26 * x32)) + (((0.103375806095808) * x1)) + ((x10 * ((((x20 * x51)) + ((x16 * x48)))))) + ((x10 * (((((0.0663017633834929) * x1)) + (((0.013184694620632) * x0)))))) + ((x6 * ((((x18 * x40)) + ((x22 * x38)))))) + ((x24 * (((((-1.0) * x14)) + x15)))) + (((0.0205573202558309) * x0)) + ((x11 * (((((3.92318126529544e-8) * x1)) + (((7.80159445007813e-9) * x0)))))));
        eerot[3] = (((x8 * ((((x53 * x7)) + ((x54 * x6)))))) + ((x9 * ((((x10 * x59)) + x39)))));
        eerot[4] = (((x9 * (((((-1.0) * x19 * x53)) + (((-1.0) * x17 * x54)))))) + ((x8 * ((x39 + ((x10 * ((((x25 * x49)) + ((x23 * x53)))))))))));
        eerot[5] = (((x10 * x30)) + ((x11 * x59)));
        eetrans[1] = ((((-0.0205573202558309) * x1)) + ((x26 * x37)) + ((x7 * ((((x26 * x41)) + ((x18 * x2 * x34)))))) + (((0.103375806095808) * x0)) + ((x6 * ((((x18 * x43)) + ((x22 * x41)))))) + ((x11 * ((((x27 * x49)) + ((x29 * x55)))))) + ((x10 * ((((x20 * x55)) + ((x16 * x49)))))) + ((x11 * (((((3.92318126529544e-8) * x0)) + (((-7.80159445007813e-9) * x1)))))) + ((x10 * (((((0.0663017633834929) * x0)) + (((-0.013184694620632) * x1)))))) + ((x24 * x36)) + ((x37 * x5)));
        eerot[6] = (((x8 * ((((x46 * x7)) + ((x44 * x6)))))) + ((x10 * x56 * x9)));
        eerot[7] = (((x9 * (((((-1.0) * x17 * x44)) + (((-1.0) * x19 * x46)))))) + ((x10 * x8 * ((((x23 * x46)) + ((x25 * x42)))))));
        eerot[8] = (x11 * x56);
        eetrans[2] = ((0.1125) + (((0.452) * x2)) + ((x10 * ((((x20 * x46)) + ((x16 * x42)))))) + ((x6 * ((((x18 * x2)) + (((-0.1135) * x26)))))) + ((x7 * ((((x18 * x5)) + (((0.1135) * x22)))))) + ((x11 * ((((x27 * x42)) + ((x29 * x46)))))) + (((0.452) * x22)) + (((0.452) * x28)));
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

    IKFAST_API const char *GetKinematicsHash() { return "<robot:GenericRobot - avena (a019696743b6231c5ec013d3b7bfb3a9)>"; }

    IKFAST_API const char *GetIkFastVersion() { return "0x1000004b"; }
} // namespace ik_avena
