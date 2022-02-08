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
            IkReal x63 = ((1.0) * px);
            IkReal x64 = ((1.0) * pz);
            IkReal x65 = ((1.0) * py);
            pp = ((px * px) + (py * py) + (pz * pz));
            npx = (((px * r00)) + ((py * r10)) + ((pz * r20)));
            npy = (((px * r01)) + ((py * r11)) + ((pz * r21)));
            npz = (((px * r02)) + ((py * r12)) + ((pz * r22)));
            rxp0_0 = ((((-1.0) * r20 * x65)) + ((pz * r10)));
            rxp0_1 = (((px * r20)) + (((-1.0) * r00 * x64)));
            rxp0_2 = ((((-1.0) * r10 * x63)) + ((py * r00)));
            rxp1_0 = ((((-1.0) * r21 * x65)) + ((pz * r11)));
            rxp1_1 = (((px * r21)) + (((-1.0) * r01 * x64)));
            rxp1_2 = ((((-1.0) * r11 * x63)) + ((py * r01)));
            rxp2_0 = (((pz * r12)) + (((-1.0) * r22 * x65)));
            rxp2_1 = (((px * r22)) + (((-1.0) * r02 * x64)));
            rxp2_2 = ((((-1.0) * r12 * x63)) + ((py * r02)));
            IkReal op[8 + 1], zeror[8];
            int numroots;
            IkReal x66 = ((-0.132000001500007) + npz);
            IkReal x67 = ((-0.132000001500007) + (((-1.0) * npz)));
            IkReal gconst0 = x66;
            IkReal gconst1 = x67;
            IkReal gconst2 = x66;
            IkReal gconst3 = x67;
            IkReal gconst4 = x66;
            IkReal gconst5 = x67;
            IkReal gconst6 = x66;
            IkReal gconst7 = x67;
            IkReal x68 = r22 * r22;
            IkReal x69 = r21 * r21;
            IkReal x70 = npx * npx;
            IkReal x71 = r20 * r20;
            IkReal x72 = npy * npy;
            IkReal x73 = ((4.0) * npy);
            IkReal x74 = ((1.0) * gconst4);
            IkReal x75 = ((2.0) * gconst1);
            IkReal x76 = ((1.0) * gconst1);
            IkReal x77 = (r20 * r22);
            IkReal x78 = ((2.0) * gconst5);
            IkReal x79 = ((1.0) * gconst0);
            IkReal x80 = (gconst5 * gconst6);
            IkReal x81 = ((16.0) * npx);
            IkReal x82 = (r21 * r22);
            IkReal x83 = ((1.0) * gconst5);
            IkReal x84 = ((8.0) * gconst5);
            IkReal x85 = ((2.0) * gconst4);
            IkReal x86 = ((8.0) * npy);
            IkReal x87 = (r20 * r21);
            IkReal x88 = ((8.0) * gconst1);
            IkReal x89 = ((4.0) * gconst1);
            IkReal x90 = (gconst1 * gconst6);
            IkReal x91 = (gconst2 * gconst5);
            IkReal x92 = ((2.0) * gconst0);
            IkReal x93 = ((2.1904e-12) * x68);
            IkReal x94 = ((5.476e-13) * x68);
            IkReal x95 = (npy * x68);
            IkReal x96 = (gconst2 * x82);
            IkReal x97 = (gconst3 * x68);
            IkReal x98 = (gconst2 * x68);
            IkReal x99 = ((2.0) * x68);
            IkReal x100 = (gconst7 * x68);
            IkReal x101 = (gconst2 * x69);
            IkReal x102 = (gconst6 * x68);
            IkReal x103 = ((16.0) * gconst1 * gconst2);
            IkReal x104 = (npx * x68);
            IkReal x105 = ((16.0) * x71);
            IkReal x106 = (gconst6 * x69);
            IkReal x107 = ((1.48e-6) * gconst5 * x82);
            IkReal x108 = ((1.48e-6) * gconst6 * x82);
            IkReal x109 = ((1.48e-6) * gconst1 * x82);
            IkReal x110 = ((1.48e-6) * x96);
            IkReal x111 = ((2.96e-6) * gconst6 * x77);
            IkReal x112 = ((1.776e-5) * x104);
            IkReal x113 = ((2.96e-6) * gconst5 * x77);
            IkReal x114 = ((5.92e-6) * x95);
            IkReal x115 = ((2.96e-6) * gconst2 * x77);
            IkReal x116 = ((2.96e-6) * gconst1 * x77);
            IkReal x117 = ((5.92e-6) * x104);
            IkReal x118 = ((2.96e-6) * x95);
            IkReal x119 = ((8.0) * npx * x82);
            IkReal x120 = (x68 * x72);
            IkReal x121 = (x68 * x70);
            IkReal x122 = (gconst6 * x73 * x82);
            IkReal x123 = (gconst5 * x73 * x82);
            IkReal x124 = (x81 * x95);
            IkReal x125 = ((16.0) * x80 * x87);
            IkReal x126 = (gconst5 * x77 * x81);
            IkReal x127 = (x73 * x96);
            IkReal x128 = (x102 * x74);
            IkReal x129 = ((4.0) * x69 * x80);
            IkReal x130 = ((1.0) * x68 * x80);
            IkReal x131 = (x100 * x83);
            IkReal x132 = (x100 * x74);
            IkReal x133 = (gconst1 * x73 * x82);
            IkReal x134 = (gconst6 * x77 * x81);
            IkReal x135 = (npx * x82 * x84);
            IkReal x136 = (npy * x77 * x84);
            IkReal x137 = (gconst6 * x77 * x86);
            IkReal x138 = (gconst6 * x119);
            IkReal x139 = ((16.0) * x87 * x90);
            IkReal x140 = ((16.0) * x87 * x91);
            IkReal x141 = (gconst1 * x77 * x81);
            IkReal x142 = ((16.0) * x121);
            IkReal x143 = (x100 * x79);
            IkReal x144 = (x83 * x98);
            IkReal x145 = (x100 * x76);
            IkReal x146 = (x74 * x98);
            IkReal x147 = ((4.0) * x69 * x91);
            IkReal x148 = (gconst2 * x77 * x81);
            IkReal x149 = (x106 * x89);
            IkReal x150 = (x102 * x76);
            IkReal x151 = (x83 * x97);
            IkReal x152 = (x74 * x97);
            IkReal x153 = (x102 * x79);
            IkReal x154 = (npx * x82 * x88);
            IkReal x155 = (gconst1 * x77 * x86);
            IkReal x156 = ((8.0) * npx * x96);
            IkReal x157 = (gconst2 * x77 * x86);
            IkReal x158 = (x103 * x87);
            IkReal x159 = ((4.0) * x120);
            IkReal x160 = (x76 * x98);
            IkReal x161 = (x79 * x97);
            IkReal x162 = (x79 * x98);
            IkReal x163 = (x76 * x97);
            IkReal x164 = (x101 * x89);
            IkReal x165 = (x159 + x94);
            IkReal x166 = (x142 + x93);
            IkReal x167 = (x127 + x108);
            IkReal x168 = (x122 + x109);
            IkReal x169 = (x137 + x138);
            IkReal x170 = (x140 + x139);
            IkReal x171 = (x157 + x156);
            IkReal x172 = (x134 + x123 + x110);
            IkReal x173 = (x148 + x133 + x107);
            IkReal x174 = (x135 + x136 + x124);
            IkReal x175 = (x155 + x154 + x124);
            IkReal x176 = (x131 + x130 + x132 + x128 + x129);
            IkReal x177 = (x162 + x163 + x160 + x161 + x164);
            IkReal x178 = (x153 + x152 + x151 + x150 + x143 + x144 + x145 + x146 + x147 + x149);
            op[0] = ((((-1.0) * x176)) + (((-1.0) * x108)) + (((-1.0) * x122)) + x165 + x123 + x118 + x107);
            op[1] = ((((-1.0) * x174)) + (((-1.0) * x113)) + (((-1.0) * x117)) + x169 + x125 + x111);
            op[2] = ((((-1.0) * x167)) + (((-1.0) * x102 * x85)) + (((-1.0) * x178)) + (((-1.0) * x172)) + (((-1.0) * x105 * x80)) + x168 + x166 + x133 + x126 + x114 + x107 + (((8.0) * x69 * x80)) + (((-1.0) * x100 * x85)) + (((-1.0) * x100 * x78)) + (((-1.0) * x102 * x78)));
            op[3] = ((((-1.0) * x175)) + (((5.92e-6) * gconst6 * x77)) + (((-1.0) * x112)) + (((-1.0) * x116)) + (((-1.0) * x125)) + x171 + x170 + x115 + (((-5.92e-6) * gconst5 * x77)));
            op[4] = ((((-1.0) * x102 * x92)) + ((x101 * x84)) + (((-1.0) * x176)) + (((-1.0) * x177)) + (((-1.0) * x172)) + (((-1.0) * x173)) + x168 + x167 + x141 + x126 + (((-1.0) * x105 * x90)) + (((-1.0) * x105 * x91)) + (((-1.0) * x78 * x98)) + (((-1.0) * x78 * x97)) + ((x106 * x88)) + (((3.2856e-12) * x68)) + (((-8.0) * x120)) + (((32.0) * x121)) + (((-1.0) * x100 * x92)) + (((-1.0) * x100 * x75)) + (((-1.0) * x102 * x75)) + (((-1.0) * x85 * x98)) + (((-1.0) * x85 * x97)));
            op[5] = ((((5.92e-6) * gconst2 * x77)) + (((-1.0) * x169)) + (((-1.0) * x170)) + (((-5.92e-6) * gconst1 * x77)) + (((-1.0) * x112)) + (((-1.0) * x113)) + x174 + x158 + x111);
            op[6] = ((((-1.0) * x168)) + ((x101 * x88)) + (((-1.0) * x178)) + (((-1.0) * x173)) + (((-1.0) * x114)) + x166 + x167 + x141 + x123 + x110 + (((-1.0) * x92 * x98)) + (((-1.0) * x92 * x97)) + (((-1.0) * x75 * x98)) + (((-1.0) * x75 * x97)) + (((-1.0) * x103 * x71)));
            op[7] = ((((-1.0) * x158)) + (((-1.0) * x171)) + (((-1.0) * x116)) + (((-1.0) * x117)) + x175 + x115);
            op[8] = ((((-1.0) * x177)) + (((-1.0) * x109)) + (((-1.0) * x118)) + (((-1.0) * x127)) + x165 + x133 + x110);
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
                    IkReal x179 = (r20 * sj5);
                    IkReal x180 = (cj5 * r21);
                    IkReal x181 = ((2702702.7027027) * npz);
                    IkReal x182 = ((1.0) * npz);
                    IkReal x183 = (npx * r22 * sj5);
                    IkReal x184 = (cj5 * npy * r22);
                    j4eval[0] = ((((-1.0) * x180 * x181)) + (((-1.0) * r22)) + (((-1.0) * x179 * x181)) + (((2702702.7027027) * x184)) + (((2702702.7027027) * x183)));
                    j4eval[1] = IKsign(((((-1.0) * x180 * x182)) + x184 + x183 + (((-1.0) * x179 * x182)) + (((-3.7e-7) * r22))));
                    j4eval[2] = ((((333333666666.667) * (IKabs(((((-0.132000001500007) * x180)) + (((-0.132000001500007) * x179))))))) + (IKabs(((44000044500.0029) * r22))));
                    if (IKabs(j4eval[0]) < 0.0000010000000000 || IKabs(j4eval[1]) < 0.0000010000000000 || IKabs(j4eval[2]) < 0.0000010000000000)
                    {
                        {
                            IkReal evalcond[1];
                            bool bgotonextstatement = true;
                            do
                            {
                                IkReal x185 = r22 * r22;
                                IkReal x186 = npz * npz;
                                IkReal x187 = ((2702702.7027027) * r22);
                                IkReal x188 = ((2702702.7027027) * npz);
                                IkReal x189 = ((7304601899196.49) * x186);
                                IkReal x190 = ((7304601899196.49) * x185);
                                IkReal x191 = (((npy * x187)) + (((-1.0) * r21 * x188)));
                                IkReal x192 = (((npx * x187)) + (((-1.0) * r20 * x188)));
                                IkReal x193 = (((x190 * (npy * npy))) + ((x185 * x189)) + x189 + ((x190 * (npx * npx))) + (((-14609203798393.0) * npz * pz * r22)));
                                if ((x193) < -0.00001)
                                    continue;
                                IkReal x194 = IKabs(IKsqrt(x193));
                                IkReal x200 = x193;
                                if (IKabs(x200) == 0)
                                {
                                    continue;
                                }
                                IkReal x195 = pow(x200, -0.5);
                                CheckValue<IkReal> x201 = IKPowWithIntegerCheck(x194, -1);
                                if (!x201.valid)
                                {
                                    continue;
                                }
                                IkReal x196 = x201.value;
                                if ((((1.0) + (((-1.0) * x185 * (x196 * x196))))) < -0.00001)
                                    continue;
                                IkReal x197 = IKsqrt(((1.0) + (((-1.0) * x185 * (x196 * x196)))));
                                IkReal x198 = (r22 * x195 * x196);
                                IkReal x199 = (x195 * x197);
                                CheckValue<IkReal> x202 = IKatan2WithCheck(IkReal(x191), IkReal(x192), IKFAST_ATAN2_MAGTHRESH);
                                if (!x202.valid)
                                {
                                    continue;
                                }
                                if ((((x192 * x192) + (x191 * x191))) < -0.00001)
                                    continue;
                                CheckValue<IkReal> x203 = IKPowWithIntegerCheck(IKabs(IKsqrt(((x192 * x192) + (x191 * x191)))), -1);
                                if (!x203.valid)
                                {
                                    continue;
                                }
                                if (((r22 * (x203.value))) < -1 - IKFAST_SINCOS_THRESH || ((r22 * (x203.value))) > 1 + IKFAST_SINCOS_THRESH)
                                    continue;
                                IkReal gconst24 = ((((-1.0) * (x202.value))) + (IKasin((r22 * (x203.value)))));
                                IkReal gconst25 = ((((-1.0) * x191 * x199)) + ((x192 * x198)));
                                IkReal gconst26 = (((x192 * x199)) + ((x191 * x198)));
                                IkReal x204 = ((2702702.7027027) * r22);
                                IkReal x205 = ((2702702.7027027) * npz);
                                IkReal x206 = x192;
                                IkReal x207 = x191;
                                if ((((x206 * x206) + (x207 * x207))) < -0.00001)
                                    continue;
                                CheckValue<IkReal> x208 = IKPowWithIntegerCheck(IKabs(IKsqrt(((x206 * x206) + (x207 * x207)))), -1);
                                if (!x208.valid)
                                {
                                    continue;
                                }
                                if (((r22 * (x208.value))) < -1 - IKFAST_SINCOS_THRESH || ((r22 * (x208.value))) > 1 + IKFAST_SINCOS_THRESH)
                                    continue;
                                CheckValue<IkReal> x209 = IKatan2WithCheck(IkReal(x207), IkReal(x206), IKFAST_ATAN2_MAGTHRESH);
                                if (!x209.valid)
                                {
                                    continue;
                                }
                                evalcond[0] = ((-3.14159265358979) + (IKfmod(((3.14159265358979) + (IKabs(((((-1.0) * (IKasin((r22 * (x208.value)))))) + (x209.value) + j5)))), 6.28318530717959)));
                                if (IKabs(evalcond[0]) < 0.0000050000000000)
                                {
                                    bgotonextstatement = false;
                                    {
                                        IkReal j4array[1], cj4array[1], sj4array[1];
                                        bool j4valid[1] = {false};
                                        _nj4 = 1;
                                        IkReal x210 = ((1.0) * npz);
                                        IkReal x211 = (gconst25 * r20);
                                        IkReal x212 = (gconst26 * r21);
                                        CheckValue<IkReal> x213 = IKatan2WithCheck(IkReal(((-0.132000001500007) * r22)), IkReal(((((-0.132000001500007) * x211)) + (((-0.132000001500007) * x212)))), IKFAST_ATAN2_MAGTHRESH);
                                        if (!x213.valid)
                                        {
                                            continue;
                                        }
                                        CheckValue<IkReal> x214 = IKPowWithIntegerCheck(IKsign((((gconst25 * npx * r22)) + (((-1.0) * x210 * x211)) + (((-1.0) * x210 * x212)) + ((gconst26 * npy * r22)) + (((-3.7e-7) * r22)))), -1);
                                        if (!x214.valid)
                                        {
                                            continue;
                                        }
                                        j4array[0] = ((-1.5707963267949) + (x213.value) + (((1.5707963267949) * (x214.value))));
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
                                                IkReal x215 = IKsin(j4);
                                                IkReal x216 = IKcos(j4);
                                                IkReal x217 = ((1.0) * gconst25 * x215);
                                                IkReal x218 = ((1.0) * gconst26 * x215);
                                                evalcond[0] = ((((-1.0) * r20 * x217)) + ((r22 * x216)) + (((-1.0) * r21 * x218)));
                                                evalcond[1] = ((-0.132000001500007) + (((-1.0) * npy * x218)) + (((3.7e-7) * x215)) + (((-1.0) * npx * x217)) + ((npz * x216)));
                                                if (IKabs(evalcond[0]) > IKFAST_EVALCOND_THRESH || IKabs(evalcond[1]) > IKFAST_EVALCOND_THRESH)
                                                {
                                                    continue;
                                                }
                                            }

                                            {
                                                IkReal j0eval[1];
                                                IkReal x219 = ((1.00000049999988) * sj4);
                                                j0eval[0] = ((-1.0) + ((r00 * sj5 * x219)) + ((cj5 * r01 * x219)) + (((-1.00000049999988) * cj4 * r02)));
                                                if (IKabs(j0eval[0]) < 0.0000010000000000)
                                                {
                                                    {
                                                        IkReal j0eval[1];
                                                        IkReal x220 = ((1000.00049999988) * sj4);
                                                        j0eval[0] = ((1.0) + (((-1.0) * r10 * sj5 * x220)) + (((1000.00049999988) * cj4 * r12)) + (((-1.0) * cj5 * r11 * x220)));
                                                        if (IKabs(j0eval[0]) < 0.0000010000000000)
                                                        {
                                                            {
                                                                IkReal j0eval[1];
                                                                IkReal x221 = (sj4 * sj5);
                                                                IkReal x222 = (cj5 * sj4);
                                                                j0eval[0] = ((((1000.0) * r11 * x222)) + (((-1000.0) * cj4 * r12)) + (((1000.0) * r10 * x221)) + (((-1.0) * r01 * x222)) + (((-1.0) * r00 * x221)) + ((cj4 * r02)));
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
                                                                        IkReal x223 = cj4 * cj4;
                                                                        IkReal x224 = cj5 * cj5;
                                                                        IkReal x225 = r00 * r00;
                                                                        IkReal x226 = r10 * r10;
                                                                        IkReal x227 = r01 * r01;
                                                                        IkReal x228 = r11 * r11;
                                                                        IkReal x229 = (cj5 * sj5);
                                                                        IkReal x230 = ((1.999998000002) * r10);
                                                                        IkReal x231 = ((0.001999998000002) * r01);
                                                                        IkReal x232 = (sj4 * sj5);
                                                                        IkReal x233 = (cj4 * r12);
                                                                        IkReal x234 = (cj4 * r02);
                                                                        IkReal x235 = (cj5 * sj4);
                                                                        IkReal x236 = ((0.001999998000002) * r11);
                                                                        IkReal x237 = ((0.001999998000002) * r00);
                                                                        IkReal x238 = (r11 * x223);
                                                                        IkReal x239 = ((1.999998000002e-6) * r00 * r01);
                                                                        IkReal x240 = ((9.99999000001e-7) * x223);
                                                                        IkReal x241 = (r10 * x237);
                                                                        IkReal x242 = ((0.999999000001) * x223);
                                                                        IkReal x243 = ((0.999999000001) * x224);
                                                                        IkReal x244 = (x224 * x227);
                                                                        IkReal x245 = (x224 * x225);
                                                                        IkReal x246 = (x224 * x242);
                                                                        CheckValue<IkReal> x250 = IKPowWithIntegerCheck(((((0.999999500000375) * r11 * x235)) + (((-0.000999999500000375) * r00 * x232)) + (((-0.000999999500000375) * r01 * x235)) + (((0.000999999500000375) * x234)) + (((-0.999999500000375) * x233)) + (((0.999999500000375) * r10 * x232))), -1);
                                                                        if (!x250.valid)
                                                                        {
                                                                            continue;
                                                                        }
                                                                        IkReal x247 = x250.value;
                                                                        if ((((1.0) + ((x226 * x243)) + ((x226 * x242)) + ((r00 * x229 * x236)) + (((-1.0) * x228 * x243)) + (((-0.999999000001) * x226)) + ((x240 * x244)) + (((0.001999998000002) * r02 * r12 * x223)) + (((-1.0) * r10 * x223 * x229 * x231)) + (((-1.0) * r00 * x223 * x229 * x236)) + (((-1.0) * x226 * x246)) + (((-1.0) * x224 * x231 * x238)) + (((-9.99999000001e-7) * x225)) + (((-1.0) * x234 * x235 * x236)) + (((-1.0) * x232 * x233 * x237)) + ((x228 * x246)) + (((-1.0) * x231 * x233 * x235)) + ((x223 * x229 * x239)) + (((-1.0) * x223 * x241)) + ((x223 * x224 * x241)) + (((-0.001999998000002) * r10 * x232 * x234)) + x241 + ((x230 * x232 * x233)) + (((-1.0) * x240 * (r02 * r02))) + (((1.999998000002) * r11 * x233 * x235)) + (((1.999998000002e-6) * r01 * x234 * x235)) + (((-1.0) * x240 * x245)) + ((x225 * x240)) + (((1.999998000002e-6) * r00 * x232 * x234)) + (((-1.0) * x242 * (r12 * r12))) + (((-1.0) * x229 * x239)) + ((x229 * x230 * x238)) + (((9.99999000001e-7) * x245)) + (((-1.0) * x224 * x241)) + ((r10 * x229 * x231)) + (((-9.99999000001e-7) * x244)) + (((-1.0) * r11 * x229 * x230)) + ((r11 * x224 * x231)))) < -0.00001)
                                                                            continue;
                                                                        IkReal x248 = IKsqrt(((1.0) + ((x226 * x243)) + ((x226 * x242)) + ((r00 * x229 * x236)) + (((-1.0) * x228 * x243)) + (((-0.999999000001) * x226)) + ((x240 * x244)) + (((0.001999998000002) * r02 * r12 * x223)) + (((-1.0) * r10 * x223 * x229 * x231)) + (((-1.0) * r00 * x223 * x229 * x236)) + (((-1.0) * x226 * x246)) + (((-1.0) * x224 * x231 * x238)) + (((-9.99999000001e-7) * x225)) + (((-1.0) * x234 * x235 * x236)) + (((-1.0) * x232 * x233 * x237)) + ((x228 * x246)) + (((-1.0) * x231 * x233 * x235)) + ((x223 * x229 * x239)) + (((-1.0) * x223 * x241)) + ((x223 * x224 * x241)) + (((-0.001999998000002) * r10 * x232 * x234)) + x241 + ((x230 * x232 * x233)) + (((-1.0) * x240 * (r02 * r02))) + (((1.999998000002) * r11 * x233 * x235)) + (((1.999998000002e-6) * r01 * x234 * x235)) + (((-1.0) * x240 * x245)) + ((x225 * x240)) + (((1.999998000002e-6) * r00 * x232 * x234)) + (((-1.0) * x242 * (r12 * r12))) + (((-1.0) * x229 * x239)) + ((x229 * x230 * x238)) + (((9.99999000001e-7) * x245)) + (((-1.0) * x224 * x241)) + ((r10 * x229 * x231)) + (((-9.99999000001e-7) * x244)) + (((-1.0) * r11 * x229 * x230)) + ((r11 * x224 * x231))));
                                                                        IkReal x249 = (x247 * x248);
                                                                        j0array[0] = ((2.0) * (atan(((((1.0) * x249)) + (((-1.0) * x247))))));
                                                                        sj0array[0] = IKsin(j0array[0]);
                                                                        cj0array[0] = IKcos(j0array[0]);
                                                                        j0array[1] = ((-2.0) * (atan((x247 + x249))));
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
                                                                IkReal x1244 = (sj4 * sj5);
                                                                IkReal x1245 = (cj5 * sj4);
                                                                CheckValue<IkReal> x1247 = IKPowWithIntegerCheck(((0.000999999500000375) + (((-1.0) * r11 * x1245)) + (((-1.0) * r10 * x1244)) + ((cj4 * r12))), -1);
                                                                if (!x1247.valid)
                                                                {
                                                                    continue;
                                                                }
                                                                IkReal x1246 = x1247.value;
                                                                CheckValue<IkReal> x1248 = IKPowWithIntegerCheck(((0.000999999500000375) + (((-1.0) * r11 * x1245)) + (((-1.0) * r10 * sj4 * sj5)) + ((cj4 * r12))), -1);
                                                                if (!x1248.valid)
                                                                {
                                                                    continue;
                                                                }
                                                                CheckValue<IkReal> x1249 = IKPowWithIntegerCheck(((0.000999999500000375) + (((-1.0) * r10 * x1244)) + (((-1.0) * cj5 * r11 * sj4)) + ((cj4 * r12))), -1);
                                                                if (!x1249.valid)
                                                                {
                                                                    continue;
                                                                }
                                                                j0array[0] = ((2.0) * (atan(((((-1.0) * cj4 * r02 * x1246)) + ((r00 * x1244 * (x1248.value))) + ((r01 * x1245 * (x1249.value))) + (((0.999999500000375) * x1246))))));
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
                                                        IkReal x1250 = ((1.0) * cj4);
                                                        IkReal x1251 = (cj5 * sj4);
                                                        IkReal x1252 = (sj4 * sj5);
                                                        CheckValue<IkReal> x1254 = IKPowWithIntegerCheck(((-0.999999500000375) + (((-1.0) * r02 * x1250)) + ((r00 * x1252)) + ((r01 * x1251))), -1);
                                                        if (!x1254.valid)
                                                        {
                                                            continue;
                                                        }
                                                        IkReal x1253 = x1254.value;
                                                        CheckValue<IkReal> x1255 = IKPowWithIntegerCheck(((-0.999999500000375) + ((r00 * sj4 * sj5)) + (((-1.0) * r02 * x1250)) + ((r01 * x1251))), -1);
                                                        if (!x1255.valid)
                                                        {
                                                            continue;
                                                        }
                                                        CheckValue<IkReal> x1256 = IKPowWithIntegerCheck(((-0.999999500000375) + (((-1.0) * cj4 * r02)) + ((r00 * x1252)) + ((r01 * x1251))), -1);
                                                        if (!x1256.valid)
                                                        {
                                                            continue;
                                                        }
                                                        CheckValue<IkReal> x1257 = IKPowWithIntegerCheck(((-0.999999500000375) + ((cj5 * r01 * sj4)) + (((-1.0) * r02 * x1250)) + ((r00 * x1252))), -1);
                                                        if (!x1257.valid)
                                                        {
                                                            continue;
                                                        }
                                                        j0array[0] = ((2.0) * (atan(((((0.000999999500000375) * x1253)) + ((r10 * x1252 * (x1255.value))) + (((-1.0) * r12 * x1250 * (x1256.value))) + ((r11 * x1251 * (x1257.value)))))));
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
                                    IkReal x1258 = r22 * r22;
                                    IkReal x1259 = npz * npz;
                                    IkReal x1260 = ((2702702.7027027) * r22);
                                    IkReal x1261 = ((2702702.7027027) * npz);
                                    IkReal x1262 = ((7304601899196.49) * x1259);
                                    IkReal x1263 = ((7304601899196.49) * x1258);
                                    IkReal x1264 = (((npx * x1260)) + (((-1.0) * r20 * x1261)));
                                    IkReal x1265 = ((((-1.0) * r21 * x1261)) + ((npy * x1260)));
                                    IkReal x1266 = (((x1263 * (npy * npy))) + x1262 + ((x1258 * x1262)) + ((x1263 * (npx * npx))) + (((-14609203798393.0) * npz * pz * r22)));
                                    if ((x1266) < -0.00001)
                                        continue;
                                    IkReal x1267 = IKabs(IKsqrt(x1266));
                                    IkReal x1275 = x1266;
                                    if (IKabs(x1275) == 0)
                                    {
                                        continue;
                                    }
                                    IkReal x1268 = pow(x1275, -0.5);
                                    IkReal x1269 = ((1.0) * x1268);
                                    CheckValue<IkReal> x1276 = IKPowWithIntegerCheck(x1267, -1);
                                    if (!x1276.valid)
                                    {
                                        continue;
                                    }
                                    IkReal x1270 = x1276.value;
                                    IkReal x1271 = (r22 * x1270);
                                    CheckValue<IkReal> x1277 = IKPowWithIntegerCheck(x1267, -2);
                                    if (!x1277.valid)
                                    {
                                        continue;
                                    }
                                    IkReal x1272 = x1277.value;
                                    IkReal x1273 = (x1258 * x1272);
                                    IkReal x1274 = (x1265 * x1269);
                                    if ((((x1264 * x1264) + (x1265 * x1265))) < -0.00001)
                                        continue;
                                    CheckValue<IkReal> x1278 = IKPowWithIntegerCheck(IKabs(IKsqrt(((x1264 * x1264) + (x1265 * x1265)))), -1);
                                    if (!x1278.valid)
                                    {
                                        continue;
                                    }
                                    if (((r22 * (x1278.value))) < -1 - IKFAST_SINCOS_THRESH || ((r22 * (x1278.value))) > 1 + IKFAST_SINCOS_THRESH)
                                        continue;
                                    CheckValue<IkReal> x1279 = IKatan2WithCheck(IkReal(x1265), IkReal(x1264), IKFAST_ATAN2_MAGTHRESH);
                                    if (!x1279.valid)
                                    {
                                        continue;
                                    }
                                    IkReal gconst27 = ((3.14159265358979) + (((-1.0) * (IKasin((r22 * (x1278.value)))))) + (((-1.0) * (x1279.value))));
                                    if ((((1.0) + (((-1.0) * x1273)))) < -0.00001)
                                        continue;
                                    IkReal gconst28 = (((x1264 * x1269 * x1271)) + ((x1274 * (IKsqrt(((1.0) + (((-1.0) * x1273))))))));
                                    if ((((1.0) + (((-1.0) * x1273)))) < -0.00001)
                                        continue;
                                    IkReal gconst29 = (((x1271 * x1274)) + (((-1.0) * x1264 * x1269 * (IKsqrt(((1.0) + (((-1.0) * x1273))))))));
                                    IkReal x1280 = r22 * r22;
                                    IkReal x1281 = npz * npz;
                                    IkReal x1282 = j5;
                                    IkReal x1283 = (npz * r21);
                                    IkReal x1284 = ((2702702.7027027) * r22);
                                    IkReal x1285 = ((14609203798393.0) * r22);
                                    IkReal x1286 = (npz * r20);
                                    IkReal x1287 = ((7304601899196.49) * x1280);
                                    IkReal x1288 = ((7304601899196.49) * x1281);
                                    CheckValue<IkReal> x1293 = IKatan2WithCheck(IkReal(((((-2702702.7027027) * x1283)) + ((npy * x1284)))), IkReal(((((-2702702.7027027) * x1286)) + ((npx * x1284)))), IKFAST_ATAN2_MAGTHRESH);
                                    if (!x1293.valid)
                                    {
                                        continue;
                                    }
                                    IkReal x1289 = x1293.value;
                                    IkReal x1290 = x1289;
                                    if ((((((-1.0) * npx * x1285 * x1286)) + ((x1288 * (r21 * r21))) + ((x1287 * (npy * npy))) + ((x1288 * (r20 * r20))) + ((x1287 * (npx * npx))) + (((-1.0) * npy * x1283 * x1285)))) < -0.00001)
                                        continue;
                                    CheckValue<IkReal> x1294 = IKPowWithIntegerCheck(IKabs(IKsqrt(((((-1.0) * npx * x1285 * x1286)) + ((x1288 * (r21 * r21))) + ((x1287 * (npy * npy))) + ((x1288 * (r20 * r20))) + ((x1287 * (npx * npx))) + (((-1.0) * npy * x1283 * x1285))))), -1);
                                    if (!x1294.valid)
                                    {
                                        continue;
                                    }
                                    if (((r22 * (x1294.value))) < -1 - IKFAST_SINCOS_THRESH || ((r22 * (x1294.value))) > 1 + IKFAST_SINCOS_THRESH)
                                        continue;
                                    IkReal x1291 = IKasin((r22 * (x1294.value)));
                                    IkReal x1292 = x1291;
                                    if ((((9.86960440108936) + (((-3.14159265358979) * x1289)) + (((-3.14159265358979) * x1282)) + ((j5 * x1290)) + ((j5 * x1292)) + ((x1282 * x1291)) + ((x1290 * x1291)) + ((x1291 * x1292)) + (((-3.14159265358979) * x1290)) + (((-3.14159265358979) * x1291)) + (((-3.14159265358979) * x1292)) + ((j5 * x1282)) + (((-3.14159265358979) * j5)) + ((x1282 * x1289)) + ((x1289 * x1290)) + ((x1289 * x1292)))) < -0.00001)
                                        continue;
                                    evalcond[0] = ((-3.14159265358979) + (IKfmod(((3.14159265358979) + (IKsqrt(((9.86960440108936) + (((-3.14159265358979) * x1289)) + (((-3.14159265358979) * x1282)) + ((j5 * x1290)) + ((j5 * x1292)) + ((x1282 * x1291)) + ((x1290 * x1291)) + ((x1291 * x1292)) + (((-3.14159265358979) * x1290)) + (((-3.14159265358979) * x1291)) + (((-3.14159265358979) * x1292)) + ((j5 * x1282)) + (((-3.14159265358979) * j5)) + ((x1282 * x1289)) + ((x1289 * x1290)) + ((x1289 * x1292)))))), 6.28318530717959)));
                                    if (IKabs(evalcond[0]) < 0.0000050000000000)
                                    {
                                        bgotonextstatement = false;
                                        {
                                            IkReal j4array[1], cj4array[1], sj4array[1];
                                            bool j4valid[1] = {false};
                                            _nj4 = 1;
                                            IkReal x1295 = (gconst28 * r20);
                                            IkReal x1296 = (gconst29 * r21);
                                            IkReal x1297 = ((1.0) * npz);
                                            CheckValue<IkReal> x1298 = IKPowWithIntegerCheck(IKsign((((gconst28 * npx * r22)) + (((-1.0) * x1295 * x1297)) + (((-1.0) * x1296 * x1297)) + ((gconst29 * npy * r22)) + (((-3.7e-7) * r22)))), -1);
                                            if (!x1298.valid)
                                            {
                                                continue;
                                            }
                                            CheckValue<IkReal> x1299 = IKatan2WithCheck(IkReal(((-0.132000001500007) * r22)), IkReal(((((-0.132000001500007) * x1296)) + (((-0.132000001500007) * x1295)))), IKFAST_ATAN2_MAGTHRESH);
                                            if (!x1299.valid)
                                            {
                                                continue;
                                            }
                                            j4array[0] = ((-1.5707963267949) + (((1.5707963267949) * (x1298.value))) + (x1299.value));
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
                                                    IkReal x1300 = IKsin(j4);
                                                    IkReal x1301 = IKcos(j4);
                                                    IkReal x1302 = ((1.0) * x1300);
                                                    evalcond[0] = (((r22 * x1301)) + (((-1.0) * gconst28 * r20 * x1302)) + (((-1.0) * gconst29 * r21 * x1302)));
                                                    evalcond[1] = ((-0.132000001500007) + (((3.7e-7) * x1300)) + (((-1.0) * gconst28 * npx * x1302)) + (((-1.0) * gconst29 * npy * x1302)) + ((npz * x1301)));
                                                    if (IKabs(evalcond[0]) > IKFAST_EVALCOND_THRESH || IKabs(evalcond[1]) > IKFAST_EVALCOND_THRESH)
                                                    {
                                                        continue;
                                                    }
                                                }

                                                {
                                                    IkReal j0eval[1];
                                                    IkReal x1303 = ((1.00000049999988) * sj4);
                                                    j0eval[0] = ((-1.0) + ((r00 * sj5 * x1303)) + ((cj5 * r01 * x1303)) + (((-1.00000049999988) * cj4 * r02)));
                                                    if (IKabs(j0eval[0]) < 0.0000010000000000)
                                                    {
                                                        {
                                                            IkReal j0eval[1];
                                                            IkReal x1304 = ((1000.00049999988) * sj4);
                                                            j0eval[0] = ((1.0) + (((-1.0) * cj5 * r11 * x1304)) + (((1000.00049999988) * cj4 * r12)) + (((-1.0) * r10 * sj5 * x1304)));
                                                            if (IKabs(j0eval[0]) < 0.0000010000000000)
                                                            {
                                                                {
                                                                    IkReal j0eval[1];
                                                                    IkReal x1305 = (sj4 * sj5);
                                                                    IkReal x1306 = (cj5 * sj4);
                                                                    j0eval[0] = ((((-1.0) * r00 * x1305)) + (((-1000.0) * cj4 * r12)) + (((-1.0) * r01 * x1306)) + (((1000.0) * r10 * x1305)) + (((1000.0) * r11 * x1306)) + ((cj4 * r02)));
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
                                                                            IkReal x1307 = cj4 * cj4;
                                                                            IkReal x1308 = cj5 * cj5;
                                                                            IkReal x1309 = r00 * r00;
                                                                            IkReal x1310 = r10 * r10;
                                                                            IkReal x1311 = r01 * r01;
                                                                            IkReal x1312 = r11 * r11;
                                                                            IkReal x1313 = (cj5 * sj5);
                                                                            IkReal x1314 = ((1.999998000002) * r10);
                                                                            IkReal x1315 = ((0.001999998000002) * r01);
                                                                            IkReal x1316 = (sj4 * sj5);
                                                                            IkReal x1317 = (cj4 * r12);
                                                                            IkReal x1318 = (cj4 * r02);
                                                                            IkReal x1319 = (cj5 * sj4);
                                                                            IkReal x1320 = ((0.001999998000002) * r11);
                                                                            IkReal x1321 = ((0.001999998000002) * r00);
                                                                            IkReal x1322 = (r11 * x1307);
                                                                            IkReal x1323 = ((1.999998000002e-6) * r00 * r01);
                                                                            IkReal x1324 = ((9.99999000001e-7) * x1307);
                                                                            IkReal x1325 = (r10 * x1321);
                                                                            IkReal x1326 = ((0.999999000001) * x1307);
                                                                            IkReal x1327 = ((0.999999000001) * x1308);
                                                                            IkReal x1328 = (x1308 * x1311);
                                                                            IkReal x1329 = (x1308 * x1309);
                                                                            IkReal x1330 = (x1308 * x1326);
                                                                            CheckValue<IkReal> x1334 = IKPowWithIntegerCheck(((((0.999999500000375) * r10 * x1316)) + (((-0.000999999500000375) * r01 * x1319)) + (((0.999999500000375) * r11 * x1319)) + (((0.000999999500000375) * x1318)) + (((-0.000999999500000375) * r00 * x1316)) + (((-0.999999500000375) * x1317))), -1);
                                                                            if (!x1334.valid)
                                                                            {
                                                                                continue;
                                                                            }
                                                                            IkReal x1331 = x1334.value;
                                                                            if ((((1.0) + x1325 + ((x1312 * x1330)) + (((-9.99999000001e-7) * x1309)) + (((-1.0) * x1312 * x1327)) + (((-1.0) * r10 * x1307 * x1313 * x1315)) + ((x1314 * x1316 * x1317)) + (((-1.0) * r00 * x1307 * x1313 * x1320)) + (((-1.0) * x1308 * x1325)) + (((9.99999000001e-7) * x1329)) + ((r00 * x1313 * x1320)) + (((-1.0) * x1324 * x1329)) + ((r11 * x1308 * x1315)) + (((-1.0) * x1313 * x1323)) + (((0.001999998000002) * r02 * r12 * x1307)) + (((-1.0) * x1307 * x1325)) + (((1.999998000002) * r11 * x1317 * x1319)) + ((x1313 * x1314 * x1322)) + ((x1309 * x1324)) + ((x1307 * x1308 * x1325)) + (((-1.0) * x1324 * (r02 * r02))) + (((-1.0) * x1310 * x1330)) + (((-1.0) * x1315 * x1317 * x1319)) + ((x1324 * x1328)) + ((r10 * x1313 * x1315)) + (((-0.001999998000002) * r10 * x1316 * x1318)) + (((-1.0) * x1308 * x1315 * x1322)) + (((-0.999999000001) * x1310)) + (((-1.0) * r11 * x1313 * x1314)) + (((-1.0) * x1316 * x1317 * x1321)) + (((-1.0) * x1326 * (r12 * r12))) + (((1.999998000002e-6) * r00 * x1316 * x1318)) + (((-1.0) * x1318 * x1319 * x1320)) + (((1.999998000002e-6) * r01 * x1318 * x1319)) + ((x1307 * x1313 * x1323)) + (((-9.99999000001e-7) * x1328)) + ((x1310 * x1327)) + ((x1310 * x1326)))) < -0.00001)
                                                                                continue;
                                                                            IkReal x1332 = IKsqrt(((1.0) + x1325 + ((x1312 * x1330)) + (((-9.99999000001e-7) * x1309)) + (((-1.0) * x1312 * x1327)) + (((-1.0) * r10 * x1307 * x1313 * x1315)) + ((x1314 * x1316 * x1317)) + (((-1.0) * r00 * x1307 * x1313 * x1320)) + (((-1.0) * x1308 * x1325)) + (((9.99999000001e-7) * x1329)) + ((r00 * x1313 * x1320)) + (((-1.0) * x1324 * x1329)) + ((r11 * x1308 * x1315)) + (((-1.0) * x1313 * x1323)) + (((0.001999998000002) * r02 * r12 * x1307)) + (((-1.0) * x1307 * x1325)) + (((1.999998000002) * r11 * x1317 * x1319)) + ((x1313 * x1314 * x1322)) + ((x1309 * x1324)) + ((x1307 * x1308 * x1325)) + (((-1.0) * x1324 * (r02 * r02))) + (((-1.0) * x1310 * x1330)) + (((-1.0) * x1315 * x1317 * x1319)) + ((x1324 * x1328)) + ((r10 * x1313 * x1315)) + (((-0.001999998000002) * r10 * x1316 * x1318)) + (((-1.0) * x1308 * x1315 * x1322)) + (((-0.999999000001) * x1310)) + (((-1.0) * r11 * x1313 * x1314)) + (((-1.0) * x1316 * x1317 * x1321)) + (((-1.0) * x1326 * (r12 * r12))) + (((1.999998000002e-6) * r00 * x1316 * x1318)) + (((-1.0) * x1318 * x1319 * x1320)) + (((1.999998000002e-6) * r01 * x1318 * x1319)) + ((x1307 * x1313 * x1323)) + (((-9.99999000001e-7) * x1328)) + ((x1310 * x1327)) + ((x1310 * x1326))));
                                                                            IkReal x1333 = (x1331 * x1332);
                                                                            j0array[0] = ((-2.0) * (atan((x1331 + (((-1.0) * x1333))))));
                                                                            sj0array[0] = IKsin(j0array[0]);
                                                                            cj0array[0] = IKcos(j0array[0]);
                                                                            j0array[1] = ((-2.0) * (atan((x1331 + x1333))));
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
                                                                    IkReal x1335 = (sj4 * sj5);
                                                                    IkReal x1336 = (cj5 * sj4);
                                                                    CheckValue<IkReal> x1338 = IKPowWithIntegerCheck(((0.000999999500000375) + (((-1.0) * r11 * x1336)) + (((-1.0) * r10 * x1335)) + ((cj4 * r12))), -1);
                                                                    if (!x1338.valid)
                                                                    {
                                                                        continue;
                                                                    }
                                                                    IkReal x1337 = x1338.value;
                                                                    CheckValue<IkReal> x1339 = IKPowWithIntegerCheck(((0.000999999500000375) + (((-1.0) * r10 * x1335)) + (((-1.0) * cj5 * r11 * sj4)) + ((cj4 * r12))), -1);
                                                                    if (!x1339.valid)
                                                                    {
                                                                        continue;
                                                                    }
                                                                    CheckValue<IkReal> x1340 = IKPowWithIntegerCheck(((0.000999999500000375) + (((-1.0) * r11 * x1336)) + (((-1.0) * r10 * sj4 * sj5)) + ((cj4 * r12))), -1);
                                                                    if (!x1340.valid)
                                                                    {
                                                                        continue;
                                                                    }
                                                                    j0array[0] = ((2.0) * (atan(((((0.999999500000375) * x1337)) + (((-1.0) * cj4 * r02 * x1337)) + ((r01 * x1336 * (x1339.value))) + ((r00 * x1335 * (x1340.value)))))));
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
                                                            IkReal x1341 = ((1.0) * cj4);
                                                            IkReal x1342 = (cj5 * sj4);
                                                            IkReal x1343 = (sj4 * sj5);
                                                            CheckValue<IkReal> x1345 = IKPowWithIntegerCheck(((-0.999999500000375) + (((-1.0) * r02 * x1341)) + ((r00 * x1343)) + ((r01 * x1342))), -1);
                                                            if (!x1345.valid)
                                                            {
                                                                continue;
                                                            }
                                                            IkReal x1344 = x1345.value;
                                                            CheckValue<IkReal> x1346 = IKPowWithIntegerCheck(((-0.999999500000375) + (((-1.0) * r02 * x1341)) + ((cj5 * r01 * sj4)) + ((r00 * x1343))), -1);
                                                            if (!x1346.valid)
                                                            {
                                                                continue;
                                                            }
                                                            CheckValue<IkReal> x1347 = IKPowWithIntegerCheck(((-0.999999500000375) + ((r00 * sj4 * sj5)) + (((-1.0) * r02 * x1341)) + ((r01 * x1342))), -1);
                                                            if (!x1347.valid)
                                                            {
                                                                continue;
                                                            }
                                                            CheckValue<IkReal> x1348 = IKPowWithIntegerCheck(((-0.999999500000375) + (((-1.0) * cj4 * r02)) + ((r00 * x1343)) + ((r01 * x1342))), -1);
                                                            if (!x1348.valid)
                                                            {
                                                                continue;
                                                            }
                                                            j0array[0] = ((2.0) * (atan((((r11 * x1342 * (x1346.value))) + ((r10 * x1343 * (x1347.value))) + (((0.000999999500000375) * x1344)) + (((-1.0) * r12 * x1341 * (x1348.value)))))));
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
                    else
                    {
                        {
                            IkReal j4array[1], cj4array[1], sj4array[1];
                            bool j4valid[1] = {false};
                            _nj4 = 1;
                            IkReal x1349 = (r20 * sj5);
                            IkReal x1350 = (cj5 * r21);
                            IkReal x1351 = ((1.0) * npz);
                            CheckValue<IkReal> x1352 = IKatan2WithCheck(IkReal(((-0.132000001500007) * r22)), IkReal(((((-0.132000001500007) * x1350)) + (((-0.132000001500007) * x1349)))), IKFAST_ATAN2_MAGTHRESH);
                            if (!x1352.valid)
                            {
                                continue;
                            }
                            CheckValue<IkReal> x1353 = IKPowWithIntegerCheck(IKsign(((((-1.0) * x1350 * x1351)) + ((npx * r22 * sj5)) + (((-1.0) * x1349 * x1351)) + ((cj5 * npy * r22)) + (((-3.7e-7) * r22)))), -1);
                            if (!x1353.valid)
                            {
                                continue;
                            }
                            j4array[0] = ((-1.5707963267949) + (x1352.value) + (((1.5707963267949) * (x1353.value))));
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
                                    IkReal x1354 = IKsin(j4);
                                    IkReal x1355 = IKcos(j4);
                                    IkReal x1356 = ((1.0) * x1354);
                                    evalcond[0] = (((r22 * x1355)) + (((-1.0) * cj5 * r21 * x1356)) + (((-1.0) * r20 * sj5 * x1356)));
                                    evalcond[1] = ((-0.132000001500007) + (((3.7e-7) * x1354)) + (((-1.0) * npx * sj5 * x1356)) + (((-1.0) * cj5 * npy * x1356)) + ((npz * x1355)));
                                    if (IKabs(evalcond[0]) > IKFAST_EVALCOND_THRESH || IKabs(evalcond[1]) > IKFAST_EVALCOND_THRESH)
                                    {
                                        continue;
                                    }
                                }

                                {
                                    IkReal j0eval[1];
                                    IkReal x1357 = ((1.00000049999988) * sj4);
                                    j0eval[0] = ((-1.0) + ((cj5 * r01 * x1357)) + (((-1.00000049999988) * cj4 * r02)) + ((r00 * sj5 * x1357)));
                                    if (IKabs(j0eval[0]) < 0.0000010000000000)
                                    {
                                        {
                                            IkReal j0eval[1];
                                            IkReal x1358 = ((1000.00049999988) * sj4);
                                            j0eval[0] = ((1.0) + (((-1.0) * r10 * sj5 * x1358)) + (((-1.0) * cj5 * r11 * x1358)) + (((1000.00049999988) * cj4 * r12)));
                                            if (IKabs(j0eval[0]) < 0.0000010000000000)
                                            {
                                                {
                                                    IkReal j0eval[1];
                                                    IkReal x1359 = (sj4 * sj5);
                                                    IkReal x1360 = (cj5 * sj4);
                                                    j0eval[0] = ((((-1.0) * r00 * x1359)) + (((-1000.0) * cj4 * r12)) + (((1000.0) * r11 * x1360)) + (((1000.0) * r10 * x1359)) + (((-1.0) * r01 * x1360)) + ((cj4 * r02)));
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
                                                            IkReal x1361 = cj4 * cj4;
                                                            IkReal x1362 = cj5 * cj5;
                                                            IkReal x1363 = r00 * r00;
                                                            IkReal x1364 = r10 * r10;
                                                            IkReal x1365 = r01 * r01;
                                                            IkReal x1366 = r11 * r11;
                                                            IkReal x1367 = (cj5 * sj5);
                                                            IkReal x1368 = ((1.999998000002) * r10);
                                                            IkReal x1369 = ((0.001999998000002) * r01);
                                                            IkReal x1370 = (sj4 * sj5);
                                                            IkReal x1371 = (cj4 * r12);
                                                            IkReal x1372 = (cj4 * r02);
                                                            IkReal x1373 = (cj5 * sj4);
                                                            IkReal x1374 = ((0.001999998000002) * r11);
                                                            IkReal x1375 = ((0.001999998000002) * r00);
                                                            IkReal x1376 = (r11 * x1361);
                                                            IkReal x1377 = ((1.999998000002e-6) * r00 * r01);
                                                            IkReal x1378 = ((9.99999000001e-7) * x1361);
                                                            IkReal x1379 = (r10 * x1375);
                                                            IkReal x1380 = ((0.999999000001) * x1361);
                                                            IkReal x1381 = ((0.999999000001) * x1362);
                                                            IkReal x1382 = (x1362 * x1365);
                                                            IkReal x1383 = (x1362 * x1363);
                                                            IkReal x1384 = (x1362 * x1380);
                                                            CheckValue<IkReal> x1388 = IKPowWithIntegerCheck(((((0.999999500000375) * r10 * x1370)) + (((0.999999500000375) * r11 * x1373)) + (((-0.000999999500000375) * r00 * x1370)) + (((-0.999999500000375) * x1371)) + (((-0.000999999500000375) * r01 * x1373)) + (((0.000999999500000375) * x1372))), -1);
                                                            if (!x1388.valid)
                                                            {
                                                                continue;
                                                            }
                                                            IkReal x1385 = x1388.value;
                                                            if ((((1.0) + x1379 + (((-1.0) * x1364 * x1384)) + ((x1366 * x1384)) + ((x1364 * x1380)) + ((x1364 * x1381)) + (((-1.0) * x1362 * x1379)) + (((-0.001999998000002) * r10 * x1370 * x1372)) + (((-1.0) * r00 * x1361 * x1367 * x1374)) + (((-9.99999000001e-7) * x1382)) + (((-1.0) * x1369 * x1371 * x1373)) + (((-1.0) * x1372 * x1373 * x1374)) + (((-1.0) * r10 * x1361 * x1367 * x1369)) + (((-1.0) * x1380 * (r12 * r12))) + (((-0.999999000001) * x1364)) + (((-1.0) * x1378 * (r02 * r02))) + ((x1361 * x1367 * x1377)) + (((-1.0) * x1366 * x1381)) + ((x1368 * x1370 * x1371)) + (((-1.0) * x1362 * x1369 * x1376)) + (((-9.99999000001e-7) * x1363)) + (((-1.0) * r11 * x1367 * x1368)) + (((0.001999998000002) * r02 * r12 * x1361)) + (((-1.0) * x1361 * x1379)) + ((r00 * x1367 * x1374)) + (((9.99999000001e-7) * x1383)) + (((-1.0) * x1367 * x1377)) + (((1.999998000002e-6) * r01 * x1372 * x1373)) + ((x1367 * x1368 * x1376)) + ((r10 * x1367 * x1369)) + (((-1.0) * x1378 * x1383)) + ((r11 * x1362 * x1369)) + (((-1.0) * x1370 * x1371 * x1375)) + (((1.999998000002e-6) * r00 * x1370 * x1372)) + ((x1363 * x1378)) + ((x1378 * x1382)) + (((1.999998000002) * r11 * x1371 * x1373)) + ((x1361 * x1362 * x1379)))) < -0.00001)
                                                                continue;
                                                            IkReal x1386 = IKsqrt(((1.0) + x1379 + (((-1.0) * x1364 * x1384)) + ((x1366 * x1384)) + ((x1364 * x1380)) + ((x1364 * x1381)) + (((-1.0) * x1362 * x1379)) + (((-0.001999998000002) * r10 * x1370 * x1372)) + (((-1.0) * r00 * x1361 * x1367 * x1374)) + (((-9.99999000001e-7) * x1382)) + (((-1.0) * x1369 * x1371 * x1373)) + (((-1.0) * x1372 * x1373 * x1374)) + (((-1.0) * r10 * x1361 * x1367 * x1369)) + (((-1.0) * x1380 * (r12 * r12))) + (((-0.999999000001) * x1364)) + (((-1.0) * x1378 * (r02 * r02))) + ((x1361 * x1367 * x1377)) + (((-1.0) * x1366 * x1381)) + ((x1368 * x1370 * x1371)) + (((-1.0) * x1362 * x1369 * x1376)) + (((-9.99999000001e-7) * x1363)) + (((-1.0) * r11 * x1367 * x1368)) + (((0.001999998000002) * r02 * r12 * x1361)) + (((-1.0) * x1361 * x1379)) + ((r00 * x1367 * x1374)) + (((9.99999000001e-7) * x1383)) + (((-1.0) * x1367 * x1377)) + (((1.999998000002e-6) * r01 * x1372 * x1373)) + ((x1367 * x1368 * x1376)) + ((r10 * x1367 * x1369)) + (((-1.0) * x1378 * x1383)) + ((r11 * x1362 * x1369)) + (((-1.0) * x1370 * x1371 * x1375)) + (((1.999998000002e-6) * r00 * x1370 * x1372)) + ((x1363 * x1378)) + ((x1378 * x1382)) + (((1.999998000002) * r11 * x1371 * x1373)) + ((x1361 * x1362 * x1379))));
                                                            IkReal x1387 = (x1385 * x1386);
                                                            j0array[0] = ((2.0) * (atan(((((-1.0) * x1385)) + (((1.0) * x1387))))));
                                                            sj0array[0] = IKsin(j0array[0]);
                                                            cj0array[0] = IKcos(j0array[0]);
                                                            j0array[1] = ((-2.0) * (atan((x1385 + x1387))));
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
                                                    IkReal x1389 = (sj4 * sj5);
                                                    IkReal x1390 = (cj5 * sj4);
                                                    CheckValue<IkReal> x1392 = IKPowWithIntegerCheck(((0.000999999500000375) + (((-1.0) * r10 * x1389)) + ((cj4 * r12)) + (((-1.0) * r11 * x1390))), -1);
                                                    if (!x1392.valid)
                                                    {
                                                        continue;
                                                    }
                                                    IkReal x1391 = x1392.value;
                                                    CheckValue<IkReal> x1393 = IKPowWithIntegerCheck(((0.000999999500000375) + (((-1.0) * r10 * sj4 * sj5)) + ((cj4 * r12)) + (((-1.0) * r11 * x1390))), -1);
                                                    if (!x1393.valid)
                                                    {
                                                        continue;
                                                    }
                                                    CheckValue<IkReal> x1394 = IKPowWithIntegerCheck(((0.000999999500000375) + (((-1.0) * r10 * x1389)) + (((-1.0) * cj5 * r11 * sj4)) + ((cj4 * r12))), -1);
                                                    if (!x1394.valid)
                                                    {
                                                        continue;
                                                    }
                                                    j0array[0] = ((2.0) * (atan(((((0.999999500000375) * x1391)) + ((r00 * x1389 * (x1393.value))) + (((-1.0) * cj4 * r02 * x1391)) + ((r01 * x1390 * (x1394.value)))))));
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
                                            IkReal x1395 = ((1.0) * cj4);
                                            IkReal x1396 = (cj5 * sj4);
                                            IkReal x1397 = (sj4 * sj5);
                                            CheckValue<IkReal> x1399 = IKPowWithIntegerCheck(((-0.999999500000375) + (((-1.0) * r02 * x1395)) + ((r01 * x1396)) + ((r00 * x1397))), -1);
                                            if (!x1399.valid)
                                            {
                                                continue;
                                            }
                                            IkReal x1398 = x1399.value;
                                            CheckValue<IkReal> x1400 = IKPowWithIntegerCheck(((-0.999999500000375) + (((-1.0) * cj4 * r02)) + ((r01 * x1396)) + ((r00 * x1397))), -1);
                                            if (!x1400.valid)
                                            {
                                                continue;
                                            }
                                            CheckValue<IkReal> x1401 = IKPowWithIntegerCheck(((-0.999999500000375) + ((r00 * sj4 * sj5)) + (((-1.0) * r02 * x1395)) + ((r01 * x1396))), -1);
                                            if (!x1401.valid)
                                            {
                                                continue;
                                            }
                                            CheckValue<IkReal> x1402 = IKPowWithIntegerCheck(((-0.999999500000375) + (((-1.0) * r02 * x1395)) + ((cj5 * r01 * sj4)) + ((r00 * x1397))), -1);
                                            if (!x1402.valid)
                                            {
                                                continue;
                                            }
                                            j0array[0] = ((2.0) * (atan(((((0.000999999500000375) * x1398)) + (((-1.0) * r12 * x1395 * (x1400.value))) + ((r10 * x1397 * (x1401.value))) + ((r11 * x1396 * (x1402.value)))))));
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
            IkReal x251 = ((3.7e-7) * cj5);
            IkReal x252 = ((0.1135) * sj5);
            IkReal x253 = ((3.7e-7) * sj5);
            IkReal x254 = ((0.1135) * cj5);
            IkReal x255 = (r21 * x252);
            IkReal x256 = (r21 * x251);
            IkReal x257 = (r20 * x253);
            IkReal x258 = ((((-1.3419993290005) * sj0)) + (((-0.0013419993290005) * cj0)));
            IkReal x259 = ((((0.00271399864300102) * cj0)) + (((2.71399864300102) * sj0)));
            IkReal x260 = ((((0.0299999850000113) * sj0)) + (((2.99999850000112e-5) * cj0)));
            IkReal x261 = ((((0.0013419993290005) * cj0)) + (((1.3419993290005) * sj0)));
            IkReal x262 = (((r20 * x254)) + (((1.0) * pz)));
            IkReal x263 = (x255 + x256 + x257);
            IkReal x264 = ((1.357) + x263 + (((-1.0) * x262)));
            IkReal x265 = ((0.015) + x263 + (((-1.0) * x262)));
            IkReal x266 = ((-1.357) + x263 + (((-1.0) * x262)));
            IkReal x267 = ((-0.015) + x263 + (((-1.0) * x262)));
            IkReal x268 = (((r00 * x253)) + ((r01 * x252)) + ((r01 * x251)) + (((-0.000131999965500026) * sj0)) + (((-1.0) * r00 * x254)) + (((-1.0) * px)) + (((0.131999935500026) * cj0)));
            IkReal gconst30 = x264;
            IkReal gconst31 = x265;
            IkReal gconst32 = x268;
            IkReal gconst33 = x258;
            IkReal gconst34 = x268;
            IkReal gconst35 = x264;
            IkReal gconst36 = x265;
            IkReal gconst37 = x268;
            IkReal gconst38 = x258;
            IkReal gconst39 = x268;
            IkReal gconst40 = x259;
            IkReal gconst41 = x260;
            IkReal gconst42 = x259;
            IkReal gconst43 = x260;
            IkReal gconst44 = x266;
            IkReal gconst45 = x267;
            IkReal gconst46 = x268;
            IkReal gconst47 = x261;
            IkReal gconst48 = x268;
            IkReal gconst49 = x266;
            IkReal gconst50 = x267;
            IkReal gconst51 = x268;
            IkReal gconst52 = x261;
            IkReal gconst53 = x268;
            IkReal x269 = ((1.0) * gconst30);
            IkReal x270 = (gconst35 * gconst53);
            IkReal x271 = (gconst45 * gconst49);
            IkReal x272 = (gconst39 * gconst46);
            IkReal x273 = (gconst45 * gconst51);
            IkReal x274 = ((1.0) * gconst36);
            IkReal x275 = ((2.684) * gconst47);
            IkReal x276 = (gconst43 * gconst49);
            IkReal x277 = (gconst34 * gconst37);
            IkReal x278 = (gconst50 * gconst51);
            IkReal x279 = (gconst34 * gconst44);
            IkReal x280 = (gconst43 * gconst46);
            IkReal x281 = (gconst42 * gconst45);
            IkReal x282 = (gconst46 * gconst53);
            IkReal x283 = (gconst31 * gconst49);
            IkReal x284 = (gconst40 * gconst43);
            IkReal x285 = (gconst31 * gconst40);
            IkReal x286 = ((1.0) * gconst44);
            IkReal x287 = (gconst41 * gconst44);
            IkReal x288 = (gconst36 * gconst51);
            IkReal x289 = (gconst44 * gconst48);
            IkReal x290 = (gconst35 * gconst39);
            IkReal x291 = (gconst30 * gconst34);
            IkReal x292 = (gconst39 * gconst49);
            IkReal x293 = (gconst36 * gconst37);
            IkReal x294 = (gconst31 * gconst37);
            IkReal x295 = ((1.0) * gconst50);
            IkReal x296 = (gconst42 * gconst50);
            IkReal x297 = (gconst30 * gconst48);
            IkReal x298 = (gconst32 * gconst53);
            IkReal x299 = (gconst32 * gconst39);
            IkReal x300 = (gconst31 * gconst35);
            IkReal x301 = (gconst35 * gconst45);
            IkReal x302 = (gconst31 * gconst32);
            IkReal x303 = ((1.0) * gconst38);
            IkReal x304 = (gconst32 * gconst45);
            IkReal x305 = (gconst30 * gconst41);
            IkReal x306 = (gconst45 * gconst46);
            IkReal x307 = (gconst37 * gconst50);
            IkReal x308 = ((2.684) * gconst33);
            IkReal x309 = ((2.684) * gconst35);
            IkReal x310 = (gconst34 * gconst38);
            IkReal x311 = ((2.684) * gconst49);
            IkReal x312 = (gconst31 * gconst46);
            IkReal x313 = (gconst39 * gconst40);
            IkReal x314 = ((7.203856) * gconst41);
            IkReal x315 = (gconst38 * gconst48);
            IkReal x316 = ((7.203856) * gconst48);
            IkReal x317 = (gconst31 * gconst42);
            IkReal x318 = (gconst32 * gconst43);
            IkReal x319 = (gconst49 * gconst53);
            IkReal x320 = (gconst35 * gconst43);
            IkReal x321 = (gconst36 * gconst42);
            IkReal x322 = (gconst31 * gconst51);
            IkReal x323 = ((7.203856) * gconst34);
            IkReal x324 = (gconst48 * gconst52);
            IkReal x325 = (gconst40 * gconst53);
            IkReal x326 = (gconst37 * gconst45);
            IkReal x327 = (gconst34 * gconst52);
            IkReal x328 = (gconst40 * gconst45);
            IkReal x329 = ((1.0) * gconst47 * gconst52);
            IkReal x330 = (gconst48 * x320);
            IkReal x331 = (gconst32 * x295);
            IkReal x332 = (gconst41 * x311);
            IkReal x333 = (gconst41 * x320);
            IkReal x334 = ((1.0) * gconst33 * gconst52);
            op[0] = (((x278 * x289)) + (((-1.0) * x271 * x329)) + ((x271 * x282)) + (((-1.0) * gconst46 * x273 * x295)) + (((-1.0) * gconst48 * x286 * x319)));
            op[1] = (((x273 * x275)) + ((x289 * x296)) + ((x278 * x287)) + ((x271 * x325)) + (((-1.0) * gconst48 * x276 * x286)) + (((-1.0) * gconst40 * x273 * x295)) + (((-1.0) * gconst46 * x281 * x295)) + ((x271 * x280)) + (((-1.0) * gconst41 * x286 * x319)) + ((x311 * x324)));
            op[2] = (((x271 * x298)) + (((-1.0) * x271 * x334)) + (((-1.0) * gconst37 * x295 * x306)) + ((x271 * x272)) + (((-1.0) * gconst48 * x286 * x292)) + (((-1.0) * gconst40 * x281 * x295)) + (((-1.0) * gconst48 * x269 * x319)) + ((x287 * x296)) + (((-1.0) * x279 * x319)) + (((-1.0) * gconst51 * x316)) + ((x271 * x284)) + (((-1.0) * gconst46 * x273 * x274)) + (((-1.0) * x278 * x312)) + ((x288 * x289)) + (((-1.0) * x301 * x329)) + ((x278 * x297)) + (((-1.0) * gconst47 * x271 * x303)) + ((x275 * x281)) + ((x270 * x306)) + (((-1.0) * x273 * x331)) + (((-1.0) * x283 * x329)) + ((gconst52 * x332)) + ((x289 * x307)) + ((x278 * x279)) + (((-1.0) * gconst41 * x276 * x286)) + (((-1.0) * gconst48 * x270 * x286)) + ((x282 * x283)));
            op[3] = (((x276 * x312)) + (((-1.0) * x276 * x279)) + (((-1.0) * gconst41 * x269 * x319)) + (((-1.0) * gconst42 * x316)) + ((x287 * x307)) + ((x289 * x321)) + ((x271 * x313)) + ((x271 * x318)) + (((-1.0) * gconst40 * x295 * x326)) + ((x278 * x305)) + ((x280 * x301)) + (((-1.0) * gconst51 * x314)) + ((x275 * x322)) + ((x275 * x326)) + (((-1.0) * x286 * x330)) + (((-1.0) * gconst41 * x270 * x286)) + ((x279 * x296)) + (((-1.0) * x278 * x285)) + (((-1.0) * x281 * x331)) + ((x270 * x328)) + (((-1.0) * gconst42 * x295 * x312)) + ((x311 * x315)) + ((x309 * x324)) + ((x296 * x297)) + (((-1.0) * gconst40 * x273 * x274)) + ((x311 * x327)) + (((-1.0) * gconst41 * x286 * x292)) + ((x283 * x325)) + ((x287 * x288)) + (((-1.0) * gconst46 * x274 * x281)) + (((-1.0) * gconst48 * x269 * x276)) + ((x273 * x308)));
            op[4] = ((((-1.0) * gconst47 * x301 * x303)) + ((x271 * x299)) + ((x289 * x293)) + (((-1.0) * gconst37 * x295 * x304)) + (((-1.0) * gconst42 * x314)) + (((-1.0) * gconst48 * x286 * x290)) + ((x279 * x288)) + (((-1.0) * x283 * x334)) + (((-1.0) * gconst51 * x323)) + (((-1.0) * gconst37 * x316)) + ((gconst44 * gconst50 * x277)) + ((gconst41 * gconst52 * x309)) + ((x275 * x317)) + (((-1.0) * gconst32 * x273 * x274)) + (((-1.0) * gconst42 * x285 * x295)) + ((x284 * x301)) + ((gconst38 * x332)) + ((x283 * x298)) + ((x287 * x321)) + (((-1.0) * gconst48 * x269 * x292)) + (((-1.0) * x286 * x333)) + (((-1.0) * gconst34 * x269 * x319)) + (((-1.0) * gconst40 * x274 * x281)) + ((x281 * x308)) + (((-1.0) * x300 * x329)) + ((x272 * x283)) + (((-1.0) * gconst51 * x274 * x312)) + ((x278 * x291)) + (((-1.0) * gconst33 * x271 * x303)) + ((x297 * x307)) + ((x270 * x304)) + (((-1.0) * gconst37 * x274 * x306)) + ((x296 * x305)) + (((-1.0) * gconst47 * x283 * x303)) + (((-1.0) * gconst46 * x294 * x295)) + (((-1.0) * x279 * x292)) + ((x276 * x285)) + ((x270 * x312)) + (((-1.0) * x270 * x279)) + (((-1.0) * x278 * x302)) + ((x288 * x297)) + (((-1.0) * x301 * x334)) + (((-1.0) * gconst48 * x269 * x270)) + ((x272 * x301)) + (((-1.0) * gconst41 * x269 * x276)));
            op[5] = (((x310 * x311)) + ((x291 * x296)) + ((x290 * x328)) + ((x275 * x294)) + (((-1.0) * gconst37 * x314)) + ((x308 * x326)) + ((x308 * x322)) + (((-1.0) * gconst42 * x323)) + ((x301 * x318)) + (((-1.0) * gconst37 * x285 * x295)) + (((-1.0) * gconst41 * x269 * x292)) + ((x283 * x313)) + ((x287 * x293)) + ((x280 * x300)) + (((-1.0) * x269 * x330)) + ((x309 * x315)) + (((-1.0) * gconst51 * x274 * x285)) + (((-1.0) * gconst34 * x269 * x276)) + (((-1.0) * gconst32 * x274 * x281)) + ((x297 * x321)) + ((x288 * x305)) + ((x309 * x327)) + (((-1.0) * gconst42 * x295 * x302)) + ((x305 * x307)) + (((-1.0) * x279 * x320)) + (((-1.0) * gconst41 * x286 * x290)) + ((x276 * x302)) + ((x279 * x321)) + (((-1.0) * gconst40 * x274 * x326)) + (((-1.0) * gconst42 * x274 * x312)) + ((x270 * x285)) + (((-1.0) * gconst41 * x269 * x270)));
            op[6] = ((((-1.0) * x300 * x334)) + ((x290 * x304)) + ((gconst30 * gconst50 * x277)) + (((-1.0) * gconst42 * x274 * x285)) + (((-1.0) * gconst46 * x274 * x294)) + ((gconst38 * gconst41 * x309)) + ((x305 * x321)) + ((gconst36 * gconst44 * x277)) + ((x308 * x317)) + ((x284 * x300)) + ((x283 * x299)) + (((-1.0) * gconst33 * x283 * x303)) + (((-1.0) * gconst51 * x274 * x302)) + (((-1.0) * x269 * x333)) + (((-1.0) * gconst48 * x269 * x290)) + (((-7.203856) * x277)) + (((-1.0) * x294 * x331)) + (((-1.0) * gconst34 * x269 * x270)) + ((x270 * x302)) + (((-1.0) * gconst37 * x274 * x304)) + (((-1.0) * gconst34 * x269 * x292)) + ((x293 * x297)) + (((-1.0) * x279 * x290)) + (((-1.0) * gconst47 * x300 * x303)) + ((x288 * x291)) + ((x272 * x300)) + (((-1.0) * gconst33 * x301 * x303)));
            op[7] = (((x291 * x321)) + ((x300 * x318)) + (((-1.0) * gconst37 * x274 * x285)) + (((-1.0) * gconst41 * x269 * x290)) + ((x294 * x308)) + (((-1.0) * gconst42 * x274 * x302)) + ((x309 * x310)) + ((x293 * x305)) + ((x285 * x290)) + (((-1.0) * gconst34 * x269 * x320)));
            op[8] = (((x290 * x302)) + (((-1.0) * gconst32 * x274 * x294)) + (((-1.0) * gconst33 * x300 * x303)) + (((-1.0) * gconst34 * x269 * x290)) + ((gconst30 * gconst36 * x277)));
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
                    j2eval[0] = ((((-1000.0) * sj0)) + (((-1.0) * cj0)));
                    j2eval[1] = IKsign(((((-450.240774879669) * sj0)) + (((-0.450240774879669) * cj0))));
                    if (IKabs(j2eval[0]) < 0.0000010000000000 || IKabs(j2eval[1]) < 0.0000010000000000)
                    {
                        {
                            IkReal j2eval[2];
                            j2eval[0] = ((((-1.0) * sj0)) + (((1000.0) * cj0)));
                            j2eval[1] = IKsign(((((-0.450240774879669) * sj0)) + (((450.240774879669) * cj0))));
                            if (IKabs(j2eval[0]) < 0.0000010000000000 || IKabs(j2eval[1]) < 0.0000010000000000)
                            {
                                {
                                    IkReal evalcond[1];
                                    bool bgotonextstatement = true;
                                    do
                                    {
                                        evalcond[0] = ((-3.14159265358979) + (IKfmod(((3.14159265358979) + (IKabs(((1.57179632646156) + j0)))), 6.28318530717959)));
                                        if (IKabs(evalcond[0]) < 0.0000050000000000)
                                        {
                                            bgotonextstatement = false;
                                            {
                                                IkReal j2array[1], cj2array[1], sj2array[1];
                                                bool j2valid[1] = {false};
                                                _nj2 = 1;
                                                IkReal x335 = (r21 * sj1);
                                                IkReal x336 = ((5.51415797317437e-7) * cj5);
                                                IkReal x337 = ((1.49031296572224) * px);
                                                IkReal x338 = ((0.169150521609538) * cj1);
                                                IkReal x339 = ((1.4903129657228) * pz);
                                                IkReal x340 = (cj5 * r20);
                                                IkReal x341 = (sj1 * sj5);
                                                IkReal x342 = ((5.5141579731723e-7) * r00);
                                                IkReal x343 = ((0.169150521609475) * r01);
                                                IkReal x344 = (cj1 * sj5);
                                                IkReal x345 = ((5.51415797317437e-7) * r20 * sj5);
                                                IkReal x346 = ((0.169150521609475) * cj5 * r00);
                                                IkReal x347 = ((5.5141579731723e-7) * cj5 * r01);
                                                if (IKabs(((((-1.0) * cj1 * x347)) + ((cj1 * x337)) + (((-5.51415797317437e-7) * r20 * x341)) + (((-0.169150521609538) * sj5 * x335)) + ((cj1 * x346)) + (((0.169150521609538) * sj1 * x340)) + (((5.36511918778309e-11) * cj1)) + (((-1.0) * x343 * x344)) + (((-1.0) * x342 * x344)) + (((-1.0) * x335 * x336)) + ((sj1 * x339)))) < IKFAST_ATAN2_MAGTHRESH && IKabs(((-1.02235469448584) + (((-1.0) * cj1 * r21 * x336)) + (((-1.0) * r21 * sj5 * x338)) + ((cj1 * x339)) + (((-5.51415797317437e-7) * r20 * x344)) + ((x341 * x342)) + ((x341 * x343)) + ((sj1 * x347)) + ((x338 * x340)) + (((-1.0) * sj1 * x346)) + (((-5.36511918778309e-11) * sj1)) + (((-1.0) * sj1 * x337)))) < IKFAST_ATAN2_MAGTHRESH && IKabs(IKsqr(((((-1.0) * cj1 * x347)) + ((cj1 * x337)) + (((-5.51415797317437e-7) * r20 * x341)) + (((-0.169150521609538) * sj5 * x335)) + ((cj1 * x346)) + (((0.169150521609538) * sj1 * x340)) + (((5.36511918778309e-11) * cj1)) + (((-1.0) * x343 * x344)) + (((-1.0) * x342 * x344)) + (((-1.0) * x335 * x336)) + ((sj1 * x339)))) + IKsqr(((-1.02235469448584) + (((-1.0) * cj1 * r21 * x336)) + (((-1.0) * r21 * sj5 * x338)) + ((cj1 * x339)) + (((-5.51415797317437e-7) * r20 * x344)) + ((x341 * x342)) + ((x341 * x343)) + ((sj1 * x347)) + ((x338 * x340)) + (((-1.0) * sj1 * x346)) + (((-5.36511918778309e-11) * sj1)) + (((-1.0) * sj1 * x337)))) - 1) <= IKFAST_SINCOS_THRESH)
                                                    continue;
                                                j2array[0] = IKatan2(((((-1.0) * cj1 * x347)) + ((cj1 * x337)) + (((-5.51415797317437e-7) * r20 * x341)) + (((-0.169150521609538) * sj5 * x335)) + ((cj1 * x346)) + (((0.169150521609538) * sj1 * x340)) + (((5.36511918778309e-11) * cj1)) + (((-1.0) * x343 * x344)) + (((-1.0) * x342 * x344)) + (((-1.0) * x335 * x336)) + ((sj1 * x339))), ((-1.02235469448584) + (((-1.0) * cj1 * r21 * x336)) + (((-1.0) * r21 * sj5 * x338)) + ((cj1 * x339)) + (((-5.51415797317437e-7) * r20 * x344)) + ((x341 * x342)) + ((x341 * x343)) + ((sj1 * x347)) + ((x338 * x340)) + (((-1.0) * sj1 * x346)) + (((-5.36511918778309e-11) * sj1)) + (((-1.0) * sj1 * x337))));
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
                                                        IkReal x348 = IKsin(j2);
                                                        IkReal x349 = IKcos(j2);
                                                        IkReal x350 = ((3.7e-7) * sj5);
                                                        IkReal x351 = ((3.7e-7) * cj5);
                                                        IkReal x352 = ((0.1135) * cj5);
                                                        IkReal x353 = ((0.1135) * sj5);
                                                        IkReal x354 = (cj1 * x348);
                                                        IkReal x355 = (sj1 * x349);
                                                        evalcond[0] = (((r21 * x353)) + ((r21 * x351)) + (((0.671) * cj1 * x349)) + (((-1.0) * pz)) + (((0.671) * sj1 * x348)) + (((-1.0) * r20 * x352)) + (((0.686) * cj1)) + ((r20 * x350)));
                                                        evalcond[1] = ((-3.59999497500381e-11) + ((r01 * x353)) + ((r01 * x351)) + ((r00 * x350)) + (((0.671000000000252) * x354)) + (((-1.0) * px)) + (((-0.671000000000252) * x355)) + (((-1.0) * r00 * x352)) + (((-0.686000000000257) * sj1)));
                                                        evalcond[2] = ((-0.132000001500057) + ((r11 * x351)) + ((r11 * x353)) + (((3.35499664500294e-10) * x355)) + (((-3.35499664500294e-10) * x354)) + (((-1.0) * py)) + (((3.429996570003e-10) * sj1)) + ((r10 * x350)) + (((-1.0) * r10 * x352)));
                                                        if (IKabs(evalcond[0]) > IKFAST_EVALCOND_THRESH || IKabs(evalcond[1]) > IKFAST_EVALCOND_THRESH || IKabs(evalcond[2]) > IKFAST_EVALCOND_THRESH)
                                                        {
                                                            continue;
                                                        }
                                                    }

                                                    {
                                                        IkReal j3eval[2];
                                                        sj0 = -0.99999950000025;
                                                        cj0 = -0.001;
                                                        j0 = -1.57179631563085;
                                                        IkReal x356 = (((cj5 * r11 * sj4)) + ((r10 * sj4 * sj5)) + (((-1.0) * cj4 * r12)));
                                                        j3eval[0] = x356;
                                                        j3eval[1] = IKsign(x356);
                                                        if (IKabs(j3eval[0]) < 0.0000010000000000 || IKabs(j3eval[1]) < 0.0000010000000000)
                                                        {
                                                            {
                                                                IkReal j3eval[2];
                                                                sj0 = -0.99999950000025;
                                                                cj0 = -0.001;
                                                                j0 = -1.57179631563085;
                                                                IkReal x357 = ((1.0) * sj4);
                                                                IkReal x358 = ((((-1.0) * r00 * sj5 * x357)) + (((-1.0) * cj5 * r01 * x357)) + ((cj4 * r02)));
                                                                j3eval[0] = x358;
                                                                j3eval[1] = IKsign(x358);
                                                                if (IKabs(j3eval[0]) < 0.0000010000000000 || IKabs(j3eval[1]) < 0.0000010000000000)
                                                                {
                                                                    {
                                                                        IkReal j3eval[2];
                                                                        sj0 = -0.99999950000025;
                                                                        cj0 = -0.001;
                                                                        j0 = -1.57179631563085;
                                                                        IkReal x359 = cj4 * cj4;
                                                                        IkReal x360 = cj5 * cj5;
                                                                        IkReal x361 = r22 * r22;
                                                                        IkReal x362 = r21 * r21;
                                                                        IkReal x363 = r20 * r20;
                                                                        IkReal x364 = (r20 * sj5);
                                                                        IkReal x365 = (cj5 * r21);
                                                                        IkReal x366 = ((1.0) * x362);
                                                                        IkReal x367 = ((1.0) * x363);
                                                                        IkReal x368 = (x359 * x360);
                                                                        IkReal x369 = ((2.0) * cj4 * r22 * sj4);
                                                                        IkReal x370 = ((((-1.0) * x361)) + (((-1.0) * x360 * x367)) + (((-1.0) * x366)) + ((x359 * x361)) + (((-1.0) * x366 * x368)) + (((-1.0) * x359 * x367)) + (((-1.0) * x364 * x369)) + (((-2.0) * x359 * x364 * x365)) + ((x360 * x362)) + (((-1.0) * x365 * x369)) + ((x363 * x368)) + (((2.0) * x364 * x365)));
                                                                        j3eval[0] = x370;
                                                                        j3eval[1] = IKsign(x370);
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
                                                                                IkReal x371 = cj4 * cj4;
                                                                                IkReal x372 = cj5 * cj5;
                                                                                IkReal x373 = r22 * r22;
                                                                                IkReal x374 = r21 * r21;
                                                                                IkReal x375 = r20 * r20;
                                                                                IkReal x376 = ((1.0) * sj1);
                                                                                IkReal x377 = (r22 * sj4);
                                                                                IkReal x378 = (sj2 * sj5);
                                                                                IkReal x379 = (cj4 * r20);
                                                                                IkReal x380 = (cj5 * sj2);
                                                                                IkReal x381 = (r21 * sj5);
                                                                                IkReal x382 = (cj4 * r21);
                                                                                IkReal x383 = ((2.0) * cj5);
                                                                                IkReal x384 = (cj2 * cj5 * r20);
                                                                                IkReal x385 = ((1.0) * x374);
                                                                                IkReal x386 = ((1.0) * cj1 * cj2);
                                                                                IkReal x387 = ((1.0) * x375);
                                                                                IkReal x388 = (x371 * x372);
                                                                                CheckValue<IkReal> x389 = IKPowWithIntegerCheck(IKsign(((((-1.0) * x373)) + (((-2.0) * sj5 * x377 * x379)) + ((x372 * x374)) + (((-1.0) * r20 * x371 * x381 * x383)) + (((-1.0) * x385)) + ((r20 * x381 * x383)) + (((-1.0) * x377 * x382 * x383)) + (((-1.0) * x385 * x388)) + (((-1.0) * x371 * x387)) + ((x375 * x388)) + (((-1.0) * x372 * x387)) + ((x371 * x373)))), -1);
                                                                                if (!x389.valid)
                                                                                {
                                                                                    continue;
                                                                                }
                                                                                CheckValue<IkReal> x390 = IKatan2WithCheck(IkReal(((((-1.0) * sj5 * x379 * x386)) + ((cj1 * r20 * x380)) + (((-1.0) * cj5 * x382 * x386)) + (((-1.0) * x376 * x378 * x379)) + (((-1.0) * x376 * x384)) + (((-1.0) * x376 * x380 * x382)) + (((-1.0) * cj1 * r21 * x378)) + ((cj2 * sj1 * x381)) + (((-1.0) * sj2 * x376 * x377)) + (((-1.0) * x377 * x386)))), IkReal((((cj1 * x384)) + (((-1.0) * x381 * x386)) + ((cj1 * x378 * x379)) + ((cj1 * x380 * x382)) + (((-1.0) * r21 * x376 * x378)) + (((-1.0) * cj2 * x376 * x377)) + ((cj1 * sj2 * x377)) + (((-1.0) * cj2 * cj5 * x376 * x382)) + ((r20 * sj1 * x380)) + (((-1.0) * cj2 * sj5 * x376 * x379)))), IKFAST_ATAN2_MAGTHRESH);
                                                                                if (!x390.valid)
                                                                                {
                                                                                    continue;
                                                                                }
                                                                                j3array[0] = ((-1.5707963267949) + (((1.5707963267949) * (x389.value))) + (x390.value));
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
                                                                                        IkReal x391 = IKcos(j3);
                                                                                        IkReal x392 = IKsin(j3);
                                                                                        IkReal x393 = (sj1 * sj2);
                                                                                        IkReal x394 = (r00 * sj5);
                                                                                        IkReal x395 = (r10 * sj5);
                                                                                        IkReal x396 = ((1.0) * r11);
                                                                                        IkReal x397 = (cj1 * cj2);
                                                                                        IkReal x398 = (cj2 * sj1);
                                                                                        IkReal x399 = (cj1 * sj2);
                                                                                        IkReal x400 = (cj5 * x391);
                                                                                        IkReal x401 = (sj4 * x391);
                                                                                        IkReal x402 = (cj5 * x392);
                                                                                        IkReal x403 = (sj5 * x391);
                                                                                        IkReal x404 = (cj4 * x391);
                                                                                        IkReal x405 = ((1.0) * cj4 * x392);
                                                                                        IkReal x406 = ((1.0) * sj4 * x392);
                                                                                        IkReal x407 = ((1.0) * sj5 * x392);
                                                                                        evalcond[0] = (((cj4 * r20 * x403)) + (((-1.0) * x398)) + ((cj4 * r21 * x400)) + ((r22 * x401)) + (((-1.0) * r21 * x407)) + x399 + ((r20 * x402)));
                                                                                        evalcond[1] = (x397 + x393 + (((-1.0) * r21 * x403)) + ((r20 * x400)) + (((-1.0) * cj4 * r21 * x402)) + (((-1.0) * r20 * sj5 * x405)) + (((-1.0) * r22 * x406)));
                                                                                        evalcond[2] = ((((-1.00000000000038) * x397)) + (((-1.00000000000038) * x393)) + ((cj4 * r01 * x400)) + ((r00 * x402)) + ((x394 * x404)) + (((-1.0) * r01 * x407)) + ((r02 * x401)));
                                                                                        evalcond[3] = ((((-1.0) * sj5 * x392 * x396)) + ((r12 * x401)) + ((r10 * x402)) + ((x395 * x404)) + (((4.99999500000438e-10) * x393)) + (((4.99999500000438e-10) * x397)) + ((cj4 * r11 * x400)));
                                                                                        evalcond[4] = ((((1.00000000000038) * x399)) + (((-1.00000000000038) * x398)) + (((-1.0) * r01 * x403)) + (((-1.0) * r02 * x406)) + (((-1.0) * x394 * x405)) + ((r00 * x400)) + (((-1.0) * cj4 * r01 * x402)));
                                                                                        evalcond[5] = ((((-1.0) * cj4 * x396 * x402)) + (((-1.0) * x395 * x405)) + ((r10 * x400)) + (((-4.99999500000438e-10) * x399)) + (((-1.0) * x396 * x403)) + (((4.99999500000438e-10) * x398)) + (((-1.0) * r12 * x406)));
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
                                                                        IkReal x408 = (r20 * sj5);
                                                                        IkReal x409 = (cj5 * r21);
                                                                        IkReal x410 = (cj5 * sj2);
                                                                        IkReal x411 = ((4.99999500000438e-10) * sj1);
                                                                        IkReal x412 = (cj1 * cj4);
                                                                        IkReal x413 = (r22 * sj4);
                                                                        IkReal x414 = ((4.99999500000438e-10) * cj2);
                                                                        IkReal x415 = (r21 * sj5);
                                                                        IkReal x416 = ((1.0) * sj4);
                                                                        IkReal x417 = (cj2 * sj1);
                                                                        IkReal x418 = (cj1 * sj2);
                                                                        IkReal x419 = ((1.0) * r10);
                                                                        IkReal x420 = (r11 * sj5);
                                                                        IkReal x421 = (cj4 * sj2 * x411);
                                                                        CheckValue<IkReal> x422 = IKatan2WithCheck(IkReal(((((-1.0) * x409 * x412 * x414)) + (((-1.0) * r12 * x416 * x417)) + ((r10 * sj2 * sj5 * x412)) + (((-1.0) * cj4 * cj5 * r11 * x417)) + (((-1.0) * cj1 * x413 * x414)) + (((-1.0) * sj2 * x411 * x413)) + (((-1.0) * x409 * x421)) + (((-1.0) * x408 * x421)) + ((r11 * x410 * x412)) + (((-1.0) * x408 * x412 * x414)) + ((r12 * sj4 * x418)) + (((-1.0) * cj4 * sj5 * x417 * x419)))), IkReal((((cj1 * cj5 * r20 * x414)) + ((r20 * x410 * x411)) + ((x418 * x420)) + (((-1.0) * cj1 * x410 * x419)) + (((-1.0) * cj1 * x414 * x415)) + (((-1.0) * x417 * x420)) + ((cj5 * r10 * x417)) + (((-1.0) * sj2 * x411 * x415)))), IKFAST_ATAN2_MAGTHRESH);
                                                                        if (!x422.valid)
                                                                        {
                                                                            continue;
                                                                        }
                                                                        CheckValue<IkReal> x423 = IKPowWithIntegerCheck(IKsign(((((-1.0) * cj5 * r01 * x416)) + (((-1.0) * r00 * sj5 * x416)) + ((cj4 * r02)))), -1);
                                                                        if (!x423.valid)
                                                                        {
                                                                            continue;
                                                                        }
                                                                        j3array[0] = ((-1.5707963267949) + (x422.value) + (((1.5707963267949) * (x423.value))));
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
                                                                                IkReal x424 = IKcos(j3);
                                                                                IkReal x425 = IKsin(j3);
                                                                                IkReal x426 = (sj1 * sj2);
                                                                                IkReal x427 = (r00 * sj5);
                                                                                IkReal x428 = (r10 * sj5);
                                                                                IkReal x429 = ((1.0) * r11);
                                                                                IkReal x430 = (cj1 * cj2);
                                                                                IkReal x431 = (cj2 * sj1);
                                                                                IkReal x432 = (cj1 * sj2);
                                                                                IkReal x433 = (cj5 * x424);
                                                                                IkReal x434 = (sj4 * x424);
                                                                                IkReal x435 = (cj5 * x425);
                                                                                IkReal x436 = (sj5 * x424);
                                                                                IkReal x437 = (cj4 * x424);
                                                                                IkReal x438 = ((1.0) * cj4 * x425);
                                                                                IkReal x439 = ((1.0) * sj4 * x425);
                                                                                IkReal x440 = ((1.0) * sj5 * x425);
                                                                                evalcond[0] = ((((-1.0) * x431)) + ((cj4 * r20 * x436)) + ((cj4 * r21 * x433)) + ((r22 * x434)) + x432 + (((-1.0) * r21 * x440)) + ((r20 * x435)));
                                                                                evalcond[1] = (x426 + x430 + (((-1.0) * r20 * sj5 * x438)) + (((-1.0) * cj4 * r21 * x435)) + (((-1.0) * r22 * x439)) + (((-1.0) * r21 * x436)) + ((r20 * x433)));
                                                                                evalcond[2] = (((r02 * x434)) + ((r00 * x435)) + (((-1.0) * r01 * x440)) + (((-1.00000000000038) * x426)) + (((-1.00000000000038) * x430)) + ((cj4 * r01 * x433)) + ((x427 * x437)));
                                                                                evalcond[3] = (((r12 * x434)) + ((x428 * x437)) + (((4.99999500000438e-10) * x430)) + (((4.99999500000438e-10) * x426)) + ((cj4 * r11 * x433)) + ((r10 * x435)) + (((-1.0) * sj5 * x425 * x429)));
                                                                                evalcond[4] = ((((-1.0) * x427 * x438)) + (((-1.0) * r01 * x436)) + ((r00 * x433)) + (((1.00000000000038) * x432)) + (((-1.0) * r02 * x439)) + (((-1.00000000000038) * x431)) + (((-1.0) * cj4 * r01 * x435)));
                                                                                evalcond[5] = ((((-1.0) * r12 * x439)) + (((4.99999500000438e-10) * x431)) + (((-1.0) * x429 * x436)) + (((-4.99999500000438e-10) * x432)) + ((r10 * x433)) + (((-1.0) * x428 * x438)) + (((-1.0) * cj4 * x429 * x435)));
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
                                                                IkReal x441 = (sj2 * sj5);
                                                                IkReal x442 = ((1.0) * r01);
                                                                IkReal x443 = (r02 * sj4);
                                                                IkReal x444 = (cj1 * cj2);
                                                                IkReal x445 = (cj4 * r21);
                                                                IkReal x446 = ((1.00000000000038) * cj1);
                                                                IkReal x447 = (sj1 * sj2);
                                                                IkReal x448 = (cj4 * sj1);
                                                                IkReal x449 = ((1.00000000000038) * cj2);
                                                                IkReal x450 = (r22 * sj4);
                                                                IkReal x451 = (cj5 * sj2);
                                                                IkReal x452 = (cj5 * r00);
                                                                IkReal x453 = (cj4 * cj5 * r01);
                                                                IkReal x454 = (cj5 * sj1 * x449);
                                                                CheckValue<IkReal> x455 = IKatan2WithCheck(IkReal(((((-1.0) * sj1 * x441 * x442)) + ((r20 * x454)) + (((-1.0) * r20 * x446 * x451)) + ((x447 * x452)) + (((-1.0) * sj5 * x442 * x444)) + ((r21 * x441 * x446)) + ((x444 * x452)) + (((-1.0) * r21 * sj1 * sj5 * x449)))), IkReal(((((-1.0) * x445 * x446 * x451)) + ((sj1 * x449 * x450)) + ((x447 * x453)) + (((-1.0) * cj4 * r20 * x441 * x446)) + (((-1.0) * sj2 * x446 * x450)) + ((r20 * sj5 * x448 * x449)) + ((x445 * x454)) + ((x443 * x444)) + ((x443 * x447)) + ((x444 * x453)) + ((r00 * x441 * x448)) + ((cj4 * r00 * sj5 * x444)))), IKFAST_ATAN2_MAGTHRESH);
                                                                if (!x455.valid)
                                                                {
                                                                    continue;
                                                                }
                                                                CheckValue<IkReal> x456 = IKPowWithIntegerCheck(IKsign((((cj5 * r11 * sj4)) + ((r10 * sj4 * sj5)) + (((-1.0) * cj4 * r12)))), -1);
                                                                if (!x456.valid)
                                                                {
                                                                    continue;
                                                                }
                                                                j3array[0] = ((-1.5707963267949) + (x455.value) + (((1.5707963267949) * (x456.value))));
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
                                                                        IkReal x457 = IKcos(j3);
                                                                        IkReal x458 = IKsin(j3);
                                                                        IkReal x459 = (sj1 * sj2);
                                                                        IkReal x460 = (r00 * sj5);
                                                                        IkReal x461 = (r10 * sj5);
                                                                        IkReal x462 = ((1.0) * r11);
                                                                        IkReal x463 = (cj1 * cj2);
                                                                        IkReal x464 = (cj2 * sj1);
                                                                        IkReal x465 = (cj1 * sj2);
                                                                        IkReal x466 = (cj5 * x457);
                                                                        IkReal x467 = (sj4 * x457);
                                                                        IkReal x468 = (cj5 * x458);
                                                                        IkReal x469 = (sj5 * x457);
                                                                        IkReal x470 = (cj4 * x457);
                                                                        IkReal x471 = ((1.0) * cj4 * x458);
                                                                        IkReal x472 = ((1.0) * sj4 * x458);
                                                                        IkReal x473 = ((1.0) * sj5 * x458);
                                                                        evalcond[0] = (((r22 * x467)) + (((-1.0) * r21 * x473)) + ((r20 * x468)) + (((-1.0) * x464)) + ((cj4 * r21 * x466)) + x465 + ((cj4 * r20 * x469)));
                                                                        evalcond[1] = (((r20 * x466)) + (((-1.0) * cj4 * r21 * x468)) + (((-1.0) * r21 * x469)) + (((-1.0) * r22 * x472)) + (((-1.0) * r20 * sj5 * x471)) + x459 + x463);
                                                                        evalcond[2] = ((((-1.00000000000038) * x463)) + ((cj4 * r01 * x466)) + (((-1.00000000000038) * x459)) + (((-1.0) * r01 * x473)) + ((x460 * x470)) + ((r02 * x467)) + ((r00 * x468)));
                                                                        evalcond[3] = ((((4.99999500000438e-10) * x459)) + (((4.99999500000438e-10) * x463)) + ((r12 * x467)) + ((cj4 * r11 * x466)) + ((x461 * x470)) + (((-1.0) * sj5 * x458 * x462)) + ((r10 * x468)));
                                                                        evalcond[4] = ((((-1.0) * x460 * x471)) + (((1.00000000000038) * x465)) + (((-1.00000000000038) * x464)) + (((-1.0) * r01 * x469)) + ((r00 * x466)) + (((-1.0) * r02 * x472)) + (((-1.0) * cj4 * r01 * x468)));
                                                                        evalcond[5] = ((((-1.0) * x462 * x469)) + (((-1.0) * cj4 * x462 * x468)) + (((4.99999500000438e-10) * x464)) + (((-1.0) * r12 * x472)) + (((-4.99999500000438e-10) * x465)) + (((-1.0) * x461 * x471)) + ((r10 * x466)));
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
                                            evalcond[0] = ((-3.14159265358979) + (IKfmod(((3.14159265358979) + (IKabs(((-1.56979632712823) + j0)))), 6.28318530717959)));
                                            if (IKabs(evalcond[0]) < 0.0000050000000000)
                                            {
                                                bgotonextstatement = false;
                                                {
                                                    IkReal j2array[1], cj2array[1], sj2array[1];
                                                    bool j2valid[1] = {false};
                                                    _nj2 = 1;
                                                    IkReal x474 = (r21 * sj1);
                                                    IkReal x475 = ((5.51415797317437e-7) * cj5);
                                                    IkReal x476 = ((1.49031296572224) * px);
                                                    IkReal x477 = ((0.169150521609538) * cj1);
                                                    IkReal x478 = ((1.4903129657228) * pz);
                                                    IkReal x479 = (cj5 * r20);
                                                    IkReal x480 = (sj1 * sj5);
                                                    IkReal x481 = ((5.5141579731723e-7) * r00);
                                                    IkReal x482 = ((0.169150521609475) * r01);
                                                    IkReal x483 = (cj1 * sj5);
                                                    IkReal x484 = ((5.51415797317437e-7) * r20 * sj5);
                                                    IkReal x485 = ((0.169150521609475) * cj5 * r00);
                                                    IkReal x486 = ((5.5141579731723e-7) * cj5 * r01);
                                                    if (IKabs(((((-1.0) * cj1 * x485)) + (((0.169150521609538) * sj1 * x479)) + (((-0.169150521609538) * sj5 * x474)) + ((x481 * x483)) + (((-1.0) * cj1 * x476)) + ((cj1 * x486)) + ((sj1 * x478)) + (((5.36511918778309e-11) * cj1)) + ((x482 * x483)) + (((-5.51415797317437e-7) * r20 * x480)) + (((-1.0) * x474 * x475)))) < IKFAST_ATAN2_MAGTHRESH && IKabs(((-1.02235469448584) + (((-1.0) * sj1 * x486)) + ((cj1 * x478)) + ((sj1 * x485)) + (((-1.0) * x480 * x481)) + (((-1.0) * x480 * x482)) + ((sj1 * x476)) + ((x477 * x479)) + (((-5.51415797317437e-7) * r20 * x483)) + (((-1.0) * r21 * sj5 * x477)) + (((-1.0) * cj1 * r21 * x475)) + (((-5.36511918778309e-11) * sj1)))) < IKFAST_ATAN2_MAGTHRESH && IKabs(IKsqr(((((-1.0) * cj1 * x485)) + (((0.169150521609538) * sj1 * x479)) + (((-0.169150521609538) * sj5 * x474)) + ((x481 * x483)) + (((-1.0) * cj1 * x476)) + ((cj1 * x486)) + ((sj1 * x478)) + (((5.36511918778309e-11) * cj1)) + ((x482 * x483)) + (((-5.51415797317437e-7) * r20 * x480)) + (((-1.0) * x474 * x475)))) + IKsqr(((-1.02235469448584) + (((-1.0) * sj1 * x486)) + ((cj1 * x478)) + ((sj1 * x485)) + (((-1.0) * x480 * x481)) + (((-1.0) * x480 * x482)) + ((sj1 * x476)) + ((x477 * x479)) + (((-5.51415797317437e-7) * r20 * x483)) + (((-1.0) * r21 * sj5 * x477)) + (((-1.0) * cj1 * r21 * x475)) + (((-5.36511918778309e-11) * sj1)))) - 1) <= IKFAST_SINCOS_THRESH)
                                                        continue;
                                                    j2array[0] = IKatan2(((((-1.0) * cj1 * x485)) + (((0.169150521609538) * sj1 * x479)) + (((-0.169150521609538) * sj5 * x474)) + ((x481 * x483)) + (((-1.0) * cj1 * x476)) + ((cj1 * x486)) + ((sj1 * x478)) + (((5.36511918778309e-11) * cj1)) + ((x482 * x483)) + (((-5.51415797317437e-7) * r20 * x480)) + (((-1.0) * x474 * x475))), ((-1.02235469448584) + (((-1.0) * sj1 * x486)) + ((cj1 * x478)) + ((sj1 * x485)) + (((-1.0) * x480 * x481)) + (((-1.0) * x480 * x482)) + ((sj1 * x476)) + ((x477 * x479)) + (((-5.51415797317437e-7) * r20 * x483)) + (((-1.0) * r21 * sj5 * x477)) + (((-1.0) * cj1 * r21 * x475)) + (((-5.36511918778309e-11) * sj1))));
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
                                                            IkReal x487 = IKcos(j2);
                                                            IkReal x488 = IKsin(j2);
                                                            IkReal x489 = ((3.7e-7) * cj5);
                                                            IkReal x490 = ((3.7e-7) * sj5);
                                                            IkReal x491 = ((0.1135) * cj5);
                                                            IkReal x492 = ((0.1135) * sj5);
                                                            IkReal x493 = (cj1 * x488);
                                                            IkReal x494 = (sj1 * x487);
                                                            evalcond[0] = ((((0.671) * sj1 * x488)) + (((0.671) * cj1 * x487)) + (((-1.0) * pz)) + ((r21 * x489)) + ((r21 * x492)) + (((0.686) * cj1)) + (((-1.0) * r20 * x491)) + ((r20 * x490)));
                                                            evalcond[1] = ((3.59999497500381e-11) + (((0.686000000000257) * sj1)) + (((-0.671000000000252) * x493)) + (((-1.0) * r00 * x491)) + (((0.671000000000252) * x494)) + (((-1.0) * px)) + ((r01 * x489)) + ((r00 * x490)) + ((r01 * x492)));
                                                            evalcond[2] = ((0.132000001500057) + (((-1.0) * r10 * x491)) + (((-3.429996570003e-10) * sj1)) + ((r11 * x492)) + (((-1.0) * py)) + ((r11 * x489)) + ((r10 * x490)) + (((3.35499664500294e-10) * x493)) + (((-3.35499664500294e-10) * x494)));
                                                            if (IKabs(evalcond[0]) > IKFAST_EVALCOND_THRESH || IKabs(evalcond[1]) > IKFAST_EVALCOND_THRESH || IKabs(evalcond[2]) > IKFAST_EVALCOND_THRESH)
                                                            {
                                                                continue;
                                                            }
                                                        }

                                                        {
                                                            IkReal j3eval[2];
                                                            sj0 = 0.99999950000025;
                                                            cj0 = 0.001;
                                                            j0 = 1.5697963151606;
                                                            IkReal x495 = (((cj5 * r11 * sj4)) + ((r10 * sj4 * sj5)) + (((-1.0) * cj4 * r12)));
                                                            j3eval[0] = x495;
                                                            j3eval[1] = IKsign(x495);
                                                            if (IKabs(j3eval[0]) < 0.0000010000000000 || IKabs(j3eval[1]) < 0.0000010000000000)
                                                            {
                                                                {
                                                                    IkReal j3eval[2];
                                                                    sj0 = 0.99999950000025;
                                                                    cj0 = 0.001;
                                                                    j0 = 1.5697963151606;
                                                                    IkReal x496 = ((1.0) * sj4);
                                                                    IkReal x497 = ((((-1.0) * r00 * sj5 * x496)) + (((-1.0) * cj5 * r01 * x496)) + ((cj4 * r02)));
                                                                    j3eval[0] = x497;
                                                                    j3eval[1] = IKsign(x497);
                                                                    if (IKabs(j3eval[0]) < 0.0000010000000000 || IKabs(j3eval[1]) < 0.0000010000000000)
                                                                    {
                                                                        {
                                                                            IkReal j3eval[2];
                                                                            sj0 = 0.99999950000025;
                                                                            cj0 = 0.001;
                                                                            j0 = 1.5697963151606;
                                                                            IkReal x498 = cj4 * cj4;
                                                                            IkReal x499 = cj5 * cj5;
                                                                            IkReal x500 = r22 * r22;
                                                                            IkReal x501 = r21 * r21;
                                                                            IkReal x502 = r20 * r20;
                                                                            IkReal x503 = (r20 * sj5);
                                                                            IkReal x504 = (cj5 * r21);
                                                                            IkReal x505 = ((1.0) * x501);
                                                                            IkReal x506 = ((1.0) * x502);
                                                                            IkReal x507 = (x498 * x499);
                                                                            IkReal x508 = ((2.0) * cj4 * r22 * sj4);
                                                                            IkReal x509 = ((((-1.0) * x505 * x507)) + (((-1.0) * x499 * x506)) + (((-1.0) * x505)) + (((-1.0) * x503 * x508)) + (((2.0) * x503 * x504)) + ((x498 * x500)) + ((x502 * x507)) + (((-1.0) * x504 * x508)) + (((-1.0) * x500)) + ((x499 * x501)) + (((-1.0) * x498 * x506)) + (((-2.0) * x498 * x503 * x504)));
                                                                            j3eval[0] = x509;
                                                                            j3eval[1] = IKsign(x509);
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
                                                                                    IkReal x510 = cj4 * cj4;
                                                                                    IkReal x511 = cj5 * cj5;
                                                                                    IkReal x512 = r22 * r22;
                                                                                    IkReal x513 = r21 * r21;
                                                                                    IkReal x514 = r20 * r20;
                                                                                    IkReal x515 = ((1.0) * sj1);
                                                                                    IkReal x516 = (r22 * sj4);
                                                                                    IkReal x517 = (sj2 * sj5);
                                                                                    IkReal x518 = (cj4 * r20);
                                                                                    IkReal x519 = (cj5 * sj2);
                                                                                    IkReal x520 = (r21 * sj5);
                                                                                    IkReal x521 = (cj4 * r21);
                                                                                    IkReal x522 = ((2.0) * cj5);
                                                                                    IkReal x523 = (cj2 * cj5 * r20);
                                                                                    IkReal x524 = ((1.0) * x513);
                                                                                    IkReal x525 = ((1.0) * cj1 * cj2);
                                                                                    IkReal x526 = ((1.0) * x514);
                                                                                    IkReal x527 = (x510 * x511);
                                                                                    CheckValue<IkReal> x528 = IKPowWithIntegerCheck(IKsign((((x511 * x513)) + (((-1.0) * x524 * x527)) + ((x510 * x512)) + (((-1.0) * x511 * x526)) + (((-2.0) * sj5 * x516 * x518)) + (((-1.0) * x512)) + ((r20 * x520 * x522)) + (((-1.0) * x524)) + (((-1.0) * r20 * x510 * x520 * x522)) + ((x514 * x527)) + (((-1.0) * x510 * x526)) + (((-1.0) * x516 * x521 * x522)))), -1);
                                                                                    if (!x528.valid)
                                                                                    {
                                                                                        continue;
                                                                                    }
                                                                                    CheckValue<IkReal> x529 = IKatan2WithCheck(IkReal(((((-1.0) * sj5 * x518 * x525)) + ((cj1 * r20 * x519)) + (((-1.0) * x515 * x517 * x518)) + (((-1.0) * cj1 * r21 * x517)) + (((-1.0) * cj5 * x521 * x525)) + (((-1.0) * x515 * x523)) + (((-1.0) * x516 * x525)) + (((-1.0) * sj2 * x515 * x516)) + ((cj2 * sj1 * x520)) + (((-1.0) * x515 * x519 * x521)))), IkReal(((((-1.0) * cj2 * x515 * x516)) + (((-1.0) * r21 * x515 * x517)) + (((-1.0) * x520 * x525)) + ((cj1 * x517 * x518)) + ((cj1 * x519 * x521)) + ((cj1 * sj2 * x516)) + ((r20 * sj1 * x519)) + (((-1.0) * cj2 * cj5 * x515 * x521)) + ((cj1 * x523)) + (((-1.0) * cj2 * sj5 * x515 * x518)))), IKFAST_ATAN2_MAGTHRESH);
                                                                                    if (!x529.valid)
                                                                                    {
                                                                                        continue;
                                                                                    }
                                                                                    j3array[0] = ((-1.5707963267949) + (((1.5707963267949) * (x528.value))) + (x529.value));
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
                                                                                            IkReal x530 = IKcos(j3);
                                                                                            IkReal x531 = IKsin(j3);
                                                                                            IkReal x532 = (sj1 * sj2);
                                                                                            IkReal x533 = (r00 * sj5);
                                                                                            IkReal x534 = (r10 * sj5);
                                                                                            IkReal x535 = ((1.0) * r11);
                                                                                            IkReal x536 = (cj1 * cj2);
                                                                                            IkReal x537 = (cj2 * sj1);
                                                                                            IkReal x538 = (cj1 * sj2);
                                                                                            IkReal x539 = (cj5 * x530);
                                                                                            IkReal x540 = (sj4 * x530);
                                                                                            IkReal x541 = (cj5 * x531);
                                                                                            IkReal x542 = (sj5 * x530);
                                                                                            IkReal x543 = (cj4 * x530);
                                                                                            IkReal x544 = ((1.0) * cj4 * x531);
                                                                                            IkReal x545 = ((1.0) * sj4 * x531);
                                                                                            IkReal x546 = ((1.0) * sj5 * x531);
                                                                                            evalcond[0] = (((r20 * x541)) + (((-1.0) * x537)) + (((-1.0) * r21 * x546)) + ((r22 * x540)) + x538 + ((cj4 * r20 * x542)) + ((cj4 * r21 * x539)));
                                                                                            evalcond[1] = ((((-1.0) * r22 * x545)) + ((r20 * x539)) + (((-1.0) * cj4 * r21 * x541)) + (((-1.0) * r21 * x542)) + (((-1.0) * r20 * sj5 * x544)) + x532 + x536);
                                                                                            evalcond[2] = (((r02 * x540)) + ((cj4 * r01 * x539)) + (((-1.0) * r01 * x546)) + (((1.00000000000038) * x536)) + (((1.00000000000038) * x532)) + ((x533 * x543)) + ((r00 * x541)));
                                                                                            evalcond[3] = ((((-1.0) * sj5 * x531 * x535)) + (((-4.99999500000438e-10) * x532)) + (((-4.99999500000438e-10) * x536)) + ((r12 * x540)) + ((cj4 * r11 * x539)) + ((r10 * x541)) + ((x534 * x543)));
                                                                                            evalcond[4] = ((((-1.0) * cj4 * r01 * x541)) + (((-1.0) * x533 * x544)) + (((1.00000000000038) * x537)) + (((-1.0) * r02 * x545)) + (((-1.0) * r01 * x542)) + (((-1.00000000000038) * x538)) + ((r00 * x539)));
                                                                                            evalcond[5] = ((((-4.99999500000438e-10) * x537)) + (((-1.0) * x534 * x544)) + (((-1.0) * cj4 * x535 * x541)) + (((4.99999500000438e-10) * x538)) + ((r10 * x539)) + (((-1.0) * x535 * x542)) + (((-1.0) * r12 * x545)));
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
                                                                            IkReal x547 = (r20 * sj5);
                                                                            IkReal x548 = (cj5 * r21);
                                                                            IkReal x549 = (cj5 * sj2);
                                                                            IkReal x550 = ((4.99999500000438e-10) * sj1);
                                                                            IkReal x551 = (cj1 * cj4);
                                                                            IkReal x552 = (sj2 * sj4);
                                                                            IkReal x553 = (r21 * sj5);
                                                                            IkReal x554 = ((1.0) * sj4);
                                                                            IkReal x555 = (cj2 * sj1);
                                                                            IkReal x556 = ((4.99999500000438e-10) * cj2);
                                                                            IkReal x557 = ((1.0) * r10);
                                                                            IkReal x558 = (r11 * sj5);
                                                                            IkReal x559 = (cj4 * sj2 * x550);
                                                                            CheckValue<IkReal> x560 = IKatan2WithCheck(IkReal((((x548 * x559)) + ((r10 * sj2 * sj5 * x551)) + ((r11 * x549 * x551)) + (((-1.0) * r12 * x554 * x555)) + (((-1.0) * cj4 * cj5 * r11 * x555)) + ((x547 * x559)) + ((r22 * x550 * x552)) + ((x547 * x551 * x556)) + ((cj1 * r22 * sj4 * x556)) + ((cj1 * r12 * x552)) + (((-1.0) * cj4 * sj5 * x555 * x557)) + ((x548 * x551 * x556)))), IkReal((((sj2 * x550 * x553)) + (((-1.0) * cj1 * x549 * x557)) + ((cj5 * r10 * x555)) + ((cj1 * x553 * x556)) + (((-1.0) * x555 * x558)) + (((-1.0) * r20 * x549 * x550)) + (((-1.0) * cj1 * cj5 * r20 * x556)) + ((cj1 * sj2 * x558)))), IKFAST_ATAN2_MAGTHRESH);
                                                                            if (!x560.valid)
                                                                            {
                                                                                continue;
                                                                            }
                                                                            CheckValue<IkReal> x561 = IKPowWithIntegerCheck(IKsign(((((-1.0) * cj5 * r01 * x554)) + (((-1.0) * r00 * sj5 * x554)) + ((cj4 * r02)))), -1);
                                                                            if (!x561.valid)
                                                                            {
                                                                                continue;
                                                                            }
                                                                            j3array[0] = ((-1.5707963267949) + (x560.value) + (((1.5707963267949) * (x561.value))));
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
                                                                                    IkReal x562 = IKcos(j3);
                                                                                    IkReal x563 = IKsin(j3);
                                                                                    IkReal x564 = (sj1 * sj2);
                                                                                    IkReal x565 = (r00 * sj5);
                                                                                    IkReal x566 = (r10 * sj5);
                                                                                    IkReal x567 = ((1.0) * r11);
                                                                                    IkReal x568 = (cj1 * cj2);
                                                                                    IkReal x569 = (cj2 * sj1);
                                                                                    IkReal x570 = (cj1 * sj2);
                                                                                    IkReal x571 = (cj5 * x562);
                                                                                    IkReal x572 = (sj4 * x562);
                                                                                    IkReal x573 = (cj5 * x563);
                                                                                    IkReal x574 = (sj5 * x562);
                                                                                    IkReal x575 = (cj4 * x562);
                                                                                    IkReal x576 = ((1.0) * cj4 * x563);
                                                                                    IkReal x577 = ((1.0) * sj4 * x563);
                                                                                    IkReal x578 = ((1.0) * sj5 * x563);
                                                                                    evalcond[0] = (((r20 * x573)) + x570 + ((cj4 * r20 * x574)) + (((-1.0) * x569)) + (((-1.0) * r21 * x578)) + ((r22 * x572)) + ((cj4 * r21 * x571)));
                                                                                    evalcond[1] = ((((-1.0) * cj4 * r21 * x573)) + ((r20 * x571)) + (((-1.0) * r20 * sj5 * x576)) + x568 + x564 + (((-1.0) * r21 * x574)) + (((-1.0) * r22 * x577)));
                                                                                    evalcond[2] = (((r00 * x573)) + (((-1.0) * r01 * x578)) + ((r02 * x572)) + ((x565 * x575)) + ((cj4 * r01 * x571)) + (((1.00000000000038) * x568)) + (((1.00000000000038) * x564)));
                                                                                    evalcond[3] = (((r12 * x572)) + ((cj4 * r11 * x571)) + (((-1.0) * sj5 * x563 * x567)) + ((x566 * x575)) + ((r10 * x573)) + (((-4.99999500000438e-10) * x564)) + (((-4.99999500000438e-10) * x568)));
                                                                                    evalcond[4] = (((r00 * x571)) + (((-1.0) * x565 * x576)) + (((-1.0) * cj4 * r01 * x573)) + (((-1.00000000000038) * x570)) + (((-1.0) * r02 * x577)) + (((1.00000000000038) * x569)) + (((-1.0) * r01 * x574)));
                                                                                    evalcond[5] = ((((-1.0) * x566 * x576)) + (((4.99999500000438e-10) * x570)) + (((-1.0) * r12 * x577)) + (((-1.0) * x567 * x574)) + (((-1.0) * cj4 * x567 * x573)) + ((r10 * x571)) + (((-4.99999500000438e-10) * x569)));
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
                                                                    IkReal x579 = (sj2 * sj5);
                                                                    IkReal x580 = ((1.0) * r01);
                                                                    IkReal x581 = (r02 * sj4);
                                                                    IkReal x582 = (cj1 * cj2);
                                                                    IkReal x583 = (cj4 * r21);
                                                                    IkReal x584 = ((1.00000000000038) * cj1);
                                                                    IkReal x585 = (sj1 * sj2);
                                                                    IkReal x586 = (cj4 * sj1);
                                                                    IkReal x587 = ((1.00000000000038) * cj2);
                                                                    IkReal x588 = (r22 * sj4);
                                                                    IkReal x589 = (cj5 * sj2);
                                                                    IkReal x590 = (cj5 * r00);
                                                                    IkReal x591 = (cj4 * cj5 * r01);
                                                                    IkReal x592 = (cj5 * sj1 * x587);
                                                                    CheckValue<IkReal> x593 = IKPowWithIntegerCheck(IKsign((((cj5 * r11 * sj4)) + ((r10 * sj4 * sj5)) + (((-1.0) * cj4 * r12)))), -1);
                                                                    if (!x593.valid)
                                                                    {
                                                                        continue;
                                                                    }
                                                                    CheckValue<IkReal> x594 = IKatan2WithCheck(IkReal((((x585 * x590)) + (((-1.0) * r21 * x579 * x584)) + ((r21 * sj1 * sj5 * x587)) + (((-1.0) * r20 * x592)) + (((-1.0) * sj1 * x579 * x580)) + (((-1.0) * sj5 * x580 * x582)) + ((r20 * x584 * x589)) + ((x582 * x590)))), IkReal((((x585 * x591)) + (((-1.0) * r20 * sj5 * x586 * x587)) + ((x581 * x585)) + ((x581 * x582)) + ((sj2 * x584 * x588)) + ((cj4 * r20 * x579 * x584)) + ((cj4 * r00 * sj5 * x582)) + ((x583 * x584 * x589)) + (((-1.0) * x583 * x592)) + ((x582 * x591)) + (((-1.0) * sj1 * x587 * x588)) + ((r00 * x579 * x586)))), IKFAST_ATAN2_MAGTHRESH);
                                                                    if (!x594.valid)
                                                                    {
                                                                        continue;
                                                                    }
                                                                    j3array[0] = ((-1.5707963267949) + (((1.5707963267949) * (x593.value))) + (x594.value));
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
                                                                            IkReal x595 = IKcos(j3);
                                                                            IkReal x596 = IKsin(j3);
                                                                            IkReal x597 = (sj1 * sj2);
                                                                            IkReal x598 = (r00 * sj5);
                                                                            IkReal x599 = (r10 * sj5);
                                                                            IkReal x600 = ((1.0) * r11);
                                                                            IkReal x601 = (cj1 * cj2);
                                                                            IkReal x602 = (cj2 * sj1);
                                                                            IkReal x603 = (cj1 * sj2);
                                                                            IkReal x604 = (cj5 * x595);
                                                                            IkReal x605 = (sj4 * x595);
                                                                            IkReal x606 = (cj5 * x596);
                                                                            IkReal x607 = (sj5 * x595);
                                                                            IkReal x608 = (cj4 * x595);
                                                                            IkReal x609 = ((1.0) * cj4 * x596);
                                                                            IkReal x610 = ((1.0) * sj4 * x596);
                                                                            IkReal x611 = ((1.0) * sj5 * x596);
                                                                            evalcond[0] = ((((-1.0) * r21 * x611)) + ((r20 * x606)) + (((-1.0) * x602)) + ((cj4 * r21 * x604)) + x603 + ((cj4 * r20 * x607)) + ((r22 * x605)));
                                                                            evalcond[1] = ((((-1.0) * r21 * x607)) + ((r20 * x604)) + (((-1.0) * r22 * x610)) + (((-1.0) * r20 * sj5 * x609)) + (((-1.0) * cj4 * r21 * x606)) + x597 + x601);
                                                                            evalcond[2] = (((r00 * x606)) + (((1.00000000000038) * x597)) + (((1.00000000000038) * x601)) + (((-1.0) * r01 * x611)) + ((cj4 * r01 * x604)) + ((r02 * x605)) + ((x598 * x608)));
                                                                            evalcond[3] = (((cj4 * r11 * x604)) + ((r12 * x605)) + (((-1.0) * sj5 * x596 * x600)) + (((-4.99999500000438e-10) * x601)) + (((-4.99999500000438e-10) * x597)) + ((x599 * x608)) + ((r10 * x606)));
                                                                            evalcond[4] = (((r00 * x604)) + (((-1.00000000000038) * x603)) + (((1.00000000000038) * x602)) + (((-1.0) * x598 * x609)) + (((-1.0) * cj4 * r01 * x606)) + (((-1.0) * r02 * x610)) + (((-1.0) * r01 * x607)));
                                                                            evalcond[5] = ((((-1.0) * r12 * x610)) + (((-4.99999500000438e-10) * x602)) + (((-1.0) * x599 * x609)) + (((-1.0) * x600 * x607)) + (((-1.0) * cj4 * x600 * x606)) + ((r10 * x604)) + (((4.99999500000438e-10) * x603)));
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
                                                evalcond[0] = ((-3.14159265358979) + (IKfmod(((3.14159265358979) + (IKabs(((-3.14059265392313) + j0)))), 6.28318530717959)));
                                                if (IKabs(evalcond[0]) < 0.0000050000000000)
                                                {
                                                    bgotonextstatement = false;
                                                    {
                                                        IkReal j2array[1], cj2array[1], sj2array[1];
                                                        bool j2valid[1] = {false};
                                                        _nj2 = 1;
                                                        IkReal x612 = (cj5 * sj1);
                                                        IkReal x613 = ((1102.83269746661) * r01);
                                                        IkReal x614 = ((2980628912.07191) * px);
                                                        IkReal x615 = ((5.51415797317437e-7) * r21);
                                                        IkReal x616 = ((0.169150521609538) * cj1);
                                                        IkReal x617 = (r21 * sj5);
                                                        IkReal x618 = (sj1 * sj5);
                                                        IkReal x619 = ((1102.83269746661) * r00);
                                                        IkReal x620 = ((338301381.520162) * r01);
                                                        IkReal x621 = ((1.4903129657228) * pz);
                                                        IkReal x622 = (cj1 * sj5);
                                                        IkReal x623 = (cj1 * cj5);
                                                        IkReal x624 = ((5.51415797317437e-7) * r20 * sj5);
                                                        IkReal x625 = ((338301381.520162) * cj5 * r00);
                                                        if (IKabs(((((-1.0) * x612 * x615)) + ((x619 * x622)) + (((-1.0) * cj1 * x614)) + (((0.169150521609538) * r20 * x612)) + ((x613 * x623)) + (((-338301381.520162) * r00 * x623)) + ((sj1 * x621)) + (((-5.51415797317437e-7) * r20 * x618)) + (((-0.169150521609538) * sj1 * x617)) + (((-393443020.864604) * cj1)) + ((x620 * x622)))) < IKFAST_ATAN2_MAGTHRESH && IKabs(((-1.02235469448584) + (((-1.0) * x612 * x613)) + ((cj5 * r20 * x616)) + (((338301381.520162) * r00 * x612)) + ((sj1 * x614)) + (((-1.0) * x618 * x619)) + (((-5.51415797317437e-7) * r20 * x622)) + (((-1.0) * x618 * x620)) + (((-1.0) * x615 * x623)) + (((-1.0) * x616 * x617)) + ((cj1 * x621)) + (((393443020.864604) * sj1)))) < IKFAST_ATAN2_MAGTHRESH && IKabs(IKsqr(((((-1.0) * x612 * x615)) + ((x619 * x622)) + (((-1.0) * cj1 * x614)) + (((0.169150521609538) * r20 * x612)) + ((x613 * x623)) + (((-338301381.520162) * r00 * x623)) + ((sj1 * x621)) + (((-5.51415797317437e-7) * r20 * x618)) + (((-0.169150521609538) * sj1 * x617)) + (((-393443020.864604) * cj1)) + ((x620 * x622)))) + IKsqr(((-1.02235469448584) + (((-1.0) * x612 * x613)) + ((cj5 * r20 * x616)) + (((338301381.520162) * r00 * x612)) + ((sj1 * x614)) + (((-1.0) * x618 * x619)) + (((-5.51415797317437e-7) * r20 * x622)) + (((-1.0) * x618 * x620)) + (((-1.0) * x615 * x623)) + (((-1.0) * x616 * x617)) + ((cj1 * x621)) + (((393443020.864604) * sj1)))) - 1) <= IKFAST_SINCOS_THRESH)
                                                            continue;
                                                        j2array[0] = IKatan2(((((-1.0) * x612 * x615)) + ((x619 * x622)) + (((-1.0) * cj1 * x614)) + (((0.169150521609538) * r20 * x612)) + ((x613 * x623)) + (((-338301381.520162) * r00 * x623)) + ((sj1 * x621)) + (((-5.51415797317437e-7) * r20 * x618)) + (((-0.169150521609538) * sj1 * x617)) + (((-393443020.864604) * cj1)) + ((x620 * x622))), ((-1.02235469448584) + (((-1.0) * x612 * x613)) + ((cj5 * r20 * x616)) + (((338301381.520162) * r00 * x612)) + ((sj1 * x614)) + (((-1.0) * x618 * x619)) + (((-5.51415797317437e-7) * r20 * x622)) + (((-1.0) * x618 * x620)) + (((-1.0) * x615 * x623)) + (((-1.0) * x616 * x617)) + ((cj1 * x621)) + (((393443020.864604) * sj1))));
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
                                                                IkReal x626 = IKcos(j2);
                                                                IkReal x627 = IKsin(j2);
                                                                IkReal x628 = ((3.7e-7) * sj5);
                                                                IkReal x629 = ((3.7e-7) * cj5);
                                                                IkReal x630 = ((0.1135) * cj5);
                                                                IkReal x631 = ((0.1135) * sj5);
                                                                IkReal x632 = (cj1 * x627);
                                                                IkReal x633 = (sj1 * x626);
                                                                evalcond[0] = (((r21 * x629)) + ((r21 * x631)) + (((0.671) * cj1 * x626)) + (((-1.0) * pz)) + (((-1.0) * r20 * x630)) + (((0.686) * cj1)) + ((r20 * x628)) + (((0.671) * sj1 * x627)));
                                                                evalcond[1] = ((-0.132000001500057) + (((3.35499664500294e-10) * x633)) + (((-1.0) * r00 * x630)) + (((-3.35499664500294e-10) * x632)) + (((-1.0) * px)) + ((r01 * x631)) + (((3.429996570003e-10) * sj1)) + ((r01 * x629)) + ((r00 * x628)));
                                                                evalcond[2] = ((3.59999497500381e-11) + (((0.686000000000257) * sj1)) + (((-0.671000000000252) * x632)) + ((r10 * x628)) + (((-1.0) * py)) + (((0.671000000000252) * x633)) + (((-1.0) * r10 * x630)) + ((r11 * x631)) + ((r11 * x629)));
                                                                if (IKabs(evalcond[0]) > IKFAST_EVALCOND_THRESH || IKabs(evalcond[1]) > IKFAST_EVALCOND_THRESH || IKabs(evalcond[2]) > IKFAST_EVALCOND_THRESH)
                                                                {
                                                                    continue;
                                                                }
                                                            }

                                                            {
                                                                IkReal j3eval[2];
                                                                sj0 = 0.001;
                                                                cj0 = -0.99999950000025;
                                                                j0 = 3.14059264993284;
                                                                IkReal x634 = (((cj5 * r11 * sj4)) + ((r10 * sj4 * sj5)) + (((-1.0) * cj4 * r12)));
                                                                j3eval[0] = x634;
                                                                j3eval[1] = IKsign(x634);
                                                                if (IKabs(j3eval[0]) < 0.0000010000000000 || IKabs(j3eval[1]) < 0.0000010000000000)
                                                                {
                                                                    {
                                                                        IkReal j3eval[2];
                                                                        sj0 = 0.001;
                                                                        cj0 = -0.99999950000025;
                                                                        j0 = 3.14059264993284;
                                                                        IkReal x635 = ((1.0) * sj4);
                                                                        IkReal x636 = ((((-1.0) * cj5 * r01 * x635)) + (((-1.0) * r00 * sj5 * x635)) + ((cj4 * r02)));
                                                                        j3eval[0] = x636;
                                                                        j3eval[1] = IKsign(x636);
                                                                        if (IKabs(j3eval[0]) < 0.0000010000000000 || IKabs(j3eval[1]) < 0.0000010000000000)
                                                                        {
                                                                            {
                                                                                IkReal j3eval[2];
                                                                                sj0 = 0.001;
                                                                                cj0 = -0.99999950000025;
                                                                                j0 = 3.14059264993284;
                                                                                IkReal x637 = cj4 * cj4;
                                                                                IkReal x638 = cj5 * cj5;
                                                                                IkReal x639 = r22 * r22;
                                                                                IkReal x640 = r21 * r21;
                                                                                IkReal x641 = r20 * r20;
                                                                                IkReal x642 = (r20 * sj5);
                                                                                IkReal x643 = (cj5 * r21);
                                                                                IkReal x644 = ((1.0) * x640);
                                                                                IkReal x645 = ((1.0) * x641);
                                                                                IkReal x646 = (x637 * x638);
                                                                                IkReal x647 = ((2.0) * cj4 * r22 * sj4);
                                                                                IkReal x648 = ((((-1.0) * x642 * x647)) + ((x638 * x640)) + (((2.0) * x642 * x643)) + ((x641 * x646)) + (((-1.0) * x644)) + (((-1.0) * x644 * x646)) + (((-1.0) * x643 * x647)) + (((-1.0) * x637 * x645)) + (((-1.0) * x639)) + (((-2.0) * x637 * x642 * x643)) + (((-1.0) * x638 * x645)) + ((x637 * x639)));
                                                                                j3eval[0] = x648;
                                                                                j3eval[1] = IKsign(x648);
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
                                                                                        IkReal x649 = cj4 * cj4;
                                                                                        IkReal x650 = cj5 * cj5;
                                                                                        IkReal x651 = r22 * r22;
                                                                                        IkReal x652 = r21 * r21;
                                                                                        IkReal x653 = r20 * r20;
                                                                                        IkReal x654 = ((1.0) * sj1);
                                                                                        IkReal x655 = (r22 * sj4);
                                                                                        IkReal x656 = (sj2 * sj5);
                                                                                        IkReal x657 = (cj4 * r20);
                                                                                        IkReal x658 = (cj5 * sj2);
                                                                                        IkReal x659 = (r21 * sj5);
                                                                                        IkReal x660 = (cj4 * r21);
                                                                                        IkReal x661 = ((2.0) * cj5);
                                                                                        IkReal x662 = (cj2 * cj5 * r20);
                                                                                        IkReal x663 = ((1.0) * x652);
                                                                                        IkReal x664 = ((1.0) * cj1 * cj2);
                                                                                        IkReal x665 = ((1.0) * x653);
                                                                                        IkReal x666 = (x649 * x650);
                                                                                        CheckValue<IkReal> x667 = IKPowWithIntegerCheck(IKsign(((((-2.0) * sj5 * x655 * x657)) + (((-1.0) * x649 * x665)) + (((-1.0) * x655 * x660 * x661)) + (((-1.0) * x663 * x666)) + (((-1.0) * x651)) + ((r20 * x659 * x661)) + (((-1.0) * r20 * x649 * x659 * x661)) + ((x649 * x651)) + (((-1.0) * x663)) + ((x653 * x666)) + (((-1.0) * x650 * x665)) + ((x650 * x652)))), -1);
                                                                                        if (!x667.valid)
                                                                                        {
                                                                                            continue;
                                                                                        }
                                                                                        CheckValue<IkReal> x668 = IKatan2WithCheck(IkReal(((((-1.0) * x654 * x662)) + ((cj2 * sj1 * x659)) + (((-1.0) * sj5 * x657 * x664)) + (((-1.0) * cj5 * x660 * x664)) + (((-1.0) * x654 * x656 * x657)) + (((-1.0) * sj2 * x654 * x655)) + (((-1.0) * x655 * x664)) + ((cj1 * r20 * x658)) + (((-1.0) * x654 * x658 * x660)) + (((-1.0) * cj1 * r21 * x656)))), IkReal(((((-1.0) * cj2 * sj5 * x654 * x657)) + (((-1.0) * cj2 * x654 * x655)) + ((r20 * sj1 * x658)) + (((-1.0) * x659 * x664)) + ((cj1 * sj2 * x655)) + (((-1.0) * cj2 * cj5 * x654 * x660)) + ((cj1 * x656 * x657)) + (((-1.0) * r21 * x654 * x656)) + ((cj1 * x658 * x660)) + ((cj1 * x662)))), IKFAST_ATAN2_MAGTHRESH);
                                                                                        if (!x668.valid)
                                                                                        {
                                                                                            continue;
                                                                                        }
                                                                                        j3array[0] = ((-1.5707963267949) + (((1.5707963267949) * (x667.value))) + (x668.value));
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
                                                                                                IkReal x669 = IKcos(j3);
                                                                                                IkReal x670 = IKsin(j3);
                                                                                                IkReal x671 = (sj1 * sj2);
                                                                                                IkReal x672 = (r00 * sj5);
                                                                                                IkReal x673 = (r10 * sj5);
                                                                                                IkReal x674 = ((1.0) * r11);
                                                                                                IkReal x675 = (cj1 * cj2);
                                                                                                IkReal x676 = (cj2 * sj1);
                                                                                                IkReal x677 = (cj1 * sj2);
                                                                                                IkReal x678 = (cj5 * x669);
                                                                                                IkReal x679 = (sj4 * x669);
                                                                                                IkReal x680 = (cj5 * x670);
                                                                                                IkReal x681 = (sj5 * x669);
                                                                                                IkReal x682 = (cj4 * x669);
                                                                                                IkReal x683 = ((1.0) * cj4 * x670);
                                                                                                IkReal x684 = ((1.0) * sj4 * x670);
                                                                                                IkReal x685 = ((1.0) * sj5 * x670);
                                                                                                evalcond[0] = (((r22 * x679)) + (((-1.0) * r21 * x685)) + x677 + (((-1.0) * x676)) + ((r20 * x680)) + ((cj4 * r21 * x678)) + ((cj4 * r20 * x681)));
                                                                                                evalcond[1] = ((((-1.0) * r21 * x681)) + (((-1.0) * cj4 * r21 * x680)) + ((r20 * x678)) + (((-1.0) * r20 * sj5 * x683)) + x671 + x675 + (((-1.0) * r22 * x684)));
                                                                                                evalcond[2] = (((r02 * x679)) + (((-1.0) * r01 * x685)) + ((x672 * x682)) + (((4.99999500000438e-10) * x675)) + (((4.99999500000438e-10) * x671)) + ((r00 * x680)) + ((cj4 * r01 * x678)));
                                                                                                evalcond[3] = (((r12 * x679)) + (((-1.0) * sj5 * x670 * x674)) + ((r10 * x680)) + ((x673 * x682)) + (((1.00000000000038) * x675)) + (((1.00000000000038) * x671)) + ((cj4 * r11 * x678)));
                                                                                                evalcond[4] = ((((-4.99999500000438e-10) * x677)) + ((r00 * x678)) + (((-1.0) * r02 * x684)) + (((-1.0) * cj4 * r01 * x680)) + (((-1.0) * r01 * x681)) + (((4.99999500000438e-10) * x676)) + (((-1.0) * x672 * x683)));
                                                                                                evalcond[5] = ((((-1.0) * r12 * x684)) + ((r10 * x678)) + (((-1.0) * cj4 * x674 * x680)) + (((-1.0) * x673 * x683)) + (((-1.0) * x674 * x681)) + (((-1.00000000000038) * x677)) + (((1.00000000000038) * x676)));
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
                                                                                IkReal x686 = (cj4 * sj2);
                                                                                IkReal x687 = (cj1 * cj5);
                                                                                IkReal x688 = ((1.00000000000038) * sj1);
                                                                                IkReal x689 = (r20 * sj5);
                                                                                IkReal x690 = (sj2 * sj4);
                                                                                IkReal x691 = ((1.00000000000038) * cj2);
                                                                                IkReal x692 = ((1.0) * sj4);
                                                                                IkReal x693 = (r21 * sj5);
                                                                                IkReal x694 = (cj2 * sj1);
                                                                                IkReal x695 = ((1.0) * cj4);
                                                                                IkReal x696 = (r10 * sj5);
                                                                                IkReal x697 = (r11 * sj5);
                                                                                CheckValue<IkReal> x698 = IKPowWithIntegerCheck(IKsign(((((-1.0) * cj5 * r01 * x692)) + (((-1.0) * r00 * sj5 * x692)) + ((cj4 * r02)))), -1);
                                                                                if (!x698.valid)
                                                                                {
                                                                                    continue;
                                                                                }
                                                                                CheckValue<IkReal> x699 = IKatan2WithCheck(IkReal(((((-1.0) * x694 * x695 * x696)) + (((-1.0) * x686 * x688 * x689)) + (((-1.0) * cj5 * r21 * x686 * x688)) + (((-1.0) * cj1 * r22 * sj4 * x691)) + (((-1.0) * cj1 * cj4 * x689 * x691)) + ((cj1 * x686 * x696)) + (((-1.0) * r12 * x692 * x694)) + ((cj1 * r12 * x690)) + (((-1.0) * cj5 * r11 * x694 * x695)) + (((-1.0) * r22 * x688 * x690)) + ((r11 * x686 * x687)) + (((-1.0) * cj4 * r21 * x687 * x691)))), IkReal(((((-1.0) * r10 * sj2 * x687)) + ((cj5 * r10 * x694)) + (((-1.0) * x694 * x697)) + (((-1.0) * sj2 * x688 * x693)) + ((r20 * x687 * x691)) + ((cj5 * r20 * sj2 * x688)) + (((-1.0) * cj1 * x691 * x693)) + ((cj1 * sj2 * x697)))), IKFAST_ATAN2_MAGTHRESH);
                                                                                if (!x699.valid)
                                                                                {
                                                                                    continue;
                                                                                }
                                                                                j3array[0] = ((-1.5707963267949) + (((1.5707963267949) * (x698.value))) + (x699.value));
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
                                                                                        IkReal x700 = IKcos(j3);
                                                                                        IkReal x701 = IKsin(j3);
                                                                                        IkReal x702 = (sj1 * sj2);
                                                                                        IkReal x703 = (r00 * sj5);
                                                                                        IkReal x704 = (r10 * sj5);
                                                                                        IkReal x705 = ((1.0) * r11);
                                                                                        IkReal x706 = (cj1 * cj2);
                                                                                        IkReal x707 = (cj2 * sj1);
                                                                                        IkReal x708 = (cj1 * sj2);
                                                                                        IkReal x709 = (cj5 * x700);
                                                                                        IkReal x710 = (sj4 * x700);
                                                                                        IkReal x711 = (cj5 * x701);
                                                                                        IkReal x712 = (sj5 * x700);
                                                                                        IkReal x713 = (cj4 * x700);
                                                                                        IkReal x714 = ((1.0) * cj4 * x701);
                                                                                        IkReal x715 = ((1.0) * sj4 * x701);
                                                                                        IkReal x716 = ((1.0) * sj5 * x701);
                                                                                        evalcond[0] = (((r22 * x710)) + (((-1.0) * x707)) + (((-1.0) * r21 * x716)) + ((r20 * x711)) + x708 + ((cj4 * r21 * x709)) + ((cj4 * r20 * x712)));
                                                                                        evalcond[1] = ((((-1.0) * r22 * x715)) + (((-1.0) * r21 * x712)) + ((r20 * x709)) + x702 + x706 + (((-1.0) * r20 * sj5 * x714)) + (((-1.0) * cj4 * r21 * x711)));
                                                                                        evalcond[2] = ((((4.99999500000438e-10) * x702)) + (((4.99999500000438e-10) * x706)) + ((r00 * x711)) + ((x703 * x713)) + (((-1.0) * r01 * x716)) + ((r02 * x710)) + ((cj4 * r01 * x709)));
                                                                                        evalcond[3] = (((cj4 * r11 * x709)) + ((x704 * x713)) + (((-1.0) * sj5 * x701 * x705)) + ((r12 * x710)) + ((r10 * x711)) + (((1.00000000000038) * x706)) + (((1.00000000000038) * x702)));
                                                                                        evalcond[4] = ((((-1.0) * r02 * x715)) + (((-1.0) * cj4 * r01 * x711)) + (((4.99999500000438e-10) * x707)) + ((r00 * x709)) + (((-1.0) * x703 * x714)) + (((-1.0) * r01 * x712)) + (((-4.99999500000438e-10) * x708)));
                                                                                        evalcond[5] = ((((-1.0) * r12 * x715)) + (((-1.0) * x704 * x714)) + (((-1.00000000000038) * x708)) + (((-1.0) * x705 * x712)) + (((-1.0) * cj4 * x705 * x711)) + ((r10 * x709)) + (((1.00000000000038) * x707)));
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
                                                                        IkReal x717 = (sj1 * sj5);
                                                                        IkReal x718 = ((1.0) * r01);
                                                                        IkReal x719 = (r02 * sj4);
                                                                        IkReal x720 = (cj1 * cj2);
                                                                        IkReal x721 = (cj1 * sj2);
                                                                        IkReal x722 = (sj1 * sj2);
                                                                        IkReal x723 = (cj4 * r00);
                                                                        IkReal x724 = ((4.99999500000438e-10) * cj5);
                                                                        IkReal x725 = (cj2 * r21);
                                                                        IkReal x726 = (cj2 * sj1);
                                                                        IkReal x727 = (cj5 * r00);
                                                                        IkReal x728 = ((4.99999500000438e-10) * cj4 * r20);
                                                                        IkReal x729 = ((4.99999500000438e-10) * r22 * sj4);
                                                                        IkReal x730 = (cj4 * cj5 * r01);
                                                                        CheckValue<IkReal> x731 = IKatan2WithCheck(IkReal((((x720 * x727)) + (((-1.0) * r20 * x724 * x726)) + ((r20 * x721 * x724)) + (((-1.0) * sj5 * x718 * x720)) + ((x722 * x727)) + (((-4.99999500000438e-10) * r21 * sj5 * x721)) + (((-1.0) * sj2 * x717 * x718)) + (((4.99999500000438e-10) * x717 * x725)))), IkReal((((x721 * x729)) + ((x720 * x730)) + (((-1.0) * cj2 * x717 * x728)) + ((sj2 * x717 * x723)) + ((sj5 * x721 * x728)) + ((x722 * x730)) + ((sj5 * x720 * x723)) + (((-1.0) * cj4 * sj1 * x724 * x725)) + ((x719 * x722)) + ((x719 * x720)) + (((-1.0) * x726 * x729)) + ((cj4 * r21 * x721 * x724)))), IKFAST_ATAN2_MAGTHRESH);
                                                                        if (!x731.valid)
                                                                        {
                                                                            continue;
                                                                        }
                                                                        CheckValue<IkReal> x732 = IKPowWithIntegerCheck(IKsign((((cj5 * r11 * sj4)) + ((r10 * sj4 * sj5)) + (((-1.0) * cj4 * r12)))), -1);
                                                                        if (!x732.valid)
                                                                        {
                                                                            continue;
                                                                        }
                                                                        j3array[0] = ((-1.5707963267949) + (x731.value) + (((1.5707963267949) * (x732.value))));
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
                                                                                IkReal x733 = IKcos(j3);
                                                                                IkReal x734 = IKsin(j3);
                                                                                IkReal x735 = (sj1 * sj2);
                                                                                IkReal x736 = (r00 * sj5);
                                                                                IkReal x737 = (r10 * sj5);
                                                                                IkReal x738 = ((1.0) * r11);
                                                                                IkReal x739 = (cj1 * cj2);
                                                                                IkReal x740 = (cj2 * sj1);
                                                                                IkReal x741 = (cj1 * sj2);
                                                                                IkReal x742 = (cj5 * x733);
                                                                                IkReal x743 = (sj4 * x733);
                                                                                IkReal x744 = (cj5 * x734);
                                                                                IkReal x745 = (sj5 * x733);
                                                                                IkReal x746 = (cj4 * x733);
                                                                                IkReal x747 = ((1.0) * cj4 * x734);
                                                                                IkReal x748 = ((1.0) * sj4 * x734);
                                                                                IkReal x749 = ((1.0) * sj5 * x734);
                                                                                evalcond[0] = ((((-1.0) * r21 * x749)) + ((r22 * x743)) + ((cj4 * r20 * x745)) + ((cj4 * r21 * x742)) + ((r20 * x744)) + (((-1.0) * x740)) + x741);
                                                                                evalcond[1] = ((((-1.0) * r21 * x745)) + (((-1.0) * r22 * x748)) + ((r20 * x742)) + x735 + x739 + (((-1.0) * cj4 * r21 * x744)) + (((-1.0) * r20 * sj5 * x747)));
                                                                                evalcond[2] = (((cj4 * r01 * x742)) + ((r02 * x743)) + ((r00 * x744)) + (((-1.0) * r01 * x749)) + (((4.99999500000438e-10) * x735)) + (((4.99999500000438e-10) * x739)) + ((x736 * x746)));
                                                                                evalcond[3] = ((((1.00000000000038) * x739)) + (((1.00000000000038) * x735)) + ((r10 * x744)) + (((-1.0) * sj5 * x734 * x738)) + ((x737 * x746)) + ((r12 * x743)) + ((cj4 * r11 * x742)));
                                                                                evalcond[4] = ((((-4.99999500000438e-10) * x741)) + (((-1.0) * r01 * x745)) + (((-1.0) * r02 * x748)) + (((4.99999500000438e-10) * x740)) + ((r00 * x742)) + (((-1.0) * cj4 * r01 * x744)) + (((-1.0) * x736 * x747)));
                                                                                evalcond[5] = ((((-1.0) * cj4 * x738 * x744)) + (((-1.0) * x737 * x747)) + (((-1.0) * r12 * x748)) + ((r10 * x742)) + (((-1.0) * x738 * x745)) + (((-1.00000000000038) * x741)) + (((1.00000000000038) * x740)));
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
                                                    evalcond[0] = ((-3.14159265358979) + (IKfmod(((3.14159265358979) + (IKabs(((-6.28218530751292) + j0)))), 6.28318530717959)));
                                                    if (IKabs(evalcond[0]) < 0.0000050000000000)
                                                    {
                                                        bgotonextstatement = false;
                                                        {
                                                            IkReal j2array[1], cj2array[1], sj2array[1];
                                                            bool j2valid[1] = {false};
                                                            _nj2 = 1;
                                                            IkReal x750 = (cj5 * sj1);
                                                            IkReal x751 = ((1102.83269746661) * r01);
                                                            IkReal x752 = ((2980628912.07191) * px);
                                                            IkReal x753 = ((5.51415797317437e-7) * r21);
                                                            IkReal x754 = ((0.169150521609538) * cj1);
                                                            IkReal x755 = (r21 * sj5);
                                                            IkReal x756 = (sj1 * sj5);
                                                            IkReal x757 = ((1102.83269746661) * r00);
                                                            IkReal x758 = ((338301381.520162) * r01);
                                                            IkReal x759 = ((1.4903129657228) * pz);
                                                            IkReal x760 = (cj1 * sj5);
                                                            IkReal x761 = (cj1 * cj5);
                                                            IkReal x762 = ((5.51415797317437e-7) * r20 * sj5);
                                                            IkReal x763 = ((338301381.520162) * cj5 * r00);
                                                            if (IKabs(((((-1.0) * x758 * x760)) + (((338301381.520162) * r00 * x761)) + ((cj1 * x752)) + (((-5.51415797317437e-7) * r20 * x756)) + (((-1.0) * x750 * x753)) + (((-1.0) * x751 * x761)) + ((sj1 * x759)) + (((-1.0) * x757 * x760)) + (((-0.169150521609538) * sj1 * x755)) + (((-393443020.864604) * cj1)) + (((0.169150521609538) * r20 * x750)))) < IKFAST_ATAN2_MAGTHRESH && IKabs(((-1.02235469448584) + ((x756 * x757)) + ((x756 * x758)) + (((-1.0) * sj1 * x752)) + ((cj1 * x759)) + (((-1.0) * x754 * x755)) + (((-1.0) * x753 * x761)) + ((cj5 * r20 * x754)) + (((-338301381.520162) * r00 * x750)) + (((-5.51415797317437e-7) * r20 * x760)) + ((x750 * x751)) + (((393443020.864604) * sj1)))) < IKFAST_ATAN2_MAGTHRESH && IKabs(IKsqr(((((-1.0) * x758 * x760)) + (((338301381.520162) * r00 * x761)) + ((cj1 * x752)) + (((-5.51415797317437e-7) * r20 * x756)) + (((-1.0) * x750 * x753)) + (((-1.0) * x751 * x761)) + ((sj1 * x759)) + (((-1.0) * x757 * x760)) + (((-0.169150521609538) * sj1 * x755)) + (((-393443020.864604) * cj1)) + (((0.169150521609538) * r20 * x750)))) + IKsqr(((-1.02235469448584) + ((x756 * x757)) + ((x756 * x758)) + (((-1.0) * sj1 * x752)) + ((cj1 * x759)) + (((-1.0) * x754 * x755)) + (((-1.0) * x753 * x761)) + ((cj5 * r20 * x754)) + (((-338301381.520162) * r00 * x750)) + (((-5.51415797317437e-7) * r20 * x760)) + ((x750 * x751)) + (((393443020.864604) * sj1)))) - 1) <= IKFAST_SINCOS_THRESH)
                                                                continue;
                                                            j2array[0] = IKatan2(((((-1.0) * x758 * x760)) + (((338301381.520162) * r00 * x761)) + ((cj1 * x752)) + (((-5.51415797317437e-7) * r20 * x756)) + (((-1.0) * x750 * x753)) + (((-1.0) * x751 * x761)) + ((sj1 * x759)) + (((-1.0) * x757 * x760)) + (((-0.169150521609538) * sj1 * x755)) + (((-393443020.864604) * cj1)) + (((0.169150521609538) * r20 * x750))), ((-1.02235469448584) + ((x756 * x757)) + ((x756 * x758)) + (((-1.0) * sj1 * x752)) + ((cj1 * x759)) + (((-1.0) * x754 * x755)) + (((-1.0) * x753 * x761)) + ((cj5 * r20 * x754)) + (((-338301381.520162) * r00 * x750)) + (((-5.51415797317437e-7) * r20 * x760)) + ((x750 * x751)) + (((393443020.864604) * sj1))));
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
                                                                    IkReal x764 = IKsin(j2);
                                                                    IkReal x765 = IKcos(j2);
                                                                    IkReal x766 = ((3.7e-7) * sj5);
                                                                    IkReal x767 = ((3.7e-7) * cj5);
                                                                    IkReal x768 = ((0.1135) * cj5);
                                                                    IkReal x769 = ((0.1135) * sj5);
                                                                    IkReal x770 = (cj1 * x764);
                                                                    IkReal x771 = (sj1 * x765);
                                                                    evalcond[0] = (((r20 * x766)) + (((-1.0) * r20 * x768)) + ((r21 * x767)) + ((r21 * x769)) + (((-1.0) * pz)) + (((0.671) * sj1 * x764)) + (((0.686) * cj1)) + (((0.671) * cj1 * x765)));
                                                                    evalcond[1] = ((0.132000001500057) + (((-1.0) * r00 * x768)) + (((3.35499664500294e-10) * x770)) + (((-3.429996570003e-10) * sj1)) + (((-1.0) * px)) + (((-3.35499664500294e-10) * x771)) + ((r00 * x766)) + ((r01 * x767)) + ((r01 * x769)));
                                                                    evalcond[2] = ((-3.59999497500381e-11) + ((r10 * x766)) + (((-1.0) * r10 * x768)) + (((-1.0) * py)) + (((0.671000000000252) * x770)) + ((r11 * x769)) + ((r11 * x767)) + (((-0.671000000000252) * x771)) + (((-0.686000000000257) * sj1)));
                                                                    if (IKabs(evalcond[0]) > IKFAST_EVALCOND_THRESH || IKabs(evalcond[1]) > IKFAST_EVALCOND_THRESH || IKabs(evalcond[2]) > IKFAST_EVALCOND_THRESH)
                                                                    {
                                                                        continue;
                                                                    }
                                                                }

                                                                {
                                                                    IkReal j3eval[2];
                                                                    sj0 = -0.001;
                                                                    cj0 = 0.99999950000025;
                                                                    j0 = 6.28218520535333;
                                                                    IkReal x772 = (((cj5 * r11 * sj4)) + ((r10 * sj4 * sj5)) + (((-1.0) * cj4 * r12)));
                                                                    j3eval[0] = x772;
                                                                    j3eval[1] = IKsign(x772);
                                                                    if (IKabs(j3eval[0]) < 0.0000010000000000 || IKabs(j3eval[1]) < 0.0000010000000000)
                                                                    {
                                                                        {
                                                                            IkReal j3eval[2];
                                                                            sj0 = -0.001;
                                                                            cj0 = 0.99999950000025;
                                                                            j0 = 6.28218520535333;
                                                                            IkReal x773 = ((1.0) * sj4);
                                                                            IkReal x774 = ((((-1.0) * r00 * sj5 * x773)) + (((-1.0) * cj5 * r01 * x773)) + ((cj4 * r02)));
                                                                            j3eval[0] = x774;
                                                                            j3eval[1] = IKsign(x774);
                                                                            if (IKabs(j3eval[0]) < 0.0000010000000000 || IKabs(j3eval[1]) < 0.0000010000000000)
                                                                            {
                                                                                {
                                                                                    IkReal j3eval[2];
                                                                                    sj0 = -0.001;
                                                                                    cj0 = 0.99999950000025;
                                                                                    j0 = 6.28218520535333;
                                                                                    IkReal x775 = cj4 * cj4;
                                                                                    IkReal x776 = cj5 * cj5;
                                                                                    IkReal x777 = r22 * r22;
                                                                                    IkReal x778 = r21 * r21;
                                                                                    IkReal x779 = r20 * r20;
                                                                                    IkReal x780 = (r20 * sj5);
                                                                                    IkReal x781 = (cj5 * r21);
                                                                                    IkReal x782 = ((1.0) * x778);
                                                                                    IkReal x783 = ((1.0) * x779);
                                                                                    IkReal x784 = (x775 * x776);
                                                                                    IkReal x785 = ((2.0) * cj4 * r22 * sj4);
                                                                                    IkReal x786 = (((x776 * x778)) + (((-1.0) * x782)) + (((-1.0) * x775 * x783)) + (((-1.0) * x781 * x785)) + (((2.0) * x780 * x781)) + ((x779 * x784)) + (((-1.0) * x776 * x783)) + (((-2.0) * x775 * x780 * x781)) + (((-1.0) * x782 * x784)) + (((-1.0) * x777)) + (((-1.0) * x780 * x785)) + ((x775 * x777)));
                                                                                    j3eval[0] = x786;
                                                                                    j3eval[1] = IKsign(x786);
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
                                                                                            IkReal x787 = cj4 * cj4;
                                                                                            IkReal x788 = cj5 * cj5;
                                                                                            IkReal x789 = r22 * r22;
                                                                                            IkReal x790 = r21 * r21;
                                                                                            IkReal x791 = r20 * r20;
                                                                                            IkReal x792 = ((1.0) * sj1);
                                                                                            IkReal x793 = (r22 * sj4);
                                                                                            IkReal x794 = (sj2 * sj5);
                                                                                            IkReal x795 = (cj4 * r20);
                                                                                            IkReal x796 = (cj5 * sj2);
                                                                                            IkReal x797 = (r21 * sj5);
                                                                                            IkReal x798 = (cj4 * r21);
                                                                                            IkReal x799 = ((2.0) * cj5);
                                                                                            IkReal x800 = (cj2 * cj5 * r20);
                                                                                            IkReal x801 = ((1.0) * x790);
                                                                                            IkReal x802 = ((1.0) * cj1 * cj2);
                                                                                            IkReal x803 = ((1.0) * x791);
                                                                                            IkReal x804 = (x787 * x788);
                                                                                            CheckValue<IkReal> x805 = IKatan2WithCheck(IkReal((((cj1 * r20 * x796)) + (((-1.0) * x792 * x796 * x798)) + ((cj2 * sj1 * x797)) + (((-1.0) * cj1 * r21 * x794)) + (((-1.0) * cj5 * x798 * x802)) + (((-1.0) * x792 * x794 * x795)) + (((-1.0) * x793 * x802)) + (((-1.0) * x792 * x800)) + (((-1.0) * sj2 * x792 * x793)) + (((-1.0) * sj5 * x795 * x802)))), IkReal(((((-1.0) * cj2 * x792 * x793)) + (((-1.0) * cj2 * cj5 * x792 * x798)) + ((cj1 * x794 * x795)) + ((cj1 * x796 * x798)) + (((-1.0) * x797 * x802)) + ((cj1 * x800)) + (((-1.0) * r21 * x792 * x794)) + ((r20 * sj1 * x796)) + ((cj1 * sj2 * x793)) + (((-1.0) * cj2 * sj5 * x792 * x795)))), IKFAST_ATAN2_MAGTHRESH);
                                                                                            if (!x805.valid)
                                                                                            {
                                                                                                continue;
                                                                                            }
                                                                                            CheckValue<IkReal> x806 = IKPowWithIntegerCheck(IKsign(((((-2.0) * sj5 * x793 * x795)) + (((-1.0) * r20 * x787 * x797 * x799)) + (((-1.0) * x787 * x803)) + ((x787 * x789)) + ((x788 * x790)) + (((-1.0) * x793 * x798 * x799)) + ((x791 * x804)) + (((-1.0) * x788 * x803)) + (((-1.0) * x789)) + ((r20 * x797 * x799)) + (((-1.0) * x801)) + (((-1.0) * x801 * x804)))), -1);
                                                                                            if (!x806.valid)
                                                                                            {
                                                                                                continue;
                                                                                            }
                                                                                            j3array[0] = ((-1.5707963267949) + (x805.value) + (((1.5707963267949) * (x806.value))));
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
                                                                                                    IkReal x807 = IKcos(j3);
                                                                                                    IkReal x808 = IKsin(j3);
                                                                                                    IkReal x809 = (sj1 * sj2);
                                                                                                    IkReal x810 = (r00 * sj5);
                                                                                                    IkReal x811 = (r10 * sj5);
                                                                                                    IkReal x812 = ((1.0) * r11);
                                                                                                    IkReal x813 = (cj1 * cj2);
                                                                                                    IkReal x814 = (cj2 * sj1);
                                                                                                    IkReal x815 = (cj1 * sj2);
                                                                                                    IkReal x816 = (cj5 * x807);
                                                                                                    IkReal x817 = (sj4 * x807);
                                                                                                    IkReal x818 = (cj5 * x808);
                                                                                                    IkReal x819 = (sj5 * x807);
                                                                                                    IkReal x820 = (cj4 * x807);
                                                                                                    IkReal x821 = ((1.0) * cj4 * x808);
                                                                                                    IkReal x822 = ((1.0) * sj4 * x808);
                                                                                                    IkReal x823 = ((1.0) * sj5 * x808);
                                                                                                    evalcond[0] = ((((-1.0) * r21 * x823)) + ((r20 * x818)) + (((-1.0) * x814)) + ((cj4 * r21 * x816)) + ((cj4 * r20 * x819)) + x815 + ((r22 * x817)));
                                                                                                    evalcond[1] = (((r20 * x816)) + (((-1.0) * r21 * x819)) + (((-1.0) * r22 * x822)) + x813 + x809 + (((-1.0) * r20 * sj5 * x821)) + (((-1.0) * cj4 * r21 * x818)));
                                                                                                    evalcond[2] = ((((-4.99999500000438e-10) * x813)) + (((-1.0) * r01 * x823)) + ((r00 * x818)) + ((x810 * x820)) + (((-4.99999500000438e-10) * x809)) + ((cj4 * r01 * x816)) + ((r02 * x817)));
                                                                                                    evalcond[3] = ((((-1.00000000000038) * x809)) + ((x811 * x820)) + (((-1.0) * sj5 * x808 * x812)) + ((r10 * x818)) + ((cj4 * r11 * x816)) + (((-1.00000000000038) * x813)) + ((r12 * x817)));
                                                                                                    evalcond[4] = ((((-1.0) * r02 * x822)) + (((-4.99999500000438e-10) * x814)) + (((-1.0) * cj4 * r01 * x818)) + (((4.99999500000438e-10) * x815)) + ((r00 * x816)) + (((-1.0) * x810 * x821)) + (((-1.0) * r01 * x819)));
                                                                                                    evalcond[5] = ((((-1.0) * x812 * x819)) + (((-1.0) * r12 * x822)) + ((r10 * x816)) + (((-1.0) * x811 * x821)) + (((-1.0) * cj4 * x812 * x818)) + (((-1.00000000000038) * x814)) + (((1.00000000000038) * x815)));
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
                                                                                    IkReal x824 = ((1.0) * sj4);
                                                                                    IkReal x825 = (cj4 * sj2);
                                                                                    IkReal x826 = (cj1 * cj5);
                                                                                    IkReal x827 = ((1.00000000000038) * sj1);
                                                                                    IkReal x828 = (r20 * sj5);
                                                                                    IkReal x829 = (sj2 * sj4);
                                                                                    IkReal x830 = ((1.00000000000038) * cj2);
                                                                                    IkReal x831 = (r21 * sj5);
                                                                                    IkReal x832 = (cj2 * sj1);
                                                                                    IkReal x833 = ((1.0) * r10);
                                                                                    IkReal x834 = (r11 * sj5);
                                                                                    CheckValue<IkReal> x835 = IKPowWithIntegerCheck(IKsign(((((-1.0) * r00 * sj5 * x824)) + (((-1.0) * cj5 * r01 * x824)) + ((cj4 * r02)))), -1);
                                                                                    if (!x835.valid)
                                                                                    {
                                                                                        continue;
                                                                                    }
                                                                                    CheckValue<IkReal> x836 = IKatan2WithCheck(IkReal((((r11 * x825 * x826)) + (((-1.0) * cj4 * cj5 * r11 * x832)) + ((x825 * x827 * x828)) + ((cj1 * r10 * sj5 * x825)) + (((-1.0) * r12 * x824 * x832)) + ((r22 * x827 * x829)) + ((cj4 * r21 * x826 * x830)) + ((cj1 * cj4 * x828 * x830)) + ((cj5 * r21 * x825 * x827)) + ((cj1 * r22 * sj4 * x830)) + (((-1.0) * cj4 * sj5 * x832 * x833)) + ((cj1 * r12 * x829)))), IkReal(((((-1.0) * cj5 * r20 * sj2 * x827)) + (((-1.0) * r20 * x826 * x830)) + (((-1.0) * sj2 * x826 * x833)) + ((cj1 * sj2 * x834)) + ((cj1 * x830 * x831)) + ((sj2 * x827 * x831)) + ((cj5 * r10 * x832)) + (((-1.0) * x832 * x834)))), IKFAST_ATAN2_MAGTHRESH);
                                                                                    if (!x836.valid)
                                                                                    {
                                                                                        continue;
                                                                                    }
                                                                                    j3array[0] = ((-1.5707963267949) + (((1.5707963267949) * (x835.value))) + (x836.value));
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
                                                                                            IkReal x837 = IKcos(j3);
                                                                                            IkReal x838 = IKsin(j3);
                                                                                            IkReal x839 = (sj1 * sj2);
                                                                                            IkReal x840 = (r00 * sj5);
                                                                                            IkReal x841 = (r10 * sj5);
                                                                                            IkReal x842 = ((1.0) * r11);
                                                                                            IkReal x843 = (cj1 * cj2);
                                                                                            IkReal x844 = (cj2 * sj1);
                                                                                            IkReal x845 = (cj1 * sj2);
                                                                                            IkReal x846 = (cj5 * x837);
                                                                                            IkReal x847 = (sj4 * x837);
                                                                                            IkReal x848 = (cj5 * x838);
                                                                                            IkReal x849 = (sj5 * x837);
                                                                                            IkReal x850 = (cj4 * x837);
                                                                                            IkReal x851 = ((1.0) * cj4 * x838);
                                                                                            IkReal x852 = ((1.0) * sj4 * x838);
                                                                                            IkReal x853 = ((1.0) * sj5 * x838);
                                                                                            evalcond[0] = (((r22 * x847)) + ((r20 * x848)) + (((-1.0) * r21 * x853)) + (((-1.0) * x844)) + ((cj4 * r21 * x846)) + x845 + ((cj4 * r20 * x849)));
                                                                                            evalcond[1] = ((((-1.0) * r22 * x852)) + ((r20 * x846)) + x839 + x843 + (((-1.0) * r20 * sj5 * x851)) + (((-1.0) * r21 * x849)) + (((-1.0) * cj4 * r21 * x848)));
                                                                                            evalcond[2] = (((cj4 * r01 * x846)) + (((-1.0) * r01 * x853)) + ((r02 * x847)) + ((r00 * x848)) + ((x840 * x850)) + (((-4.99999500000438e-10) * x839)) + (((-4.99999500000438e-10) * x843)));
                                                                                            evalcond[3] = (((cj4 * r11 * x846)) + (((-1.0) * sj5 * x838 * x842)) + ((x841 * x850)) + ((r10 * x848)) + (((-1.00000000000038) * x843)) + (((-1.00000000000038) * x839)) + ((r12 * x847)));
                                                                                            evalcond[4] = ((((4.99999500000438e-10) * x845)) + (((-1.0) * cj4 * r01 * x848)) + ((r00 * x846)) + (((-1.0) * r02 * x852)) + (((-1.0) * r01 * x849)) + (((-1.0) * x840 * x851)) + (((-4.99999500000438e-10) * x844)));
                                                                                            evalcond[5] = ((((-1.0) * r12 * x852)) + (((-1.0) * x842 * x849)) + (((-1.0) * x841 * x851)) + ((r10 * x846)) + (((-1.00000000000038) * x844)) + (((1.00000000000038) * x845)) + (((-1.0) * cj4 * x842 * x848)));
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
                                                                            IkReal x854 = (sj1 * sj5);
                                                                            IkReal x855 = ((1.0) * r01);
                                                                            IkReal x856 = (r02 * sj4);
                                                                            IkReal x857 = (cj1 * cj2);
                                                                            IkReal x858 = (cj1 * sj2);
                                                                            IkReal x859 = (sj1 * sj2);
                                                                            IkReal x860 = (cj4 * r00);
                                                                            IkReal x861 = ((4.99999500000438e-10) * cj5);
                                                                            IkReal x862 = (cj4 * r21);
                                                                            IkReal x863 = ((4.99999500000438e-10) * r21);
                                                                            IkReal x864 = (cj2 * sj1);
                                                                            IkReal x865 = (cj5 * r00);
                                                                            IkReal x866 = ((4.99999500000438e-10) * cj4 * r20);
                                                                            IkReal x867 = ((4.99999500000438e-10) * r22 * sj4);
                                                                            IkReal x868 = (cj4 * cj5 * r01);
                                                                            CheckValue<IkReal> x869 = IKPowWithIntegerCheck(IKsign((((cj5 * r11 * sj4)) + ((r10 * sj4 * sj5)) + (((-1.0) * cj4 * r12)))), -1);
                                                                            if (!x869.valid)
                                                                            {
                                                                                continue;
                                                                            }
                                                                            CheckValue<IkReal> x870 = IKatan2WithCheck(IkReal((((r20 * x861 * x864)) + (((-1.0) * sj5 * x855 * x857)) + (((-1.0) * sj2 * x854 * x855)) + ((x859 * x865)) + (((-1.0) * cj2 * x854 * x863)) + ((sj5 * x858 * x863)) + (((-1.0) * r20 * x858 * x861)) + ((x857 * x865)))), IkReal(((((-1.0) * sj5 * x858 * x866)) + ((cj2 * x854 * x866)) + ((sj2 * x854 * x860)) + ((x856 * x859)) + ((x856 * x857)) + ((x859 * x868)) + (((-1.0) * x858 * x867)) + ((x861 * x862 * x864)) + ((x864 * x867)) + ((x857 * x868)) + (((-1.0) * x858 * x861 * x862)) + ((sj5 * x857 * x860)))), IKFAST_ATAN2_MAGTHRESH);
                                                                            if (!x870.valid)
                                                                            {
                                                                                continue;
                                                                            }
                                                                            j3array[0] = ((-1.5707963267949) + (((1.5707963267949) * (x869.value))) + (x870.value));
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
                                                                                    IkReal x871 = IKcos(j3);
                                                                                    IkReal x872 = IKsin(j3);
                                                                                    IkReal x873 = (sj1 * sj2);
                                                                                    IkReal x874 = (r00 * sj5);
                                                                                    IkReal x875 = (r10 * sj5);
                                                                                    IkReal x876 = ((1.0) * r11);
                                                                                    IkReal x877 = (cj1 * cj2);
                                                                                    IkReal x878 = (cj2 * sj1);
                                                                                    IkReal x879 = (cj1 * sj2);
                                                                                    IkReal x880 = (cj5 * x871);
                                                                                    IkReal x881 = (sj4 * x871);
                                                                                    IkReal x882 = (cj5 * x872);
                                                                                    IkReal x883 = (sj5 * x871);
                                                                                    IkReal x884 = (cj4 * x871);
                                                                                    IkReal x885 = ((1.0) * cj4 * x872);
                                                                                    IkReal x886 = ((1.0) * sj4 * x872);
                                                                                    IkReal x887 = ((1.0) * sj5 * x872);
                                                                                    evalcond[0] = (((r20 * x882)) + (((-1.0) * x878)) + ((cj4 * r21 * x880)) + x879 + ((r22 * x881)) + ((cj4 * r20 * x883)) + (((-1.0) * r21 * x887)));
                                                                                    evalcond[1] = ((((-1.0) * cj4 * r21 * x882)) + ((r20 * x880)) + (((-1.0) * r21 * x883)) + (((-1.0) * r20 * sj5 * x885)) + x873 + x877 + (((-1.0) * r22 * x886)));
                                                                                    evalcond[2] = (((r02 * x881)) + ((x874 * x884)) + (((-4.99999500000438e-10) * x877)) + (((-4.99999500000438e-10) * x873)) + ((r00 * x882)) + ((cj4 * r01 * x880)) + (((-1.0) * r01 * x887)));
                                                                                    evalcond[3] = ((((-1.00000000000038) * x873)) + (((-1.00000000000038) * x877)) + ((r10 * x882)) + ((cj4 * r11 * x880)) + ((x875 * x884)) + (((-1.0) * sj5 * x872 * x876)) + ((r12 * x881)));
                                                                                    evalcond[4] = ((((-4.99999500000438e-10) * x878)) + (((-1.0) * cj4 * r01 * x882)) + ((r00 * x880)) + (((-1.0) * r01 * x883)) + (((4.99999500000438e-10) * x879)) + (((-1.0) * r02 * x886)) + (((-1.0) * x874 * x885)));
                                                                                    evalcond[5] = ((((-1.00000000000038) * x878)) + (((-1.0) * cj4 * x876 * x882)) + ((r10 * x880)) + (((1.00000000000038) * x879)) + (((-1.0) * r12 * x886)) + (((-1.0) * x876 * x883)) + (((-1.0) * x875 * x885)));
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
                                    IkReal x888 = ((76.1584619207786) * r20);
                                    IkReal x889 = ((0.000248269875865093) * r21);
                                    IkReal x890 = (sj1 * sj5);
                                    IkReal x891 = ((671.0) * py);
                                    IkReal x892 = (cj0 * sj1);
                                    IkReal x893 = (cj1 * sj5);
                                    IkReal x894 = ((76.1585) * r11);
                                    IkReal x895 = ((2.48269875865093e-7) * r21);
                                    IkReal x896 = ((0.0761584619207786) * r20);
                                    IkReal x897 = (cj1 * cj5);
                                    IkReal x898 = ((670.999664500252) * pz);
                                    IkReal x899 = ((76.1585) * r10);
                                    IkReal x900 = ((0.00024827) * r10);
                                    IkReal x901 = ((88.5719567205174) * sj0);
                                    IkReal x902 = (cj0 * cj1);
                                    IkReal x903 = (cj5 * x892);
                                    IkReal x904 = ((0.000248269875865093) * cj0 * r20);
                                    IkReal x905 = ((0.0761584619207786) * r21 * sj0);
                                    IkReal x906 = ((0.00024827) * cj5 * r11);
                                    IkReal x907 = ((2.48269875865093e-7) * r20 * sj0);
                                    IkReal x908 = ((76.1584619207786) * cj0 * r21);
                                    IkReal x909 = ((0.670999664500252) * pz * sj0);
                                    IkReal x910 = (cj5 * sj0 * sj1);
                                    CheckValue<IkReal> x911 = IKatan2WithCheck(IkReal(((((-1.0) * x890 * x904)) + (((-1.0) * x890 * x908)) + (((-1.0) * x893 * x900)) + ((x890 * x905)) + ((x890 * x907)) + (((-0.00024827) * r11 * x897)) + (((-1.0) * x889 * x903)) + ((x895 * x910)) + (((-1.0) * sj1 * x909)) + ((x897 * x899)) + ((x892 * x898)) + (((-0.0885719768505174) * x902)) + (((-1.0) * x893 * x894)) + ((x888 * x903)) + ((cj1 * x891)) + (((-1.0) * x896 * x910)) + (((-1.0) * cj1 * x901)))), IkReal(((((-1.0) * cj0 * x889 * x897)) + (((0.460305769847173) * sj0)) + (((-1.0) * x893 * x908)) + (((-1.0) * x893 * x904)) + ((x890 * x900)) + (((0.0885719768505174) * x892)) + ((sj0 * x895 * x897)) + ((x893 * x905)) + ((x893 * x907)) + (((-1.0) * cj5 * sj1 * x899)) + (((-460.305769847173) * cj0)) + (((-1.0) * sj0 * x896 * x897)) + ((x890 * x894)) + ((x898 * x902)) + (((-1.0) * sj1 * x891)) + (((-1.0) * cj1 * x909)) + ((cj0 * x888 * x897)) + ((sj1 * x901)) + ((sj1 * x906)))), IKFAST_ATAN2_MAGTHRESH);
                                    if (!x911.valid)
                                    {
                                        continue;
                                    }
                                    CheckValue<IkReal> x912 = IKPowWithIntegerCheck(IKsign(((((-0.450240774879669) * sj0)) + (((450.240774879669) * cj0)))), -1);
                                    if (!x912.valid)
                                    {
                                        continue;
                                    }
                                    j2array[0] = ((-1.5707963267949) + (x911.value) + (((1.5707963267949) * (x912.value))));
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
                                            IkReal x913 = IKcos(j2);
                                            IkReal x914 = IKsin(j2);
                                            IkReal x915 = ((0.000670999664500252) * cj0);
                                            IkReal x916 = ((3.7e-7) * sj5);
                                            IkReal x917 = ((0.000670999664500252) * sj0);
                                            IkReal x918 = ((3.7e-7) * cj5);
                                            IkReal x919 = ((0.670999664500252) * sj0);
                                            IkReal x920 = ((0.1135) * cj5);
                                            IkReal x921 = ((0.1135) * sj5);
                                            IkReal x922 = (cj0 * sj1);
                                            IkReal x923 = (sj0 * sj1);
                                            IkReal x924 = ((0.670999664500252) * cj0);
                                            IkReal x925 = (sj1 * x913);
                                            IkReal x926 = (cj1 * x914);
                                            evalcond[0] = ((((-1.0) * r20 * x920)) + ((r20 * x916)) + ((r21 * x921)) + (((0.671) * sj1 * x914)) + (((-1.0) * pz)) + (((0.671) * cj1 * x913)) + ((r21 * x918)) + (((0.686) * cj1)));
                                            evalcond[1] = ((((0.685999657000257) * x923)) + ((x915 * x925)) + ((x919 * x925)) + (((0.000685999657000257) * x922)) + (((-0.000131999965500026) * sj0)) + (((-1.0) * px)) + (((-1.0) * x919 * x926)) + (((0.131999935500026) * cj0)) + ((r01 * x921)) + (((-1.0) * x915 * x926)) + (((-1.0) * r00 * x920)) + ((r01 * x918)) + ((r00 * x916)));
                                            evalcond[2] = ((((-0.670999664500252) * x913 * x922)) + (((-1.0) * r10 * x920)) + ((x924 * x926)) + (((0.000685999657000257) * x923)) + (((-0.685999657000257) * x922)) + (((0.000131999965500026) * cj0)) + ((r10 * x916)) + ((r11 * x918)) + (((-1.0) * py)) + ((r11 * x921)) + ((x917 * x925)) + (((0.131999935500026) * sj0)) + (((-1.0) * x917 * x926)));
                                            if (IKabs(evalcond[0]) > IKFAST_EVALCOND_THRESH || IKabs(evalcond[1]) > IKFAST_EVALCOND_THRESH || IKabs(evalcond[2]) > IKFAST_EVALCOND_THRESH)
                                            {
                                                continue;
                                            }
                                        }

                                        {
                                            IkReal j3eval[2];
                                            IkReal x927 = cj4 * cj4;
                                            IkReal x928 = cj5 * cj5;
                                            IkReal x929 = r22 * r22;
                                            IkReal x930 = r21 * r21;
                                            IkReal x931 = r20 * r20;
                                            IkReal x932 = (r20 * sj5);
                                            IkReal x933 = (cj5 * r21);
                                            IkReal x934 = ((1.0) * x930);
                                            IkReal x935 = ((1.0) * x931);
                                            IkReal x936 = (x927 * x928);
                                            IkReal x937 = ((2.0) * cj4 * r22 * sj4);
                                            IkReal x938 = ((((-1.0) * x928 * x935)) + ((x927 * x929)) + ((x928 * x930)) + (((-2.0) * x927 * x932 * x933)) + (((-1.0) * x927 * x935)) + (((-1.0) * x934 * x936)) + (((-1.0) * x929)) + ((x931 * x936)) + (((-1.0) * x932 * x937)) + (((2.0) * x932 * x933)) + (((-1.0) * x933 * x937)) + (((-1.0) * x934)));
                                            j3eval[0] = x938;
                                            j3eval[1] = IKsign(x938);
                                            if (IKabs(j3eval[0]) < 0.0000010000000000 || IKabs(j3eval[1]) < 0.0000010000000000)
                                            {
                                                {
                                                    IkReal j3eval[2];
                                                    IkReal x939 = (((cj5 * r11 * sj4)) + ((r10 * sj4 * sj5)) + (((-1.0) * cj4 * r12)));
                                                    j3eval[0] = x939;
                                                    j3eval[1] = IKsign(x939);
                                                    if (IKabs(j3eval[0]) < 0.0000010000000000 || IKabs(j3eval[1]) < 0.0000010000000000)
                                                    {
                                                        {
                                                            IkReal j3eval[2];
                                                            IkReal x940 = ((1.0) * sj4);
                                                            IkReal x941 = ((((-1.0) * cj5 * r01 * x940)) + (((-1.0) * r00 * sj5 * x940)) + ((cj4 * r02)));
                                                            j3eval[0] = x941;
                                                            j3eval[1] = IKsign(x941);
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
                                                                    IkReal x942 = (cj4 * r20);
                                                                    IkReal x943 = (cj5 * sj2);
                                                                    IkReal x944 = (cj1 * cj4);
                                                                    IkReal x945 = (r20 * sj1);
                                                                    IkReal x946 = ((0.999999500000375) * cj0);
                                                                    IkReal x947 = (sj2 * sj5);
                                                                    IkReal x948 = (cj4 * r21);
                                                                    IkReal x949 = ((0.000999999500000375) * sj0);
                                                                    IkReal x950 = (cj1 * sj4);
                                                                    IkReal x951 = (cj2 * r22);
                                                                    IkReal x952 = ((1.0) * sj4);
                                                                    IkReal x953 = (cj2 * sj1);
                                                                    IkReal x954 = ((1.0) * r10);
                                                                    IkReal x955 = (cj2 * r21);
                                                                    IkReal x956 = (cj1 * sj5);
                                                                    IkReal x957 = ((1.0) * r11);
                                                                    IkReal x958 = (sj1 * x949);
                                                                    IkReal x959 = (r22 * sj2 * sj4);
                                                                    IkReal x960 = (cj1 * cj2 * cj5 * r20);
                                                                    IkReal x961 = (sj1 * x946 * x947);
                                                                    CheckValue<IkReal> x962 = IKPowWithIntegerCheck(IKsign(((((-1.0) * cj5 * r01 * x952)) + (((-1.0) * r00 * sj5 * x952)) + ((cj4 * r02)))), -1);
                                                                    if (!x962.valid)
                                                                    {
                                                                        continue;
                                                                    }
                                                                    CheckValue<IkReal> x963 = IKatan2WithCheck(IkReal(((((-1.0) * x942 * x947 * x958)) + (((-1.0) * cj4 * cj5 * x953 * x957)) + (((-1.0) * cj4 * sj5 * x953 * x954)) + (((-1.0) * r12 * x952 * x953)) + ((cj5 * x944 * x946 * x955)) + (((-1.0) * cj2 * x942 * x949 * x956)) + ((r10 * x944 * x947)) + ((sj1 * x943 * x946 * x948)) + ((x942 * x961)) + ((r12 * sj2 * x950)) + ((x946 * x950 * x951)) + (((-1.0) * cj5 * x944 * x949 * x955)) + (((-1.0) * x958 * x959)) + (((-1.0) * x943 * x948 * x958)) + ((sj1 * x946 * x959)) + (((-1.0) * x949 * x950 * x951)) + ((r11 * x943 * x944)) + ((cj2 * x942 * x946 * x956)))), IkReal((((x946 * x955 * x956)) + ((cj5 * r10 * x953)) + ((x949 * x960)) + (((-1.0) * sj5 * x953 * x957)) + (((-1.0) * x949 * x955 * x956)) + ((x943 * x945 * x949)) + (((-1.0) * r21 * x947 * x958)) + (((-1.0) * x946 * x960)) + ((r21 * x961)) + (((-1.0) * cj1 * x943 * x954)) + (((-1.0) * x943 * x945 * x946)) + ((cj1 * r11 * x947)))), IKFAST_ATAN2_MAGTHRESH);
                                                                    if (!x963.valid)
                                                                    {
                                                                        continue;
                                                                    }
                                                                    j3array[0] = ((-1.5707963267949) + (((1.5707963267949) * (x962.value))) + (x963.value));
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
                                                                            IkReal x964 = IKcos(j3);
                                                                            IkReal x965 = IKsin(j3);
                                                                            IkReal x966 = (r00 * sj5);
                                                                            IkReal x967 = ((0.000999999500000375) * cj0);
                                                                            IkReal x968 = (cj2 * sj1);
                                                                            IkReal x969 = (sj1 * sj2);
                                                                            IkReal x970 = (r10 * sj5);
                                                                            IkReal x971 = (cj1 * sj2);
                                                                            IkReal x972 = ((1.0) * r11);
                                                                            IkReal x973 = ((0.999999500000375) * sj0);
                                                                            IkReal x974 = (cj1 * cj2);
                                                                            IkReal x975 = ((0.000999999500000375) * sj0);
                                                                            IkReal x976 = ((0.999999500000375) * cj0);
                                                                            IkReal x977 = (cj5 * x964);
                                                                            IkReal x978 = (cj1 * x976);
                                                                            IkReal x979 = (sj4 * x964);
                                                                            IkReal x980 = (cj5 * x965);
                                                                            IkReal x981 = (sj5 * x964);
                                                                            IkReal x982 = (cj4 * x964);
                                                                            IkReal x983 = ((1.0) * cj4 * x965);
                                                                            IkReal x984 = ((1.0) * sj4 * x965);
                                                                            IkReal x985 = ((1.0) * sj5 * x965);
                                                                            evalcond[0] = ((((-1.0) * r21 * x985)) + ((r22 * x979)) + (((-1.0) * x968)) + ((cj4 * r21 * x977)) + ((cj4 * r20 * x981)) + ((r20 * x980)) + x971);
                                                                            evalcond[1] = ((((-1.0) * r20 * sj5 * x983)) + (((-1.0) * r21 * x981)) + ((r20 * x977)) + (((-1.0) * cj4 * r21 * x980)) + x974 + x969 + (((-1.0) * r22 * x984)));
                                                                            evalcond[2] = (((x969 * x973)) + (((-1.0) * r01 * x985)) + ((r00 * x980)) + ((x973 * x974)) + ((x967 * x969)) + ((x966 * x982)) + ((x967 * x974)) + ((cj4 * r01 * x977)) + ((r02 * x979)));
                                                                            evalcond[3] = (((x969 * x975)) + (((-1.0) * x969 * x976)) + ((cj4 * r11 * x977)) + (((-1.0) * x974 * x976)) + ((r12 * x979)) + ((x974 * x975)) + ((r10 * x980)) + ((x970 * x982)) + (((-1.0) * sj5 * x965 * x972)));
                                                                            evalcond[4] = ((((-1.0) * x966 * x983)) + ((r00 * x977)) + (((-1.0) * x971 * x973)) + ((x968 * x973)) + ((x967 * x968)) + (((-1.0) * x967 * x971)) + (((-1.0) * r02 * x984)) + (((-1.0) * cj4 * r01 * x980)) + (((-1.0) * r01 * x981)));
                                                                            evalcond[5] = ((((-1.0) * x971 * x975)) + ((x968 * x975)) + (((-1.0) * cj4 * x972 * x980)) + (((-1.0) * x970 * x983)) + ((x971 * x976)) + (((-1.0) * x972 * x981)) + (((-1.0) * r12 * x984)) + ((r10 * x977)) + (((-1.0) * x968 * x976)));
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
                                                            IkReal x986 = (cj4 * sj2);
                                                            IkReal x987 = (cj1 * cj5);
                                                            IkReal x988 = (sj1 * sj5);
                                                            IkReal x989 = ((1.0) * r01);
                                                            IkReal x990 = ((0.999999500000375) * sj0);
                                                            IkReal x991 = (cj2 * sj4);
                                                            IkReal x992 = ((0.000999999500000375) * cj0);
                                                            IkReal x993 = (r22 * sj1);
                                                            IkReal x994 = (sj2 * sj4);
                                                            IkReal x995 = (cj1 * r22);
                                                            IkReal x996 = (cj1 * sj5);
                                                            IkReal x997 = (cj2 * cj4);
                                                            IkReal x998 = (r20 * sj2);
                                                            IkReal x999 = (cj5 * sj1);
                                                            IkReal x1000 = (r21 * x992);
                                                            IkReal x1001 = (r20 * x997);
                                                            IkReal x1002 = (cj2 * x999);
                                                            CheckValue<IkReal> x1003 = IKPowWithIntegerCheck(IKsign((((cj5 * r11 * sj4)) + ((r10 * sj4 * sj5)) + (((-1.0) * cj4 * r12)))), -1);
                                                            if (!x1003.valid)
                                                            {
                                                                continue;
                                                            }
                                                            CheckValue<IkReal> x1004 = IKatan2WithCheck(IkReal((((r00 * sj2 * x999)) + (((-1.0) * cj2 * x989 * x996)) + (((-1.0) * sj2 * x988 * x989)) + ((x987 * x992 * x998)) + ((cj2 * x1000 * x988)) + ((cj2 * r00 * x987)) + ((x987 * x990 * x998)) + ((cj2 * r21 * x988 * x990)) + (((-1.0) * r21 * sj2 * x990 * x996)) + (((-1.0) * sj2 * x1000 * x996)) + (((-1.0) * r20 * x1002 * x990)) + (((-1.0) * r20 * x1002 * x992)))), IkReal(((((-1.0) * r21 * x990 * x997 * x999)) + ((r00 * x986 * x988)) + ((r01 * x987 * x997)) + ((x1000 * x986 * x987)) + ((r21 * x986 * x987 * x990)) + (((-1.0) * x1001 * x988 * x990)) + (((-1.0) * x1001 * x988 * x992)) + ((r20 * x986 * x990 * x996)) + (((-1.0) * x990 * x991 * x993)) + (((-1.0) * x991 * x992 * x993)) + ((r02 * sj1 * x994)) + ((cj1 * r02 * x991)) + ((r01 * x986 * x999)) + ((r00 * x996 * x997)) + ((x992 * x994 * x995)) + ((x990 * x994 * x995)) + (((-1.0) * x1000 * x997 * x999)) + ((r20 * x986 * x992 * x996)))), IKFAST_ATAN2_MAGTHRESH);
                                                            if (!x1004.valid)
                                                            {
                                                                continue;
                                                            }
                                                            j3array[0] = ((-1.5707963267949) + (((1.5707963267949) * (x1003.value))) + (x1004.value));
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
                                                                    IkReal x1005 = IKcos(j3);
                                                                    IkReal x1006 = IKsin(j3);
                                                                    IkReal x1007 = (r00 * sj5);
                                                                    IkReal x1008 = ((0.000999999500000375) * cj0);
                                                                    IkReal x1009 = (cj2 * sj1);
                                                                    IkReal x1010 = (sj1 * sj2);
                                                                    IkReal x1011 = (r10 * sj5);
                                                                    IkReal x1012 = (cj1 * sj2);
                                                                    IkReal x1013 = ((1.0) * r11);
                                                                    IkReal x1014 = ((0.999999500000375) * sj0);
                                                                    IkReal x1015 = (cj1 * cj2);
                                                                    IkReal x1016 = ((0.000999999500000375) * sj0);
                                                                    IkReal x1017 = ((0.999999500000375) * cj0);
                                                                    IkReal x1018 = (cj5 * x1005);
                                                                    IkReal x1019 = (cj1 * x1017);
                                                                    IkReal x1020 = (sj4 * x1005);
                                                                    IkReal x1021 = (cj5 * x1006);
                                                                    IkReal x1022 = (sj5 * x1005);
                                                                    IkReal x1023 = (cj4 * x1005);
                                                                    IkReal x1024 = ((1.0) * cj4 * x1006);
                                                                    IkReal x1025 = ((1.0) * sj4 * x1006);
                                                                    IkReal x1026 = ((1.0) * sj5 * x1006);
                                                                    evalcond[0] = (((r20 * x1021)) + (((-1.0) * x1009)) + x1012 + ((r22 * x1020)) + ((cj4 * r21 * x1018)) + (((-1.0) * r21 * x1026)) + ((cj4 * r20 * x1022)));
                                                                    evalcond[1] = ((((-1.0) * r22 * x1025)) + (((-1.0) * cj4 * r21 * x1021)) + x1015 + x1010 + (((-1.0) * r21 * x1022)) + ((r20 * x1018)) + (((-1.0) * r20 * sj5 * x1024)));
                                                                    evalcond[2] = (((x1014 * x1015)) + ((cj4 * r01 * x1018)) + ((r02 * x1020)) + (((-1.0) * r01 * x1026)) + ((x1007 * x1023)) + ((x1008 * x1015)) + ((x1008 * x1010)) + ((x1010 * x1014)) + ((r00 * x1021)));
                                                                    evalcond[3] = (((x1011 * x1023)) + (((-1.0) * x1010 * x1017)) + ((cj4 * r11 * x1018)) + (((-1.0) * x1015 * x1017)) + ((r12 * x1020)) + (((-1.0) * sj5 * x1006 * x1013)) + ((x1015 * x1016)) + ((x1010 * x1016)) + ((r10 * x1021)));
                                                                    evalcond[4] = ((((-1.0) * x1012 * x1014)) + (((-1.0) * cj4 * r01 * x1021)) + ((x1008 * x1009)) + ((x1009 * x1014)) + (((-1.0) * x1008 * x1012)) + (((-1.0) * r01 * x1022)) + (((-1.0) * r02 * x1025)) + (((-1.0) * x1007 * x1024)) + ((r00 * x1018)));
                                                                    evalcond[5] = ((((-1.0) * x1012 * x1016)) + (((-1.0) * r12 * x1025)) + ((x1009 * x1016)) + (((-1.0) * x1009 * x1017)) + (((-1.0) * cj4 * x1013 * x1021)) + ((x1012 * x1017)) + ((r10 * x1018)) + (((-1.0) * x1013 * x1022)) + (((-1.0) * x1011 * x1024)));
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
                                                    IkReal x1027 = cj4 * cj4;
                                                    IkReal x1028 = cj5 * cj5;
                                                    IkReal x1029 = r22 * r22;
                                                    IkReal x1030 = r21 * r21;
                                                    IkReal x1031 = r20 * r20;
                                                    IkReal x1032 = ((1.0) * sj1);
                                                    IkReal x1033 = (r22 * sj4);
                                                    IkReal x1034 = (sj2 * sj5);
                                                    IkReal x1035 = (cj4 * r20);
                                                    IkReal x1036 = (cj5 * sj2);
                                                    IkReal x1037 = (r21 * sj5);
                                                    IkReal x1038 = (cj4 * r21);
                                                    IkReal x1039 = ((2.0) * cj5);
                                                    IkReal x1040 = (cj2 * cj5 * r20);
                                                    IkReal x1041 = ((1.0) * x1030);
                                                    IkReal x1042 = ((1.0) * cj1 * cj2);
                                                    IkReal x1043 = ((1.0) * x1031);
                                                    IkReal x1044 = (x1027 * x1028);
                                                    CheckValue<IkReal> x1045 = IKPowWithIntegerCheck(IKsign((((r20 * x1037 * x1039)) + (((-1.0) * x1029)) + (((-1.0) * x1028 * x1043)) + (((-1.0) * x1027 * x1043)) + (((-1.0) * x1041 * x1044)) + ((x1031 * x1044)) + (((-2.0) * sj5 * x1033 * x1035)) + ((x1028 * x1030)) + (((-1.0) * x1033 * x1038 * x1039)) + (((-1.0) * r20 * x1027 * x1037 * x1039)) + ((x1027 * x1029)) + (((-1.0) * x1041)))), -1);
                                                    if (!x1045.valid)
                                                    {
                                                        continue;
                                                    }
                                                    CheckValue<IkReal> x1046 = IKatan2WithCheck(IkReal(((((-1.0) * x1032 * x1034 * x1035)) + (((-1.0) * x1032 * x1040)) + (((-1.0) * x1033 * x1042)) + (((-1.0) * x1032 * x1036 * x1038)) + (((-1.0) * sj2 * x1032 * x1033)) + (((-1.0) * cj1 * r21 * x1034)) + (((-1.0) * cj5 * x1038 * x1042)) + (((-1.0) * sj5 * x1035 * x1042)) + ((cj2 * sj1 * x1037)) + ((cj1 * r20 * x1036)))), IkReal((((cj1 * sj2 * x1033)) + (((-1.0) * cj2 * sj5 * x1032 * x1035)) + ((cj1 * x1034 * x1035)) + (((-1.0) * cj2 * cj5 * x1032 * x1038)) + ((cj1 * x1040)) + (((-1.0) * x1037 * x1042)) + (((-1.0) * cj2 * x1032 * x1033)) + ((r20 * sj1 * x1036)) + ((cj1 * x1036 * x1038)) + (((-1.0) * r21 * x1032 * x1034)))), IKFAST_ATAN2_MAGTHRESH);
                                                    if (!x1046.valid)
                                                    {
                                                        continue;
                                                    }
                                                    j3array[0] = ((-1.5707963267949) + (((1.5707963267949) * (x1045.value))) + (x1046.value));
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
                                                            IkReal x1047 = IKcos(j3);
                                                            IkReal x1048 = IKsin(j3);
                                                            IkReal x1049 = (r00 * sj5);
                                                            IkReal x1050 = ((0.000999999500000375) * cj0);
                                                            IkReal x1051 = (cj2 * sj1);
                                                            IkReal x1052 = (sj1 * sj2);
                                                            IkReal x1053 = (r10 * sj5);
                                                            IkReal x1054 = (cj1 * sj2);
                                                            IkReal x1055 = ((1.0) * r11);
                                                            IkReal x1056 = ((0.999999500000375) * sj0);
                                                            IkReal x1057 = (cj1 * cj2);
                                                            IkReal x1058 = ((0.000999999500000375) * sj0);
                                                            IkReal x1059 = ((0.999999500000375) * cj0);
                                                            IkReal x1060 = (cj5 * x1047);
                                                            IkReal x1061 = (cj1 * x1059);
                                                            IkReal x1062 = (sj4 * x1047);
                                                            IkReal x1063 = (cj5 * x1048);
                                                            IkReal x1064 = (sj5 * x1047);
                                                            IkReal x1065 = (cj4 * x1047);
                                                            IkReal x1066 = ((1.0) * cj4 * x1048);
                                                            IkReal x1067 = ((1.0) * sj4 * x1048);
                                                            IkReal x1068 = ((1.0) * sj5 * x1048);
                                                            evalcond[0] = ((((-1.0) * r21 * x1068)) + x1054 + ((cj4 * r20 * x1064)) + ((r20 * x1063)) + ((cj4 * r21 * x1060)) + (((-1.0) * x1051)) + ((r22 * x1062)));
                                                            evalcond[1] = (x1052 + x1057 + (((-1.0) * r22 * x1067)) + ((r20 * x1060)) + (((-1.0) * r20 * sj5 * x1066)) + (((-1.0) * cj4 * r21 * x1063)) + (((-1.0) * r21 * x1064)));
                                                            evalcond[2] = (((cj4 * r01 * x1060)) + (((-1.0) * r01 * x1068)) + ((x1056 * x1057)) + ((x1052 * x1056)) + ((x1050 * x1052)) + ((x1050 * x1057)) + ((x1049 * x1065)) + ((r00 * x1063)) + ((r02 * x1062)));
                                                            evalcond[3] = ((((-1.0) * x1052 * x1059)) + ((r12 * x1062)) + ((x1057 * x1058)) + ((x1052 * x1058)) + ((cj4 * r11 * x1060)) + ((x1053 * x1065)) + (((-1.0) * sj5 * x1048 * x1055)) + (((-1.0) * x1057 * x1059)) + ((r10 * x1063)));
                                                            evalcond[4] = ((((-1.0) * r01 * x1064)) + (((-1.0) * r02 * x1067)) + ((x1050 * x1051)) + (((-1.0) * x1054 * x1056)) + ((x1051 * x1056)) + (((-1.0) * cj4 * r01 * x1063)) + (((-1.0) * x1049 * x1066)) + ((r00 * x1060)) + (((-1.0) * x1050 * x1054)));
                                                            evalcond[5] = (((x1054 * x1059)) + (((-1.0) * r12 * x1067)) + (((-1.0) * x1054 * x1058)) + ((x1051 * x1058)) + (((-1.0) * x1055 * x1064)) + (((-1.0) * cj4 * x1055 * x1063)) + ((r10 * x1060)) + (((-1.0) * x1053 * x1066)) + (((-1.0) * x1051 * x1059)));
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
                            IkReal x1069 = (cj1 * sj5);
                            IkReal x1070 = ((76.1585) * r01);
                            IkReal x1071 = ((0.000248269875865093) * sj0);
                            IkReal x1072 = (cj1 * cj5);
                            IkReal x1073 = ((76.1585) * r00);
                            IkReal x1074 = (cj0 * cj1);
                            IkReal x1075 = (cj5 * sj1);
                            IkReal x1076 = ((0.00024827) * r01);
                            IkReal x1077 = (cj0 * r20);
                            IkReal x1078 = (sj1 * sj5);
                            IkReal x1079 = (cj0 * r21);
                            IkReal x1080 = ((76.1584619207786) * sj0);
                            IkReal x1081 = (pz * sj1);
                            IkReal x1082 = ((670.999664500252) * sj0);
                            IkReal x1083 = ((0.00024827) * r00);
                            IkReal x1084 = ((0.0885719768505174) * sj0);
                            IkReal x1085 = ((671.0) * px);
                            CheckValue<IkReal> x1086 = IKatan2WithCheck(IkReal(((((-0.670999664500252) * cj0 * x1081)) + ((r21 * x1078 * x1080)) + ((r20 * x1071 * x1078)) + (((-88.5719567205174) * x1074)) + ((cj1 * x1085)) + ((cj1 * x1084)) + (((-0.0761584619207786) * x1075 * x1077)) + (((0.0761584619207786) * x1078 * x1079)) + (((-1.0) * x1081 * x1082)) + (((-1.0) * r20 * x1075 * x1080)) + (((-1.0) * x1069 * x1083)) + (((2.48269875865093e-7) * x1075 * x1079)) + ((x1072 * x1073)) + (((-1.0) * x1072 * x1076)) + (((2.48269875865093e-7) * x1077 * x1078)) + (((-1.0) * x1069 * x1070)) + ((r21 * x1071 * x1075)))), IkReal(((((-1.0) * sj1 * x1084)) + (((-1.0) * sj1 * x1085)) + (((460.305769847173) * sj0)) + (((-1.0) * x1073 * x1075)) + ((x1070 * x1078)) + (((2.48269875865093e-7) * x1069 * x1077)) + (((-0.670999664500252) * pz * x1074)) + ((x1078 * x1083)) + (((0.460305769847173) * cj0)) + ((x1075 * x1076)) + (((2.48269875865093e-7) * x1072 * x1079)) + (((88.5719567205174) * cj0 * sj1)) + (((-1.0) * r20 * x1072 * x1080)) + (((0.0761584619207786) * x1069 * x1079)) + ((r20 * x1069 * x1071)) + ((r21 * x1071 * x1072)) + ((r21 * x1069 * x1080)) + (((-1.0) * cj1 * pz * x1082)) + (((-0.0761584619207786) * x1072 * x1077)))), IKFAST_ATAN2_MAGTHRESH);
                            if (!x1086.valid)
                            {
                                continue;
                            }
                            CheckValue<IkReal> x1087 = IKPowWithIntegerCheck(IKsign(((((-450.240774879669) * sj0)) + (((-0.450240774879669) * cj0)))), -1);
                            if (!x1087.valid)
                            {
                                continue;
                            }
                            j2array[0] = ((-1.5707963267949) + (x1086.value) + (((1.5707963267949) * (x1087.value))));
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
                                    IkReal x1088 = IKcos(j2);
                                    IkReal x1089 = IKsin(j2);
                                    IkReal x1090 = ((0.000670999664500252) * cj0);
                                    IkReal x1091 = ((3.7e-7) * sj5);
                                    IkReal x1092 = ((0.000670999664500252) * sj0);
                                    IkReal x1093 = ((3.7e-7) * cj5);
                                    IkReal x1094 = ((0.670999664500252) * sj0);
                                    IkReal x1095 = ((0.1135) * cj5);
                                    IkReal x1096 = ((0.1135) * sj5);
                                    IkReal x1097 = (cj0 * sj1);
                                    IkReal x1098 = (sj0 * sj1);
                                    IkReal x1099 = ((0.670999664500252) * cj0);
                                    IkReal x1100 = (sj1 * x1088);
                                    IkReal x1101 = (cj1 * x1089);
                                    evalcond[0] = ((((0.671) * sj1 * x1089)) + (((0.671) * cj1 * x1088)) + ((r21 * x1096)) + ((r21 * x1093)) + (((-1.0) * pz)) + (((0.686) * cj1)) + (((-1.0) * r20 * x1095)) + ((r20 * x1091)));
                                    evalcond[1] = (((x1094 * x1100)) + (((-1.0) * x1090 * x1101)) + ((r01 * x1093)) + ((r01 * x1096)) + (((0.685999657000257) * x1098)) + ((r00 * x1091)) + (((-0.000131999965500026) * sj0)) + ((x1090 * x1100)) + (((-1.0) * px)) + (((-1.0) * x1094 * x1101)) + (((0.131999935500026) * cj0)) + (((-1.0) * r00 * x1095)) + (((0.000685999657000257) * x1097)));
                                    evalcond[2] = (((x1092 * x1100)) + (((-1.0) * x1092 * x1101)) + (((-0.685999657000257) * x1097)) + (((-0.670999664500252) * x1088 * x1097)) + ((r10 * x1091)) + (((0.000131999965500026) * cj0)) + (((-1.0) * py)) + ((x1099 * x1101)) + ((r11 * x1093)) + ((r11 * x1096)) + (((-1.0) * r10 * x1095)) + (((0.131999935500026) * sj0)) + (((0.000685999657000257) * x1098)));
                                    if (IKabs(evalcond[0]) > IKFAST_EVALCOND_THRESH || IKabs(evalcond[1]) > IKFAST_EVALCOND_THRESH || IKabs(evalcond[2]) > IKFAST_EVALCOND_THRESH)
                                    {
                                        continue;
                                    }
                                }

                                {
                                    IkReal j3eval[2];
                                    IkReal x1102 = cj4 * cj4;
                                    IkReal x1103 = cj5 * cj5;
                                    IkReal x1104 = r22 * r22;
                                    IkReal x1105 = r21 * r21;
                                    IkReal x1106 = r20 * r20;
                                    IkReal x1107 = (r20 * sj5);
                                    IkReal x1108 = (cj5 * r21);
                                    IkReal x1109 = ((1.0) * x1105);
                                    IkReal x1110 = ((1.0) * x1106);
                                    IkReal x1111 = (x1102 * x1103);
                                    IkReal x1112 = ((2.0) * cj4 * r22 * sj4);
                                    IkReal x1113 = ((((-2.0) * x1102 * x1107 * x1108)) + (((-1.0) * x1103 * x1110)) + ((x1102 * x1104)) + (((-1.0) * x1109 * x1111)) + (((-1.0) * x1104)) + (((-1.0) * x1107 * x1112)) + (((-1.0) * x1108 * x1112)) + (((2.0) * x1107 * x1108)) + (((-1.0) * x1109)) + ((x1103 * x1105)) + (((-1.0) * x1102 * x1110)) + ((x1106 * x1111)));
                                    j3eval[0] = x1113;
                                    j3eval[1] = IKsign(x1113);
                                    if (IKabs(j3eval[0]) < 0.0000010000000000 || IKabs(j3eval[1]) < 0.0000010000000000)
                                    {
                                        {
                                            IkReal j3eval[2];
                                            IkReal x1114 = (((cj5 * r11 * sj4)) + ((r10 * sj4 * sj5)) + (((-1.0) * cj4 * r12)));
                                            j3eval[0] = x1114;
                                            j3eval[1] = IKsign(x1114);
                                            if (IKabs(j3eval[0]) < 0.0000010000000000 || IKabs(j3eval[1]) < 0.0000010000000000)
                                            {
                                                {
                                                    IkReal j3eval[2];
                                                    IkReal x1115 = ((1.0) * sj4);
                                                    IkReal x1116 = ((((-1.0) * r00 * sj5 * x1115)) + (((-1.0) * cj5 * r01 * x1115)) + ((cj4 * r02)));
                                                    j3eval[0] = x1116;
                                                    j3eval[1] = IKsign(x1116);
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
                                                            IkReal x1117 = (cj4 * r20);
                                                            IkReal x1118 = (cj5 * sj2);
                                                            IkReal x1119 = (cj1 * cj4);
                                                            IkReal x1120 = (r20 * sj1);
                                                            IkReal x1121 = ((0.999999500000375) * cj0);
                                                            IkReal x1122 = (sj2 * sj5);
                                                            IkReal x1123 = (cj4 * r21);
                                                            IkReal x1124 = ((0.000999999500000375) * sj0);
                                                            IkReal x1125 = (cj1 * sj4);
                                                            IkReal x1126 = (cj2 * r22);
                                                            IkReal x1127 = ((1.0) * sj4);
                                                            IkReal x1128 = (cj2 * sj1);
                                                            IkReal x1129 = ((1.0) * r10);
                                                            IkReal x1130 = (cj2 * r21);
                                                            IkReal x1131 = (cj1 * sj5);
                                                            IkReal x1132 = ((1.0) * r11);
                                                            IkReal x1133 = (sj1 * x1124);
                                                            IkReal x1134 = (r22 * sj2 * sj4);
                                                            IkReal x1135 = (cj1 * cj2 * cj5 * r20);
                                                            IkReal x1136 = (sj1 * x1121 * x1122);
                                                            CheckValue<IkReal> x1137 = IKPowWithIntegerCheck(IKsign(((((-1.0) * r00 * sj5 * x1127)) + (((-1.0) * cj5 * r01 * x1127)) + ((cj4 * r02)))), -1);
                                                            if (!x1137.valid)
                                                            {
                                                                continue;
                                                            }
                                                            CheckValue<IkReal> x1138 = IKatan2WithCheck(IkReal(((((-1.0) * cj5 * x1119 * x1124 * x1130)) + ((sj1 * x1118 * x1121 * x1123)) + ((cj2 * x1117 * x1121 * x1131)) + (((-1.0) * x1118 * x1123 * x1133)) + ((sj1 * x1121 * x1134)) + ((r11 * x1118 * x1119)) + (((-1.0) * cj4 * sj5 * x1128 * x1129)) + (((-1.0) * x1117 * x1122 * x1133)) + (((-1.0) * x1124 * x1125 * x1126)) + ((r12 * sj2 * x1125)) + ((r10 * x1119 * x1122)) + ((x1117 * x1136)) + (((-1.0) * r12 * x1127 * x1128)) + ((x1121 * x1125 * x1126)) + (((-1.0) * cj4 * cj5 * x1128 * x1132)) + (((-1.0) * cj2 * x1117 * x1124 * x1131)) + ((cj5 * x1119 * x1121 * x1130)) + (((-1.0) * x1133 * x1134)))), IkReal((((cj5 * r10 * x1128)) + ((cj1 * r11 * x1122)) + (((-1.0) * r21 * x1122 * x1133)) + (((-1.0) * x1124 * x1130 * x1131)) + (((-1.0) * x1121 * x1135)) + ((x1118 * x1120 * x1124)) + ((x1121 * x1130 * x1131)) + ((r21 * x1136)) + ((x1124 * x1135)) + (((-1.0) * sj5 * x1128 * x1132)) + (((-1.0) * x1118 * x1120 * x1121)) + (((-1.0) * cj1 * x1118 * x1129)))), IKFAST_ATAN2_MAGTHRESH);
                                                            if (!x1138.valid)
                                                            {
                                                                continue;
                                                            }
                                                            j3array[0] = ((-1.5707963267949) + (((1.5707963267949) * (x1137.value))) + (x1138.value));
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
                                                                    IkReal x1139 = IKcos(j3);
                                                                    IkReal x1140 = IKsin(j3);
                                                                    IkReal x1141 = (r00 * sj5);
                                                                    IkReal x1142 = ((0.000999999500000375) * cj0);
                                                                    IkReal x1143 = (cj2 * sj1);
                                                                    IkReal x1144 = (sj1 * sj2);
                                                                    IkReal x1145 = (r10 * sj5);
                                                                    IkReal x1146 = (cj1 * sj2);
                                                                    IkReal x1147 = ((1.0) * r11);
                                                                    IkReal x1148 = ((0.999999500000375) * sj0);
                                                                    IkReal x1149 = (cj1 * cj2);
                                                                    IkReal x1150 = ((0.000999999500000375) * sj0);
                                                                    IkReal x1151 = ((0.999999500000375) * cj0);
                                                                    IkReal x1152 = (cj5 * x1139);
                                                                    IkReal x1153 = (cj1 * x1151);
                                                                    IkReal x1154 = (sj4 * x1139);
                                                                    IkReal x1155 = (cj5 * x1140);
                                                                    IkReal x1156 = (sj5 * x1139);
                                                                    IkReal x1157 = (cj4 * x1139);
                                                                    IkReal x1158 = ((1.0) * cj4 * x1140);
                                                                    IkReal x1159 = ((1.0) * sj4 * x1140);
                                                                    IkReal x1160 = ((1.0) * sj5 * x1140);
                                                                    evalcond[0] = (x1146 + (((-1.0) * r21 * x1160)) + ((r22 * x1154)) + (((-1.0) * x1143)) + ((r20 * x1155)) + ((cj4 * r20 * x1156)) + ((cj4 * r21 * x1152)));
                                                                    evalcond[1] = (x1144 + x1149 + (((-1.0) * r21 * x1156)) + ((r20 * x1152)) + (((-1.0) * r22 * x1159)) + (((-1.0) * r20 * sj5 * x1158)) + (((-1.0) * cj4 * r21 * x1155)));
                                                                    evalcond[2] = (((r00 * x1155)) + ((cj4 * r01 * x1152)) + ((x1141 * x1157)) + ((x1142 * x1149)) + ((x1142 * x1144)) + ((r02 * x1154)) + (((-1.0) * r01 * x1160)) + ((x1148 * x1149)) + ((x1144 * x1148)));
                                                                    evalcond[3] = ((((-1.0) * x1144 * x1151)) + ((x1149 * x1150)) + (((-1.0) * x1149 * x1151)) + ((r12 * x1154)) + ((x1145 * x1157)) + ((cj4 * r11 * x1152)) + ((x1144 * x1150)) + (((-1.0) * sj5 * x1140 * x1147)) + ((r10 * x1155)));
                                                                    evalcond[4] = (((r00 * x1152)) + (((-1.0) * x1141 * x1158)) + (((-1.0) * r01 * x1156)) + ((x1143 * x1148)) + ((x1142 * x1143)) + (((-1.0) * cj4 * r01 * x1155)) + (((-1.0) * x1146 * x1148)) + (((-1.0) * r02 * x1159)) + (((-1.0) * x1142 * x1146)));
                                                                    evalcond[5] = (((x1146 * x1151)) + ((x1143 * x1150)) + (((-1.0) * x1143 * x1151)) + (((-1.0) * x1147 * x1156)) + (((-1.0) * x1146 * x1150)) + (((-1.0) * r12 * x1159)) + (((-1.0) * x1145 * x1158)) + (((-1.0) * cj4 * x1147 * x1155)) + ((r10 * x1152)));
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
                                                    IkReal x1161 = (cj4 * sj2);
                                                    IkReal x1162 = (cj1 * cj5);
                                                    IkReal x1163 = (sj1 * sj5);
                                                    IkReal x1164 = ((1.0) * r01);
                                                    IkReal x1165 = ((0.999999500000375) * sj0);
                                                    IkReal x1166 = (cj2 * sj4);
                                                    IkReal x1167 = ((0.000999999500000375) * cj0);
                                                    IkReal x1168 = (r22 * sj1);
                                                    IkReal x1169 = (sj2 * sj4);
                                                    IkReal x1170 = (cj1 * r22);
                                                    IkReal x1171 = (cj1 * sj5);
                                                    IkReal x1172 = (cj2 * cj4);
                                                    IkReal x1173 = (r20 * sj2);
                                                    IkReal x1174 = (cj5 * sj1);
                                                    IkReal x1175 = (r21 * x1167);
                                                    IkReal x1176 = (r20 * x1172);
                                                    IkReal x1177 = (cj2 * x1174);
                                                    CheckValue<IkReal> x1178 = IKatan2WithCheck(IkReal((((cj2 * r00 * x1162)) + (((-1.0) * r20 * x1167 * x1177)) + (((-1.0) * r20 * x1165 * x1177)) + ((cj2 * r21 * x1163 * x1165)) + (((-1.0) * r21 * sj2 * x1165 * x1171)) + (((-1.0) * cj2 * x1164 * x1171)) + (((-1.0) * sj2 * x1171 * x1175)) + ((cj2 * x1163 * x1175)) + ((r00 * sj2 * x1174)) + ((x1162 * x1167 * x1173)) + ((x1162 * x1165 * x1173)) + (((-1.0) * sj2 * x1163 * x1164)))), IkReal((((x1165 * x1169 * x1170)) + ((r20 * x1161 * x1167 * x1171)) + (((-1.0) * x1166 * x1167 * x1168)) + ((r00 * x1171 * x1172)) + ((r20 * x1161 * x1165 * x1171)) + ((r01 * x1161 * x1174)) + ((x1161 * x1162 * x1175)) + (((-1.0) * x1163 * x1167 * x1176)) + (((-1.0) * r21 * x1165 * x1172 * x1174)) + (((-1.0) * x1172 * x1174 * x1175)) + ((r00 * x1161 * x1163)) + ((r02 * sj1 * x1169)) + ((r01 * x1162 * x1172)) + (((-1.0) * x1163 * x1165 * x1176)) + ((cj1 * r02 * x1166)) + ((r21 * x1161 * x1162 * x1165)) + ((x1167 * x1169 * x1170)) + (((-1.0) * x1165 * x1166 * x1168)))), IKFAST_ATAN2_MAGTHRESH);
                                                    if (!x1178.valid)
                                                    {
                                                        continue;
                                                    }
                                                    CheckValue<IkReal> x1179 = IKPowWithIntegerCheck(IKsign((((cj5 * r11 * sj4)) + ((r10 * sj4 * sj5)) + (((-1.0) * cj4 * r12)))), -1);
                                                    if (!x1179.valid)
                                                    {
                                                        continue;
                                                    }
                                                    j3array[0] = ((-1.5707963267949) + (x1178.value) + (((1.5707963267949) * (x1179.value))));
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
                                                            IkReal x1180 = IKcos(j3);
                                                            IkReal x1181 = IKsin(j3);
                                                            IkReal x1182 = (r00 * sj5);
                                                            IkReal x1183 = ((0.000999999500000375) * cj0);
                                                            IkReal x1184 = (cj2 * sj1);
                                                            IkReal x1185 = (sj1 * sj2);
                                                            IkReal x1186 = (r10 * sj5);
                                                            IkReal x1187 = (cj1 * sj2);
                                                            IkReal x1188 = ((1.0) * r11);
                                                            IkReal x1189 = ((0.999999500000375) * sj0);
                                                            IkReal x1190 = (cj1 * cj2);
                                                            IkReal x1191 = ((0.000999999500000375) * sj0);
                                                            IkReal x1192 = ((0.999999500000375) * cj0);
                                                            IkReal x1193 = (cj5 * x1180);
                                                            IkReal x1194 = (cj1 * x1192);
                                                            IkReal x1195 = (sj4 * x1180);
                                                            IkReal x1196 = (cj5 * x1181);
                                                            IkReal x1197 = (sj5 * x1180);
                                                            IkReal x1198 = (cj4 * x1180);
                                                            IkReal x1199 = ((1.0) * cj4 * x1181);
                                                            IkReal x1200 = ((1.0) * sj4 * x1181);
                                                            IkReal x1201 = ((1.0) * sj5 * x1181);
                                                            evalcond[0] = (((cj4 * r21 * x1193)) + x1187 + ((r22 * x1195)) + (((-1.0) * r21 * x1201)) + ((r20 * x1196)) + (((-1.0) * x1184)) + ((cj4 * r20 * x1197)));
                                                            evalcond[1] = (x1185 + x1190 + (((-1.0) * r22 * x1200)) + (((-1.0) * r20 * sj5 * x1199)) + (((-1.0) * cj4 * r21 * x1196)) + (((-1.0) * r21 * x1197)) + ((r20 * x1193)));
                                                            evalcond[2] = (((x1183 * x1185)) + ((x1182 * x1198)) + ((x1183 * x1190)) + ((r00 * x1196)) + ((x1185 * x1189)) + ((x1189 * x1190)) + (((-1.0) * r01 * x1201)) + ((cj4 * r01 * x1193)) + ((r02 * x1195)));
                                                            evalcond[3] = ((((-1.0) * x1190 * x1192)) + ((x1186 * x1198)) + (((-1.0) * sj5 * x1181 * x1188)) + ((r10 * x1196)) + ((x1185 * x1191)) + ((cj4 * r11 * x1193)) + (((-1.0) * x1185 * x1192)) + ((r12 * x1195)) + ((x1190 * x1191)));
                                                            evalcond[4] = (((x1184 * x1189)) + ((x1183 * x1184)) + ((r00 * x1193)) + (((-1.0) * r02 * x1200)) + (((-1.0) * r01 * x1197)) + (((-1.0) * x1187 * x1189)) + (((-1.0) * cj4 * r01 * x1196)) + (((-1.0) * x1182 * x1199)) + (((-1.0) * x1183 * x1187)));
                                                            evalcond[5] = (((x1184 * x1191)) + (((-1.0) * cj4 * x1188 * x1196)) + ((x1187 * x1192)) + ((r10 * x1193)) + (((-1.0) * x1188 * x1197)) + (((-1.0) * x1186 * x1199)) + (((-1.0) * x1184 * x1192)) + (((-1.0) * x1187 * x1191)) + (((-1.0) * r12 * x1200)));
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
                                            IkReal x1202 = cj4 * cj4;
                                            IkReal x1203 = cj5 * cj5;
                                            IkReal x1204 = r22 * r22;
                                            IkReal x1205 = r21 * r21;
                                            IkReal x1206 = r20 * r20;
                                            IkReal x1207 = ((1.0) * sj1);
                                            IkReal x1208 = (r22 * sj4);
                                            IkReal x1209 = (sj2 * sj5);
                                            IkReal x1210 = (cj4 * r20);
                                            IkReal x1211 = (cj5 * sj2);
                                            IkReal x1212 = (r21 * sj5);
                                            IkReal x1213 = (cj4 * r21);
                                            IkReal x1214 = ((2.0) * cj5);
                                            IkReal x1215 = (cj2 * cj5 * r20);
                                            IkReal x1216 = ((1.0) * x1205);
                                            IkReal x1217 = ((1.0) * cj1 * cj2);
                                            IkReal x1218 = ((1.0) * x1206);
                                            IkReal x1219 = (x1202 * x1203);
                                            CheckValue<IkReal> x1220 = IKatan2WithCheck(IkReal(((((-1.0) * x1207 * x1211 * x1213)) + (((-1.0) * x1207 * x1215)) + (((-1.0) * x1208 * x1217)) + (((-1.0) * cj5 * x1213 * x1217)) + (((-1.0) * sj2 * x1207 * x1208)) + (((-1.0) * sj5 * x1210 * x1217)) + ((cj1 * r20 * x1211)) + (((-1.0) * cj1 * r21 * x1209)) + ((cj2 * sj1 * x1212)) + (((-1.0) * x1207 * x1209 * x1210)))), IkReal((((cj1 * x1211 * x1213)) + ((cj1 * x1209 * x1210)) + ((r20 * sj1 * x1211)) + (((-1.0) * r21 * x1207 * x1209)) + (((-1.0) * x1212 * x1217)) + (((-1.0) * cj2 * x1207 * x1208)) + (((-1.0) * cj2 * cj5 * x1207 * x1213)) + ((cj1 * sj2 * x1208)) + ((cj1 * x1215)) + (((-1.0) * cj2 * sj5 * x1207 * x1210)))), IKFAST_ATAN2_MAGTHRESH);
                                            if (!x1220.valid)
                                            {
                                                continue;
                                            }
                                            CheckValue<IkReal> x1221 = IKPowWithIntegerCheck(IKsign((((r20 * x1212 * x1214)) + (((-1.0) * x1203 * x1218)) + (((-2.0) * sj5 * x1208 * x1210)) + (((-1.0) * x1216 * x1219)) + (((-1.0) * x1202 * x1218)) + (((-1.0) * r20 * x1202 * x1212 * x1214)) + ((x1202 * x1204)) + ((x1206 * x1219)) + ((x1203 * x1205)) + (((-1.0) * x1204)) + (((-1.0) * x1216)) + (((-1.0) * x1208 * x1213 * x1214)))), -1);
                                            if (!x1221.valid)
                                            {
                                                continue;
                                            }
                                            j3array[0] = ((-1.5707963267949) + (x1220.value) + (((1.5707963267949) * (x1221.value))));
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
                                                    IkReal x1222 = IKcos(j3);
                                                    IkReal x1223 = IKsin(j3);
                                                    IkReal x1224 = (r00 * sj5);
                                                    IkReal x1225 = ((0.000999999500000375) * cj0);
                                                    IkReal x1226 = (cj2 * sj1);
                                                    IkReal x1227 = (sj1 * sj2);
                                                    IkReal x1228 = (r10 * sj5);
                                                    IkReal x1229 = (cj1 * sj2);
                                                    IkReal x1230 = ((1.0) * r11);
                                                    IkReal x1231 = ((0.999999500000375) * sj0);
                                                    IkReal x1232 = (cj1 * cj2);
                                                    IkReal x1233 = ((0.000999999500000375) * sj0);
                                                    IkReal x1234 = ((0.999999500000375) * cj0);
                                                    IkReal x1235 = (cj5 * x1222);
                                                    IkReal x1236 = (cj1 * x1234);
                                                    IkReal x1237 = (sj4 * x1222);
                                                    IkReal x1238 = (cj5 * x1223);
                                                    IkReal x1239 = (sj5 * x1222);
                                                    IkReal x1240 = (cj4 * x1222);
                                                    IkReal x1241 = ((1.0) * cj4 * x1223);
                                                    IkReal x1242 = ((1.0) * sj4 * x1223);
                                                    IkReal x1243 = ((1.0) * sj5 * x1223);
                                                    evalcond[0] = (x1229 + ((r20 * x1238)) + (((-1.0) * x1226)) + (((-1.0) * r21 * x1243)) + ((cj4 * r20 * x1239)) + ((cj4 * r21 * x1235)) + ((r22 * x1237)));
                                                    evalcond[1] = ((((-1.0) * r20 * sj5 * x1241)) + x1232 + x1227 + ((r20 * x1235)) + (((-1.0) * r22 * x1242)) + (((-1.0) * cj4 * r21 * x1238)) + (((-1.0) * r21 * x1239)));
                                                    evalcond[2] = (((x1224 * x1240)) + ((r00 * x1238)) + ((x1225 * x1232)) + ((x1225 * x1227)) + ((r02 * x1237)) + ((x1231 * x1232)) + ((cj4 * r01 * x1235)) + (((-1.0) * r01 * x1243)) + ((x1227 * x1231)));
                                                    evalcond[3] = (((cj4 * r11 * x1235)) + (((-1.0) * sj5 * x1223 * x1230)) + ((r10 * x1238)) + (((-1.0) * x1227 * x1234)) + ((x1228 * x1240)) + ((x1232 * x1233)) + ((x1227 * x1233)) + (((-1.0) * x1232 * x1234)) + ((r12 * x1237)));
                                                    evalcond[4] = ((((-1.0) * r02 * x1242)) + (((-1.0) * cj4 * r01 * x1238)) + ((r00 * x1235)) + (((-1.0) * x1225 * x1229)) + (((-1.0) * r01 * x1239)) + ((x1225 * x1226)) + ((x1226 * x1231)) + (((-1.0) * x1229 * x1231)) + (((-1.0) * x1224 * x1241)));
                                                    evalcond[5] = ((((-1.0) * cj4 * x1230 * x1238)) + (((-1.0) * x1230 * x1239)) + (((-1.0) * x1228 * x1241)) + (((-1.0) * x1226 * x1234)) + ((r10 * x1235)) + (((-1.0) * r12 * x1242)) + ((x1226 * x1233)) + (((-1.0) * x1229 * x1233)) + ((x1229 * x1234)));
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
        x0 = IKcos(j[0]);
        x1 = IKsin(j[0]);
        x2 = IKcos(j[1]);
        x3 = IKcos(j[2]);
        x4 = IKsin(j[1]);
        x5 = IKsin(j[2]);
        x6 = IKcos(j[3]);
        x7 = IKsin(j[3]);
        x8 = IKsin(j[5]);
        x9 = IKcos(j[5]);
        x10 = IKcos(j[4]);
        x11 = IKsin(j[4]);
        x12 = ((0.999999500000375) * x0);
        x13 = ((0.000999999500000375) * x1);
        x14 = ((0.000670999664500252) * x1);
        x15 = ((0.670999664500252) * x0);
        x16 = ((0.1135) * x3);
        x17 = ((1.0) * x6);
        x18 = ((1.0) * x7);
        x19 = (x2 * x3);
        x20 = (x2 * x5);
        x21 = (x3 * x4);
        x22 = ((-0.0676) * x6);
        x23 = (x4 * x5);
        x24 = ((-3.7e-7) * x7);
        x25 = ((-1.0) * x7);
        x26 = ((-0.0676) * x7);
        x27 = ((-1.0) * x6);
        x28 = ((((-1.0) * x12)) + x13);
        x29 = ((((-1.0) * x13)) + x12);
        x30 = ((1.0) * x21);
        x31 = ((((0.999999500000375) * x1)) + (((0.000999999500000375) * x0)));
        x32 = ((((0.670999664500252) * x1)) + (((0.000670999664500252) * x0)));
        x33 = ((-1.0) * x31);
        x34 = ((1.0) * x28);
        x35 = (x11 * x28);
        x36 = (x28 * x3);
        x37 = (x28 * x4);
        x38 = (x2 * x28);
        x39 = ((-1.0) * x28);
        x40 = (x11 * x33);
        x41 = ((((-1.0) * x30)) + x20);
        x42 = ((((-1.0) * x20)) + x30);
        x43 = (x19 * x28);
        x44 = (x23 * x28);
        x45 = ((((-1.0) * x23)) + (((-1.0) * x19)));
        x46 = (x41 * x6);
        x47 = (x45 * x7);
        x48 = (x44 + x43);
        x49 = (((x19 * x31)) + ((x23 * x31)));
        x50 = ((((-1.0) * x28 * x30)) + (((-1.0) * x20 * x29)));
        x51 = (x48 * x6);
        x52 = (((x21 * x39)) + (((-1.0) * x20 * x29)));
        x53 = ((((-1.0) * x30 * x31)) + (((-1.0) * x20 * x33)));
        IkReal x60 = ((1.0) * x31);
        x54 = ((((-1.0) * x19 * x60)) + (((-1.0) * x23 * x60)));
        x55 = (x49 * x6);
        x56 = ((((-1.0) * x18 * x45)) + (((-1.0) * x17 * x41)));
        x57 = (x53 * x7);
        x58 = ((((-1.0) * x18 * x50)) + (((-1.0) * x17 * x48)));
        x59 = ((((-1.0) * x18 * x53)) + (((-1.0) * x17 * x49)));
        eerot[0] = (((x9 * ((((x53 * x6)) + ((x54 * x7)))))) + ((x8 * ((((x10 * x59)) + x35)))));
        eerot[1] = (((x8 * (((((-1.0) * x18 * x54)) + (((-1.0) * x17 * x53)))))) + ((x9 * ((((x10 * ((((x27 * x49)) + ((x25 * x53)))))) + x35)))));
        eerot[2] = (((x11 * x59)) + ((x10 * x29)));
        IkReal x61 = (x16 * x31);
        eetrans[0] = ((((-0.000131999965500026) * x1)) + (((0.131999935500026) * x0)) + (((-1.0) * x20 * x32)) + ((x10 * (((((-6.75999662000253e-5) * x1)) + (((0.0675999662000253) * x0)))))) + ((x21 * x32)) + ((x7 * (((((0.1135) * x23 * x31)) + ((x2 * x61)))))) + ((x6 * ((((x4 * x61)) + (((0.1135) * x20 * x33)))))) + ((x11 * (((((-0.0676) * x55)) + (((-0.0676) * x57)))))) + ((x11 * (((((-3.69999815000139e-7) * x0)) + (((3.69999815000139e-10) * x1)))))) + ((x10 * ((((x24 * x53)) + (((-3.7e-7) * x55)))))) + ((x4 * (((((0.000685999657000257) * x0)) + (((0.685999657000257) * x1)))))));
        IkReal x62 = ((1.0) * x34);
        eerot[3] = (((x8 * ((x40 + ((x10 * ((((x27 * x48)) + ((x25 * x52)))))))))) + ((x9 * ((((x7 * (((((-1.0) * x19 * x62)) + (((-1.0) * x23 * x62)))))) + ((x52 * x6)))))));
        eerot[4] = (((x9 * ((((x10 * x58)) + x40)))) + ((x8 * ((((x27 * x50)) + ((x25 * ((((x19 * x39)) + ((x23 * x39)))))))))));
        eerot[5] = (((x10 * x31)) + ((x11 * x58)));
        eetrans[1] = ((((0.131999935500026) * x1)) + ((x10 * (((((6.75999662000253e-5) * x0)) + (((0.0675999662000253) * x1)))))) + ((x11 * (((((-3.69999815000139e-10) * x0)) + (((-3.69999815000139e-7) * x1)))))) + ((x10 * ((((x24 * x52)) + (((-3.7e-7) * x51)))))) + ((x6 * ((((x16 * x37)) + (((0.1135) * x20 * x29)))))) + ((x7 * (((((0.1135) * x44)) + ((x16 * x38)))))) + ((x20 * (((((-1.0) * x14)) + x15)))) + ((x11 * ((((x26 * x52)) + ((x22 * x48)))))) + ((x4 * (((((-0.685999657000257) * x0)) + (((0.000685999657000257) * x1)))))) + (((0.000131999965500026) * x0)) + ((x21 * (((((-1.0) * x15)) + x14)))));
        eerot[6] = (((x10 * x56 * x8)) + ((x9 * ((((x42 * x7)) + ((x45 * x6)))))));
        eerot[7] = (((x10 * x9 * ((((x27 * x41)) + ((x25 * x45)))))) + ((x8 * (((((-1.0) * x18 * x42)) + (((-1.0) * x17 * x45)))))));
        eerot[8] = (x11 * x56);
        eetrans[2] = ((0.1393) + ((x7 * (((((-1.0) * x16 * x4)) + (((0.1135) * x20)))))) + ((x6 * ((((x16 * x2)) + (((0.1135) * x23)))))) + (((0.671) * x19)) + (((0.671) * x23)) + ((x10 * (((((-3.7e-7) * x47)) + (((-3.7e-7) * x46)))))) + (((0.686) * x2)) + ((x11 * ((((x26 * x45)) + ((x22 * x41)))))));
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
}
// namespace ik_avena
