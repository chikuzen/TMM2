/*
TMM2 - rewrite of TMM for Avisynth2.6/Avisynth+.
Copyright (C) 2016 OKA Motofumi

TMM - builds a motion-mask for TDeint.
Copyright (C) 2007 Kevin Stone

This program is free software; you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation; either version 2 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program; if not, write to the Free Software
Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
*/


#include "TMM2.h"
#include "proc_thmask.h"



static const proc_thmask functions[] = {
    proc_c<0>, proc_c<1>, proc_c<2>, proc_c<3>, proc_c<4>, proc_c<5>,
    // template <typename V, int TTYPE, int HS, int VS>
    proc_simd<__m128i, 0, 0, 1>, proc_simd<__m128i, 1, 0, 1>,
    proc_simd<__m128i, 2, 0, 1>, proc_simd<__m128i, 3, 0, 1>,
    proc_simd<__m128i, 4, 0, 1>, proc_simd<__m128i, 5, 0, 1>,

    proc_simd<__m128i, 0, 0, 2>, proc_simd<__m128i, 1, 0, 2>,
    proc_simd<__m128i, 2, 0, 2>, proc_simd<__m128i, 3, 0, 2>,
    proc_simd<__m128i, 4, 0, 2>, proc_simd<__m128i, 5, 0, 2>,

    proc_simd<__m128i, 0, 1, 1>, proc_simd<__m128i, 1, 1, 1>,
    proc_simd<__m128i, 2, 1, 1>, proc_simd<__m128i, 3, 1, 1>,
    proc_simd<__m128i, 4, 1, 1>, proc_simd<__m128i, 5, 1, 1>,

    proc_simd<__m128i, 0, 1, 2>, proc_simd<__m128i, 1, 1, 2>,
    proc_simd<__m128i, 2, 1, 2>, proc_simd<__m128i, 3, 1, 2>,
    proc_simd<__m128i, 4, 1, 2>, proc_simd<__m128i, 5, 1, 2>,
#if defined(__AVX2__)
    proc_simd<__m256i, 0, 0, 1>, proc_simd<__m256i, 1, 0, 1>,
    proc_simd<__m256i, 2, 0, 1>, proc_simd<__m256i, 3, 0, 1>,
    proc_simd<__m256i, 4, 0, 1>, proc_simd<__m256i, 5, 0, 1>,

    proc_simd<__m256i, 0, 0, 2>, proc_simd<__m256i, 1, 0, 2>,
    proc_simd<__m256i, 2, 0, 2>, proc_simd<__m256i, 3, 0, 2>,
    proc_simd<__m256i, 4, 0, 2>, proc_simd<__m256i, 5, 0, 2>,

    proc_simd<__m256i, 0, 1, 1>, proc_simd<__m256i, 1, 1, 1>,
    proc_simd<__m256i, 2, 1, 1>, proc_simd<__m256i, 3, 1, 1>,
    proc_simd<__m256i, 4, 1, 1>, proc_simd<__m256i, 5, 1, 1>,

    proc_simd<__m256i, 0, 1, 2>, proc_simd<__m256i, 1, 1, 2>,
    proc_simd<__m256i, 2, 1, 2>, proc_simd<__m256i, 3, 1, 2>,
    proc_simd<__m256i, 4, 1, 2>, proc_simd<__m256i, 5, 1, 2>,
#endif // __AVX2__
};


ThreshMask::
ThreshMask(PClip c, int ttype, int mtql, int mthl, int mtqc, int mthc,
           arch_t arch) :
    GVFmod(c, arch)
{
    mtq[0] = mtql; mtq[1] = mtq[2] = mtqc;
    mth[0] = mthl; mth[1] = mth[2] = mthc;

    hs[0] = 0;
    hs[1] = hs[2] = vi.IsYV24() ? 0 : 1;
    vs[0] = 1;
    vs[1] = vs[2] = vi.IsYV12() ? 2 : 1;

    vi.width += 4; // prepare for image edge processing
    vi.height *= 3;

    for (int i = 0; i < 3; ++i) {
        if (arch == NO_SIMD) {
            proc[i] = functions[ttype];
            continue;
        }
        int idx = arch == USE_SSE2 ? 6 : 30;
        idx += hs[i] * 12;
        idx += vs[i] == 2 ? 6 : 0;
        proc[i] = functions[idx + ttype];
    }

    child->SetCacheHints(CACHE_NOTHING, 0);
}


static inline void
mirror_copy(uint8_t* dstp, const int dpitch, const uint8_t* srcp,
            const int spitch, const int width, const int height) noexcept
{
    for (int y = 0; y < height; ++y) {
        memcpy(dstp, srcp, width);
        dstp[-1] = dstp[1];
        dstp[width] = dstp[width - 2];
        dstp += dpitch;
        srcp += spitch;
    }
}


PVideoFrame __stdcall ThreshMask::GetFrame(int n, ise_t* env)
{
    auto src = child->GetFrame(n, env);
    auto dst = env->NewVideoFrame(vi, align);

    for (int p = 0; p < numPlanes; ++p) {
        int plane = planes[p];

        const int pitch = dst->GetPitch(plane);
        const int width = src->GetRowSize(plane);
        const int height = src->GetHeight(plane);

        uint8_t* dstp = dst->GetWritePtr(plane);

        mirror_copy(dstp + pitch * height * 2, pitch, src->GetReadPtr(plane),
                    src->GetPitch(plane), width, height);

        const uint8_t* srcp = dstp + pitch * height * 2;

        if (mtq[p] > -1 && mth[p] > -1) {
            memset(dstp, mtq[p], pitch * height);
            memset(dstp + pitch * height, mth[p], pitch* height);
            continue;
        }

        proc[p](dstp, srcp, pitch, width, height, hs[p], vs[p]);

        if (mtq[p] > -1) {
            memset(dstp, mtq[p], pitch * height);
        } else if (mth[p] > -1) {
            memset(dstp + pitch * height, mth[p], pitch * height);
        }
    }

    return dst;
}