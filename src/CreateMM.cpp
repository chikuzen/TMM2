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


static void __stdcall
and_masks_c(uint8_t* dstp0, uint8_t* dstp1, const uint8_t* srcp0,
            const uint8_t* srcp1, const uint8_t* srcp2, const int dpitch,
            const int spitch0, const int spitch1, const int spitch2,
            const int width, const int height) noexcept
{
    for (int y = 0; y < height; ++y) {
        for (int x = 0; x < width; ++x) {
            dstp0[x] = srcp0[x] & srcp1[x] & srcp2[x];
        }
        dstp0[-1] = dstp0[1];
        dstp0[width] = dstp0[width - 2];
        srcp0 += spitch0;
        srcp1 += spitch1;
        srcp2 += spitch2;
        dstp0 += dpitch;
    }
    for (int y = 0; y < height; ++y) {
        for (int x = 0; x < width; ++x) {
            dstp1[x] = srcp0[x] & srcp1[x] & srcp2[x];
        }
        dstp1[-1] = dstp1[1];
        dstp1[width] = dstp1[width - 2];
        srcp0 += spitch0;
        srcp1 += spitch1;
        srcp2 += spitch2;
        dstp1 += dpitch;
    }
}


static void __stdcall
combine_masks_c(uint8_t* dstp, const uint8_t* sqp, const uint8_t* shp,
    const int dpitch, const int spitch, const int width,
    const int height, const int cstr) noexcept
{
    const uint8_t* sqp0 = sqp + spitch;
    const uint8_t* sqp1 = sqp;
    const uint8_t* sqp2 = sqp + spitch;

    for (int y = 0; y < height; ++y) {
        for (int x = 0; x < width; ++x) {
            if (sqp1[x] > 0) {
                dstp[x] = 0xFF;
                continue;
            }
            if (shp[x] == 0) {
                dstp[x] = 0x00;
                continue;
            }
            int count = sqp0[x - 1] + sqp0[x] + sqp0[x + 1] + sqp1[x - 1]
                + sqp1[x + 1] + sqp2[x - 1] + sqp2[x] + sqp2[x + 1];
            dstp[x] = count >= cstr ? 0xFF : 0x00;
        }
        sqp0 = sqp1;
        sqp1 = sqp2;
        sqp2 += y < height - 1 ? spitch: -spitch;
        shp += spitch;
        dstp += dpitch;
    }
}


#include "simd.h"

template <typename V>
static void __stdcall
and_masks_simd(uint8_t* dstp0, uint8_t* dstp1, const uint8_t* srcp0,
               const uint8_t* srcp1, const uint8_t* srcp2, const int dpitch,
               const int spitch0, const int spitch1, const int spitch2,
               const int width, const int height) noexcept
{
    for (int y = 0; y < height; ++y) {
        for (int x = 0; x < width; x += sizeof(V)) {
            V s0 = loada<V>(srcp0 + x);
            V s1 = loada<V>(srcp1 + x);
            V s2 = loada<V>(srcp2 + x);
            stream(dstp0 + x, and3(s0, s1, s2));
        }
        dstp0[-1] = dstp0[1];
        dstp0[width] = dstp0[width - 2];
        srcp0 += spitch0;
        srcp1 += spitch1;
        srcp2 += spitch2;
        dstp0 += dpitch;
    }
    for (int y = 0; y < height; ++y) {
        for (int x = 0; x < width; x += sizeof(V)) {
            V s0 = loada<V>(srcp0 + x);
            V s1 = loada<V>(srcp1 + x);
            V s2 = loada<V>(srcp2 + x);
            stream(dstp1 + x, and3(s0, s1, s2));
        }
        dstp1[-1] = dstp1[1];
        dstp1[width] = dstp1[width - 2];
        srcp0 += spitch0;
        srcp1 += spitch1;
        srcp2 += spitch2;
        dstp1 += dpitch;
    }
}


template <typename V>
static void __stdcall
combine_masks_simd(uint8_t* dstp, const uint8_t* sqp, const uint8_t* shp,
                   const int dpitch, const int spitch, const int width,
                   const int height, const int _c) noexcept
{
    const uint8_t* sqp0 = sqp + spitch;
    const uint8_t* sqp1 = sqp;
    const uint8_t* sqp2 = sqp + spitch;

    const V zero = setzero<V>();
    const V cstr = set1<V>(static_cast<int8_t>(_c));

    for (int y = 0; y < height; ++y) {
        for (int x = 0; x < width; x += sizeof(V)) {
            V count =           loadu<V>(sqp0 + x - 1);
            count = adds(count, loada<V>(sqp0 + x));
            count = adds(count, loadu<V>(sqp0 + x + 1));
            count = adds(count, loadu<V>(sqp1 + x - 1));
            V sq = loada<V>(sqp1 + x);
            count = adds(count, loadu<V>(sqp1 + x + 1));
            count = adds(count, loadu<V>(sqp2 + x - 1));
            count = adds(count, loada<V>(sqp2 + x));
            count = adds(count, loadu<V>(sqp2 + x + 1));

            count = cmpeq(max(count, cstr), count);
            sq = cmpgt(sq, zero);
            V sh = loada<V>(shp + x);
            sh = cmpgt(sh, zero);

            V count2 = and_reg(count, sh);
            V count3 = or_reg(count2, sq);

            stream(dstp + x, count3);
        }
        sqp0 = sqp1;
        sqp1 = sqp2;
        sqp2 += y < height - 1 ? spitch: -spitch;
        shp += spitch;
        dstp += dpitch;
    }
}


CreateMM::CreateMM(PClip mm1, PClip mm2, int _cstr, arch_t arch) :
    GVFmod(mm1, arch), mmask2(mm2), cstr(_cstr), simd(arch != NO_SIMD)
{
    vi.height /= 2;

    switch (arch) {
#if defined(__AVX2__)
    case USE_AVX2:
        and_masks = and_masks_simd<__m256i>;
        combine_masks = combine_masks_simd<__m256i>;
        break;
#endif
    case USE_SSE2:
        and_masks = and_masks_simd<__m128i>;
        combine_masks = combine_masks_simd<__m128i>;
        break;
    default:
        and_masks = and_masks_c;
        combine_masks = combine_masks_c;
    }

    child->SetCacheHints(CACHE_WINDOW, 3);
}


struct AndBuff {
    void* orig;
    uint8_t* am0;
    uint8_t* am1;
    const int pitch;

    AndBuff(int width, int height, size_t align) :
        pitch((width + 2 + align - 1) & ~(align - 1))
    {
        orig = _mm_malloc(pitch * (height * 2 + 1), align);
        am0 = reinterpret_cast<uint8_t*>(orig) + pitch;
        am1 = am0 + pitch * height;
    }
    ~AndBuff()
    {
        _mm_free(orig);
        orig = nullptr;
    }
};


PVideoFrame __stdcall CreateMM::GetFrame(int n, ise_t* env)
{
    auto src2 = mmask2->GetFrame(n, env);
    auto src1 = child->GetFrame(std::min(n + 1, vi.num_frames - 1), env);
    auto src0 = child->GetFrame(n, env);
    auto dst = env->NewVideoFrame(vi, align);

    auto buff = AndBuff(vi.width, vi.height, align);

    if (!buff.orig) {
        env->ThrowError("TMM: failed to allocate AndBuff.");
    }

    for (int p = 0; p < numPlanes; ++p) {
        const int plane = planes[p];

        const int width = dst->GetRowSize(plane);
        const int height = dst->GetHeight(plane);
        uint8_t* dstp = dst->GetWritePtr(plane);
        const int dpitch = dst->GetPitch(plane);

        and_masks(buff.am0, buff.am1, src0->GetReadPtr(plane),
                  src1->GetReadPtr(plane), src2->GetReadPtr(plane),
                  buff.pitch, src0->GetPitch(plane), src1->GetPitch(plane),
                  src2->GetPitch(plane), width, height);

        combine_masks(dstp, buff.am0, buff.am1, dpitch, buff.pitch, width,
                      height, cstr);
    }

    return dst;
}



