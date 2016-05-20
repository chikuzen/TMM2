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


#ifndef PROC_THRESH_MASK_H
#define PROC_THRESH_MASK_H


static __forceinline int absd(int x, int y) noexcept
{
    return x > y ? x - y : y - x;
}

template <int TTYPE>
static __forceinline int
get_at4_c(const int c, const int u, const int d, const int l, const int r,
    const int hs, const int vs) noexcept
{
    int min0 = u, max0 = u;

    if (TTYPE == 0) {
        int min1 =l, max1 = l;
        if (min1 > r) min1 = r;
        if (max1 < r) max1 = r;
        if (min0 > d) min0 = d;
        if (max0 < d) max0 = d;
        min0 = (absd(c, min0) + vs) >> vs;
        max0 = (absd(c, max0) + vs) >> vs;
        min1 = (absd(c, min1) + hs) >> hs;
        max1 = (absd(c, max1) + hs) >> hs;
        return std::max(std::max(min0, max0), std::max(min1, max1));
    }
    if (max0 < l) max0 = l;
    if (min0 > l) min0 = l;
    if (max0 < r) max0 = r;
    if (min0 > r) min0 = r;
    if (min0 > d) min0 = d;
    if (max0 < d) max0 = d;
    if (TTYPE == 2) {
        return std::max(absd(c, min0), absd(c, max0));
    }
    if (max0 < c) max0 = c;
    if (min0 > c) min0 = c;
    return max0 - min0;
}


template <int TTYPE>
static __forceinline int
get_at8_c(const int c, const int ul, const int u, const int ur, const int l,
    const int r, const int dl, const int d, const int dr, const int hs,
    const int vs) noexcept
{
    int min0 = ul, max0 = ul;
    if (max0 < u)  max0 = u;
    if (min0 > u)  min0 = u;
    if (max0 < ur) max0 = ur;
    if (min0 > ur) min0 = ur;

    if (TTYPE == 1) {
        int min1 = l, max1 = l;
        if (min1 > r) min1 = r;
        if (max1 < r) max1 = r;
        if (max0 < dl) max0 = dl;
        if (min0 > dl) min0 = dl;
        if (max0 < d)  max0 = d;
        if (min0 > d)  min0 = d;
        if (max0 < dr) max0 = dr;
        if (min0 > dr) min0 = dr;
        min0 = (absd(c, min0) + vs) >> vs;
        max0 = (absd(c, max0) + vs) >> vs;
        min1 = (absd(c, min1) + hs) >> hs;
        max1 = (absd(c, max1) + hs) >> hs;
        return std::max(std::max(min0, max0), std::max(min1, max1));
    }
    if (max0 < l) max0 = l;
    if (min0 > l) min0 = l;
    if (max0 < r) max0 = r;
    if (min0 > r) min0 = r;
    if (max0 < dl) max0 = dl;
    if (min0 > dl) min0 = dl;
    if (max0 < d)  max0 = d;
    if (min0 > d)  min0 = d;
    if (max0 < dr) max0 = dr;
    if (min0 > dr) min0 = dr;
    if (TTYPE == 3) {
        return std::max(absd(c, min0), absd(c, max0));
    }
    if (max0 < c) max0 = c;
    if (min0 > c) min0 = c;
    return max0 - min0;
}



template <int TTYPE>
static void __stdcall
proc_c(uint8_t* dqp, const uint8_t* srcp, const int pitch,
    const int width, const int height, const int hs, const int vs)
    noexcept
{
    uint8_t* dhp = dqp + pitch * height;

    const uint8_t* s0 = srcp + pitch;
    const uint8_t* s1 = srcp;
    const uint8_t* s2 = srcp + pitch;

    for (int y = 0; y < height; ++y) {
        for (int x = 0; x < width; ++x) {
            int at;
            if (TTYPE == 0 || TTYPE == 2 || TTYPE == 4) {
                at = get_at4_c<TTYPE>(
                    s1[x], s0[x], s2[x], s1[x - 1], s1[x + 1], hs, vs);
            } else {
                at = get_at8_c<TTYPE>(
                    s1[x], s0[x - 1], s0[x], s0[x + 1], s1[x - 1], s1[x + 1],
                    s2[x - 1], s2[x], s2[x + 1], hs, vs);
            }
            dqp[x] = (at + 2) / 4;
            dhp[x] = (at + 1) / 2;
        }
        s0 = s1;
        s1 = s2;
        s2 += (y < height - 1) ? pitch : -pitch;
        dqp += pitch;
        dhp += pitch;
    }
}


#include "simd.h"


template <typename V, int TTYPE, int HS, int VS>
static __forceinline V
get_at_simd(const uint8_t* s0, const uint8_t* s1, const uint8_t* s2,
            const V& zero, const V& two, const V& mask) noexcept
{
    if (TTYPE == 0 || TTYPE == 2 || TTYPE == 4) {
        V up = loada<V>(s0);
        V left = loadu<V>(s1 - 1);
        V center = loada<V>(s1);
        V right = loadu<V>(s1 + 1);
        V down = loada<V>(s2);

        if (TTYPE == 0) {
            V min0 = absdiff(min(up, down), center);
            V max0 = absdiff(max(up, down), center);
            if (VS == 1) {
                min0 = avg(min0, zero);
                max0 = avg(max0, zero);
            }
            if (VS == 2) {
                min0 = rshift2(min0, two, mask);
                max0 = rshift2(max0, two, mask);
            }
            V min1 = absdiff(min(left, right), center);
            V max1 = absdiff(max(left, right), center);
            if (HS == 1) {
                min1 = avg(min1, zero);
                max1 = avg(max1, zero);
            }
            return max(min0, max0, min1, max1);
        } else if (TTYPE == 2) {
            V min0 = absdiff(min(left, right, up, down), center);
            V max0 = absdiff(max(left, right, up, down), center);
            return max(min0, max0);
        } else if (TTYPE == 4) {
            V min0 = min(min(up, down, left, right), center);
            V max0 = max(max(up, down, left, right), center);
            return subs(max0, min0);
        }
    } else {
        V ul = loadu<V>(s0 - 1);
        V up = loada<V>(s0);
        V ur = loadu<V>(s0 + 1);
        V left = loadu<V>(s1 - 1);
        V center = loada<V>(s1);
        V right = loadu<V>(s1 + 1);
        V dl = loadu<V>(s2 - 1);
        V down = loada<V>(s2);
        V dr = loadu<V>(s2 + 1);

        if (TTYPE == 1) {
            V min0 = absdiff(min(min(ul, ur, dl, dr), min(up, down)), center);
            V max0 = absdiff(max(max(ul, ur, dl, dr), max(up, down)), center);
            if (VS == 1) {
                min0 = avg(min0, zero);
                max0 = avg(max0, zero);
            }
            if (VS == 2) {
                min0 = rshift2(min0, two, mask);
                max0 = rshift2(max0, two, mask);
            }
            V min1 = absdiff(min(left, right), center);
            V max1 = absdiff(max(left, right), center);
            if (HS == 1) {
                min1 = avg(min1, zero);
                max1 = avg(max1, zero);
            }
            return max(min0, max0, min1, max1);
        } else if (TTYPE == 3) {
            V min0 = min(min(ul, ur, dl, dr), min(up, down, left, right));
            V max0 = max(max(ul, ur, dl, dr), max(up, down, left, right));
            return max(absdiff(min0, center), absdiff(max0, center));
        } else {
            V min0 = min(min(min(ul, ur, dl, dr), min(up, down, left, right)), center);
            V max0 = max(max(max(ul, ur, dl, dr), max(up, down, left, right)), center);
            return subs(max0, min0);
        }
    }
}


template <typename V, int TTYPE, int HS, int VS>
static void __stdcall
proc_simd(uint8_t* dqp, const uint8_t* srcp, const int pitch,
                 const int width, const int height, const int hs, const int vs)
    noexcept
{
    uint8_t* dhp = dqp + pitch * height;

    const uint8_t* s0 = srcp + pitch;
    const uint8_t* s1 = srcp;
    const uint8_t* s2 = srcp + pitch;

    const V zero = setzero<V>();
    const V two = set1<V>(0x02);
    const V mask = set1<V>(0x3F);

    for (int y = 0; y < height; ++y) {
        for (int x = 0; x < width; x += sizeof(V)) {
            V at = get_at_simd<V, TTYPE, HS, VS>(
                s0 + x, s1 + x, s2 + x, zero, two, mask);
            V dq = rshift2(at, two, mask);
            V dh = avg(at, zero);

            stream(dqp + x, dq);
            stream(dhp + x, dh);
        }
        s0 = s1;
        s1 = s2;
        s2 += (y < height - 1) ? pitch : -pitch;
        dqp += pitch;
        dhp += pitch;
    }
}



#endif // PROC_THRESH_MASK_H

