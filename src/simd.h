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


#ifndef TMM2_SIMD_H
#define TMM2_SIMD_H

#include <cstdint>

#if defined(__AVX2__)
#include <immintrin.h>
#else
#include <emmintrin.h>
#endif

#define F_INLINE __forceinline

template <typename V>
static F_INLINE V setzero();

template <typename V>
static F_INLINE V set1(int8_t x);

template <typename V>
static F_INLINE V loada(const uint8_t* p);

template <typename V>
static F_INLINE V loadu(const uint8_t* p);


template <>
F_INLINE __m128i setzero()
{
    return _mm_setzero_si128();
}

template <>
F_INLINE __m128i set1(int8_t x)
{
    return _mm_set1_epi8(x);
}

template <>
F_INLINE __m128i loada(const uint8_t* p)
{
    return _mm_load_si128(reinterpret_cast<const __m128i*>(p));
}

template <>
F_INLINE __m128i loadu(const uint8_t* p)
{
    return _mm_loadu_si128(reinterpret_cast<const __m128i*>(p));
}


static F_INLINE void stream(uint8_t* p, const __m128i& x)
{
    _mm_stream_si128(reinterpret_cast<__m128i*>(p), x);
}

static F_INLINE __m128i min(const __m128i& x, const __m128i& y)
{
    return _mm_min_epu8(x, y);
}

static F_INLINE __m128i max(const __m128i& x, const __m128i& y)
{
    return _mm_max_epu8(x, y);
}

static F_INLINE __m128i adds(const __m128i& x, const __m128i& y)
{
    return _mm_adds_epu8(x, y);
}

static F_INLINE __m128i subs(const __m128i& x, const __m128i& y)
{
    return _mm_subs_epu8(x, y);
}

static F_INLINE __m128i sub(const __m128i& x, const __m128i& y)
{
    return _mm_sub_epi8(x, y);
}

static F_INLINE __m128i avg(const __m128i& x, const __m128i& y)
{
    return _mm_avg_epu8(x, y);
}

static F_INLINE __m128i or_reg(const __m128i& x, const __m128i& y)
{
    return _mm_or_si128(x, y);
}

static F_INLINE __m128i xor_reg(const __m128i& x, const __m128i& y)
{
    return _mm_xor_si128(x, y);
}

static F_INLINE __m128i and_reg(const __m128i& x, const __m128i& y)
{
    return _mm_and_si128(x, y);
}

static F_INLINE __m128i andnot(const __m128i& x, const __m128i& y)
{
    return _mm_andnot_si128(x, y);
}

static F_INLINE __m128i cmpeq(const __m128i& x, const __m128i& y)
{
    return _mm_cmpeq_epi8(x, y);
}

static F_INLINE __m128i cmpgt(const __m128i& x, const __m128i& y)
{
    return _mm_cmpgt_epi8(x, y);
}

static F_INLINE __m128i
blendv(const __m128i& x, const __m128i& y, const __m128i& m)
{
    return _mm_or_si128(_mm_and_si128(m, y), _mm_andnot_si128(m, x));
}

static F_INLINE __m128i rshift2(const __m128i& x, const __m128i& two, const __m128i& mask)
{
    return _mm_and_si128(_mm_srli_epi16(_mm_adds_epu8(x, two), 2), mask);
}


#if defined(__AVX2__)

template <>
F_INLINE __m256i setzero()
{
    return _mm256_setzero_si256();
}

template <>
F_INLINE __m256i set1(int8_t x)
{
    return _mm256_set1_epi8(x);
}

template <>
F_INLINE __m256i loada(const uint8_t* p)
{
    return _mm256_load_si256(reinterpret_cast<const __m256i*>(p));
}

template <>
F_INLINE __m256i loadu(const uint8_t* p)
{
    return _mm256_loadu_si256(reinterpret_cast<const __m256i*>(p));
}


static F_INLINE void stream(uint8_t* p, const __m256i& x)
{
    _mm256_stream_si256(reinterpret_cast<__m256i*>(p), x);
}

static F_INLINE __m256i min(const __m256i& x, const __m256i& y)
{
    return _mm256_min_epu8(x, y);
}

static F_INLINE __m256i max(const __m256i& x, const __m256i& y)
{
    return _mm256_max_epu8(x, y);
}

static F_INLINE __m256i adds(const __m256i& x, const __m256i& y)
{
    return _mm256_adds_epu8(x, y);
}

static F_INLINE __m256i subs(const __m256i& x, const __m256i& y)
{
    return _mm256_subs_epu8(x, y);
}

static F_INLINE __m256i sub(const __m256i& x, const __m256i& y)
{
    return _mm256_sub_epi8(x, y);
}

static F_INLINE __m256i avg(const __m256i& x, const __m256i& y)
{
    return _mm256_avg_epu8(x, y);
}

static F_INLINE __m256i or_reg(const __m256i& x, const __m256i& y)
{
    return _mm256_or_si256(x, y);
}

static F_INLINE __m256i xor_reg(const __m256i& x, const __m256i& y)
{
    return _mm256_xor_si256(x, y);
}

static F_INLINE __m256i and_reg(const __m256i& x, const __m256i& y)
{
    return _mm256_and_si256(x, y);
}

static F_INLINE __m256i andnot(const __m256i& x, const __m256i& y)
{
    return _mm256_andnot_si256(x, y);
}

static F_INLINE __m256i cmpeq(const __m256i& x, const __m256i& y)
{
    return _mm256_cmpeq_epi8(x, y);
}

static F_INLINE __m256i cmpgt(const __m256i& x, const __m256i& y)
{
    return _mm256_cmpgt_epi8(x, y);
}

static F_INLINE __m256i
blendv(const __m256i& x, const __m256i& y, const __m256i& m)
{
    return _mm256_blendv_epi8(x, y, m);
}

static F_INLINE __m256i rshift2(const __m256i& x, const __m256i& two, const __m256i& mask)
{
    return _mm256_and_si256(mask, _mm256_srli_epi16(_mm256_adds_epu8(x, two), 2));
}
#endif // __AVX2__


template <typename V>
static F_INLINE V absdiff(const V& x, const V& y)
{
    return or_reg(subs(x, y), subs(y, x));
}

template <typename V>
static F_INLINE V cmple(const V& x, const V& y)
{
    return cmpeq(min(x, y), x);
}

template <typename V>
static F_INLINE V cmpge(const V& x, const V& y)
{
    return cmpeq(max(x, y), x);
}

template <typename V>
static F_INLINE V cmpneq(const V& x, const V& y)
{
    return xor_reg(cmpeq(x, y), cmpeq(x, x));
}

template <typename V>
static F_INLINE V min(const V& a, const V& b, const V& c, const V& d)
{
    return min(min(a, b), min(c, d));
}

template <typename V>
static F_INLINE V max(const V& a, const V& b, const V& c, const V& d)
{
    return max(max(a, b), max(c, d));
}

template <typename V>
static F_INLINE V and3(const V& x, const V& y, const V& z)
{
    return and_reg(and_reg(x, y), z);
}

#endif // TMM2_SIMD_H
