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


static const int vluts[3][64] = {
    0, 1, 2, 2, 3, 0, 2, 2,
    1, 1, 2, 2, 0, 1, 2, 2,
    2, 2, 2, 2, 2, 2, 2, 2,
    2, 2, 2, 2, 2, 2, 2, 2,
    3, 0, 2, 2, 3, 3, 2, 2,
    0, 1, 2, 2, 3, 1, 2, 2,
    2, 2, 2, 2, 2, 2, 2, 2,
    2, 2, 2, 2, 2, 2, 2, 2,

    0, 0, 2, 2, 0, 0, 2, 2,
    0, 1, 2, 2, 0, 1, 2, 2,
    2, 2, 2, 2, 2, 2, 2, 2,
    2, 2, 2, 2, 2, 2, 2, 2,
    0, 0, 2, 2, 3, 3, 2, 2,
    0, 1, 2, 2, 3, 1, 2, 2,
    2, 2, 2, 2, 2, 2, 2, 2,
    2, 2, 2, 2, 2, 2, 2, 2,

    0, 0, 0, 0, 0, 0, 0, 0,
    0, 1, 0, 1, 0, 1, 0, 1,
    0, 0, 2, 2, 0, 0, 2, 2,
    0, 1, 2, 2, 0, 1, 2, 2,
    0, 0, 0, 0, 3, 3, 3, 3,
    0, 1, 0, 1, 3, 1, 3, 1,
    0, 0, 2, 2, 3, 3, 2, 2,
    0, 1, 2, 2, 3, 1, 2, 2,
};

static const int tmmlut16[2][8] = {
    60, 20, 50, 10, 60, 10, 40, 30,
    60, 10, 40, 30, 60, 20, 50, 10,
};


void BuildMM::setVals()
{
    int tn = order == 0 ? -1 : 0;
    vals[0].tstart = tn - (length - 2) / 2;
    vals[0].tstop = tn + (length - 2) / 2 - 1;
    vals[0].bstart = (1 - length) / 2;
    vals[0].bstop = (length - 1) / 2 - 2;
    vals[0].ocount = vals[0].bstop - vals[0].bstart + 1;
    vals[0].ccount = vals[0].tstop - vals[0].tstart + 1;

    int bn = order == 0 ? 0 : -1;
    vals[1].tstart = (1 - length) / 2;
    vals[1].tstop = (length - 1) / 2 - 2;
    vals[1].bstart = bn - (length - 2) / 2;
    vals[1].bstop = bn + (length - 2) / 2 - 1;
    vals[1].ocount = vals[1].tstop - vals[1].tstart + 1;
    vals[1].ccount = vals[1].bstop - vals[1].bstart + 1;

    int tmax0 = std::max(vals[0].tstop, -vals[0].tstart);
    int bmax0 = std::max(vals[0].bstop, -vals[0].bstart);
    int tmax1 = std::max(vals[1].tstop, -vals[1].tstart);
    int bmax1 = std::max(vals[0].tstop, -vals[0].tstart);

    child->SetCacheHints(CACHE_WINDOW, std::max(tmax0, tmax1) * 2 + 1);
    btmf->SetCacheHints(CACHE_WINDOW, std::max(bmax0, bmax1) * 2 + 1);
}


BuildMM::BuildMM(PClip tf, PClip bf, int m, int o, int f, int l, int mtype,
                 arch_t arch, ise_t* env) :
     GVFmod(tf, arch), btmf(bf), mode(m), order(o), field(f), length(l)
{
    const VideoInfo& vibf = btmf->GetVideoInfo();
    validate(vi.width != vibf.width || vi.height != vibf.height
             || vi.pixel_type != vibf.pixel_type
             || vi.num_frames != vibf.num_frames,
             "tfields and bfields clips don't match.");

    if (order == -1) {
        order = child->GetParity(0) ? 1 : 0;
    }
    if (field == -1) {
        field = order;
    }

    black = env->NewVideoFrame(vi);
    for (int p = 0; p < numPlanes; ++p) {
        int plane = planes[p];
        memset(black->GetWritePtr(plane), 0,
               black->GetPitch(plane) * black->GetHeight(plane));
    }

    nfSrc = vi.num_frames - 2;

    vi.height *= 2;
    vi.SetFieldBased(false);
    if (mode == 1) {
        vi.num_frames *= 2;
    }

    gvlut.resize(length, 2);
    gvlut[0] = 1;
    gvlut[length - 1] = 4;

    vlut = vluts[mtype];
    for (int i = 0; i < 2; ++i) {
        const int* tmmlut = tmmlut16[order] + i * 4;
        for (int j = 0; j < 64; ++j) {
            tmmlutf[i][j] = tmmlut[vlut[j]];
        }
    }

    setVals();
}


BuildMM::~BuildMM() {}


struct BMMBuffer {
    std::vector<PVideoFrame> srcs;
    std::vector<int> intVals;
    std::vector<const uint8_t*> ptrs;

    PVideoFrame* tops;
    PVideoFrame* btms;

    BMMBuffer(const int ocount, const int ccount, const int length,
              const int num_tops)
    {
        srcs.resize(ocount + ccount);
        intVals.resize(ocount + ccount + 4 * length - 2);
        ptrs.resize(2 * ocount + ccount);

        tops = srcs.data();
        btms = tops + num_tops;
    }
};


 
PVideoFrame __stdcall BuildMM::GetFrame(int n, ise_t* env)
{
    int ft = field;
    if (mode == 1) {
        ft = (n & 1) ? 1 - order : order;
        n /= 2;
    }

    const int* tmmlut = tmmlutf[ft];

    const int tstart = n + vals[ft].tstart, tstop = n + vals[ft].tstop,
        bstart = n + vals[ft].bstart, bstop = n + vals[ft].bstop,
        ocount = vals[ft].ocount, ccount = vals[ft].ccount;

    auto b = BMMBuffer(ocount, ccount, length, tstop - tstart + 1);

    int* opitch = b.intVals.data();
    int* cpitch = opitch + ocount;
    int* plut0 = cpitch + ccount;
    int* plut1 = plut0 + 2 * length - 1;

    const uint8_t** srcp0 = b.ptrs.data();
    const uint8_t** srcp1 = srcp0 + ocount;
    const uint8_t** srcp2 = srcp1 + ccount;

    int nf = vi.num_frames - 1;
    for (int i = tstop; i >= tstart; --i) {
        if (i < 0 || i >= nfSrc) {
            b.tops[i - tstart] = black;
            continue;
        }
        b.tops[i - tstart] = child->GetFrame(clamp(i, 0, nf), env);
    }
    for (int i = bstop; i >= bstart; --i) {
        if (i < 0 || i >= nfSrc) {
            b.btms[i - bstart] = black;
            continue;
        }
        b.btms[i - bstart] = btmf->GetFrame(clamp(i, 0, nf), env);
    }
    PVideoFrame* oclips = ft == 0 ? b.btms : b.tops;
    PVideoFrame* cclips = ft == 0 ? b.tops : b.btms;

    auto dst = env->NewVideoFrame(vi);

    const int offo = (length & 1) ? 0 : 1;
    const int offc = !offo;

    for (int p = 0; p < numPlanes; ++p) {
        const int plane = planes[p];

        uint8_t* dstp = dst->GetWritePtr(plane);
        const int dpitch = dst->GetPitch(plane);
        const int width = dst->GetRowSize(plane);
        const int height = dst->GetHeight(plane);

        for (int i = !ft; i < height; i += 2) {
            memset(dstp + dpitch * i, 10, width);
        }
        dstp += dpitch * ft;

        for (int i = 0; i < ocount; ++i) {
            opitch[i] = oclips[i]->GetPitch(plane);
            srcp0[i] = oclips[i]->GetReadPtr(plane);
            srcp2[i] = srcp0[i] + opitch[i] * ft;
        }
        for (int i = 0; i < ccount; ++i) {
            srcp1[i] = cclips[i]->GetReadPtr(plane);
            cpitch[i] = cclips[i]->GetPitch(plane);
        }

        const int ct = ccount / 2;

        for (int y = ft; y < height; y += 2) {
            for (int x = 0; x < width; ++x) {

                if (srcp1[ct - 2][x] == 0 && srcp1[ct][x] == 0
                        && srcp1[ct + 1][x] == 0) {
                    dstp[x] = 60;
                    continue;
                }

                for (int i = 0; i < ccount; ++i) {
                    plut0[i * 2 + offc] = plut1[i * 2 + offc] = srcp1[i][x];
                }

                for (int i = 0; i < ocount; ++i) {
                    plut0[i * 2 + offo] = srcp0[i][x];
                    plut1[i * 2 + offo] = srcp2[i][x];
                }

                int val = 0;
                for (int i = 0; i < length; ++i) {
                    for (int j = 0; j < length - 4; ++j) {
                        if (plut0[i + j] == 0) goto j1;
                    }
                    val |= (gvlut[i] << 3);
                j1:
                    for (int j = 0; j < length - 4; ++j) {
                        if (plut1[i + j] == 0) goto j2;
                    }
                    val |= gvlut[i];
                j2:
                    if (vlut[val] == 2) break;
                }
                dstp[x] = tmmlut[val];
            }
            dstp += dpitch * 2;
            for (int i = 0; i < ccount; ++i) {
                srcp1[i] += cpitch[i];
            }
            int y0 = y == 0 ? 0 : 1;
            int y1 = y == height - 3 ? 0 : 1;
            for (int i = 0; i < ocount; ++i) {
                srcp0[i] += opitch[i] * y0;
                srcp2[i] += opitch[i] * y1;
            }
        }
    }
    return dst;
}

