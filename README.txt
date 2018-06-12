

                         TMM2 v0.1  -  HELP FILE


REQUIREMENTS:

      Avisyth2.6.0 / Avisynth+r2005 or later
      Windows Vista sp2 or later
      Microsoft Visual C++ 2015 Redistributable Packages




GENERAL INFO:


      TMM2 supports YV12, YV16, YV24 and Y8 colorspaces

      TMM2 is a rewrite of TMM for Avisynth2.6/Avisynth+ which written by
   Kevin Stone (aka tritical).

      TMM2 builds a motion-mask for TDeint, which TDeint uses via its 'emask'
   parameter.  TMM2 can use fixed or per-pixel adaptive motion thresholds, as
   well as any length static period greater than or equal to six fields.  It
   checks backwards, across, and forwards when looking for motion.

   Syntax=>

    TMM2(clip, int mode, int order, int field, int length, int mtype,
           int ttype, int mtqL, int mthL, int mtqC, int mthC, int nt,
           int minthresh, int maxthresh, int cstr, int opt)


   Examples =>

        mpeg2source()  // assume tff source that we want to bob
        tdeint(order=1,mode=1,emask=TMM(order=1,mode=1))


        mpeg2source()  // assume bff source, keep top field, same rate
        tdeint(order=0,field=1,emask=TMM(order=0,field=1))



PARAMETERS:


     mode -

         Same as the 'mode' parameter of TDeint.  This should be set equal to
         the value of 'mode' given to TDeint.  If using mode=2 in TDeint use
         mode=1 in TMM.

            0 -  same rate output
            1 -  double rate output

         Default:  0  (int)


     order -

         Same as the 'order' parameter of TDeint.  This should be set equal to
         the value of 'order' given to TDeint.

           -1 -  use order from Avisynth
            0 -  bff
            1 -  tff

         Default:  -1  (int)


     field -

         Same as the 'field' parameter of TDeint.  This should be set equal to
         the value of 'field' given to TDeint.  If using mode=1 (double rate
         output) then this setting does nothing.

           -1 -  set field equal to order
            0 -  keep bottom field
            1 -  keep top field

         Default:  -1  (int)


     length -

         Sets the number of fields required for declaring pixels as stationary.
         length=6 means six fields (3 top/3 bottom), length=8 means 8 fields
         (4 top/4 bottom), etc... This can be any value greater than or equal
         to 6 (can be even or odd).  A larger value for length will prevent
         more motion-adaptive related artifacts, but will result in fewer pixels
         being weaved.

         Default:  10  (int)


     mtype -

         Sets whether or not both vertical neighboring lines in the current
         field of the line in the opposite parity field attempting to be
         weaved have to agree on both stationarity and direction.  Possible
         values:

            0 - no
            1 - no for across, but yes for backwards/forwards
            2 - yes

         0 will result in the most pixels being weaved, while 2 will have the
         least artifacts.

         Default:  1  (int)


     ttype -

         Sets how to determine the per-pixel adaptive (quarter pel/half pel)
         motion thresholds. Possible values:

            0 - 4 neighbors, diff to center pixel, compensated
            1 - 8 neighbors, diff to center pixel, compensated
            2 - 4 neighbors, diff to center pixel, uncompensated
            3 - 8 neighbors, diff to center pixel, uncompensated
            4 - 4 neighbors, range (max-min) of neighborhood
            5 - 8 neighbors, range (max-min) of neighborhood

         compensated means adjusted for distance differences due to field
         vs frames and chroma downsampling. The compensated versions will
         always result in thresholds <= to the uncompensated versions.

         Default:  1  (int)


     mtqL/mthL/mtqC/mthC -

         These parameters allow the specification of hard thresholds instead
         of using per-pixel adaptive motion thresholds.  mtqL sets the quarter
         pel threshold for luma, mthL sets the half pel threshold for luma,
         mtqC/mthC are the same but for chroma. If these parameters are set to
         -1 then an adaptive threshold is used. Otherwise, if they are between
         0 and 255 (inclusive) then the value of the parameter is used as the
         threshold for every pixel.

         Default:  mtqL -  -1 (int)
                   mthL -  -1 (int)
                   mtqC -  -1 (int)
                   mthC -  -1 (int)


     nt/minthresh/maxthresh -

         nt sets the noise threshold, which will be added to the value of each
         per-pixel threshold when determining if a pixel is stationary or not.
         After the addition of 'nt', any threshold less than minthresh will be
         increased to minthresh and any threshold greater than maxthresh will
         be decreased to maxthresh.

         Default:  nt        -   2 (int)
                   minthresh -   4 (int)
                   maxthresh -  75 (int)


     cstr -

         Sets the number of required neighbor pixels (3x3 neighborhood) in the
         quarter pel mask, of a pixel marked as moving in the quarter pel mask,
         but stationary in the half pel mask, marked as stationary for the pixel
         to be marked as stationary in the combined mask.

         Default:  4  (int)

     opt -

         Controls which cpu optimizations are used for create motion masks.

         0 - Use C++ routine.
         1 - Use SSE2 routine if possible. When SSE2 can't be used, fallback to 0.
         others - Use AVX2 routine if possible. When AVX2 can't be used, fallback to 1.

         Default: -1  (int)


NOTE:
    - TMM2_avx2.dll is compiled with /arch:AVX2.

    - On Avisynth2.6, AVX2 is always disabled even if you use TMM2_avx2.dll.

    - On Avisynth+ MT, TMM2 is set as MT_NICE_FILTER automatically.
      You don't have to set SetFilterMTMode() yourself for this filter.

    - This filter requires appropriate memory alignments.
      Thus, if you want to crop the left side of your source clip before this filter,
      you have to set crop(align=true).


CHANGE LIST:

   v0.1.1 - (2016-07-05)
      + Update avisynth.h to Avisynth+MT r2005
      + CreateMM: allocate temporal buffer at constructor unless the enviroment is not avs+MT.


   v0.1 - (2016-05-25)
      + Use buffer pool on avs+ MT.
      + Disable AVX2 always on Avisynth2.6.


   v0.0 - (2016-05-20)
      + initial release



TO DO LIST:


    - optimize




