// g2o - General Graph Optimization
// Copyright (C) 2011 R. Kuemmerle, G. Grisetti, W. Burgard
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are
// met:
//
// * Redistributions of source code must retain the above copyright notice,
//   this list of conditions and the following disclaimer.
// * Redistributions in binary form must reproduce the above copyright
//   notice, this list of conditions and the following disclaimer in the
//   documentation and/or other materials provided with the distribution.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS
// IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED
// TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
// PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
// HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
// SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED
// TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
// PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
// LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
// NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

#ifndef G2O_MACROS_H
#define G2O_MACROS_H

#ifndef DEG2RAD
#define DEG2RAD(x) ((x) * 0.01745329251994329575)
#endif

#ifndef RAD2DEG
#define RAD2DEG(x) ((x) * 57.29577951308232087721)
#endif

// GCC the one and only
#if defined(__GNUC__)
#  define G2O_ATTRIBUTE_CONSTRUCTOR(func) \
     static void func(void)__attribute__ ((constructor)); \
     static void func(void)

#  define G2O_ATTRIBUTE_UNUSED __attribute__((unused))
#  define G2O_ATTRIBUTE_FORMAT12 __attribute__ ((format (printf, 1, 2)))
#  define G2O_ATTRIBUTE_FORMAT23 __attribute__ ((format (printf, 2, 3)))
#  define G2O_ATTRIBUTE_WARNING(func) func __attribute__((warning))
#  define G2O_ATTRIBUTE_DEPRECATED(func) func __attribute__((deprecated))

#  define g2o_isnan(x)     std::isnan(x)
#  define g2o_isinf(x)     std::isinf(x)
#  define g2o_isfinite(x)  std::isfinite(x)

#else
#  ifndef __PRETTY_FUNCTION__
#    define __PRETTY_FUNCTION__ ""
#  endif
#  define G2O_ATTRIBUTE_CONSTRUCTOR(func) func
#  define G2O_ATTRIBUTE_UNUSED
#  define G2O_ATTRIBUTE_FORMAT12
#  define G2O_ATTRIBUTE_FORMAT23
#  define G2O_ATTRIBUTE_WARNING(func) func
#  define G2O_ATTRIBUTE_DEPRECATED(func) func

#include <math.h>
#define g2o_isnan(x)    isnan(x)
#define g2o_isinf(x)    isinf(x)
#define g2o_isfinite(x) isfinite(x)

#endif

// some macros that are only useful for c++
#ifdef __cplusplus

#define G2O_FSKIP_LINE(f) \
   {char c=' ';while(c != '\n' && f.good() && !(f).eof()) (f).get(c);}

#ifndef PVAR
  #define PVAR(s) \
    #s << " = " << (s) << std::flush
#endif

#ifndef PVARA
#define PVARA(s) \
  #s << " = " << RAD2DEG(s) << "deg" << std::flush
#endif

#ifndef FIXED
#define FIXED(s) \
  std::fixed << s << std::resetiosflags(std::ios_base::fixed)
#endif

#endif // __cplusplus

#endif
