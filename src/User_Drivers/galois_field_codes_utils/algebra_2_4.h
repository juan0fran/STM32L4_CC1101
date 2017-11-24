/* $Id: algebra_2_4.h 207 2014-12-10 19:47:50Z roca $ */
/*
 * OpenFEC.org AL-FEC Library.
 * (C) 1997-98 Luigi Rizzo (luigi@iet.unipi.it)
 *
 * Portions derived from code by Phil Karn (karn@ka9q.ampr.org),
 * Robert Morelos-Zaragoza (robert@spectra.eng.hawaii.edu) and Hari
 * Thirumoorthy (harit@spectra.eng.hawaii.edu), Aug 1995
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above
 *    copyright notice, this list of conditions and the following
 *    disclaimer in the documentation and/or other materials
 *    provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
 * PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE AUTHORS
 * BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY,
 * OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA,
 * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR
 * TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 * OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY
 * OF SUCH DAMAGE.
 */

#ifndef OF_GALOIS_FIELD_ALGEBRA_2_4_H
#define OF_GALOIS_FIELD_ALGEBRA_2_4_H

#include "../of_reed-solomon_gf_2_m.h"
#include "of_galois_field_code.h"

#ifdef OF_USE_REED_SOLOMON_2_M_CODEC


#define SWAP(a,b,t) {t tmp; tmp=a; a=b; b=tmp;}

extern const gf of_gf_2_4_log[16];
extern const gf of_gf_2_4_exp[16];
extern const gf of_gf_2_4_inv[16];
extern const gf of_gf_2_4_mul_table[16][16];

/*
 * addmul() computes dst[] = dst[] + c * src[]
 */
void    of_galois_field_2_4_addmul1(gf *dst1, gf *src1, gf c, int sz);

/*
 * of_galois_field_2_4_addmul1_compact() computes dst[] = dst[] + c * src[] when src and dst
 * both point to vectors where each GF(2^4) element is actually stored in 4-bits.
 * Said differently, there are two elements per byte.
 * This is used for matrix operations where source/repair symbols are involved (encoding
 * and decoding).
 * This function has been optimized so that two elements are accessed each time, and the
 * mulc table works on two elements at a time. This is a highly effective solution which
 * warrants high performances.
 */
void         of_galois_field_2_4_addmul1_compact (gf *dst1, gf *src1, gf c, int sz);

/*
 * computes C = AB where A is n*k, B is k*m, C is n*m
 */
void    of_galois_field_2_4_matmul (gf *a, gf *b, gf *c, int n, int k, int m);
int    of_galois_field_2_4_invert_mat (of_galois_field_code_cb_t* ofcb, gf *src, int k);
int    of_galois_field_2_4_invert_vdm (of_galois_field_code_cb_t* ofcb, gf *src, int k);

#endif //OF_USE_GALOIS_FIELD_CODES_UTILS

#endif //OF_GALOIS_FIELD_ALGEBRA_2_4_H
