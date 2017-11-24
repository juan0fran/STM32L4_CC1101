/* $Id: of_galois_field_code.h 148 2014-07-08 08:01:56Z roca $ */
/*
 * OpenFEC.org AL-FEC Library.
 * (c) Copyright 2009 - 2012 INRIA - All rights reserved
 * Contact: vincent.roca@inria.fr
 *
 * This software is governed by the CeCILL-C license under French law and
 * abiding by the rules of distribution of free software.  You can  use,
 * modify and/ or redistribute the software under the terms of the CeCILL-C
 * license as circulated by CEA, CNRS and INRIA at the following URL
 * "http://www.cecill.info".
 *
 * As a counterpart to the access to the source code and  rights to copy,
 * modify and redistribute granted by the license, users are provided only
 * with a limited warranty  and the software's author,  the holder of the
 * economic rights,  and the successive licensors  have only  limited
 * liability.
 *
 * In this respect, the user's attention is drawn to the risks associated
 * with loading,  using,  modifying and/or developing or reproducing the
 * software by the user in light of its specific status of free software,
 * that may mean  that it is complicated to manipulate,  and  that  also
 * therefore means  that it is reserved for developers  and  experienced
 * professionals having in-depth computer knowledge. Users are therefore
 * encouraged to load and test the software's suitability as regards their
 * requirements in conditions enabling the security of their systems and/or
 * data to be ensured and,  more generally, to use and operate it in the
 * same conditions as regards security.
 *
 * The fact that you are presently reading this means that you have had
 * knowledge of the CeCILL-C license and that you accept its terms.
 */

#ifndef GALOIS_FIELD_CODE_H
#define GALOIS_FIELD_CODE_H

#include "../of_reed-solomon_gf_2_m.h"

#define bcmp(s1 ,s2, n) memcmp((s1), (s2), (size_t)(n))

#define bzero(s1, n) memset(s1, 0, n)

#define bcopy(s1, s2, n) memcpy (s2, s1, n)

#ifdef OF_USE_REED_SOLOMON_2_M_CODEC

#define FEC_MAGIC    0xFECC0DEC


/*
 * Primitive polynomials - see Lin & Costello, Appendix A,
 * and  Lee & Messerschmitt, p. 453.
 */

/**
 * Galois-Field-Code stable codec specific control block structure.
 */
typedef    of_rs_2_m_cb_t    of_galois_field_code_cb_t;    /* XXX: the two types are synonymous in fact! */

/**
 * just a helper to init all we need to use GF
 */
of_status_t    of_rs_2m_init(of_galois_field_code_cb_t* ofcb);

/**
 * and the helper to release memory
 */
void        of_rs_2m_release(of_galois_field_code_cb_t* ofcb);

/**
 * even if only decoder is defined, we need an encoding matrix.
 */
of_status_t    of_rs_2m_build_encoding_matrix(of_galois_field_code_cb_t* ofcb);

#ifdef OF_USE_DECODER
of_status_t    of_rs_2m_build_decoding_matrix(of_galois_field_code_cb_t* ofcb,int* index);
of_status_t    of_rs_2m_decode(of_galois_field_code_cb_t* ofcb,gf *pkt[], int index[], int sz);
#endif

#ifdef OF_USE_ENCODER
of_status_t    of_rs_2m_encode(of_galois_field_code_cb_t* ofcb,gf *_src[], gf *_fec, int index, int sz);
#endif

#endif //OF_USE_GALOIS_FIELD_CODES_UTILS

#endif //GALOIS_FIELD_CODE_H
