/***************************************************************************************************
*  File:        of_reed-solomon_gf_2_m_api.c                                                       *
*  Authors:     Joan Francesc Muñoz Martin <JFM>                                                   *
*                                                                                                  *
*  Creation:    27-11-2017                                                                         *
*  Description: Reed-Solomon Erasure Code Corrector implementation and definition                  *
*  This code is copied and adapted from OpenFEC project.                                           *
*                                                                                                  *
*  This file is part of a project developed by Nano-Satellite and Payload Laboratory (NanoSat Lab) *
*  at Technical University of Catalonia - UPC BarcelonaTech.                                       *
*                                                                                                  *
* ------------------------------------------------------------------------------------------------ *
*  Changelog:                                                                                      *
*  v#   Date            Author  Description                                                        *
*  0.1  27-11-2017      <JFM>   <First version>                                                    *
***************************************************************************************************/

/* $Id: of_reed-solomon_gf_2_m_api.c 186 2014-07-16 07:17:53Z roca $ */
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

#include "of_reed-solomon_gf_2_m.h"
#include "galois_field_codes_utils/algebra_2_4.h"

#ifdef OF_USE_REED_SOLOMON_2_M_CODEC

of_status_t of_rs_2_m_create_codec_instance (of_rs_2_m_cb_t*    of_cb)
{
    of_codec_type_t            codec_type;    /* temporary value */

    codec_type                = of_cb->codec_type;
    of_cb->codec_type         = codec_type;
    of_cb->codec_id            = OF_CODEC_REED_SOLOMON_GF_2_M_STABLE;

    of_cb->max_m            = OF_REED_SOLOMON_2_M_MAX_M;    /* init it immediately... */
    of_cb->m                 = of_cb->max_m;
    return OF_STATUS_OK;
}

of_status_t    of_rs_2_m_release_codec_instance (of_rs_2_m_cb_t*    ofcb)
{
    return OF_STATUS_OK;
}

of_status_t    of_rs_2_m_set_fec_parameters   (of_rs_2_m_cb_t*        ofcb,
                        of_rs_2_m_parameters_t*    params)
{

    ofcb->field_size        = (1 << ofcb->m) - 1;
    ofcb->max_nb_encoding_symbols    = ofcb->max_nb_source_symbols = ofcb->field_size;
    if(ofcb->max_nb_encoding_symbols > OF_MAX_ENCODING_SYMBOLS){
        goto error;
    }
    if((ofcb->nb_source_symbols    = params->nb_source_symbols) > ofcb->max_nb_source_symbols) {
        goto error;
    }
    ofcb->nb_source_symbols        = params->nb_source_symbols;
    ofcb->nb_repair_symbols        = params->nb_repair_symbols;
    ofcb->encoding_symbol_length    = params->encoding_symbol_length;
    ofcb->nb_encoding_symbols    = ofcb->nb_source_symbols + ofcb->nb_repair_symbols;

    memset(ofcb->available_symbols_tab, 0, sizeof(void *) * ofcb->nb_encoding_symbols);
    memset(ofcb->matrix, 0, sizeof(ofcb->matrix));
    ofcb->nb_available_symbols    = 0;
    ofcb->nb_available_source_symbols = 0;
    ofcb->decoding_finished = false;

    if(of_rs_2m_build_encoding_matrix((of_galois_field_code_cb_t*)ofcb) != OF_STATUS_OK) {
        goto error;
    }

    return OF_STATUS_OK;

error:

    return OF_STATUS_FATAL_ERROR;
}

#ifdef OF_USE_ENCODER

of_status_t    of_rs_2_m_build_repair_symbol  (of_rs_2_m_cb_t*    ofcb,
                        void*        encoding_symbols_tab[],
                        UINT32        esi_of_symbol_to_build)
{

    if(esi_of_symbol_to_build < ofcb->nb_source_symbols || esi_of_symbol_to_build >= ofcb->nb_encoding_symbols) {
        goto error;
    }

    if(of_rs_2m_encode((of_galois_field_code_cb_t*) ofcb, (gf**)encoding_symbols_tab,
                encoding_symbols_tab[esi_of_symbol_to_build], esi_of_symbol_to_build,
                ofcb->encoding_symbol_length) != OF_STATUS_OK) {
          goto error;
    }

    return OF_STATUS_OK;

error:

    return OF_STATUS_ERROR;
}
#endif //OF_USE_ENCODER


#ifdef OF_USE_DECODER
of_status_t of_rs_2_m_decode_with_new_symbol (of_rs_2_m_cb_t*    ofcb,
                      void*        new_symbol,
                      UINT32    new_symbol_esi)
{

    if(ofcb->decoding_finished) {
        return OF_STATUS_OK;
    }
    if(ofcb->available_symbols_tab[new_symbol_esi] != NULL) {
        goto end;
    }
    ofcb->available_symbols_tab[new_symbol_esi] = new_symbol;
    ofcb->nb_available_symbols++;
    if(new_symbol_esi < ofcb->nb_source_symbols) {
        /* remember a new source symbol is available */
        ofcb->nb_available_source_symbols++;
    }

    if(ofcb->nb_available_source_symbols == ofcb->nb_source_symbols) {
        ofcb->decoding_finished = true;

        return OF_STATUS_OK;
    }
    if(ofcb->nb_available_symbols >= ofcb->nb_source_symbols) {
        /* we received a sufficient number of symbols, so let's decode */
        if(of_rs_2_m_finish_decoding(ofcb) != OF_STATUS_OK)
        {
            goto error;
        }

        return OF_STATUS_OK;
    }
end:

    return OF_STATUS_OK;
error:

    return OF_STATUS_ERROR;
}


of_status_t    of_rs_2_m_set_available_symbols    (of_rs_2_m_cb_t*    ofcb,
                        void* const    encoding_symbols_tab[])
{
    UINT32    i;


    ofcb->nb_available_symbols = 0;
    ofcb->nb_available_source_symbols = 0;
    for(i = 0; i < ofcb->nb_encoding_symbols; i++) {
        if((ofcb->available_symbols_tab[i] = encoding_symbols_tab[i]) == NULL) {
            continue;
        }
        if(i < ofcb->nb_source_symbols) {
            ofcb->nb_available_source_symbols++;
        }
        ofcb->nb_available_symbols++;
    }

    return OF_STATUS_OK;
}

of_status_t    of_rs_2_m_finish_decoding (of_rs_2_m_cb_t*    ofcb)
{
    UINT32         k;
    char        *tmp_buf[OF_MAX_ENCODING_SYMBOLS];/* keep available source/repair symbol buffers here... */
    int            tmp_esi[OF_MAX_ENCODING_SYMBOLS]; /* ...and their esi here. In fact we only need k entries
                         * in these tables, but in order to avoid using malloc (time
                         * consumming), we use an automatic table of maximum size for
                         * both tmp_buf[] and tmp_esi[]. */
    INT32        tmp_idx;        /* index in tmp_buf[] and tmp_esi[] tabs */
    void        **ass_buf;        /* tmp pointer to the current available source symbol entry in available_symbols_tab[] */
    UINT32        ass_esi;        /* corresponding available source symbol ESI */
    void        **ars_buf;        /* tmp pointer to the current available repair symbol entry in available_symbols_tab[] */
    UINT32        ars_esi;        /* corresponding available repair symbol ESI */


    if(ofcb->decoding_finished) {
        return OF_STATUS_OK;
    }
    k = ofcb->nb_source_symbols;
    if(ofcb->nb_available_symbols < k) {
        return OF_STATUS_FAILURE;
    }
    if(ofcb->nb_available_source_symbols == k) {
        /* we received all the k source symbols, so it's finished */
        ofcb->decoding_finished = true;

        return OF_STATUS_OK;
    }
    /*
     * Because of of_rs_decode internal details, we put source symbols at their right location
     * and fill in the gaps (i.e. erased source symbols) with repair symbols.
     */
    ass_esi = 0;
    ars_esi = k;
    ass_buf = ofcb->available_symbols_tab;
    ars_buf = ofcb->available_symbols_tab + k;
    for(tmp_idx = 0; tmp_idx < k; tmp_idx++) {
        if(*ass_buf == NULL) {
            /* this source symbol is not available, replace it with a repair */
            while (*ars_buf == NULL) {
                ars_esi++;
                ars_buf++;
            }
            tmp_buf[tmp_idx] = *ars_buf;
            tmp_esi[tmp_idx] = ars_esi;
            ars_esi++;
            ars_buf++;
        } else {
            tmp_buf[tmp_idx] = *ass_buf;
            tmp_esi[tmp_idx] = ass_esi;
        }
        ass_esi++;
        ass_buf++;
    }

    if(of_rs_2m_decode((of_galois_field_code_cb_t*)ofcb,
        (gf**)tmp_buf, (int*)tmp_esi, ofcb->encoding_symbol_length) != OF_STATUS_OK) {
        goto error;
    }
    ofcb->decoding_finished = true;

    for(tmp_idx = 0; tmp_idx < k; tmp_idx++) {
        if(! (tmp_idx < k)) {
            goto error;
        }
        /* Point available symbols_tab to the temporary buffer */
        ofcb->available_symbols_tab[tmp_idx] = tmp_buf[tmp_idx];

    }


    return OF_STATUS_OK;

error:

    return OF_STATUS_ERROR;
}


bool    of_rs_2_m_is_decoding_complete (of_rs_2_m_cb_t* ofcb)
{
    return ofcb->decoding_finished;
}


of_status_t    of_rs_2_m_get_source_symbols_tab (of_rs_2_m_cb_t*    ofcb,
                          void*            source_symbols_tab[])
{

    if(of_rs_2_m_is_decoding_complete(ofcb) == false) {
        return OF_STATUS_ERROR;
    }

    UINT32    i;
    for(i = 0; i < ofcb->nb_source_symbols; i++) {
        source_symbols_tab[i] = ofcb->available_symbols_tab[i];
    }

    return OF_STATUS_OK;
}


#endif  //OF_USE_DECODER

#endif
