/* $Id: of_galois_field_code.c 185 2014-07-15 09:57:16Z roca $ */
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

static gf decoding_packet_buffer[OF_MAX_ENCODING_SYMBOLS * OF_MAX_SYMBOL_SIZE];


gf of_modnn(of_galois_field_code_cb_t* ofcb, INT32 x)
{
    UINT16 field_size = ofcb->field_size;
    while (x >= field_size)
    {
        x -= field_size;
        x = (x >> ofcb->m) + (x & field_size);
    }
    return x;
}


/*
 * shuffle move src packets in their position
 */
static int
of_rs_2m_shuffle (gf *pkt[], int index[], int k)
{
     int i;

     for (i = 0 ; i < k ;)
     {
         if (index[i] >= k || index[i] == i)
             i++ ;
         else
         {
             /*
              * put pkt in the right position (first check for conflicts).
              */
             int c = index[i] ;

             if (index[c] == c)
             {
                 return 1 ;
             }
             SWAP (index[i], index[c], int) ;
             SWAP (pkt[i], pkt[c], gf *) ;
         }
     }
     return 0 ;
}


void        of_rs_2m_release(of_galois_field_code_cb_t* ofcb)
{

}

gf tmp_m[OF_MAX_ENCODING_SYMBOLS * OF_MAX_ENCODING_SYMBOLS];

of_status_t of_rs_2m_build_encoding_matrix(of_galois_field_code_cb_t* ofcb)
{
    gf *p;
    UINT32 k,r,col,row;
    k=ofcb->nb_source_symbols;
    r = ofcb->nb_repair_symbols;
    memset(tmp_m, 0, sizeof(tmp_m));
    memset(ofcb->matrix, 0, OF_MAX_ENCODING_SYMBOLS * OF_MAX_ENCODING_SYMBOLS);

    /*
     * fill the matrix with powers of field elements, starting from 0.
     * The first row is special, cannot be computed with exp. table.
     */
    tmp_m[0] = 1 ;
    for (col = 1; col < k ; col++)
        tmp_m[col] = 0 ;
    for (p = tmp_m + k, row = 0; row < k+r - 1 ; row++, p += k)
    {
        for (col = 0 ; col < k ; col ++)
            p[col] = of_gf_2_4_exp[of_modnn (ofcb,row*col) ];
    }

    /*
     * quick code to build systematic matrix: invert the top
     * k*k vandermonde matrix, multiply right the bottom n-k rows
     * by the inverse, and construct the identity matrix at the top.
     */
    of_galois_field_2_4_invert_vdm(ofcb, tmp_m, k);
    of_galois_field_2_4_matmul(tmp_m + k*k, tmp_m, ofcb->matrix + k*k, r, k, k);

    /*
     * the upper matrix is I so do not bother with a slow multiply
     */
    bzero (ofcb->matrix, k*k*sizeof (gf));
    for (p = ofcb->matrix, col = 0 ; col < k ; col++, p += k + 1)
        *p = 1 ;

    //of_free (tmp_m);
    return OF_STATUS_OK;
}


#ifdef OF_USE_DECODER
of_status_t of_rs_2m_build_decoding_matrix(of_galois_field_code_cb_t* ofcb, int *index)
{
    UINT32 k,r,i;
    gf *p;
    k = ofcb->nb_source_symbols;
    r = ofcb->nb_repair_symbols;
    //ofcb->dec_matrix = decoding_matrix_buffer;
    /*
    if ((ofcb->dec_matrix = of_malloc((k)*(k))) == NULL)
    {
        goto no_mem;
    }
    */
    memset(ofcb->dec_matrix, 0, OF_MAX_ENCODING_SYMBOLS * OF_MAX_ENCODING_SYMBOLS);
    for (i = 0, p = ofcb->dec_matrix ; i < k ; i++, p += k)
    {
#if 1 /* this is simply an optimization, not very useful indeed */
        if (index[i] < k)
        {
            bzero (p, k*sizeof (gf));
            p[i] = 1 ;
        }
        //else
#endif
            if (index[i] < (k+r))
                bcopy (& (ofcb->matrix[index[i]*k]), p, k*sizeof (gf));
            else
            {
                //of_free (ofcb->dec_matrix);
                return OF_STATUS_FATAL_ERROR ;
            }
    }
    int result;
    result = of_galois_field_2_4_invert_mat(ofcb, ofcb->dec_matrix, k);

    if (result)
    {
        //of_free (ofcb->dec_matrix);
        //ofcb->dec_matrix = NULL ;
        goto error;
    }
    return OF_STATUS_OK;

error:
    return OF_STATUS_FATAL_ERROR;
}


of_status_t of_rs_2m_decode (of_galois_field_code_cb_t* ofcb, gf *_pkt[], int index[], int sz)
{
    gf **pkt = (gf**) _pkt;                /* VR */
    //gf **new_pkt;
    /* New pkt is now converted into a static buffer DECODING PACKET BUFFER */
    int row, col, k = ofcb->nb_source_symbols ;
    memset(decoding_packet_buffer, 0, sizeof(decoding_packet_buffer));

    if (ofcb->m > 8)
        sz /= 2 ;
    if (of_rs_2m_shuffle(pkt, index, k))
    {
        /* error if true */
        return OF_STATUS_ERROR ;
    }
    if (of_rs_2m_build_decoding_matrix(ofcb, index) != OF_STATUS_OK)
    {
        goto error;
    }

    /*
     * do the actual decoding
     */
    /* Change new packet with another story compatible */
    for (row = 0 ; row < k ; row++)
    {
        if (index[row] >= k)
        {
            for (col = 0 ; col < k ; col++)
            {
                if (ofcb->dec_matrix[row*k + col] != 0)
                {
                    of_galois_field_2_4_addmul1_compact(&decoding_packet_buffer[row*sz], pkt[col], ofcb->dec_matrix[row*k + col], sz);
                }
            }
        }
    }

    /*
     * move pkts to their final destination
     * Warning: this function does not update the index[] table to contain
     * the actual reconstructed packet index.
     */
    for (row = 0 ; row < k ; row++)
    {
        if (index[row] >= k)
        {
            bcopy (&decoding_packet_buffer[row*sz], pkt[row], sz*sizeof(gf));
            //of_free (new_pkt[row]);
        }
    }
    //of_free (new_pkt);
    //of_free(ofcb->dec_matrix);
    //ofcb->dec_matrix = NULL;
    return OF_STATUS_OK;

error:
    return OF_STATUS_FATAL_ERROR;
}
#endif


#ifdef OF_USE_ENCODER
of_status_t    of_rs_2m_encode(of_galois_field_code_cb_t* ofcb,gf *_src[], gf *_fec, int index, int sz)
{
    gf **src = (gf**) _src;                /* VR */
    gf *fec = (gf*) _fec;                /* VR */
    int i, k = ofcb->nb_source_symbols ;
    gf *p ;

    if (ofcb->m > 8)
        sz /= 2 ;

    if (index < k)
    {
        bcopy (src[index], fec, sz * sizeof(gf));
    }
    else if (index < (ofcb->nb_source_symbols + ofcb->nb_repair_symbols))
    {
        p = &(ofcb->matrix[index*k]);
        bzero (fec, sz * sizeof (gf));
        for (i = 0; i < k ; i++)
        {
            if (p[i] != 0 )
            {
                of_galois_field_2_4_addmul1_compact(fec, src[i], p[i], sz);

            }
        }
        return OF_STATUS_OK;
    }
    return OF_STATUS_ERROR;
}
#endif

#endif //OF_USE_GALOIS_FIELD_CODES_UTILS
