/***************************************************************************************************
*  File:        of_reed-solomon_gf_2_m.h                                                           *
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

/* $Id: of_reed-solomon_uint8_t_2_m.h 182 2014-07-15 09:27:51Z roca $ */
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

#define OF_USE_REED_SOLOMON_2_M_CODEC

#ifdef OF_USE_REED_SOLOMON_2_M_CODEC

#ifndef OF_REED_SOLOMON_uint8_t_2_M_H
#define OF_REED_SOLOMON_uint8_t_2_M_H

#include <stdint.h>
#include <stdbool.h>
#include <string.h>

#define OF_USE_DECODER
#define OF_USE_ENCODER

#define OF_REED_SOLOMON_2_M_MAX_M     4
#define OF_MAX_ENCODING_SYMBOLS     15
#define OF_MAX_FIELD_SIZE             15
#define OF_MAX_SYMBOL_SIZE             255

#define gf         uint8_t
#define UINT32     uint32_t
#define UINT8     uint8_t
#define UINT16     uint16_t
#define INT32   int32_t

typedef enum
{
    OF_CODEC_NIL                = 0,
    OF_CODEC_REED_SOLOMON_GF_2_8_STABLE    = 1,
    OF_CODEC_REED_SOLOMON_GF_2_M_STABLE    = 2,
    OF_CODEC_LDPC_STAIRCASE_STABLE        = 3,
//    OF_CODEC_LDPC_TRIANGLE_STABLE        = 4,
    OF_CODEC_2D_PARITY_MATRIX_STABLE    = 5,
    OF_CODEC_LDPC_FROM_FILE_ADVANCED    = 6
} of_codec_id_t;


/**
 * Specifies if this codec instance is a pure encoder, a pure decoder, or a codec capable of
 * both encoding and decoding.
 */
typedef  uint8_t    of_codec_type_t;

#define    OF_ENCODER         0x1
#define    OF_DECODER         0x2
#define    OF_ENCODER_AND_DECODER     (OF_ENCODER | OF_DECODER)


/**
 * Function return value, indicating wether the function call succeeded (OF_STATUS_OK)
 * or not. In case of failure, the detailed error type is returned in a global variable,
 * of_errno (see of_errno.h).
 *
 *    OF_STATUS_OK = 0        Success
 *    OF_STATUS_FAILURE,        Failure. The function called did not succeed to perform
 *                    its task, however this is not an error. This can happen
 *                    for instance when decoding did not succeed (which is a
 *                    valid output).
 *    OF_STATUS_ERROR,        Generic error type. The caller is expected to be able
 *                    to call the library in the future after having corrected
 *                    the error cause.
 *    OF_STATUS_FATAL_ERROR        Fatal error. The caller is expected to stop using this
 *                    codec instance immediately (it replaces an exit() system
 *                    call).
 */
typedef enum
{
    OF_STATUS_OK = 0,
    OF_STATUS_FAILURE,
    OF_STATUS_ERROR,
    OF_STATUS_FATAL_ERROR
} of_status_t;


/**
 * Throughout the API, a pointer to this structure is used as an identifier of the current
 * codec instance, also known as "session".
 *
 * This generic structure is meant to be extended by each codec and new pieces of information
 * that are specific to each codec be specified there. However, all the codec specific structures
 * MUST begin the same entries as the ones provided in this generic structure, otherwise
 * hazardeous behaviors may happen.
 */
typedef struct of_session
{
    of_codec_id_t    codec_id;
    of_codec_type_t    codec_type;
} of_session_t;


/**
 * Generic FEC parameter structure used by of_set_fec_parameters().
 *
 * This generic structure is meant to be extended by each codec and new pieces of information
 * that are specific to each codec be specified there. However, all the codec specific structures
 * MUST begin the same entries as the ones provided in this generic structure, otherwise
 * hazardeous behaviors may happen.
 */
typedef struct of_parameters
{
    uint32_t        nb_source_symbols;
    uint32_t        nb_repair_symbols;
    uint32_t        encoding_symbol_length;
} of_parameters_t;



typedef struct of_rs_2_m_parameters
{
    uint32_t        nb_source_symbols;    /* must be 1st item */
    uint32_t        nb_repair_symbols;    /* must be 2nd item */
    uint32_t        encoding_symbol_length; /* must be 3rd item */
} of_rs_2_m_parameters_t;

/**
 * Reed-Solomon stable codec specific control block structure.
 */
typedef struct of_rs_2_m_cb
{
/************************************************************************************
 *                 of_rs_2_m_cb                        *
 ***********************************************************************************/
/************************************************************************************
 *                              of_cb_t                                             *
 ***********************************************************************************/
    of_codec_id_t        codec_id;        /* must begin with fec_codec_id          */
    of_codec_type_t        codec_type;        /* must be 2nd item                      */
    uint32_t            nb_source_symbols;    /** k parameter (AKA code dimension).    */
    uint32_t            nb_repair_symbols;    /** r = n - k parameter.                 */
    uint32_t            encoding_symbol_length;    /** symbol length.                       */
/***********************************************************************************/

    uint16_t            m;        /* in theory between 2..16. Currently only values 4 and 8 are supported */
    uint16_t            field_size;     /* 2^m */
    uint8_t*            of_rs_uint8_t_exp;    /* index->poly form conversion table. */
    int*                of_rs_uint8_t_log;    /* Poly->index form conversion table. */
    uint8_t*            of_rs_inverse;    /* inverse of field elem. */
    uint8_t**            of_uint8_t_mul_table; /* table of 2 numbers multiplication. */

    /**
     * Encoding/Decoding matrix (G).
     * The encoding matrix is computed starting with a Vandermonde matrix,
     * and then transforming it into a systematic matrix.
     */
    uint8_t                matrix[OF_MAX_ENCODING_SYMBOLS * OF_MAX_ENCODING_SYMBOLS];
    uint8_t                dec_matrix[OF_MAX_ENCODING_SYMBOLS * OF_MAX_ENCODING_SYMBOLS];
    /***********************************************************************************/

    uint32_t            magic;

    /** Maximum number of source symbols supported by this codec for practical reasons. */
    uint32_t            max_nb_source_symbols;
    /** Maximum number of encoding symbols supported by this codec for practical reasons. */
    uint32_t            max_nb_encoding_symbols;
    /* maximum m value in uint8_t(2^m) supported by the codec */
    uint16_t            max_m;
    uint32_t             nb_encoding_symbols;

    /*
     * decoder specific variables.
     */
    /**
     * Table of available source and repair symbols. This table is ordered, no matter
     * the symbol arrival order.
     */
    void                 *available_symbols_tab[OF_MAX_ENCODING_SYMBOLS];
    /** Number of available source and repair symbols. This is the number of entries in
     * the available_symbols_tab tables. */
    uint32_t             nb_available_symbols;
    /** Number of available source symbols. */
    uint32_t            nb_available_source_symbols;
    bool                 decoding_finished;    /** true as soon as decoding completed. */

} of_rs_2_m_cb_t;


/*
 * API function prototypes.
 */
 /**
 * This function create the codec instance for the Reed-Solomon codec.
 *
 * @fn of_status_t    of_rs_2_m_create_codec_instance (of_rs_2_m_cb_t** of_cb)
 * @brief        create a Reed-Solomon codec instance
 * @param of_cb        (IN/OUT) address of the pointer to a Reed-Solomon codec control block. This pointer is updated
 *            by this function.
 *            In case of success, it points to a session structure allocated by the
 *            library. In case of failure it points to NULL.
 * @return        Error status. The ofcb pointer is updated according to the success return
 *            status.
 */
of_status_t    of_rs_2_m_create_codec_instance    (of_rs_2_m_cb_t*        of_cb);

/**
 * This function releases all the internal resources used by this FEC codec instance.
 * None of the source symbol buffers will be free'ed by this function, even those decoded by
 * the library if any, regardless of whether a callback has been registered or not. It's the
 * responsibility of the caller to free them.
 *
 * @fn of_status_t    of_rs_2_m_release_codec_instance (of_rs_2_m_cb_t* ofcb)
 * @brief release all resources used by the codec
 * @param ofcb        (IN) Pointer to the control block.
 * @return        Error status.
 */
of_status_t    of_rs_2_m_release_codec_instance    (of_rs_2_m_cb_t*        ofcb);

/**
 *
 * @fn of_status_t    of_rs_2_m_set_fec_parameters  (of_rs_2_m_cb_t* ofcb, of_rs_parameters_t* params)
 * @brief        set all the FEC codec parameters (e.g. k, n, or symbol size)
 * @param ofcb        (IN) Pointer to the control block.
 * @param params    (IN) pointer to a structure containing the FEC parameters associated to
 *            a specific FEC codec.
 * @return        Error status.
 */
of_status_t    of_rs_2_m_set_fec_parameters       (of_rs_2_m_cb_t*        ofcb,
                        of_rs_2_m_parameters_t*    params);

of_status_t of_rs_2_m_reinitialize_codec       (of_rs_2_m_cb_t * ofcb);

#ifdef OF_USE_ENCODER
/**
 * @fn        of_status_t    of_rs_2_m_build_repair_symbol (of_rs_2_m_cb_t* ofcb, void* encoding_symbols_tab[], uint32_t    esi_of_symbol_to_build)
 * @brief            build a repair symbol (encoder only)
 * @param ofcb            (IN) Pointer to the session.
 * @param encoding_symbols_tab    (IN/OUT) table of source and repair symbols.
 *                The entry for the repair symbol to build can either point
 *                to a buffer allocated by the application, or let to NULL
 *                meaning that of_build_repair_symbol will allocate memory.
 * @param esi_of_symbol_to_build
 *                (IN) encoding symbol ID of the repair symbol to build in
 *                {k..n-1}
 * @return            Error status.
 */
of_status_t    of_rs_2_m_build_repair_symbol      (of_rs_2_m_cb_t*    ofcb,
                        void*        encoding_symbols_tab[],
                        uint32_t        esi_of_symbol_to_build);
#endif //OF_USE_ENCODER

#ifdef OF_USE_DECODER
/**
 * @fn      of_status_t    of_rs_2_m_decode_with_new_symbol (of_rs_2_m_cb_t*    ofcb, void*    const    new_symbol_buf, uint32_t        new_symbol_esi)
 * @brief (try to) decode with a newly received symbol
 * @param ofcb            (IN) Pointer to the session.
 * @param new_symbol        (IN) Pointer to the encoding symbol now available (i.e. a new
 *                symbol received by the application, or a decoded symbol in case
 *                of a recursive call).
 * @param new_symbol_esi    (IN) Encoding symbol ID of the newly symbol available, in {0..n-1}.
 * @return            Error status (NB: this function does not return OF_STATUS_FAILURE).
 */
of_status_t    of_rs_2_m_decode_with_new_symbol   (of_rs_2_m_cb_t*    ofcb,
                        void*        new_symbol,
                        uint32_t        new_symbol_esi);

/**
 * @fn                of_status_t    of_rs_2_m_set_available_symbols (of_rs_2_m_cb_t*    ofcb, void* const    encoding_symbols_tab[]);
 * @brief            inform the decoder of all the available (received) symbols
 * @param ofcb            (IN) Pointer to the session.
 * @param encoding_symbols_tab    (IN) Pointer to the available encoding symbols table. To each
 *                available symbol the corresponding entry in the table must point
 *                to the associated buffer. Entries set to NULL are interpreted as
 *                corresponding to erased symbols.
 * @return            Error status.
 */
of_status_t    of_rs_2_m_set_available_symbols    (of_rs_2_m_cb_t*    ofcb,
                        void* const    encoding_symbols_tab[]);

/**
 * @fn            of_status_t    of_rs_2_m_finish_decoding (of_rs_2_m_cb_t*    ofcb)
 * @brief        finish decoding with available symbols
 * @param ofcb        (IN) Pointer to the session.
 * @return        Error status. Returns OF_STATUS_FAILURE if decoding failed, or
 *            OF_STATUS_OK if decoding succeeded, or OF_STATUS_*_ERROR in case
 *            of (fatal) error.
 */
of_status_t    of_rs_2_m_finish_decoding        (of_rs_2_m_cb_t*    ofcb);

/**
 * @fn            bool        of_rs_2_m_is_decoding_complete (of_rs_2_m_cb_t*    ofcb)
 * @brief         check if decoding is finished
 * @param ofcb        (IN) Pointer to the session.
 * @return        Boolean. Warning, this is one of the very functions of the library that
 *            does not return an error status.
 */
bool        of_rs_2_m_is_decoding_complete    (of_rs_2_m_cb_t*    ofcb);

/**
 * @fn            of_status_t    of_rs_2_m_get_source_symbols_tab (of_rs_2_m_cb_t* ofcb, void* source_symbols_tab[])
 * @brief        get the table of available source symbols (after decoding)
 * @param ofcb        (IN) Pointer to the session.
 * @param source_symbols_tab    (IN/OUT) table, that will be filled by the library and returned
 *            to the application.
 * @return        Error status.
 */
of_status_t    of_rs_2_m_get_source_symbols_tab    (of_rs_2_m_cb_t*    ofcb,
                         void*        source_symbols_tab[]);

#endif  //OF_USE_DECODER

/**
 * @fn            of_status_t    of_rs_2_m_set_control_parameter (of_rs_2_m_cb_t* ofcb,uint32_t    type,void* value,uint32_t    length)
 * @brief        set a specific FEC parameter
 * @param ofcb        (IN) Pointer to the session.
 * @param type        (IN) Type of parameter. This type is FEC codec ID specific.
 * @param value        (IN) Pointer to the value of the parameter. The type of the object pointed
 *            is FEC codec ID specific.
 * @param length    (IN) length of pointer
 * @return        Error status.
 */
of_status_t    of_rs_2_m_set_control_parameter    (of_rs_2_m_cb_t*    ofcb,
                        uint32_t        type,
                        void*        value,
                        uint32_t        length);

/**
 * @fn            of_status_t    of_rs_2_m_get_control_parameter (of_rs_2_m_cb_t* ofcb,uint32_t    type,void* value,uint32_t    length)
 * @brief        get a specific FEC parameter
 * @param ofcb        (IN) Pointer to the session.
 * @param type        (IN) Type of parameter. This type is FEC codec ID specific.
 * @param value        (IN/OUT) Pointer to the value of the parameter. The type of the object
 *            pointed is FEC codec ID specific. This function updates the value object
 *            accordingly. The application, who knows the FEC codec ID, is responsible
 *            to allocating the approriate object pointed by the value pointer.
 * @param length    (IN) length of pointer
 * @return        Error status.
 */
of_status_t    of_rs_2_m_get_control_parameter    (of_rs_2_m_cb_t*    ofcb,
                        uint32_t        type,
                        void*        value,
                        uint32_t        length);



#endif //OF_REED_SOLOMON_2_M_H

#endif /* #ifdef OF_USE_REED_SOLOMON_CODEC_2_M */
