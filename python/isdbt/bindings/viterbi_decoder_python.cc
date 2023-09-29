/*
 * Copyright 2022 Free Software Foundation, Inc.
 *
 * This file is part of GNU Radio
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 */

/***********************************************************************************/
/* This file is automatically generated using bindtool and can be manually edited  */
/* The following lines can be configured to regenerate this file during cmake      */
/* If manual edits are made, the following tags should be modified accordingly.    */
/* BINDTOOL_GEN_AUTOMATIC(0)                                                       */
/* BINDTOOL_USE_PYGCCXML(0)                                                        */
/* BINDTOOL_HEADER_FILE(viterbi_decoder.h)                                        */
/* BINDTOOL_HEADER_FILE_HASH(333d5fdcf2925aff2e78c60a1f71b672)                     */
/***********************************************************************************/

#include <pybind11/complex.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

namespace py = pybind11;

#include <gnuradio/isdbt/viterbi_decoder.h>
// pydoc.h is automatically generated in the build directory
#include <viterbi_decoder_pydoc.h>

void bind_viterbi_decoder(py::module& m)
{

    using viterbi_decoder    = ::gr::isdbt::viterbi_decoder;


    py::class_<viterbi_decoder, gr::block, gr::basic_block,
        std::shared_ptr<viterbi_decoder>>(m, "viterbi_decoder", D(viterbi_decoder))

        .def(py::init(&viterbi_decoder::make),
           py::arg("constellation_size"),
           py::arg("rate"),
           D(viterbi_decoder,make)
        )
        



        ;




}








