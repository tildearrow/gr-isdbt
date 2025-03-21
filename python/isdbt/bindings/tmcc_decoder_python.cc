/*
 * Copyright 2025 Free Software Foundation, Inc.
 *
 * This file is part of GNU Radio
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 */

/***********************************************************************************/
/* This file is automatically generated using bindtool and can be manually
 * edited  */
/* The following lines can be configured to regenerate this file during cmake */
/* If manual edits are made, the following tags should be modified accordingly.
 */
/* BINDTOOL_GEN_AUTOMATIC(0) */
/* BINDTOOL_USE_PYGCCXML(0) */
/* BINDTOOL_HEADER_FILE(tmcc_decoder.h) */
/* BINDTOOL_HEADER_FILE_HASH(0d332aa957272d6d06081dacfc4fb6ac) */
/***********************************************************************************/

#include <pybind11/complex.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

namespace py = pybind11;

#include <gnuradio/isdbt/tmcc_decoder.h>
// pydoc.h is automatically generated in the build directory
#include <tmcc_decoder_pydoc.h>

void bind_tmcc_decoder(py::module &m) {

  using tmcc_decoder = ::gr::isdbt::tmcc_decoder;

  py::class_<tmcc_decoder, gr::block, gr::basic_block,
             std::shared_ptr<tmcc_decoder>>(m, "tmcc_decoder", D(tmcc_decoder))

      .def(py::init(&tmcc_decoder::make), py::arg("mode"),
           py::arg("print_params"), D(tmcc_decoder, make))

      ;
}
