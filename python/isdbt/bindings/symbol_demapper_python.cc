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
/* BINDTOOL_HEADER_FILE(symbol_demapper.h) */
/* BINDTOOL_HEADER_FILE_HASH(6907fe9f67df5de09abcba7ba6e25a93) */
/***********************************************************************************/

#include <pybind11/complex.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

namespace py = pybind11;

#include <gnuradio/isdbt/symbol_demapper.h>
// pydoc.h is automatically generated in the build directory
#include <symbol_demapper_pydoc.h>

void bind_symbol_demapper(py::module &m) {

  using symbol_demapper = ::gr::isdbt::symbol_demapper;

  py::class_<symbol_demapper, gr::sync_block, gr::block, gr::basic_block,
             std::shared_ptr<symbol_demapper>>(m, "symbol_demapper",
                                               D(symbol_demapper))

      .def(py::init(&symbol_demapper::make), py::arg("mode"),
           py::arg("segments_A"), py::arg("constellation_size_A"),
           py::arg("segments_B"), py::arg("constellation_size_B"),
           py::arg("segments_C"), py::arg("constellation_size_C"),
           D(symbol_demapper, make))

      ;
}
