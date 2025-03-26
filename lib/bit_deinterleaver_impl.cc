/* -*- c++ -*- */
/*  
 * Copyright 2015, 2016, 2017, 2018, 2019, 2020, 2021
 *   Federico "Larroca" La Rocca <flarroca@fing.edu.uy>
 *   Pablo Belzarena 
 *   Gabriel Gomez Sena 
 *   Pablo Flores Guridi 
 *   Victor Gonzalez Barbone
 *
 *   Instituto de Ingenieria Electrica, Facultad de Ingenieria,
 *   Universidad de la Republica, Uruguay.
 *  
 * This is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 3, or (at your option)
 * any later version.
 *  
 * This software is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *  
 * You should have received a copy of the GNU General Public License
 * along with this software; see the file COPYING.  If not, write to
 * the Free Software Foundation, Inc., 51 Franklin Street,
 * Boston, MA 02110-1301, USA.
 *
 */

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include <gnuradio/io_signature.h>
#include "bit_deinterleaver_impl.h"

#include <stdio.h>

namespace gr {
    namespace isdbt {

        // shouldn't these go somewhere else??
        const int bit_deinterleaver_impl::
            d_data_carriers_mode1 = 96; 
        const int bit_deinterleaver_impl::
            d_total_segments = 13; 
        const int bit_deinterleaver_impl::
            d_total_layers = 1; 

        const int bit_deinterleaver_impl::
            d_delay_qpsk[2] = {0, 120}; 
        const int bit_deinterleaver_impl::
            d_delay_16qam[4] = {0, 40, 80, 120}; 
        const int bit_deinterleaver_impl::
            d_delay_64qam[6] = {0, 24, 48, 72, 96, 120}; 

        bit_deinterleaver::sptr
            bit_deinterleaver::make(int mode, int layer, int segments, int constellation_size)
            {
                return gnuradio::get_initial_sptr
                    (new bit_deinterleaver_impl(mode, layer, segments, constellation_size));
            }

        /*
         * The private constructor
         */
        bit_deinterleaver_impl::
            bit_deinterleaver_impl(int mode, int layer, int segments, int constellation_size)
            	: gr::sync_interpolator("bit_deinterleaver",
                    gr::io_signature::make(1, 1, 
                        sizeof(unsigned char) * d_total_segments *
                        d_data_carriers_mode1 * ((int)pow(2.0, mode-1))),
                   // gr::io_signature::make3(1, 1, sizeof(unsigned char)), d_total_segments * d_data_carriers_mode1 * ((int)pow(2.0, mode-1))) Original implementation
                       gr::io_signature::make(1, 1, sizeof(unsigned char)), segments * d_data_carriers_mode1 * ((int)pow(2.0, mode-1))) //Implementation that still works
                                       //gr::io_signature::make3(1, 3, sizeof(unsigned char), sizeof(unsigned char), sizeof(unsigned char)), d_total_segments* d_data_carriers_mode1 * ((int)pow(2.0, mode-1)))
                                       //gr::io_signature::make3(1, 3, sizeof(unsigned char), sizeof(unsigned char), sizeof(unsigned char)), (segments_A* d_data_carriers_mode1 * ((int)pow(2.0, mode-1)), segments_B* d_data_carriers_mode1 * ((int)pow(2.0, mode-1)), segments_C* d_data_carriers_mode1 * ((int)pow(2.0, mode-1))))
        {
            d_mode = mode; 
            d_layer = layer;
            d_carriers_per_segment = d_data_carriers_mode1 * 
                ((int)pow(2.0, mode-1)); 
            d_noutput = d_total_segments * d_carriers_per_segment; 
	    printf("d_noutput: %d\n",d_noutput);

            init_params(segments,constellation_size);
            for (int lay = 0; lay<d_total_layers; lay++){
                d_shift.push_back(new std::deque<unsigned char>(120,0)); 
            }

            message_port_register_in(pmt::mp("params"));
            set_msg_handler(pmt::mp("params"),[this](const pmt::pmt_t& msg) {
              handle_tmcc(msg);
            });
        }

        /*
         * Our virtual destructor.
         */
        bit_deinterleaver_impl::~bit_deinterleaver_impl()
        {
        }

        void bit_deinterleaver_impl::init_params(int segments, int constellation_size) {
            d_const_size = constellation_size;
            d_num_bits = log2(constellation_size);
	    d_nsegments = segments;
            printf("Segments: %d\n", segments);

            d_noutput_real = d_nsegments * d_carriers_per_segment; 
			printf("d_noutput REAL: %d\n",d_noutput_real);

            if (d_const_size==4) {
				d_delay = d_delay_qpsk;
			} else if (d_const_size==16) {
				d_delay = d_delay_16qam;
			} else if (d_const_size==64) {
				d_delay = d_delay_64qam;
			} else {
				printf("bit_deinterleaver: error in d_const_size %d\n",d_const_size);                  
			}

            set_min_noutput_items(d_noutput_real);
        }

        void bit_deinterleaver_impl::handle_tmcc(const pmt::pmt_t& msg) {
          printf("bit deinterleaver: I've got my eye on you.\n");
        }

        /*
         * Our work function.
         */
        int bit_deinterleaver_impl::work(int noutput_items,
                        gr_vector_const_void_star &input_items,
                        gr_vector_void_star &output_items)
            {	
                const unsigned char *in = (const unsigned char*) input_items[0];
                unsigned char *out = (unsigned char *) output_items[0];
                //unsigned char *out_A = (unsigned char *) output_items[0];
				//unsigned char *out_B = (unsigned char *) output_items[1];
				//unsigned char *out_C = (unsigned char *) output_items[2];
               
				//bool out_B_connected = output_items.size()>=2;
				//bool out_C_connected = output_items.size()>=3;

				// Do <+signal processing+>
                unsigned char aux; 
                unsigned char mask; 
			for (int i=0; i<noutput_items/d_noutput; i++)
			{
			printf("---BIT DEINTERLEAVER-> noutput_items = %d\n", noutput_items);				
                        for (int carrier = 0; carrier<d_noutput_real; carrier++)
                        {
                            	// add new input symbol at beginning of container
                            	// Older symbols are at bigger indexes for consistency
                            	// with the d_delay implementation. 
                            	d_shift[0]-> push_front(in[i*d_noutput + carrier]); 
                            	// Initialize aux variables to construct output
                            	aux = 0; 
                            	mask = 1; 
                            	for (int b=0; b<d_num_bits; b++){
                                // Least significant bits more delayed in interleaver,
                                // so now delay more the most significant bits
                                aux |= (*d_shift[0])[d_delay[b]] & mask; 
                                //aux |= d_shift[layer]->at(d_delay[b]) & mask; 
                                mask = mask << 1;  
                            	
								}
								
								d_shift[0]->pop_back();
								out[i*d_noutput+carrier] = aux; 
								
						}
                          
					//} 
                }
                // Tell runtime system how many output items we produced.
                return noutput_items;
            }

    } /* namespace isdbt */
} /* namespace gr */

