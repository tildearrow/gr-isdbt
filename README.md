### ISDB-T transceiver in GNU Radio

**An open source implementation of a transceiver (i.e. receiver and transmitter) for the Digital Television standard ISDB-T (ARIB's STD-B31) in GNU Radio.**

**If you find the code useful, please consider starring the repository and/or citing our research paper (e.g. https://iie.fing.edu.uy/publicaciones/2020/LFAIM20/ regarding the transmitter or https://iie.fing.edu.uy/publicaciones/2016/LFGGB16/ regarding the receiver).**

**IMPORTANT**: It should work for all versions of GNU Radio starting at 3.7. Switch to the corresponding branch if using either GNU Radio 3.7 or 3.8. If you're using 3.9 or above, the master branch should work fine. It's been tested in Ubuntu 22.04 and 24.04.

Here in Uruguay we use the so-called "international" version, or ISDB-Tb (ABNT NBR 15601). However, the transmission scheme is exactly the same as in the original version, thus both references are equivalent.  

Since ISDB-T is similar to DVB-T, some of our blocks are heavily based on Bogdan Diaconescu's (BogdanDIA on github) DVB-T implementation (see https://github.com/BogdanDIA/gr-dvbt, which is now part of GNU Radio's base code). The transmitter side is based on the work of Javier and Santiago at https://github.com/jhernandezbaraibar/gr-isdbt-Tx. 

Any help and/or feedback is  welcome. 

**Notes and examples**

The **receiver** is now complete and tested. It is capable of demodulating the complete and 1-seg ISDB-T spectrum and its operation is such that a i5 PC is capable of showing the HD signal on-line. We foresee four modes of operation: 
- **Obtaining the transmission parameters.** ISDB-T has several transmission parameters which may be selected by the transmitter and may (and probably will) change among channels and segments. Our current implementation is not yet capable of automatically changing its parameters, and it's up to the user to configure the receiver correctly. However, a simple example is provided (see examples/obtaining_parameters.grc) which may be used to obtain such parameters. Transmission mode and CP length should be changed until a clear peak is viewed. Then, the OFDM Synchronization block should be configured accordingly, which will result in the TMCC Decoder printing the rest of the parameters. 
- **Vector Analyzer.** See examples/viewing_the_constellation.grc on how to view the received complex symbols. The receiver has been tested with all three transmission modes (2k, 4k and 8k FFT) and several CP lengths.  
- **Complete decoding and displaying the video/audio.** The receiver is capable of displaying on-line whole segments. **If a 1-seg transmission is present, the example in examples/rx_1seg_demo.grc may be used to decode it (it has been tested with a RTL cheap SDR)**. Layer B is decoded in examples/rx_demo.grc. Note that the transmission's parameters should be set accordingly by using examples/obtaining_parameters.grc. The output of both flowgraphs is a Transport Stream file which may be played by ffplay or mplayer (for instance). If before executing the flowgraph we create the file as a pipe (mkfifo test_out.ts), it should display the video online.   
- **Signal analyzer.** Our actual objective with the project is to develop a relatively inexpensive ISDB-T measurement equipment. In examples/fullseg_receiver_and_measurements.grc we present a flowgraph which measures several indicators, such as pre and post Viterbi BER, MER, channel frequency response, etc. In order to use it **you should install our accompanying OOT module gr-mer** (available at https://github.com/git-artes/gr-mer).  

The **transmitter** is complete and tested. It is currently capable of the following: 
- It works perfectly fine on software together with the receiver (see examples/full_transceiver.grc). A transport stream file to be used in this example is available on the project's webpage. If you are not interested in "seeing" a video image, simply feed it with random bytes. 
- Over the air transmission. We are currently able to transmit and receive the signal with SDRs (a B200 worked best for us as a transmitter) and we are successfully testing it on commercial TVs (see examples/tx_demo.grc which uses the TS files we share in our webpage). Please send us your feedback, **we are very interested**. 
- If you want to transmit your own videos (or your webcam) you may follow our step-by-step guide in https://iie.fing.edu.uy/investigacion/grupos/artes/wp-content/uploads/sites/13/2019/12/transmitting_webcam_or_videos.pdf. 

For more information (papers, recordings of ISDB-T signals, etc.) please visit our webpage: http://iie.fing.edu.uy/investigacion/grupos/artes/gr-isdbt/.    

**Build instructions**

For a system wide installation:

    git clone https://github.com/git-artes/gr-isdbt.git  
    cd gr-isdbt  
    mkdir build  
    cd build  
    cmake ../  
    make && sudo make install  

For a user space installation, or GNU Radio installed in a location different from the default location /usr/local:

    git clone https://github.com/git-artes/gr-isdbt.git  
    cd gr-isdbt  
    mkdir build  
    cd build  
    cmake -DCMAKE_INSTALL_PREFIX=<your_GNURadio_install_dir> ../
    make
    make install  

Please note that if you used PyBOMBS to install GNU Radio the DCMAKE_INSTALL_PREFIX should point to the PyBOMBS prefix. 

On Debian/Ubuntu based distributions, you may have to run:

    sudo ldconfig  

**Remarks**
- Instructions above assume you have downloaded and compiled GNU Radio (either with the build-gnuradio script or with PyBOMBS). If you've used a pre-compiled binary package (like in $ sudo apt-get install gnuradio), then you should also install gnuradio-dev (and naturally cmake and git, if they were not installed, plus libboost-all-dev, libcppunit-dev, liblog4cpp5-dev, swig, liborc-dev and libgsl-dev). A one-liner for all these is `$sudo apt-get install gnuradio-dev cmake git libboost-all-dev libcppunit-dev liblog4cpp5-dev swig liborc-dev libgsl-dev clang-format`.
- The above build instructions are general. You may accelerate the compilation time by using the -j flag. Please visit http://www.math-linux.com/linux/tip-of-the-day/article/speedup-gnu-make-build-and-compilation-process for a guide.   
- Note that gr-isdbt makes heavy use of VOLK. A profile should be run to make the best out of it. Please visit https://gnuradio.org/redmine/projects/gnuradio/wiki/Volk for instructions, or simply run:   

    volk_profile 


**FAQ**

*Q*: Cmake complains about unmet requirements. What's the problem?   
*A*: You should read the errors carefully (though we reckon they are sometimes mysterious). Most probably is a missing library. Candidates are Boost (in Ubuntu libboost-all-dev) or libcppunit (in Ubuntu libcppunit-dev).   

*Q*: Cmake complains about some Policy CMP0026 and LOCATION target property and who knows what else. Again, what's the problem?  
*A*: This is a problem with using Cmake with a version >= 3, which is installed in Ubuntu 16, for instance. The good news is that you may ignore all these warnings. 

*Q*: It is not compiling. What's the problem?  
*A*: Again, you should read carefully the errors. Again, it's most probably a missing library, for instance log4cpp (in Ubuntu liblog4cpp5-dev). If the problem is with the API of GNU Radio, you should update it.   

*Q*: I got the following error: "ModuleNotFoundError: No module named 'isdbt'". What's wrong? 

*A*: You probably didn't setup the PYTHONPATH correctly. It should include at least /usr/local/lib/python3/dist-packages. For a system-wide solution, you may edit `/etc/environment` and include the following line `PYTHONPATH="$PYTHONPATH:/usr/local/lib/python3/dist-packages"`. 

*Q*: I got the following error: "AttributeError: 'module' object has no attribute 'viterbi_decoder'" (or some other block). Why?  
*A*: This problem may be generated by several factors. Did you "sudo ldconfig"? Do you have PYTHONPATH set? (It should include at least /usr/local/lib/python3/dist-packages). Another possibility is that you don't have swig installed (in this case, you must uninstall gr-isdbt, delete CMakeCache.txt in the build directory, and re-install; that is, after installing swig).   

*Q*: Video is not playing. Why?   
*A*: In our experience, the best player is ffplay, which comes along ffmpeg. You should try it.    

*Q*: It complains about `AttributeError: 'gnuradio.gr.gr_python.logger' object has no attribute 'warning'`. Why?
*A*: This is a bug in certain versions of GNU Radio. See https://github.com/gnuradio/gnuradio/issues/6923 for a solution (or simply turn real-time scheduler to `off`).

IIE Instituto de Ingeniería Eléctrica  
Facultad de Ingeniería  
Universidad de la República  
Montevideo, Uruguay  
http://iie.fing.edu.uy/investigacion/grupos/artes/  
  
Please refer to the LICENSE file for contact information and further credits.   
