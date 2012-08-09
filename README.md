Parallel Rapidly-exploring Randomized Trees *
=============================================

This code is an implementation of Parallel RRT*--an asymptoptically
optimal robotic motion planning algorithm.  It utilizes
multi-core/multi-processor SMP threading and atomic operations to
implement a RRT* that scales linearly with the number of cores in use.

To enable super-linear speedup, each thread may be given a partition
of the configuation space to sample.  This approach keeps the
working-set of each thread smaller than the non-partitioned approach,
allowing for more effective use of CPU caches.  It's not without its
downside however, it is unknown how partitioning will effect the
speedup when given certain edge-case robotic systems, such as ones
that would have partitions in a completed obstructed portion of
configuration space.

Note on Turbo Boost:
On some SMP configurations, having fewer cores in use allows the
hardware to run at faster clock speeds (see Intel 'Turbo Boost').  In
this case, linear and super-linear speedup may not be seen, however
there will still be an overall speedup benefit.

Note on Simultaneous Multithreading (SMT)
-----------------------------------------

On some SMP configurations, logical processor threads share
computational cores (e.g. Intel Hyper-Threading), which allows more
threads to run simultenously at the expense of stalling for competing
access to shared computational units.  The net effect is usually that
2 threads on 1 core do not run as fast on 2 threads on 2 cores.  PRRT*
has been tested to work well in the presense of SMT, but the
(super-)linear-speedup is not easily observed or for that matter
computed.  As an example, a 2-core processor might experience a 2.1x
speedup on one simulation with SMT disabled.  Enabling SMT (for a
total of 4 logical threads in this example) and running 4-threads in
parallel might see a 3.2x speedup.  In terms of threads the speedup is
sub-linear, but in terms of cores, we'll... we're doing pretty well.

As a cautionary note however, partitioned sampling may run unevenly
with SMT enabled, since threads will run slower or faster depending on
what other thread is scheduled on the same core.  In our experience it
is best to run with all available threads (e.g. 4-core w/ 2x SMT
should use 8 threads), in the presense of SMT when using partitioned
sampling.


Tested implementation:
______________________

This implementation has been tested on a Intel and AMD x86_64-based
SMP machines in various configurations.  The memory model of the these
machines is strong enough to enable certain assumptions about the
memory access patterns.  This implementation may not run well on other
SMP architectures.  If you are interested in testing/porting the code
to run on another architecture, please contact the authors.


BUILDING
--------

Note: this build requires GNU make.  It make be available as "gmake"
on your system, if "make" below does not work, then try "gmake" where
it says "make".

    % ./configure
    % make

Optionally you can specify after "./configure":

    --disable-openmp   to use pthreads directly
    --enable-crc       to enable runtime crc checking of data-structures

If changing options, you should "make clean" before running make again.

For example:

    % ./configure --disable-openmp --enable-crc
    % make clean
    % make

To run the example program:

    % src/prrts -t4 -n20000 -r

Runs (-n=)20000 samples with 4(-t)hreads with partitioned (-r)egional sampling.



