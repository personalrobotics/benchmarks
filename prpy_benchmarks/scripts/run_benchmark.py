#!/usr/bin/env python

import logging
logger = logging.getLogger('prpy_benchmarks')

if __name__ == '__main__':
    
    import argparse
    parser = argparse.ArgumentParser('Execute a single benchmark')
    parser.add_argument("-q", "--queryfile", type=str, required=True,
                        help="The query file to execute")
    parser.add_argument("-p", "--plannerfile", type=str, required=True,
                        help="The file containing planner metadata to use for this benchmark")
    parser.add_argument("-o", "--outfile", type=str,
                        help="The file to write results to")

    args = parser.parse_args()

    import herbpy
    env, robot = herbpy.initialize(sim=True)

    from prpy_benchmarks.benchmark_utils import execute_benchmark
    execute_benchmark(args.queryfile, args.plannerfile, 
                      env=env, robot=robot,
                      outfile=args.outfile)
                        
