#!/usr/bin/env python
import os

import argparse, numpy, yaml
from ss_plotting.make_plots import plot_bar_graph
import logging
logger = logging.getLogger('or_benchmarks')

def analyze_planning_benchmarks(datafiles, title=None, out_basename=None):
    data = {}

    for datafile in datafiles:
        with open(datafile, 'r') as f:
            print 'Loading results from file: %s' % datafile
            data[datafile] = yaml.load(f)
            print 'Done'


    # Calculate checks per second
    series = []
    series_success = []
    for datafile in datafiles:
        d = data[datafile]['configurations']
        elapsed = [pt['elapsed_ms'] for pt in d]
        found_solutions = [int(pt['found_solution']) for pt in d]
        num_pts = len(d)
        avg_plan_time = sum(elapsed)/float(num_pts)
        series.append([avg_plan_time])
        series_success.append([sum(found_solutions)/float(num_pts)])

    from palettable.colorbrewer import diverging
    colors = diverging.BrBG_4.colors
    series_labels = [name.split('_')[0] for name in datafiles]

    # Generate the plot of checks per second
    outfile = None
    if out_basename is not None:
        outfile = '%s.%s.%s' % (out_basename, 'plantime', 'png')
    plot_bar_graph(series, colors[:len(series)],
                   plot_ylabel='Average plan time (s)',
                   plot_title = title,
                   fontsize=12,
                   series_use_labels=True,
                   legend_location=None,
                   series_labels=series_labels,
		   show_plot=True,
                   savefile=outfile,
                   savefile_size=(7,3))
    if outfile is not None:
        logger.info('Saved average plan time to file %s' % outfile)
        
    # Now generate the plot of average second per check
    if out_basename is not None:
        outfile = '%s.%s.%s' % (out_basename, 'success', 'png')
    plot_bar_graph(series_success, colors[:len(series)],
                   plot_ylabel='Success Percent',
                   plot_title = title,
                   fontsize=12,
                   legend_location=None,
                   series_use_labels=True,
                   series_labels=series_labels,
		   show_plot=True,
                   savefile=outfile,
                   savefile_size=(7,3))
    if outfile is not None:
        logger.info('Saved success percentage to file %s' % outfile)

if __name__ == '__main__':

    parser = argparse.ArgumentParser(description="Generate plots for a set of datafiles")
    parser.add_argument("--title", type=str, default=None,
                        help="The title for plots")
    parser.add_argument("--datafiles", type=str, nargs='+',
                        help="The datafiles to load and analyze")
    parser.add_argument("--outdir", type=str, default=None,
                        help="The directory to save plots to")


    args = parser.parse_args()

    if len(args.datafiles) == 0:
        print 'Must provide a datafile to analyze.'
        exit(0)

    analyze_planning_benchmarks(args.datafiles, title=args.title,
                                out_basename=args.outdir)
