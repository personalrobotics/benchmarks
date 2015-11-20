#!/usr/bin/env python
import os

# Thanks to Chris Dellin for this nice hack. :-)
if ord(os.environ.get('ROS_DISTRO', 'hydro')[0]) <= ord('f'):
    package_name='or_benchmarks'
    import roslib; roslib.load_manifest(package_name)

import argparse, numpy, yaml
from ss_plotting.make_plots import plot_bar_graph
import logging
logger = logging.getLogger('or_benchmarks')

def analyze(datafiles, title=None, out_basename=None):
    data = {}

    for datafile in datafiles:
        with open(datafile, 'r') as f:
            print 'Loading results from file: %s' % datafile
            data[datafile] = yaml.load(f)
            print 'Done'


    # Calculate checks per second
    series = []
    for datafile in datafiles:
        d = data[datafile]
        elapsed = float(d['elapsed_ms'])/1000.0
        checks = int(d['checks'])
        checks_per_second = float(checks)/elapsed
        series.append([checks_per_second])

    from palettable.colorbrewer import diverging
    colors = diverging.BrBG_7.colors
    colors.reverse()
    series_labels = [name.split('_')[0] for name in datafiles]

    # Generate the plot of checks per second
    outfile = None
    if out_basename is not None:
        outfile = '%s.%s.%s' % (out_basename, 'cps', 'png')
    plot_bar_graph(series, colors[:len(series)],
                   plot_ylabel='Checks per second',
                   plot_title = title,
                   fontsize=12,
                   series_use_labels=True,
                   legend_location=None,
                   series_labels=series_labels,
                   savefile=outfile,
                   savefile_size=(7,3))
    if outfile is not None:
        logger.info('Saved checks per second data to file %s' % outfile)
        
    # Now generate the plot of average second per check
    if out_basename is not None:
        outfile = '%s.%s.%s' % (out_basename, 'mspc', 'png')
    new_series = [[1000.*1./s[0]] for s in series]
    plot_bar_graph(new_series, colors[:len(series)],
                   plot_ylabel='Milliseconds per check',
                   plot_title = title,
                   fontsize=12,
                   legend_location=None,
                   series_use_labels=True,
                   series_labels=series_labels,
                   savefile=outfile,
                   savefile_size=(7,3))
    if outfile is not None:
        logger.info('Saved milliseconds per check data to file %s' % outfile)

if __name__ == '__main__':

    parser = argparse.ArgumentParser(description="Generate plots for a set of datafiles")
    parser.add_argument("--title", type=str, default=None,
                        help="The title for plots")
    parser.add_argument("--datafiles", type=str, nargs='+',
                        help="The datafiles to load and analyze")
    parser.add_argument("--hists", action='store_true',
                        help="Generate histograms of time values")


    args = parser.parse_args()

    if len(args.datafiles) == 0:
        print 'Must provide a datafile to analyze.'
        exit(0)

    analyze(args.datafiles, title=args.title)
