#!/usr/bin/env python
package_name='collision_checking'
import roslib; roslib.load_manifest(package_name)
import argparse, numpy, yaml
import matplotlib.pyplot as plt
import matplotlib.mlab as mlab
import pylab

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

    data = {}

    for datafile in args.datafiles:
        with open(datafile, 'r') as f:
            data[datafile] = yaml.load(f)


    # Calculate checks per second
    vals = []
    means = []
    variances = []
    for datafile in args.datafiles:
        d = data[datafile]
        elapsed = float(d['elapsed_ms'])/1000.0
        checks = int(d['checks'])
        vals.append(float(checks)/elapsed)
        means.append(float(d['mean_ms']))
        variances.append(float(d['variance_ms']))

        evals = [float(pt['elapsed_ms']) for pt in d['data']]
        if args.hists:
            pylab.figure()
            pylab.hist(evals, 10, histtype='stepfilled')
        

    # Generate the plot of checks per second
    fig = plt.figure()
    ax = fig.add_subplot(211)

    bar_width = 0.35
    offset = 0.2
    colors = ['r', 'g', 'b', 'c', 'm', 'y', 'k']
    rects = []
    for idx in range(len(vals)):
        rects = plt.bar(offset + idx*bar_width,
                       vals[idx],
                       bar_width,
                       color = colors[idx],
                       alpha = 0.4,
                       label = args.datafiles[idx])
        rect = rects[0]
        plt.text(rect.get_x()+rect.get_width()/2., 
                 1.05*rect.get_height(),
                 '%0.2f' % (vals[idx]),
                 ha='center',
                 va='bottom')

    plt.xlabel('Collision Checker')
    plt.ylabel('Checks per second')
    plt.legend()
    if args.title is not None:
        plt.title(args.title)
    ax.set_xticks([])
    ax.set_xlim([0, 2.*offset + len(vals)*bar_width])

    # Now generate the plot of average second per check
    ax = fig.add_subplot(212)
    rects = []
    for idx in range(len(means)):
        rects = plt.bar(offset + idx*bar_width,
                        means[idx],
                        bar_width,
                        color = colors[idx],
                        alpha = 0.4,
                        yerr = numpy.sqrt(variances[idx]),
                        label = args.datafiles[idx])
        rect = rects[0]
        plt.text(rect.get_x()+rect.get_width()/2., 
                 1.02*rect.get_height(),
                 '%0.6f' % (means[idx]),
                 ha='center',
                 va='bottom')
        
    plt.xlabel('Collision Checker')
    plt.ylabel('Milliseconds per check')
    plt.legend()
    if args.title is not None:
        plt.title(args.title)
    ax.set_xticks([])
    ax.set_xlim([0, 2.*offset + len(vals)*bar_width])

    plt.tight_layout()
    plt.show()

