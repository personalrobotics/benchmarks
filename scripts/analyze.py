#!/usr/bin/env python
package_name='or_benchmarks'
import roslib; roslib.load_manifest(package_name)
import argparse, numpy, yaml
import matplotlib.pyplot as plt
import matplotlib.mlab as mlab
import pylab



def output(fig, path, size, fontsize=10):
    if path is not None:
#        fig.set_size_inches(size)
        fig.savefig(path, bbox_inches='tight', pad_inches=0.02)
    else:
        plt.show()

def analyze(datafiles, hists=False, title=None, outfile=None):
    data = {}

    for datafile in datafiles:
        with open(datafile, 'r') as f:
            print 'Loading results from file: %s' % datafile
            data[datafile] = yaml.load(f)
            print 'Done'


    # Calculate checks per second
    vals = []
    for datafile in datafiles:
        d = data[datafile]
        elapsed = float(d['elapsed_ms'])/1000.0
        checks = int(d['checks'])
        vals.append(float(checks)/elapsed)

        evals = [float(pt['elapsed_ms']) for pt in d['data']]
        if hists:
            pylab.figure()
            pylab.hist(evals, 10, histtype='stepfilled')
        

    # Generate the plot of checks per second
    fig = plt.figure()
    ax = fig.add_subplot(211)

    bar_width = 0.35
    offset = 0.2
    colors = ['r', 'g', 'b', 'c', 'm', 'y', 'k']
    rects = []
    max_height = 0.
    for idx in range(len(vals)):
        rects = plt.bar(offset + idx*bar_width,
                       vals[idx],
                       bar_width,
                       color = colors[idx],
                       alpha = 0.4,
                       label = datafiles[idx].split('_')[0])
        rect = rects[0]
        plt.text(rect.get_x()+rect.get_width()/2., 
                 1.05*rect.get_height(),
                 '%0.2f' % (vals[idx]),
                 ha='center',
                 va='bottom')
        if 1.05*rect.get_height() > max_height:
            max_height = 1.05*rect.get_height()

    plt.ylabel('Checks per second')
    plt.legend(frameon=False)
    if title is not None:
        plt.title(title)
    ax.set_xticks([])
    ax.set_xlim([0, 2.*offset + len(vals)*bar_width])
    ax.set_ylim([0, 1.2*max_height])

    # Now generate the plot of average second per check
    ax = fig.add_subplot(212)
    rects = []
    max_height = 0
    for idx in range(len(vals)):
        v = 1000.*1./vals[idx]
        rects = plt.bar(offset + idx*bar_width,
                        v,
                        bar_width,
                        color = colors[idx],
                        alpha = 0.4,
                        label = datafiles[idx].split('_')[0])
        rect = rects[0]
        plt.text(rect.get_x()+rect.get_width()/2., 
                 1.02*rect.get_height(),
                 '%0.6f' % (v),
                 ha='center',
                 va='bottom')

        if 1.02*rect.get_height() > max_height:
            max_height = 1.02*rect.get_height()
        
    plt.ylabel('Milliseconds per check')
    plt.legend(frameon=False)
    if title is not None:
        plt.title(title)
    ax.set_xticks([])
    ax.set_xlim([0, 2.*offset + len(vals)*bar_width])
    ax.set_ylim([0., 1.2*max_height])

    if outfile:
        output(fig, outfile, (5,5))

    plt.tight_layout()
    plt.show()


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

    analyze(args.datafiles, hists=args.hists, title=args.title)
