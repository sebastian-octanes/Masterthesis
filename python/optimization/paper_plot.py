# -*- coding: utf-8 -*-
"""
Created on Wed Feb  3 15:04:31 2016

@author: wuerll
"""
from __future__ import division
import os
import numpy as np
from matplotlib import rc
import matplotlib.pyplot as plt
from cycler import cycler

marker='xosd^v><phx+'
color='cmykrgb'
inches_per_pt = 1.0/72.27 # Convert pt to inch
# IEEE Paper
# fig_width_pt = 252  # Get this from LaTeX using \the\textwidth
# Fraunhofer docu
fig_width_pt = 374.87  # Get this from LaTeX using \the\textwidth

def latexify():
	# IEEE style
	font_size = 7
	#font_size = 9

	rc('text', usetex=True)
	rc('text.latex',preamble= [r'\usepackage{helvet}',
							   r'\usepackage{amsmath}'])
	rc('ps', usedistiller='xpdf') #required for conversion to .eps standard
	rc('font',
	   size=font_size,
	   family='sansserif',
	   serif='LMRoman10',
	)
	rc('lines',markersize = 3)
	rc('lines',linewidth = 1) #the width of line plots made using pyplot.plot
	#the square surrounding the entire plot (not ticks, not the y=0 line)
	rc('axes',linewidth =  0.5)
	#the width of rectangle strokes including the bars of a pyplot.bar plot,
	# legends, and legend labels of bar plots
	rc('patch',linewidth = 0.5)
	rc('axes',labelsize = font_size)
	rc('axes',titlesize = font_size)
	rc('legend',fontsize = font_size)
	rc('xtick',labelsize = font_size)
	rc('ytick',labelsize = font_size)
	rc('xtick.major',width = 0.5) #default: 0.5
	rc('xtick.minor',width = 0.5) #default: 0.5
	rc('ytick.major',width = 0.5) #default: 0.5
	rc('ytick.minor',width = 0.5) #default: 0.5
	rc('figure',dpi=100, figsize=(5,4.2))
	rc('savefig', dpi=300)
	rc('axes', color_cycle=['c', 'm', 'y', 'k','r', 'g', 'b'])
	#rc('axes', prop_cycle=(cycler('color', ['c', 'm', 'y', 'k', 'r', 'g', 'b'])+
	 #                          cycler('linestyle', ['-', ':', '--', '-.',
	 #                                               '-', '--', ':'])+
	 #                          cycler('marker', ['x', 'o', 's', 'd', 'p', '*', 'v'])))

def figsize(scale, ratio=0):
    """

    Parameters
    ----------
    scale : float
        the relative scale of the figure
    ratio : float, optional
        the aspect ratio, height = ratio * width. Defaults to zero, which
        results in the golden mean being used (width about 1.6 times height)

    Returns
    -------
    fig_size : list
        list of calculated width and heigth
    """
    golden_mean = (np.sqrt(5.0)-1.0)/2.0  # Aesthetic ratio
    fig_width = fig_width_pt*inches_per_pt*scale    # width in inches
    if(ratio == 0):
        fig_height = fig_width*golden_mean
    else:
        fig_height = fig_width*ratio
    fig_size = [fig_width,fig_height]
    return fig_size

def newfig(scale=1.0, ratio=0):
    """
    get a figure object and axes handle with the ma_plot default settings
    height = width*ratio
    Parameters
    ----------
    scale : float
        the relative scale of the figure
    ratio : float, optional
        the aspect ratio, height = ratio * width. Defaults to zero, which
        results in the golden mean being used (width about 1.6 times height)

    Returns
    -------
    fig, ax : matplotlib.pyplot.figure, axes

    """

    #width in x*\textwidth scale (0,1]
    fig = plt.figure(figsize=figsize(scale, ratio))
    ax = fig.add_subplot(111)
    return fig, ax

def new_empty_fig(scale=1.0, ratio=0):
    """
    Get an empty figure of the given scale and ratio with the axes not being
    intialized. This is useful if there are multiple axes to be set or you
    want to use some 3D graphics.

    Parameters
    ----------
    scale : float
        the relative scale of the figure
    ratio : float, optional
        the aspect ratio, height = ratio * width. Defaults to zero, which
        results in the golden mean being used (width about 1.6 times height)

    Returns
    -------
    fig : matplotlib.pyplot.figure
    """
    fig = plt.figure(figsize=figsize(scale, ratio))
    return fig


def savefig(basic_file_path=None, filename=None, extension='pdf', pad_pt=0, figure=None):
    """
    save the current figure object to the file system in a specified format.

    Parameters
    ----------
    basic_file_path : string
        could be a path or a path including the file e.g.
        /home/wuerll/workspace/detection/presentation/presentation_plot.py
    filename : string, optional
        the name of the file to save to. Default: None: The script will try to
        extract the filename from the basic_file_path. In the example above
        this would correspond to the filename being presentation_plot
    extension : string
        the format of the saved file. All formates that
         matplotlib.pyplot.savefig can handle are vaild
    pad_pt : float
        pad additional space around the figure. The unit is in points

    Returns
    -------

    """
    if basic_file_path is not None and os.path.exists(basic_file_path):
        # extract the path form the file path
        path = os.path.dirname(os.path.realpath(basic_file_path))
    else:
        #print("Given basic_file_path does not exist or None was provided. "
        #      "Using os.getcwd().")
        path = os.getcwd()

    path = os.path.join(path, "plots")

    if (filename is None and basic_file_path is not None
        and basic_file_path[-3:] == '.py'):
        # try to extract a name from the sender
        filename = os.path.basename(os.path.realpath(basic_file_path))[:-3]

    if filename is None:
        raise AttributeError("Nor could the filename not be determined,"
                             " neither one was provided as a parameter")
    savename = os.path.join(path, filename)

    # create the plots path if it does not exist
    if not os.path.exists(path):
        os.makedirs(path)

    # add extension
    if savename[-len(extension)+1:] != '.' + extension:
        savename = savename + '.' + extension
    if figure == None:
         plt.savefig(savename, format=extension, bbox_inches='tight',
                pad_inches=0.01+pad_pt*inches_per_pt)
    else:
         figure.savefig(savename, format=extension, bbox_inches='tight',
                pad_inches=0.01+pad_pt*inches_per_pt)

if __name__ == "__main__":
    latexify()
    fig, ax = newfig()
    for i in range(7):
        ax.plot(np.arange(10), np.arange(10)+2*i,
                label="test $i = ${:d}".format(i))
    ax.set_ylabel("Some formula: $y = x$",  labelpad=1)
    ax.set_xlabel("$x$ magic", labelpad=1)
    ax.legend(handlelength=3)
    ax.grid()
    savefig(__file__)
    plt.show()
