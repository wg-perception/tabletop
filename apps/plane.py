#!/usr/bin/env python

import ecto

from ecto_opencv.highgui import imshow
from ecto_opencv.calib import DepthTo3d
from ecto_opencv.imgproc import cvtColor, Conversion
from ecto.opts import run_plasm, scheduler_options
from ecto_image_pipeline.io.source import create_source

from capture.ecto_cells.odometry import Odometry

from tabletop.ecto_cells.tabletop_table import PlaneFinder

def parse_args():
    import argparse
    parser = argparse.ArgumentParser(description='Find a plane in an RGBD image.')
    scheduler_options(parser.add_argument_group('Scheduler'))
    options = parser.parse_args()

    return options


if __name__ == '__main__':
    options = parse_args()

    plasm = ecto.Plasm()

    #setup the input source, grayscale conversion
    from ecto_openni import SXGA_RES, FPS_15
    source = create_source('image_pipeline','OpenNISource',image_mode=SXGA_RES,image_fps=FPS_15)
    rgb2gray = cvtColor('Grayscale', flag=Conversion.RGB2GRAY)
    plane_finder = PlaneFinder(l=600, nu=100)
    depth_to_3d = DepthTo3d()

    plasm.connect(source['image'] >> rgb2gray ['image'])

    #connect up the pose_est
    connections = [ source['image'] >> plane_finder['image'],
                    source['depth_raw'] >> depth_to_3d['depth'],
                    source['K'] >> depth_to_3d['K'],
                    depth_to_3d['points3d'] >> plane_finder['points3d'],
                    source['K'] >> plane_finder['K'] ]
    connections += [plane_finder['image'] >> imshow(name='hulls')[:],
                    source['image'] >> imshow(name='original')[:]]
    plasm.connect(connections)

    run_plasm(options, plasm, locals=vars())
