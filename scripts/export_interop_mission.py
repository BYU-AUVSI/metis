# -*- coding: utf-8 -*-
# Copyright 2019-2020 Sequoia Ploeg

"""
Simple Python script that downloads the active mission from the 
AUVSI interoperability server and pickles it to a file.

To use:
    1) An instance of the AUVSI Interop Server must be running and available
        to incoming connections
    2) interop_pkg must be up and running and connected to that server
    2) Run this script to save the mission.
"""

import os, pickle

import Tkinter, tkFileDialog

import metis.ros.utils as utils
from metis import Mission

def export(mission):
    root = Tkinter.Tk()
    root.withdraw()
    filename = tkFileDialog.asksaveasfilename(initialdir=os.path.expanduser('~/Documents'), confirmoverwrite=True, title = "Save mission as", defaultextension='.pkl')
    if type(filename) is str:
        with open(filename, 'wb') as f:
            pickle.dump(mission, f)
    
if __name__ == "__main__":
    mission = utils.get_mission()
    export(mission)
    