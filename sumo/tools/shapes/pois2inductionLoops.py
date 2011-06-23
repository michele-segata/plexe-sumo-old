#!/usr/bin/env python
"""
@file    pois2inductionLoops.py
@author  Daniel.Krajzewicz@dlr.de
@date    2010-02-18
@version $Id$


Converts a given pois located on lanes into induction loop detectors;
Each poi is replicated to cover all lanes of the road.
The detectors are named <POINAME>__l<LANE_INDEX>

Call: pois2inductionLoops.py <NET> <POIS> <OUTPUT>

Copyright (C) 2010-2011 DLR (http://www.dlr.de/) and contributors
All rights reserved
"""


import os, string, sys, StringIO
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
import sumolib


if len(sys.argv) < 4:
    print "Usage: " + sys.argv[0] + " <NET> <POIS> <OUTPUT>"
    sys.exit()
parser = make_parser()
print "Reading net..."
net = sumolib.net.readNet(sys.argv[1])

print "Reading PoIs..."
pois = sumolib.poi.readPois(sys.argv[2])

fdo = open(sys.argv[3], "w")
print >> fdo, '<additional>'
for poi in pois:
    if not poi._lane:
        print "Error: poi '%s' is not on a lane" % poi._id
        continue
    edge = poi._lane[:poi._lane.rfind('_')]
    edge = net._id2edge[edge]
    for i, l in enumerate(edge._lanes):
        print >> fdo, '    <e1-detector id="%s__l%s" lane="%s" pos="%s" freq="60" file="e1_output.xml"/>' % (poi._id, i, l.getID(), poi._pos)
    print >> fdo, ''
print >> fdo, '</additional>'
fdo.close()


