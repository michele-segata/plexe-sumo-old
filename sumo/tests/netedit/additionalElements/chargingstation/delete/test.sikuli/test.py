# import common functions for netedit tests
import os
import sys

testRoot = os.path.join(os.environ.get('SUMO_HOME', '.'), 'tests')
neteditTestRoot = os.path.join(
    os.environ.get('TEXTTEST_HOME', testRoot), 'netedit')
sys.path.append(neteditTestRoot)
import neteditTestFunctions as netedit

# Open netedit
neteditProcess, match = netedit.setupAndStart(neteditTestRoot, False)

# go to additional mode
netedit.additionalMode()

# select chargingStation
netedit.changeAdditional("chargingStation")

# create chargingStation in mode "reference left"
netedit.leftClick(match, 250, 250)

# Change to delete
netedit.deleteMode()

# delete created chargingStation
netedit.leftClick(match, 260, 250)

# delete first loaded chargingStation
netedit.leftClick(match, 450, 250)

# delete lane with the second loaded chargingStation
netedit.leftClick(match, 200, 200)

# Check undo
netedit.undo(match, 3)

# Change to delete
netedit.deleteMode()

# disble 'Automatically delete additionals'
netedit.changeAutomaticallyDeleteAdditionals(match)

# try to delete lane with the second loaded charging station (doesn't allowed)
netedit.leftClick(match, 200, 200)

# wait warning
netedit.waitAutomaticallyDeleteAdditionalsWarning()

# check redo
netedit.redo(match, 3)

# save additionals
netedit.saveAdditionals()

# save newtork
netedit.saveNetwork()

# quit netedit
netedit.quit(neteditProcess, False, False)
