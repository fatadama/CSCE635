## @file
# Parse logs in a directory and determine which have empty control files, 
# i.d. which ones weren't used for automatic control

import sys
import os
# regular expressions
import re
# numpy
import numpy as np

## set Verbose to True to output if files are empty, etc.
VERBOSE = False

def main(logDir):
    print("Printing folders with nonempty logs in directory %s" % logDir)
    for x in os.walk(logDir):
        # x[0] is the name of folders in directory
        num = len(logDir)
        # relative path to the folder
        fpath = x[0]
        # name pf tje folder
        fname = x[0][num:]
        #  if the name looks like 2016%2d%2d_%2d%2d%2d it's a log directory
        try:
            year = int(fname[0:4])
            month = int(fname[4:6])
            day = int(fname[6:8])
            hr = int(fname[9:11])
            minute = int(fname[11:13])
            second = int(fname[13:15])
            # if we get this far, assume it's a log directory
            controlObj = np.genfromtxt(fpath+'/controlObjectLog.csv',dtype='string',delimiter=',',skip_header=0)
            # get number of rows
            rown = controlObj.shape[1]
            if controlObj.shape[0] > 1:
                print(fname)
            else:
                if VERBOSE:
                    print("Empty %s" % fname)
        except ValueError:
            if VERBOSE:
                print("ValueError in %s" % fname)
            pass
        except IOError:
            if VERBOSE:
                print("No log in %s" % fname)
            pass
        except IndexError:
            if VERBOSE:
                print("Empty file in %s" % (fname))
            pass
    return

if __name__=="__main__":
    logDir = '../xbee_bridge/'
    if len(sys.argv)>1:
        logDir = sys.argv[1]
    main(logDir)
