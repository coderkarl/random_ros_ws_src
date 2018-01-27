import numpy as np

print "Reading test_path.txt"
#read test_path.txt before starting threads and processes
pathX,pathY,pathAction = \
np.loadtxt('test_path.txt',skiprows=0,unpack = True,usecols=(0,1,2))
print "Done reading test_path.txt"
