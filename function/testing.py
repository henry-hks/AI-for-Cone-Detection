import numpy as np

mylist = np.array([[-1,-1],[1,1]])

if mylist[0].all() >= 1:
    print("yes")
if mylist[0].all() == -1:
    print("yes2")
if mylist[1].all() == 1:
    print("yes3")
