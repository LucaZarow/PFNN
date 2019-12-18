"""
Generate footsetp & phase from BVH
Author: Crazii(Xiaofeng)
"""

import sys
import os
import numpy as np
import scipy.interpolate as interpolate
import scipy.ndimage.filters as filters

sys.path.append('./motion')

import BVH as BVH
import Animation as Animation
from Quaternions import Quaternions
from Pivots import Pivots

to_meters = 5.6444
njoints = 31

lfoot = 4
rfoot = 9
ltoe = 5
rtoe = 10

data_terrain = [
    './data/animations/05_01.bvh',
]

for data in data_terrain:

    print('Processing Clip %s' % data)
    
    footsteps = []
    
    """ Load Data """
    
    anim, names, _ = BVH.load(data)
    anim.offsets *= to_meters
    anim.positions *= to_meters
    num_frames = anim.positions.shape[0]
    #anim = anim[::2]
    global_xforms = Animation.transforms_global(anim)
    global_positions = global_xforms[:,:,:3,3] / global_xforms[:,:,3:,3]
    tlen = len(global_positions)
    #print(tlen)
    
    avg_lfs = 0
    avg_lts = 0
    avg_rfs = 0
    avg_rts = 0

    AVG_WINDOW = 5
    AVG_RANGE = int((AVG_WINDOW-1)/2)
    # RANGE_LFS = [0 for x in range(AVG_WINDOW)]
    # RANGE_LTS = [0 for x in range(AVG_WINDOW)]
    # RANGE_RFS = [0 for x in range(AVG_WINDOW)]
    # RANGE_RTS = [0 for x in range(AVG_WINDOW)]
    # for j in range(0, AVG_WINDOW, 1):
        # RANGE_LFS[j] = np.linalg.norm(global_positions[1+j,lfoot] - global_positions[j,lfoot])
        # RANGE_LTS[j] = np.linalg.norm(global_positions[1+j,ltoe] - global_positions[j,ltoe])
        # RANGE_RFS[j] = np.linalg.norm(global_positions[1+j,rfoot] - global_positions[j,rfoot])
        # RANGE_RTS[j] = np.linalg.norm(global_positions[1+j,rtoe] - global_positions[j,rtoe])
    # avg_lfs = np.mean(RANGE_LFS)
    # avg_lts = np.mean(RANGE_LTS)
    # avg_rfs = np.mean(RANGE_RFS)
    # avg_rts = np.mean(RANGE_RTS)

    LR = 0
    for i in range(AVG_RANGE, tlen-AVG_RANGE-1, 1):
        #ACC_EPSILON = 0
        ACC_EPSILON = 0.175
        #ACC_EPSILON = 0.006
        MULTIPLIER = 2.875
        LHEEL_SPEED_EPSILON = 0.375
        RHEEL_SPEED_EPSILON = 0.375
        LTOE_SPEED_EPSILON = 0.0875
        RTOE_SPEED_EPSILON = 0.0875
        # LHEEL_SPEED_EPSILON = avg_lfs * 0.285
        # RHEEL_SPEED_EPSILON = avg_rfs * 0.285
        # LTOE_SPEED_EPSILON = avg_lts * 0.285
        # RTOE_SPEED_EPSILON = avg_rts * 0.285

        LFS = np.linalg.norm(global_positions[i,lfoot] - global_positions[i-1,lfoot]) # left foot
        LTS = np.linalg.norm(global_positions[i,ltoe] - global_positions[i-1,ltoe]) # left toe	
        RFS = np.linalg.norm(global_positions[i,rfoot] - global_positions[i-1,rfoot])
        RTS = np.linalg.norm(global_positions[i,rtoe] - global_positions[i-1,rtoe]) 

        # #uppdate average
        # RANGE_LFS.append(np.linalg.norm(global_positions[i+AVG_RANGE,lfoot] - global_positions[i+AVG_RANGE-1,lfoot]))
        # RANGE_LTS.append(np.linalg.norm(global_positions[i+AVG_RANGE,ltoe] - global_positions[i+AVG_RANGE-1,ltoe]))
        # RANGE_RFS.append(np.linalg.norm(global_positions[i+AVG_RANGE,rfoot] - global_positions[i+AVG_RANGE-1,rfoot]))
        # RANGE_RTS.append(np.linalg.norm(global_positions[i+AVG_RANGE,rtoe] - global_positions[i+AVG_RANGE-1,rtoe]))
    
        # avg_lfs -= RANGE_LFS[0]/AVG_WINDOW
        # avg_lfs += RANGE_LFS[-1]/AVG_WINDOW
        # avg_lts -= RANGE_LTS[0]/AVG_WINDOW
        # avg_lts += RANGE_LTS[-1]/AVG_WINDOW
        # avg_rfs -= RANGE_RFS[0]/AVG_WINDOW
        # avg_rfs += RANGE_RFS[-1]/AVG_WINDOW
        # avg_rts -= RANGE_RTS[0]/AVG_WINDOW
        # avg_rts += RANGE_RTS[-1]/AVG_WINDOW
        
        # RANGE_LFS.pop(0)
        # RANGE_LTS.pop(0)
        # RANGE_RFS.pop(0)
        # RANGE_RTS.pop(0)
        # #end update average
                        
        if LR != -1: #and LHEEL_SPEED_EPSILON > 0.001 and LTOE_SPEED_EPSILON > 0.001:
        
            OLD_LFS = np.linalg.norm(global_positions[i-1,lfoot] - global_positions[i-2,lfoot]) # left foot
            OLD_LTS = np.linalg.norm(global_positions[i-1,ltoe] - global_positions[i-2,ltoe])# left toe
            NEW_LFS = np.linalg.norm(global_positions[i+1,lfoot] - global_positions[i,lfoot]) # left foot
            NEW_LTS = np.linalg.norm(global_positions[i+1,ltoe] - global_positions[i,ltoe]) # left toe

            #deviate of speed (acceleration)
            # pi-2   pi-1     pi-1  pi   pi   pi+1
            #	 \  /            \ /      \  /
            #    si-1            si      si+1
            #           \       /   \    /
            #             -ai        +ai+1
            dLFS = NEW_LFS - LFS
            dLTS = NEW_LTS - LTS
            dOLD_LFS = LFS - OLD_LFS
            dOLD_LTS = LTS - OLD_LTS
        
            if(LFS < LHEEL_SPEED_EPSILON and LTS < LTOE_SPEED_EPSILON \
            or ((LFS < LHEEL_SPEED_EPSILON*MULTIPLIER and LTS < LTOE_SPEED_EPSILON*MULTIPLIER) and ((dLFS >= ACC_EPSILON and dOLD_LFS <= -ACC_EPSILON) or (dLTS >= ACC_EPSILON and dOLD_LTS < -ACC_EPSILON))) ):
                if LR == 0:
                    LR = -1
                if len(footsteps)>0:	#R must start first
                    if LR != -1:
                        LR = -1
                        print("L %d: " %(i))
                        footsteps.append(i)

        elif LR != 1 :#and RHEEL_SPEED_EPSILON > 0.00175 and RTOE_SPEED_EPSILON > 0.00175:
        
            OLD_RFS = np.linalg.norm(global_positions[i-1,rfoot] - global_positions[i-2,rfoot])
            OLD_RTS = np.linalg.norm(global_positions[i-1,rtoe] - global_positions[i-2,rtoe])
            NEW_RFS = np.linalg.norm(global_positions[i+1,rfoot] - global_positions[i,rfoot])
            NEW_RTS = np.linalg.norm(global_positions[i+1,rtoe] - global_positions[i,rtoe])
            dRFS = NEW_RFS - RFS
            dRTS = NEW_RTS - RTS
            dOLD_RFS = RFS - OLD_RFS
            dOLD_RTS = RTS - OLD_RTS
        
            if( RFS < RHEEL_SPEED_EPSILON and RTS < RTOE_SPEED_EPSILON \
            or (RFS < RHEEL_SPEED_EPSILON*MULTIPLIER and RTS < RTOE_SPEED_EPSILON*MULTIPLIER and ((dRFS >= ACC_EPSILON and dOLD_RFS < -ACC_EPSILON) or (dRTS >= ACC_EPSILON and dOLD_RTS < -ACC_EPSILON)))):
                LR = 1
                print("R %d: " %(i))
                footsteps.append(i)
    
    #remove redundant frames
    i = 0
    deleted = 0
    while i < len(footsteps):
        # odd frame number: leave first one and delete the others
        c = 0
        while i+c < len(footsteps)-1 and footsteps[i+c+1] - footsteps[i+c] <= 5:
            c = c + 1
        if c > 0:
            c = c + 1
        if c % 2 == 1 and deleted > 0:
            i = i + 1
            
        while i < len(footsteps)-1 and footsteps[i+1] - footsteps[i] <= 5:
            del footsteps[i]
            del footsteps[i]
            deleted = deleted + 1
        i=i+1
    
    #print(footsteps)
    f = open(data.replace('.bvh','_footsteps.txt'), 'w')
        
    flen = len(footsteps)
    for i in range(0, int(flen/2), 1):
        f.writelines("%d %d\n" % (footsteps[i*2], footsteps[i*2+1]) )
    
    if flen%2 == 1:
        f.writelines("%d " % footsteps[flen-1])
    f.close()
    
    f = open(data.replace('.bvh','_contact_nodes.txt'), 'w')

    pivot_frame=0
    for i in range(0, flen):
        for j in range(pivot_frame, footsteps[i]):
            if(i%2==0):
                last_node = "RightFoot"                
            else:
                last_node = "LeftFoot"
            f.writelines(last_node + "\n")
        pivot_frame = footsteps[i]

    for i in range(pivot_frame, num_frames-1):
        f.writelines(last_node + "\n")
    f.writelines(last_node)

    f.close()

    f = open(data.replace('.bvh','.phase'), 'w')

    for i in range(0, footsteps[0]):
        f.writelines("%f\n" % float(0) )
        
    for i in range(0, flen-2, 2):
        d = float(footsteps[i+1]-footsteps[i])
        for j in range(footsteps[i], footsteps[i+1], 1):
            f.writelines("%f\n" % (float(j - footsteps[i]) / d * 0.5) )
            #print("%f\n" % (float(j - footsteps[i]) / d * 0.5), f)
            
        d = float(footsteps[i+2]-footsteps[i+1])
        for j in range(footsteps[i+1], footsteps[i+2], 1):
            f.writelines("%f\n" % (float(j - footsteps[i+1]) / d * 0.5 + 0.5) )
            #print("%f\n" % (float(j - footsteps[i+2]) / d * 0.5), f)
            
        #print("%d %d %d " %(footsteps[i], footsteps[i+1], footsteps[i+2]))
        
    if (flen-3)%2==1:
        d = float(footsteps[flen-1]-footsteps[flen-2])
        for j in range(footsteps[flen-2], footsteps[flen-1], 1):
            f.writelines("%f\n" % (float(j - footsteps[flen-2]) / d * 0.5) )
        
    for i in range(footsteps[flen-1], tlen, 1):
        f.writelines("%f\n" % float(0) )
