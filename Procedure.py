# -*- coding: utf-8 -*-
"""
Created on Tue Mar  7 14:00:11 2023

@author: Alec
"""

import SUAVE
from SUAVE.Core import Units, Data 
import numpy as np
from SUAVE.Analyses.Process import Process
from SUAVE.Optimization.Nexus import Nexus
import Vehicles
import Analyses
import Missions
import Procedure
from SUAVE.Optimization.write_optimization_outputs import write_optimization_outputs
import pickle as pkl

def setup():

    procedure = Process()
    procedure.missions = Process()
    
    
    
    procedure.missions = noiseRun
    procedure.post_process = postProcess
    
    
    return procedure


def noiseRun(nexus):
      
    
    N_gm_x = 2 #number of microphones 
    N_gm_y = 2
    
    #defines the max and min positions of the microphone grid
    max_x = 3.6 *Units.nmi
    min_x = -1.8 *Units.nmi
    max_y = 2 *Units.nmi
    min_y = -3.4*Units.nmi
    configs = nexus.vehicle_configurations
    configs_analyses = Analyses.setup(configs,N_gm_x,N_gm_y,min_y,max_y,min_x,max_x,False) #Running the analysis without the noise simulation
    configs_analyses.finalize()

    #somehow pass the new analyses into mission from nexus
    
    nexus.analyses = configs_analyses
    
    
    nexus.analyses.finalize()
    #finalize the analyses



    mission = nexus.missions.base

    print('Evaluating position mission')
    positionResults = mission.evaluate()
    print('Done')
    final_position_vector = positionResults.base.segments[-1].conditions.frames.inertial.position_vector
    
    x_center = final_position_vector[-1][0]
    y_center = final_position_vector[-1][1]
    max_x += x_center
    min_x += x_center
    max_y +=y_center
    min_y += y_center
    
    
    print('run noise')
    configs_analyses = Analyses.setup(configs,N_gm_x,N_gm_y,min_y,max_y,min_x,max_x,True) 
    noise_mission = nexus.missions.base
    configs.finalize()
    configs_analyses.finalize()
    results = nexus.results
    results.base = noise_mission.evaluate()

    print('done')
    
    return nexus

def postProcess(nexus):
    
    nexus.total_number_of_iterations +=1
    res = nexus.results.base.base
    control_points = len(res.segments[0].conditions.noise.total_SPL_dBA)
    micro = res.segments[0].conditions.noise.number_ground_microphones
    
    
    SPL = []
    for segment in res.segments:
            for i in range(0,control_points):
                SPL.append(segment.conditions.noise.total_SPL_dBA[i])
                
    
    maxDba = np.nanmax(SPL,axis=0)
    
    maxLinear = maxDba
    for i in range(0,len(maxDba)):
        maxLinear[i] = 10**(maxDba[i]/10)
    
    
    avg = maxLinear.sum()/micro
    summary = nexus.summary
    max_throttle = 0
    min_throttle = 1
    for segment in res.segments.values():
        max_segment_throttle = np.max(segment.conditions.propulsion.throttle[:,0])
        min_segment_throttle = np.min(segment.conditions.propulsion.throttle[:,0])
        if max_segment_throttle > max_throttle:
            max_throttle = max_segment_throttle
        if min_throttle > min_segment_throttle:
            min_throttle = min_segment_throttle
            
            
    summary.max_throttle = max_throttle
    summary.min_throttle = min_throttle
    summary.avgdBA = avg
    
    filename = 'results.txt'
    write_optimization_outputs(nexus, filename)
 

    return nexus      

