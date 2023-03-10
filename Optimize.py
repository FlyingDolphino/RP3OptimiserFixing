# -*- coding: utf-8 -*-
"""
Created on Mon Jan 30 11:39:42 2023

@author: Alec
"""

from SUAVE.Core import Units, Data
import numpy as np
import Vehicles
import Analyses
import Missions
import Procedure
#import Plot_Mission
import SUAVE.Optimization.Package_Setups.scipy_setup as scipy_setup
from SUAVE.Optimization.Nexus import Nexus
import pylab as plt


def main():
    
    problem = setup()
    output = problem.objective()
    output = scipy_setup.SciPy_Solve(problem,solver='SLSQP')
    print(output)
    problem.translate(output)

    return

def setup():

    nexus = Nexus()
    problem = Data()
    nexus.optimization_problem = problem

    # -------------------------------------------------------------------
    # Inputs
    # -------------------------------------------------------------------

    #   [ tag                   , initial,     lb , ub        , scaling , units ]
    problem.inputs = np.array([
        [  'speed'     ,50     ,   1.,      100.,    10.,1*Units['m/s']],
        [ 'approach_rate'     ,   4.5    ,   15. ,    100.    ,   100.   , 1*Units['m/s']],

    ],dtype=object)

    # -------------------------------------------------------------------
    # Objective
    # -------------------------------------------------------------------

    # [ tag, scaling, units ]
    problem.objective = np.array([
        [ 'avgdBA', 1000, 1*Units.less ]
    ],dtype=object)
    
    # -------------------------------------------------------------------
    # Constraints
    # -------------------------------------------------------------------
    
    # [ tag, sense, edge, scaling, units ]
    problem.constraints = np.array([
        [ 'Throttle_min' , '>', 0.5, 1, 1*Units.less],
        [ 'Throttle_max' , '<',1, 1, 1*Units.less],
    ],dtype=object)
    
    # -------------------------------------------------------------------
    #  Aliases
    # -------------------------------------------------------------------
    
    # [ 'alias' , ['data.path1.name','data.path2.name'] ]

    problem.aliases = [
        [ 'speed'                        ,  'missions.base.segments.initial_approach.air_speed','missions.base.segments.turn_to_final.air_speed','missions.base.segments.final_approach.air_speed','missions.base.segments.Descent_Transition.air_speed_start'],
        [ 'approach_rate'                  , 'missions.base.segments.final_approach.descent_rate'    ],
        [ 'height_of_Vertical'                        ,    'missions.base.segments.descent_transition.altitude','missions.base.segments.vertical_landing.altitude_start','missions.base.segments.final_approach.altitude_end '],
        [ 'angle_to_base'         ,    'missions.base.segments.initial_approach.true_course'    ],
        ['angle_to_final'           ,     'missions.base.segments.turn_to_final.true_course'],
        [ 'Throttle_min'            ,   'summary.min_throttle'  ],
        ['Throttle_max'                , 'summary.max_throttle'],
        ['avgdBA'               ,       'summary.avgdBA']
    ]    
    
    
    # -------------------------------------------------------------------
    #  Vehicles
    # -------------------------------------------------------------------
    nexus.vehicle_configurations = Vehicles.setup()
    
    # -------------------------------------------------------------------
    #  Analyses
    # -------------------------------------------------------------------
    
    N_gm_x           = 10
    N_gm_y           = 5
    
    min_y = 1E-3 
    max_y = 0.25*Units.nmi
    min_x = -0.25*Units.nmi
    max_x = 0.25*Units.nmi
    
    
    nexus.analyses = Analyses.setup(nexus.vehicle_configurations,N_gm_x,N_gm_y,min_y,max_y,min_x,max_x,False)
    
    # -------------------------------------------------------------------
    #  Missions
    # -------------------------------------------------------------------
    nexus.missions = Missions.setup(nexus.analyses,nexus.vehicle_configurations)
    
    # -------------------------------------------------------------------
    #  Procedure
    # -------------------------------------------------------------------    
    nexus.procedure = Procedure.setup()
    
    # -------------------------------------------------------------------
    #  Summary
    # -------------------------------------------------------------------    
    nexus.summary = Data()    
    nexus.total_number_of_iterations = 0
    return nexus


if __name__ == '__main__':
    main()
