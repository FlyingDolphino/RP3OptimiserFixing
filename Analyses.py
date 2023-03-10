# -*- coding: utf-8 -*-
"""
Created on Wed Feb  8 11:36:21 2023

@author: Alec
"""


import SUAVE
from SUAVE.Core import Units

import numpy as np






def setup(configs,N_gm_x,N_gm_y,min_y,max_y,min_x,max_x,bol):
      

    analyses = SUAVE.Analyses.Analysis.Container()
    
    if bol == True:
        for tag,config in configs.items():
            analysis = base(config,N_gm_x,N_gm_y,min_y,max_y,min_x,max_x)
        
            analyses[tag] = analysis
    else:
        for tag,config in configs.items():
            analysis = position(config)
        
            analyses[tag] = analysis
    
    
    return analyses




def base(vehicle,N_gm_x,N_gm_y,min_y,max_y,min_x,max_x):
    
    analyses = SUAVE.Analyses.Vehicle()
    
    
    sizing = SUAVE.Analyses.Sizing.Sizing()
    sizing.features.vehicle = vehicle
    analyses.append(sizing)
    
    weights = SUAVE.Analyses.Weights.Weights_eVTOL()
    weights.vehicle = vehicle
    analyses.append(weights)
    
    aerodynamics = SUAVE.Analyses.Aerodynamics.Fidelity_Zero()
    aerodynamics.geometry = vehicle 
    aerodynamics.settings.model_fuselage = True 
    analyses.append(aerodynamics) 
    
    planet = SUAVE.Analyses.Planets.Planet()
    analyses.append(planet)
    
    atmosphere = SUAVE.Analyses.Atmospheric.US_Standard_1976()
    atmosphere.features.planet = planet.features
    analyses.append(atmosphere)   

    energy= SUAVE.Analyses.Energy.Energy()
    energy.network = vehicle.networks 
    analyses.append(energy)

    
    noise = SUAVE.Analyses.Noise.Fidelity_One()   
    noise.geometry = vehicle
    noise.settings.level_ground_microphone_x_resolution = N_gm_x
    noise.settings.level_ground_microphone_y_resolution = N_gm_y
    noise.settings.level_ground_microphone_min_y        = min_y
    noise.settings.level_ground_microphone_max_y        = max_y
    noise.settings.level_ground_microphone_min_x        = min_x
    noise.settings.level_ground_microphone_max_x        = max_x
    analyses.append(noise)
    
    return analyses
    
    
def position(vehicle):
    analyses = SUAVE.Analyses.Vehicle()
    
    
    sizing = SUAVE.Analyses.Sizing.Sizing()
    sizing.features.vehicle = vehicle
    analyses.append(sizing)
    
    weights = SUAVE.Analyses.Weights.Weights_eVTOL()
    weights.vehicle = vehicle
    analyses.append(weights)
    
    aerodynamics = SUAVE.Analyses.Aerodynamics.Fidelity_Zero()
    aerodynamics.geometry = vehicle 
    aerodynamics.settings.model_fuselage = True 
    analyses.append(aerodynamics) 
    
    planet = SUAVE.Analyses.Planets.Planet()
    analyses.append(planet)
    
    atmosphere = SUAVE.Analyses.Atmospheric.US_Standard_1976()
    atmosphere.features.planet = planet.features
    analyses.append(atmosphere)   

    energy= SUAVE.Analyses.Energy.Energy()
    energy.network = vehicle.networks 
    analyses.append(energy)

    return analyses