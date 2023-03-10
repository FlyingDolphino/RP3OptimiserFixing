# -*- coding: utf-8 -*-
"""
Created on Wed Feb  1 10:37:51 2023

@author: Alec
"""

import SUAVE
from SUAVE.Core import Units, Data 
import numpy as nps

# ----------------------------------------------------------------------
#   Define the Mission
# ----------------------------------------------------------------------
    
def setup(analyses,vehicle):
    
    # the mission container
    missions = SUAVE.Analyses.Mission.Mission.Container()   
    base_mission = base(analyses,vehicle)
    missions.base = base_mission
    return missions  

def base(analyses,vehicle):
    # ------------------------------------------------------------------
    #   Initialize the Mission
    # ------------------------------------------------------------------

    mission = SUAVE.Analyses.Mission.Sequential_Segments()
    mission.tag = 'the_mission'


    starting_elevation  = 0 * Units.ft
    mission = SUAVE.Analyses.Mission.Sequential_Segments()
    mission.tag = 'mission'

    # airport
    airport            = SUAVE.Attributes.Airports.Airport() 
    airport.delta_isa  =  0.0
    airport.atmosphere = SUAVE.Attributes.Atmospheres.Earth.US_Standard_1976() 
    mission.airport    = airport     

    mission.atmosphere = SUAVE.Analyses.Atmospheric.US_Standard_1976()
    atmo = mission.atmosphere.compute_values(altitude = 0, temperature_deviation=1)
    mission.planet = SUAVE.Attributes.Planets.Earth()

    # unpack Segments module
    Segments = SUAVE.Analyses.Mission.Segments
    
    
    # ------------------------------------------------------------------
    #   base segment
    # ------------------------------------------------------------------

    # base segment
    control_points = 2
    base_segment = Segments.Segment()
    base_segment.battery_discharge                           = True 
    ones_row  = base_segment.state.ones_row    
    base_segment.state.numerics.number_control_points = control_points
    

    base_segment.process.initialize.initialize_battery       = SUAVE.Methods.Missions.Segments.Common.Energy.initialize_battery
    base_segment.process.finalize.post_process.update_battery_state_of_health = SUAVE.Methods.Missions.Segments.Common.Energy.update_battery_state_of_health  
    base_segment.state.numerics.number_control_points        = control_points
    bat                                                      = vehicle.base.networks.battery_propeller.battery
    base_segment.charging_SOC_cutoff                         = bat.cell.charging_SOC_cutoff
    base_segment.charging_current                            = bat.charging_current
    base_segment.charging_voltage                            = bat.charging_voltage
    base_segment.battery_discharge                           = True  

    # ------------------------------------------------------------------    
    #   initial approach Segment: Constant Speed, Constant Altitude
    # ------------------------------------------------------------------    

    segment = Segments.Cruise.Constant_Speed_Constant_Altitude(base_segment)
    segment.tag = "initial_approach"
    segment.analyses.extend(analyses.cruise)
    
    
    segment.battery_energy = vehicle.base.networks.battery_propeller.battery.max_energy
 
    segment.altitude = 2000   * Units.feet
    segment.air_speed  = 10 * Units['m/s']
    segment.distance   = 2. * Units.nautical_miles
    segment.true_course = 180 *Units.degrees
    segment.state.unknowns.throttle          = 0.6  * ones_row(1) 
  

    # add to mission
    segment = vehicle.base.networks.battery_propeller.add_unknowns_and_residuals_to_segment(segment)  
    mission.append_segment(segment)
    
    
    
    
    # ------------------------------------------------------------------    
    #   turning approach Segment: Constant Speed, Constant Altitude
    # ------------------------------------------------------------------    

    
    segment = Segments.Cruise.Constant_Speed_Constant_Altitude(base_segment)
    segment.tag = "turn_to_final"
    segment.analyses.extend(analyses.cruise)
    
    
    segment.battery_energy = vehicle.base.networks.battery_propeller.battery.max_energy
 
    segment.altitude = 2000   * Units.feet
    segment.air_speed  = 10 * Units['m/s']
    segment.true_course = 90*Units.degrees
    segment.distance   = 1. * Units.nautical_miles
    segment.state.unknowns.throttle          = 0.6  * ones_row(1) 
  

    # add to mission
    segment = vehicle.base.networks.battery_propeller.add_unknowns_and_residuals_to_segment(segment)  
    mission.append_segment(segment)
    


    # ------------------------------------------------------------------
    #   Final approach Segment: Constant Speed, Constant Rate
    # ------------------------------------------------------------------

    segment = Segments.Descent.Constant_Speed_Constant_Rate(base_segment)
    segment.tag = "final_approach"
    segment.analyses.extend( analyses.cruise)
    
     
    segment.altitude_start = 2000   *Units.feet
    segment.air_speed  = 10 * Units['m/s']
    segment.descent_rate = 4.5   * Units['m/s']
    segment.true_course = 0*Units.degrees
    segment.altitude_end = 20.0   * Units.ft
    segment.state.unknowns.throttle          = 0.7  * ones_row(1)  
    segment.battery_pack_temperature                   = atmo.temperature[0,0]



    # add to mission
    segment = vehicle.base.networks.battery_propeller.add_unknowns_and_residuals_to_segment(segment)  
    mission.append_segment(segment)
    
    
    
   # ------------------------------------------------------------------
   #   Transition Segment
   # ------------------------------------------------------------------

    segment                           = Segments.Cruise.Constant_Acceleration_Constant_Altitude(base_segment)
    segment.tag                       = "descent_transition"  
    segment.analyses.extend( analyses.vertical_transition) 
    segment.altitude                  = 20   * Units.ft
    segment.air_speed_start           = 10.  * Units['m/s'] 
    segment.air_speed_end             = 1. * Units['m/s']  
    segment.true_course = 0*Units.degrees
    segment.state.unknowns.throttle   = 0.8  * ones_row(1)  
    segment = vehicle.base.networks.battery_propeller.add_unknowns_and_residuals_to_segment(segment) 
    mission.append_segment(segment)

    

    
    # ------------------------------------------------------------------
    #   vertical approach Segment: hover descent
    # ------------------------------------------------------------------

    segment = Segments.Hover.Descent(base_segment)
    segment.tag = "vertical_landing"
    segment.analyses.extend( analyses.hover)
    
    
    ones_row = segment.state.ones_row
    segment.altitude_start = 20 *Units.ft    
    segment.altitude_end = 0.0   * Units.ft
    segment.air_speed    = 1 * Units['m/s']
    segment.true_course = 0*Units.degrees
    segment.descent_rate = 4.5   * Units['m/s']
    segment.state.unknowns.throttle          = 0.9  * ones_row(1)  


    # add to mission
    segment = vehicle.base.networks.battery_propeller.add_unknowns_and_residuals_to_segment(segment)  
    mission.append_segment(segment)
    
    return mission