# -*- coding: utf-8 -*-
"""
Created on Wed Feb  1 11:37:52 2023

@author: Alec
"""

import SUAVE
from SUAVE.Core import Units, Data  
import pickle
import time 
import os
from copy import deepcopy
from SUAVE.Plots.Performance.Mission_Plots import *  
from SUAVE.Plots.Geometry  import *  
from SUAVE.Analyses.Propulsion.Rotor_Wake_Fidelity_Zero import Rotor_Wake_Fidelity_Zero
from SUAVE.Analyses.Propulsion.Rotor_Wake_Fidelity_One import Rotor_Wake_Fidelity_One
from SUAVE.Methods.Center_of_Gravity.compute_component_centers_of_gravity import compute_component_centers_of_gravity
from SUAVE.Methods.Power.Battery.Sizing                        import initialize_from_circuit_configuration 
from SUAVE.Methods.Propulsion                                  import propeller_design 
from SUAVE.Methods.Weights.Buildups.eVTOL.empty                import empty 
from SUAVE.Methods.Propulsion.electric_motor_sizing            import size_optimal_motor
from SUAVE.Methods.Weights.Correlations.Propulsion             import nasa_motor 
from SUAVE.Methods.Weights.Buildups.Common.fuselage            import fuselage
import numpy as np








def setup():
    
    base_vehicle = base_setup()
    configs = configs_setup(base_vehicle)
    
    return configs

def base_setup():
   # ------------------------------------------------------------------
    #   Initialize the Vehicle
    # ------------------------------------------------------------------    
    vehicle                                     = SUAVE.Vehicle()
    vehicle.tag                                 = 'Multirotor_CRM'
    vehicle.configuration                       = 'eVTOL'
    
    # ------------------------------------------------------------------
    #   Vehicle-level Properties
    # ------------------------------------------------------------------    
    # mass properties
    vehicle.mass_properties.takeoff             = 3200 #3800   
    vehicle.mass_properties.operating_empty     = 3200 ##3800  
    vehicle.mass_properties.max_takeoff         = 3200 ##3800  
    vehicle.mass_properties.center_of_gravity   = [[2.6, 0., 0. ] ] 
                                                
    # This needs updating                       
    vehicle.passengers                          = 6
    vehicle.reference_area                      = 73  * Units.feet**2 
    vehicle.envelope.ultimate_load              = 5.7   
    vehicle.envelope.limit_load                 = 3.  
                                                
    wing                                        = SUAVE.Components.Wings.Main_Wing()  # this is the body of the vehicle 
    wing.tag                                    = 'main_wing'   
    wing.aspect_ratio                           = 0.5 
    wing.sweeps.quarter_chord                   = 0.  
    wing.thickness_to_chord                     = 0.01   
    wing.spans.projected                        = 0.01  
    wing.chords.root                            = 0.01
    wing.total_length                           = 0.01
    wing.chords.tip                             = 0.01
    wing.chords.mean_aerodynamic                = 0.01
    wing.dihedral                               = 0.0  
    wing.areas.reference                        = 0.0001 
    wing.areas.wetted                           = 0.01
    wing.areas.exposed                          = 0.01  
    wing.symbolic                               = True 
    wing.symmetric                              = True 
    
    vehicle.append_component(wing)
    
    # ------------------------------------------------------    
    # FUSELAGE    
    # ------------------------------------------------------    
    # FUSELAGE PROPERTIES
    fuselage                                    = SUAVE.Components.Fuselages.Fuselage()
    fuselage.tag                                = 'fuselage' 
    fuselage.seats_abreast                      = 2.  
    fuselage.seat_pitch                         = 3.  
    fuselage.fineness.nose                      = 0.88   
    fuselage.fineness.tail                      = 1.13   
    fuselage.lengths.nose                       = 0.5 
    fuselage.lengths.tail                       = 0.5
    fuselage.lengths.cabin                      = 4.
    fuselage.lengths.total                      = 5.
    fuselage.width                              = 1.8
    fuselage.heights.maximum                    = 1.8
    fuselage.heights.at_quarter_length          = 1.8
    fuselage.heights.at_wing_root_quarter_chord = 1.8
    fuselage.heights.at_three_quarters_length   = 1.8
    fuselage.areas.wetted                       = 19.829265
    fuselage.areas.front_projected              = 1.4294246 
    fuselage.effective_diameter                 = 1.4
    fuselage.differential_pressure              = 1. 
    
    # Segment  
    segment                          = SUAVE.Components.Lofted_Body_Segment.Segment() 
    segment.tag                      = 'segment_0'   
    segment.percent_x_location       = 0.  
    segment.percent_z_location       = 0.0 
    segment.height                   = 0.1   
    segment.width                    = 0.1   
    fuselage.append_segment(segment)            
                                                
    # Segment                                   
    segment                         = SUAVE.Components.Lofted_Body_Segment.Segment()
    segment.tag                     = 'segment_1'   
    segment.percent_x_location      = 0.200/4.
    segment.percent_z_location      = 0.1713/4.
    segment.height                  = 0.737
    segment.width                   = 1.2
    segment.vsp_data.top_angle      = 53.79 * Units.degrees 
    segment.vsp_data.bottom_angle   = 28.28 * Units.degrees     
    fuselage.append_segment(segment)            
                                                
    # Segment                                   
    segment                         = SUAVE.Components.Lofted_Body_Segment.Segment()
    segment.tag                     = 'segment_2'   
    segment.percent_x_location      = 0.8251/4.
    segment.percent_z_location      = 0.2840/4.
    segment.height                  = 1.40 
    segment.width                   = 1.8
    segment.vsp_data.top_angle      = 0 * Units.degrees 
    segment.vsp_data.bottom_angle   = 0 * Units.degrees     
    fuselage.append_segment(segment)            
                                                
    # Segment                                  
    segment                         = SUAVE.Components.Lofted_Body_Segment.Segment()
    segment.tag                     = 'segment_3'   
    segment.percent_x_location      = 3.342/4.
    segment.percent_z_location      = 0.356/4.
    segment.height                  = 1.40
    segment.width                   = 1.8
    #segment.vsp_data.top_angle      = 0 * Units.degrees 
    #segment.vsp_data.bottom_angle   = 0 * Units.degrees     
    fuselage.append_segment(segment)  
                                                
    # Segment                                   
    segment                         = SUAVE.Components.Lofted_Body_Segment.Segment()
    segment.tag                     = 'segment_4'   
    segment.percent_x_location      = 3.70004/4.
    segment.percent_z_location      = 0.4636/4.
    segment.height                  = 0.9444
    segment.width                   = 1.2
    segment.vsp_data.top_angle      = -36.59 * Units.degrees 
    segment.vsp_data.bottom_angle   = -57.94 * Units.degrees 
    fuselage.append_segment(segment)             
    
    # Segment                                   
    segment                         = SUAVE.Components.Lofted_Body_Segment.Segment()
    segment.tag                     = 'segment_5'   
    segment.percent_x_location      = 1.
    segment.percent_z_location      = 0.6320/4.
    segment.height                  = 0.1    
    segment.width                   = 0.1    
    fuselage.append_segment(segment)             
    
                                                 
    # add to vehicle
    vehicle.append_component(fuselage)   
    
    
     
    #------------------------------------------------------------------
    # Network
    #------------------------------------------------------------------
    net                      = SUAVE.Components.Energy.Networks.Battery_Propeller()
    net.number_of_propeller_engines    = 6
    net.nacelle_diameter     = 0.6 * Units.feet # need to check 
    net.engine_length        = 0.5 * Units.feet
    net.areas                = Data()
    net.areas.wetted         = np.pi*net.nacelle_diameter*net.engine_length + 0.5*np.pi*net.nacelle_diameter**2 
    net.identical_propellers= True

        #------------------------------------------------------------------
    # Design Electronic Speed Controller 
    #------------------------------------------------------------------
    esc             = SUAVE.Components.Energy.Distributors.Electronic_Speed_Controller()
    esc.efficiency  = 0.95
    net.esc         = esc
    
    #------------------------------------------------------------------
    # Design Payload
    #------------------------------------------------------------------
    payload                       = SUAVE.Components.Energy.Peripherals.Avionics()
    payload.power_draw            = 0. 
    net.payload                   = payload

    #------------------------------------------------------------------
    # Design Avionics
    #------------------------------------------------------------------
    avionics            = SUAVE.Components.Energy.Peripherals.Avionics()
    avionics.power_draw = 200. * Units.watts
    net.avionics        = avionics
    
    #------------------------------------------------------------------
    # Design Battery
    #------------------------------------------------------------------  
    total_cells                           = 150*270
    max_module_voltage                    = 50
    safety_factor                         = 1.5
     
    bat                                   = SUAVE.Components.Energy.Storages.Batteries.Constant_Mass.Lithium_Ion_LiNiMnCoO2_18650() 
    bat.pack_config.series                = 150 
    bat.pack_config.parallel              = int(total_cells/bat.pack_config.series)
    initialize_from_circuit_configuration(bat,module_weight_factor = 1.05)    
    net.voltage                           = bat.max_voltage  
    bat.module_config.number_of_modules   = 20  
    bat.module_config.total               = int(np.ceil(bat.pack_config.total/bat.module_config.number_of_modules))
    bat.module_config.voltage             = net.voltage/bat.module_config.number_of_modules # must be less than max_module_voltage/safety_factor 
    bat.module_config.layout_ratio        = 0.5
    bat.module_config.normal_count        = int(bat.module_config.total**(bat.module_config.layout_ratio))
    bat.module_config.parallel_count      = int(bat.module_config.total**(1-bat.module_config.layout_ratio)) 
    net.battery                           = bat        
     
    #------------------------------------------------------------------
    # Design Rotors  
    #------------------------------------------------------------------ 
    # atmosphere and flight conditions for propeller/lift_rotor design
    g                            = 9.81                                   # gravitational acceleration  
    speed_of_sound               = 340                                    # speed of sound 
    Hover_Load                   = vehicle.mass_properties.takeoff*g      # hover load   
    design_tip_mach              = 0.7                                    # design tip mach number 
    
    rotor                        = SUAVE.Components.Energy.Converters.Lift_Rotor() 
    rotor.tip_radius             = 2.5
    rotor.hub_radius             = 0.1*rotor.tip_radius  
    rotor.disc_area              = np.pi*(rotor.tip_radius**2) 
    rotor.number_of_blades       = 3
    rotor.freestream_velocity    = 10.0
    rotor.angular_velocity       = (design_tip_mach*speed_of_sound)/rotor.tip_radius   
    rotor.design_Cl              = 0.7
    rotor.design_altitude        = 3000 * Units.feet                   
    rotor.design_thrust          = Hover_Load/(net.number_of_propeller_engines) # contingency for one-engine-inoperative condition and then turning off off-diagonal rotor
    ospath    = os.path.abspath(__file__)
    separator = os.path.sep
    rel_path  = os.path.dirname(ospath) + separator 
    rotor.airfoil_geometry         =  [ rel_path + '/Airfoils/NACA_4412.txt']
    rotor.airfoil_polars           = [[ rel_path + '/Airfoils/Polars/NACA_4412_polar_Re_50000.txt' ,
                                        rel_path + '/Airfoils/Polars/NACA_4412_polar_Re_100000.txt' ,
                                        rel_path + '/Airfoils/Polars/NACA_4412_polar_Re_200000.txt' ,
                                        rel_path + '/Airfoils/Polars/NACA_4412_polar_Re_500000.txt' ,
                                        rel_path + '/Airfoils/Polars/NACA_4412_polar_Re_1000000.txt' ]]  
    rotor.airfoil_polar_stations = [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]      
    rotor.variable_pitch         = True                                                    
    
    
    rotor                        = propeller_design(rotor)     
    
    # Appending rotors with different origins
    origins                 = [[ -1.5,2.6,1.7],[ -1.5,-2.6,1.7],
                                [2.5,6.0,1.7] ,[2.5,-6.,1.7],
                                [6.5,2.6,1.7] ,[6.5,-2.6,1.7]]   
    
    for ii in range(6):
        lift_rotor          = deepcopy(rotor)
        lift_rotor.tag      = 'mr_lift_rotor_' + str(ii+1)
        lift_rotor.origin   = [origins[ii]]
        net.propellers.append(lift_rotor)
        
    # ------------------------------------------------------------------
    #   Nacelles
    # ------------------------------------------------------------------ 
    nacelle                = SUAVE.Components.Nacelles.Nacelle()
    nacelle.tag            = 'lift_rotor_nacelle'
    nacelle.length         = 0.5
    nacelle.diameter       = 0.8
    nacelle.inlet_diameter = 0.6
    nacelle.orientation_euler_angles  = [0,-90*Units.degrees,0.]    
    nacelle.flow_through   = True 
 
    lift_rotor_nacelle_origins   =  [[ -1.5,2.6,1.9],[ -1.5,-2.6,1.9],
                                [2.5,6.0,1.9] ,[2.5,-6.,1.9],
                                [6.5,2.6,1.9] ,[6.5,-2.6,1.9]]   
 
    for ii in range(6):
        rotor_nacelle          = deepcopy(nacelle)
        rotor_nacelle.tag      = 'engine_nacelle_' + str(ii+1) 
        rotor_nacelle.origin   = [lift_rotor_nacelle_origins[ii]]
        vehicle.append_component(rotor_nacelle)   
       
        
    # ------------------------------------------------------------------
    #   Nacelles
    # ------------------------------------------------------------------ 
    nacelle                = SUAVE.Components.Nacelles.Nacelle()
    nacelle.tag            = 'lift_rotor_nacelle'
    nacelle.length         = 0.5
    nacelle.diameter       = 0.8
    nacelle.inlet_diameter = 0.6
    nacelle.orientation_euler_angles  = [0,-90*Units.degrees,0.]    
    nacelle.flow_through   = True 
    
    lift_rotor_nacelle_origins   =  [[ -1.5,2.6,1.9],[ -1.5,-2.6,1.9],
                                [2.5,6.0,1.9] ,[2.5,-6.,1.9],
                                [6.5,2.6,1.9] ,[6.5,-2.6,1.9]]   
    
    for ii in range(6):
        rotor_nacelle          = deepcopy(nacelle)
        rotor_nacelle.tag      = 'engine_nacelle_' + str(ii+1) 
        rotor_nacelle.origin   = [lift_rotor_nacelle_origins[ii]]
        vehicle.append_component(rotor_nacelle)   
        
    #------------------------------------------------------------------
    # Design Motors
    #------------------------------------------------------------------
    # Motor 
    lift_motor                      = SUAVE.Components.Energy.Converters.Motor() 
    lift_motor.efficiency           = 0.95
    lift_motor.nominal_voltage      = bat.max_voltage 
    lift_motor.mass_properties.mass = 3. * Units.kg  
    lift_motor.propeller_radius     = rotor.tip_radius   
    lift_motor.no_load_current      = 2.0     
    lift_motor                      = size_optimal_motor(lift_motor,rotor)
    lift_motor.mass_properties.mass   = nasa_motor(lift_motor.design_torque)   - 20 # NEED BETTER MOTOR MODEL
    net.lift_motor                    = lift_motor   
     
    # Appending motors with different origins    
    for ii in range(6):
        lift_rotor_motor = deepcopy(lift_motor)
        lift_rotor_motor.tag = 'motor_' + str(ii+1)
        lift_motor.origin   = [origins[ii]]
        net.propeller_motors.append(lift_rotor_motor)       
    
    vehicle.append_component(net)
    
    vehicle.wings['main_wing'].motor_spanwise_locations   = np.array(origins)[:,1]/ vehicle.wings['main_wing'].spans.projected
    
    
    return vehicle

def configs_setup(vehicle):
    configs = SUAVE.Components.Configs.Config.Container()

    base_config = SUAVE.Components.Configs.Config(vehicle)
    base_config.tag = 'base'
    configs.append(base_config)
    
    config = SUAVE.Components.Configs.Config(base_config)
    config.tag = 'hover' 
    for rotor in config.networks.battery_propeller.propellers:  
        rotor.inputs.pitch_command                     = 0.  * Units.degrees
    configs.append(config)
    
    config = SUAVE.Components.Configs.Config(base_config)
    config.tag = 'vertical_transition'   
    for rotor in config.networks.battery_propeller.propellers:  
        rotor.inputs.pitch_command                     = 3.  * Units.degrees
    configs.append(config)
    
    config = SUAVE.Components.Configs.Config(base_config)
    config.tag = 'cruise'   
    for rotor in config.networks.battery_propeller.propellers:  
        rotor.inputs.pitch_command                     = 5.  * Units.degrees
    configs.append(config)     
    
    return configs


