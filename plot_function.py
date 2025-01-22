
def define_colors():
    import matplotlib.pyplot as plt
    import matplotlib.colors as mcolors
    ROADVIEW_colors = {"ROADVIEW1": "#64798C",
                       "ROADVIEW2": "#B0C6D9",
                       "ROADVIEW3": "#F2C879",
                       "ROADVIEW4": "#F28066"}

    mcolors.get_named_colors_mapping().update(ROADVIEW_colors)
    return mcolors.LinearSegmentedColormap.from_list("", ["ROADVIEW1", "ROADVIEW2", "ROADVIEW3"])


def plot_function(output_states=False, manoeuvre=True, sync_adma=True, sync_zgw=True, savefig=False, plot_type = "all", xlim=(), fig_save_dir="../../figures",static_parameters=0, initial_iteraction=0):
    import matplotlib.pyplot as plt
    import numpy as np
    size = 17
    figsize = (size, size*.588)
    simulation_range = range(0, len(manoeuvre))
    throttle = manoeuvre[:].throttle
    plt.rcParams.update({'font.size': 16})

    define_colors()

    vehicle_time = np.array([i.timestamp for i in sync_zgw]) - sync_zgw[initial_iteraction].timestamp
    if output_states:
        states = output_states[:].x_a
        if "body" in plot_type or plot_type == "all":
            Fx_fl = (np.array([i.acc_hor.x for i in sync_adma]) * static_parameters.body.mass - np.array([i.ins_vel_hor.y for i in sync_adma]) * np.array([i.rate_hor.z for i in sync_adma]))  * static_parameters.powertrain.bias[0]    
            Fx_fr = (np.array([i.acc_hor.x for i in sync_adma]) * static_parameters.body.mass - np.array([i.ins_vel_hor.y for i in sync_adma]) * np.array([i.rate_hor.z for i in sync_adma]))  * static_parameters.powertrain.bias[1]    
            Fx_rl = (np.array([i.acc_hor.x for i in sync_adma]) * static_parameters.body.mass - np.array([i.ins_vel_hor.y for i in sync_adma]) * np.array([i.rate_hor.z for i in sync_adma]))  * static_parameters.powertrain.bias[2]    
            Fx_rr = (np.array([i.acc_hor.x for i in sync_adma]) * static_parameters.body.mass - np.array([i.ins_vel_hor.y for i in sync_adma]) * np.array([i.rate_hor.z for i in sync_adma]))  * static_parameters.powertrain.bias[3]    

            Fy_fl = (np.array([i.acc_hor.y for i in sync_adma]) * static_parameters.body.mass + np.array([i.ins_vel_hor.x for i in sync_adma]) * np.array([i.rate_hor.z for i in sync_adma]))  * static_parameters.powertrain.bias[0]    
            Fy_fr = (np.array([i.acc_hor.y for i in sync_adma]) * static_parameters.body.mass + np.array([i.ins_vel_hor.x for i in sync_adma]) * np.array([i.rate_hor.z for i in sync_adma]))  * static_parameters.powertrain.bias[1]    
            Fy_rl = (np.array([i.acc_hor.y for i in sync_adma]) * static_parameters.body.mass + np.array([i.ins_vel_hor.x for i in sync_adma]) * np.array([i.rate_hor.z for i in sync_adma]))  * static_parameters.powertrain.bias[2]    
            Fy_rr = (np.array([i.acc_hor.y for i in sync_adma]) * static_parameters.body.mass + np.array([i.ins_vel_hor.x for i in sync_adma]) * np.array([i.rate_hor.z for i in sync_adma]))  * static_parameters.powertrain.bias[3]    
            plt.figure(figsize=figsize)
            name = " Chassis Angles"
            plt.title(name)
            var_name = "Vhcl_Roll"
            plt.plot(vehicle_time,[(i.ins_roll)*3.14/180 for i in sync_adma[:]], "ROADVIEW1", label="M8 Roll")
            plt.plot(vehicle_time,[(i.ins_pitch)*3.14/180 for i in sync_adma[:]], "ROADVIEW2", label="M8 Pitch")
            plt.plot(vehicle_time,[(i.ins_yaw-sync_adma[initial_iteraction].ins_yaw)*3.14/180 for i in sync_adma[:]], "ROADVIEW3", label="M8 Yaw")
            plt.plot(manoeuvre.time, [i.roll for i in output_states[:].x_a], "ROADVIEW1", linestyle="--", label="Calculated Roll")
            plt.plot(manoeuvre.time, [i.pitch for i in output_states[:].x_a], "ROADVIEW2", linestyle="--", label="Calculated Pitch")
            plt.plot(manoeuvre.time, [i.yaw -output_states.x_a[0].yaw for i in output_states[:].x_a], "ROADVIEW3", linestyle="--", label="Calculated Yaw")    
            plt.grid()
            plt.legend()
            plt.xlim(xlim)
            plt.xlabel("Time (s)")
            plt.ylabel("Angle (rad)")
            plt.tight_layout()
            if savefig:
                plt.savefig(fig_save_dir + "/" + name + ".png")  
            plt.figure(figsize=figsize)
            name = " Chassis Angles Velocity"
            plt.title(name)
            plt.plot(manoeuvre.time, [i.wx for i in output_states[:].x_a], "ROADVIEW1", linestyle="--", label="Calculated Roll Velocity")
            plt.plot(manoeuvre.time, [i.wy for i in output_states[:].x_a], "ROADVIEW2", linestyle="--", label="Calculated Pitch Velocity")
            plt.plot(manoeuvre.time, [i.wz for i in output_states[:].x_a], "ROADVIEW3", linestyle="--", label="Calculated Yaw Velocity")

            plt.plot(vehicle_time,[i.rate_hor.x*3.14/180 for i in sync_adma[:]], "ROADVIEW1", label="M8 Roll Velocity")
            plt.plot(vehicle_time,[i.rate_hor.y*3.14/180 for i in sync_adma[:]], "ROADVIEW2", label="M8 Pitch Velocity")
            plt.plot(vehicle_time,[i.rate_hor.z*3.14/180 for i in sync_adma[:]], "ROADVIEW3", label="M8 Yaw Velocity")
            plt.xlabel("Time (s)")
            plt.ylabel("Angle Velocity (rad/s)")
            plt.legend()
            plt.xlim(xlim)
            plt.grid()
            plt.tight_layout()
            if savefig:
                plt.savefig(fig_save_dir + "/" + name + ".png")   

            plt.figure(figsize=figsize)
            name = " Chassis Acceleration"
            plt.title(name)
            plt.plot(vehicle_time, [(i.acc_hor.x)*9.81 for i in sync_adma[:]], "ROADVIEW1", label="M8 Acceleration X")
            plt.plot(vehicle_time, [(i.acc_hor.y)*9.81 for i in sync_adma[:]], "ROADVIEW2", label="M8 Acceleration Y")
            plt.plot(vehicle_time, [(i.acc_hor.z -1)*9.81 for i in sync_adma[:]], "ROADVIEW3", label="M8 Acceleration Z")
            plt.plot(manoeuvre.time, [i.acc_x for i in output_states[:].x_a], "ROADVIEW1", linestyle="--", label="Calculated Acceleration X")
            plt.plot(manoeuvre.time, [i.acc_y for i in output_states[:].x_a], "ROADVIEW2", linestyle="--", label="Calculated Acceleration Y")
            plt.plot(manoeuvre.time, [i.acc_z for i in output_states[:].x_a], "ROADVIEW3", linestyle="--", label="Calculated Acceleration Z")
            plt.grid()
            plt.xlabel("Time (s)")
            plt.ylabel("Acceleration (m/s²)")
            plt.legend()
            plt.xlim(xlim)
            plt.tight_layout()
            if savefig:
                plt.savefig(fig_save_dir + "/" + name + ".png")     
            plt.figure(figsize=figsize)
            name = " Chassis Velocity"
            plt.title(name)
            plt.plot(vehicle_time, [i.ins_vel_hor.x for i in sync_adma[:]], "ROADVIEW1", label="M8 Velocity X")
            plt.plot(vehicle_time, [i.ins_vel_hor.y for i in sync_adma[:]], "ROADVIEW2", label="M8 Velocity Y")
            plt.plot(vehicle_time, [i.ins_vel_hor.z for i in sync_adma[:]], "ROADVIEW3", label="M8 Velocity Z")

            plt.plot(manoeuvre.time, [i.vx for i in output_states[:].x_a], "ROADVIEW1", linestyle="--", label="Calculated Velocity X")
            plt.plot(manoeuvre.time, [i.vy for i in output_states[:].x_a], "ROADVIEW2", linestyle="--", label="Calculated Velocity Y")
            plt.plot(manoeuvre.time, [i.vz for i in output_states[:].x_a], "ROADVIEW3", linestyle="--", label="Calculated Velocity Z")

            plt.grid()
            plt.xlabel("Time (s)")
            plt.ylabel("Velocity (m/s)")
            plt.legend()
            plt.xlim(xlim)
            plt.tight_layout()
            if savefig:
                plt.savefig(fig_save_dir + "/" + name + ".png")   

            plt.figure(figsize=figsize)
            name = " Chassis Position"
            plt.title(name)
            plt.plot([i.ins_pos_rel_y for i in sync_adma[:]], [i.ins_pos_rel_x for i in sync_adma[:]], "ROADVIEW1", label="M8 Position")

            plt.plot([i.x for i in output_states[:].x_a], [i.y for i in output_states[:].x_a], "ROADVIEW4", linestyle="--", label="Calculated Position")

            plt.grid()
            plt.xlabel("Position X (m)")
            plt.ylabel("Position Y (m)")
            plt.legend()
            plt.tight_layout()
            if savefig:
                plt.savefig(fig_save_dir + "/" + name + ".png")   

        if "powertrain" in plot_type or plot_type == "all":
            gear_changes_idx =np.where(np.roll(output_states[:].gear, 1) != output_states[:].gear)[0]
            gear_changes = manoeuvre.time[gear_changes_idx]

            f, (a0, a1) = plt.subplots(2, 1, gridspec_kw={'height_ratios': [2, 2]}, figsize=figsize, sharex=True)
            powertrain_net_torque = [np.sum(i) for i in output_states[:].powertrain_net_torque]
            a0.plot(manoeuvre.time, powertrain_net_torque, "ROADVIEW1", linestyle="-", label="Calculated")
            engine_torque_M8 = np.array([i.trq_whl_sum_engine for i in sync_zgw])
            break_torque_M8 = np.array([i.break_torque_driver for i in sync_zgw])
            
            powertrain_net_torque_M8 = engine_torque_M8 + break_torque_M8
            a0.plot(vehicle_time, powertrain_net_torque_M8, "ROADVIEW2",  linestyle="--", label = "BMW M8")
            if len(gear_changes)>0:
                a0.axvline(gear_changes[0], color="ROADVIEW4", linestyle=":", label="Gear Change Moment")
                a1.axvline(gear_changes[0], color="ROADVIEW4", linestyle=":", label="Gear Change Moment") 
                for j,i in enumerate(gear_changes[1:]):
                    a0.axvline(i, color="ROADVIEW4", linestyle=":")
                    a1.axvline(i, color="ROADVIEW4", linestyle=":") 
            a0.legend()
            a0.grid()
            a0.set_ylabel("Powertrain Net Torque (Nm)")
            a0.set_xlim(xlim)
            a1.plot(manoeuvre.time, manoeuvre.throttle, "ROADVIEW3", linestyle = "-", label = "Gas pedal(BMW M8)")
            a1.plot(manoeuvre.time, manoeuvre.brake, "ROADVIEW4", linestyle = "-", label = "Brake pedal(BMW M8)")
            a1.step(manoeuvre.time, output_states[:].gear, "ROADVIEW1", linestyle = "-", label = "Calculated Gear Number")
            a1.legend(loc=1)
            a1.set_xlabel("Time (s)")
            a1.grid()
            a1.set_xlim(xlim)
            a1.set_ylim([-0.1, 2.3])
            plt.subplots_adjust(wspace = 0, hspace = 0)
            if savefig:
                plt.savefig(fig_save_dir + "/Powertrain Nettorque.png")

            f, (a0, a1) = plt.subplots(2, 1, gridspec_kw = {'height_ratios': [2, 2]}, figsize = figsize, sharex=True)

            a0.plot(manoeuvre.time, (np.array(output_states[:].engine_w) * 30 / np.pi), "ROADVIEW1", linestyle = "--", label = "Calculated")
            a0.set_ylabel("Engine Rotation (RPM)")

            a0.plot(vehicle_time, [i.rpm_engine for i in sync_zgw], "ROADVIEW2", label = "BMW M8")
            a0.legend()
            a0.grid()
            a0.set_xlim(xlim)
            
            a1.plot(vehicle_time, [i.gear_level - 4 for i in sync_zgw], "ROADVIEW2", linestyle = "-", label = "Gear Number (BMW M8)")
            a1.step(manoeuvre.time, output_states[:].gear, "ROADVIEW1", linestyle = "-", label = "Gear Number (Calculated)")
            a1.plot(manoeuvre.time, manoeuvre.throttle, "ROADVIEW3", linestyle = "-", label = "Gas pedal (BMW M8)")
            a1.plot(manoeuvre.time, manoeuvre.brake, "ROADVIEW4", linestyle = "-", label = "Brake pedal (BMW M8)")
            a1.set_xlim(xlim)
            a1.set_ylim([-0.1, 4.3])
            a1.grid()
            if len(gear_changes)>0:
                a0.axvline(gear_changes[0], color="ROADVIEW4", linestyle=":", label="Gear Change Moment")
                a1.axvline(gear_changes[0], color="ROADVIEW4", linestyle=":", label="Gear Change Moment") 
                for j,i in enumerate(gear_changes[1:]):
                    a0.axvline(i, color="ROADVIEW4", linestyle=":")
                    a1.axvline(i, color="ROADVIEW4", linestyle=":") 
            a1.legend(loc =1)
            a1.set_xlabel("Time (s)")
            plt.subplots_adjust(wspace = 0, hspace = 0) 
            if savefig:
                plt.savefig(fig_save_dir + "/Powertrain RPM.png")

        if "wheels" in plot_type or plot_type == "all":


            gear_changes_idx =np.where(np.roll(output_states[:].gear, 1) != output_states[:].gear)[0]
            gear_changes = manoeuvre.time[gear_changes_idx]

            f, (a0, a1) = plt.subplots(2, 1, gridspec_kw={'height_ratios': [3, 1]}, figsize=figsize, sharex=True)
            powertrain_net_torque = [np.sum(i) for i in output_states[:].powertrain_net_torque]
            a0.plot(manoeuvre.time, powertrain_net_torque, "ROADVIEW3", linestyle="--", label="Calculated")
            engine_torque_M8 = np.array([i.trq_whl_sum_engine for i in sync_zgw])
            break_torque_M8 = np.array([i.break_torque_driver for i in sync_zgw])
            
            powertrain_net_torque_M8 = engine_torque_M8 + break_torque_M8
            a0.plot(vehicle_time, powertrain_net_torque_M8, "ROADVIEW1", label = "BMW M8")
            if len(gear_changes)>0:
                a0.axvline(gear_changes[0], color="ROADVIEW4", linestyle=":", label="Gear Change Moment")
                a1.axvline(gear_changes[0], color="ROADVIEW4", linestyle=":", label="Gear Change Moment") 
                for j,i in enumerate(gear_changes[1:]):
                    a0.axvline(i, color="ROADVIEW4", linestyle=":")
                    a1.axvline(i, color="ROADVIEW4", linestyle=":") 
            a0.legend()
            a0.grid()
            a0.set_ylabel("Powertrain Net Torque (Nm)")
            a0.set_xlim(xlim)
            a1.plot(manoeuvre.time, manoeuvre.throttle, "ROADVIEW3", linestyle = "--", label = "Gas pedal(BMW M8)")
            a1.plot(manoeuvre.time, manoeuvre.brake, "ROADVIEW4", linestyle = "--", label = "Brake pedal(BMW M8)")
            a1.step(manoeuvre.time, output_states[:].gear, "ROADVIEW1", linestyle = "-", label = "Calculated Gear Number")
            a1.legend(loc=1)
            a1.set_xlabel("Time (s)")
            a1.grid()
            a1.set_xlim(xlim)
            plt.subplots_adjust(wspace = 0, hspace = 0)
            if savefig:
                plt.savefig(fig_save_dir + "/Powertrain Nettorque.png")

            f, (a0, a1) = plt.subplots(2, 1, gridspec_kw = {'height_ratios': [3, 1]}, figsize = figsize, sharex=True)

            a0.plot(manoeuvre.time, (np.array(output_states[:].engine_w) * 30 / np.pi), "ROADVIEW1", linestyle = "--", label = "Calculated")
            a0.set_ylabel("Engine Rotation (RPM)")

            a0.plot(vehicle_time, [i.rpm_engine for i in sync_zgw], "ROADVIEW2", label = "BMW M8")
            a0.legend()
            a0.grid()
            a0.set_xlim(xlim)
            
            a1.plot(vehicle_time, [i.gear_level - 4 for i in sync_zgw], "ROADVIEW2", linestyle = "-", label = "Gear Number (BMW M8)")
            a1.step(manoeuvre.time, output_states[:].gear, "ROADVIEW1", linestyle = "-", label = "Gear Number (Calculated)")
            a1.plot(manoeuvre.time, manoeuvre.throttle, "ROADVIEW3", linestyle = "-", label = "Gas pedal (BMW M8)")
            a1.plot(manoeuvre.time, manoeuvre.brake, "ROADVIEW4", linestyle = "-", label = "Brake pedal (BMW M8)")
            a1.set_xlim(xlim)
            a1.grid()
            if len(gear_changes)>0:
                a0.axvline(gear_changes[0], color="ROADVIEW4", linestyle=":", label="Gear Change Moment")
                a1.axvline(gear_changes[0], color="ROADVIEW4", linestyle=":", label="Gear Change Moment") 
                for j,i in enumerate(gear_changes[1:]):
                    a0.axvline(i, color="ROADVIEW4", linestyle=":")
                    a1.axvline(i, color="ROADVIEW4", linestyle=":") 
            a1.legend(loc =1)
            a1.set_xlabel("Time (s)")
            plt.subplots_adjust(wspace = 0, hspace = 0) 
            if savefig:
                plt.savefig(fig_save_dir + "/Powertrain RPM.png")
                

            vx = [i.ins_vel_hor.x for i in sync_adma[:]]
            MINIMUM_SPEED_VALUE = 10


            slip_fr = np.zeros(len(vx))
            slip_rr = np.zeros(len(vx)) 
            slip_fl = np.zeros(len(vx))
            slip_rl = np.zeros(len(vx))

            v_wheel_fl = [i.whl_speed_fl for i in sync_zgw]
            v_wheel_fr = [i.whl_speed_fr for i in sync_zgw]
            v_wheel_rl = [i.whl_speed_rl for i in sync_zgw]
            v_wheel_rr = [i.whl_speed_rr for i in sync_zgw]


            for i in range(len(vx)):
              
                if (abs(static_parameters.tire.dynamic_radius[0] * v_wheel_fl[i]) <= MINIMUM_SPEED_VALUE) and (abs(vx[i]) <= MINIMUM_SPEED_VALUE):
                    slip_fl[i] = ((((static_parameters.tire.dynamic_radius[0] * v_wheel_fl[i]) - vx[i]) / (MINIMUM_SPEED_VALUE)))        
                else:
                    slip_fl[i] = ((((static_parameters.tire.dynamic_radius[0] * v_wheel_fl[i]) - vx[i]) / np.maximum(np.absolute(static_parameters.tire.dynamic_radius[0] * v_wheel_fl[i]), np.absolute(vx[i]))))

                if (abs(static_parameters.tire.dynamic_radius[0] * v_wheel_fr[i]) <= MINIMUM_SPEED_VALUE) and (abs(vx[i]) <= MINIMUM_SPEED_VALUE):
                    slip_fr[i] = ((((static_parameters.tire.dynamic_radius[0] * v_wheel_fr[i]) - vx[i]) / (MINIMUM_SPEED_VALUE)))        
                else:
                    slip_fr[i] = ((((static_parameters.tire.dynamic_radius[0] * v_wheel_fr[i]) - vx[i]) / np.maximum(np.absolute(static_parameters.tire.dynamic_radius[0] * v_wheel_fr[i]), np.absolute(vx[i]))))

                if (abs(static_parameters.tire.dynamic_radius[1] * v_wheel_rl[i]) <= MINIMUM_SPEED_VALUE) and (abs(vx[i]) <= MINIMUM_SPEED_VALUE):
                    slip_rl[i] = ((((static_parameters.tire.dynamic_radius[1] * v_wheel_rl[i]) - vx[i]) / (MINIMUM_SPEED_VALUE)))       
                else:
                    slip_rl[i] = ((((static_parameters.tire.dynamic_radius[1] * v_wheel_rl[i]) - vx[i]) / np.maximum(np.absolute(static_parameters.tire.dynamic_radius[1] * v_wheel_rl[i]), np.absolute(vx[i]))))

                if (abs(static_parameters.tire.dynamic_radius[1] * v_wheel_rr[i]) <= MINIMUM_SPEED_VALUE) and (abs(vx[i]) <= MINIMUM_SPEED_VALUE):
                    slip_rr[i] = ((((static_parameters.tire.dynamic_radius[1] * v_wheel_rr[i]) - vx[i]) / (MINIMUM_SPEED_VALUE)))       
                else:
                    slip_rr[i] = ((((static_parameters.tire.dynamic_radius[1] * v_wheel_rr[i]) - vx[i]) / np.maximum(np.absolute(static_parameters.tire.dynamic_radius[1] * v_wheel_rr[i]), np.absolute(vx[i]))))

            f, ((a0, a1),(a2,a3)) = plt.subplots(2, 2, figsize=figsize, sharex=True,sharey=True)
            a0.plot(vehicle_time, slip_fl, "ROADVIEW1", label="Longitudinal Slip FL")
            a0.plot(manoeuvre.time, [i[0] for i in output_states[:].slip_x], "ROADVIEW1", linestyle ="--", label ="Calculated Longitudinal Slip FL")
            a0.set_xlim(xlim)
            a0.legend()
            a0.set_ylim(-0.2, 0.2)
            a0.set_xlabel("Time (s)")
            a0.set_ylabel("Wheel Slip")
            a2.set_xlabel("Time (s)")
            a2.set_ylabel("Wheel Slip")
            a2.plot(vehicle_time, slip_rl, "ROADVIEW2", label="Longitudinal Slip RL")
            a2.plot(manoeuvre.time, [i[1] for i in output_states[:].slip_x], "ROADVIEW2", linestyle ="--", label ="Calculated Longitudinal Slip RL")
            a2.set_xlim(xlim)
            a2.legend()
            a1.set_ylim(-0.2, 0.2)
            a1.set_xlabel("Time (s)")
            a1.set_ylabel("Wheel Slip")
            a1.plot(vehicle_time, slip_fr, "ROADVIEW3", label="Longitudinal Slip FR")
            a1.plot(manoeuvre.time, [i[2] for i in output_states[:].slip_x], "ROADVIEW3", linestyle ="--", label ="Calculated Longitudinal Slip FR")
            a1.set_xlim(xlim)
            a1.legend()
            a1.set_ylim(-0.2, 0.2)
            a3.set_ylim(-0.2, 0.2)
            a3.set_xlabel("Time (s)")
            a3.set_ylabel("Wheel Slip")
            a3.plot(vehicle_time, slip_rr, "ROADVIEW4", label="Longitudinal Slip RR")
            a3.plot(manoeuvre.time, [i[3] for i in output_states[:].slip_x], "ROADVIEW4", linestyle ="--", label ="Calculated Longitudinal Slip RR")
            a3.set_xlim(xlim)
            a3.legend()
            plt.tight_layout()
            if savefig:
                plt.savefig(fig_save_dir + "/Unoptimized wheel_slip_long_subplot-wheelLoad.png")

            f, ((a0, a1),(a2,a3)) = plt.subplots(2, 2, figsize=figsize, sharex=True,sharey=True)
            a0.plot(vehicle_time, [i.slip_angle_vehicle for i in sync_zgw], "ROADVIEW1", label="Lateral Slip FL")
            a0.plot(manoeuvre.time, [i[0] for i in output_states[:].slip_y], "ROADVIEW1", linestyle ="--", label ="Calculated Lateral Slip FL")
            a0.set_xlim(xlim)
            a0.legend()
            a0.set_ylim(-0.2, 0.2)
            a0.set_xlabel("Time (s)")
            a0.set_ylabel("Wheel Slip")
            a2.set_xlabel("Time (s)")
            a2.set_ylabel("Wheel Slip")
            a2.plot(vehicle_time, [i.slip_angle_vehicle for i in sync_zgw], "ROADVIEW2", label="Lateral Slip RL")
            a2.plot(manoeuvre.time, [i[1] for i in output_states[:].slip_y], "ROADVIEW2", linestyle ="--", label ="Calculated Lateral Slip RL")
            a2.set_xlim(xlim)
            a2.legend()
            a1.set_ylim(-0.2, 0.2)
            a1.set_xlabel("Time (s)")
            a1.set_ylabel("Wheel Slip")
            a1.plot(vehicle_time, [i.slip_angle_vehicle for i in sync_zgw], "ROADVIEW3", label="Lateral Slip FR")
            a1.plot(manoeuvre.time, [i[2] for i in output_states[:].slip_y], "ROADVIEW3", linestyle ="--", label ="Calculated Lateral Slip FR")
            a1.set_xlim(xlim)
            a1.legend()
            a1.set_ylim(-0.2, 0.2)
            a3.set_ylim(-0.2, 0.2)
            a3.set_xlabel("Time (s)")
            a3.set_ylabel("Wheel Slip")
            a3.plot(vehicle_time, [i.slip_angle_vehicle for i in sync_zgw], "ROADVIEW4", label="Lateral Slip RR")
            a3.plot(manoeuvre.time, [i[3] for i in output_states[:].slip_y], "ROADVIEW4", linestyle ="--", label ="Calculated Lateral Slip RR")
            a3.set_xlim(xlim)
            a3.legend()
            plt.tight_layout()
            if savefig:
                plt.savefig(fig_save_dir + "/Unoptimized wheel_slip_side_subplot-wheelLoad.png")
            
            plt.figure(figsize=figsize)
            plt.plot(vehicle_time, [i.slip_angle_vehicle for i in sync_zgw], "ROADVIEW1", label="Lateral Slip FL")
            plt.plot(vehicle_time, [i.slip_angle_vehicle for i in sync_zgw], "ROADVIEW2", label="Lateral Slip RL")
            plt.plot(vehicle_time, [i.slip_angle_vehicle for i in sync_zgw], "ROADVIEW3", label="Lateral Slip FR")
            plt.plot(vehicle_time, [i.slip_angle_vehicle for i in sync_zgw], "ROADVIEW4", label="Lateral Slip RR")
            plt.legend()
            plt.tight_layout()  
            if savefig:
                plt.savefig(fig_save_dir + "/Unoptimized wheel_slip_side_subplot-wheelLoad.png")

            f, ((a0, a1),(a2,a3)) = plt.subplots(2, 2, figsize=figsize, sharex=True,sharey=True)
            a0.plot(vehicle_time,  [i.whl_speed_fl for i in sync_zgw], "ROADVIEW1", label="Wheel Velocity FL")
            a0.plot(manoeuvre.time, [i[0] for i in output_states[:].wheel_w_vel], "ROADVIEW1", linestyle ="--", label ="Calculated Wheel Velocity FL")
            a0.set_xlim(xlim)
            a0.legend()
            a0.set_xlabel("Time (s)")
            a0.set_ylabel("Wheel Velocity")
            a2.set_xlabel("Time (s)")
            a2.set_ylabel("Wheel Velocity")
            a2.plot(vehicle_time, [i.whl_speed_rl for i in sync_zgw], "ROADVIEW2", label="Wheel Velocity RL")
            a2.plot(manoeuvre.time, [i[1] for i in output_states[:].wheel_w_vel], "ROADVIEW2", linestyle ="--", label ="Calculated Wheel Velocity RL")
            a2.set_xlim(xlim)
            a2.legend()
            a1.set_xlabel("Time (s)")
            a1.set_ylabel("Wheel Velocity")
            a1.plot(vehicle_time, [i.whl_speed_fr for i in sync_zgw], "ROADVIEW3", label="Wheel Velocity FR")
            a1.plot(manoeuvre.time, [i[2] for i in output_states[:].wheel_w_vel], "ROADVIEW3", linestyle ="--", label ="Calculated Wheel Velocity FR")
            a1.set_xlim(xlim)
            a1.legend()
            a3.set_xlabel("Time (s)")
            a3.set_ylabel("Wheel Velocity")
            a3.plot(vehicle_time, [i.whl_speed_rr for i in sync_zgw], "ROADVIEW4", label="Wheel Velocity RR")
            a3.plot(manoeuvre.time, [i[3] for i in output_states[:].wheel_w_vel], "ROADVIEW4", linestyle ="--", label ="Calculated Wheel Velocity RR")
            a3.set_xlim(xlim)
            a3.legend()
            plt.tight_layout()
            if savefig:
                plt.savefig(fig_save_dir + "/Unoptimized wheel_velocity_subplot-wheelLoad.png")
            # x_rf.fx
            f, ((a0, a1),(a2,a3)) = plt.subplots(2, 2, figsize=figsize, sharex=True,sharey=True)
            
            a0.plot(manoeuvre.time, [i.fy[0] for i in output_states[:].x_rf], "ROADVIEW1", linestyle ="--", label ="Fy FL")
            a0.set_xlim(xlim)
            a0.legend()
            a0.set_xlabel("Time (s)")
            a0.set_ylabel("Wheel Forces in y")
            a2.set_xlabel("Time (s)")
            a2.set_ylabel("Wheel Forces in y")
            a2.plot(manoeuvre.time, [i.fy[1] for i in output_states[:].x_rf], "ROADVIEW2", linestyle ="--", label ="Fy RL")
            a2.set_xlim(xlim)
            a2.legend()
            a1.set_xlabel("Time (s)")
            a1.set_ylabel("Wheel Forces in y")
            a1.plot(manoeuvre.time, [i.fy[2] for i in output_states[:].x_rf], "ROADVIEW3", linestyle ="--", label ="Fy FR")
            a1.set_xlim(xlim)
            a1.legend()
            a3.set_xlabel("Time (s)")
            a3.set_ylabel("Wheel Forces in y")
            a3.plot(manoeuvre.time, [i.fy[3] for i in output_states[:].x_rf], "ROADVIEW4", linestyle ="--", label ="Fy RR")
            a3.set_xlim(xlim)
            a3.legend()
            plt.tight_layout()
            if savefig:
                plt.savefig(fig_save_dir + "/Unoptimized wheel_velocity_subplot-forcesY.png")
 
        #Comparison of data from ISO
        if "iso" in plot_type or plot_type == "all":
            
            plt.figure(figsize=figsize)
            name = " Steer vs. lateral acceleration"
            plt.title(name)
            
            steering = manoeuvre.steering * 100         # Steering in Prozent 
            
            aligned_steering = np.zeros(len(vehicle_time))  # Steering richtige Dimension

            # Für jeden Zeitstempel in vehicle_time den nächstgelegenen Zeitstempel in steering_time finden
            for i, vt in enumerate(vehicle_time):
                # Index des minimalen Zeitdifferenzwerts finden
                closest_index = np.abs(manoeuvre.time - vt).argmin()
                # Wert von steering an der nächstgelegenen Zeit zuweisen
                aligned_steering[i] = steering[closest_index]

            #Reale Daten
            plt.plot(np.abs([(i.acc_hor.y)*9.81 for i in sync_adma[:]]), np.abs(aligned_steering), "o", label="M8 Acceleration Y", color='blue', markersize=5) 

            aligned_steering_prev = np.zeros(len(aligned_steering))  
            aligned_steering_prev[1:] = aligned_steering[:-1]        
            acc_Y_prev = [0] + [i.acc_hor.y for i in sync_adma[:-1]]  

            #Deltas
            delta_acc_Y = np.abs(np.abs([(i.acc_hor.y)*9.81 for i in sync_adma[:]]) - (np.abs(acc_Y_prev)*9.81))
            delta_steering = np.abs((np.abs(np.abs(aligned_steering) - np.abs(aligned_steering_prev))))

            #Toleranzen
            toleranz_X = 0.1 + 0.06 * np.abs([(i.acc_hor.y)*9.81 for i in sync_adma[:]])    #Toleranzen sind falsch weil nicht das vorgegebene Fahrmanöver gefahren würde
            toleranz_Y = 5.0 + 0.03 * np.abs(aligned_steering)

            D = ((delta_acc_Y * toleranz_Y)**2 + (delta_steering * toleranz_X)**2)**(1/2)

            #Top-Toleranz
            plt.plot(np.abs([(i.acc_hor.y)*9.81 for i in sync_adma[:]]) - ((delta_steering * (toleranz_X)**2) / D), np.abs(aligned_steering) + ((delta_acc_Y * (toleranz_Y**2)) / D), "o",  label="Top-Toleranz", color='red', markersize=3) 
    	    #Bot-Toleranz
            plt.plot(np.abs([(i.acc_hor.y)*9.81 for i in sync_adma[:]]) + ((delta_steering * (toleranz_X)**2) / D), np.abs(aligned_steering) - ((delta_acc_Y * (toleranz_Y**2)) / D), "o",  label="Bot-Toleranz", color='red', markersize=3) 

            #Aus Simulation
            plt.plot(np.abs([i.acc_y for i in output_states[:].x_a]), np.abs(steering), "o",  label="Calculated Acceleration Y" , color='purple', markersize=5)
            
            plt.grid()
            plt.xlabel("Acceleration [m/s²]")
            plt.ylabel("Steering [deg]")
            plt.legend()
            plt.tight_layout()



            f, (a0, a1, a2) = plt.subplots(3, 1, figsize=figsize, sharex=True,sharey=True)
            a0.plot(manoeuvre.time, [i.vx for i in output_states[:].x_a], "ROADVIEW1", linestyle="--", label="Calculated Velocity X")
            a0.set_xlim(xlim)
            a0.legend()
            a0.set_xlabel("Time (s)")
            a0.set_ylabel("Velocity (m/s)")
            
            a2.set_xlabel("Time (s)")
            a2.set_ylabel("Steering (deg)")
            a2.plot(manoeuvre.time, manoeuvre.steering * 1440.11, "ROADVIEW3", label="M8 Steering")
            a2.plot(vehicle_time, [i.steering_wheel_angle for i in sync_zgw], "ROADVIEW1", label="Steering Wheel Angle")
            a2.set_xlim(xlim)
            a2.legend()
            
            a1.set_xlabel("Time (s)")
            a1.set_ylabel("Acceleration (m/s²)")
            a1.plot(manoeuvre.time, [i.acc_y for i in output_states[:].x_a], "ROADVIEW2", linestyle="--", label="Calculated Acceleration Y")
            a1.plot(vehicle_time, [(i.acc_hor.y)*9.81 for i in sync_adma[:]], "ROADVIEW2", label="M8 Acceleration Y")
            a1.set_xlim(xlim)
            a1.legend()
            
            plt.tight_layout()
            
    else:
        plt.figure(figsize=figsize)
        name = " Chassis Angles"
        plt.title(name)
        var_name = "Vhcl_Roll"
        plt.plot(vehicle_time,[(i.ins_roll)*3.14/180 for i in sync_adma[:]], "ROADVIEW1", label="M8 Roll")
        plt.plot(vehicle_time,[(i.ins_pitch)*3.14/180 for i in sync_adma[:]], "ROADVIEW2", label="M8 Pitch")
        plt.plot(vehicle_time,[(i.ins_yaw-270)*3.14/180 for i in sync_adma[:]], "ROADVIEW3", label="M8 Yaw")
        plt.grid()
        plt.legend()
        plt.xlim(xlim)
        plt.xlabel("Time (s)")
        plt.ylabel("Angle (rad)")
        plt.tight_layout()
        if savefig:
            plt.savefig(fig_save_dir + "/" + name + ".png")  
        plt.figure(figsize=figsize)
        name = " Chassis Angles Velocity"
        plt.title(name)

        plt.plot(vehicle_time,[i.rate_hor.x*3.14/180 for i in sync_adma[:]], "ROADVIEW1", label="M8 Roll Velocity")
        plt.plot(vehicle_time,[i.rate_hor.y*3.14/180 for i in sync_adma[:]], "ROADVIEW2", label="M8 Pitch Velocity")
        plt.plot(vehicle_time,[(i.rate_hor.z)*3.14/180 for i in sync_adma[:]], "ROADVIEW3", label="M8 Yaw Velocity")
        plt.xlabel("Time (s)")
        plt.ylabel("Angle Velocity (rad/s)")
        plt.legend()
        plt.xlim(xlim)
        plt.grid()
        plt.tight_layout()
        if savefig:
            plt.savefig(fig_save_dir + "/" + name + ".png")   
        plt.figure(figsize=figsize)
        name = " Chassis Acceleration"
        plt.title(name)
        plt.plot(vehicle_time, [(i.acc_hor.x)*9.81 for i in sync_adma[:]], "ROADVIEW1", label="M8 Acceleration X")
        plt.plot(vehicle_time, [(i.acc_hor.y)*9.81 for i in sync_adma[:]], "ROADVIEW2", label="M8 Acceleration Y")
        plt.plot(vehicle_time, [(i.acc_hor.z -1)*9.81 for i in sync_adma[:]], "ROADVIEW3", label="M8 Acceleration Z")
        plt.grid()
        plt.xlabel("Time (s)")
        plt.ylabel("Acceleration (m/s²)")
        plt.legend()
        plt.xlim(xlim)
        plt.tight_layout()
        if savefig:
            plt.savefig(fig_save_dir + "/" + name + ".png")     
        plt.figure(figsize=figsize)
        name = " Chassis Velocity"
        plt.title(name)
        plt.plot(vehicle_time, [i.ins_vel_hor.x for i in sync_adma[:]], "ROADVIEW1", label="M8 Velocity X")
        plt.plot(vehicle_time, [i.ins_vel_hor.y for i in sync_adma[:]], "ROADVIEW2", label="M8 Velocity Y")
        plt.plot(vehicle_time, [i.ins_vel_hor.z for i in sync_adma[:]], "ROADVIEW3", label="M8 Velocity Z")

        plt.grid()
        plt.xlabel("Time (s)")
        plt.ylabel("Velocity (m/s)")
        plt.legend()
        plt.xlim(xlim)
        plt.tight_layout()
        if savefig:
            plt.savefig(fig_save_dir + "/" + name + ".png")   

        plt.figure(figsize=figsize)
        name = " Chassis Position"
        plt.title(name)
        plt.plot([i.ins_pos_rel_y for i in sync_adma[:]], [i.ins_pos_rel_x for i in sync_adma[:]], "ROADVIEW1", label="M8 Position")
        plt.grid()
        plt.xlabel("Position X (m)")
        plt.ylabel("Position Y (m)")
        plt.legend()
        plt.tight_layout()

        plt.figure(figsize=figsize)
        name = " Absolute Position"
        plt.title(name)
        plt.plot([i.ins_long_abs for i in sync_adma[:]], [i.ins_lat_abs for i in sync_adma[:]], "ROADVIEW1", label="M8 Position")
        plt.grid()
        plt.xlabel("Position long (m)")
        plt.ylabel("Position lat (m)")
        plt.legend()
        plt.tight_layout()



 



        gear_changes = []

        f, (a0, a1) = plt.subplots(2, 1, gridspec_kw={'height_ratios': [3, 1]}, figsize=figsize, sharex=True)
        powertrain_net_torque_M8 = [i.trq_whl_sum_engine for i in sync_zgw]
        a0.plot(vehicle_time, powertrain_net_torque_M8, "ROADVIEW1", label = "BMW M8")
        if len(gear_changes)>0:
            a0.axvline(gear_changes[0], color="ROADVIEW4", linestyle=":", label="Gear Change Moment")
            a1.axvline(gear_changes[0], color="ROADVIEW4", linestyle=":", label="Gear Change Moment") 
            for j,i in enumerate(gear_changes[1:]):
                a0.axvline(i, color="ROADVIEW4", linestyle=":")
                a1.axvline(i, color="ROADVIEW4", linestyle=":") 
        a0.legend()
        a0.grid()
        a0.set_ylabel("Powertrain Net Torque (Nm)")
        a0.set_xlim(xlim)
        a1.plot(manoeuvre.time, manoeuvre.throttle, "ROADVIEW3", linestyle = "--", label = "Gas pedal(BMW M8)")
        a1.plot(manoeuvre.time, manoeuvre.brake, "ROADVIEW4", linestyle = "--", label = "Brake pedal(BMW M8)")
        a1.legend(loc=1)
        a1.set_xlabel("Time (s)")
        a1.grid()
        a1.set_xlim(xlim)
        plt.subplots_adjust(wspace = 0, hspace = 0)
        if savefig:
            plt.savefig(fig_save_dir + "/Powertrain Nettorque.png")

        vx = [i.ins_vel_hor.x for i in sync_adma[:]]
        MINIMUM_SPEED_VALUE = 10


        slip_fr = np.zeros(len(vx))
        slip_rr = np.zeros(len(vx)) 
        slip_fl = np.zeros(len(vx))
        slip_rl = np.zeros(len(vx))

        v_wheel_fl = [i.whl_speed_fl for i in sync_zgw]
        v_wheel_fr = [i.whl_speed_fr for i in sync_zgw]
        v_wheel_rl = [i.whl_speed_rl for i in sync_zgw]
        v_wheel_rr = [i.whl_speed_rr for i in sync_zgw]


        for i in range(len(vx)):
          
            if (abs(static_parameters.tire.dynamic_radius[0] * v_wheel_fl[i]) <= MINIMUM_SPEED_VALUE) and (abs(vx[i]) <= MINIMUM_SPEED_VALUE):
                slip_fl[i] = ((((static_parameters.tire.dynamic_radius[0] * v_wheel_fl[i]) - vx[i]) / (MINIMUM_SPEED_VALUE)))        
                slip_fl[i] = ((((static_parameters.tire.dynamic_radius[0] * v_wheel_fl[i]) - vx[i]) / np.maximum(np.absolute(static_parameters.tire.dynamic_radius[0] * v_wheel_fl[i]), np.absolute(vx[i]))))

            if (abs(static_parameters.tire.dynamic_radius[0] * v_wheel_fr[i]) <= MINIMUM_SPEED_VALUE) and (abs(vx[i]) <= MINIMUM_SPEED_VALUE):
                slip_fr[i] = ((((static_parameters.tire.dynamic_radius[0] * v_wheel_fr[i]) - vx[i]) / (MINIMUM_SPEED_VALUE)))        
                slip_fr[i] = ((((static_parameters.tire.dynamic_radius[0] * v_wheel_fr[i]) - vx[i]) / np.maximum(np.absolute(static_parameters.tire.dynamic_radius[0] * v_wheel_fr[i]), np.absolute(vx[i]))))

            if (abs(static_parameters.tire.dynamic_radius[1] * v_wheel_rl[i]) <= MINIMUM_SPEED_VALUE) and (abs(vx[i]) <= MINIMUM_SPEED_VALUE):
                slip_rl[i] = ((((static_parameters.tire.dynamic_radius[1] * v_wheel_rl[i]) - vx[i]) / (MINIMUM_SPEED_VALUE)))       
                slip_rl[i] = ((((static_parameters.tire.dynamic_radius[1] * v_wheel_rl[i]) - vx[i]) / np.maximum(np.absolute(static_parameters.tire.dynamic_radius[1] * v_wheel_rl[i]), np.absolute(vx[i]))))

            if (abs(static_parameters.tire.dynamic_radius[1] * v_wheel_rr[i]) <= MINIMUM_SPEED_VALUE) and (abs(vx[i]) <= MINIMUM_SPEED_VALUE):
                slip_rr[i] = ((((static_parameters.tire.dynamic_radius[1] * v_wheel_rr[i]) - vx[i]) / (MINIMUM_SPEED_VALUE)))       
                slip_rr[i] = ((((static_parameters.tire.dynamic_radius[1] * v_wheel_rr[i]) - vx[i]) / np.maximum(np.absolute(static_parameters.tire.dynamic_radius[1] * v_wheel_rr[i]), np.absolute(vx[i]))))

        f, ((a0, a1),(a2,a3)) = plt.subplots(2, 2, figsize=figsize, sharex=True,sharey=True)
        a0.plot(vehicle_time, slip_fl, "ROADVIEW1", label="Longitudinal Slip FL")
        a0.set_xlim(xlim)
        a0.legend()
        a0.set_ylim(-0.2, 0.2)
        a0.set_xlabel("Time (s)")
        a0.set_ylabel("Wheel Slip")
        a2.set_xlabel("Time (s)")
        a2.set_ylabel("Wheel Slip")
        a2.plot(vehicle_time, slip_rl, "ROADVIEW2", label="Longitudinal Slip RL")

        a2.set_xlim(xlim)
        a2.legend()
        a1.set_ylim(-0.2, 0.2)
        a1.set_xlabel("Time (s)")
        a1.set_ylabel("Wheel Slip")
        a1.plot(vehicle_time, slip_fr, "ROADVIEW3", label="Longitudinal Slip FR")
        a1.set_xlim(xlim)
        a1.legend()
        a1.set_ylim(-0.2, 0.2)
        a3.set_ylim(-0.2, 0.2)
        a3.set_xlabel("Time (s)")
        a3.set_ylabel("Wheel Slip")
        a3.plot(vehicle_time, slip_rr, "ROADVIEW4", label="Longitudinal Slip RR")
        a3.set_xlim(xlim)
        a3.legend()
        plt.tight_layout()
        if savefig:
            plt.savefig(fig_save_dir + "/Unoptimized wheel_slip_long_subplot-wheelLoad.png")

        f, ((a0, a1),(a2,a3)) = plt.subplots(2, 2, figsize=figsize, sharex=True,sharey=True)
        a0.plot(vehicle_time, [i.slip_angle_vehicle for i in sync_zgw], "ROADVIEW1", label="Lateral Slip FL")
        a0.set_xlim(xlim)
        a0.legend()
        a0.set_ylim(-0.2, 0.2)
        a0.set_xlabel("Time (s)")
        a0.set_ylabel("Wheel Slip")
        a2.set_xlabel("Time (s)")
        a2.set_ylabel("Wheel Slip")
        a2.plot(vehicle_time, [i.slip_angle_vehicle for i in sync_zgw], "ROADVIEW2", label="Lateral Slip RL")
        a2.set_xlim(xlim)
        a2.legend()
        a1.set_ylim(-0.2, 0.2)
        a1.set_xlabel("Time (s)")
        a1.set_ylabel("Wheel Slip")
        a1.plot(vehicle_time, [i.slip_angle_vehicle for i in sync_zgw], "ROADVIEW3", label="Lateral Slip FR")
        a1.set_xlim(xlim)
        a1.legend()
        a1.set_ylim(-0.2, 0.2)
        a3.set_ylim(-0.2, 0.2)
        a3.set_xlabel("Time (s)")
        a3.set_ylabel("Wheel Slip")
        a3.plot(vehicle_time, [i.slip_angle_vehicle for i in sync_zgw], "ROADVIEW4", label="Lateral Slip RR")
        a3.set_xlim(xlim)
        a3.legend()
        plt.tight_layout()
        if savefig:
            plt.savefig(fig_save_dir + "/Unoptimized wheel_slip_side_subplot-wheelLoad.png")

        plt.figure(figsize=figsize)
        plt.plot(vehicle_time, [i.steering_wheel_angle/1440.11 for i in sync_zgw], "ROADVIEW1", label="Steering Wheel Angle")
        #plt.plot(vehicle_time, [i.slip_angle_vehicle for i in sync_zgw], "ROADVIEW2", label="Lateral Slip RL")
        #plt.plot(vehicle_time, [i.slip_angle_vehicle for i in sync_zgw], "ROADVIEW3", label="Lateral Slip FR")
        #plt.plot(vehicle_time, [i.slip_angle_vehicle for i in sync_zgw], "ROADVIEW4", label="Lateral Slip RR")
        plt.legend()
        plt.tight_layout()  
        if savefig:
            plt.savefig(fig_save_dir + "/Unoptimized wheel_slip_side_subplot-wheelLoad.png")

        f, ((a0, a1),(a2,a3)) = plt.subplots(2, 2, figsize=figsize, sharex=True,sharey=True)
        a0.plot(vehicle_time,  [i.whl_speed_fl for i in sync_zgw], "ROADVIEW1", label="Wheel Velocity FL")
        a0.set_xlim(xlim)
        a0.legend()
        a0.set_xlabel("Time (s)")
        a0.set_ylabel("Wheel Velocity")
        a2.set_xlabel("Time (s)")
        a2.set_ylabel("Wheel Velocity")
        a2.plot(vehicle_time, [i.whl_speed_rl for i in sync_zgw], "ROADVIEW2", label="Wheel Velocity RL") 
        a2.set_xlim(xlim)
        a2.legend()
        a1.set_xlabel("Time (s)")
        a1.set_ylabel("Wheel Velocity")
        a1.plot(vehicle_time, [i.whl_speed_fr for i in sync_zgw], "ROADVIEW3", label="Wheel Velocity FR")
        a1.set_xlim(xlim)
        a1.legend()
        a3.set_xlabel("Time (s)")
        a3.set_ylabel("Wheel Velocity")
        a3.plot(vehicle_time, [i.whl_speed_rr for i in sync_zgw], "ROADVIEW4", label="Wheel Velocity RR")
        a3.set_xlim(xlim)
        a3.legend()
        plt.tight_layout()
        if savefig:
            plt.savefig(fig_save_dir + "/Unoptimized wheel_velocity_subplot-wheelLoad.png")

        

    
            
            

 


    plt.show()
