import pandas as pd
import numpy as np

'''#Inertial Properties'''
# Design Parameters
M_sphere = 6 # [kg]
D = 0.45 # [m]
L = 0.3 # [m]
v_ball = 1.0 # [m/s]

# Respective parameters
design_torque = 10.5 # [Nm]

#Power
V_nom = 24 # [Volts]
desired_run_time = 60  # [minutes]


#Motor Data
torque_lst = np.array([6.62, 0.15, 3.5, 2.0, 0.65])  # [Nm]
kv_lst = np.array([100, 90, 130, 560, 270])  # [RPM/V]
gear_ratio_lst = np.array(torque_lst)/design_torque  # torque_lst/design_torque
torque_gear_lst = np.ceil(gear_ratio_lst)*torque_lst  # Torque with gear ratio
max_rpm_lst = 1/(np.ceil(gear_ratio_lst)) *kv_lst*V_nom  # Max RPM with gear ratio
motor_data = {
    "Name": [
        "ODrive S1 and M8325s Motor Kit",
        "Rctimer GBM5010-150T 90KV",
        "Eaglepower LA8308 130kv",
        "C4250 Outrunner",
        "Motor 5065"
    ],
    "Torque (kg)": torque_lst,
    "KV (RPM/V)":  kv_lst,
    "Gear ratio ": gear_ratio_lst,  # Assuming current ratings
    "Torque with gear ratio": torque_gear_lst,
    "Max RPM": max_rpm_lst,
    "Price (€)": [117.71, 15.80, 55.77, 37.34, None]  # None for missing value
}

pd_motor = pd.DataFrame(motor_data)


# power data
battery_data = [
    # {"name": "18650 Li-ion (NMC/NCA)", "nominal_voltage_V": 3.6,  "capacity_Ah": 3.0, "max_continuous_discharge_A": 15},
    # {"name": "21700 Li-ion (NMC/NCA)", "nominal_voltage_V": 3.6,  "capacity_Ah": 4.8, "max_continuous_discharge_A": 25},
    # {"name": "26650 LiFePO4",          "nominal_voltage_V": 3.2,  "capacity_Ah": 3.3, "max_continuous_discharge_A": 25},
    # {"name": "32700 LiFePO4",          "nominal_voltage_V": 3.2,  "capacity_Ah": 6.0, "max_continuous_discharge_A": 25},
    # {"name": "LTO cylindrical",        "nominal_voltage_V": 2.3,  "capacity_Ah": 20.0,"max_continuous_discharge_A": 200},

    {"name": "LiPo 2S (7.4V) 2200mAh 30C",  "nominal_voltage_V": 7.4,  "capacity_Ah": 2.2, "max_continuous_discharge_A": 66},
    {"name": "LiPo 3S (11.1V) 2200mAh 30C", "nominal_voltage_V": 11.1, "capacity_Ah": 2.2, "max_continuous_discharge_A": 66},
    {"name": "LiPo 4S (14.8V) 5000mAh 25C", "nominal_voltage_V": 14.8, "capacity_Ah": 5.0, "max_continuous_discharge_A": 125},
    {"name": "LiPo 6S (22.2V) 5000mAh 25C", "nominal_voltage_V": 22.2, "capacity_Ah": 5.0, "max_continuous_discharge_A": 125},

    {"name": "NiMH 6S Sub-C pack (7.2V) 3000mAh", "nominal_voltage_V": 7.2, "capacity_Ah": 3.0, "max_continuous_discharge_A": 15},
    {"name": "NiMH 8S Sub-C pack (9.6V) 3000mAh", "nominal_voltage_V": 9.6, "capacity_Ah": 3.0, "max_continuous_discharge_A": 15},
]

pd_battery = pd.DataFrame(
    battery_data,
    columns=["name", "nominal_voltage_V", "capacity_Ah", "max_continuous_discharge_A"],
)





