import numpy as np
import scipy.signal as ss
import plotly.graph_objects as go
import pandas as pd
import re
from easydict import EasyDict as edict
from plotly.offline import init_notebook_mode, iplot
import tqdm
#
# filenames = ['2021-07-26-11-46-55_zhongweibattery2.csv']
filenames=['2021-08-09-11-18-47_battary_bugfix_test.csv']
csvdata = pd.read_csv(filenames[0], names=[
    'count', 'correction', 'battery_current_', 'battery_voltage_', 'filtered_battery_voltage_', 'duration_in_secs', 'soc_from_voltage_and_current', 'capacity_loss_coulomb_', 'soc_est_','soc_init_','accumulated_coulomb_'], delim_whitespace=True)
PARAMETER_ORDER = 2
FULLY_CHARGED_SOC_LATCH = 1.005
BATTERY_EMPTY_VOLTAGE = 12
BATTERY_VOLTAGE_ERROR_EPS = 0.001
BATTERY_VOLTAGE_FILTER_COEFF = 0.8
MAX_CORRECTION = 20
soc = []
battery_current_ = np.array(csvdata['battery_current_'])
battery_voltage_ = np.array(csvdata['battery_voltage_'])
duration_in_secs = np.array(csvdata['duration_in_secs'])
soc_voltage_and_current = np.array(csvdata['soc_from_voltage_and_current'])
count=np.array(csvdata['count'])
soc_init_from_firmare=np.array(csvdata['soc_init_'])
class BatteryParameters:
    Es_coeff = [13.335511, 0.214605, -0.020467]
    r_coeff = [0.018017, 0.104015, -0.011780]
    K_coeff = [0.0744809, -0.0348637, 0.0043584]
    A_coeff = [3.435135, -0.237635, 0.029449]
    B_coeff = [1.883096, 0.292615, -0.034244]
    Q_coeff = [5.439359, -0.139724, 0.024220]
    # Es_coeff = [13.652775, 0.111510, -0.017809]
    # r_coeff = [0.012284, 0.082752, -0.011202]
    # K_coeff = [0.0565515, -0.0246055, 0.0029482]
    # A_coeff = [3.0594967, -0.0515255, 0.0078525]
    # B_coeff = [2.293349, 0.087986, -0.012899]
    # Q_coeff = [5.490381, -0.212213, 0.035084]


battery_parameters = BatteryParameters()
max_battery_charge_current = 0
filtered_battery_voltage = battery_voltage_[0]
accumulated_coulomb = 0.0
Q_rated = 5.2
last_current = 0
capacity_loss_coulomb = 0
soc_est_ = 0
socinitflag = 0
IsDetachedBatteryFromCharger = False

def DischargingBatteryVoltageToSOC(voltage, current, index):
    current_order = 1.0
    Es = 0.0
    r = 0
    K = 0
    A = 0
    B = 0
    Q = 0
    for i in range(PARAMETER_ORDER+1):
        Es += battery_parameters.Es_coeff[i] * current_order
        r += battery_parameters.r_coeff[i] * current_order
        K += battery_parameters.K_coeff[i] * current_order
        A += battery_parameters.A_coeff[i] * current_order
        B += battery_parameters.B_coeff[i] * current_order
        Q += battery_parameters.Q_coeff[i] * current_order
        current_order *= current
    if Q >= Q_rated:
        Q = Q_rated
    # print(f'{Es},{r},{K},{A},{B},{Q},{current}')
    soc_left_bound = 0.0
    soc_right_bound = Q
    current_discharged = (soc_left_bound + soc_right_bound) / 2
    battery_full_voltage = (Es - r * current - (K * Q / (Q - soc_left_bound)
                            * (current)) + A * np.exp(-B / Q * soc_left_bound))
    v_residual = 10000
    last_v_residual = 10000
    if (voltage > battery_full_voltage):  # full battery
        return FULLY_CHARGED_SOC_LATCH
    elif (voltage < BATTERY_EMPTY_VOLTAGE):  # empty battery
        return 0
    else:
        while (abs(v_residual) > BATTERY_VOLTAGE_ERROR_EPS):
            current_discharged = (soc_left_bound + soc_right_bound) / 2

            # this is the modified Shepherd's model to describe SOC vs battery
            # voltage with a given battery discharge current
            v_est = (Es - r * current - (K * Q / (Q - current_discharged) * (current)) + A * np.exp(-B / Q * current_discharged))
            v_residual = voltage - v_est
            if last_v_residual*v_residual > 0 and abs(v_residual) > abs(last_v_residual):
                # print(index)
                break
            if (v_residual < 0):
                soc_left_bound = current_discharged
            else:
                soc_right_bound = current_discharged
            last_v_residual = v_residual
        # convert the to state of charge in percentage
        soc_est = (Q - current_discharged) / Q
        return soc_est





def SocEstimation(battery_voltage_, battery_current_, duration_in_secs, index, soc_voltage):
    global socinitflag, accumulated_coulomb, capacity_loss_coulomb, last_current, soc_est_, IsDetachedBatteryFromCharger, max_battery_charge_current, filtered_battery_voltage,soc_init_
    if battery_current_ > 0:
        charging_status = 1
    else:
        charging_status = 0
    if battery_current_ > max_battery_charge_current:
        max_battery_charge_current = battery_current_

    if (filtered_battery_voltage == 0.0):
        filtered_battery_voltage = battery_voltage_
    else:
        filtered_battery_voltage = (BATTERY_VOLTAGE_FILTER_COEFF * battery_voltage_ + (
            1.0 - BATTERY_VOLTAGE_FILTER_COEFF) * filtered_battery_voltage)
    if socinitflag == 0:
        soc_est_ = soc_init_
        socinitflag = 1
    else:
        correction = 1
        soc_from_voltage_and_current = DischargingBatteryVoltageToSOC(filtered_battery_voltage, abs(battery_current_), index)
        # soc_from_voltage_and_current = soc_voltage
        if (charging_status == 0):
            # this is the heuristic correction between coulomb counting SOC and
            # voltage current based SOC. The coulomb counting SOC will try to
            # "follow" the SOC estimated by voltage and current.
            if(soc_from_voltage_and_current > 0):
                correction2 = pow(soc_est_ / soc_from_voltage_and_current, 2)
                soc_error=(soc_from_voltage_and_current-soc_est_)
                # correction=pow(0.001,soc_error-0.1)
                if soc_est_ !=0:
                    correction=pow((soc_from_voltage_and_current-0.03)/soc_est_,-6)
                    if correction>MAX_CORRECTION:
                        correction=MAX_CORRECTION
                if index==696519:
                    print(correction)
            else:
                correction = MAX_CORRECTION
            accumulated_coulomb+=capacity_loss_coulomb
            capacity_loss_coulomb = 0.0
        if charging_status == 1:
            if battery_current_ > 0 and battery_current_ <= 0.25:
                IsDetachedBatteryFromCharger = True
        else:
            IsDetachedBatteryFromCharger = False
        if(not IsDetachedBatteryFromCharger and soc_est_>0):
            accumulated_coulomb += correction * battery_current_ * duration_in_secs
        soc_est_ = soc_init_ + (accumulated_coulomb +capacity_loss_coulomb) / (3600.0 * Q_rated)
        if soc_est_ < 0.0:
            soc_est_ = 0.0
        elif soc_est_ > FULLY_CHARGED_SOC_LATCH:
            soc_est_ = FULLY_CHARGED_SOC_LATCH
        if(IsDetachedBatteryFromCharger and soc_est_ < 1.0):
            capacity_loss_coulomb += correction * max_battery_charge_current * duration_in_secs
        if(IsDetachedBatteryFromCharger and soc_est_ >= 1.0):
            soc_init_ = FULLY_CHARGED_SOC_LATCH
            accumulated_coulomb = 0.0
            capacity_loss_coulomb = 0.0
        last_current = battery_current_
soc_init_= soc_init_from_firmare[0]
soc.append(csvdata['soc_est_'][0])
csv_soc_est=[csvdata['soc_est_'][0]]
csv_soc_VC=[soc_voltage_and_current[0]]
for i in tqdm.tqdm(range(1, len(battery_voltage_))):
    if battery_current_[i] > 10 or battery_current_[i] < -10:
        battery_current_[i] = battery_current_[i-1]
    if battery_voltage_[i] < 12 or battery_voltage_[i] > 20:
        battery_voltage_[i] = battery_voltage_[i-1]
    if duration_in_secs[i] < 0 or duration_in_secs[i] > 5:
        duration_in_secs[i] = duration_in_secs[i-1]
    if count[i]-count[i-1]<0:
        max_battery_charge_current = 0
        filtered_battery_voltage = battery_voltage_[0]
        accumulated_coulomb = 0.0
        Q_rated = 5.2
        last_current = 0
        capacity_loss_coulomb = 0
        soc_est_ = 0
        socinitflag = 0
        IsDetachedBatteryFromCharger = False
    soc_init_=soc_init_from_firmare[i]
    SocEstimation(battery_voltage_[i], battery_current_[i], duration_in_secs[i], i,soc_voltage_and_current[i])
    if csvdata['soc_est_'][i]<2 and csvdata['soc_est_'][i]>=0:
        csv_soc_est.append(csvdata['soc_est_'][i])
    else:
        csv_soc_est.append(csv_soc_est[-1])
    
    if soc_voltage_and_current[i]<2 and soc_voltage_and_current[i]>=0:
        csv_soc_VC.append(soc_voltage_and_current[i])
    else:
        csv_soc_VC.append(csv_soc_VC[-1])
    soc.append(soc_est_)




iplot([go.Scatter(y=soc), go.Scatter(y=csv_soc_est),go.Scatter(y=csv_soc_VC)])
