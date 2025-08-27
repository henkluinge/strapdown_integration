import numpy as np
import pandas as pd

def save_to_reference_log_simple_strapdown(t,y_imu, q_sdi_ls, v_ref_l):
    """
    For comparison with the C counterpart
    """
    df_imu = [pd.DataFrame(y_imu[sensor_type], index=t, columns=[f'{sensor_type}_{i}' for i in ('x', 'y', 'z')]) for sensor_type in ('y_gyr', 'y_acc')	]
    df_q = pd.DataFrame(q_sdi_ls, index=t, columns=[f'q_{i}' for i in range(4)]) 
    df_v = pd.DataFrame(v_ref_l, index=t, columns=[f'v_{i}' for i in range(3)]) 

    df_strapdown = pd.concat(df_imu + [df_q, df_v], axis=1) #.to_csv('./tests/imu_signals.csv', index=True, float_format='%.16f')
    df_strapdown.index.name = 'time'
    df_strapdown.to_csv('../tests/simple_strapdown_reference.csv', index=True, float_format='%.16f')
    return None


def get_odr_indices(t, t_odr):
    """
    Returns the indices of the ODR samples in the original signal.
    """
    i_odr = np.asarray([list(t).index(ti) for ti in t if ti in t_odr])
    return i_odr


def sample_to_odr(q_ref_ls, t, t_odr):
    q_ref_ls_odr = np.asarray([q_ref_ls[i] for i in get_odr_indices(t, t_odr)])
    return q_ref_ls_odr


def extract_values_from_movement(movement):
    omega = movement['angvel']
    q_ref_ls = movement['q_ls']
    v_ref_l = movement['vel']
    t = movement['time']
    T = movement['time'][1] - movement['time'][0]
    nt = omega.shape[0]
    return omega, q_ref_ls, v_ref_l, t, T, nt