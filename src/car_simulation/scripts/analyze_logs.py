#!/usr/bin/env python3
import pandas as pd
import numpy as np
import os

def main():
    log_dir = os.path.expanduser('~/f1tenth_ws/src/car_simulation/sim_log')

    # 1) Beolvasás egyedi timestamp-névvel
    leader = pd.read_csv(
        os.path.join(log_dir, 'leader.csv'),
        names=['t_leader','x_leader','y_leader','z_leader'],
        header=0
    )
    control = pd.read_csv(
        os.path.join(log_dir, 'control.csv'),
        names=['t_control','steer_cmd','speed_cmd'],
        header=0
    )
    real = pd.read_csv(
        os.path.join(log_dir, 'real.csv'),
        names=['t_real','x_real','y_real','z_real','yaw_real'],
        header=0
    )
    model = pd.read_csv(
        os.path.join(log_dir, 'model.csv'),
        names=['t_model','x_model','y_model','z_model','yaw_model'],
        header=0
    )

    # 2) Másodpercre váltás és rendezés
    for df, tcol in [(leader,'t_leader'),
                     (control,'t_control'),
                     (real,'t_real'),
                     (model,'t_model')]:
        df['t_s'] = df[tcol] * 1e-9
        df.sort_values('t_s', inplace=True)

    # 3) merge_asof sorban
    df = pd.merge_asof(model, real, on='t_s')
    df = pd.merge_asof(df, control, on='t_s')
    df = pd.merge_asof(df, leader, on='t_s')

    # 4) Hibák számítása
    df['pos_err'] = np.hypot(df['x_model'] - df['x_real'],
                             df['y_model'] - df['y_real'])
    # modell-sebességet a parancsokból vesszük, vagy számolhatod dx/dt-vel is
    df['model_speed'] = np.hypot(
        df['x_model'].diff() / df['t_s'].diff(),
        df['y_model'].diff() / df['t_s'].diff()
    ).fillna(method='bfill')
    df['speed_err'] = np.abs(df['model_speed'] - df['speed_cmd'])

    # 5) Statisztikák
    stats = {
        'pos_err_mean': df['pos_err'].mean(),
        'pos_err_max':  df['pos_err'].max(),
        'pos_err_rmse': np.sqrt((df['pos_err']**2).mean()),
        'speed_err_mean': df['speed_err'].mean(),
        'speed_err_max':  df['speed_err'].max(),
        'speed_err_rmse': np.sqrt((df['speed_err']**2).mean()),
    }

    stats_df = pd.DataFrame([stats])
    print('\n=== Hibastatisztikák ===')
    print(stats_df.to_string(index=False))

    # 6) Eredmény mentése
    out_csv = os.path.join(log_dir, 'error_stats.csv')
    stats_df.to_csv(out_csv, index=False)
    print(f'\nRészletes hibák elmentve: {out_csv}')

if __name__ == '__main__':
    main()
