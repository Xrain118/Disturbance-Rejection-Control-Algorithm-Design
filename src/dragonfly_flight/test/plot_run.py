"""Plot recorded simulation truth, joint tracking and actuator diagnostics."""
import csv
import json
import sys
from pathlib import Path
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np

def plot(directory):
    rows=list(csv.DictReader((directory/'flight.csv').open()))
    events=json.loads((directory/'events.json').read_text()) if (directory/'events.json').exists() else {}
    def data(key):return np.array([float(r[key]) for r in rows])
    t=data('t');fig,axs=plt.subplots(4,2,figsize=(13,12),sharex=True)
    for axis in 'xyz':axs[0,0].plot(t,data('e'+axis),label=axis)
    axs[0,0].set_ylabel('Position error (m)');axs[0,0].legend(ncol=3)
    axs[0,1].plot(t,np.degrees(data('angle')));axs[0,1].set_ylabel('Attitude error (deg)')
    for index,side in ((1,'left'),(6,'right')):
        axs[1,0].plot(t,np.degrees(data('q'+str(index))),label=side)
        if 'qref1' in rows[0]:axs[1,0].plot(t,np.degrees(data('qref'+str(index))),'--',alpha=.6)
    axs[1,0].set_ylabel('Shoulder angle (deg)');axs[1,0].legend()
    if 'qref1' in rows[0]:
        for i in range(10):axs[1,1].plot(t,np.degrees(data(f'q{i}')-data(f'qref{i}')))
        axs[1,1].set_ylabel('Joint tracking error (deg)')
    else:
        for i in range(10):axs[1,1].plot(t,data(f'effort{i}'))
        axs[1,1].set_ylabel('Joint effort (Nm)')
    for i in range(6):
        axs[2,0].plot(t,data(f'actual_f{i}'),label=str(i+1))
        axs[2,1].plot(t,np.degrees(data(f'actual_tilt{i}')))
    axs[2,0].set_ylabel('Actual rotor thrust (N)');axs[2,0].legend(ncol=6)
    axs[2,1].set_ylabel('Actual tilt (deg)')
    axs[3,0].plot(t,data('force_residual'));axs[3,0].set_ylabel('Allocation force residual (N)')
    axs[3,1].plot(t,data('torque_residual'));axs[3,1].set_ylabel('Allocation torque residual (Nm)')
    for ax in axs.flat:
        ax.grid(alpha=.25)
        if 'experiment_start' in events:ax.axvspan(events['experiment_start'],events['experiment_start']+30,color='green',alpha=.05)
        if 'left_start' in events:ax.axvspan(events['left_start'],t[-1],color='orange',alpha=.07)
    for ax in axs[-1]:ax.set_xlabel('Simulation time since arming (s)')
    fig.suptitle(directory.name+' | green: fixed-arm hover; orange: arm replay')
    fig.tight_layout();fig.savefig(directory/'curves.png',dpi=140);plt.close(fig)

if __name__=='__main__':
    for value in sys.argv[1:]:plot(Path(value))
