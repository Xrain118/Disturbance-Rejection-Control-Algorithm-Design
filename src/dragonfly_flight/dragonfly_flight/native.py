"""URDF-derived parameters for the native PX4 vector control profile."""
import math
import numpy as np

def px4_parameters(model):
    cfg=model['config'];dim=model['dimensions'];est=dim['estimated']
    mass=model['mass'];inertia=np.asarray(model['inertia']);com=model['com']
    acceleration=9.81*cfg['thrust_to_weight']
    params=dict(DF_EN=1, CA_AIRFRAME=16, CA_METHOD=3,
        DF_MASS=mass, DF_RADIUS=dim['measured']['triangle_side']/math.sqrt(3),
        DF_SPACING=dim['measured']['motor_spacing'],DF_HEIGHT=est['unit_height'],
        DF_HUB=est['motor_center_height']+est['motor_height']/2+.003,
        DF_FMAX=model['max_thrust'], DF_MOMENT=cfg['moment_constant'],
        DF_TILT=cfg['tilt_limit'],DF_TILTRATE=cfg['tilt_rate'],DF_ACC=acceleration,
        DF_RATE=4/.15, MPC_USE_HTE=0,MPC_THR_HOVER=9.80665/acceleration,
        MPC_THR_MIN=.01,MPC_THR_MAX=1.,MPC_THR_XY_MARG=.3,
        MPC_XY_P=2/2.8,MPC_Z_P=3/3.2,MPC_XY_VEL_P_ACC=2.8,MPC_Z_VEL_P_ACC=3.2,
        MPC_XY_VEL_I_ACC=.3,MPC_Z_VEL_I_ACC=.4,MPC_XY_VEL_D_ACC=0.,MPC_Z_VEL_D_ACC=0.,
        MPC_XY_VEL_MAX=2.,MPC_Z_VEL_MAX_UP=1.,MPC_Z_VEL_MAX_DN=1.,
        MPC_TKO_RAMP_T=1.,MPC_TKO_SPEED=.8,
        MC_ROLL_P=2.,MC_PITCH_P=2.,MC_YAW_P=1.25,
        MC_ROLLRATE_P=.15,MC_PITCHRATE_P=.15,MC_YAWRATE_P=.15,
        MC_ROLLRATE_I=.03,MC_PITCHRATE_I=.03,MC_YAWRATE_I=.03,
        MC_ROLLRATE_D=0.,MC_PITCHRATE_D=0.,MC_YAWRATE_D=0.,
        MC_ROLLRATE_K=1.,MC_PITCHRATE_K=1.,MC_YAWRATE_K=1.,MC_BAT_SCALE_EN=0,
        THR_MDL_FAC=0,COM_RC_IN_MODE=4)
    for axis,value in zip('XYZ',com):params['DF_COM_'+axis]=float(value)
    for name,i,j in [('XX',0,0),('YY',1,1),('ZZ',2,2),('XY',0,1),('XZ',0,2),('YZ',1,2)]:
        params['DF_I'+name]=float(inertia[i,j])
    return params
