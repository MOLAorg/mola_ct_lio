"""Score a grand-tour trajectory the way this dataset's own benchmark requires.

Its ground truth is anchored to a physical sensor mount, not to `base`, so the
estimate has to be carried out to that mount first:

    T_world_sensor(t) = T_world_base(t) . T_base_sensor

The lever arm is rotated into the world frame by the estimate's own
orientation at each timestamp. A constant shift would not do: alignment
absorbs that, while a legged robot pitching under a 0.29 m arm does not.
"""
import sys, os, subprocess
import numpy as np

def quat_to_R(q):  # q = (qx,qy,qz,qw)
    x, y, z, w = q
    return np.array([
        [1-2*(y*y+z*z), 2*(x*y-z*w),   2*(x*z+y*w)],
        [2*(x*y+z*w),   1-2*(x*x+z*z), 2*(y*z-x*w)],
        [2*(x*z-y*w),   2*(y*z+x*w),   1-2*(x*x+y*y)]])

def R_to_quat(R):
    t = np.trace(R)
    if t > 0:
        s = np.sqrt(t+1.0)*2; w = 0.25*s
        x = (R[2,1]-R[1,2])/s; y = (R[0,2]-R[2,0])/s; z = (R[1,0]-R[0,1])/s
    elif R[0,0] > R[1,1] and R[0,0] > R[2,2]:
        s = np.sqrt(1.0+R[0,0]-R[1,1]-R[2,2])*2; w = (R[2,1]-R[1,2])/s
        x = 0.25*s; y = (R[0,1]+R[1,0])/s; z = (R[0,2]+R[2,0])/s
    elif R[1,1] > R[2,2]:
        s = np.sqrt(1.0+R[1,1]-R[0,0]-R[2,2])*2; w = (R[0,2]-R[2,0])/s
        x = (R[0,1]+R[1,0])/s; y = 0.25*s; z = (R[1,2]+R[2,1])/s
    else:
        s = np.sqrt(1.0+R[2,2]-R[0,0]-R[1,1])*2; w = (R[1,0]-R[0,1])/s
        x = (R[0,2]+R[2,0])/s; y = (R[1,2]+R[2,1])/s; z = 0.25*s
    return np.array([x,y,z,w])

def ypr_to_R(yaw, pitch, roll):  # degrees, MRPT order Rz.Ry.Rx
    y, p, r = np.radians([yaw, pitch, roll])
    Rz = np.array([[np.cos(y),-np.sin(y),0],[np.sin(y),np.cos(y),0],[0,0,1]])
    Ry = np.array([[np.cos(p),0,np.sin(p)],[0,1,0],[-np.sin(p),0,np.cos(p)]])
    Rx = np.array([[1,0,0],[0,np.cos(r),-np.sin(r)],[0,np.sin(r),np.cos(r)]])
    return Rz @ Ry @ Rx

# base -> cpt7_imu, from the missions' own /tf_static
OFF_T = np.array([0.0764, -0.0361, 0.2803])
OFF_R = ypr_to_R(0.0, -0.0, 179.9954)

def compose(estf, outf):
    d = np.loadtxt(estf)
    out = np.empty_like(d)
    out[:,0] = d[:,0]
    for i in range(len(d)):
        R = quat_to_R(d[i,4:8])
        out[i,1:4] = d[i,1:4] + R @ OFF_T
        out[i,4:8] = R_to_quat(R @ OFF_R)
    np.savetxt(outf, out, fmt="%.9f")

if __name__ == "__main__":
    gtf, estf, label = sys.argv[1], sys.argv[2], sys.argv[3]
    if not os.path.exists(estf):
        print("  %-26s MISSING" % label); sys.exit(0)
    tmp = estf + ".body.tum"
    compose(estf, tmp)
    for tag, f in (("raw ", estf), ("body", tmp)):
        o = subprocess.run(["evo_ape","tum",gtf,f,"--align","--t_max_diff","0.05"],
                           capture_output=True, text=True).stdout
        r = [l.split()[-1] for l in o.splitlines() if l.strip().startswith("rmse")]
        print("  %-26s %s  APE %s" % (label, tag, r[0] if r else "?"))
