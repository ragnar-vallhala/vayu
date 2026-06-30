import os,sys,time,csv
HD=os.path.expanduser("~/Documents/Drone/stack/vayu/navigator/headless-sdk")
sys.path.insert(0,HD); sys.path.insert(0,os.path.join(HD,"examples"))
from vayu_headless import SitlSession, Pilot
SP=os.path.expanduser("~/Documents/Drone/stack/vayu/docs/journal/log-analysis/20260625-233852-pitch-indi-campaign/sim_parity")
tag=sys.argv[1]; rows=[]
with SitlSession(rig=False, conf=os.path.join(SP,os.environ.get("RINGCONF","ring.conf"))) as s:
    p=Pilot(s,alt=-3.0)
    try:p.arm_takeoff()
    except Exception as e:print('to',e)
    t0=time.time()
    while time.time()-t0<10:
        tr=s.truth(); ct=s.telem.get('ControlTrace')
        if tr and ct: rows.append((time.time()-t0,tr['omega'][1]*57.2958,getattr(ct,'thro_out',0)))
        time.sleep(0.008)
    try:p.land()
    except Exception:pass
with open(os.path.join(SP,f"ff_{tag}.csv"),'w',newline='') as f:
    w=csv.writer(f); w.writerow(['t','prate','thr']); w.writerows(rows)
print(f"ff_{tag}: {len(rows)} rows")
