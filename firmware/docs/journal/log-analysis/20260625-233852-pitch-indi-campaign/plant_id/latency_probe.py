import os,sys,time,csv,statistics as st
HD=os.path.expanduser("~/Documents/Drone/stack/vayu/navigator/headless-sdk")
sys.path.insert(0,HD); sys.path.insert(0,os.path.join(HD,"examples"))
from vayu_headless import SitlSession
from _common import arm_on_rig
SP=os.path.expanduser("~/Documents/Drone/stack/vayu/docs/journal/log-analysis/20260625-233852-pitch-indi-campaign/sim_parity")
tag=sys.argv[1]; pr=[]
with SitlSession(rig=True, conf=os.path.join(SP,"ring.conf")) as s:
    arm_on_rig(s,thr=0.4); t0=time.time()
    while time.time()-t0<7:
        s.stick(thr=0.4,pitch=0,roll=0); s.set_rc(swa=2000)
        tr=s.truth()
        if tr: pr.append(tr['omega'][1]*57.2958)
        time.sleep(0.006)
pr=[x for x in pr if x==x]
print("%s: peak|prate|=%.0f dps  RMS=%.0f"%(tag,max(abs(x) for x in pr),st.pstdev(pr)))
