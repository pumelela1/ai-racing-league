import sys, json, numpy as np, time
sys.path.insert(0,'/home/claude/sim/work')
from ev import *
x=json.load(open('/home/claude/sim/build/opt2_base.json'))['x']; prm,P=split(x)
B=float(sys.argv[1]); K=int(sys.argv[2]); p=float(sys.argv[3]); tag=sys.argv[4]
lo=-B*np.ones(N); hi=B*np.ones(N)
o0=np.load('/home/claude/sim/build/o_fast43.npy'); v0=gen.speed_profile(C,o0,prm)[0]
w=gen.smooth((v0.mean()/v0)**p,9); t=time.time()
o=gen.racing_line_fast(C,lo,hi,K=K,extra=w); np.save('/home/claude/sim/build/o_%s.npy'%tag,o); np.save('/home/claude/sim/build/lo_%s.npy'%tag,lo); np.save('/home/claude/sim/build/hi_%s.npy'%tag,hi)
v=gen.speed_profile(C,o,prm)[0]; print(tag,'bound %.1f K=%d p=%.1f: plan %.2f min %.1f km/h solve %.0fs'%(B,K,p,gen.laptime(C,o,v),v.min()*3.6,time.time()-t)); sys.stdout.flush()
a=run1(o,x,lo,hi); b=run1(o,x,lo,hi,rt=LOW); print(' sim: '+fmt(a)+' | low: '+fmt(b))
