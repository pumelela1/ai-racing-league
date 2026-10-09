import subprocess, time, os, sys, warnings
warnings.simplefilter('ignore')
SIM='/home/claude/sim/build/simtorcs'; RT='/home/claude/sim/rt'
sys.path.insert(0,'/home/claude/sim/drv')
_argv=sys.argv; sys.argv=[sys.argv[0]]
import snakeoil3_gym as so
sys.argv=_argv

def run(make_driver, trace=None, max_time=200.0, cfg='config/raceman/quickrace.xml', port=3001, quiet=True, delay=0, rt=None, noise=None):
    """Run one headless race. make_driver() -> object with .drive(S,R). Returns result dict."""
    rt=rt or RT
    sim=subprocess.Popen([SIM,rt+'/',os.path.join(rt,cfg),'3000000'],cwd=rt,stdout=subprocess.DEVNULL,stderr=subprocess.DEVNULL)
    time.sleep(0.25)
    out=sys.stdout
    if quiet: sys.stdout=open(os.devnull,'w')
    res=dict(lap=None,damage=0.0,finished=False,t_end=0.0,dist=0.0,top=0.0)
    try:
        _a=sys.argv; sys.argv=[_a[0]]
        try: c=so.Client(p=port)
        finally: sys.argv=_a
        drv=make_driver()
        rows=[]; queue=[]
        import random as _rnd
        rng=_rnd.Random(noise[0]) if noise else None
        while True:
            c.get_servers_input()
            if c.so is None: break
            S,R=c.S.d,c.R.d
            if 'track' not in S: continue
            info=drv.drive(S,R)
            if rng:
                R['steer']=max(-1.0,min(1.0,R['steer']+rng.gauss(0,noise[1])))
                R['accel']=max(0.0,min(1.0,R['accel']*(1+rng.gauss(0,2*noise[1]))))
                R['brake']=max(0.0,min(1.0,R['brake']*(1+rng.gauss(0,2*noise[1]))))
            if delay:
                queue.append(dict(R)); 
                if len(queue)>delay: R.update(queue.pop(0))
                else: R.update(dict(steer=0.0,accel=0.0,brake=0.0,gear=1))
            res['damage']=max(res['damage'],S.get('damage',0)); res['top']=max(res['top'],S['speedX'])
            t=S.get('curLapTime',0); res['t_end']=max(res['t_end'],t); res['dist']=S.get('distRaced',0)
            if trace is not None:
                rows.append((t,S['distFromStart'],S['speedX'],S['trackPos'],S['angle'],R['steer'],R['accel'],R['brake'],S['gear'],S['rpm'],S.get('speedY',0),S.get('damage',0))+tuple(info or ()))
            if S.get('lastLapTime',0)>0 and res['lap'] is None:
                res['lap']=S['lastLapTime']; res['finished']=True
            if t>max_time or (res['finished']):
                break
            c.respond_to_server()
        if res['lap'] is None and res['dist']>3600: res['lap']=res['t_end']; res['finished']=True
        if trace is not None: trace.extend(rows)
        res['drv']=drv
        try: c.shutdown()
        except Exception: pass
    finally:
        if quiet: sys.stdout.close(); sys.stdout=out
        sim.kill(); sim.wait()
    return res
