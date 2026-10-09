import numpy as np
def load(path='/home/claude/sim/build/track_dump.txt'):
    segs=[]; pts=[]
    for line in open(path):
        if line.startswith('SEG '):
            f=line.split(); segs.append(dict(id=int(f[1]),type=int(f[2]),lg=float(f[3]),length=float(f[4]),radius=float(f[5]),arc=float(f[6]),w0=float(f[7]),w1=float(f[8]),fric=float(f[9]),mat=f[10],sides=f[11:]))
        elif line.startswith('PT '):
            pts.append([float(v) for v in line.split()[1:]])
        elif line.startswith('#TRACK'):
            length=float(line.split('length=')[1].split()[0])
    P=np.array(pts)
    return dict(length=length, segs=segs, dist=P[:,0], x=P[:,1], y=P[:,2], z=P[:,3], head=P[:,4], width=P[:,5], segid=P[:,6].astype(int), segtype=P[:,7].astype(int), radius=P[:,8], nx=P[:,9], ny=P[:,10], nz=P[:,11])
if __name__=='__main__':
    T=load(); segs=T['segs']
    print('length',T['length'],'pts',len(T['dist']))
    # group consecutive segments of same type/radius into corners
    groups=[]; 
    for s in segs:
        key=(s['type'], round(s['radius'],1))
        if groups and groups[-1]['key']==key: groups[-1]['len']+=s['length']; groups[-1]['arc']+=s['arc']
        else: groups.append(dict(key=key,start=s['lg'],len=s['length'],arc=s['arc']))
    print('n groups',len(groups))
    for g in groups:
        t={1:'R',2:'L',3:'S'}[g['key'][0]]
        print('%7.1f %s len=%6.1f R=%6.1f arc=%5.1fdeg'%(g['start'],t,g['len'],g['key'][1],np.degrees(g['arc'])))
