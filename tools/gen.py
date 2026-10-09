"""Offline tools: racing line + speed profile for the Corkscrew track (uses numpy/scipy)."""
import numpy as np, scipy.sparse as sp
from scipy.optimize import lsq_linear
import trk
G=9.81

def resample(T):
    L=T['length']; N=int(round(L)); ds=L/N; s=np.arange(N)*ds
    de=np.append(T['dist'],L)
    itp=lambda a: np.interp(s,de,np.append(a,a[0]))
    h=np.unwrap(T['head']); k=round((h[-1]-h[0])/(2*np.pi)); hc=np.append(h,h[0]+2*np.pi*k)
    return dict(N=N,ds=ds,s=s,L=L,cx=itp(T['x']),cy=itp(T['y']),z=itp(T['z']),hd=np.interp(s,de,hc),w=itp(T['width']))

def smooth(a,win):
    if win<=1: return a
    k=np.ones(win)/win; n=len(a)
    return np.convolve(np.concatenate([a[-win:],a,a[:win]]),k,mode='same')[win:win+n]

def racing_line(C,lo,hi,len_w=0.0):
    """Minimum-curvature line: offsets o (left positive) with lo<=o<=hi."""
    N=C['N']; nx=-np.sin(C['hd']); ny=np.cos(C['hd'])
    I=np.arange(N)
    D2=sp.csr_matrix((np.concatenate([np.ones(N),-2*np.ones(N),np.ones(N)]),(np.concatenate([I,I,I]),np.concatenate([(I-1)%N,I,(I+1)%N]))),shape=(N,N))
    A=sp.vstack([D2@sp.diags(nx),D2@sp.diags(ny)]).tocsr(); b=-np.concatenate([D2@C['cx'],D2@C['cy']])
    if len_w>0:
        D1=sp.csr_matrix((np.concatenate([-np.ones(N),np.ones(N)]),(np.concatenate([I,I]),np.concatenate([I,(I+1)%N]))),shape=(N,N))
        A=sp.vstack([A,len_w*(D1@sp.diags(nx)),len_w*(D1@sp.diags(ny))]).tocsr(); b=np.concatenate([b,-len_w*(D1@C['cx']),-len_w*(D1@C['cy'])])
    r=lsq_linear(A,b,bounds=(lo,hi),lsmr_tol='auto',max_iter=200,verbose=0)
    return r.x

def line_geometry(C,o):
    nx=-np.sin(C['hd']); ny=np.cos(C['hd'])
    px=C['cx']+nx*o; py=C['cy']+ny*o
    dx=np.roll(px,-1)-px; dy=np.roll(py,-1)-py; dl=np.hypot(dx,dy)
    th=np.arctan2(dy,dx)                         # heading of chord i -> i+1
    dth=(th-np.roll(th,1)+np.pi)%(2*np.pi)-np.pi # turn at point i
    kap=dth/(0.5*(dl+np.roll(dl,1)))
    return px,py,dl,kap

MU0=1.6*1.1          # tyre mu x road friction
def load_factor(n):
    """TORCS tyre load sensitivity: n = wheel load / static load."""
    return 0.8+0.8*np.exp(-1.386294*np.maximum(n,0.0)/1.2)
def grip(v,kz,nz,ca):
    """Maximum tyre acceleration (m/s^2) at speed v: mu(load) * normal acceleration."""
    an=np.maximum(G*nz+v*v*(ca+kz),0.0)
    return MU0*load_factor(an/G)*an

def speed_profile(C,o,prm,vmax=95.0):
    px,py,dl,kap=line_geometry(C,o)
    kap=smooth(kap,prm.get('ksm',5))
    z=smooth(C['z'],prm.get('zsm',15)); N=C['N']
    slope=(np.roll(z,-1)-np.roll(z,1))/(2*C['ds'])
    kz=smooth((np.roll(z,-1)-2*z+np.roll(z,1))/C['ds']**2,prm.get('zsm',15))*prm.get('kz_w',1.0)
    nz=1.0/np.sqrt(1.0+slope*slope)
    bank=C.get('bank',np.zeros(N))*prm.get('bank_w',1.0)
    uy=prm['uy']; ub=prm['ub']; ca=prm['ca']; cd=prm['cd']; cal=prm.get('ca_lat',prm['ca'])
    ak=np.abs(kap); sgn=np.sign(kap)
    def aymax(v,i=slice(None)):
        return np.maximum(uy*grip(v,kz[i],nz[i],cal)+sgn[i]*G*bank[i],0.5)
    # lateral limit by bisection on v
    lo=np.full(N,1.0); hi=np.full(N,vmax)
    ok_hi=(aymax(hi)-hi*hi*ak)>=0
    for _ in range(40):
        mid=0.5*(lo+hi); ok=(aymax(mid)-mid*mid*ak)>=0
        lo=np.where(ok,mid,lo); hi=np.where(ok,hi,mid)
    vlat=np.where(ok_hi,vmax,lo)
    crest=-(kz+ca); vlat=np.minimum(vlat,np.where(crest>1e-6,np.sqrt(0.65*G/np.maximum(crest,1e-6)),vmax))
    v=np.minimum(vlat,vmax).copy()
    def acc(vv):
        return min(prm['a_trac0']+prm['a_trac2']*vv*vv, prm['p_m']/max(vv,5.0)) - cd*vv*vv
    for _ in range(2):
        for i in range(N):
            j=(i+1)%N; vv=v[i]
            fr=min(1.0,(vv*vv*ak[i])/float(aymax(vv,i))); ax=acc(vv)*np.sqrt(max(0.0,1-fr*fr))-G*slope[i]
            v[j]=min(v[j],np.sqrt(max(vv*vv+2*ax*dl[i],1.0)))
        for i in range(N-1,-1,-1):
            j=(i+1)%N; vv=v[j]
            fr=min(1.0,(vv*vv*ak[j])/float(aymax(vv,j))); ab=ub*float(grip(vv,kz[j],nz[j],ca))*np.sqrt(max(0.0,1-fr*fr))+cd*vv*vv+G*slope[j]
            v[i]=min(v[i],np.sqrt(vv*vv+2*max(ab,1.0)*dl[i]))
    ax=(np.roll(v,-1)**2-v**2)/(2*dl)
    return v,kap,kz,vlat,ax

def laptime(C,o,v):
    px,py,dl,kap=line_geometry(C,o)
    return float(np.sum(dl/(0.5*(v+np.roll(v,-1)))))

def racing_line_fast(C,lo,hi,K=4,extra=None):
    """Minimum-curvature line with offsets as a periodic cubic B-spline (control point every K metres).
    extra: optional per-point weights on curvature (emphasise parts of the lap)."""
    N=C['N']; M=N//K; nx=-np.sin(C['hd']); ny=np.cos(C['hd'])
    I=np.arange(N); t=I*(M/N); k=np.floor(t).astype(int); f=t-k
    w=[(1-f)**3/6,(3*f**3-6*f**2+4)/6,(-3*f**3+3*f**2+3*f+1)/6,f**3/6]
    B=sp.csr_matrix((np.concatenate(w),(np.concatenate([I]*4),np.concatenate([(k-1)%M,k%M,(k+1)%M,(k+2)%M]))),shape=(N,M))
    D2=sp.csr_matrix((np.concatenate([np.ones(N),-2*np.ones(N),np.ones(N)]),(np.concatenate([I,I,I]),np.concatenate([(I-1)%N,I,(I+1)%N]))),shape=(N,N))
    W=sp.diags(extra) if extra is not None else sp.identity(N)
    A=sp.vstack([W@D2@sp.diags(nx)@B,W@D2@sp.diags(ny)@B]).tocsr(); b=-np.concatenate([W@(D2@C['cx']),W@(D2@C['cy'])])
    # conservative control-point bounds: tightest bound over each control point's support
    span=2*K
    ext=lambda a: np.concatenate([a[-span:],a,a[:span]])
    hi_u=np.array([ext(hi)[j*N//M:j*N//M+2*span+1].min() for j in range(M)]); lo_u=np.array([ext(lo)[j*N//M:j*N//M+2*span+1].max() for j in range(M)])
    r=lsq_linear(A,b,bounds=(lo_u,hi_u),lsmr_tol='auto',max_iter=300,verbose=0)
    return B@r.x
