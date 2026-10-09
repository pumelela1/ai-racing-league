import sys, numpy as np
sys.path.insert(0,'/home/claude/sim/work')
from exp import *
def write_line(path,o,prm,note=''):
    D,v=make_data(o,prm)
    def arr(name,a,fmt):
        vals=[fmt%x for x in a]; lines=[', '.join(vals[i:i+12]) for i in range(0,len(vals),12)]
        return '%s = [\n    %s]\n'%(name,',\n    '.join(lines))
    with open(path,'w') as f:
        f.write('"""Racing line and speed plan for the TORCS Corkscrew track (auto-generated).\n\nOne entry per %.4f m of track centre line, starting at the start/finish line.\n  CX, CY     centre line position (m)\n  HEADING    centre line direction (rad)\n  OFFSET     racing line, metres to the left of the centre line\n  SPEED      planned speed on the racing line (m/s)\n  CURVATURE  racing line curvature (1/m, left positive)\n  ACCEL      planned acceleration (m/s^2)\n%s"""\n'%(D['ds'],note))
        f.write('DS = %.6f\nHALF_WIDTH = 6.0\n'%D['ds'])
        f.write(arr('CX',D['cx'],'%.3f')); f.write(arr('CY',D['cy'],'%.3f')); f.write(arr('HEADING',D['hd'],'%.5f'))
        f.write(arr('OFFSET',D['off'],'%.3f')); f.write(arr('SPEED',D['v'],'%.2f')); f.write(arr('CURVATURE',D['k'],'%.6f')); f.write(arr('ACCEL',D['ax'],'%.2f'))
    return gen.laptime(C,o,v)
if __name__=='__main__':
    o=np.load('/home/claude/sim/build/o_base.npy')
    print('plan lap', write_line('/home/claude/sim/pkg/corkscrew_line.py',o,dict(BASE,uy=0.90,ub=0.65)))
