#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <math.h>
#include <tgf.h>
#include <track.h>
#include <robottools.h>
extern void LinuxSpecInit(void);
extern void ReRunRaceOnConsole(const char* raceconfig);
static int w_=0,p_=0,z_=0,s_=0; static uint8_t buf_[16];
int* pwritten=&w_; uint8_t* pdata=buf_; int* ppause=&p_; int* pzmq_flag=&z_; int* psave_flag=&s_;

static void sideInfo(tTrackSeg* s, int right){
  // walk the side segments outward: print width@start, width@end, friction, style
  tTrackSeg* q = right ? s->rside : s->lside; int n=0;
  while(q && n<3){ printf(" %c%d:%.2f/%.2f/%.2f/%s", right?'R':'L', n, q->startWidth, q->endWidth, q->surface?q->surface->kFriction:-1, q->surface?q->surface->material:"?"); q = right ? q->rside : q->lside; n++; }
  tTrackBarrier* b = s->barrier[right?TR_SIDE_RGT:TR_SIDE_LFT];
  if(b) printf(" %cB:%.2f/%.2f/%d", right?'R':'L', b->width, b->height, b->style);
}
static int dump(const char* root, const char* trackfile, double step){
  char buf[1024]; tModList* ml=0; tTrackItf itf;
  snprintf(buf,sizeof(buf),"%smodules/track/track.so",root);
  if(GfModLoad(0,buf,&ml)) { fprintf(stderr,"cannot load track module\n"); return 1; }
  ml->modInfo->fctInit(ml->modInfo->index,&itf);
  tTrack* t = itf.trkBuild(trackfile);
  printf("#TRACK name=%s length=%.3f width=%.3f nseg=%d\n", t->name, t->length, t->width, t->nseg);
  tTrackSeg* s = t->seg->next; // first segment
  printf("#SEG id type lgfromstart length radius arc startW endW fric material  sides...\n");
  for(int i=0;i<t->nseg;i++, s=s->next){
    printf("SEG %d %d %.3f %.3f %.3f %.5f %.2f %.2f %.3f %s", s->id, s->type, s->lgfromstart, s->length, s->radius, s->arc, s->startWidth, s->endWidth, s->surface->kFriction, s->surface->material);
    sideInfo(s,0); sideInfo(s,1); printf("\n");
  }
  printf("#PT dist x y z heading width segid segtype radius bankAngle slope\n");
  s = t->seg->next;
  for(int i=0;i<t->nseg;i++, s=s->next){
    int n = (int)ceil(s->length/step); if(n<1) n=1;
    for(int k=0;k<n;k++){
      double d = s->length*k/n; tTrkLocPos p; p.seg=s; p.type=TR_LPOS_MAIN; p.toMiddle=0; p.toRight=s->width/2; p.toLeft=s->width/2;
      p.toStart = (s->type==TR_STR)? d : d/s->radius;
      tdble X,Y; RtTrackLocal2Global(&p,&X,&Y,TR_TOMIDDLE);
      tdble Z = RtTrackHeightL(&p);
      tdble w = s->startWidth + (s->endWidth-s->startWidth)*d/s->length;
      tdble head = RtTrackSideTgAngleL(&p);
      t3Dd nrm; RtTrackSurfaceNormalL(&p,&nrm);
      tTrkLocPos pl=p, pr=p; pl.toRight=w; pl.toLeft=0; pl.toMiddle=w/2; pr.toRight=0; pr.toLeft=w; pr.toMiddle=-w/2;
      tdble ZL=RtTrackHeightL(&pl), ZR=RtTrackHeightL(&pr);
      printf("PT %.3f %.4f %.4f %.4f %.6f %.3f %d %d %.3f %.5f %.5f %.5f %.4f %.4f\n", s->lgfromstart+d, X, Y, Z, head, w, s->id, s->type, s->radius, nrm.x, nrm.y, nrm.z, ZL, ZR);
    }
  }
  return 0;
}
int main(int argc, char** argv){
  if(argc<4){ fprintf(stderr,"usage: simtorcs <rootdir/> <raceconfig.xml> <udp_timeout_us> | simtorcs <rootdir/> --dump <track.xml> [step]\n"); return 2; }
  SetLocalDir(argv[1]); SetLibDir(argv[1]); SetDataDir(argv[1]);
  LinuxSpecInit();
  GfInit();
  if(!strcmp(argv[2],"--dump")) return dump(argv[1], argv[3], argc>4?atof(argv[4]):1.0);
  setTimeout(atol(argv[3]));
  setVersion((char*)"2013");
  ReRunRaceOnConsole(argv[2]);
  return 0;
}
