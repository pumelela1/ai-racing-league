/* Minimal re-implementation of the few PLIB sg functions used by TORCS physics. */
#ifndef STUB_SG_H
#define STUB_SG_H
#include <math.h>
typedef float SGfloat;
typedef SGfloat sgVec2[2];
typedef SGfloat sgVec3[3];
typedef SGfloat sgVec4[4];
typedef SGfloat sgMat4[4][4];
#define SG_ZERO 0.0f
#define SG_ONE 1.0f
#define SG_DEGREES_TO_RADIANS 0.0174532925f
#define SG_RADIANS_TO_DEGREES 57.2957795f
inline SGfloat sgSin(SGfloat s){ return (SGfloat)sin(s*SG_DEGREES_TO_RADIANS); }
inline SGfloat sgCos(SGfloat s){ return (SGfloat)cos(s*SG_DEGREES_TO_RADIANS); }
inline void sgCopyVec2(sgVec2 dst,const sgVec2 src){ dst[0]=src[0]; dst[1]=src[1]; }
inline void sgAddVec2(sgVec2 dst,const sgVec2 src){ dst[0]+=src[0]; dst[1]+=src[1]; }
inline void sgAddVec2(sgVec2 dst,const sgVec2 a,const sgVec2 b){ dst[0]=a[0]+b[0]; dst[1]=a[1]+b[1]; }
inline void sgSubVec2(sgVec2 dst,const sgVec2 src){ dst[0]-=src[0]; dst[1]-=src[1]; }
inline void sgSubVec2(sgVec2 dst,const sgVec2 a,const sgVec2 b){ dst[0]=a[0]-b[0]; dst[1]=a[1]-b[1]; }
inline void sgScaleVec2(sgVec2 dst,const SGfloat s){ dst[0]*=s; dst[1]*=s; }
inline void sgScaleVec2(sgVec2 dst,const sgVec2 src,const SGfloat s){ dst[0]=src[0]*s; dst[1]=src[1]*s; }
inline SGfloat sgScalarProductVec2(const sgVec2 a,const sgVec2 b){ return a[0]*b[0]+a[1]*b[1]; }
inline SGfloat sgLengthVec2(const sgVec2 s){ return (SGfloat)sqrt(sgScalarProductVec2(s,s)); }
inline void sgNormaliseVec2(sgVec2 dst){ sgScaleVec2(dst, SG_ONE/sgLengthVec2(dst)); }
inline void sgNormaliseVec2(sgVec2 dst,const sgVec2 src){ sgScaleVec2(dst, src, SG_ONE/sgLengthVec2(src)); }
inline void sgMakeCoordMat4(sgMat4 m,const SGfloat x,const SGfloat y,const SGfloat z,const SGfloat h,const SGfloat p,const SGfloat r)
{
  SGfloat ch,sh,cp,sp,cr,sr,srsp,crsp,srcp;
  if(h==SG_ZERO){ch=SG_ONE;sh=SG_ZERO;}else{sh=sgSin(h);ch=sgCos(h);}
  if(p==SG_ZERO){cp=SG_ONE;sp=SG_ZERO;}else{sp=sgSin(p);cp=sgCos(p);}
  if(r==SG_ZERO){cr=SG_ONE;sr=SG_ZERO;srsp=SG_ZERO;srcp=SG_ZERO;crsp=sp;}
  else{sr=sgSin(r);cr=sgCos(r);srsp=sr*sp;crsp=cr*sp;srcp=sr*cp;}
  m[0][0]=(SGfloat)( ch*cr - sh*srsp); m[1][0]=(SGfloat)(-sh*cp); m[2][0]=(SGfloat)( sr*ch + sh*crsp); m[3][0]=x;
  m[0][1]=(SGfloat)( cr*sh + srsp*ch); m[1][1]=(SGfloat)( ch*cp); m[2][1]=(SGfloat)( sr*sh - crsp*ch); m[3][1]=y;
  m[0][2]=(SGfloat)(-srcp);            m[1][2]=(SGfloat)( sp);    m[2][2]=(SGfloat)( cr*cp);            m[3][2]=z;
  m[0][3]=SG_ZERO; m[1][3]=SG_ZERO; m[2][3]=SG_ZERO; m[3][3]=SG_ONE;
}
inline void sgFullXformPnt3(sgVec3 dst,const sgVec3 src,const sgMat4 mat)
{
  SGfloat t0=src[0],t1=src[1],t2=src[2];
  dst[0]=t0*mat[0][0]+t1*mat[1][0]+t2*mat[2][0]+mat[3][0];
  dst[1]=t0*mat[0][1]+t1*mat[1][1]+t2*mat[2][1]+mat[3][1];
  dst[2]=t0*mat[0][2]+t1*mat[1][2]+t2*mat[2][2]+mat[3][2];
}
inline void sgFullXformPnt3(sgVec3 dst,const sgMat4 mat){ sgVec3 t={dst[0],dst[1],dst[2]}; sgFullXformPnt3(dst,t,mat); }
#endif
