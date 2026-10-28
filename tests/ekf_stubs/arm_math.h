#pragma once
#include <stdint.h>
#include <string.h>
#include <math.h>
typedef struct { uint16_t numRows, numCols; float *pData; } arm_matrix_instance_f32;
typedef int arm_status;
static inline void arm_mat_init_f32(arm_matrix_instance_f32 *m, uint16_t r, uint16_t c, float *p)
{ m->numRows=r; m->numCols=c; m->pData=p; }
static inline arm_status arm_mat_add_f32(const arm_matrix_instance_f32 *a, const arm_matrix_instance_f32 *b, arm_matrix_instance_f32 *o)
{ if(a->numRows!=b->numRows||a->numCols!=b->numCols)return -1;
  for(unsigned i=0;i<(unsigned)a->numRows*a->numCols;++i)o->pData[i]=a->pData[i]+b->pData[i]; return 0; }
static inline arm_status arm_mat_sub_f32(const arm_matrix_instance_f32 *a, const arm_matrix_instance_f32 *b, arm_matrix_instance_f32 *o)
{ if(a->numRows!=b->numRows||a->numCols!=b->numCols)return -1;
  for(unsigned i=0;i<(unsigned)a->numRows*a->numCols;++i)o->pData[i]=a->pData[i]-b->pData[i]; return 0; }
static inline arm_status arm_mat_mult_f32(const arm_matrix_instance_f32 *a, const arm_matrix_instance_f32 *b, arm_matrix_instance_f32 *o)
{ if(a->numCols!=b->numRows)return -1;
  for(unsigned r=0;r<a->numRows;++r)for(unsigned c=0;c<b->numCols;++c){float v=0;
    for(unsigned k=0;k<a->numCols;++k)v+=a->pData[r*a->numCols+k]*b->pData[k*b->numCols+c];
    o->pData[r*b->numCols+c]=v;} return 0; }
static inline arm_status arm_mat_trans_f32(const arm_matrix_instance_f32 *a, arm_matrix_instance_f32 *o)
{ for(unsigned r=0;r<a->numRows;++r)for(unsigned c=0;c<a->numCols;++c)o->pData[c*a->numRows+r]=a->pData[r*a->numCols+c];return 0; }
// 小矩阵浮点参考后端，非CMSIS-DSP性能/逐位等价实现。
static inline arm_status arm_mat_inverse_f32(const arm_matrix_instance_f32 *a, arm_matrix_instance_f32 *o)
{
  const unsigned n=a->numRows; if(n!=a->numCols||n>6)return -1;
  float aug[6][12]={{0}};
  for(unsigned r=0;r<n;++r)for(unsigned c=0;c<n;++c){aug[r][c]=a->pData[r*n+c];aug[r][c+n]=(r==c)?1.f:0.f;}
  for(unsigned c=0;c<n;++c){unsigned pivot=c;
    for(unsigned r=c+1;r<n;++r)if(fabsf(aug[r][c])>fabsf(aug[pivot][c]))pivot=r;
    if(fabsf(aug[pivot][c])<1e-30f)return -1;
    for(unsigned k=0;k<2*n;++k){float t=aug[c][k];aug[c][k]=aug[pivot][k];aug[pivot][k]=t;}
    float d=aug[c][c];for(unsigned k=0;k<2*n;++k)aug[c][k]/=d;
    for(unsigned r=0;r<n;++r)if(r!=c){float f=aug[r][c];for(unsigned k=0;k<2*n;++k)aug[r][k]-=f*aug[c][k];}}
  for(unsigned r=0;r<n;++r)for(unsigned c=0;c<n;++c)o->pData[r*n+c]=aug[r][c+n];return 0;
}
