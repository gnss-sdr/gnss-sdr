#define MUL_RE(a,b) (a.x*b.x - a.y*b.y)
#define MUL_IM(a,b) (a.x*b.y + a.y*b.x)
#define SUM_RE(a,b) (a.x + b.x)
#define SUM_IM(a,b) (a.y + b.y)

__kernel void add_vectors(__global const float2* src1, __global const float2* src2, __global float2* dest)
{
    int gid = get_global_id(0);
    dest[gid] = (float2)(SUM_RE(src1[gid],src2[gid]),SUM_IM(src1[gid],src2[gid]));
}

__kernel void mult_vectors(__global const float2* src1, __global const float2* src2, __global float2* dest)
{
    int gid = get_global_id(0);
    dest[gid] = (float2)(MUL_RE(src1[gid],src2[gid]),MUL_IM(src1[gid],src2[gid]));
}

__kernel void conj_vector(__global const float2* src, __global float2* dest)
{
    int gid = get_global_id(0);
    dest[gid] = ((float2)(1,-1)) * src[gid];
}

__kernel void magnitude_squared(__global const float2* src, __global float* dest)
{
    int gid = get_global_id(0);
    dest[gid] = src[gid].x*src[gid].x + src[gid].y*src[gid].y;
}

