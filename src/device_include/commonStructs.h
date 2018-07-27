#pragma once

#include <optixu/optixu_vector_types.h>

struct BasicLight
{
#if defined(__cplusplus)
  typedef optix::float3 float3;
#endif
  float3 pos;
  float3 color;
  int casts_shadow;
};

struct DirectionalLight
{
#if defined(__cplusplus)
  typedef optix::float3 float3;
#endif
  float3 direction;
  float radius;
  float3 v0;  // basis vectors for area sampling
  float3 v1; 
  float3 color;
  int casts_shadow;
};


