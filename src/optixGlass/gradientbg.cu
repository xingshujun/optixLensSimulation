

#include <optix.h>
#include <optixu/optixu_math_namespace.h>
#include "prd.h"

using namespace optix;

rtDeclareVariable(float3, background_light, , ); // horizon color
rtDeclareVariable(float3, background_dark, , );  // zenith color
rtDeclareVariable(float3, up, , );               // global up vector

rtDeclareVariable(optix::Ray, ray, rtCurrentRay, );
rtDeclareVariable(PerRayData_radiance, prd_radiance, rtPayload, );

// -----------------------------------------------------------------------------

RT_PROGRAM void miss()
{
  const float t = max(dot(ray.direction, up), 0.0f);
  const float3 result = lerp(background_light, background_dark, t);

  prd_radiance.radiance = result;
  prd_radiance.done = true;
}
