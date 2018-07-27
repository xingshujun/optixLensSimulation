#include <optix.h>
#include <optixu/optixu_math_namespace.h>
#include <optixu/optixu_matrix_namespace.h>
#include <optixu/optixu_aabb_namespace.h>
#include "intersection_refinement.h"
#define Pi      3.14159265358979f
#define eps     1.0e-5f

using namespace optix;

//传参列表
rtDeclareVariable(float,  radius, , );
rtDeclareVariable(float,  partialAngle, , );
rtDeclareVariable(float,  zMin, , );
rtDeclareVariable(float,  zMax, , );
rtDeclareVariable(float,  phiMax, , );
rtDeclareVariable(float,  thetaMax, , );
rtDeclareVariable(float,  thetaMin, , );


//属性列表
rtDeclareVariable(float3, geometric_normal, attribute geometric_normal, );  
rtDeclareVariable(float3, shading_normal, attribute shading_normal, );
rtDeclareVariable(float3, back_hit_point, attribute back_hit_point, );
rtDeclareVariable(float3, front_hit_point, attribute front_hit_point, );

//只读列表
rtDeclareVariable(optix::Ray, ray, rtCurrentRay, );

static __device__  float clampT(const float f, const float a, const float b)
{
	return fmaxf(a, fminf(f, b));
}
//二次曲面解析式的表达
static __device__ bool Quadratic(float a, float b, float c, float &t0, float &t1)
{
	// Find quadratic discriminant
	float discrim = b * b - 4 * a * c;
	if (discrim < 0) 
		return false;

	float rootDiscrim = sqrtf(discrim);

	// Compute quadratic _t_ values
	float q;
	if (b < 0)
		q = -.5 * (b - rootDiscrim);
	else
		q = -.5 * (b + rootDiscrim);
	t0 = q / a;
	t1 = c / q;
	float ttemp = 0;

	if (t0 > t1) 
	{
		ttemp = t0;
		t0 = t1;
		t1 = ttemp;
	}

	return true;
}

static __device__  float UpperBound(float v)
{
	float err = eps;
	return v + err;
}

static __device__  float LowerBound(float v)
{
	float err = eps;
	return v - err;
}


//参考链接：https://github.com/mmp/pbrt-v3/blob/6663b4cd4cb242ef8b923d99a6cb0b27b2fb0f37/src/shapes/sphere.cpp

RT_PROGRAM void partialsphere_intersect(int primIdx)
{
	float phi;   
	float3 dir = ray.direction;
	float3 ori = ray.origin;

	// 初始化光线的坐标值
	float tmin = ray.tmin;
	float tmax = ray.tmax;

	// 计算相应的二次方程参数
	float a = dot(dir, dir);
	float b = 2.0 * dot(dir, ori);
	float c = dot(ori, ori) - radius*radius;

	// 解二次方程求解方程组获得T值
	float t0, t1;
	if (!Quadratic(a, b, c, t0, t1)) 
		return ;

	// 检查t0,t1是否是最近的碰撞点
	if (UpperBound(t0) > tmax || LowerBound(t1) <= 0)
		return ;

	float tShapeHit = t0;
	if (UpperBound(tShapeHit) <= 0)
	{
		tShapeHit = t1;
		if (UpperBound(tShapeHit) > tmin)
			return ;
	}

	// 计算出球体的碰撞点与phi值的大小
	float3 pHit = ori + tShapeHit * dir;

	// 修正球体与光线的碰撞点
	pHit = radius / length(pHit)*pHit;

	if (pHit.x == 0 && pHit.y == 0)
		pHit.x = 1e-5f * radius;

	phi = atan2(pHit.y, pHit.x);

	if (phi < 0) 
		phi += 2 * Pi;

	// 对裁切面进行碰撞碰撞的修正
	if ((zMin > -radius && pHit.z < zMin) || (zMax < radius && pHit.z > zMax) ||phi > phiMax) 
	{
		if (tShapeHit == t1) 
			return ;
		if (UpperBound(t1) > tmax)
			return ;
		tShapeHit = t1;
		// 计算球体碰撞体点
		pHit = ori + tShapeHit * dir;

		// 修正球体碰撞点
		pHit = radius / length(pHit)*pHit;
		if (pHit.x == 0 && pHit.y == 0) pHit.x = 1e-5f * radius;
		phi = atan2(pHit.y, pHit.x);
		if (phi < 0) phi += 2 * Pi;
		if ((zMin > -radius && pHit.z < zMin) ||
			(zMax < radius && pHit.z > zMax) || phi > phiMax)
			return ;
	}

	// 使用参数表达的方式表达解析结果
	float u = phi / phiMax;
	float theta = acosf(clampT(pHit.z / radius, -1, 1));
	float v = (theta - thetaMin) / (thetaMax - thetaMin);

	if (rtPotentialIntersection(tShapeHit))
	{
		geometric_normal = shading_normal = pHit;
		if (dot(dir, geometric_normal) > 0.0f) 
		{
			back_hit_point  = offset(pHit,  geometric_normal);
			front_hit_point = offset(pHit, -geometric_normal);
		}
		else 
		{
			back_hit_point  = offset(pHit, -geometric_normal);
			front_hit_point = offset(pHit,  geometric_normal);
		}

		rtReportIntersection(0);
	}



}


RT_PROGRAM void partialsphere_bounds(int primIdx, float result[6])
{
	optix::Aabb* aabb = (optix::Aabb*)result;
	float anchor = 3.0f;
	aabb->m_min = make_float3(anchor, anchor, anchor);
	aabb->m_max = make_float3(anchor, anchor, anchor);
}
