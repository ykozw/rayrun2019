//
#include "embree3/rtcore.h"
//
#define WIN32_LEAN_AND_MEAN
#define NOMINMAX
#include <windows.h>
//
#include <cstdint>
#include <cstdio>
#define _USE_MATH_DEFINES
#include <cmath>
#include <limits>
//
#include "../rayrun/rayrun.hpp"

//
BOOL APIENTRY DllMain(HMODULE hModule,
    DWORD  ul_reason_for_call,
    LPVOID lpReserved
)
{
    switch (ul_reason_for_call)
    {
    case DLL_PROCESS_ATTACH:
    case DLL_THREAD_ATTACH:
    case DLL_THREAD_DETACH:
    case DLL_PROCESS_DETACH:
        break;
    }
    return TRUE;
}

//
class Vec3
{
public:
    Vec3() = default;
    Vec3(float x, float y, float z)
        :x_(x), y_(y), z_(z)
    {}
    Vec3(float xyz)
        :x_(xyz), y_(xyz), z_(xyz)
    {}
    Vec3(const float* ptr)
        :x_(ptr[0]), y_(ptr[1]), z_(ptr[2])
    {}
    Vec3(const Vec3& other)
        :x_(other.x_), y_(other.y_), z_(other.z_)
    {}
    float x() const { return x_; }
    float y() const { return y_; }
    float z() const { return z_; }
    //
    float lengthSq() const
    {
        return x_ * x_ + y_ * y_ + z_ * z_;
    }
    float length() const
    {
        return std::sqrt(lengthSq());
    }
    void normalize()
    {
        const float il = 1.0f / length();
        x_ *= il;
        y_ *= il;
        z_ *= il;
    }
    Vec3 normalized() const
    {
        Vec3 ret = *this;
        ret.normalize();
        return ret;
    }
    //
    Vec3 operator + (Vec3 other) const
    {
        return
            Vec3(
                x_ + other.x_,
                y_ + other.y_,
                z_ + other.z_);
    }
    Vec3 operator - (Vec3 other) const
    {
        return
            Vec3(
                x_ - other.x_,
                y_ - other.y_,
                z_ - other.z_);
    }
    Vec3 operator * (Vec3 other) const
    {
        return
            Vec3(
                x_ * other.x_,
                y_ * other.y_,
                z_ * other.z_);
    }
    //
    static float dot(Vec3 rhs, Vec3 lhs)
    {
        return 
            rhs.x_ * lhs.x_ +
            rhs.y_ * lhs.y_ +
            rhs.z_ * lhs.z_;
    }
    static Vec3 cross(Vec3 rhs, Vec3 lhs)
    {
        return
            Vec3(
                rhs.y_ * lhs.z_ - rhs.z_ * lhs.y_,
                rhs.z_ * lhs.x_ - rhs.x_ * lhs.z_,
                rhs.x_ * lhs.y_ - rhs.y_ * lhs.x_);
    }
    static float distance(Vec3 rhs, Vec3 lhs)
    {;
        return (rhs - lhs).length();
    }
private:
    float x_;
    float y_;
    float z_;
};

class OrthonormalBasis
{
public:
    OrthonormalBasis() = default;
    OrthonormalBasis(Vec3 n)
    {
        if (fabsf(n.x()) < 0.99f)
        {
            s_ = Vec3::cross(n, Vec3(1.0f, 0.0f, 0.0));
        }
        else
        {
            s_ = Vec3::cross(n, Vec3(0.0f, 1.0f, 0.0f));
        }
        s_.normalize();
        t_ = Vec3::cross(s_, n);
        n_ = n;

        //
        is_ = Vec3(s_.x(), t_.x(), n_.x());
        it_ = Vec3(s_.y(), t_.y(), n_.y());
        in_ = Vec3(s_.z(), t_.z(), n_.z());

    }
    Vec3 world2local(Vec3 world) const
    {
        return Vec3(
            Vec3::dot(world, s_),
            Vec3::dot(world, t_),
            Vec3::dot(world, n_));
    }
    Vec3 local2world(Vec3 local) const
    {
        return Vec3(
            Vec3::dot(local, is_),
            Vec3::dot(local, it_),
            Vec3::dot(local, in_));
    }

private:
    Vec3 s_;
    Vec3 t_;
    Vec3 n_;

    Vec3 is_;
    Vec3 it_;
    Vec3 in_;
};

class XorShift128
{
public:
    XorShift128(uint32_t seed)
    {
        uint32_t s = seed;
        x = s = 1812433253U * (s ^ (s >> 30)) + 0;
        y = s = 1812433253U * (s ^ (s >> 30)) + 1;
        z = s = 1812433253U * (s ^ (s >> 30)) + 2;
        w = s = 1812433253U * (s ^ (s >> 30)) + 3;
    }
    float nextFloat() const
    {
        const float s = 2.32830616e-010f;
        return (float)nextUint() * s;
    }
    uint32_t nextUint() const
    {
        uint32_t t = x ^ (x << 11);
        x = y; y = z; z = w;
        return w = w ^ (w >> 19) ^ t ^ (t >> 8);
    }
private:
    mutable uint32_t x;
    mutable uint32_t y;
    mutable uint32_t z;
    mutable uint32_t w;
};

static Vec3 getHemisphere(const XorShift128& rng)
{
    const float x = rng.nextFloat();
    const float y = rng.nextFloat();
    const float phi = 2.0f * float(M_PI) * x;
    const float sinPhi = std::sinf(phi);
    const float cosPhi = std::cosf(phi);
    const float cosTheta = 1.0f - y;
    const float sinTheta = std::sqrtf(1 - cosTheta * cosTheta);
    return Vec3(sinTheta * cosPhi, sinTheta * sinPhi, cosTheta);
}
//
void rayRun(
    const float* vertices,
    size_t numVerts,
    const uint32_t* indices,
    size_t numFace,
    Test* tests,
    size_t numTests)
{
    class Local
    {
    public:
        static void error_handler(void* userPtr, const RTCError code, const char* str)
        {
            printf("ERR:%s\n", str);
        }
    };
    // TOOD: embreeの初期化をする
    RTCDevice device = rtcNewDevice("");
    rtcSetDeviceErrorFunction(device, Local::error_handler, nullptr);
    // シーンの作成
    RTCScene scene = rtcNewScene(device);
    //
    RTCGeometry mesh = rtcNewGeometry(device, RTC_GEOMETRY_TYPE_TRIANGLE);
    struct Vertex { float x, y, z; };
    Vertex* verticesBuffer =
        (Vertex*)rtcSetNewGeometryBuffer(mesh, RTC_BUFFER_TYPE_VERTEX, 0, RTC_FORMAT_FLOAT3, sizeof(Vertex), numVerts);
    for (int32_t vi = 0; vi < numVerts; ++vi)
    {
        verticesBuffer[vi].x = *vertices; ++vertices;
        verticesBuffer[vi].y = *vertices; ++vertices;
        verticesBuffer[vi].z = *vertices; ++vertices;
    }
    struct Triangle { uint32_t v0, v1, v2; };
    Triangle* triangles = (Triangle*)rtcSetNewGeometryBuffer(
        mesh, RTC_BUFFER_TYPE_INDEX, 0, RTC_FORMAT_UINT3, sizeof(Triangle), numFace);
    for (int32_t ni = 0; ni < numFace; ++ni)
    {
        triangles[ni].v0 = *indices; ++indices;
        triangles[ni].v1 = *indices; ++indices;
        triangles[ni].v2 = *indices; ++indices;
    }
    //
    rtcSetGeometryVertexAttributeCount(mesh, 0);
    rtcCommitGeometry(mesh);
    rtcAttachGeometry(scene, mesh);
    rtcReleaseGeometry(mesh);
    rtcCommitScene(scene);
    //
    XorShift128 rng(0x123);
    //
    for (int32_t tn = 0; tn < numTests; ++tn)
    {
        const Test& test = tests[tn];
        const int32_t width = test.width;
        const int32_t height = test.height;
        const int32_t hw = width / 2;
        const float iw = 1.0f / float(width);
        const int32_t hh = height / 2;
        const float ih = 1.0f / float(height);
        const Vec3 dir = Vec3(test.dir).normalized();
        const Vec3 pos = Vec3(test.pos);
        const Vec3 right = Vec3::cross(dir, Vec3(test.up)).normalized();
        const Vec3 up = Vec3::cross(right, dir).normalized();
        const float hfovy = test.fovy * 0.5f;
        float* image = test.image;
        //
        for (int32_t y = 0; y < height; ++y)
        {
            for (int32_t x = 0; x < width; ++x)
            {
                //
                RTCIntersectContext context;
                rtcInitIntersectContext(&context);
                //
                const float xs = float(x - hw) * iw * std::tanf(hfovy*float(width) / float(height));
                const float ys = float(y - hh) * ih * std::tanf(hfovy);
                const Vec3 rd = (Vec3(ys) * up + Vec3(xs) * right + dir).normalized();
                //
                RTCRayHit rayHit = {};
                RTCRay& ray = rayHit.ray;
                ray.org_x = pos.x();
                ray.org_y = pos.y();
                ray.org_z = pos.z();
                ray.dir_x = rd.x();
                ray.dir_y = rd.y();
                ray.dir_z = rd.z();
                ray.tnear = 0.000f;
                ray.tfar = std::numeric_limits<float>::infinity();
                rayHit.hit.geomID = -1;
                rtcIntersect1(scene, &context, &rayHit);
                
                if (rayHit.hit.geomID == -1)
                {
                    continue;
                }
                Vec3 isectPos = Vec3(rayHit.ray.tfar*0.999f) * rd + pos;
                const Vec3 ng = Vec3(
                    rayHit.hit.Ng_x,
                    rayHit.hit.Ng_y,
                    rayHit.hit.Ng_z).normalized();
                const OrthonormalBasis onb(ng);
                //
                float ao = 0;
                const int32_t numSample = 512;
                const float invNumSample = 1.0f / float(numSample);
                for (int32_t sn=0;sn< numSample;++sn)
                {
                    const Vec3 wiLocal = getHemisphere(rng);
                    const Vec3 woWorld = onb.local2world(wiLocal);
                    RTCRayHit rayHit = {};
                    RTCRay& ray = rayHit.ray;
                    ray.org_x = isectPos.x();
                    ray.org_y = isectPos.y();
                    ray.org_z = isectPos.z();
                    ray.dir_x = woWorld.x();
                    ray.dir_y = woWorld.y();
                    ray.dir_z = woWorld.z();
                    ray.tnear = 0.001f;
                    ray.tfar = std::numeric_limits<float>::infinity();
                    rayHit.hit.geomID = -1;
                    rtcIntersect1(scene, &context, &rayHit);
                    ao += (rayHit.hit.geomID == -1) ? invNumSample * wiLocal.z() : 0.0f;
                }
                //
                image[x + y * width] = ao;
            }
        }
    }
}
