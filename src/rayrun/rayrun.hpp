#pragma once
//
#include <cstdint>
//
struct Shot
{
    float pos[3];
    float dir[3];
    float up[3];
    int32_t width;
    int32_t height;
    float fovy;
    float* image;
};
//
typedef void(*RayRunFun)(
    const float* vertices,
    size_t numVerts,
    uint32_t* indices,
    size_t numFace,
    Shot* shots,
    size_t numShots);
//
extern "C" __declspec(dllexport) void rayRun(
    const float* vertices,
    size_t numVerts,
    const uint32_t* indices,
    size_t numFace,
    Shot* shots,
    size_t numShots);
