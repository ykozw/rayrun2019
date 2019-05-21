#pragma once
//
#include <cstdint>
//　レイ
struct alignas(16) Ray
{
public:
    //
    bool valid;
    // レイ原点
    float pos[3];
    // レイ方向
    float dir[3];
    // 交差判定を開始するt
    float tnear;
    // 交差判定を終了するt
    float tfar;
    // 交差したか
    bool isisect;
    // 交差点
    float isect[3];
    // 交差点のシェーディング法線
    float ns[3];
    // 交差点のfaceid
    int32_t faceid;
    // -----------------------------------
    // 以下テストベット側で使用する変数
    float reserve[16];
};
static_assert(sizeof(Ray) == 144);

// メッシュの生成
extern "C" __declspec(dllexport) void preprocess(
    // 頂点座標配列
    const float* vertices,
    // 頂点数
    size_t numVerts,
    // 法線配列
    const float* normals,
    // 法線数
    size_t numNormals,
    // 頂点インデックス。(v0,n0,v1,n1...)のように格納されている。
    // またTriangulationにすでに行われているものとする
    const uint32_t* indices,
    // 頂点インデックス数
    size_t numFace);

// 交差判定
extern "C" __declspec(dllexport) void intersect(
    // レイ配列
    Ray* rays,
    // レイ数
    size_t numRay,
    // 交差が一つ以上あることが確定した段階で戻るか
    bool hitany);
