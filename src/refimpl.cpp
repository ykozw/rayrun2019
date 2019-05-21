//
#define WIN32_LEAN_AND_MEAN
#define NOMINMAX
#include <windows.h>
//
#include <cstdint>
#include <cstdio>
#include <limits>
#include <vector>
#include <array>
//
#include "rayrun.hpp"

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
    Vec3 operator / (Vec3 other) const
    {
        return
            Vec3(
                x_ / other.x_,
                y_ / other.y_,
                z_ / other.z_);
    }
    const float& operator[](int32_t index) const
    {
        return *(&x_ + index);
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
    {
        ;
        return (rhs - lhs).length();
    }
    static Vec3 min(Vec3 lhs, Vec3 rhs)
    {
        const float x = std::min(lhs.x(), rhs.x());
        const float y = std::min(lhs.y(), rhs.y());
        const float z = std::min(lhs.z(), rhs.z());
        return Vec3(x, y, z);
    }
    static Vec3 max(Vec3 lhs, Vec3 rhs)
    {
        const float x = std::max(lhs.x(), rhs.x());
        const float y = std::max(lhs.y(), rhs.y());
        const float z = std::max(lhs.z(), rhs.z());
        return Vec3(x, y, z);
    }
private:
    float x_;
    float y_;
    float z_;
};

//
struct RayExt
{
public:
    Vec3 pos;
    Vec3 dir;
    float tnear;
    float tfar;
    Vec3 dinv;
    std::array<bool, 3> sign;
    // 交差点
    Vec3 isect;
    //
    float u;
    float v;
    //
    Vec3 ns;
    //
    int32_t faceid;
};

//
class AABB
{
public:
    AABB()
    {
        clear();
    }
    void clear()
    {
        const float maxv = std::numeric_limits<float>::max();
        const float minv = std::numeric_limits<float>::lowest();
        mn = Vec3(maxv, maxv, maxv);
        mx = Vec3(minv, minv, minv);
    }
    void addPoint(Vec3 point)
    {
        mn = Vec3::min(point, mn);
        mx = Vec3::max(point, mx);
    }
    Vec3 center() const
    {
        return (mn + mx) * 0.5f;
    }
    Vec3 min() const
    {
        return mn;
    }
    Vec3 max() const
    {
        return mx;
    }
    Vec3 size() const
    {
        return max() - min();
    }
    void addAABB(const AABB& aabb)
    {
        mn = Vec3::min(aabb.mn, mn);
        mx = Vec3::max(aabb.mx, mx);
    }
    const Vec3& AABB::operator[](int32_t index) const
    {
        return *(&mn + index);
    }
    bool intersectCheck(const RayExt& ray, float currentIntersectT) const
    {
        //
        const AABB& aabb = *this;
        float tmin, tmax, tymin, tymax, tzmin, tzmax;
        tmin = (aabb[ray.sign[0]].x() - ray.pos.x()) * ray.dinv.x();
        tmax = (aabb[1 - ray.sign[0]].x() - ray.pos.x()) * ray.dinv.x();
        tymin = (aabb[ray.sign[1]].y() - ray.pos.y()) * ray.dinv.y();
        tymax = (aabb[1 - ray.sign[1]].y() - ray.pos.y()) * ray.dinv.y();
        if ((tmin > tymax) || (tymin > tmax))
        {
            return false;
        }
        if (tymin > tmin)
        {
            tmin = tymin;
        }
        if (tymax < tmax)
        {
            tmax = tymax;
        }
        tzmin = (aabb[ray.sign[2]].z() - ray.pos.z()) * ray.dinv.z();
        tzmax = (aabb[1 - ray.sign[2]].z() - ray.pos.z()) * ray.dinv.z();
        if ((tmin > tzmax) || (tzmin > tmax))
        {
            return false;
        }
        if (tzmin > tmin)
        {
            tmin = tzmin;
        }
        if (tzmax < tmax)
        {
            tmax = tzmax;
        }
        return (tmin < currentIntersectT) && (ray.tnear < tmax) && (tmin < ray.tfar);
    }


private:
    Vec3 mn;
    Vec3 mx;
};

/*
-------------------------------------------------
-------------------------------------------------
*/
bool intersectTriangle(
    RayExt& ray,
    Vec3 v0,
    Vec3 v1,
    Vec3 v2)
{
    //
    float t, u, v;
    //
    Vec3 e1, e2;
    Vec3 P, Q, T;
    float det, inv_det;
    // v0を共有するエッジ
    e1 = v1 - v0;
    e2 = v2 - v0;
    // detとuの準備
    P = Vec3::cross(ray.dir, e2);
    // ほぼ平行な場合かをチェック
    det = Vec3::dot(e1, P);
    if (det == 0.0f)
    {
        return false;
    }
    inv_det = 1.0f / det;
    // レイ原点からv0への距離
    T = ray.pos - v0;
    // uを計算し、範囲内に収まっているかをチェック
    u = Vec3::dot(T, P) * inv_det;
    if (u < 0.0f || u > 1.0f)
    {
        return false;
    }
    // vも同様の計算
    Q = Vec3::cross(T, e1);
    v = Vec3::dot(ray.dir, Q) * inv_det;
    if (v < 0.0f || u + v  > 1.0f)
    {
        return false;
    }
    // tの範囲チェック
    t = Vec3::dot(e2, Q) * inv_det;
    if (t < ray.tnear || ray.tfar < t)
    {
        return false;
    }
    // 面の方向
    //const bool isFlip = (det < 0.0f);
    //
    if (t >= ray.tfar)
    {
        return false;
    }
    //
    ray.tfar = t;
    ray.u = u;
    ray.v = v;
    return true;
}

//
struct Face
{
public:
    std::array<uint32_t, 3> idxVtx;
    std::array<uint32_t, 3> idxNorm;
};

//
class SimpleBVH
{
public:
    SimpleBVH()
    {}
    virtual ~SimpleBVH() {}
    bool construct(
        const std::vector<Vec3>& vs,
        const std::vector<Vec3>& ns,
        const std::vector<Face>& fs)
    {
        //
        vs_ = vs;
        fs_ = fs;
        // 全三角形のデータをまとめたものを作成する
        const int32_t faceNum = (int32_t)fs_.size();
        std::vector<MeshTriangle> triangles;
        triangles.reserve(faceNum);
        for (int32_t faceNo = 0; faceNo < faceNum; ++faceNo)
        {
            //
            const Face& face = fs_[faceNo];
            //
            MeshTriangle tri;
            tri.v[0] = vs[face.idxVtx[0]];
            tri.v[1] = vs[face.idxVtx[1]];
            tri.v[2] = vs[face.idxVtx[2]];
            tri.n[0] = ns[face.idxNorm[0]];
            tri.n[1] = ns[face.idxNorm[1]];
            tri.n[2] = ns[face.idxNorm[2]];
            tri.aabb.clear();
            tri.aabb.addPoint(Vec3(tri.v[0]));
            tri.aabb.addPoint(Vec3(tri.v[1]));
            tri.aabb.addPoint(Vec3(tri.v[2]));
            tri.faceid = faceNo;
            triangles.push_back(tri);
        }
        //
        nodes_.reserve(faceNum*2);
        nodes_.resize(1);
        constructNode(0, triangles.data(), (int32_t)triangles.size(), 0);
        nodes_.shrink_to_fit();
        return true;
    }
    //
    bool intersect(RayExt& ray) const
    {
        int32_t hitNodeIdx = 0;
        const bool isHit = intersectSub(0, ray, &hitNodeIdx);
        if (isHit)
        {
            const Node& node = nodes_[hitNodeIdx];
            const float u = ray.u;
            const float v = ray.v;
            ray.isect =
                node.v[0] * (1.0f - u - v) +
                node.v[1] * u +
                node.v[2] * v;
            ray.ns = 
                node.n[0] * (1.0f - u - v) +
                node.n[1] * u +
                node.n[2] * v;
            ray.faceid = node.faceid;
        }
        return isHit;
    }
    bool intersectCheck(RayExt& ray) const
    {
        // intersect()をそのまま流用
        int32_t hitNodeIdx = 0;
        return intersectSub(0, ray, &hitNodeIdx);
    }

private:
    struct MeshTriangle
    {
    public:
        // 頂点位置
        std::array<Vec3, 3> v;
        // 法線
        std::array<Vec3, 3> n;
        // AABB
        AABB aabb;
        //
        int32_t faceid;
    };
    struct Node
    {
    public:
        // 枝であった場合の子のノードインデックス。葉の場合は全て-1が格納されている。
        int32_t childlen[2];
        // AABB
        AABB aabb;
        // 葉であった場合の頂点座標。枝の場合は無効な値。
        std::array<Vec3, 3> v;
        // 葉であった場合の法線。枝の場合は無効な値。
        std::array<Vec3, 3> n;
        //
        int32_t faceid = 0;
    };

private:
    void constructNode(int32_t nodeIndex,
        MeshTriangle* triangles,
        int32_t numTriangle,
        int32_t depth)
    {
        // このノードのAABBを求める
        auto& curNode = nodes_[nodeIndex];
        curNode.childlen[0] = -1;
        curNode.childlen[1] = -1;
        curNode.aabb.clear();
        for(int32_t triNo=0;triNo< numTriangle;++triNo)
        {
            curNode.aabb.addAABB(triangles[triNo].aabb);
        }
        // 三角形が一つしかない場合は葉
        if (numTriangle == 1)
        {
            auto& tri = triangles[0];
            auto& v = tri.v;
            auto& n = tri.n;
            curNode.v[0] = v[0];
            curNode.v[1] = v[1];
            curNode.v[2] = v[2];
            curNode.n[0] = n[0];
            curNode.n[1] = n[1];
            curNode.n[2] = n[2];
            curNode.faceid = tri.faceid;
            return;
        }
        // 軸と分割位置を適当に決める
        static int32_t axisNext = 0;
        int32_t axis = (axisNext++) % 3;
        // 三角形ソート
        auto sortPred = [&axis](const MeshTriangle& lhs, const MeshTriangle& rhs) {
            Vec3 lhsc = lhs.aabb.center();
            Vec3 rhsc = rhs.aabb.center();
            return lhsc[axis] < rhsc[axis];
        };
        int32_t bestTriIndex = numTriangle / 2;
        std::sort(triangles, triangles + numTriangle, sortPred);
        //
        nodes_.resize(nodes_.size() + 1);
        curNode.childlen[0] = (int32_t)nodes_.size() - 1;
        constructNode(curNode.childlen[0], triangles, bestTriIndex, depth + 1);
        nodes_.resize(nodes_.size() + 1);
        curNode.childlen[1] = (int32_t)nodes_.size() - 1;
        constructNode(curNode.childlen[1],
            triangles + bestTriIndex,
            numTriangle - bestTriIndex,
            depth + 1);
    }

    bool intersectSub(int32_t nodeIndex,
        RayExt& ray,
        int32_t* hitNodeIndex) const
    {
        //
        const auto& node = nodes_[nodeIndex];
        // このAABBに交差しなければ終了
        if (!node.aabb.intersectCheck(ray, ray.tfar))
        {
            return false;
        }
        // 葉の場合は、ノードの三角形と交差判定
        else if (node.childlen[0] == -1)
        {
            auto& v = node.v;
            if (intersectTriangle(
                ray,
                v[0], v[1], v[2]))
            {
                *hitNodeIndex = nodeIndex;
                return true;
            }
            return false;
        }
        // 枝の場合は、子を見に行く
        else
        {
            const bool h0 = intersectSub(node.childlen[0], ray, hitNodeIndex);
            const bool h1 = intersectSub(node.childlen[1], ray, hitNodeIndex);
            return h0 || h1;
        }
    }

private:
    std::vector<Vec3> vs_;
    std::vector<Face> fs_;
    // 各三角形のAABB
    std::vector<AABB> nodeAABBs_;
    // ノード
    std::vector<Node> nodes_;
};

SimpleBVH g_bvh;

//
void preprocess(
    const float* vertices,
    size_t numVerts,
    const float* normals,
    size_t numNormals,
    const uint32_t* indices,
    size_t numFace)
{
    //
    std::vector<Vec3> verts;
    verts.reserve(numVerts);
    for (size_t vi=0;vi< numVerts;++vi)
    {
        verts.push_back(
            Vec3(vertices[vi * 3 + 0],
                vertices[vi * 3 + 1],
                vertices[vi * 3 + 2]));
    }
    //
    std::vector<Vec3> ns;
    ns.reserve(numNormals);
    for (size_t ni = 0; ni < numNormals; ++ni)
    {
        ns.push_back(
            Vec3(normals[ni * 3 + 0],
                normals[ni * 3 + 1],
                normals[ni * 3 + 2]));
    }
    //
    std::vector<Face> fs;
    fs.reserve(numFace);
    for (size_t fi = 0; fi < numFace; ++fi)
    {
        Face face;
        face.idxVtx = {
            indices[fi * 6 + 0],
            indices[fi * 6 + 2],
            indices[fi * 6 + 4]
        };
        face.idxNorm = { {
            indices[fi * 6 + 1],
            indices[fi * 6 + 3],
            indices[fi * 6 + 5]
        } };
        fs.push_back(face);
    }
    //
    g_bvh = SimpleBVH();
    g_bvh.construct(verts, ns, fs);
}

void intersect(
    Ray* rays,
    size_t numRay,
    bool hitany)
{
    static_cast<void>(hitany);
    //
    for (int32_t nr=0;nr<numRay;++nr)
    {
        Ray& ray = rays[nr];
        RayExt rayExt;
        rayExt.pos = ray.pos;
        rayExt.dir = ray.dir;
        const auto invSafe = [](float v)
        {
            return
                (v == 0.0f) ?
                std::numeric_limits<float>::infinity() : 1.0f / v;
        };
        rayExt.dinv = Vec3(
            invSafe(rayExt.dir.x()),
            invSafe(rayExt.dir.y()),
            invSafe(rayExt.dir.z()));
        rayExt.sign[0] = (ray.dir[0] < 0.0f);
        rayExt.sign[1] = (ray.dir[1] < 0.0f);
        rayExt.sign[2] = (ray.dir[2] < 0.0f);
        rayExt.tnear = ray.tnear;
        rayExt.tfar = ray.tfar;
        //
        if (!g_bvh.intersect(rayExt))
        {
            ray.isisect = false;
        }
        else
        {
            ray.isisect = true;
            const Vec3& isectPos = rayExt.isect;
            ray.isect[0] = isectPos.x();
            ray.isect[1] = isectPos.y();
            ray.isect[2] = isectPos.z();
            ray.ns[0] = rayExt.ns.x();
            ray.ns[1] = rayExt.ns.y();
            ray.ns[2] = rayExt.ns.z();
            ray.faceid = rayExt.faceid;
        }
    }
}
 