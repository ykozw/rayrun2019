//
#define _USE_MATH_DEFINES
#include "rayrun.hpp"
//
#define WIN32_LEAN_AND_MEAN
#define NOMINMAX
#include <windows.h>
#include <concurrent_vector.h>
//
#define STB_IMAGE_IMPLEMENTATION
#include "stb_image.h"
#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "stb_image_write.h"
#define TINYOBJLOADER_IMPLEMENTATION
#include "tiny_obj_loader.h"
#include "picojson.h"
//
#include <cmath>
#include <vector>
#include <string>
#include <cstdint>
#include <array>
#include <filesystem>
#include <algorithm>
#include <random>
#include <chrono>

//
typedef void(*PreprocessFun)(
    const float* vertices,
    size_t numVerts,
    const float* normals,
    size_t numNormals,
    const uint32_t* indices,
    size_t numFace);

typedef void(*IsectFun)(
    Ray* rays,
    size_t numRay,
    bool hitany);

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
    static float dist01ance(Vec3 rhs, Vec3 lhs)
    {
        ;
        return (rhs - lhs).length();
    }
private:
    float x_;
    float y_;
    float z_;
};

//
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

//
static Vec3 getHemisphere(float x, float y)
{
    const float phi = 2.0f * float(M_PI) * x;
    const float sinPhi = std::sinf(phi);
    const float cosPhi = std::cosf(phi);
    const float cosTheta = 1.0f - y;
    const float sinTheta = std::sqrtf(1 - cosTheta * cosTheta);
    return Vec3(sinTheta * cosPhi, sinTheta * sinPhi, cosTheta);

}

//
static std::tuple<
    std::vector<float>,
    std::vector<float>,
    std::vector<uint32_t>>
    loadMesh(const std::string& filename)
{
    tinyobj::attrib_t attrib;
    std::vector<tinyobj::shape_t> shapes;
    std::vector<tinyobj::material_t> materials;
    std::string warn;
    std::string err;
    std::vector<uint32_t> indices;
    tinyobj::LoadObj(&attrib, &shapes, &materials, &warn, &err, filename.c_str(), nullptr, true);
    //
    for (auto& shape : shapes)
    {
        for (auto& index : shape.mesh.indices)
        {
            indices.push_back(index.vertex_index);
            indices.push_back(index.normal_index);
        }
    }
    return { attrib.vertices , attrib.normals, indices };
}

//
struct SceneSetting
{
public:
    std::string model;
    std::array<float, 3> pos;
    std::array<float, 3> dir;
    std::array<float, 3> up;
    float fovy;
    int32_t width;
    int32_t height;
    int32_t samplePerPixel;
    int32_t sampleAo;

public:
    void load(const std::string& filename)
    {
        // JSONデータの読み込み。
        std::ifstream file(filename, std::ios::in);
        const std::string json((std::istreambuf_iterator<char>(file)), std::istreambuf_iterator<char>());
        file.close();
        picojson::value root;
        const std::string err = picojson::parse(root, json);
        if (err != "")
        {
            printf("%s\n", err.c_str());
            return;
        }
        //
        const auto getV3 = [](picojson::value& obj)->std::array<float, 3>
        {
            picojson::array& posArr = obj.get<picojson::array>();
            return {
                float(posArr[0].get<double>()),
                float(posArr[1].get<double>()),
                float(posArr[2].get<double>()) };
        };
        //
        SceneSetting& SceneSetting = *this;
        picojson::object& obj = root.get<picojson::object>();
        SceneSetting.model = obj["model"].get<std::string>();
        SceneSetting.pos = getV3(obj["pos"]);
        SceneSetting.dir = getV3(obj["dir"]);
        SceneSetting.up = getV3(obj["up"]);
        SceneSetting.fovy = float(obj["fovy"].get<double>());
        SceneSetting.width = int32_t(obj["width"].get<double>());
        SceneSetting.height = int32_t(obj["height"].get<double>());
        SceneSetting.samplePerPixel = int32_t(obj["samplePerPixel"].get<double>());
        SceneSetting.sampleAo = int32_t(obj["sampleAo"].get<double>());
    }
};

//
class Stopwatch
{
    using clock = std::chrono::system_clock;
public:
    void start()
    {
        start_ = std::chrono::system_clock::now();
    }
    void stop()
    {
        end_ = std::chrono::system_clock::now();
    }
    double elapsed()
    {
        return (double)std::chrono::duration_cast<std::chrono::milliseconds>(end_ - start_).count();
    }
    void print(const char* tag)
    {
        printf("%s %4.1fms\n", tag, elapsed());
    }
private:
    clock::time_point start_;
    clock::time_point end_;
};

//
void main(int32_t argc, char** argv)
{
    if (argc != 3)
    {
        return;
    }
    const char* dllname = argv[1];
    const char* jsonname = argv[2];
    //
    auto jsonpath = std::filesystem::current_path();
    jsonpath.append(jsonname);
    jsonpath = std::filesystem::canonical(jsonpath);

    //
    HMODULE dll = LoadLibrary(dllname);
    const PreprocessFun preprocess = (PreprocessFun)GetProcAddress(dll, "preprocess");
    const IsectFun intersect = (IsectFun)GetProcAddress(dll, "intersect");
    //
    SceneSetting setting;
    setting.load(jsonpath.string());
    //
    const int32_t width = setting.width;
    const int32_t height = setting.height;
    const int32_t hw = width / 2;
    const float iw = 1.0f / float(width);
    const int32_t hh = height / 2;
    const float ih = 1.0f / float(height);
    const Vec3 dir = Vec3(setting.dir.data()).normalized();
    const Vec3 pos = Vec3(setting.pos.data());
    const Vec3 right = Vec3::cross(dir, Vec3(setting.up.data())).normalized();
    const Vec3 up = Vec3::cross(right, dir).normalized();
    const float hfovy = setting.fovy * 0.5f;
    const int32_t numPrimRay = setting.samplePerPixel;
    const int32_t numAoSample = setting.sampleAo;
    const float invNumSample = 1.0f / float(numAoSample*numPrimRay);
    //
    std::uniform_real_distribution<float> dist01(0.0f, 1.0f);
    // objをロード
    Stopwatch swLoad;
    swLoad.start();
    auto objpath = jsonpath.parent_path();
    objpath.append(setting.model);
    auto[vertices, normals, indices] = loadMesh(objpath.string());
    swLoad.stop();
    swLoad.print("Load");
    //
    std::vector<std::array<float,3>> image;
    image.resize(setting.width*setting.height);
    //
    Stopwatch swPreprocess;
    swPreprocess.start();
    preprocess(vertices.data(), vertices.size() / 3, normals.data(), normals.size() / 3, indices.data(), indices.size() / 6);
    swPreprocess.stop();
    swPreprocess.print("preprocess");
    //
    std::atomic<int32_t> doneLine = 0;
    size_t rayCount = 0;

    Stopwatch swIsect;
    swIsect.start();
#pragma omp parallel for schedule(dynamic, 16) reduction(+:rayCount) 
    for (int32_t y = 0; y < height; ++y)
    {
        std::mt19937 rng(uint32_t(std::hash<int32_t>{}(y)));
        //
        std::vector<Ray> rays(setting.sampleAo);
        std::vector<float> coss(setting.sampleAo);
        //
        for (int32_t x = 0; x < width; ++x)
        {
            float ao = 0.0f;
            for (int32_t np = 0; np < numPrimRay; ++np)
            {
                //
                const float px = float(x - hw) + dist01(rng);
                const float py = float(y - hh) + dist01(rng);
                const float xs = px * iw * std::tanf(hfovy*float(width) / float(height));
                const float ys = py * ih * std::tanf(hfovy);
                const Vec3 rd = (Vec3(ys) * up + Vec3(xs) * right + dir).normalized();
                Ray primRay;
                primRay.pos[0] = pos.x();
                primRay.pos[1] = pos.y();
                primRay.pos[2] = pos.z();
                primRay.dir[0] = rd.x();
                primRay.dir[1] = rd.y();
                primRay.dir[2] = rd.z();
                primRay.tnear = 0.000f;
                primRay.tfar = std::numeric_limits<float>::infinity();
                primRay.valid = true;
                intersect(&primRay, 1, false);
                ++rayCount;
                //
                if (!primRay.isisect)
                {
                    continue;
                }
                Vec3 isectPos = primRay.isect;
                const Vec3 ns = Vec3(primRay.ns).normalized();
                const OrthonormalBasis onb(ns);
                //
                for (int32_t sn = 0; sn < numAoSample; ++sn)
                {
                    const Vec3 wiLocal = getHemisphere(dist01(rng), dist01(rng));
                    const Vec3 woWorld = onb.local2world(wiLocal);
                    coss[sn] = wiLocal.z();
                    auto& ray = rays[sn];
                    ray.pos[0] = isectPos.x();
                    ray.pos[1] = isectPos.y();
                    ray.pos[2] = isectPos.z();
                    ray.dir[0] = woWorld.x();
                    ray.dir[1] = woWorld.y();
                    ray.dir[2] = woWorld.z();
                    ray.tnear = 0.001f;
                    ray.tfar = std::numeric_limits<float>::infinity();
                    ray.valid = true;
                }
                rayCount += numAoSample;
                // isect
                intersect(rays.data(), numAoSample, true);
                //
                for (int32_t ri = 0; ri < numAoSample; ++ri)
                {
                    auto& ray = rays[ri];
                    ao += (!ray.isisect) ? invNumSample * coss[ri] : 0.0f;
                }
            }
            const size_t pi = (x + y * width);
            image[pi][0] = ao;
            image[pi][1] = ao;
            image[pi][2] = ao;
        }

        //
        const int32_t done = doneLine.fetch_add(1);
        const int32_t t0 = ((done - 1) * 100 / height);
        const int32_t t1 = (done * 100 / height);
        if (t0 != t1)
        {
            printf("%d%% done\n", t1);
        }
    }
    swIsect.stop();

#if 1 // デバッグでファイルを出力する
    std::vector<std::array<uint8_t, 3>> ldrImage;
    ldrImage.resize(image.size());
    for (size_t pi = 0; pi < image.size(); ++pi)
    {
        auto& src = image[pi];
        auto& dst = ldrImage[pi];
        const auto conv = [](float v)
        {
            v = std::powf(v, 1.0f / 2.2f);
            v = std::max(std::min(1.0f, v), 0.0f);
            return uint8_t(255.0f * v + 0.5f);
        };
        dst[0] = conv(src[0]);
        dst[1] = conv(src[1]);
        dst[2] = conv(src[2]);
    }
    stbi_flip_vertically_on_write(true);
    stbi_write_png("output.png", setting.width, setting.height, 3, ldrImage.data(), setting.width * 3);
#endif
    //
    swIsect.print("isect");
    const float mrays = float(double(rayCount) / double(swIsect.elapsed() * 1000.0));
    printf("%.2fMRays/sec\n", mrays);
    //
    FreeLibrary(dll);
}
