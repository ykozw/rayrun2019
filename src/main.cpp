//
#define _USE_MATH_DEFINES
#define WIN32_LEAN_AND_MEAN
#define NOMINMAX
#define STB_IMAGE_IMPLEMENTATION
#define STB_IMAGE_WRITE_IMPLEMENTATION
#define TINYOBJLOADER_IMPLEMENTATION
//
#include "rayrun.hpp"
//
#include "stb_image.h"
#include "stb_image_write.h"
#include "tiny_obj_loader.h"
#include "picojson.h"
#include "glm/glm.hpp"
#include "imgui.h"
#include "imgui_impl_win32.h"
#include "imgui_impl_dx11.h"
//
#include <windows.h>
#include <Commdlg.h>
#include <concurrent_vector.h>
#include <d3d11.h>
#include <tchar.h>
#include <omp.h>
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
#include <vector>
#include <thread>
#include <array>

//
typedef bool(*neverUseOpenMPFun)();

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
class OrthonormalBasis
{
public:
    OrthonormalBasis() = default;
    OrthonormalBasis(glm::vec3 n)
    {
        if (fabsf(n.x) < 0.99f)
        {
            s_ = glm::cross(n, glm::vec3(1.0f, 0.0f, 0.0));
        }
        else
        {
            s_ = glm::cross(n, glm::vec3(0.0f, 1.0f, 0.0f));
        }
        s_ = glm::normalize(s_);
        t_ = glm::cross(s_, n);
        n_ = n;

        //
        is_ = glm::vec3(s_.x, t_.x, n_.x);
        it_ = glm::vec3(s_.y, t_.y, n_.y);
        in_ = glm::vec3(s_.z, t_.z, n_.z);

    }
    glm::vec3 world2local(glm::vec3 world) const
    {
        return glm::vec3(
            glm::dot(world, s_),
            glm::dot(world, t_),
            glm::dot(world, n_));
    }
    glm::vec3 local2world(glm::vec3 local) const
    {
        return glm::vec3(
            glm::dot(local, is_),
            glm::dot(local, it_),
            glm::dot(local, in_));
    }

private:
    glm::vec3 s_;
    glm::vec3 t_;
    glm::vec3 n_;
    glm::vec3 is_;
    glm::vec3 it_;
    glm::vec3 in_;
};

//
static glm::vec3 getHemisphere(float x, float y)
{
    const float phi = 2.0f * float(M_PI) * x;
    const float sinPhi = std::sinf(phi);
    const float cosPhi = std::cosf(phi);
    const float cosTheta = 1.0f - y;
    const float sinTheta = std::sqrtf(1 - cosTheta * cosTheta);
    return glm::vec3(sinTheta * cosPhi, sinTheta * sinPhi, cosTheta);

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

    // 法線がない場合は生成する
    if (attrib.normals.empty())
    {
        const auto& verts = attrib.vertices;
        const int32_t numVerts = verts.size() / 3;
        std::vector<glm::vec3> normals(numVerts, glm::vec3(0.0f, 0.0f, 0.0f));
        for (auto& shape : shapes)
        {
            const auto& indices = shape.mesh.indices;
            const int32_t numFace = indices.size() / 3;
            for(int32_t fi=0;fi< numFace;++fi)
            {
                const int32_t vi0 = indices[fi * 3 + 0].vertex_index;
                const int32_t vi1 = indices[fi * 3 + 1].vertex_index;
                const int32_t vi2 = indices[fi * 3 + 2].vertex_index;
                const glm::vec3 v0(verts[vi0 * 3 + 0], verts[vi0 * 3 + 1], verts[vi0 * 3 + 2]);
                const glm::vec3 v1(verts[vi1 * 3 + 0], verts[vi1 * 3 + 1], verts[vi1 * 3 + 2]);
                const glm::vec3 v2(verts[vi2 * 3 + 0], verts[vi2 * 3 + 1], verts[vi2 * 3 + 2]);
                const glm::vec3 e01 = v1 - v0;
                const glm::vec3 e02 = v2 - v0;
                const glm::vec3 n = glm::cross(e01, e02);
                normals[vi0] += n;
                normals[vi1] += n;
                normals[vi2] += n;
            }
            for (auto& index : shape.mesh.indices)
            {
                index.normal_index = index.vertex_index;
            }
        }
        attrib.normals.resize(attrib.vertices.size());
        for (int32_t ni = 0; ni < normals.size(); ++ni)
        {
            const glm::vec3 n = glm::normalize(normals[ni]);
            attrib.normals[ni * 3 + 0] = n.x;
            attrib.normals[ni * 3 + 1] = n.y;
            attrib.normals[ni * 3 + 2] = n.z;
        }
    }

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
    glm::vec3 pos;
    glm::vec3 dir;
    glm::vec3 up;
    float fovy;
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
        const auto getV3 = [](picojson::value& obj)->glm::vec3
        {
            picojson::array& posArr = obj.get<picojson::array>();
            return glm::vec3(
                float(posArr[0].get<double>()),
                float(posArr[1].get<double>()),
                float(posArr[2].get<double>()) );
        };
        //
        SceneSetting& SceneSetting = *this;
        picojson::object& obj = root.get<picojson::object>();
        SceneSetting.model = obj["model"].get<std::string>();
        SceneSetting.pos = getV3(obj["pos"]);
        SceneSetting.dir = glm::normalize(getV3(obj["dir"]));
        SceneSetting.up = getV3(obj["up"]);
        SceneSetting.fovy = float(obj["fovy"].get<double>());
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
    double elapsedNow()
    {
        return (double)std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now() - start_).count();
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
void renderingMain(
    int32_t width,
    int32_t height,
    std::vector<std::array<float,4>>& pixels,
    const std::string& dllName, 
    const std::string& jsonName,
    int32_t& preprocessTime,
    int32_t& renderingTime,
    int32_t& renderingPercent,
    std::string& renderingState)
{
    renderingState = "LOAD CONFIG";
    if (dllName.empty() || jsonName.empty())
    {
        return;
    }
    //
    preprocessTime = 0;
    renderingTime = 0;
    //
    std::filesystem::path jsonpath = jsonName;
    //
    HMODULE dll = LoadLibrary(dllName.c_str());
    const neverUseOpenMPFun neverUseOpenMP = (neverUseOpenMPFun)GetProcAddress(dll, "neverUseOpenMP");
    const PreprocessFun preprocess = (PreprocessFun)GetProcAddress(dll, "preprocess");
    const IsectFun intersect = (IsectFun)GetProcAddress(dll, "intersect");
    const bool useOpenMP = (neverUseOpenMP != nullptr) ? !neverUseOpenMP() : true;
    if (!useOpenMP)
    {
        omp_set_num_threads(1);
    }
    //
    SceneSetting setting;
    setting.load(jsonpath.string());
    //
    const int32_t hw = width / 2;
    const float iw = 1.0f / float(width);
    const int32_t hh = height / 2;
    const float ih = 1.0f / float(height);
    const glm::vec3 dir = glm::normalize(setting.dir);
    const glm::vec3 pos = glm::vec3(setting.pos);
    const glm::vec3 right = glm::cross(dir, glm::normalize(setting.up));
    const glm::vec3 up = glm::normalize(glm::cross(right, dir));
    const float hfovy = setting.fovy * 0.5f;
    const int32_t numPrimRay = setting.samplePerPixel;
    const int32_t numAoSample = setting.sampleAo;
    const float invNumSample = 1.0f / float(numAoSample*numPrimRay);
    //
    std::uniform_real_distribution<float> dist01(0.0f, 1.0f);
    // objをロード
    renderingState = "LOAD OBJ";
    Stopwatch swLoad;
    swLoad.start();
    auto objpath = jsonpath.parent_path();
    objpath.append(setting.model);
    auto[vertices, normals, indices] = loadMesh(objpath.string());
    swLoad.stop();
    swLoad.print("Load");
    //
    renderingState = "CONSTRUCT BVH";
    Stopwatch swPreprocess;
    swPreprocess.start();
    preprocess(vertices.data(), vertices.size() / 3, normals.data(), normals.size() / 3, indices.data(), indices.size() / 6);
    swPreprocess.stop();
    swPreprocess.print("preprocess");
    //
    std::atomic<size_t> rayCountTotal = 0;
    //
    renderingState = "RENDERING";
    Stopwatch swIsect;
    swIsect.start();
    std::atomic<int32_t> doneLine = 0;
    bool timeout = false;
#pragma omp parallel for schedule(dynamic, 16)
    for (int32_t y = 0; y < height; ++y)
    {
        std::mt19937 rng(uint32_t(std::hash<int32_t>{}(y)));
        size_t rayCount = 0;
        //
        std::vector<Ray> rays(setting.sampleAo);
        std::vector<float> coss(setting.sampleAo);

        // 60秒でタイムアウト
        if (swIsect.elapsedNow() > 60000)
        {
            timeout = true;
            break;
        }

        //
        for (int32_t x = 0; x < width; ++x)
        {
            float ao = 0.0f;
            for (int32_t np = 0; np < numPrimRay; ++np)
            {
                //
                const float px = float(x - hw) + dist01(rng);
                const float py = float(y - hh) + dist01(rng);
                const float xs = px * iw * std::tanf(hfovy * float(width) / float(height));
                const float ys = py * ih * std::tanf(hfovy);
                const glm::vec3 rd = glm::normalize(glm::vec3(ys) * up + glm::vec3(xs) * right + dir);
                Ray primRay;
                primRay.pos[0] = pos.x;
                primRay.pos[1] = pos.y;
                primRay.pos[2] = pos.z;
                primRay.dir[0] = rd.x;
                primRay.dir[1] = rd.y;
                primRay.dir[2] = rd.z;
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
                glm::vec3 isectPos =
                    glm::vec3(
                        primRay.isect[0],
                        primRay.isect[1],
                        primRay.isect[2]);
                const glm::vec3 ns = glm::normalize(glm::vec3(
                    primRay.ns[0],
                    primRay.ns[1],
                    primRay.ns[2]));
                const OrthonormalBasis onb(ns);
                //
                for (int32_t sn = 0; sn < numAoSample; ++sn)
                {
                    const glm::vec3 wiLocal = getHemisphere(dist01(rng), dist01(rng));
                    const glm::vec3 woWorld = onb.local2world(wiLocal);
                    coss[sn] = wiLocal.z;
                    auto& ray = rays[sn];
                    ray.pos[0] = isectPos.x;
                    ray.pos[1] = isectPos.y;
                    ray.pos[2] = isectPos.z;
                    ray.dir[0] = woWorld.x;
                    ray.dir[1] = woWorld.y;
                    ray.dir[2] = woWorld.z;
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
            pixels[pi][0] = ao;
            pixels[pi][1] = ao;
            pixels[pi][2] = ao;
            pixels[pi][3] = 1.0f;
        }
        // OMPはreductionに参照型を渡せないのでここでコピー
        rayCountTotal += rayCount;
        //
        const int32_t done = doneLine.fetch_add(1);
        const int32_t t0 = ((done - 1) * 100 / height);
        const int32_t t1 = (done * 100 / height);
        if (t0 != t1)
        {
            printf("%d%% done\n", t1);
            renderingPercent = t1;
        }
    }
    swIsect.stop();
    renderingPercent = 100;
    //
    swIsect.print("isect");
    const float mrays = float(double(rayCountTotal) / double(swIsect.elapsed() * 1000.0));
    printf("%.2fMRays/sec\n", mrays);
    //
    FreeLibrary(dll);
    //
    if (timeout)
    {
        renderingState = "TIMEOUT...";
    }
    else
    {
        renderingState =
            "TIME:" + std::_Floating_to_string("%.3f", (swPreprocess.elapsed() + swIsect.elapsed()) / 1000.0f) + "sec (" +
            "BVH:" + std::_Floating_to_string("%.3f", swPreprocess.elapsed() / 1000.0f) +
            " RT: " + std::_Floating_to_string("%.3f", swIsect.elapsed() / 1000.0f) + ")";
    }
}
//
static ID3D11Device* g_pd3dDevice = nullptr;
static ID3D11DeviceContext* g_pd3dDeviceContext = nullptr;
static IDXGISwapChain* g_pSwapChain = nullptr;
static ID3D11RenderTargetView* g_mainRenderTargetView = nullptr;
//
class Texture
{
public:
    Texture() = default;
    //
    void resize(int32_t width, int32_t height)
    {
        //
        width_ = width;
        height_ = height;
        pixels_.resize(width * height);
        fillWithCheckeredPattern();
        //
        if (texture_ != nullptr)
        {
            texture_->Release();
        }
        if (shaderResourceView_ != nullptr)
        {
            shaderResourceView_->Release();
        }
        //
        D3D11_TEXTURE2D_DESC tex2dDesc;
        ZeroMemory(&tex2dDesc, sizeof(tex2dDesc));
        tex2dDesc.Usage = D3D11_USAGE_DYNAMIC;
        tex2dDesc.BindFlags = D3D11_BIND_SHADER_RESOURCE;
        tex2dDesc.Width = width_;
        tex2dDesc.Height = height_;
        tex2dDesc.Format = DXGI_FORMAT_R32G32B32A32_FLOAT;
        tex2dDesc.MipLevels = 1;
        tex2dDesc.ArraySize = 1;
        tex2dDesc.MiscFlags = 0;
        tex2dDesc.CPUAccessFlags = D3D11_CPU_ACCESS_WRITE;
        tex2dDesc.SampleDesc.Count = 1;
        tex2dDesc.SampleDesc.Quality = 0;
        //
        D3D11_SUBRESOURCE_DATA sd;
        sd.pSysMem = pixels_.data();
        sd.SysMemPitch = width_ * sizeof(float) * 4;
        if (FAILED(g_pd3dDevice->CreateTexture2D(&tex2dDesc, &sd, &texture_)))
        {
            return;
        }
        //
        D3D11_SHADER_RESOURCE_VIEW_DESC srvd;
        ZeroMemory(&srvd, sizeof(srvd));
        srvd.Format = tex2dDesc.Format;
        srvd.ViewDimension = D3D11_SRV_DIMENSION_TEXTURE2D;
        srvd.Texture2D.MostDetailedMip = 0;
        srvd.Texture2D.MipLevels = 1;
        if (FAILED(g_pd3dDevice->CreateShaderResourceView(texture_, &srvd, &shaderResourceView_)))
        {
            return;
        }
    }
    //
    void fillWithCheckeredPattern()
    {
        for (int32_t y = 0; y < height_; ++y)
        {
            for (int32_t x = 0; x < width_; ++x)
            {
                const float color = ((((x / 32) % 2) + ((y / 32) % 2)) % 2 == 0) ? 0.5f : 0.8f;
                pixels_[x + y * width_] = { color, color, color, 1.0f };
            }
        }
    }
    //
    void updatePixels()
    {
        D3D11_MAPPED_SUBRESOURCE mappedResource;
        ZeroMemory(&mappedResource, sizeof(D3D11_MAPPED_SUBRESOURCE));
        g_pd3dDeviceContext->Map(texture_, 0, D3D11_MAP_WRITE_DISCARD, 0, &mappedResource);
        memcpy(mappedResource.pData, pixels_.data(), sizeof(float) * 4 * pixels_.size());
        g_pd3dDeviceContext->Unmap(texture_, 0);
    }
    void* texId()
    {
        return shaderResourceView_;
    }
    int32_t Texture::width() const
    {
        return width_;
    }
    int32_t Texture::height() const
    {
        return height_;
    }
    std::vector<std::array<float, 4>> & pixels()
    {
        return pixels_;
    }
private:
    int32_t width_ = 0;
    int32_t height_ = 0;
    std::vector<std::array<float, 4>> pixels_;
    //
    ID3D11Texture2D* texture_ = nullptr;
    ID3D11ShaderResourceView* shaderResourceView_ = nullptr;
};
//
Texture g_texture;
//
static void createRenderTarget()
{
    ID3D11Texture2D* pBackBuffer;
    g_pSwapChain->GetBuffer(0, IID_PPV_ARGS(&pBackBuffer));
    g_pd3dDevice->CreateRenderTargetView(pBackBuffer, nullptr, &g_mainRenderTargetView);
    pBackBuffer->Release();
}
//
static bool createDeviceD3D(HWND hWnd)
{
    //
    DXGI_SWAP_CHAIN_DESC sd;
    ZeroMemory(&sd, sizeof(sd));
    sd.BufferCount = 2;
    sd.BufferDesc.Width = 0;
    sd.BufferDesc.Height = 0;
    sd.BufferDesc.Format = DXGI_FORMAT_R8G8B8A8_UNORM;
    sd.BufferDesc.RefreshRate.Numerator = 60;
    sd.BufferDesc.RefreshRate.Denominator = 1;
    sd.Flags = DXGI_SWAP_CHAIN_FLAG_ALLOW_MODE_SWITCH;
    sd.BufferUsage = DXGI_USAGE_RENDER_TARGET_OUTPUT;
    sd.OutputWindow = hWnd;
    sd.SampleDesc.Count = 1;
    sd.SampleDesc.Quality = 0;
    sd.Windowed = TRUE;
    sd.SwapEffect = DXGI_SWAP_EFFECT_DISCARD;
    //
    UINT createDeviceFlags = 0;
    D3D_FEATURE_LEVEL featureLevel;
    const D3D_FEATURE_LEVEL featureLevelArray[2] = { D3D_FEATURE_LEVEL_11_0, D3D_FEATURE_LEVEL_10_0, };
    if (D3D11CreateDeviceAndSwapChain(nullptr, D3D_DRIVER_TYPE_HARDWARE, nullptr, createDeviceFlags, featureLevelArray, 2, D3D11_SDK_VERSION, &sd, &g_pSwapChain, &g_pd3dDevice, &featureLevel, &g_pd3dDeviceContext) != S_OK)
        return false;

    createRenderTarget();
    return true;
}
//
static void cleanupRenderTarget()
{
    if (g_mainRenderTargetView) { g_mainRenderTargetView->Release(); g_mainRenderTargetView = nullptr; }
}
//
static void cleanupDeviceD3D()
{
    cleanupRenderTarget();
    if (g_pSwapChain) { g_pSwapChain->Release(); g_pSwapChain = nullptr; }
    if (g_pd3dDeviceContext) { g_pd3dDeviceContext->Release(); g_pd3dDeviceContext = nullptr; }
    if (g_pd3dDevice) { g_pd3dDevice->Release(); g_pd3dDevice = nullptr; }
}
//
LRESULT WINAPI WndProc(HWND hWnd, UINT msg, WPARAM wParam, LPARAM lParam)
{
    extern LRESULT ImGui_ImplWin32_WndProcHandler(HWND hWnd, UINT msg, WPARAM wParam, LPARAM lParam);
    if (ImGui_ImplWin32_WndProcHandler(hWnd, msg, wParam, lParam))
    {
        return true;
    }
    //
    switch (msg)
    {
    case WM_SIZE:
        if (g_pd3dDevice != nullptr && wParam != SIZE_MINIMIZED)
        {
            cleanupRenderTarget();
            g_pSwapChain->ResizeBuffers(0, (UINT)LOWORD(lParam), (UINT)HIWORD(lParam), DXGI_FORMAT_UNKNOWN, 0);
            createRenderTarget();
        }
        return 0;
    case WM_SYSCOMMAND:
        if ((wParam & 0xfff0) == SC_KEYMENU) // Disable ALT application menu
            return 0;
        break;
    case WM_DESTROY:
        ::PostQuitMessage(0);
        return 0;
    }
    return ::DefWindowProc(hWnd, msg, wParam, lParam);
}
//
bool openFile(std::filesystem::path& filePath)
{
    //
    OPENFILENAME ofn;
    TCHAR szFile[260] = { 0 };
    ZeroMemory(&ofn, sizeof(ofn));
    ofn.lStructSize = sizeof(ofn);
    ofn.hwndOwner = nullptr;
    ofn.lpstrFile = szFile;
    ofn.nMaxFile = sizeof(szFile);
    ofn.lpstrFilter = _T("All\0*.*\0Text\0*.TXT\0");
    ofn.nFilterIndex = 1;
    ofn.lpstrFileTitle = nullptr;
    ofn.nMaxFileTitle = 0;
    ofn.lpstrInitialDir = nullptr;
    ofn.Flags = OFN_PATHMUSTEXIST | OFN_FILEMUSTEXIST;
    //
    if (!GetOpenFileName(&ofn))
    {
        return false;
    }
    filePath = ofn.lpstrFile;
    filePath = std::filesystem::path(ofn.lpstrFile);
    return true;
}
//
void main(int32_t argc, char** argv)
{
    //
    WNDCLASSEX wc = { sizeof(WNDCLASSEX), CS_CLASSDC, WndProc, 0L, 0L, GetModuleHandle(nullptr), nullptr, nullptr, nullptr, nullptr, _T("ImGui Example"), nullptr };
    ::RegisterClassEx(&wc);
    HWND hwnd = ::CreateWindow(wc.lpszClassName, _T("RayRun2019"), WS_OVERLAPPEDWINDOW, 100, 100, 1280, 800, nullptr, nullptr, wc.hInstance, nullptr);
    //
    if (!createDeviceD3D(hwnd))
    {
        cleanupDeviceD3D();
        ::UnregisterClass(wc.lpszClassName, wc.hInstance);
        return;
    }
    //
    ::ShowWindow(hwnd, SW_SHOWDEFAULT);
    ::UpdateWindow(hwnd);
    //
    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImGuiIO& io = ImGui::GetIO(); (void)io;
    //
    ImGui::StyleColorsDark();
    //
    ImGui_ImplWin32_Init(hwnd);
    ImGui_ImplDX11_Init(g_pd3dDevice, g_pd3dDeviceContext);
    //
    ImGuiStyle& style = ImGui::GetStyle();
    style.WindowPadding = ImVec2(0.0f, 0.0f);
    //
    g_texture.resize(1280, 720);
    //
    std::thread renderingThread = std::thread([]() {});
    //
    std::string jsonPath;
    std::string dllPath;
    int32_t preprocesssTime = 0;
    int32_t renderingTime = 0;
    int32_t bvhPercent = 0;
    int32_t renderingPercent = 0;
    std::string renderingState = "SET UP...";
    //
    MSG msg;
    ZeroMemory(&msg, sizeof(msg));
    while (msg.message != WM_QUIT)
    {
        //
        if (::PeekMessage(&msg, nullptr, 0U, 0U, PM_REMOVE))
        {
            ::TranslateMessage(&msg);
            ::DispatchMessage(&msg);
            continue;
        }
        // 毎フレーム更新
        g_texture.updatePixels();
        //
        ImGui_ImplDX11_NewFrame();
        ImGui_ImplWin32_NewFrame();
        ImGui::NewFrame();
        // BG
        {
            ImGui::SetNextWindowPos(ImVec2(0, 0));
            ImGui::SetNextWindowSize(ImGui::GetIO().DisplaySize);
            ImGui::Begin("BG", nullptr, ImVec2(0, 0), 0.0f,
                ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_NoResize |
                ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoScrollbar |
                ImGuiWindowFlags_NoScrollWithMouse |
                ImGuiWindowFlags_NoBringToFrontOnFocus);
            ImGui::Image(g_texture.texId(), ImGui::GetIO().DisplaySize);
            //
            const auto drawRect = [&](float posY, float barLength, float barThick, float ratio, ImU32 color)
            {
                const ImVec2 dispSize = ImGui::GetIO().DisplaySize;
                const glm::vec2 barCenter = glm::vec2(dispSize.x * 0.5f, dispSize.y * posY);
                const glm::vec2 barSize = glm::vec2(dispSize.x * barLength, dispSize.y * barThick);
                const glm::vec2 barBegin = barCenter - glm::vec2(barSize.x, barSize.y) * 0.5f;
                const glm::vec2 barEnd = barBegin + glm::vec2(ratio * barSize.x, barSize.y);
                ImGui::GetWindowDrawList()->AddRectFilled(
                    ImVec2(barBegin.x, barBegin.y), ImVec2(barEnd.x, barEnd.y), color);
            };
            drawRect(0.9f, 0.9f, 0.08f, 1.0f, 0xFFFFFFFF);
            drawRect(0.9f, 0.89f, 0.07f, 1.0f, 0xFF000000);
            drawRect(0.9f, 0.88f, 0.054f, float(renderingPercent)/100.0f, 0xFFFFFFFF);
            //
            const ImVec2 dispSize = ImGui::GetIO().DisplaySize;
            const ImVec2 barCenter = ImVec2(dispSize.x * 0.06f, dispSize.y * 0.875f);
            ImGui::SetCursorPos(barCenter);
            ImGui::SetWindowFontScale(3.0);
            ImGui::TextColored(ImVec4(1.0f, 0.0f, 0.0f, 1.0f), renderingState.c_str());
            //
            ImGui::End();
        }

        // 設定画面
        {
            ImGui::Begin("RayRun2019", nullptr);
            //
            ImGui::Text("%.1f FPS", ImGui::GetIO().Framerate);
            //
            if (ImGui::Button("JSON"))
            {
                std::filesystem::path path;
                openFile(path);
                if (path.extension() == ".json")
                {
                    jsonPath = path.string();
                }
            }
            ImGui::SameLine();
            //
            ImGui::Text(jsonPath.c_str());
            if (ImGui::Button("DLL"))
            {
                std::filesystem::path path;
                openFile(path);
                if (path.extension() == ".dll")
                {
                    dllPath = path.string();
                }
            }
            ImGui::SameLine();
            ImGui::Text(dllPath.c_str());
            if (ImGui::Button("Render"))
            {
                //
                renderingThread.join();
                //
                g_texture.fillWithCheckeredPattern();
                g_texture.updatePixels();
                //
                renderingThread = std::thread([&]()
                    {
                        //
                        renderingMain(
                            g_texture.width(),
                            g_texture.height(),
                            g_texture.pixels(),
                            dllPath, jsonPath,
                            preprocesssTime,
                            renderingTime,
                            renderingPercent,
                            renderingState);
                        //
                    });
            }
            ImGui::SameLine();
            ImGui::End();
        }
        //
        const ImVec4 clear_color = ImVec4(0.45f, 0.55f, 0.60f, 1.00f);
        ImGui::Render();
        g_pd3dDeviceContext->OMSetRenderTargets(1, &g_mainRenderTargetView, nullptr);
        g_pd3dDeviceContext->ClearRenderTargetView(g_mainRenderTargetView, (float*)& clear_color);
        ImGui_ImplDX11_RenderDrawData(ImGui::GetDrawData());
        //
        g_pSwapChain->Present(1, 0);
    }
    //
    renderingThread.join();
    //
    ImGui_ImplDX11_Shutdown();
    ImGui_ImplWin32_Shutdown();
    ImGui::DestroyContext();

    cleanupDeviceD3D();
    ::DestroyWindow(hwnd);
    ::UnregisterClass(wc.lpszClassName, wc.hInstance);

    return;
}
