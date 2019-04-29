//
#include "rayrun.hpp"
//
#define WIN32_LEAN_AND_MEAN
#define NOMINMAX
#include <windows.h>
//
#define STB_IMAGE_IMPLEMENTATION
#include "stb_image.h"
#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "stb_image_write.h"
#define TINYOBJLOADER_IMPLEMENTATION
#include "tiny_obj_loader.h"
#include "picojson.h"
//
#include <vector>
#include <string>
#include <cstdint>
#include <array>
#include <filesystem>

//
static std::tuple<std::vector<float>, std::vector<uint32_t>> loadMesh(const std::string& filename)
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
        }
    }
    return { attrib.vertices , indices };
}

//
//
struct TestDesc
{
public:
    std::string model;
    struct CamImage
    {
        std::array<float, 3> pos;
        std::array<float, 3> dir;
        std::array<float, 3> up;
        float fovy;
        int32_t width;
        int32_t height;
    };
    std::vector<CamImage> camImage;
};
//
std::vector<TestDesc> getTestDescs(const std::string& filename)
{
    // JSONデータの読み込み。
    std::ifstream file(filename, std::ios::in);
    const std::string json((std::istreambuf_iterator<char>(file)), std::istreambuf_iterator<char>());
    file.close();
    picojson::value v;
    const std::string err = picojson::parse(v, json);
    if (err != "")
    {
        printf("%s\n", err.c_str());
        return {};
    }
    //
    std::vector<TestDesc> testDescs;
    for (auto& test : v.get<picojson::array>())
    {
        TestDesc testDesc;
        picojson::object& obj = test.get<picojson::object>();
        testDesc.model = obj["model"].get<std::string>();
        //
        for (auto& cams : obj["tests"].get<picojson::array>())
        {
            picojson::object& obj2 = cams.get<picojson::object>();

            const auto getV3 = [](picojson::value& obj)->std::array<float, 3>
            {
                picojson::array& posArr = obj.get<picojson::array>();
                return {
                    float(posArr[0].get<double>()),
                    float(posArr[1].get<double>()),
                    float(posArr[2].get<double>()) };
            };
            TestDesc::CamImage camImage;
            camImage.pos = getV3(obj2["pos"]);
            camImage.dir = getV3(obj2["dir"]);
            camImage.up = getV3(obj2["up"]);
            camImage.fovy = float(obj2["fovy"].get<double>());
            camImage.width = int32_t(obj2["width"].get<double>());
            camImage.height = int32_t(obj2["height"].get<double>());
            testDesc.camImage.push_back(camImage);
        }
        testDescs.push_back(testDesc);
    }
    return testDescs;
}

//
static void testMain(RayRunFun rayRun, const std::filesystem::path& jsonpath)
{
    //
    for (const TestDesc& td : getTestDescs(jsonpath.string()))
    {
        // objをロード
        auto objpath = jsonpath.parent_path();
        objpath.append(td.model);
        auto[vertices, indices] = loadMesh(objpath.string());
        //
        std::vector<Test> tests;
        for (auto& ci : td.camImage)
        {
            Test test;
            test.pos[0] = ci.pos[0];
            test.pos[1] = ci.pos[1];
            test.pos[2] = ci.pos[2];
            test.dir[0] = ci.dir[0];
            test.dir[1] = ci.dir[1];
            test.dir[2] = ci.dir[2];
            test.up[0] = ci.up[0];
            test.up[1] = ci.up[1];
            test.up[2] = ci.up[2];
            test.fovy = ci.fovy;
            test.width = ci.width;
            test.height = ci.height;
            test.image = (float*)malloc(test.width*test.height * sizeof(float));
            tests.push_back(test);
        }
        //
        rayRun(vertices.data(), vertices.size() / 3, indices.data(), indices.size() / 3, tests.data(), tests.size());

        // diff
        int32_t outputCount = 0;
        for (auto& test : tests)
        {
            // diff
            std::filesystem::path imagename = objpath.replace_extension();
            imagename.replace_filename(std::filesystem::path(imagename.filename().string() + std::to_string(outputCount)));
            imagename.replace_extension("hdr");
            int32_t width, height, comp;
            stbi_set_flip_vertically_on_load(true);
            float* img = stbi_loadf(imagename.string().c_str(), &width, &height, &comp, 1);
            //
            if ((width != test.width) || (height != test.height))
            {
                printf("image size is wrong.");
                continue;
            }
            //
            const size_t numPixel = test.width*test.height;
            double se = 0;
            for (int32_t pi = 0; pi < numPixel; ++pi)
            {
                const float diff = img[pi] - test.image[pi];
                se += diff * diff;
            }
            const double rmse = std::sqrt(se / double(numPixel));
            printf("RMSE:%f\n", rmse);
            
#if 1
            // NOTE: デバッグ出力
            stbi_flip_vertically_on_write(true);       
            const std::string filename = std::string("output") + std::to_string(outputCount) + ".hdr";
            stbi_write_hdr(filename.c_str(), test.width, test.height, 1, test.image);
#endif
            ++outputCount;
        }
        
    }
}

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
    const RayRunFun rayRun = (RayRunFun)GetProcAddress(dll, "rayRun");
    testMain(rayRun, jsonpath);
    FreeLibrary(dll);
}
