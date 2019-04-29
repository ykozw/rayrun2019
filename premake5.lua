-- solution 
solution "rayrun"
	location "generated"
	configurations { "Debug", "Release" }
	platforms {"x64"}

-- 
project "rayrun"
	kind "ConsoleApp"
	language "C++"
	characterset "MBCS"
	files {
		"src/rayrun/*.hpp",
		"src/rayrun/*.cpp",
	}
	includedirs {
		"thirdparty/stb/",
		"thirdparty/tinyobjloader/",
		"thirdparty/picojson/",
	}
	cppdialect "C++17"
	dependson { "refimp" }

-- 
project "refimp"
	kind "SharedLib"
	language "C++"
	characterset "MBCS"
	files {
		"src/refimpl/**.hpp",
		"src/refimpl/**.cpp",
	}
	cppdialect "C++17"
	prebuildcommands {
		"COPY %{prj.location}..\\thirdparty\\embree\\bin\\embree3.dll %{prj.location}",
		"COPY %{prj.location}..\\thirdparty\\embree\\bin\\tbb.dll %{prj.location}"
	}
	includedirs {
		"thirdparty/embree/include/",
	}
	libdirs {
		"thirdparty/embree/lib/",
	}
	links {
		"embree3.lib",
		"tbb.lib",
		"tbbmalloc.lib",
	}