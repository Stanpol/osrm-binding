use std::io::Cursor;
use std::path::{Path, PathBuf};

fn main() {
    let out_dir = PathBuf::from(std::env::var("OUT_DIR").unwrap());
    let target_os = std::env::var("CARGO_CFG_TARGET_OS").unwrap();

    // 1. Download and Unpack
    let osrm_url = "https://github.com/Project-OSRM/osrm-backend/archive/refs/tags/v6.0.0.tar.gz";

    eprintln!("Downloading OSRM source from {}...", osrm_url);

    let mut response = reqwest::blocking::get(osrm_url).unwrap();
    let mut buffer = Vec::new();
    response.copy_to(&mut buffer).unwrap();

    eprintln!("Decompressing OSRM source...");
    let cursor = Cursor::new(buffer);
    let tar_gz = flate2::read::GzDecoder::new(cursor);
    let mut archive = tar::Archive::new(tar_gz);
    archive.unpack(&out_dir).unwrap();

    let osrm_source_path = find_osrm_source(&out_dir);
    eprintln!("OSRM source path: {}", osrm_source_path.display());

    // 2. Prepare Configuration based on OS
    let mut cxx_flags = String::from("-Wno-array-bounds -Wno-uninitialized -Wno-stringop-overflow -std=c++20 -Wno-error");
    
    // Initialize CMake config with common settings
    let mut cmake_config = cmake::Config::new(&osrm_source_path);
    cmake_config
        .define("CMAKE_CXX_STANDARD", "20")
        .define("CMAKE_CXX_STANDARD_REQUIRED", "ON")
        .define("ENABLE_ASSERTIONS", "Off")
        .define("ENABLE_LTO", "Off")
        .define("ENABLE_MASON", "Off"); // Don't download dependencies

    // OS Specific CMake Configuration
    if target_os == "macos" {
        // macOS / Homebrew Configuration
        let tbb_root = "/opt/homebrew/opt/tbb";
        let boost_root = "/opt/homebrew/opt/boost@1.85";
        let tbb_include = format!("{}/include", tbb_root);

        // Additional Mac Flags
        cxx_flags.push_str(" -Wno-suggest-destructor-override -Wno-error=suggest-destructor-override");
        let additional_cxx_flags = format!("-I{} -Wno-suggest-destructor-override -Wno-error=suggest-destructor-override", tbb_include);

        cmake_config
            .env("Boost_ROOT", boost_root)
            .env("TBB_ROOT", tbb_root)
            .define("CMAKE_PREFIX_PATH", format!("{};{}", boost_root, tbb_root))
            .define("TBB_DIR", format!("{}/lib/cmake/TBB", tbb_root))
            // Re-define TBB_ROOT for good measure as seen in your mac build
            .define("TBB_ROOT", tbb_root) 
            .cflag(format!("-I{}", tbb_include))
            .cxxflag(&additional_cxx_flags)
            .define("CMAKE_CXX_FLAGS_RELEASE", format!("-DNDEBUG {}", additional_cxx_flags));
            
    } else {
        // Linux Configuration (Default /usr/local)
        cmake_config
            .define("TBB_ROOT", "/usr/local")
            .define("TBB_INCLUDE_DIR", "/usr/local/include")
            // Linux often needs the explicit .so path defined for OSRM cmake
            .define("TBB_LIBRARY", "/usr/local/lib/libtbb.so")
            .define("CMAKE_CXX_FLAGS_RELEASE", "-DNDEBUG");
    }

    // Apply combined CXX flags
    cmake_config.env("CXXFLAGS", &cxx_flags);
    
    // 3. Build with CMake
    let dst = cmake_config.build();

    // 4. Compile the Rust/C++ Wrapper
    let build_dir = out_dir.join("build");
    let mut build = cc::Build::new();
    build
        .cpp(true)
        .file("src/wrapper.cpp")
        .flag("-std=c++20")
        .include(dst.join("include"))
        .include(osrm_source_path.join("include"))
        .include(build_dir.join("include"))  // Generated files like util/version.hpp
        .include(osrm_source_path.join("third_party/fmt/include"))
        .include(osrm_source_path.join("third_party/libosmium/include"))
        .include(osrm_source_path.join("third_party/sol2/include"))
        .include(osrm_source_path.join("third_party/rapidjson/include"))
        .include(osrm_source_path.join("third_party/protozero/include"))
        .include(osrm_source_path.join("third_party/vtzero/include"))
        .define("FMT_HEADER_ONLY", None)
        .define("OSRM_PROJECT_DIR", format!("\"{}\"", osrm_source_path.to_str().unwrap()).as_str());

    if target_os == "macos" {
        build
            .include("/opt/homebrew/opt/boost@1.85/include")
            .include("/opt/homebrew/opt/tbb/include")
            .include("/opt/homebrew/opt/libosmium/include")
            .include("/opt/homebrew/opt/lua/include/lua");
    } else {
        build
            .include("/usr/local/include");
    }

    // Disable specific warnings that cause errors with Boost 1.85 and modern compilers
    build
        .flag("-Wno-error")
        .flag("-Wno-missing-template-arg-list-after-template-kw")
        .flag("-Wno-unused-parameter")
        .flag("-Wno-deprecated-copy");

    build.compile("osrm_wrapper");

    // 5. Linking Configuration
    let lib_path = dst.join("lib");
    println!("cargo:rustc-link-search=native={}", lib_path.display());

    if target_os == "macos" {
        // Mac-specific search paths
        println!("cargo:rustc-link-search=native=/opt/homebrew/opt/boost@1.85/lib");
        println!("cargo:rustc-link-search=native=/opt/homebrew/opt/tbb/lib");
        println!("cargo:rustc-link-search=native=/opt/homebrew/opt/lua/lib");
        println!("cargo:rustc-link-search=native=/opt/homebrew/lib");
    }

    // Static OSRM Libs
    let osrm_libs = [
        "osrm_wrapper", "osrm", "osrm_store", "osrm_extract",
        "osrm_partition", "osrm_update", "osrm_guidance",
        "osrm_customize", "osrm_contract"
    ];
    for lib in osrm_libs {
        println!("cargo:rustc-link-lib=static={}", lib);
    }

    // Dynamic Libs
    println!("cargo:rustc-link-lib=dylib=boost_thread");
    println!("cargo:rustc-link-lib=dylib=boost_filesystem");
    println!("cargo:rustc-link-lib=dylib=boost_iostreams");
    println!("cargo:rustc-link-lib=dylib=tbb");
    println!("cargo:rustc-link-lib=dylib=fmt");
    println!("cargo:rustc-link-lib=dylib=lua");

    // Platform specific C++ std lib
    if target_os == "macos" {
        println!("cargo:rustc-link-lib=dylib=c++");
    } else {
        println!("cargo:rustc-link-lib=dylib=stdc++");
    }

    // Common system libs
    println!("cargo:rustc-link-lib=dylib=z");
    println!("cargo:rustc-link-lib=dylib=bz2");
    println!("cargo:rustc-link-lib=dylib=expat");
}

fn find_osrm_source(path: &Path) -> PathBuf {
    for entry in path.read_dir().expect("Failed to read directory") {
        let entry = entry.expect("Failed to read directory entry");
        let path = entry.path();
        if path.is_dir() && path.file_name().unwrap().to_str().unwrap().starts_with("osrm-backend-") {
            return path;
        }
    }
    panic!("Could not find OSRM source directory");
}
