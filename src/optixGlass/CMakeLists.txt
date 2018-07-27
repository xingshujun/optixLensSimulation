include_directories(${SAMPLES_INCLUDE_DIR})

# See top level CMakeLists.txt file for documentation of OPTIX_add_sample_executable.
OPTIX_add_sample_executable( optixGlass
    optixGlass.cpp

    path_trace_camera.cu
    diffuse.cu
    glass.cu
    gradientbg.cu
    parallelogram_iterative.cu
    prd.h
    triangle_mesh.cu

	particalSphere.cu

	

    # common headers
    ${SAMPLES_INCLUDE_DIR}/commonStructs.h
    ${SAMPLES_INCLUDE_DIR}/helpers.h
    ${SAMPLES_INCLUDE_DIR}/random.h
    )

