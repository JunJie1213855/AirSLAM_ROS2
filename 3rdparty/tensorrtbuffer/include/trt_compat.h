#ifndef AIRSLAM_TRT_COMPAT_H
#define AIRSLAM_TRT_COMPAT_H

#include <NvInfer.h>
#include <memory>

namespace tensorrt_buffer {
inline uint32_t networkCreationFlags() {
#if NV_TENSORRT_MAJOR < 11
    return 1U << static_cast<uint32_t>(nvinfer1::NetworkDefinitionCreationFlag::kEXPLICIT_BATCH);
#else
    return 0U;  // TensorRT 11 networks are always explicit-batch and strongly typed.
#endif
}

inline void configureModelPrecision(nvinfer1::IBuilderConfig& config) {
#if NV_TENSORRT_MAJOR < 11
    config.setFlag(nvinfer1::BuilderFlag::kFP16);
#else
    // Preserve ONNX tensor types. FP16 requires an explicitly converted ONNX model.
    (void)config;
#endif
}

// TensorRT requires the runtime to outlive every engine it deserializes.
inline std::shared_ptr<nvinfer1::ICudaEngine> deserializeEngine(
        nvinfer1::IRuntime* runtime, const void* data, size_t size) {
    std::shared_ptr<nvinfer1::IRuntime> owner(runtime);
    if (!owner) return {};
    auto* engine = owner->deserializeCudaEngine(data, size);
    if (!engine) return {};
    return std::shared_ptr<nvinfer1::ICudaEngine>(
        engine, [owner](nvinfer1::ICudaEngine* value) { delete value; });
}
}  // namespace tensorrt_buffer
#endif
