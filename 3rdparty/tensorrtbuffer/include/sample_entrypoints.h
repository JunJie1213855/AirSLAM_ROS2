/*
 * SPDX-FileCopyrightText: Copyright (c) 1993-2023 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
 * SPDX-License-Identifier: Apache-2.0
 *
 * Modified for TensorRT 10.x: removed NvCaffeParser, NvUffParser (removed in TRT 10.x).
 */

#ifndef TRT_SAMPLE_ENTRYPOINTS_H
#define TRT_SAMPLE_ENTRYPOINTS_H

#include "NvInfer.h"
#include "NvOnnxParser.h"
#include "logger.h"

extern nvinfer1::IBuilder* createBuilder();
extern nvinfer1::IRuntime* createRuntime();
extern nvinfer1::IRefitter* createRefitter(nvinfer1::ICudaEngine& engine);
extern nvonnxparser::IParser* createONNXParser(nvinfer1::INetworkDefinition& network);

#if !defined(DEFINE_TRT_ENTRYPOINTS)
#define DEFINE_TRT_ENTRYPOINTS 0
#endif

#if !defined(DEFINE_TRT_BUILDER_ENTRYPOINT)
#define DEFINE_TRT_BUILDER_ENTRYPOINT 1
#endif
#if !defined(DEFINE_TRT_RUNTIME_ENTRYPOINT)
#define DEFINE_TRT_RUNTIME_ENTRYPOINT 1
#endif
#if !defined(DEFINE_TRT_REFITTER_ENTRYPOINT)
#define DEFINE_TRT_REFITTER_ENTRYPOINT 1
#endif
#if !defined(DEFINE_TRT_ONNX_PARSER_ENTRYPOINT)
#define DEFINE_TRT_ONNX_PARSER_ENTRYPOINT 1
#endif

#if DEFINE_TRT_ENTRYPOINTS
nvinfer1::IBuilder* createBuilder()
{
#if DEFINE_TRT_BUILDER_ENTRYPOINT
    return nvinfer1::createInferBuilder(sample::gLogger.getTRTLogger());
#else
    return {};
#endif
}

nvinfer1::IRuntime* createRuntime()
{
#if DEFINE_TRT_RUNTIME_ENTRYPOINT
    return nvinfer1::createInferRuntime(sample::gLogger.getTRTLogger());
#else
    return {};
#endif
}

nvinfer1::IRefitter* createRefitter(nvinfer1::ICudaEngine& engine)
{
#if DEFINE_TRT_REFITTER_ENTRYPOINT
    return nvinfer1::createInferRefitter(engine, sample::gLogger.getTRTLogger());
#else
    return {};
#endif
}

nvonnxparser::IParser* createONNXParser(nvinfer1::INetworkDefinition& network)
{
#if DEFINE_TRT_ONNX_PARSER_ENTRYPOINT
    return nvonnxparser::createParser(network, sample::gLogger.getTRTLogger());
#else
    return {};
#endif
}

#endif // DEFINE_TRT_ENTRYPOINTS

#endif // TRT_SAMPLE_ENTRYPOINTS_H
