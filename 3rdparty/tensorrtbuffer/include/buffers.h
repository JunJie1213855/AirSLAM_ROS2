/*
 * SPDX-FileCopyrightText: Copyright (c) 1993-2023 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
 * SPDX-License-Identifier: Apache-2.0
 *
 * Modified for TensorRT 10.x compatibility:
 *   getNbBindings       → getNbIOTensors
 *   getBindingIndex     → tensor-name lookup (getIOTensorName loop)
 *   getBindingDimensions→ getTensorShape
 *   getBindingDataType  → getTensorDataType
 *   getBindingVectorizedDim → getTensorVectorizedDim
 *   getBindingComponentsPerElement → getTensorComponentsPerElement
 *   bindingIsInput      → getTensorIOMode == TensorIOMode::kINPUT
 */
#ifndef TENSORRT_BUFFERS_H
#define TENSORRT_BUFFERS_H

#include "NvInfer.h"
#include "common.h"
#include "half.h"
#include <cassert>
#include <cstring>
#include <cuda_runtime_api.h>
#include <iostream>
#include <iterator>
#include <memory>
#include <new>
#include <numeric>
#include <string>
#include <unordered_map>
#include <vector>

namespace tensorrt_buffer
{

template <typename AllocFunc, typename FreeFunc>
class GenericBuffer
{
public:
    GenericBuffer(nvinfer1::DataType type = nvinfer1::DataType::kFLOAT)
            : mSize(0), mCapacity(0), mType(type), mBuffer(nullptr) {}

    GenericBuffer(size_t size, nvinfer1::DataType type)
            : mSize(size), mCapacity(size), mType(type) {
        if (!allocFn(&mBuffer, this->nbBytes())) throw std::bad_alloc();
    }

    GenericBuffer(GenericBuffer&& buf)
            : mSize(buf.mSize), mCapacity(buf.mCapacity), mType(buf.mType), mBuffer(buf.mBuffer) {
        buf.mSize = 0; buf.mCapacity = 0; buf.mType = nvinfer1::DataType::kFLOAT; buf.mBuffer = nullptr;
    }

    GenericBuffer& operator=(GenericBuffer&& buf) {
        if (this != &buf) {
            freeFn(mBuffer);
            mSize = buf.mSize; mCapacity = buf.mCapacity; mType = buf.mType; mBuffer = buf.mBuffer;
            buf.mSize = 0; buf.mCapacity = 0; buf.mBuffer = nullptr;
        }
        return *this;
    }

    void* data() { return mBuffer; }
    const void* data() const { return mBuffer; }
    size_t size() const { return mSize; }
    size_t nbBytes() const { return this->size() * tensorrt_buffer::getElementSize(mType); }

    void resize(size_t newSize) {
        mSize = newSize;
        if (mCapacity < newSize) {
            freeFn(mBuffer);
            if (!allocFn(&mBuffer, this->nbBytes())) throw std::bad_alloc{};
            mCapacity = newSize;
        }
    }

    void resize(const nvinfer1::Dims& dims) { return this->resize(tensorrt_buffer::volume(dims)); }
    ~GenericBuffer() { freeFn(mBuffer); }

private:
    size_t mSize{0}, mCapacity{0};
    nvinfer1::DataType mType;
    void* mBuffer;
    AllocFunc allocFn;
    FreeFunc freeFn;
};

class DeviceAllocator { public: bool operator()(void** ptr, size_t size) const { return cudaMalloc(ptr, size) == cudaSuccess; } };
class DeviceFree { public: void operator()(void* ptr) const { cudaFree(ptr); } };
class HostAllocator { public: bool operator()(void** ptr, size_t size) const { *ptr = malloc(size); return *ptr != nullptr; } };
class HostFree { public: void operator()(void* ptr) const { free(ptr); } };

using DeviceBuffer = GenericBuffer<DeviceAllocator, DeviceFree>;
using HostBuffer = GenericBuffer<HostAllocator, HostFree>;

class ManagedBuffer {
public:
    DeviceBuffer deviceBuffer;
    HostBuffer hostBuffer;
};

//! BufferManager — updated for TensorRT 10.x tensor-name API
class BufferManager
{
public:
    static const size_t kINVALID_SIZE_VALUE = ~size_t(0);

    BufferManager(std::shared_ptr<nvinfer1::ICudaEngine> engine, const int batchSize = 0,
                  const nvinfer1::IExecutionContext* context = nullptr)
            : mEngine(engine), mBatchSize(batchSize)
    {
        assert(engine->hasImplicitBatchDimension() || mBatchSize == 0);
        for (int32_t i = 0; i < mEngine->getNbIOTensors(); i++)
        {
            auto const* tensorName = mEngine->getIOTensorName(i);
            auto dims = context ? context->getTensorShape(tensorName) : mEngine->getTensorShape(tensorName);
            size_t vol = context || !mBatchSize ? 1 : static_cast<size_t>(mBatchSize);
            nvinfer1::DataType type = mEngine->getTensorDataType(tensorName);
            int32_t vecDim = mEngine->getTensorVectorizedDim(tensorName);
            if (-1 != vecDim) {
                int32_t scalarsPerVec = mEngine->getTensorComponentsPerElement(tensorName);
                dims.d[vecDim] = divUp(dims.d[vecDim], scalarsPerVec);
                vol *= scalarsPerVec;
            }
            vol *= tensorrt_buffer::volume(dims);
            std::unique_ptr<ManagedBuffer> manBuf{new ManagedBuffer()};
            manBuf->deviceBuffer = DeviceBuffer(vol, type);
            manBuf->hostBuffer = HostBuffer(vol, type);
            mDeviceBindings.emplace_back(manBuf->deviceBuffer.data());
            mManagedBuffers.emplace_back(std::move(manBuf));
            mTensorNameToIndex[std::string(tensorName)] = i;
        }
    }

    std::vector<void*>& getDeviceBindings() { return mDeviceBindings; }
    const std::vector<void*>& getDeviceBindings() const { return mDeviceBindings; }

    void* getDeviceBuffer(const std::string& tensorName) const { return getBuffer(false, tensorName); }
    void* getHostBuffer(const std::string& tensorName) const { return getBuffer(true, tensorName); }

    size_t size(const std::string& tensorName) const {
        auto it = mTensorNameToIndex.find(tensorName);
        if (it == mTensorNameToIndex.end()) return kINVALID_SIZE_VALUE;
        return mManagedBuffers[it->second]->hostBuffer.nbBytes();
    }

    template <typename T>
    void print(std::ostream& os, void* buf, size_t bufSize, size_t rowCount) {
        assert(rowCount != 0);
        assert(bufSize % sizeof(T) == 0);
        T* typedBuf = static_cast<T*>(buf);
        size_t numItems = bufSize / sizeof(T);
        for (int i = 0; i < static_cast<int>(numItems); i++) {
            if (rowCount == 1 && i != static_cast<int>(numItems) - 1) os << typedBuf[i] << std::endl;
            else if (rowCount == 1) os << typedBuf[i];
            else if (i % rowCount == 0) os << typedBuf[i];
            else if (i % rowCount == rowCount - 1) os << " " << typedBuf[i] << std::endl;
            else os << " " << typedBuf[i];
        }
    }

    void copyInputToDevice() { memcpyBuffers(true, false, false); }
    void copyOutputToHost() { memcpyBuffers(false, true, false); }
    void copyInputToDeviceAsync(const cudaStream_t& stream = 0) { memcpyBuffers(true, false, true, stream); }
    void copyOutputToHostAsync(const cudaStream_t& stream = 0) { memcpyBuffers(false, true, true, stream); }
    ~BufferManager() = default;

private:
    void* getBuffer(const bool isHost, const std::string& tensorName) const {
        auto it = mTensorNameToIndex.find(tensorName);
        if (it == mTensorNameToIndex.end()) return nullptr;
        return (isHost ? mManagedBuffers[it->second]->hostBuffer.data()
                       : mManagedBuffers[it->second]->deviceBuffer.data());
    }

    void memcpyBuffers(const bool copyInput, const bool deviceToHost, const bool async,
                       const cudaStream_t& stream = 0) {
        for (int32_t i = 0; i < mEngine->getNbIOTensors(); i++) {
            auto const* tensorName = mEngine->getIOTensorName(i);
            auto ioMode = mEngine->getTensorIOMode(tensorName);
            void* dstPtr = deviceToHost ? mManagedBuffers[i]->hostBuffer.data()
                                        : mManagedBuffers[i]->deviceBuffer.data();
            const void* srcPtr = deviceToHost ? mManagedBuffers[i]->deviceBuffer.data()
                                              : mManagedBuffers[i]->hostBuffer.data();
            const size_t byteSize = mManagedBuffers[i]->hostBuffer.nbBytes();
            const cudaMemcpyKind memcpyType = deviceToHost ? cudaMemcpyDeviceToHost : cudaMemcpyHostToDevice;
            if ((copyInput && ioMode == nvinfer1::TensorIOMode::kINPUT) ||
                (!copyInput && ioMode == nvinfer1::TensorIOMode::kOUTPUT)) {
                if (async) CHECK(cudaMemcpyAsync(dstPtr, srcPtr, byteSize, memcpyType, stream));
                else       CHECK(cudaMemcpy(dstPtr, srcPtr, byteSize, memcpyType));
            }
        }
    }

    std::shared_ptr<nvinfer1::ICudaEngine> mEngine;
    int mBatchSize;
    std::vector<std::unique_ptr<ManagedBuffer>> mManagedBuffers;
    std::vector<void*> mDeviceBindings;
    std::unordered_map<std::string, int32_t> mTensorNameToIndex;
};

} // namespace tensorrt_buffer
#endif // TENSORRT_BUFFERS_H
