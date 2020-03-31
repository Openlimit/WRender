#pragma once
#include "geometry.h"

template<class T>
struct Texture{
    Texture(int width, int height, int depth = 1) :width(width), height(height), depth(depth)
    {
        data = new T[width * height * depth];
    }

    virtual ~Texture() {
        if (data != nullptr)
            delete data;
    }

    T& operator[](int idx) {
        return data[idx];
    }

    T get_by_uv(float u, float v, float s = 0)
    {
        if (u < 0 || u>1 || v < 0 || v>1 || s < 0 || s>1)
            return T(0);
        int x = u * width;
        int y = v * height;
        int z = s * depth;
        int idx = (y * width + x) * depth + z;
        return data[idx];
    }

    T get(int x, int y, int z = 0) {
        if (x < 0 || x >= width || y < 0 || y >= height || z < 0 || z >= depth)
            return T(0);
        int idx = (y * width + x) * depth + z;
        return data[idx];
    }

    T get(int idx) {
        return data[idx];
    }

    void set(int x, int y, int z, T v) {
        int idx = (y * width + x) * depth + z;
        data[idx] = v;
    }

    void set(int idx, T v) {
        data[idx] = v;
    }

    void clear() {
        memset(data, 0, sizeof(T) * width * height * depth);
    }

    void clear(T v) {
        for (int i = 0; i < width * height * depth; i++)
        {
            data[i] = v;
        }
    }

    Vec3f texel_size() {
        return Vec3f(1. / width, 1. / height, 1. / depth);
    }

    void copyTo(Texture<T>* other) {
        assert(width == other->width && height == other->height && depth == other->height);
        memcpy(other->data, data, sizeof(T) * width * height * depth);
    }

    int width;
    int height;
    int depth;
    T* data;
};

typedef Texture<float> Texture1f;
typedef Texture<Vec3f> Texture3f;
typedef Texture<Vec4f> Texture4f;
typedef Texture<Vec4u> Texture4u;
typedef Texture<bool> Texture1b;