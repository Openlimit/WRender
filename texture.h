#pragma once
#include "geometry.h"

template<class T>
struct Texture{
    Texture(int width, int height, int depth = 1) :width(width), height(height), depth(depth), maxMipLevel(0)
    {
        data = new T[width * height * depth];
    }

    virtual ~Texture() {
        if (data != nullptr)
            delete data;
        if (maxMipLevel > 0) {
            for (int i = 1; i < mipmap_datas.size(); i++)
            {
                delete mipmap_datas[i];
            }
        }
    }

    T& operator[](int idx) {
        return data[idx];
    }

    T get_by_uv(float u, float v, float s = 0)
    {
        int x = clamp(u * width, 0, width - 1);
        int y = clamp(v * height, 0, height - 1);
        int z = clamp(s * depth, 0, depth - 1);
        int idx = (y * width + x) * depth + z;
        return data[idx];
    }

    T get(int x, int y, int z = 0) {
        x = clamp(x, 0, width - 1);
        y = clamp(y, 0, height - 1);
        z = clamp(z, 0, depth - 1);
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
        assert(width == other->width && height == other->height && depth == other->depth);
        memcpy(other->data, data, sizeof(T) * width * height * depth);
    }

    void initMipmap_from_data(T* _data, int mipLevel) {
        assert(maxMipLevel > mipLevel);
        int mipWidth = width * std::pow(0.5, mipLevel);
        int mipHeight = height * std::pow(0.5, mipLevel);
        memcpy(mipmap_datas[mipLevel], _data, sizeof(T) * mipWidth * mipHeight);
    }

    void init_from_data(T* _data) {
        memcpy(data, _data, sizeof(T) * width * height * depth);
    }

    int width;
    int height;
    int depth;
    T* data;

    int maxMipLevel;
    std::vector<T*> mipmap_datas;
};

template<class T>
struct TextureCube {
    int width;
    int height;
    int depth;
    Texture<T>* textures[6];

    TextureCube(int width, int height, int depth = 1) :width(width), height(height), depth(depth)
    {
        for (int i = 0; i < 6; i++)
        {
            textures[i] = new Texture<T>(width, height, depth);
        }
    }

    virtual ~TextureCube() {
        for (int i = 0; i < 6; i++)
        {
            if (textures[i] != nullptr)
                delete textures[i];
        }
    }

    void init_from_data(T* data, int i) {
        textures[i]->init_from_data(data);
    }

    T get(float x, float y, float z) {
        float xa = std::abs(x);
        float ya = std::abs(y);
        float za = std::abs(z);
        float mag = std::fmax(std::fmax(xa, ya), za);
        if (mag == 0)
            return T(0);

        int idx = 0;
        float s, t;
        if (mag == xa)
        {
            if (x > 0) {
                s = -z;
                t = -y;
                idx = 0;
            }
            else {
                s = z;
                t = -y;
                idx = 1;
            }
        }
        else if (mag == ya)
        {
            if (y > 0) {
                s = x;
                t = z;
                idx = 2;
            }
            else {
                s = x;
                t = -z;
                idx = 3;
            }
        }
        else if (mag == za)
        {
            if (z > 0) {
                s = x;
                t = -y;
                idx = 4;
            }
            else {
                s = -x;
                t = -y;
                idx = 5;
            }
        }

        s = (s / mag + 1) / 2;
        t = (t / mag + 1) / 2;
        return textures[idx]->get_by_uv(s, t);
    }

    void get_idx_and_uv(float x, float y, float z, int &idx, float &u, float &v) {
        float xa = std::abs(x);
        float ya = std::abs(y);
        float za = std::abs(z);
        float mag = std::fmax(std::fmax(xa, ya), za);
        if (mag == 0)
            return;

        float s, t;
        if (mag == xa)
        {
            if (x > 0) {
                s = -z;
                t = -y;
                idx = 0;
            }
            else {
                s = z;
                t = -y;
                idx = 1;
            }
        }
        else if (mag == ya)
        {
            if (y > 0) {
                s = x;
                t = z;
                idx = 2;
            }
            else {
                s = x;
                t = -z;
                idx = 3;
            }
        }
        else if (mag == za)
        {
            if (z > 0) {
                s = x;
                t = -y;
                idx = 4;
            }
            else {
                s = -x;
                t = -y;
                idx = 5;
            }
        }

        u = (s / mag + 1) / 2;
        v = (t / mag + 1) / 2;
    }
};

typedef Texture<float> Texture1f;
typedef Texture<Vec3f> Texture3f;
typedef Texture<Vec3u> Texture3u;
typedef Texture<Vec4f> Texture4f;
typedef Texture<Vec4u> Texture4u;
typedef Texture<bool> Texture1b;

Vec4f get_textureCube(TextureCube<Vec4f>* texture_cube, Vec3f text);
Vec4f get_textureCubeLod(TextureCube<Vec4f>* texture_cube, Vec3f text, float mipLevel);
Vec3f get_texture2D(Texture3f* texture, float u, float v);
Vec4f get_texture2D(Texture4f* texture, float u, float v);
Vec4f get_texture2DLod(Texture4f* texture, float u, float v, float mipLevel);

void generateMipmap(Texture4f* texture, int _maxMipLevel);
void generateMipmap(TextureCube<Vec4f>* texture_cube, int maxMipLevel);
void generateMipmapWithoutInit(Texture4f* texture, int maxMipLevel);
void generateMipmapWithoutInit(TextureCube<Vec4f>* texture_cube, int maxMipLevel);
