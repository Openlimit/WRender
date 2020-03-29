#pragma once
struct Texture {
    Texture(int width, int height) :width(width), height(height)
    {
        data = new float[width * height];
    }

    virtual ~Texture() {
        if (data != nullptr)
            delete data;
    }

    float get(float u, float v)
    {
        if (u < 0 || u>1 || v < 0 || v>1)
            return 0;
        int x = u * width;
        int y = v * height;
        int idx = y * width + x;
        return data[idx];
    }

    Vec2f texel_size() {
        return Vec2f(1. / width, 1. / height);
    }

    int width;
    int height;
    float* data;
 };