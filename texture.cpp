#include "texture.h"

Vec4f get_texture2DLod(Texture4f* texture, float u, float v, float mipLevel) {
    if (mipLevel > 0) {
        int mipLevels[2];
        mipLevels[0] = std::floorf(mipLevel);
        mipLevels[1] = std::ceilf(mipLevel);
        assert(texture->maxMipLevel > mipLevels[1]);

        Vec4f mip_v[2];
        for (int i = 0; i < 2; i++)
        {
            int width = texture->width * std::pow(0.5, mipLevels[i]);
            int height = texture->height * std::pow(0.5, mipLevels[i]);

            float x = clamp(u * width, 0, width - 1);
            float y = clamp(v * height, 0, height - 1);

            int x0 = std::floorf(x);
            int y0 = std::floorf(y);
            int x1 = clamp(std::ceilf(x), 0, width - 1);
            int y1 = clamp(std::ceilf(y), 0, height - 1);

            Vec4f v00 = texture->mipmap_datas[mipLevels[i]][y0 * width + x0];
            Vec4f v10 = texture->mipmap_datas[mipLevels[i]][y0 * width + x1];
            Vec4f v01 = texture->mipmap_datas[mipLevels[i]][y1 * width + x0];
            Vec4f v11 = texture->mipmap_datas[mipLevels[i]][y1 * width + x1];

            float dx = x - x0;
            float dy = y - y0;
            Vec4f d0 = (1 - dx) * v00 + dx * v10;
            Vec4f d1 = (1 - dx) * v01 + dx * v11;
            mip_v[i] = (1 - dy) * d0 + dy * d1;
        }

        float dm = mipLevel - mipLevels[0];
        return (1 - dm) * mip_v[0] + dm * mip_v[1];
    }
    else {
        return get_texture2D(texture, u, v);
    }
}

Vec4f get_texture2D(Texture4f* texture, float u, float v) {
    int width = texture->width;
    int height = texture->height;

    float x = clamp(u * width, 0, width - 1);
    float y = clamp(v * height, 0, height - 1);

    int x0 = std::floorf(x);
    int y0 = std::floorf(y);
    int x1 = clamp(std::ceilf(x), 0, width - 1);
    int y1 = clamp(std::ceilf(y), 0, height - 1);

    Vec4f v00 = texture->data[y0 * width + x0];
    Vec4f v10 = texture->data[y0 * width + x1];
    Vec4f v01 = texture->data[y1 * width + x0];
    Vec4f v11 = texture->data[y1 * width + x1];

    float dx = x - x0;
    float dy = y - y0;
    Vec4f d0 = (1 - dx) * v00 + dx * v10;
    Vec4f d1 = (1 - dx) * v01 + dx * v11;
    return (1 - dy) * d0 + dy * d1;
}

Vec3f get_texture2D(Texture3f* texture, float u, float v) {
    int width = texture->width;
    int height = texture->height;

    float x = clamp(u * width, 0, width - 1);
    float y = clamp(v * height, 0, height - 1);

    int x0 = std::floorf(x);
    int y0 = std::floorf(y);
    int x1 = clamp(std::ceilf(x), 0, width - 1);
    int y1 = clamp(std::ceilf(y), 0, height - 1);

    Vec3f v00 = texture->data[y0 * width + x0];
    Vec3f v10 = texture->data[y0 * width + x1];
    Vec3f v01 = texture->data[y1 * width + x0];
    Vec3f v11 = texture->data[y1 * width + x1];

    float dx = x - x0;
    float dy = y - y0;
    Vec3f d0 = (1 - dx) * v00 + dx * v10;
    Vec3f d1 = (1 - dx) * v01 + dx * v11;
    return (1 - dy) * d0 + dy * d1;
}

Vec4f get_textureCubeLod(TextureCube<Vec4f>* texture_cube, Vec3f text, float mipLevel) {
    int idx;
    float u, v;
    texture_cube->get_idx_and_uv(text[0], text[1], text[2], idx, u, v);
    return get_texture2DLod(texture_cube->textures[idx], u, v, mipLevel);
}

Vec4f get_textureCube(TextureCube<Vec4f>* texture_cube, Vec3f text) {
    int idx;
    float u, v;
    texture_cube->get_idx_and_uv(text[0], text[1], text[2], idx, u, v);
    return get_texture2D(texture_cube->textures[idx], u, v);
}

void generateMipmap(Texture4f* texture, int maxMipLevel) {
    texture->maxMipLevel = maxMipLevel;
    texture->mipmap_datas.resize(maxMipLevel);
    texture->mipmap_datas[0] = texture->data;
    int pre_width = texture->width;//上一层的width可能是奇数
    for (int mip = 1; mip < maxMipLevel; mip++)
    {
        int mipWidth = texture->width * std::pow(0.5, mip);
        int mipHeight = texture->height * std::pow(0.5, mip);
        texture->mipmap_datas[mip] = new Vec4f[mipWidth * mipHeight];
        for (int x = 0; x < mipWidth; x++)
        {
            for (int y = 0; y < mipHeight; y++)
            {
                Vec4f v = Vec4f::Zero();
                for (int i = 0; i < 2; i++)
                {
                    for (int j = 0; j < 2; j++)
                    {
                        int pre_idx = (y * 2 + i) * pre_width + x * 2 + j;
                        v += texture->mipmap_datas[mip - 1][pre_idx];
                    }
                }
                int idx = y * mipWidth + x;
                texture->mipmap_datas[mip][idx] = v / 4;
            }
        }
        pre_width = mipWidth;
    }
}

void generateMipmapWithoutInit(Texture4f* texture, int maxMipLevel) {
    texture->maxMipLevel = maxMipLevel;
    texture->mipmap_datas.resize(maxMipLevel);
    texture->mipmap_datas[0] = texture->data;
    for (int mip = 1; mip < maxMipLevel; mip++)
    {
        int mipWidth = texture->width * std::pow(0.5, mip);
        int mipHeight = texture->height * std::pow(0.5, mip);
        texture->mipmap_datas[mip] = new Vec4f[mipWidth * mipHeight];
    }
}

void generateMipmap(TextureCube<Vec4f>* texture_cube, int maxMipLevel) {
    for (int i = 0; i < 6; i++)
    {
        generateMipmap(texture_cube->textures[i], maxMipLevel);
    }
}

void generateMipmapWithoutInit(TextureCube<Vec4f>* texture_cube, int maxMipLevel) {
    for (int i = 0; i < 6; i++)
    {
        generateMipmapWithoutInit(texture_cube->textures[i], maxMipLevel);
    }
}