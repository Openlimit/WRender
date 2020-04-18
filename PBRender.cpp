#include "PBRender.h"
#include <stb_image.h>
#define STB_IMAGE_WRITE_IMPLEMENTATION
#include <stb_image_write.h>

void PBRender::init(int width, int height) {
    frame_width = width;
    frame_height = height;
    rotate_speed = 2;
    dnear = 0.1;
    dfar = 100;

    renderer = new Renderer(dnear, dfar, width, height, 0, 0, false);

    model_mat = Mat4f::Identity();
    project_mat = perspective(60. * PI / 180, (float)width / (float)height, dnear, dfar);

    Vec3f camera_pos(0, 0, 10);
    Vec3f lookatpos(0, 0, 0);
    Vec3f up(0, 1, 0);
    camera = new Camera(camera_pos, lookatpos, up);
    view_mat = camera->get_view();

    init_sphere();
    shader = new PBRShader();
    shader->model_mat = model_mat;
    shader->view_mat = view_mat;
    shader->project_mat = project_mat;
    shader->model_mat_IT = model_mat.inverse().transpose();
    shader->albedo = Vec3f(0.5, 0, 0);
    shader->ao = 1;
    shader->viewPos = camera->get_camera_pos();

    std::vector<Light*> lights;
    PointLight* light1 = new PointLight();
    light1->position = Vec3f(-10.0f, 10.0f, 10.0f);
    light1->color = Vec3f(300.0f, 300.0f, 300.0f);
    lights.emplace_back(light1);

    PointLight* light2 = new PointLight();
    light2->position = Vec3f(10.0f, 10.0f, 10.0f);
    light2->color = Vec3f(300.0f, 300.0f, 300.0f);
    lights.emplace_back(light2);

    PointLight* light3 = new PointLight();
    light3->position = Vec3f(-10.0f, -10.0f, 10.0f);
    light3->color = Vec3f(300.0f, 300.0f, 300.0f);
    lights.emplace_back(light3);

    PointLight* light4 = new PointLight();
    light4->position = Vec3f(10.0f, -10.0f, 10.0f);
    light4->color = Vec3f(300.0f, 300.0f, 300.0f);
    lights.emplace_back(light4);

    shader->lights = lights;

    enviroment_model = new Model("obj/enviroment_model.obj");
    skybox_model = new Model("obj/skybox_model.obj");
    deffered_model = new Model("obj/deffered_model.obj");

    maxMipLevels = 5;

    init_enviroment_map();

    init_irradiance_map();

    init_prefilter_map();

    init_LUT_map();

    skyboxShader = new EnviromentBoxShader();
    skyboxShader->view_mat = Mat4f::Identity();
    skyboxShader->view_mat.topLeftCorner(3, 3) = view_mat.topLeftCorner(3, 3);
    skyboxShader->project_mat = perspective(90 * PI / 180, (float)width / (float)height, dnear, dfar);
    skyboxShader->skybox = enviroment_map_cube;

    shader->irradiance_map_cube = irradiance_map_cube;
    shader->prefilter_map_cube = prefilter_map_cube;
    shader->LUT_map = LUT_map;
}

void PBRender::release() {
	if (model != nullptr)
		delete model;
    if (enviroment_model != nullptr)
        delete enviroment_model;
    if (skybox_model != nullptr)
        delete skybox_model;
    if (deffered_model != nullptr)
        delete deffered_model;
	if (camera != nullptr)
		delete camera;
    if (enviroment_map_hdr != nullptr)
        delete enviroment_map_hdr;
    if (enviroment_map_cube != nullptr)
        delete enviroment_map_cube;
    if (irradiance_map_cube != nullptr)
        delete irradiance_map_cube;
    if (prefilter_map_cube != nullptr)
        delete prefilter_map_cube;
    if (LUT_map != nullptr)
        delete LUT_map;
    if (renderer != nullptr)
        delete renderer;
    if (shader != nullptr)
        delete shader;
    if (skyboxShader != nullptr)
        delete skyboxShader;
}

void PBRender::resize(int width, int height) {
	if (renderer != nullptr)
		delete renderer;
	renderer = new Renderer(dnear, dfar, width, height, 0, 0, false);

	frame_width = width;
	frame_height = height;
}

void PBRender::render() {
    renderer->clear_colorbuffer();
    renderer->clear_zbuffer();

    int nrRows = 5;
    int nrColumns = 5;
    float spacing = 2.5;
    for (int row = 0; row < nrRows; ++row)
    {
        shader->metallic = (float)row / (float)nrRows;
        for (int col = 0; col < nrColumns; ++col)
        {
            shader->roughness = clamp((float)col / (float)nrColumns, 0.05f, 1.0f);
            model_mat(0, 3) = (col - (nrColumns / 2)) * spacing;
            model_mat(1, 3) = (row - (nrRows / 2)) * spacing;
            shader->model_mat = model_mat;
            shader->model_mat_IT = model_mat.inverse().transpose();
            renderer->render(model, shader, screenBits);
        }
    }

    renderer->render(skybox_model, skyboxShader, screenBits);
}

void PBRender::update_view() {
	view_mat = camera->get_view();

    shader->view_mat = view_mat;
    shader->viewPos = camera->get_camera_pos();

    skyboxShader->view_mat.topLeftCorner(3, 3) = view_mat.topLeftCorner(3, 3);
}

void PBRender::init_sphere() {
	model = new Model();

    const int X_SEGMENTS = 20;
    const int Y_SEGMENTS = 20;
    for (int y = 0; y <= Y_SEGMENTS; ++y)
    {
        for (int x = 0; x <= X_SEGMENTS; ++x)
        {
            float xSegment = (float)x / (float)X_SEGMENTS;
            float ySegment = (float)y / (float)Y_SEGMENTS;
            float xPos = std::cos(xSegment * 2.0 * PI) * std::sin(ySegment * PI);
            float yPos = std::cos(ySegment * PI);
            float zPos = std::sin(xSegment * 2.0 * PI) * std::sin(ySegment * PI);

            model->add_vert(Vec3f(xPos, yPos, zPos));
            model->add_text(Vec3f(xSegment, ySegment, 0));
            model->add_normal(Vec3f(xPos, yPos, zPos));
        }
    }

    bool oddRow = false;
    for (int y = 0; y < Y_SEGMENTS; ++y)
    {
        if (!oddRow) // even rows: y == 0, y == 2; and so on
        {
            for (int x = 1; x <= X_SEGMENTS; ++x)
            {
                int n0 = y * (X_SEGMENTS + 1) + x - 1;
                int n1 = (y + 1) * (X_SEGMENTS + 1) + x - 1;
                int n2 = y * (X_SEGMENTS + 1) + x;
                int n3 = (y + 1) * (X_SEGMENTS + 1) + x;

                Vec3i i0(n0, n0, n0);
                Vec3i i1(n1, n1, n1);
                Vec3i i2(n2, n2, n2);
                Vec3i i3(n3, n3, n3);

                std::vector<Vec3i> face1 = { i2, i1, i0 };
                std::vector<Vec3i> face2 = { i3, i1, i2 };
                model->add_face(face1);
                model->add_face(face2);
            }
        }
        else
        {
            for (int x = X_SEGMENTS - 1; x >= 0; --x)
            {
                int n0 = (y + 1) * (X_SEGMENTS + 1) + x + 1;
                int n1 = y * (X_SEGMENTS + 1) + x + 1;
                int n2 = (y + 1) * (X_SEGMENTS + 1) + x;
                int n3 = y * (X_SEGMENTS + 1) + x;

                Vec3i i0(n0, n0, n0);
                Vec3i i1(n1, n1, n1);
                Vec3i i2(n2, n2, n2);
                Vec3i i3(n3, n3, n3);

                std::vector<Vec3i> face1 = { i2, i1, i0 };
                std::vector<Vec3i> face2 = { i3, i1, i2 };
                model->add_face(face1);
                model->add_face(face2);
            }
        }
        oddRow = !oddRow;
    }
}

void PBRender::init_enviroment_map(){
    printf("init enviroment_map");
    stbi_set_flip_vertically_on_load(true);
    int width, height, nrComponents;
    float* data = stbi_loadf("obj/newport_loft.hdr", &width, &height, &nrComponents, 0);
    if (data)
    {
        enviroment_map_hdr = new Texture3f(width, height);
        enviroment_map_hdr->init_from_data((Vec3f*)data);
        stbi_image_free(data);
    }
    else
    {
        std::cout << "Failed to load HDR image." << std::endl;
        exit(-1);
    }

    EquirectangularShader* etShader = new EquirectangularShader();
    etShader->enviroment_map_hdr = enviroment_map_hdr;
    etShader->project_mat = perspective(90. * PI / 180., 1, 0.1, 10);

    Camera* cameras[6];
    cameras[0] = new Camera(Vec3f(0, 0, 0), Vec3f(1, 0, 0), Vec3f(0, -1, 0));
    cameras[1] = new Camera(Vec3f(0, 0, 0), Vec3f(-1, 0, 0), Vec3f(0, -1, 0));
    cameras[2] = new Camera(Vec3f(0, 0, 0), Vec3f(0, 1, 0), Vec3f(0, 0, 1));
    cameras[3] = new Camera(Vec3f(0, 0, 0), Vec3f(0, -1, 0), Vec3f(0, 0, -1));
    cameras[4] = new Camera(Vec3f(0, 0, 0), Vec3f(0, 0, 1), Vec3f(0, -1, 0));
    cameras[5] = new Camera(Vec3f(0, 0, 0), Vec3f(0, 0, -1), Vec3f(0, -1, 0));

    Renderer etRenderer(dnear, dfar, 512, 512, 0, 0, false);
    etRenderer.set_cullingFace(false);

    enviroment_map_cube = new TextureCube<Vec4f>(512, 512);
    for (int i = 0; i < 6; i++)
    {
        etShader->view_mat = cameras[i]->get_view();
        etRenderer.clear_colorbuffer();
        etRenderer.clear_zbuffer();
        etRenderer.render(enviroment_model, etShader, nullptr);
        etRenderer.get_colorbuffer(enviroment_map_cube->textures[i]);
        delete cameras[i];
    }
    generateMipmap(enviroment_map_cube, maxMipLevels);

    delete etShader;
    printf("init enviroment_map done");
}

void PBRender::init_irradiance_map() {
    printf("init irradiance_map");
    Camera* cameras[6];
    cameras[0] = new Camera(Vec3f(0, 0, 0), Vec3f(1, 0, 0), Vec3f(0, -1, 0));
    cameras[1] = new Camera(Vec3f(0, 0, 0), Vec3f(-1, 0, 0), Vec3f(0, -1, 0));
    cameras[2] = new Camera(Vec3f(0, 0, 0), Vec3f(0, 1, 0), Vec3f(0, 0, 1));
    cameras[3] = new Camera(Vec3f(0, 0, 0), Vec3f(0, -1, 0), Vec3f(0, 0, -1));
    cameras[4] = new Camera(Vec3f(0, 0, 0), Vec3f(0, 0, 1), Vec3f(0, -1, 0));
    cameras[5] = new Camera(Vec3f(0, 0, 0), Vec3f(0, 0, -1), Vec3f(0, -1, 0));

    Renderer irRenderer(dnear, dfar, 32, 32, 0, 0, false);
    irRenderer.set_cullingFace(false);

    IrradianceConvolutionShader* irShader = new IrradianceConvolutionShader();
    irShader->enviroment_map_cube = enviroment_map_cube;
    irShader->project_mat = perspective(90. * PI / 180., 1, 0.1, 10);

    irradiance_map_cube = new TextureCube<Vec4f>(32, 32);
    for (int i = 0; i < 6; i++)
    {
        irShader->view_mat = cameras[i]->get_view();
        irRenderer.clear_colorbuffer();
        irRenderer.clear_zbuffer();
        irRenderer.render(enviroment_model, irShader, nullptr);
        irRenderer.get_colorbuffer(irradiance_map_cube->textures[i]);
        delete cameras[i];
    }

    delete irShader;
    printf("init irradiance_map done");
}

void PBRender::init_prefilter_map() {
    printf("init prefilter_map");
    Camera* cameras[6];
    cameras[0] = new Camera(Vec3f(0, 0, 0), Vec3f(1, 0, 0), Vec3f(0, -1, 0));
    cameras[1] = new Camera(Vec3f(0, 0, 0), Vec3f(-1, 0, 0), Vec3f(0, -1, 0));
    cameras[2] = new Camera(Vec3f(0, 0, 0), Vec3f(0, 1, 0), Vec3f(0, 0, 1));
    cameras[3] = new Camera(Vec3f(0, 0, 0), Vec3f(0, -1, 0), Vec3f(0, 0, -1));
    cameras[4] = new Camera(Vec3f(0, 0, 0), Vec3f(0, 0, 1), Vec3f(0, -1, 0));
    cameras[5] = new Camera(Vec3f(0, 0, 0), Vec3f(0, 0, -1), Vec3f(0, -1, 0));

    PrefilterShader* preShader = new PrefilterShader();
    preShader->enviroment_map_cube = enviroment_map_cube;
    preShader->project_mat = perspective(90. * PI / 180., 1, 0.1, 10);

    prefilter_map_cube = new TextureCube<Vec4f>(128, 128);
    generateMipmapWithoutInit(prefilter_map_cube, maxMipLevels);
    for (unsigned int mip = 0; mip < maxMipLevels; ++mip)
    {
        unsigned int mipWidth = 128 * std::pow(0.5, mip);
        unsigned int mipHeight = 128 * std::pow(0.5, mip);
        Renderer render(dnear, dfar, mipWidth, mipHeight, 0, 0, false);
        render.set_cullingFace(false);
        
        Texture4f* colors = new Texture4f(mipWidth, mipHeight);
        preShader->roughness = (float)mip / (float)(maxMipLevels - 1);
        for (unsigned int i = 0; i < 6; ++i)
        {
            preShader->view_mat = cameras[i]->get_view();
            render.clear_colorbuffer();
            render.clear_zbuffer();
            render.render(enviroment_model, preShader, nullptr);
            render.get_colorbuffer(colors);
            prefilter_map_cube->textures[i]->initMipmap_from_data(colors->data, mip);
        }
        delete colors;
    }

    delete preShader;
    for (int i = 0; i < 6; i++)
    {
        delete cameras[i];
    }
    printf("init prefilter_map done");
}

void PBRender::init_LUT_map() {
    printf("init LUT_map");

    Renderer render(dnear, dfar, 512, 512, 0, 0, false);
    render.set_cullingFace(false);

    LUTShader* lutShader = new LUTShader();
    LUT_map = new Texture4f(512, 512);

    render.render(deffered_model, lutShader, nullptr);
    render.get_colorbuffer(LUT_map);

    /*unsigned char* image = new unsigned char[512 * 512 * 3];
    for (int i = 0; i < 512; i++)
    {
        for (int j = 0; j < 512; j++)
        {
            Vec4f v = LUT_map->get(i, j);
            image[(j * 512 + i) * 3] = v[0] * 255;
            image[(j * 512 + i) * 3 + 1] = v[1] * 255;
            image[(j * 512 + i) * 3 + 2] = v[2] * 255;
        }
    }
    
    stbi_flip_vertically_on_write(true);
    stbi_write_png("obj/LUT.png", 512, 512, 3, image, 0);*/

    delete lutShader;
    printf("init LUT_map done");
}