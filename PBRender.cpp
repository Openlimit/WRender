#include "PBRender.h"
#include <stb_image.h>

void PBRender::init(int width, int height) {
    frame_width = width;
    frame_height = height;
    rotate_speed = 2;
    dnear = 0.1;
    dfar = 100;

    renderer = new Renderer(dnear, dfar, width, height, 0, 0, false);
    renderer->set_cullingFace(false);

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

    init_enviroment_map();
}

void PBRender::release() {
	if (model != nullptr)
		delete model;
	if (camera != nullptr)
		delete camera;
}

void PBRender::resize(int width, int height) {
	if (renderer != nullptr)
		delete renderer;
	renderer = new Renderer(dnear, dfar, width, height, 0, 0, false);
    renderer->set_cullingFace(false);

	frame_width = width;
	frame_height = height;
}

void PBRender::render() {
    renderer->clear_colorbuffer();
    renderer->clear_zbuffer();

    /*int nrRows = 5;
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
    }*/

    renderer->render(skybox_model, etShader, screenBits);
}

void PBRender::update_view() {
	view_mat = camera->get_view();

    shader->view_mat = view_mat;
    shader->viewPos = camera->get_camera_pos();

    //etShader->view_mat.topLeftCorner(3, 3) = view_mat.topLeftCorner(3, 3);
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

                std::vector<Vec3i> face1 = { i0, i1, i2 };
                std::vector<Vec3i> face2 = { i2, i1, i3 };
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

                std::vector<Vec3i> face1 = { i0, i1, i2 };
                std::vector<Vec3i> face2 = { i2, i1, i3 };
                model->add_face(face1);
                model->add_face(face2);
            }
        }
        oddRow = !oddRow;
    }
}

void PBRender::init_enviroment_map(){
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

    etShader = new EquirectangularShader();
    etShader->enviroment_map_hdr = enviroment_map_hdr;
    etShader->view_mat = Mat4f::Identity();
    etShader->view_mat.topLeftCorner(3, 3) = view_mat.topLeftCorner(3, 3);
    etShader->project_mat = perspective(90 * PI / 180, 1, 0.1, 10);

    skybox_model = new Model("obj/skybox_model.obj");


    Mat4f captureViews[] =
    {
       glm::lookAt(glm::vec3(0.0f, 0.0f, 0.0f), glm::vec3(1.0f,  0.0f,  0.0f), glm::vec3(0.0f, -1.0f,  0.0f)),
       glm::lookAt(glm::vec3(0.0f, 0.0f, 0.0f), glm::vec3(-1.0f,  0.0f,  0.0f), glm::vec3(0.0f, -1.0f,  0.0f)),
       glm::lookAt(glm::vec3(0.0f, 0.0f, 0.0f), glm::vec3(0.0f,  1.0f,  0.0f), glm::vec3(0.0f,  0.0f,  1.0f)),
       glm::lookAt(glm::vec3(0.0f, 0.0f, 0.0f), glm::vec3(0.0f, -1.0f,  0.0f), glm::vec3(0.0f,  0.0f, -1.0f)),
       glm::lookAt(glm::vec3(0.0f, 0.0f, 0.0f), glm::vec3(0.0f,  0.0f,  1.0f), glm::vec3(0.0f, -1.0f,  0.0f)),
       glm::lookAt(glm::vec3(0.0f, 0.0f, 0.0f), glm::vec3(0.0f,  0.0f, -1.0f), glm::vec3(0.0f, -1.0f,  0.0f))
    };
}