#include "Window.h"
#include "core/Matrix.hpp"
#include "core/MatrixTransform.hpp"

#include <cassert>
#include <algorithm>
#include <random>

#include <SDL.h>


Window::Window(const std::string &title, int width, int height) :
    m_Width(width),         m_Height(height),
    m_TraslateMatrix(1.0f), m_RotationMatrix(1.0f),
    m_ScaleMatrix(1.0f),    m_ProjectionMatrix(1.0f),
    m_isActiv(false)
{
    assert(SDL_Init(SDL_INIT_VIDEO) >= 0);

    m_pWindow = SDL_CreateWindow(title.c_str(),
                                 SDL_WINDOWPOS_CENTERED,
                                 SDL_WINDOWPOS_CENTERED,
                                 m_Width, m_Height, 0);

    assert(m_pWindow != nullptr);

    m_pRenderer = SDL_CreateRenderer(m_pWindow, -1,
                                     SDL_RENDERER_ACCELERATED);

    assert(m_pRenderer != nullptr);

    m_Points = {

        // Левая часть

        // нижняя левая грань
        {-0.75f, -0.5f, 0.0f}, {-0.5f, -0.5f, 0.0f},
        {-0.75f, -0.5f, -0.25f}, {-0.5f, -0.5f, -0.25f},
        {-0.75f, -0.5f, -0.25f}, {-0.75f, -0.5f, 0.0f},
        {-0.5f, -0.5f, 0.0f}, {-0.5f, -0.5f, -0.25f},

        // левая боковая грань
        {-0.75f, -0.5f, 0.0f},   {-0.75f, 0.25f, 0.0f},
        {-0.75f, -0.5f, -0.25f}, {-0.75f, 0.25f, -0.25f},
        {-0.75f, 0.25f, -0.25f}, {-0.75f, 0.25f, 0.0f},

        // Правая боковая грань
        {-0.5f, -0.5f, 0.0f},   {-0.5f, 0.0f, 0.0f},
        {-0.5f, -0.5f, -0.25f}, {-0.5f, 0.0f, -0.25f},
        {-0.5f, 0.0f, 0.0f},    {-0.5f, 0.0f, -0.25f},


        // Правая часть

        // нижняя левая грань
        {0.75f, -0.5f, 0.0f},   {0.5f, -0.5f, 0.0f},
        {0.75f, -0.5f, -0.25f}, {0.5f, -0.5f, -0.25f},
        {0.75f, -0.5f, -0.25f}, {0.75f, -0.5f, 0.0f},
        {0.5f, -0.5f, 0.0f},    {0.5f, -0.5f, -0.25f},

        // левая боковая грань
        {0.75f, -0.5f, 0.0f},   {0.75f, 0.25f, 0.0f},
        {0.75f, -0.5f, -0.25f}, {0.75f, 0.25f, -0.25f},
        {0.75f, 0.25f, -0.25f}, {0.75f, 0.25f, 0.0f},

        // Правая боковая грань
        {0.5f, -0.5f, 0.0f},   {0.5f, 0.0f, 0.0f},
        {0.5f, -0.5f, -0.25f}, {0.5f, 0.0f, -0.25f},
        {0.5f, 0.0f, 0.0f},    {0.5f, 0.0f, -0.25f},


        // Середина
        {-0.75f, 0.25f, -0.25f}, {0.75f, 0.25f, -0.25f},
        {-0.75f, 0.25f, 0.0f},   {0.75f, 0.25f, 0.0f},
        {-0.5f, 0.0f, 0.0f},   {0.5f, 0.0f, 0.0f},
        {-0.5f, 0.0f, -0.25f}, {0.5f, 0.0f, -0.25f},

        // левая часть шапки
        {-0.5f, 0.25f, 0.0f},   {-0.5f, 1.0f, 0.0f},
        {-0.5f, 0.25f, -0.25f}, {-0.5f, 1.0f, -0.25f},
        {-0.5f, 1.0f, 0.0f},    {-0.5f, 1.0f, -0.25f},

        // правая часть шапки
        {0.5f, 0.25f, 0.0f},   {0.5f, 1.0f, 0.0f},
        {0.5f, 0.25f, -0.25f}, {0.5f, 1.0f, -0.25f},
        {0.5f, 1.0f, 0.0f},    {0.5f, 1.0f, -0.25f},

        // соединение частей
        {0.5f, 1.0f, 0.0f},     {-0.5f, 1.0f, 0.0f},
        {0.5f, 1.0f, -0.25f},   {-0.5f, 1.0f, -0.25f},

        // внутренняя часть
        {-0.25f, 0.25f, 0.0f},   {-0.25f, 0.75f, 0.0f},
        {-0.25f, 0.25f, -0.25f}, {-0.25f, 0.75f, -0.25f},

        {0.25f, 0.25f, 0.0f},   {0.25f, 0.75f, 0.0f},
        {0.25f, 0.25f, -0.25f}, {0.25f, 0.75f, -0.25f},

        {-0.25f, 0.75f, 0.0f},  {-0.25f, 0.75f, -0.25f},
        {0.25f, 0.75f, 0.0f}, {0.25f, 0.75f, -0.25f},

        // соединение частей
        {0.25f, 0.75f, 0.0f},    {-0.25f, 0.75f, 0.0f},
        {0.25f, 0.75, -0.25f},   {-0.25f, 0.75f, -0.25f},


    };

    m_AxisX = {
        {0.0f, 0.0f, 0.0f}, {0.5f, 0.0f, 0.0f}
    };

    m_AxisY = {
        {0.0f, 0.0f, 0.0f}, {0.0f, 0.5f, 0.0f}
    };

    m_AxisZ = {
        {0.0f, 0.0f, 0.0f}, {0.0f, 0.0f, -0.5f}
    };

    SDL_SetHint( SDL_HINT_RENDER_SCALE_QUALITY, "1" );

}

Window::~Window()
{
    SDL_DestroyRenderer(m_pRenderer);
    SDL_DestroyWindow(m_pWindow);

    SDL_Quit();
}

void Window::Show()
{

    std::srand(std::time(0));
    m_IsVisible = true;


    Matrix4x4 mat(1.f);


    m_CurTicks = m_PrevTicks = SDL_GetTicks();

    while (m_IsVisible)
    {
        Update();
        mat =  m_ScaleMatrix * m_TraslateMatrix * m_RotationMatrix * m_ProjectionMatrix;

        std::vector<SDL_Point> points;
        std::vector<SDL_Point>  axisX;
        std::vector<SDL_Point>  axisY;
        std::vector<SDL_Point>  axisZ;

        for (const auto& point : m_Points)
        {
            points.push_back(ToWindowCoords( ( Vector4D(point, 1.f) * mat).ToVector3D()) );
        }

        for (const auto& point : m_AxisX)
        {
            axisX.push_back(ToWindowCoords( ( Vector4D(point, 1.f) * m_ProjectionMatrix).ToVector3D()) );
        }

        for (const auto& point : m_AxisY)
        {
            axisY.push_back(ToWindowCoords( ( Vector4D(point, 1.f) * m_ProjectionMatrix).ToVector3D()) );
        }

        for (const auto& point : m_AxisZ)
        {
            axisZ.push_back(ToWindowCoords( ( Vector4D(point, 1.f) * m_ProjectionMatrix).ToVector3D()) );
        }

        SDL_SetRenderDrawColor(m_pRenderer, 50, 50, 50, 255);
        SDL_RenderClear(m_pRenderer);


        HandleEvents();

        SDL_SetRenderDrawColor(m_pRenderer, 0, 255, 0, 255);

        for (int i = 0; i < points.size(); i += 2)
        {
            SDL_Point first = points[i];
            SDL_Point second = points[i + 1];

            SDL_RenderDrawLine(m_pRenderer, first.x, first.y, second.x, second.y);
        }

        SDL_SetRenderDrawColor(m_pRenderer, 255, 0, 0, 255);
        SDL_RenderDrawLines(m_pRenderer, axisX.data(), axisX.size());

        SDL_SetRenderDrawColor(m_pRenderer, 0, 0, 255, 255);
        SDL_RenderDrawLines(m_pRenderer, axisY.data(), axisY.size());

        SDL_SetRenderDrawColor(m_pRenderer, 255, 255, 0, 255);
        SDL_RenderDrawLines(m_pRenderer, axisZ.data(), axisZ.size());


        SDL_RenderPresent(m_pRenderer);
        m_CurTicks = SDL_GetTicks();
    }
}

void Window::HandleEvents()
{
    SDL_Event event;

    while (SDL_PollEvent(&event))
    {
        switch (event.type)
        {
        case SDL_QUIT:
            m_IsVisible = false;
            break;
        case SDL_KEYUP:
            KeyUpHandler(&event.key);
            break;

        default:
            break;
        }
    }
}

void Window::KeyUpHandler(SDL_KeyboardEvent *e)
{
    switch(e->keysym.scancode) {

    case SDL_SCANCODE_9:
        m_ScaleMatrix = Scale(m_ScaleMatrix, Vector3D(0.5f, 0.5f, 0.5f));
        break;
    case SDL_SCANCODE_0:
        m_ScaleMatrix = Scale(m_ScaleMatrix, Vector3D(2.0f, 2.0f, 2.0f));
        break;
    case SDL_SCANCODE_Z:
        m_ProjectionMatrix = ProjectionX();
        break;

    case SDL_SCANCODE_X:
        m_ProjectionMatrix = ProjectionY();
        break;

    case SDL_SCANCODE_C:
        m_ProjectionMatrix = ProjectionZ();
        break;

    case SDL_SCANCODE_V:
        m_ProjectionMatrix = ObliqueProjection(std::sqrt(2.f)/2.f,
                                               std::sqrt(2.f)/2.f);
        break;

    case SDL_SCANCODE_B:
        m_ProjectionMatrix = ObliqueProjection(0.5f * std::sqrt(2.f)/2.f,
                                               0.5f * std::sqrt(2.f)/2.f);
        break;

    case SDL_SCANCODE_N:
        m_ProjectionMatrix = Perspective(45.0f,
                                         static_cast<float>(m_Width) /
                                         static_cast<float>(m_Height), 1.0f, 150.0f);
        break;

    case SDL_SCANCODE_K:
        m_isActiv = true;
        break;

    case SDL_SCANCODE_L:
        m_isActiv = false;
        break;

    default:
        break;
    };
}

void Window::Update()
{
    float delta = (m_CurTicks - m_PrevTicks) / 1000.;
    m_PrevTicks = m_CurTicks;
    const Uint8* state =  SDL_GetKeyboardState(nullptr);

    if (state[SDL_SCANCODE_A])
        m_TraslateMatrix = Translate(m_TraslateMatrix, Vector3D(delta * -m_Speed, 0.0f, 0.0f));
    if (state[SDL_SCANCODE_D])
        m_TraslateMatrix = Translate(m_TraslateMatrix, Vector3D(delta * m_Speed, 0.0f, 0.0f));
    if (state[SDL_SCANCODE_S])
        m_TraslateMatrix = Translate(m_TraslateMatrix, Vector3D(0.0f, delta * -m_Speed, 0.0f));
    if (state[SDL_SCANCODE_W])
        m_TraslateMatrix = Translate(m_TraslateMatrix, Vector3D(0.0f, delta * m_Speed, 0.0f));

    if (state[SDL_SCANCODE_Q])
        m_TraslateMatrix = Translate(m_TraslateMatrix, Vector3D(0.0f,  0.0f, delta * -m_Speed));
    if (state[SDL_SCANCODE_E])
        m_TraslateMatrix = Translate(m_TraslateMatrix, Vector3D(0.0f, 0.0f, delta * m_Speed));

    if (state[SDL_SCANCODE_1])
        m_RotationMatrix = m_RotationMatrix * RotateX(ToRadians(m_RotationSpeed * delta));
    if (state[SDL_SCANCODE_2])
        m_RotationMatrix = m_RotationMatrix * RotateX(ToRadians(-m_RotationSpeed * delta));
    if (state[SDL_SCANCODE_3])
        m_RotationMatrix = m_RotationMatrix * RotateY(ToRadians(m_RotationSpeed * delta));
    if (state[SDL_SCANCODE_4])
        m_RotationMatrix = m_RotationMatrix * RotateY(ToRadians(-m_RotationSpeed * delta));
    if (state[SDL_SCANCODE_5])
        m_RotationMatrix = m_RotationMatrix * RotateZ(ToRadians(m_RotationSpeed * delta));
    if (state[SDL_SCANCODE_6])
        m_RotationMatrix = m_RotationMatrix * RotateZ(ToRadians(-m_RotationSpeed * delta));



    if (m_isActiv)
    {
        m_RotationMatrix = m_RotationMatrix * RotateX(ToRadians(delta * m_AnimRotSpeedX));
        m_RotationMatrix = m_RotationMatrix * RotateY(ToRadians(delta * m_AnimRotSpeedY));
        m_RotationMatrix = m_RotationMatrix * RotateZ(ToRadians(delta * m_AnimRotSpeedZ));

        if ((m_Period -= delta) < 0 && !isBrake)
        {
            m_AccelerationX = -m_AnimRotSpeedX;
            m_AccelerationY = -m_AnimRotSpeedY;
            m_AccelerationZ = -m_AnimRotSpeedZ;
            isBrake = true;
        }

        if (isBrake && (m_BrakeTimer -= delta) <= 0)
        {
            m_BrakeTimer = 1.0f;
            m_Period = 5.0f;
            m_AccelerationX = rand() % 2? 7.0f: -7.0f;
            m_AccelerationY = rand() % 2? 7.0f: -7.0f;
            m_AccelerationZ = rand() % 2? 7.0f: -7.0f;

            isBrake = false;
        }



        m_AnimRotSpeedX += delta * m_AccelerationX;
        m_AnimRotSpeedY += delta * m_AccelerationY;
        m_AnimRotSpeedZ += delta * m_AccelerationZ;
    }
}

SDL_Point Window::ToWindowCoords(const Vector3D &v)
{
    return {    static_cast<int>(m_Width / 2.0f     + v.x * m_Width / 2.0f),
                static_cast<int>(m_Height / 2.0f    - v.y * m_Height / 2.0f)};
}
