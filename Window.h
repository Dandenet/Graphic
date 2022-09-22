#ifndef WINDOW_H
#define WINDOW_H

#include <string>
#include <vector>
#include "core/Vector3D.hpp"
#include "core/Matrix.hpp"

#include <SDL_rect.h>

struct SDL_Window;
struct SDL_Renderer;
struct SDL_KeyboardEvent;

class Window
{
public:
    Window(const std::string& title, int width, int height);
    ~Window();

    void Show();
private:
    SDL_Window*     m_pWindow;
    SDL_Renderer*   m_pRenderer;

    int             m_Width;
    int             m_Height;
    bool            m_IsVisible;

    Matrix4x4       m_TraslateMatrix;
    Matrix4x4       m_RotationMatrix;
    Matrix4x4       m_ScaleMatrix;
    Matrix4x4       m_ProjectionMatrix;

    unsigned int    m_PrevTicks;
    unsigned int    m_CurTicks;
    float           m_Speed = 0.7f;
    float           m_RotationSpeed = 45.0f;


    float           m_AnimRotSpeedX = 0.0f;
    float           m_AnimRotSpeedY = 0.0f;
    float           m_AnimRotSpeedZ = 0.0f;


    float           m_AccelerationX = 7.0f;
    float           m_AccelerationY = 7.0f;
    float           m_AccelerationZ = 7.0f;

    bool            m_isActiv;


    float           m_Period = 5.0f;
    float           m_BrakeTimer = 1.0f;

    bool            isBrake = false;

    std::vector<Vector3D> m_AxisX;
    std::vector<Vector3D> m_AxisY;
    std::vector<Vector3D> m_AxisZ;

    std::vector<Vector3D> m_Points;

    void HandleEvents();

    void KeyUpHandler(SDL_KeyboardEvent* e);
    void Update();

    SDL_Point ToWindowCoords(const Vector3D& v);
};

#endif // WINDOW_H
