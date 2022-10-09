#include "Window.h"

#include <cassert>
#include <algorithm>
#include <random>

#include <SDL.h>


Window::Window() :
    m_Width(1280), m_Height(720),
    m_Figure1({{150, 150, 150}, {250, 250, 50}, {300, 100, 50}}),
    m_Figure2({{100, 50,   100}, {100, 250,  100}, {250, 250,  100}, {250, 50, 100}}),
    m_BufferSize(m_Width)
{
    assert(SDL_Init(SDL_INIT_VIDEO) >= 0);

    m_pWindow = SDL_CreateWindow("window",
                                 SDL_WINDOWPOS_CENTERED,
                                 SDL_WINDOWPOS_CENTERED,
                                 m_Width, m_Height, 0);

    assert(m_pWindow != nullptr);

    m_pRenderer = SDL_CreateRenderer(m_pWindow, -1,
                                     SDL_RENDERER_ACCELERATED);

    assert(m_pRenderer != nullptr);



    m_pFrameBuffer  = new int [m_BufferSize];
    m_pZBuffer      = new float [m_BufferSize];
    m_pGroups       = new Node<Figure> [m_Height];

    FigureGroupAppend(m_Figure1);
    FigureGroupAppend(m_Figure2);
}

Window::~Window()
{
    delete [] m_pFrameBuffer;
    delete [] m_pZBuffer;
    delete [] m_pGroups;

    SDL_DestroyRenderer(m_pRenderer);
    SDL_DestroyWindow(m_pWindow);

    SDL_Quit();
}

void Window::Show()
{
    m_IsVisible = true;


    memset(m_pFrameBuffer, 0, sizeof(int) * m_BufferSize);
    while (m_IsVisible) {

        SDL_SetRenderDrawColor(m_pRenderer, 250, 250, 250, 255);
        SDL_RenderClear(m_pRenderer);

        SDL_SetRenderDrawColor(m_pRenderer, 0, 0, 0, 255);
        m_Figure1.DrawOutline(m_pRenderer);
        m_Figure2.DrawOutline(m_pRenderer);

        SDL_RenderPresent(m_pRenderer);
        HandleEvents();
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

        default:
            break;
        }
    }
}

void Window::FigureGroupAppend(Figure &figure)
{
    int maxY = figure.MaxY();
    m_pGroups[maxY].Append(&figure);
}
