#ifndef WINDOW_H
#define WINDOW_H

#include <string>
#include <vector>
#include "core/Vector3D.hpp"
#include "figure.h"

struct SDL_Window;
struct SDL_Renderer;


template<typename T>
class Node
{
    Node*   next;
    T*      value;
public:
    Node() : next(nullptr), value(nullptr) {}
    Node(T* value) : next(nullptr), value(value) {}
    ~Node()
    {
        delete next;
    }

    T* Value()
    {
        return value;
    }

    const T* Value() const
    {
        return value;
    }

   void Append(T* element)
   {
       if (value)
            next = new Node(element);
       else
           value = element;
   }


};

class Window
{
public:
    Window();
    ~Window();

    void Show();
private:
    SDL_Window*     m_pWindow;
    SDL_Renderer*   m_pRenderer;

    int             m_Width;
    int             m_Height;
    bool            m_IsVisible;

    Figure          m_Figure1;
    Figure          m_Figure2;

    int             m_BufferSize;
    int*            m_pFrameBuffer;
    float*          m_pZBuffer;
    Node<Figure>*   m_pGroups;

    void HandleEvents();

    void FigureGroupAppend(Figure& figure);
};

#endif // WINDOW_H
