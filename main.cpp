#include <iostream>
#include "Window.h"
#include <glm/glm.hpp>
#include <glm/ext.hpp>
int main()
{
    Window win("Window", 1280, 720);
    win.Show();
    return 0;
}
