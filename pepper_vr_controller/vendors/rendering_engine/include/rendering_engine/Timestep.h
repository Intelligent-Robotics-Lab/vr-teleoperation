#ifndef TIMESTEP_H
#define TIMESTEP_H

#include <GLFW/glfw3.h>

namespace rendering_engine{

class Timestep
{
    public:
        Timestep(float time = 0.0f) : m_Time(time) {}

        operator float() const {return m_Time;}
        Timestep operator- (const Timestep& t) {return Timestep(m_Time-t.m_Time);}

        float GetSeconds() const {return m_Time;}
        float GetMilliSeconds() const {return m_Time * 1000.0f;};

        static Timestep GetCurrentTime() {return Timestep((float)glfwGetTime());}

    private:
        float m_Time;
};


}

#endif