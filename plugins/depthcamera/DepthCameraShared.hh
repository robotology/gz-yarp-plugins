#pragma once

#include <cstdlib>
#include <mutex>
#include <string>

namespace gzyarp
{

struct DepthCameraData
{
    std::mutex m_mutex;
    int m_width=0;
    int m_height=0;
    int m_bufferSize=0;
    unsigned char* m_imageBuffer=nullptr;
    std::string sensorScopedName="";
    double simTime=0.0;

    void init(const int width, const int height, const std::string& _sensorScopedName)
    {
        this->m_height = height;
        this->m_width = width;
        this->m_bufferSize = 3 * this->m_width * this->m_height;
        this->m_imageBuffer = static_cast<unsigned char*>(std::calloc(this->m_bufferSize, sizeof(unsigned char)));
        this->sensorScopedName = _sensorScopedName;
        return;
    }

    ~DepthCameraData()
    {
        if (m_imageBuffer)
        {
            free(m_imageBuffer);
            m_imageBuffer = nullptr;
        }
    }
};

class IDepthCameraData
{
public:
    virtual void setDepthCameraData(DepthCameraData* dataPtr) = 0;

    virtual ~IDepthCameraData() {};
};

} // namespace gzyarp
