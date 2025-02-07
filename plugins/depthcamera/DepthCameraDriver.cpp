#include <DepthCameraShared.hh>
#include <DeviceRegistry.hh>

#include <cstdio>
#include <cstring>
#include <iostream>
#include <mutex>

#include <string>
#include <yarp/dev/DeviceDriver.h>
#include <yarp/dev/IFrameGrabberImage.h>
#include <yarp/os/Log.h>
#include <yarp/os/LogStream.h>
#include <yarp/os/Searchable.h>
#include <yarp/sig/Image.h>

namespace yarp
{
namespace dev
{
namespace gzyarp
{
class DepthCameraDriver;
}
} // namespace dev
} // namespace yarp

const std::string YarpDepthCameraScopedName = "sensorScopedName";

class yarp::dev::gzyarp::DepthCameraDriver : public yarp::dev::DeviceDriver,
                                        public yarp::dev::IFrameGrabberImage,
                                        public ::gzyarp::IDepthCameraData
{
public:
    DepthCameraDriver()
    {
        m_vertical_flip = false;
        m_horizontal_flip = false;
        m_display_time_box = false;
        m_display_timestamp = false;
        counter = 0;
        sprintf(num[0].data, "**** ** ** ****");
        sprintf(num[1].data, " *  *  *  *  * ");
        sprintf(num[2].data, "***  *****  ***");
        sprintf(num[3].data, "***  ****  ****");
        sprintf(num[4].data, "* ** ****  *  *");
        sprintf(num[5].data, "****  ***  ****");
        sprintf(num[6].data, "****  **** ****");
        sprintf(num[7].data, "***  *  *  *  *");
        sprintf(num[8].data, "**** ***** ****");
        sprintf(num[9].data, "**** ****  ****");
        sprintf(num[10].data, "               ");
        sprintf(num[11].data, "          ** **");
    }

    virtual ~DepthCameraDriver()
    {
    }

    // DEVICE DRIVER
    bool open(yarp::os::Searchable& config) override
    {
        std::string sensorScopedName(config.find(YarpDepthCameraScopedName.c_str()).asString().c_str());

        if (config.check("vertical_flip"))
            m_vertical_flip = true;
        if (config.check("horizontal_flip"))
            m_horizontal_flip = true;
        if (config.check("display_timestamp"))
            m_display_timestamp = true;
        if (config.check("display_time_box"))
            m_display_time_box = true;

        return true;
    }

    bool close() override
    {
        return true;
    }

    // IFRAMEGRABBER IMAGE
    bool getImage(yarp::sig::ImageOf<yarp::sig::PixelRgb>& _image) override
    {
        std::lock_guard<std::mutex> lock(m_sensorData->m_mutex);
        _image.resize(width(), height());

        unsigned char* pBuffer = _image.getRawImage();

        if (m_vertical_flip == true && m_horizontal_flip == false)
        {
            int r = 0;
            int c = 0;
            for (int c = 0; c < width(); c++)
            {
                for (int r = 0; r < height(); r++)
                {
                    unsigned char* pixel = _image.getPixelAddress(c, height() - r - 1);
                    pixel[0] = *(m_sensorData->m_imageBuffer + r * width() * 3 + c * 3 + 0);
                    pixel[1] = *(m_sensorData->m_imageBuffer + r * width() * 3 + c * 3 + 1);
                    pixel[2] = *(m_sensorData->m_imageBuffer + r * width() * 3 + c * 3 + 2);
                }
            }
        } else if (m_vertical_flip == false && m_horizontal_flip == true)
        {
            int r = 0;
            int c = 0;
            for (int c = 0; c < width(); c++)
            {
                for (int r = 0; r < height(); r++)
                {
                    unsigned char* pixel = _image.getPixelAddress(width() - c - 1, r);
                    pixel[0] = *(m_sensorData->m_imageBuffer + r * width() * 3 + c * 3 + 0);
                    pixel[1] = *(m_sensorData->m_imageBuffer + r * width() * 3 + c * 3 + 1);
                    pixel[2] = *(m_sensorData->m_imageBuffer + r * width() * 3 + c * 3 + 2);
                }
            }
        } else if (m_vertical_flip == true && m_horizontal_flip == true)
        {
            int r = 0;
            int c = 0;
            for (int c = 0; c < width(); c++)
            {
                for (int r = 0; r < height(); r++)
                {
                    unsigned char* pixel
                        = _image.getPixelAddress(width() - c - 1, height() - r - 1);
                    pixel[0] = *(m_sensorData->m_imageBuffer + r * width() * 3 + c * 3 + 0);
                    pixel[1] = *(m_sensorData->m_imageBuffer + r * width() * 3 + c * 3 + 1);
                    pixel[2] = *(m_sensorData->m_imageBuffer + r * width() * 3 + c * 3 + 2);
                }
            }
        } else
        {
            memcpy(pBuffer, m_sensorData->m_imageBuffer, getRawBufferSize());
        }

        if (m_display_time_box)
        {
            counter++;
            if (counter == 10)
                counter = 0;

            for (int c = 0 + counter * 30; c < 30 + counter * 30; c++)
            {
                for (int r = 0; r < 30; r++)
                {
                    if (counter % 2 == 0)
                    {
                        unsigned char* pixel
                            = _image.getPixelAddress(width() - c - 1, height() - r - 1);
                        pixel[0] = 255;
                        pixel[1] = 0;
                        pixel[2] = 0;
                    } else
                    {
                        unsigned char* pixel
                            = _image.getPixelAddress(width() - c - 1, height() - r - 1);
                        pixel[0] = 0;
                        pixel[1] = 255;
                        pixel[2] = 0;
                    }
                }
            }
        }

        if (m_display_timestamp)
        {
            char txtbuf[1000];
            sprintf(txtbuf, "%.3f", m_sensorData->simTime);
            int len = strlen(txtbuf);
            if (len < 20)
                print(pBuffer, width(), height(), 0, 0, txtbuf, len);
        }

        return true;
    }

    int height() const override
    {
        return m_sensorData->m_height;
    }

    int width() const override
    {
        return m_sensorData->m_width;
    }

    int getRawBufferSize()
    {
        return m_sensorData->m_bufferSize;
    }

    void print(unsigned char* pixbuf, int pixbuf_w, int pixbuf_h, int x, int y, char* s, int size)
    {
        int pixelsize = 5;
        for (int i = 0; i < size; i++)
        {
            char* num_p = 0;
            switch (s[i])
            {
            case '0':
                num_p = num[0].data;
                break;
            case '1':
                num_p = num[1].data;
                break;
            case '2':
                num_p = num[2].data;
                break;
            case '3':
                num_p = num[3].data;
                break;
            case '4':
                num_p = num[4].data;
                break;
            case '5':
                num_p = num[5].data;
                break;
            case '6':
                num_p = num[6].data;
                break;
            case '7':
                num_p = num[7].data;
                break;
            case '8':
                num_p = num[8].data;
                break;
            case '9':
                num_p = num[9].data;
                break;
            case ' ':
                num_p = num[10].data;
                break;
            case '.':
                num_p = num[11].data;
                break;
            }

            for (int yi = 0; yi < 5; yi++)
            {
                for (int xi = 0; xi < 3; xi++)
                {
                    int ii = yi * 3 + xi;
                    if (num_p[ii] == '*')
                    {
                        for (int r = yi * pixelsize; r < yi * pixelsize + pixelsize; r++)
                        {
                            int off = i * (pixelsize + 20);
                            for (int c = xi * pixelsize + off; c < xi * pixelsize + pixelsize + off;
                                 c++)
                            {
                                unsigned char* pixel = pixbuf + c * 3 + r * (pixbuf_w * 3);
                                pixel[0] = 0;
                                pixel[1] = 0;
                                pixel[2] = 255;
                            }
                        }
                    }
                }
            }
        }
    }

    // IDepthCameraData

    void setDepthCameraData(::gzyarp::DepthCameraData* dataPtr) override
    {
        m_sensorData = dataPtr;
    }

private:
    ::gzyarp::DepthCameraData* m_sensorData;
    int counter;
    bool m_vertical_flip;
    bool m_horizontal_flip;
    static double start_time;
    bool m_display_time_box;
    bool m_display_timestamp;

    struct txt_type
    {
        char data[16];
    };
    txt_type num[12];
};
