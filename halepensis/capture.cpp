#include <cstdlib>
#include <iostream>
#include <vector>
#include <mutex>
#include <limits>
#include <boost/interprocess/sync/scoped_lock.hpp>

#if defined(__APPLE__)
#include <GLUT/glut.h>
#else
#include <GL/glut.h>
#endif

#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Weverything"
#include <libfreenect.hpp>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

class MyFreenectDevice : public Freenect::FreenectDevice
{
public:
    MyFreenectDevice(freenect_context *_ctx, int _index)
        : Freenect::FreenectDevice(_ctx, _index),
          m_buffer_video(freenect_find_video_mode(FREENECT_RESOLUTION_MEDIUM, FREENECT_VIDEO_RGB).bytes),
          m_buffer_depth(freenect_find_depth_mode(FREENECT_RESOLUTION_MEDIUM, FREENECT_DEPTH_REGISTERED).bytes / 2),
          m_new_rgb_frame(false), m_new_depth_frame(false)
    {
        setDepthFormat(FREENECT_DEPTH_REGISTERED);
    }

    // Do not call directly, even in child
    void VideoCallback(void *_rgb, uint32_t timestamp)
    {
        boost::interprocess::scoped_lock<std::mutex> lock(m_rgb_mutex);
        uint8_t* rgb = static_cast<uint8_t*>(_rgb);
        copy(rgb, rgb+getVideoBufferSize(), m_buffer_video.begin());
        m_new_rgb_frame = true;
    }

    // Do not call directly, even in child
    void DepthCallback(void *_depth, uint32_t timestamp)
    {
        boost::interprocess::scoped_lock<std::mutex> lock(m_depth_mutex);
        uint16_t* depth = static_cast<uint16_t*>(_depth);
        copy(depth, depth+getDepthBufferSize()/2, m_buffer_depth.begin());
        m_new_depth_frame = true;
    }

    bool getRGB(std::vector<uint8_t> &buffer)
    {
        boost::interprocess::scoped_lock<std::mutex> lock(m_rgb_mutex);
        
        if (!m_new_rgb_frame)
            return false;

        buffer.swap(m_buffer_video);
        m_new_rgb_frame = false;

        return true;
    }

    bool getDepth(std::vector<uint16_t> &buffer)
    {
        boost::interprocess::scoped_lock<std::mutex> lock(m_depth_mutex);
        
        if (!m_new_depth_frame)
            return false;

        buffer.swap(m_buffer_depth);
        m_new_depth_frame = false;

        return true;
    }

private:
    std::mutex m_rgb_mutex;
    std::mutex m_depth_mutex;
    std::vector<uint8_t> m_buffer_video;
    std::vector<uint16_t> m_buffer_depth;
    bool m_new_rgb_frame;
    bool m_new_depth_frame;
};


Freenect::Freenect freenect;
MyFreenectDevice* device;

int window(0);                // Glut window identifier
int mx = -1, my = -1;         // Prevous mouse coordinates
float anglex = 0, angley = 0; // Panning angles
float zoom = 1;               // Zoom factor
bool color = true;            // Flag to indicate to use of color in the cloud


void DrawGLScene()
{
    static std::vector<uint8_t> rgb(640*480*3);
    static std::vector<uint16_t> depth(640*480);

    device->getRGB(rgb);
    device->getDepth(depth);

    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    
    glPointSize(1.0f);

    glBegin(GL_POINTS);

    if (!color) glColor3ub(255, 255, 255);
    for (int i = 0; i < 480*640; ++i)
    {
        if (color)
            glColor3ub( rgb[3*i+0],    // R
                        rgb[3*i+1],    // G
                        rgb[3*i+2] );  // B

        float f = 595.f;
        // Convert from image plane coordinates to world coordinates
        glVertex3f( (i%640 - (640-1)/2.f) * depth[i] / f,  // X = (x - cx) * d / fx
                    (i/640 - (480-1)/2.f) * depth[i] / f,  // Y = (y - cy) * d / fy
                    depth[i] );                            // Z = d
    }

    glEnd();
    
    // Draw the world coordinate frame
    glLineWidth(2.0f);
    glBegin(GL_LINES);
    glColor3ub(255, 0, 0);  // X-axis
    glVertex3f(  0, 0, 0);
    glVertex3f( 50, 0, 0);
    glColor3ub(0, 255, 0);  // Y-axis
    glVertex3f(0,   0, 0);
    glVertex3f(0,  50, 0);
    glColor3ub(0, 0, 255);  // Z-axis
    glVertex3f(0, 0,   0);
    glVertex3f(0, 0,  50);
    glEnd();

    // Place the camera
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
    glScalef(zoom, zoom, 1);
    gluLookAt( -7*anglex, -7*angley, -1000.0,
                     0.0,       0.0,  2000.0,
                     0.0,      -1.0,     0.0 );

    glutSwapBuffers();
}

void capture_pcd()
{
    static std::vector<uint8_t> rgb(640*480*3);
    static std::vector<uint16_t> depth(640*480);

    device->getRGB(rgb);
    device->getDepth(depth);
    
    pcl::PointCloud<pcl::PointXYZRGB> cloud;
    cloud.width    = 640 * 480;
    cloud.height   = 1;
    cloud.is_dense = false;
    cloud.resize (cloud.width * cloud.height);
    
    for (int i = 0; i < 480*640; ++i)
    {
        auto& point = cloud[i];
        
        if (color)
        {
            point.r = rgb[3*i+0];
            point.g = rgb[3*i+1];
            point.b = rgb[3*i+2];
        } else
        {
            point.r = 255;
            point.g = 255;
            point.b = 255;
        }
            

        float f = 595.f;
        // Convert from image plane coordinates to world coordinates
        point.x = (i%640 - (640-1)/2.f) * depth[i] / f / 1000.0f;
        point.y = (i/640 - (480-1)/2.f) * depth[i] / f / 1000.0f;
        point.z = depth[i] == 0 ? std::numeric_limits<float>::quiet_NaN() : depth[i] / 1000.0f;
    }
    
    pcl::io::savePCDFile ("/Users/lubiluk/Code/halepensis/data/capture.pcd", cloud);
}

void keyPressed(unsigned char key, int x, int y)
{
    switch (key)
    {
        case  'C':
        case  'c':
            color = !color;
            break;
        case ' ':
            capture_pcd();
            break;
        case  'Q':
        case  'q':
        case 0x1B:  // ESC
            glutDestroyWindow(window);
            device->stopDepth();
            device->stopVideo();
            exit(0);
    }
}


void mouseMoved(int x, int y)
{
    if (mx >= 0 && my >= 0)
    {
        anglex += x - mx;
        angley += y - my;
    }

    mx = x;
    my = y;
}


void mouseButtonPressed(int button, int state, int x, int y)
{
    if (state == GLUT_DOWN)
    {
        switch (button)
        {
            case GLUT_LEFT_BUTTON:
                mx = x;
                my = y;
                break;

            case 3:
                zoom *= 1.2f;
                break;

            case 4:
                zoom /= 1.2f;
                break;
        }
    }
    else if (state == GLUT_UP && button == GLUT_LEFT_BUTTON)
    {
        mx = -1;
        my = -1;
    }
}


void resizeGLScene(int width, int height)
{
    glViewport(0, 0, width, height);
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    gluPerspective(50.0, (float)width / height, 900.0, 11000.0);

    glMatrixMode(GL_MODELVIEW);
}


void idleGLScene()
{
    glutPostRedisplay();
}


void printInfo()
{
    std::cout << "\nAvailable Controls:"              << std::endl;
    std::cout << "==================="                << std::endl;
    std::cout << "Rotate       :   Mouse Left Button" << std::endl;
    std::cout << "Zoom         :   Mouse Wheel"       << std::endl;
    std::cout << "Toggle Color :   C"                 << std::endl;
    std::cout << "Quit         :   Q or Esc\n"        << std::endl;
    std::cout << "Capture PCD  :   Space\n"           << std::endl;
}


int main(int argc, char **argv)
{
    device = &freenect.createDevice<MyFreenectDevice>(0);
    device->startVideo();
    device->startDepth();
    
    glutInit(&argc, argv);

    glutInitDisplayMode(GLUT_RGB | GLUT_DOUBLE | GLUT_DEPTH);
    glutInitWindowSize(640, 480);
    glutInitWindowPosition(0, 0);

    window = glutCreateWindow("LibFreenect");
    glClearColor(0.45f, 0.45f, 0.45f, 0.0f);

    glEnable(GL_DEPTH_TEST);
    glEnable(GL_ALPHA_TEST);
    glAlphaFunc(GL_GREATER, 0.0f);

    glMatrixMode(GL_PROJECTION);
    gluPerspective(50.0, 1.0, 900.0, 11000.0);
        
    glutDisplayFunc(&DrawGLScene);
    glutIdleFunc(&idleGLScene);
    glutReshapeFunc(&resizeGLScene);
    glutKeyboardFunc(&keyPressed);
    glutMotionFunc(&mouseMoved);
    glutMouseFunc(&mouseButtonPressed);

    printInfo();

    glutMainLoop();

    return 0;
}

#pragma clang diagnostic pop
