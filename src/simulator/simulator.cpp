#include "simulator/simulator.h"



void Simulator::yawCamera(double value)
{
  applyYawRotationToModelCam(this->s_cam, value);
}


Eigen::Matrix3d Simulator::getK() {
   return K;
}


void Simulator::setCoordinates(float lat, float longi)
{
   this->coordinates.setCoordinates(lat,longi);
}

const Coordinates& Simulator::getCoordinates()
{
   return this->coordinates;
}



cv::Mat Simulator::getFrame() {
// Lock the mutex before accessing shared resources

    imgLock.lock();
    if(this->currentImg.empty()||this->bufferMat.empty())
    	return cv::Mat();

    
    // Convert RGBA image to RGB (removing the alpha channel)
    cv::Mat rgbImg;

    cv::cvtColor(bufferMat, rgbImg, cv::COLOR_RGBA2BGR);
    
  // Flip the image vertically if needed (OpenGL's origin is bottom-left)
    cv::flip(rgbImg, rgbImg, 0);

    // Unlock the mutex after modifying the shared image
    imgLock.unlock();

    // Return the processed RGB image
    return rgbImg;
}

Simulator::Simulator(std::string ORBSLAMConfigFile, std::string model_path, bool alignModelToTexture,
                     std::string modelTextureNameToAlignTo, Eigen::Vector3f startingPoint,
                     bool saveMap, std::string simulatorOutputDirPath, bool loadMap, std::string mapLoadPath,
                     double movementFactor,
                     double speedFactor,
                     std::string vocPath,double lat,double longi) : coordinates(lat,longi),stopFlag(false), ready(false), saveMapSignal(false),
                                            track(false), start(false), initSlam(false),
                                            movementFactor(movementFactor), modelPath(model_path),
                                            alignModelToTexture(alignModelToTexture),
                                            startingPoint(startingPoint),
                                            modelTextureNameToAlignTo(modelTextureNameToAlignTo),
                                            isSaveMap(saveMap),
                                            cull_backfaces(false), isInitalized(false),
                                            stopFlagSLAM(false),
                                            speedFactor(speedFactor),
                                            viewportDesiredSize(640, 480) {
    cv::FileStorage fSettings(ORBSLAMConfigFile, cv::FileStorage::READ);

    float fx = fSettings["Camera.fx"];
    float fy = fSettings["Camera.fy"];
    float cx = fSettings["Camera.cx"];
    float cy = fSettings["Camera.cy"];
    fScaleFactor = fSettings["ORBextractor.scaleFactor"];
    this->numberOfFeatures = fSettings["ORBextractor.nFeatures"];
    this->trackingNumberOfFeatures = fSettings["ORBextractor.trackingNumberOfFeatures"];
    int fIniThFAST = fSettings["ORBextractor.iniThFAST"];
    int fMinThFAST = fSettings["ORBextractor.minThFAST"];
    int nLevels = fSettings["ORBextractor.nLevels"];

    this->K << fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0;
  
    this->points = std::vector<std::pair<cv::Point3d, std::pair<float, Eigen::Vector3d>>>();
}







void Simulator::simulatorRunThread() {
    std::string windowName = "Model";
    pangolin::CreateWindowAndBind(windowName);

    // we manually need to restore the properties of the context
    glEnable(GL_DEPTH_TEST);
    this->s_cam = pangolin::OpenGlRenderState(
            pangolin::ProjectionMatrix(viewportDesiredSize(0), viewportDesiredSize(1), K(0, 0), K(1, 1), K(0, 2),
                                       K(1, 2), 0.1, 20),
            pangolin::ModelViewLookAt(this->startingPoint.x(), this->startingPoint.y(), this->startingPoint.z(), 0, 0, 0, 0.0, -1.0,
                                      pangolin::AxisY)); // the first 3 value are meaningless because we change them later

    bool show_bounds = false;
    bool show_axis = false;
    bool show_x0 = false;
    bool show_y0 = false;
    bool show_z0 = false;
    pangolin::RegisterKeyPressCallback('b', [&]() { show_bounds = !show_bounds; });
    pangolin::RegisterKeyPressCallback('0', [&]() { cull_backfaces = !cull_backfaces; });
    pangolin::RegisterKeyPressCallback('a', [&]() { show_axis = !show_axis; });
    pangolin::RegisterKeyPressCallback('k', [&]() { stopFlag = !stopFlag; });
    pangolin::RegisterKeyPressCallback('t', [&]() { track = !track; });
    pangolin::RegisterKeyPressCallback(pangolin::PANGO_KEY_TAB, [&]() { start = true; });
    pangolin::RegisterKeyPressCallback('m', [&]() { saveMapSignal = !saveMapSignal; });
    pangolin::RegisterKeyPressCallback('x', [&]() { show_x0 = !show_x0; });
    pangolin::RegisterKeyPressCallback('y', [&]() { show_y0 = !show_y0; });
    pangolin::RegisterKeyPressCallback('z', [&]() { show_z0 = !show_z0; });
    auto LoadProgram = [&]() {
        program.ClearShaders();
        program.AddShader(pangolin::GlSlAnnotatedShader, pangolin::shader);
        program.Link();
    };
    LoadProgram();
    pangolin::Handler3D handler(this->s_cam);
    pangolin::View &d_cam = pangolin::Display("simulator_d_cam")
            .SetBounds(0.0, 1.0, 0.0, 1.0, ((float) -viewportDesiredSize[0] / (float) viewportDesiredSize[1]))
            .SetHandler(&handler);
    const pangolin::Geometry modelGeometry = pangolin::LoadGeometry(modelPath);
    if (alignModelToTexture) {
        alignModelViewPointToSurface(modelGeometry, modelTextureNameToAlignTo);
    }
    geomToRender = pangolin::ToGlGeometry(modelGeometry);
    for (auto &buffer: geomToRender.buffers) {
        buffer.second.attributes.erase("normal");
    }
    pangolin::ShowFullscreen(pangolin::TrueFalseToggle::True);
    pangolin::ShowFullscreen(pangolin::TrueFalseToggle::False);
    pangolin::PixelFormat fmt = pangolin::PixelFormatFromString("RGBA32");
    int width = d_cam.v.w;
    int height = d_cam.v.h;
    while (!pangolin::ShouldQuit() && !stopFlag) {
        ready = true;
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        if (d_cam.IsShown()) {
            d_cam.Activate(this->s_cam);
            if (cull_backfaces) {
                glEnable(GL_CULL_FACE);
                glCullFace(GL_BACK);
            }

            program.Bind();
            program.SetUniform("KT_cw", this->s_cam.GetProjectionMatrix() * this->s_cam.GetModelViewMatrix());
            pangolin::GlDraw(program, geomToRender, nullptr);
            program.Unbind();
            std::vector<unsigned char> buffer(4 * width * height);
            glReadPixels(0, 0, width, height, GL_RGBA, GL_UNSIGNED_BYTE, buffer.data());
            imgLock.lock();
            cv::Mat imgBuffer = cv::Mat(height, width, CV_8UC4, buffer.data());
            bufferMat=imgBuffer;
            cv::cvtColor(imgBuffer, currentImg, cv::COLOR_RGBA2GRAY);
            cv::flip(currentImg, currentImg, 0);
            imgLock.unlock();
            this->s_cam.Apply();

            glDisable(GL_CULL_FACE);

            Auxiliary::drawPoints(this->points);

            pangolin::FinishFrame();




        } else {
            this->s_cam.Apply();
            pangolin::FinishFrame();

        }

    }

}

std::thread Simulator::run() {
    std::thread thread(&Simulator::simulatorRunThread, this);
    return thread;
}



void Simulator::extractSurface(const pangolin::Geometry &modelGeometry, std::string modelTextureNameToAlignTo,
                               Eigen::MatrixXf &surface) {
    std::vector<Eigen::Vector3<unsigned int>> surfaceIndices;
    for (auto &o: modelGeometry.objects) {
        if (o.first == modelTextureNameToAlignTo) {
            const auto &it_vert = o.second.attributes.find("vertex_indices");
            if (it_vert != o.second.attributes.end()) {
                const auto &vs = std::get<pangolin::Image<unsigned int>>(it_vert->second);
                for (size_t i = 0; i < vs.h; ++i) {
                    const Eigen::Map<const Eigen::Vector3<unsigned int>> v(vs.RowPtr(i));
                    surfaceIndices.emplace_back(v);
                }
            }
        }
    }
    surface = Eigen::MatrixXf(surfaceIndices.size() * 3, 3);
    int currentIndex = 0;
    for (const auto &b: modelGeometry.buffers) {
        const auto &it_vert = b.second.attributes.find("vertex");
        if (it_vert != b.second.attributes.end()) {
            const auto &vs = std::get<pangolin::Image<float>>(it_vert->second);
            for (auto &row: surfaceIndices) {
                for (auto &i: row) {
                    const Eigen::Map<const Eigen::Vector3f> v(vs.RowPtr(i));
                    surface.row(currentIndex++) = v;
                }
            }
        }
    }
}

void Simulator::applyPitchRotationToModelCam(pangolin::OpenGlRenderState &cam, double value) {
    double rand = double(value) * (M_PI / 180);
    double c = std::cos(rand);
    double s = std::sin(rand);

    Eigen::Matrix3d R;
    R << 1, 0, 0,
            0, c, -s,
            0, s, c;

    Eigen::Matrix4d pangolinR = Eigen::Matrix4d::Identity();;
    pangolinR.block<3, 3>(0, 0) = R;

    auto camMatrix = pangolin::ToEigen<double>(cam.GetModelViewMatrix());

    // Left-multiply the rotation
    camMatrix = pangolinR * camMatrix;

    // Convert back to pangolin matrix and set
    pangolin::OpenGlMatrix newModelView;
    for (int i = 0; i < 4; ++i) {
        for (int j = 0; j < 4; ++j) {
            newModelView.m[j * 4 + i] = camMatrix(i, j);
        }
    }

    cam.SetModelViewMatrix(newModelView);
}





void Simulator::applyYawRotationToModelCam(pangolin::OpenGlRenderState &cam, double value) {
    double rand = double(value) * (M_PI / 180);
    double c = std::cos(rand);
    double s = std::sin(rand);

    Eigen::Matrix3d R;
    R << c, 0, s,
            0, 1, 0,
            -s, 0, c;

    Eigen::Matrix4d pangolinR = Eigen::Matrix4d::Identity();
    pangolinR.block<3, 3>(0, 0) = R;

    auto camMatrix = pangolin::ToEigen<double>(cam.GetModelViewMatrix());

    // Left-multiply the rotation
    camMatrix = pangolinR * camMatrix;

    // Convert back to pangolin matrix and set
    pangolin::OpenGlMatrix newModelView;
    for (int i = 0; i < 4; ++i) {
        for (int j = 0; j < 4; ++j) {
            newModelView.m[j * 4 + i] = camMatrix(i, j);
        }
    }

    cam.SetModelViewMatrix(newModelView);
}

void Simulator::alignModelViewPointToSurface(const pangolin::Geometry &modelGeometry,
                                             std::string modelTextureNameToAlignTo) {
    Eigen::MatrixXf surface;
    extractSurface(modelGeometry, modelTextureNameToAlignTo, surface);
    Eigen::JacobiSVD<Eigen::MatrixXf> svd(surface, Eigen::ComputeThinU | Eigen::ComputeThinV);
    svd.computeV();
    Eigen::Vector3f v = svd.matrixV().col(2);
    const auto mvm = pangolin::ModelViewLookAt(v.x(), v.y(), v.z(), 0, 0, 0, 0.0,
                                               -1.0,
                                               pangolin::AxisY);
    const auto proj = pangolin::ProjectionMatrix(viewportDesiredSize(0), viewportDesiredSize(1), K(0, 0), K(1, 1),
                                                 K(0, 2), K(1, 2), 0.1, 20);
    this->s_cam.SetModelViewMatrix(mvm);
    this->s_cam.SetProjectionMatrix(proj);
    applyPitchRotationToModelCam(this->s_cam, -90);
}


void Simulator::drawPoint(cv::Point3d point, float size, Eigen::Vector3d color)
{
    std::pair<float, Eigen::Vector3d> sizeAndColor(size, color);
    this->points.push_back(std::pair<cv::Point3d, std::pair<float, Eigen::Vector3d>>(point, sizeAndColor));
}

void Simulator::cleanPoints()
{
    this->points = std::vector<std::pair<cv::Point3d, std::pair<float, Eigen::Vector3d>>>();
}


