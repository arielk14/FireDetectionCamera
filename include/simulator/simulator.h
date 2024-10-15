#ifndef ORB_SLAM2_SIMULATOR_H
#define ORB_SLAM2_SIMULATOR_H

#include <memory>
#include <Eigen/SVD>
#include <filesystem>
#include <functional>
#include <pangolin/gl/glsl.h>
#include <pangolin/pangolin.h>
#include <pangolin/gl/glvbo.h>
#include <pangolin/display/display.h>
#include <pangolin/utils/file_utils.h>
#include <pangolin/geometry/geometry.h>
#include <pangolin/display/pangolin_gl.h>
#include <pangolin/geometry/glgeometry.h>
#include "ORBextractor.h"
#include "System.h"

#include "RunModel/TextureShader.h"
#include "Auxiliary.h"
#include "Coordinates.h"


/**
 *  @class Simulator
 *  @brief This class provides a simulation environment for virtual robotic navigation and mapping.
 *
 *  The Simulator class integrates ORBSLAM2 and a 3D blender model to create a comprehensive testing bed
 *  for SLAM (Simultaneous Localization and Mapping) and navigation algorithms. It negates the need for
 *  a physical robot interface.
 *
 *  With this simulator and a 3D blender model, a robot's movement can be simulated in a variety of virtual environments.
 *  It uses the Pangolin library to capture the current viewer image from the display window, which is then
 *  sent to ORBSLAM2. This approach allows users to observe, analyze and improve the efficiency of their implemented
 *  algorithms in real-time with expansive and flexible datasets.
 *
 *  Features include:
 *  - Virtual robotic navigation in 3D environments using A,S,D,W,E,Q,R,F to move in the model
 *  - On-the-fly ORBSLAM2 map generation and navigation from the 3D model, and extraction of current location and full map.
 *  - Real-time visualization using Pangolin
 */
class Simulator
{
public:

    
    /**
     * Constructs a Simulator instance with specified parameters, and loads the ORBSLAM2 object.
     *
     * @param ORBSLAMConfigFile: A string representing the path to the ORBSLAM2 configuration file, an example can be found in the "config" folder in the project.
     *
     * @param model_path: A string representing the path to the 3D blender model used for simulation.
     *
     * @param modelTextureNameToAlignTo: A string representing the texture name used for alignment in the 3D model.
     *
     * @param saveMap: A boolean to determine whether the generated SLAM map should be saved or not. Defaults to false if not specified.
     *
     * @param simulatorOutputDirPath: A string representing the output directory for saving SLAM maps. Defaults to "../slamMaps/" if not specified.
     *
     * @param loadMap: A boolean to determine whether to load a previously saved SLAM map. Defaults to false if not specified.
     *
     * @param mapLoadPath: A string representing the path to a previously saved SLAM map to load. Defaults to "../slamMaps/example.bin" if not specified.
     *
     * @param movementFactor: A double value representing the movement speed in the simulator. Defaults to 0.01 if not specified.
     *
     * @param vocPath: A string representing the path to the ORBSLAM2 vocabulary file. Defaults to "../Vocabulary/ORBvoc.txt" if not specified.
     */
     //added lat and longi parameters
    Simulator(std::string ORBSLAMConfigFile, std::string model_path, bool alignModelToTexture, std::string modelTextureNameToAlignTo,
              Eigen::Vector3f startingPoint, bool saveMap = false, std::string simulatorOutputDirPath = "../slamMaps/", bool loadMap = false,
              std::string mapLoadPath = "../slamMaps/example.bin",
              double movementFactor = 0.01,
              double speedFactor = 1.0,
              std::string vocPath = "../Vocabulary/ORBvoc.txt",
              double lat=0,double longi=0);

//returns valid frame
    cv::Mat getFrame();
    
 //returns K camera matrix
    Eigen::Matrix3d getK() ;

 //setter and getter lat and long of coordinates parameter   
    void setCoordinates(float lat, float longi);
    
    const Coordinates& getCoordinates();
    
  //adjusts camera yaw by value
    void yawCamera(double value);
      
    std::thread run();
    /**
     * sample the simulator state, this changes from false to true once the model is loaded
     * @return is the simulator is ready
     * */
    bool isReady() { return ready; }



    void stop() { stopFlag = true; }

    /**
     * @brief enabling or disabling the ORBSLAM process.
     *
     */


    void simulatorRunThread();



    void drawPoint(cv::Point3d point, float size, Eigen::Vector3d color);

    void cleanPoints();

private:
    /**
     * @brief A map for controlling the virtual robot's actions.
     *
     * This property is an unordered map where:
     * - The key is a string representing a command for the virtual robot.
     * - The value is a boolean indicating whether the command is executable (true) or not (false).
     */
    std::unordered_map<std::string, bool> commandMap = {
            {"cw", true},
            {"ccw", true},
            {"forward", true},
            {"back", true},
            {"right", true},
            {"up", true},
            {"down", true},
            {"left", true},
            {"flip", false},
            {"rc", false}};
    std::shared_ptr<ORB_SLAM2::System> SLAM;
    std::string vocPath;
    std::string ORBSLAMConfigFile;
    std::string mapLoadPath;
    bool loadMap;
    pangolin::OpenGlRenderState s_cam;
    Eigen::Matrix3d K;
    std::shared_ptr < ORB_SLAM2::ORBextractor> orbExtractor;
    std::string simulatorOutputDir;
    bool stopFlag;
    bool stopFlagSLAM;
    bool ready;
    bool start;
    bool initSlam;
    float fScaleFactor;
    std::vector<std::pair<cv::Point3d, std::pair<float, Eigen::Vector3d>>> points;
    Coordinates coordinates;
    Eigen::Vector3f startingPoint;
    bool saveMapSignal;
    bool track;
    double movementFactor{};
    std::string modelPath;
    bool alignModelToTexture;
    std::string modelTextureNameToAlignTo;
    std::vector<Eigen::Vector3d> Picks_w;
    bool isSaveMap;
    bool isInitalized;
    bool isLocalized;
    bool cull_backfaces;
    pangolin::GlSlProgram program;
    pangolin::GlGeometry geomToRender;
    Eigen::Vector2i viewportDesiredSize;
    cv::Mat Tcw;
    cv::Mat currentImg;
    cv::Mat bufferMat;
    std::mutex locationLock;
    std::mutex imgLock;
    int numberOfFeatures;
    int trackingNumberOfFeatures;
    double speedFactor;

    void SLAMThread();

    bool feedSLAM(cv::Mat &img);

    void extractSurface(const pangolin::Geometry &modelGeometry, std::string modelTextureNameToAlignTo,
                        Eigen::MatrixXf &surface);

    void alignModelViewPointToSurface(const pangolin::Geometry &modelGeometry, std::string modelTextureNameToAlignTo);

    void saveMap(std::string prefix = "");

    void intervalOverCommand(const std::function<void(pangolin::OpenGlRenderState &, double &)> &func,
                             double value, int intervalUsleep,
                             double fps,
                             int totalCommandTimeInSeconds);

    void
    applyCommand(std::string &command, double value,
                 int intervalUsleep,
                 double fps,
                 int totalCommandTimeInSeconds);

    void static applyForwardToModelCam(pangolin::OpenGlRenderState &cam, double value);

    void static applyRightToModelCam(pangolin::OpenGlRenderState &cam, double value);

    void static applyYawRotationToModelCam(pangolin::OpenGlRenderState &cam, double value);

    void static applyUpModelCam(pangolin::OpenGlRenderState &cam, double value);

    void static applyPitchRotationToModelCam(pangolin::OpenGlRenderState &cam, double value);

    void slower();

    void faster();
};

#endif // ORB_SLAM2_SIMULATOR_H
