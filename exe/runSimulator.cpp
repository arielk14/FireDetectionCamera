//
// Created by tzuk on 6/4/23.
//
#include "include/Auxiliary.h"
#include "simulator/simulator.h"

int main(int argc, char **argv)
{
    std::string settingPath = Auxiliary::GetGeneralSettingsPath();
    std::ifstream programData(settingPath);
    nlohmann::json data;
    programData >> data;
    programData.close();

    std::string configPath = data["DroneYamlPathSlam"];
    std::string modelTextureNameToAlignTo = data["modelTextureNameToAlignTo"];
    std::string model_path = data["modelPath"];
    std::string vocabulary_path = data["VocabularyPath"];
    std::string simulatorOutputDir = data["simulatorOutputDir"];
    double movementFactor = data["movementFactor"];
    bool trackImages = data["trackImages"];

    Simulator simulator(configPath, model_path, modelTextureNameToAlignTo, trackImages, false, simulatorOutputDir, false, "", movementFactor, vocabulary_path);
    auto simulatorThread = simulator.run();
    //simulator.simulatorRunThread();
    while (!simulator.isReady())
    { // wait for the 3D model to load
        ORB_SLAM2::System::systemUsleep(1000);
    }
    std::cout << "to stop press k" << std::endl;
    std::cout << "to stop tracking press t" << std::endl;
    std::cout << "to save map point press m" << std::endl;
    std::cout << "waiting for key press to start scanning " << std::endl
              << std::endl;
    std::cin.get();
    simulator.setTrack(true);
    int currentYaw = 0;
    int angle = 10;
    cv::Mat currentLocation;
    for (int i = 0; i < std::ceil(360 / angle); i++)
    {
        std::string c = "forward 0.5";
        simulator.command(c);
        c = "right 0.7";
        simulator.command(c);
        c = "cw " + std::to_string(angle);
        simulator.command(c);
        sleep(1);
    }
    sleep(2);
    auto scanMap = simulator.getCurrentMap();

    //simulatorThread.join();
}
