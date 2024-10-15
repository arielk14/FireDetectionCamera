#include <pangolin/utils/file_utils.h>
#include <pangolin/geometry/glgeometry.h>
#include <opencv2/opencv.hpp>  
#include <firebase/database.h>  
#include <firebase/firestore.h> 
#include <firebase/app.h>
#include <firebase/future.h>
#include "Auxiliary.h"
#include "simulator/simulator.h"
#include "RunModel/TextureShader.h"
#include "FirebaseManager.h"
#include <cmath>

const double PI = 3.141592653589793;
const double EARTH_RADIUS_KM = 6371.0;  // Default radius of the Earth in kilometers

// Convert degrees to radians
double toRadians(double degrees) {
    return degrees * (PI / 180.0);
}

// Convert radians to degrees
double toDegrees(double radians) {
    return radians * (180.0 / PI);
}

// Function to calculate the new latitude and longitude
Coordinates getPointAtDistance(const Coordinates& cor, double distance, double bearing, double R = EARTH_RADIUS_KM) {
    // Convert latitude, longitude, and bearing to radians
    double lat1 = toRadians(cor.getLat());
    double lon1 = toRadians(cor.getLongi());
    bearing = toRadians(bearing);

    // Calculate the new latitude
    double lat2 = asin(sin(lat1) * cos(distance / R) + cos(lat1) * sin(distance / R) * cos(bearing));

    // Calculate the new longitude
    double lon2 = lon1 + atan2(
        sin(bearing) * sin(distance / R) * cos(lat1),
        cos(distance / R) - (sin(lat1) * sin(lat2))
    );
    //double lon2=lon1-asin(sin(bearing)*sin(distance)/cos(lat1))+PI;
    lon2 = fmod(lon2 + 3 * PI, 2 * PI) - PI; 
    // Convert the new latitude and longitude back to degrees
    return Coordinates(toDegrees(lat2), toDegrees(lon2));
}




//calculate the distance between the fire and the camera using rule 57 
double calculateDistanceUsingRuleOf57(const cv::Rect& objectRect, const Eigen::Matrix3d& K) {
    // Extract the focal length from the K matrix (assuming fx = fy) 
    double fx = K(0, 0);  // Focal length in pixels in the x direction
     double objectWidthReal=0.2;
    // Get the width of the object in the image (in pixels)
    int objectWidthPixels = objectRect.width;

    // Calculate the angular size in degrees
    double angularSize = (objectWidthPixels / fx) * (180.0 / M_PI);  // Convert to degrees

    // Apply Rule of 57 to calculate distance
    double distance = (objectWidthReal / angularSize) * 57;
    
    return distance;
}




// Convert the rectangle center to 3D world coordinates using the camera intrinsics and the bearing
Coordinates findFire(Simulator& simulator, cv::Rect fire,double bearing=180.0) {
    // Get the camera intrinsics (K matrix)
    Eigen::Matrix3d K = simulator.getK();
    if (K.isZero()) {
         std::cerr << "Error loading K" << std::endl;
         return Coordinates();
    }
    // Assume a fixed depth for the fire since we don't have depth info from 2D
    double realWidth =0.2; // assumption , can be replaced with actual width information in actual real life simulation
    double distance=calculateDistanceUsingRuleOf57(fire,K);
    Coordinates cameraCor=simulator.getCoordinates();
    Coordinates objectCoords = getPointAtDistance(cameraCor,distance,bearing);
    // Return the 3D world coordinates of the fire
    return objectCoords;
}


//function finds the largest red contour in frame and saves it in fire
bool findAndDrawLargestRedContour(cv::Mat& frame,cv::Rect& fire) {
    if (frame.empty()) {
        std::cerr << "Empty frame passed to findAndDrawRedContours!" << std::endl;
        return false;
    }
    
    // Convert the frame to HSV color space
    cv::Mat hsv;
    cv::cvtColor(frame, hsv, cv::COLOR_BGR2HSV);
    
    // Define the range of red colors in HSV
    cv::Scalar lowerRed1(0, 100, 100);
    cv::Scalar upperRed1(10, 255, 255);
    cv::Scalar lowerRed2(160, 100, 100);
    cv::Scalar upperRed2(180, 255, 255);

    // Create masks for the red color
    cv::Mat mask1, mask2, mask;
    cv::inRange(hsv, lowerRed1, upperRed1, mask1);
    cv::inRange(hsv, lowerRed2, upperRed2, mask2);
    mask = mask1 | mask2;

    // Find contours in the mask
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    // Find the largest contour by area
    double maxArea = 0;
    int maxIdx = -1;
    for (int i = 0; i < contours.size(); i++) {
        double area = cv::contourArea(contours[i]);
        if (area > maxArea) {
            maxArea = area;

            maxIdx = i;
        }
    }

    if (maxIdx == -1) {
        //std::cerr << "No valid red contours!" << std::endl;
        return false;
    }
    
    

    // Get the bounding box of the largest contour under 50X50 dimentions
    cv::Rect maxRed = cv::boundingRect(contours[maxIdx]);
    if(maxRed.width <=50 && maxRed.height <=50){
            //std::cerr << "No big red contours!" << std::endl;
             return false;
        }
    // Get the center of the red contour
    cv::Point center = (maxRed.tl() + maxRed.br()) * 0.5;

    // Draw the contour and bounding box on the original frame
    cv::drawContours(frame, contours, maxIdx, cv::Scalar(0, 255, 0), 2);
    cv::rectangle(frame, maxRed, cv::Scalar(255, 0, 0), 2);
    cv::circle(frame, center, 5, cv::Scalar(0, 255, 255), -1);  // Mark the center

    // Save the contour 
    fire=maxRed;
    std::cerr << "Fire Found!" << std::endl;
    return true;
}

//function checks if rect is in the center of the frame
bool isFireCentered(cv::Mat& frame, const cv::Rect& rect) {
    // Get frame dimensions
    int frameWidth = frame.cols;
    int frameCenterX = frameWidth / 2;

    // Get the center of the rectangle
    int rectCenterX = rect.x + rect.width / 2;

    // Calculate the offset between the rectangle center and the frame center
    int offsetX = rectCenterX - frameCenterX;

    // Adjust camera yaw based on the offset (positive offset means rect is right of center)
    if (offsetX > 2||offsetX <- 2) {
    	return false;
    } 
    else {
        // Rectangle is already centered, no yaw adjustment needed
        std::cerr << "Fire is centered. No yaw adjustment needed." << std::endl;
       return true;
    }
}


int main(int argc, char **argv) {
    // init manager 
    FirebaseManager firebaseM("1:787797472116:android:8aa92a71cfb1fe63213221","AIzaSyCTt8na_SrZszo95ID_kjCyDimHBS_Kuss","droneapp-701c4","https://droneapp-701c4-default-rtdb.asia-southeast1.firebasedatabase.app");

    
    // Create a reference to the location 
    const std::string fireAd="firelocations/data";
    const std::string droneAd="dronelocation/data";
	

    // Load the generalSettings file
    std::string settingPath = Auxiliary::GetGeneralSettingsPath();
    std::ifstream programData(settingPath);
    nlohmann::json data;
    programData >> data;
    programData.close();

    // Load Config parameters of the simulator
    std::string configPath = data["slam_configuration"]["drone_yaml_path"];
    std::string vocabulary_path = data["slam_configuration"]["vocabulary_path"];
    std::string modelTextureNameToAlignTo = data["simulator_configuration"]["align_model_to_texture"]["texture"];
    bool alignModelToTexture = data["simulator_configuration"]["align_model_to_texture"]["align"];
    std::string model_path = data["simulator_configuration"]["model_path"];
    std::string simulatorOutputDir = data["simulator_configuration"]["simulator_output_dir"];
    std::string loadMapPath = data["slam_configuration"]["load_map_path"];
    double movementFactor = data["simulator_configuration"]["movement_factor"];
    double simulatorStartingSpeed = data["simulator_configuration"]["simulator_starting_speed"];
    bool runWithSlam = data["simulator_configuration"]["run_with_slam"];
    Eigen::Vector3f startingPoint((float)data["simulator_configuration"]["starting_pose"]["x"], (float)data["simulator_configuration"]["starting_pose"]["y"],
                                (float)data["simulator_configuration"]["starting_pose"]["z"]);

    Simulator simulator(configPath, model_path, alignModelToTexture, modelTextureNameToAlignTo, startingPoint, false, simulatorOutputDir, false,
                        loadMapPath, movementFactor, simulatorStartingSpeed, vocabulary_path,32.76245,35.018);
                        
    //Coordinates set to (32.76245,35.0181) - haifa ;
    Coordinates cor=simulator.getCoordinates();

    // Write the Coordinates object to the database
    firebaseM.WriteToReference(droneAd, cor);

    auto simulatorThread = simulator.run();

    // wait for the 3D model to load
    while (!simulator.isReady()) {
        usleep(10);
    }


    // Create a window to display the frames
    cv::namedWindow("Simulator Frame", cv::WINDOW_AUTOSIZE);
    double bearing=0;
    double angleAdv=0.2;
    while (true) {
        usleep(50);
        bool foundFire=false;
        cv::Mat frame = simulator.getFrame();
        if (frame.empty()) {
            std::cerr << "Error loading frame" << std::endl;
            continue;
        }
	// Show the frame in a window - for validation purpouses 
 
        
        
        // Find and draw the largest red contour- for validation purpouses (action is irrelevent to data)
        Coordinates fire_location;
        cv::Rect maxRect;
        
        if(findAndDrawLargestRedContour(frame,maxRect)&&isFireCentered(frame,maxRect)){
             foundFire=true;
        }
        
        
        //Draw frame with contours
        cv::imshow("Simulator Frame", frame);
        
        
        //if the fire was found in the middle set the location
        if(foundFire){
        	fire_location = findFire(simulator, maxRect,bearing);
                // Write the Coordinates object to the database
          	firebaseM.WriteToReference("firelocations/data", fire_location);

        }
        else{
           simulator.yawCamera(angleAdv);
           bearing=(bearing==360)?0:bearing;
           bearing+=angleAdv;
        }
        // Exit if 'q' or 'Esc' key is pressed
        if (cv::waitKey(30) >= 0) {
            break;
        }
    }

    // Clean up and finish
    simulatorThread.join();

    return 0;
}

