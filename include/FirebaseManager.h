#ifndef FIREBASE_MANAGER_H
#define FIREBASE_MANAGER_H
#include "Coordinates.h"
#include "firebase/app.h"
#include "firebase/database.h"
#include <string>

class FirebaseManager {
public:
    // Constructor to init app and database
    FirebaseManager(const std::string& app_id, const std::string& api_key, 
                    const std::string& project_id, const std::string& database_url);

    // Destructor to clean up resources and app
    ~FirebaseManager();

    // Write a value to a given reference address in Firebase Realtime Database (by string adress)
    void WriteToReference(const std::string& reference_address, const Coordinates& value);


private:
    firebase::App* app;
    firebase::database::Database* database;

    // Helper function to wait for an operation to complete- keep from overwriting 
    void WaitForCompletion(const firebase::FutureBase& future, const std::string& operation);
};

#endif 
