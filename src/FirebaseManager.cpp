#include "FirebaseManager.h"
#include <iostream>

// Constructor to initialize Firebase app and database
FirebaseManager::FirebaseManager(const std::string& app_id, const std::string& api_key, 
                                 const std::string& project_id, const std::string& database_url) {
    // Set Firebase App options
    firebase::AppOptions app_options;
    app_options.set_app_id(app_id.c_str()); 
    app_options.set_api_key(api_key.c_str()); 
    app_options.set_project_id(project_id.c_str()); 
    app_options.set_database_url(database_url.c_str());

    // Initialize the Firebase app
    app = firebase::App::Create(app_options);

    // Initialize the Firebase Realtime Database
    database = firebase::database::Database::GetInstance(app);
    if (!database) {
        std::cerr << "Failed to initialize the database." << std::endl;
    }
}

// Destructor to clean up resources
FirebaseManager::~FirebaseManager() {
    delete app;
}

// Write a value to a given reference address in Firebase Realtime Database- only write - source code
void FirebaseManager::WriteToReference(const std::string& reference_address, const Coordinates& cor) {
    firebase::database::DatabaseReference ref = database->GetReference(reference_address.c_str());
    std::vector<double> value={cor.getLat(),cor.getLongi()};
    firebase::Future<void> result = ref.SetValue(value);
    WaitForCompletion(result, "Write to " + reference_address);
}


// Helper function to wait for an operation to complete
void FirebaseManager::WaitForCompletion(const firebase::FutureBase& future, const std::string& operation) {
    while (future.status() == firebase::kFutureStatusPending) {
        //std::cout << operation << " in progress..." << std::endl;
        continue;
    }

    if (future.status() == firebase::kFutureStatusComplete && future.error() == 0) {
        std::cout << operation << " completed successfully." << std::endl;
    } else {
        std::cerr << operation << " failed: " << future.error_message() << std::endl;
    }
}

