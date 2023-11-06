#include "tracker/Tracker.h"

#include <iostream>
#include <cmath>

/**
 * Tracker constructor initializes tracking parameters and sets the distance measurement method.
 *
 * This constructor initializes the Tracker object by setting various tracking parameters, 
 * including the current track ID, distance threshold, covariance threshold, and loss threshold. 
 * Additionally, it sets the distance measurement method using the 'DISTANCE_TYPE_METHOD' variable, 
 * with a default value of "ED" for Euclidean Distance.

 * @remark The constructor provides default values for tracking parameters and specifies the initial distance measurement method for object tracking.
 */
Tracker::Tracker()
{
    cur_id_ = 0;
    distance_threshold_ = 0.5; // meters
    covariance_threshold = 0.05; 
    loss_threshold = 10; //number of frames the track has not been seen
    DISTANCE_TYPE_METHOD = "ED";

    std::cout<<"Using default method euclideanDistance()"<<std::endl;
}

/**
 * Tracker constructor with customizable distance measurement method.
 *
 * This constructor initializes the Tracker object with the option to customize the distance measurement method. 
 * It sets various tracking parameters, including the current track ID, distance threshold, covariance threshold, 
 * and loss threshold. The 'distance_method' parameter allows users to choose between 
 * "ED" (Euclidean Distance) and "MD" (Mahalanobis Distance) for distance measurement. 
 * The default distance measurement method is "ED," and a corresponding message is displayed.

 * @param distance_method A string specifying the desired distance measurement method, either "ED" for Euclidean Distance or "MD" for Mahalanobis Distance.

 * @remark The constructor provides the flexibility to select the distance measurement method while 
 setting default values for tracking parameters and displaying a message about the chosen method.
 */
Tracker::Tracker(std::string distance_method)
{
    // Initialize tracking parameters
    cur_id_ = 0;
    distance_threshold_ = 0.5; // meters
    covariance_threshold = 0.05; 
    loss_threshold = 10; //number of frames the track has not been seen

    if (distance_method == "MD") {
        // Set the selected distance measurement method to "MD"
        DISTANCE_TYPE_METHOD = distance_method;
        std::cout<<"Using method mahalaobisDistance()"<<std::endl;
    } else {
        // Set the default distance measurement method to "ED"
        DISTANCE_TYPE_METHOD = "ED";
        std::cout<<"Using method euclideanDistance()"<<std::endl;
    }
}

/**
 * Tracker destructor.
 *
 * The destructor for the Tracker class. 
 * This destructor does not perform any additional cleanup or resource release as the class 
 * does not manage any dynamic resources directly.
 */
Tracker::~Tracker() {}

/**
 * Removes tracks from the tracker based on covariance thresholds and loss counts.
 *
 * This function iterates through the `tracks_` vector and removes tracks that do not meet specified covariance thresholds or loss count criteria. 
 * Tracks with covariances below the 'covariance_threshold' and loss counts below the 'loss_threshold' are retained in the tracker.

 * @remark The `tracks_` vector will be updated to contain only the tracks that meet the specified criteria.
 * 
 * @note The function swaps the updated vector of tracks back into the `tracks_` member variable, effectively removing unwanted tracks from the tracker.
 */
void Tracker::removeTracks()
{
    // Vector of `Tracklet` to keep in the scene.
    // At the end of the logic this struct will be reversed
    // (because of `push_back()` method) into the `tracks_` vector.
    std::vector<Tracklet> tracks_to_keep;

    for (auto &track : tracks_) {
        if (track.getXCovariance() < covariance_threshold && track.getYCovariance() < covariance_threshold) {
            if (track.getLossCount() < loss_threshold)
                tracks_to_keep.push_back(track);
        }
    }

    // Update the `tracks_` vector with the selected tracks.
    tracks_.swap(tracks_to_keep);
}

/**
 * Adds new tracks to the tracker based on associated detections.
 *
 * This function adds new tracks to the tracker for detections that are not associated with existing tracks. 
 * It iterates through the `associated_detections` vector and for each `false` value, 
 * creates a new tracklet using the corresponding centroid coordinates from 'centroids_x' and 'centroids_y'.

 * @param associated_detections A vector of boolean values indicating whether each detection is associated with an existing track.
 * @param centroids_x A vector of x-coordinates of the detected centroids.
 * @param centroids_y A vector of y-coordinates of the detected centroids.

 * @remark The function increments the track ID for each new track created and appends them to the 'tracks_' vector, effectively adding new tracks to the tracker.
 */
void Tracker::addTracks(const std::vector<bool> &associated_detections, const std::vector<double> &centroids_x, const std::vector<double> &centroids_y)
{
    // Adding not associated detections
    for (size_t i = 0; i < associated_detections.size(); ++i)
        if (!associated_detections[i])
            tracks_.push_back(Tracklet(cur_id_++, centroids_x[i], centroids_y[i]));
}

/**
 * Calculates the Euclidean distance between a tracklet's position and a point (x, y).
 *
 * @param track A reference to a Tracklet object representing the track's position.
 * @param x The x-coordinate of the point.
 * @param y The y-coordinate of the point.
 *
 * @return The Euclidean distance between the tracklet and the specified point.
 */
double euclideanDistance(Tracklet &track, double x, double y) {
    // Calculate the difference in x and y coordinates
    double x_diff = track.getX() - x;
    double y_diff = track.getY() - y;

    // Calculate the squared distance
    double squared_distance = x_diff * x_diff + y_diff * y_diff;

    // Calculate the actual Euclidean distance by taking the square root
    double distance = sqrt(squared_distance);

    return distance;
}

/**
 * Calculates the Mahalanobis Distance between a tracklet's position and a point (x, y).
 *
 * This function calculates the Mahalanobis Distance between a tracklet's position and a point (x, y) using the tracklet's covariances for x and y. 
 * The Mahalanobis Distance is a measure of the dissimilarity between the tracklet's position and the point, 
 * taking into account the uncertainties in both dimensions.

 * @param track A Tracklet object representing the track's position and covariances.
 * @param x The x-coordinate of the point.
 * @param y The y-coordinate of the point.

 * @return The Mahalanobis Distance between the tracklet and the specified point.
 */
double mahalanobisDistance(Tracklet track, double x, double y) {
    // Calculate the difference between the track point and the centroid
    Eigen::Vector2d diff(track.getX() - x, track.getY() - y);

    // Calculate the Mahalanobis Distance using the separate covariances
    double distance = std::sqrt(
        (diff[0] * diff[0] / track.getXCovariance()) + (diff[1] * diff[1] / track.getYCovariance())
    );

    return distance;
}

/**
 * Associates detections with existing tracks based on distance measures.
 *
 * This function performs data association to associate detected centroids with existing tracks. 
 * It calculates the distance between each detection and each track using either Mahalanobis Distance or Euclidean Distance based on the value of the 'DISTANCE_TYPE_METHOD'. 
 * It associates each detection with the closest track if the distance is below the specified 'distance_threshold'.

 * @param associated_detections A vector of boolean values indicating whether each detection is already associated with a track.
 * @param centroids_x A vector of x-coordinates of the detected centroids.
 * @param centroids_y A vector of y-coordinates of the detected centroids.

 * @remark The function updates the 'associated_detections' vector to mark detections that are associated with tracks and maintains a list of associated track-detection pairs in the 'associated_track_det_ids_' vector.
 */
void Tracker::dataAssociation(std::vector<bool> &associated_detections,
                            const std::vector<double> &centroids_x,
                            const std::vector<double> &centroids_y) {
    // Clear the list of associated track-detection pairs
    associated_track_det_ids_.clear();

    for (size_t i = 0; i < tracks_.size(); ++i)
    {
        int closest_point_id = -1;
        double min_dist = std::numeric_limits<double>::max();

        for (size_t j = 0; j < associated_detections.size(); ++j)
        {
            // Calculate the distance between the current track and detection.
            //
            // Logic to find the closest detection (centroids_x,centroids_y) 
            // to the current track (tracks_)
            //
            // - Mahalaobis Distance
            // 
            // - Euclidean Distance
            
            double distance = 0.0;
            if (DISTANCE_TYPE_METHOD == "MD")
                // Mahalaobis Distance
                distance = mahalanobisDistance(tracks_[i], centroids_x[j], centroids_y[j]);
            else
                // Euclidean Distance
                distance = euclideanDistance(tracks_[i], centroids_x[j], centroids_y[j]);

            // Update the closest detection if the distance is smaller
            if (distance < min_dist) {
                closest_point_id = j;
                min_dist = distance;
            }            
        }
        
        // Associate the closest detection to a tracklet
        if (min_dist < distance_threshold_ && !associated_detections[closest_point_id])
        {
            // Add the association to the list
            associated_track_det_ids_.push_back(std::make_pair(closest_point_id, i));
            // Mark the detection as associated
            associated_detections[closest_point_id] = true;
        }
    }
}

/**
 * Tracks objects in a scene using measurements of detected centroids.
 *
 * This function tracks objects in the scene based on measurements of detected centroids. 
 * It predicts the positions of existing tracks, associates these predictions with detections, 
 * updates the tracks with the associated measurements, removes tracklets that no longer meet specified criteria, 
 * and adds new tracklets for unassociated detections.

 * @param centroids_x A vector of x-coordinates of the detected centroids.
 * @param centroids_y A vector of y-coordinates of the detected centroids.
 * @param lidarStatus A boolean flag indicating the status of the Lidar sensor.

 * @remark The function performs multiple steps in object tracking, including prediction, data association, 
 update, removal, and addition of tracklets. It maintains the state of existing tracks in the 'tracks_' vector.
 */
void Tracker::track(const std::vector<double> &centroids_x,
                    const std::vector<double> &centroids_y,
                    bool lidarStatus) {
    // Initialize a vector to track associated detections
    std::vector<bool> associated_detections(centroids_x.size(), false);

    // Predict the position of each tracklet
    for (auto &track : tracks_)
        track.predict();
    
    // Associate the predictions with the detections
    dataAssociation(associated_detections, centroids_x, centroids_y);

    // Update tracklets with the new detections
    for (auto &associated_track : associated_track_det_ids_) {
        auto det_id = associated_track.first;
        auto track_id = associated_track.second;
        tracks_[track_id].update(centroids_x[det_id], centroids_y[det_id], lidarStatus);
    }

    // Remove tracklets that no longer meet specified criteria
    removeTracks();

    // Add new tracklets for unassociated detections
    addTracks(associated_detections, centroids_x, centroids_y);
}
