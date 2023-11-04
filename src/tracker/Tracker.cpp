#include "tracker/Tracker.h"

#include <iostream>
#include <cmath>

Tracker::Tracker()
{
    cur_id_ = 0;
    distance_threshold_ = 1.0; // meters
    covariance_threshold = 0.0; 
    loss_threshold = 5; //number of frames the track has not been seen
}
Tracker::~Tracker()
{
}

/*
    This function removes tracks based on any strategy
*/
void Tracker::removeTracks()
{
    // Debug flag to keep all tracklet and ignore the logic.
    bool logic_to_keep = true;

    // Vector of `Tracklet` to keep in the scene.
    // At the end of the logic this struct will be reversed
    // (because of `push_back()` method) into the `tracks_` vector.
    std::vector<Tracklet> tracks_to_keep;

    // for (size_t i = 0; i < tracks_.size(); ++i)
    // {
    //     // TODO
    //     // Implement logic to discard old tracklets
    //     // logic_to_keep is a dummy placeholder to make the code compile and should be subsituted with the real condition
    //     bool logic_to_keep = true;
    //     if (logic_to_keep)
    //         tracks_to_keep.push_back(tracks_[i]);
    // }
    for (auto &track : tracks_) {
        // std::cout<<track.getId()<<std::endl;
        if (track.getLossCount() < loss_threshold)
            tracks_to_keep.push_back(track);
    }
        

    tracks_.swap(tracks_to_keep);
}

/*
    This function add new tracks to the set of tracks ("tracks_" is the object that contains this)
*/
void Tracker::addTracks(const std::vector<bool> &associated_detections, const std::vector<double> &centroids_x, const std::vector<double> &centroids_y)
{
    // Adding not associated detections
    for (size_t i = 0; i < associated_detections.size(); ++i)
        if (!associated_detections[i])
            tracks_.push_back(Tracklet(cur_id_++, centroids_x[i], centroids_y[i]));
}

double euclideanDistance(Tracklet &track, double x, double y) {
    double x_diff = track.getX() - x;
    double y_diff = track.getY() - y;
    double squared_distance = x_diff * x_diff + y_diff * y_diff;
    double distance = sqrt(squared_distance);
    return distance;
}

/*
    This function associates detections (centroids_x,centroids_y) with the tracks (tracks_)
    Input:
        associated_detection an empty vector to host the associated detection
        centroids_x & centroids_y measurements representing the detected objects
*/
void Tracker::dataAssociation(std::vector<bool> &associated_detections,
                            const std::vector<double> &centroids_x,
                            const std::vector<double> &centroids_y) {
    // Remind this vector contains a pair of tracks and its corresponding
    associated_track_det_ids_.clear();

    // std::cout<<tracks_.size()<<std::endl;
    for (size_t i = 0; i < tracks_.size(); ++i)
    {

        int closest_point_id = -1;
        double min_dist = std::numeric_limits<double>::max();

        for (size_t j = 0; j < associated_detections.size(); ++j)
        {
            // TODO:
            //
            // - Implement logic to find the closest detection (centroids_x,centroids_y) 
            // to the current track (tracks_)
            //
            // + Mahalaobis Distance
            // 
            // + args to switch between ED e MD
            double distance = euclideanDistance(tracks_[i], centroids_x[j], centroids_y[j]);

            if (distance < min_dist) {
                closest_point_id = j;
                min_dist = distance;
                // associated_detections[j] = true;
            } 
                
            
        }
        std::cout<<"Min dist= "<<min_dist<<" - Threshold= "<<distance_threshold_<<std::endl;
        std::cout<<"Index= "<<closest_point_id<<" - Detection= "<<associated_detections[closest_point_id]<<std::endl;
        // Associate the closest detection to a tracklet
        if (min_dist < distance_threshold_ && !associated_detections[closest_point_id])
        {
            std::cout<<"Push in"<<std::endl;
            associated_track_det_ids_.push_back(std::make_pair(closest_point_id, i));
            associated_detections[closest_point_id] = true;
        }
    }
}

void Tracker::track(const std::vector<double> &centroids_x,
                    const std::vector<double> &centroids_y,
                    bool lidarStatus) {

    std::vector<bool> associated_detections(centroids_x.size(), false);

    // TODO: Predict the position
    //For each track --> Predict the position of the tracklets
    for (auto &track : tracks_)
        track.predict();
    
    // TODO: Associate the predictions with the detections
    dataAssociation(associated_detections, centroids_x, centroids_y);

    // New version with foreach, to make it more readable!
    //
    // Update tracklets with the new detections

    std::cout<<"Associated_track_det_ids_size = "<<associated_track_det_ids_.size()<<std::endl;
    for (auto &associated_track : associated_track_det_ids_) {
        auto det_id = associated_track.first;
        auto track_id = associated_track.second;

        std::cout<<det_id<<" - "<<track_id<<std::endl;
        tracks_[track_id].update(centroids_x[det_id], centroids_y[det_id], lidarStatus);
    }

    // TODO: Remove dead tracklets
    removeTracks();

    // TODO: Add new tracklets
    addTracks(associated_detections, centroids_x, centroids_y);
}
