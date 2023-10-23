#include "CloudManager.h"
using namespace std;

CloudManager::CloudManager(const std::string &path, int64_t freq, viewer::Renderer &renderer)
{
    path_ = path;
    freq_ = freq;
    renderer_ = &renderer;

    // create the cloud
    cloud_ = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);

    mtxData.lock();
    new_measurement = false;
    mtxData.unlock();
}

void CloudManager::processAndRenderPointCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_f(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);

    // Set the voxel grid
    pcl::VoxelGrid<pcl::PointXYZ> vg;
    vg.setInputCloud(cloud);
    vg.setLeafSize(0.1f, 0.1f, 0.1f);
    vg.filter(*cloud_filtered);

    // Create region based filtering object
    // We crop points that are far away from us, in which we are not interested
    pcl::CropBox<pcl::PointXYZ> cb(true);
    cb.setInputCloud(cloud_filtered);
    cb.setMin(Eigen::Vector4f(-15.0, -5.0, -3, 1.0));
    cb.setMax(Eigen::Vector4f(10.0, 6.0, 3, 1.0));
    cb.filter(*cloud_filtered);

    // Create the segmentation object for the planar model and set all the parameters
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_plane(new pcl::PointCloud<pcl::PointXYZ>());
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setMaxIterations(100);
    seg.setDistanceThreshold(0.25); // determines how close a point must be considered an inlier

    // Remove the planes from the original point cloud
    int i = 0;
    auto nr_points = cloud_filtered->size();
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients());
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices()); // inliers represent the points of the point cloud representing the plane
    bool skip = false;
    while (cloud_filtered->size() > 0.5 * nr_points)
    {
        // Segment the largest planar component from the remaining cloud
        seg.setInputCloud(cloud_filtered);
        seg.segment(*inliers, *coefficients);
        if (inliers->indices.size() == 0)
        {
            std::cout << "Could not estimate a planar model for the given dataset." << std::endl;
            skip = true;
            break;
        }

        // Extract the planar inliers from the input cloud
        pcl::ExtractIndices<pcl::PointXYZ> extract;
        extract.setInputCloud(cloud_filtered);
        extract.setIndices(inliers); // Retrieve indices to all points in cloud_filtered but only those referenced by inliers
        extract.setNegative(false);
        extract.filter(*cloud_plane); // Get the points associated with the planar surface

        // Here we will extract the plane from the original filtered point cloud
        extract.setNegative(true);
        extract.filter(*cloud_f);
        *cloud_filtered = *cloud_f;
        skip = false;
    }
    if (!skip)
    {
        // Creating the KdTree object for the search method of the clustering method
        pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
        std::vector<pcl::PointIndices> cluster_indices;
        pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;

        // Set the spatial tolerance for new cluster candidates
        tree->setInputCloud(cloud_filtered);
        ec.setClusterTolerance(0.15);

        // We impose that the clusters found must have at least 60 points and maximum 600 points
        ec.setMinClusterSize(60);
        ec.setMaxClusterSize(600);
        ec.setSearchMethod(tree);
        ec.setInputCloud(cloud_filtered);

        // Extract the clusters
        ec.extract(cluster_indices);

        int clusterId = 0;
        std::vector<double> centroids_x;
        std::vector<double> centroids_y;
        std::vector<double> centroids_z;

        std::vector<viewer::Box> boxes;

        for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin(); it != cluster_indices.end(); ++it)
        {
            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster(new pcl::PointCloud<pcl::PointXYZ>);
            for (std::vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); ++pit)
                cloud_cluster->push_back((*cloud_filtered)[*pit]);

            cloud_cluster->width = cloud_cluster->size();
            cloud_cluster->height = 1;
            cloud_cluster->is_dense = true;

            // Point cloud bounding boxes
            if (renderer_->getLidarStatus())
            {
                pcl::PointXYZ minPt, maxPt;
                pcl::getMinMax3D(*cloud_cluster, minPt, maxPt);

                viewer::Box b{minPt.x, minPt.y, minPt.z,
                              maxPt.x, maxPt.y, maxPt.z};
                boxes.push_back(b);

                centroids_x.push_back((minPt.x + maxPt.x) / 2.0);
                centroids_y.push_back((minPt.y + maxPt.y) / 2.0);
                centroids_z.push_back((minPt.z + maxPt.z) / 2.0);
            }
        }

        // update the data using mutex to avoid concurrency
        mtxData.lock();

        centroids_x_ = centroids_x;
        centroids_y_ = centroids_y;
        centroids_z_ = centroids_z;
        boxes_ = boxes;
        copyPointCloud(*cloud_filtered, *cloud_);
        new_measurement = true;

        mtxData.unlock();
    }
}

void CloudManager::startCloudManager()
{

    pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    std::vector<boost::filesystem::path> stream(boost::filesystem::directory_iterator{path_},
                                                boost::filesystem::directory_iterator{});

    // sort files in ascending (chronological) order
    std::sort(stream.begin(), stream.end());
    auto streamIterator = stream.begin();

    while (true && streamIterator != stream.end())
    {
        auto startTime = std::chrono::steady_clock::now();

        // read pointcloud from file
        pcl::PCDReader reader;
        reader.read(streamIterator->string(), *input_cloud);

        // process pointcloud
        processAndRenderPointCloud(input_cloud);

        // update iterator
        streamIterator++;

        // force frequency to this process
        auto endTime = std::chrono::steady_clock::now();
        auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
        int64_t sleep_for_milliseconds = freq_ - elapsedTime.count();
        if (elapsedTime.count() < freq_)
            std::this_thread::sleep_for(std::chrono::milliseconds(sleep_for_milliseconds));
    }
}
