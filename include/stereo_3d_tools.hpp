/**
 * @file stereo_3d_tools.hpp
 * @brief This class is responsable for transforming the disparity map into a point cloiud on the 3d space. Also it provides opertions like spinning and segmentation in 3d space.
 */
#ifndef STEREO_3D_TOOLS_HPP_
#define STEREO_3D_TOOLS_HPP_

#include <opencv2/core/core.hpp>
#include <opencv2/calib3d/calib3d.hpp>

/**
 * @brief This class is part of the stereo_camera_calibrator namespace
 */
namespace stereo_camera_calibrator{

/**
 * @brief Enum defining the possible axis for operations like rotation and translation. 
 * The flags with INV are going to be interpreted as a negative axis. For example, the
 * highest point on axis X_AXIS_INV is the lowest point on axis X_AXIS.
 */
enum axis_index {
    X_AXIS,
    Y_AXIS,
    Z_AXIS,
    X_AXIS_INV,
    Y_AXIS_INV,
    Z_AXIS_INV
};

class Stereo3DTools {
    public:
        /**
         * @brief This class keeps a copy of the projected points for itself in case the user want to 
         * return to the original state. For that reason a disparity map has to be provided toguether with
         * the rectification matrix Q
         * @param[in] cv::Mat disparity Image containing the disparity map as a 8UC1 image.
         * @param[in] cv::Mat Q matrix accquired after rectification.
         */
        Stereo3DTools(cv::Mat disparity, cv::Mat Q);
        /**
         * @brief Alternative constructor where no disparity map is provided. A disparity map has to be set later
         */
        Stereo3DTools();
        /**
         * @brief Destructor of the class
         */
        ~Stereo3DTools();

        /**
         * @brief Test if the projection points are ready. If not, the updateProjectedPoints need to be called
         * @return true if the points are ready
         */
        bool isProjectedReady();
        /**
         * @brief Updates the point cloud using the disparity map and rectification matrix Q (disparity-to-depth)
         * @param[in] cv::Mat disparity Image containing the disparity map as a 8UC1 image.
         * @param[in] cv::Mat Q matrix accquired after rectification.
         */
        void updateProjectedPoints(cv::Mat disparity, cv::Mat Q);
        /**
         * @brief Directly assign a point cloud to the object point cloud.
         * @note The matrix need to be a 32F3C where each point has the xyz coordinates
         * @param[in] cv::Mat points_matrix Matrix containing the point cloud
         * @param[in] cv::Mat disparity Disparity map of the new projection
         */
        void manualProjectedPoints(cv::Mat points_matrix, cv::Mat disparity);
        /**
         * @brief Returns the original point cloud matrix
         * @return cv::Mat 32F3C matrix with the point cloud
         */
        cv::Mat getOriginalPointMatrix();
        /**
         * @brief Return the point cloud after suffering modifications like rotation.
         * @return cv::Mat 32F3C matrix with the point cloud
         */
        cv::Mat getModifiedPointMatrix();

        /**
         * @brief Rotate the points around the specified axis. 
         * @note The angle is in degrees. It is converted to radians internely
         * @param[in] float angle By how many degrees the point cloud will rotate
         * @param[in] axis_index axis Around which axis this rotation will take place.
         */
        void rotatePoints(float angle, axis_index axis);
        /**
         * @brief Translate the pointcloud around the specified axis
         * @param float distance By how many voxels the point will move. The conversion from voxels
         * to meters can be done with the calibration params.
         * @param[in] float distance By how many voxels the point cloud will move
         * @param[in] axis_index axis Over which axis the points will translate
         */
        void translatePoints(float distance, axis_index axis);
        /**
         * @brief Cancels all operations over the point cloud returning it to its original state
         */
        void returnToOriginalPoints();
        /**
         * @brief Every point on the object point cloud that is closer than the 
         * specified threshold, it will be eliminated.
         * @note to reduce the amount of processing both the target points and the computed_points inside the object
         * have to be on the same orientation.
         * @param[in] cv::Mat target_points_disp This are the disparity image of the base points from where if points of the cloud
         * are close enough will be removed
         * @param[in] float distance_threshold How much distance to consider that the point is close
         * and prone to removal. 
         */
        void removePointsCloseTo(cv::Mat target_points_disp, float distance_threshold=0.5);

        /**
         * @brief Delete all the points that are not inside the specified enclosement box
         * @note The two points provided correspond to the close-top-right and far-botton-left corners to
         * create the enclosement box
         * @param[in] cv::Point3f top_corner close-top-right point
         * @param[in] cv::Point3f lower_corner far-botton-left point
         */
        void removeOutsideTheBox(cv::Point3f close_top_right_corner, cv::Point3f far_botton_left_corner);

        /**
         * @brief Implementation of the erode function in a 3d space
         * @note since the 3d image generated by a stereo image is layaled like a pile of paper, a pure translation of
         * the erode algorithm into 3d would end up deleting all the voxels
         * @param[in] int cube_size size of the cube to be considered for erosion
         * @param[in] int minimum_number if there is less than the minimum on the cube, the voxel is eroded
         */
        void erode3d(int cube_size, int minimum_number);

        /**
         * @brief Return the highest point regarding one axis of the point cloud
         * @note To find the lowest point on a axis, just need to use its _INV versions
         * @param[in] axis_index axis On which axis the highest point is being searched
         * @return std::vector<cv::Point3f> List of coordinates of the pickable points in 3D
         */
        std::vector<cv::Point3f> getHighestPoint(axis_index axis = Z_AXIS, int distance_threshold = 10);

        /**
         * @brief Changes the coordinade origin and sense but not its direction
         * @param[in] std::vector<cv::Point3f> original_points the original points
         * @param[in] cv::Point3f new_origin the new origin position
         * @param[in[ cv::Point3f new_sense the new sense of this origin, this is a 1 or -1 only point
         * @return std::vector<cv::Point3f> the same points in the new coordinate system
         */
        std::vector<cv::Point3f> shiftCoordinates(std::vector<cv::Point3f> original_points, cv::Point3f new_origin, cv::Point3f new_sense);

        //---file handling functions------
        /**
         * @brief Saves the computed point cloud (or the original one in case no operations were made) into a
         * .ply file for the Meshlab to load
         * @param[in] std::string filename Where to save
         * @param[in] std::vector<cv::Point3f> pointlist list of points belonging to the pickupzone. For debug purpose
         */
        void saveComputedPoints(std::string filename, cv::Mat rgb_img);
        void saveComputedPoints(std::string filename, cv::Mat rgb_img, std::vector<cv::Point3f> pointlist);

        cv::Mat getDisparityMap();
    private:
        cv::Mat projected_points, computed_points;
        cv::Mat Qmatrix;
        cv::Mat orig_disp_img, disp_img;
        int failed_pixel;
        void convertToPointCloud(cv::Mat disparity_map, cv::Mat Q);
        float pointDistance(cv::Point3f pt1, cv::Point3f pt2);
        cv::Point3f shitftOnePoint(cv::Point3f target, cv::Point3f new_origin, cv::Point3f new_sense);
};

}

#endif
