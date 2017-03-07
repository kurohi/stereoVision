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
         * return to the original state. For that reason a disparity map has to be provided together with
         * the rectification matrix Q
         * @throws runtime_error if the provided bg_mask erases the whole image 
         * @param[in] cv::Mat disparity Image containing the disparity map as a 16SC1 image.
         * @param[in] cv::Mat Q matrix accquired after rectification.
         * @param[in] cv::Mat BG_mask background substration mask to be removed from the disparity before projection
         */
        Stereo3DTools(cv::Mat disparity, const cv::Mat &Q, cv::Mat BG_mask=cv::Mat(0,0,CV_16SC1));
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
        void updateProjectedPoints(const cv::Mat &disparity, const cv::Mat &Q);
        /**
         * @brief Updates the point cloud using the disparity map and rectification matrix Q (disparity-to-depth)
         * @param[in] cv::Mat disparity Image containing the disparity map as a 8UC1 image.
         * @param[in] cv::Mat Q matrix accquired after rectification.
         */
        void updateComputedPoints(const cv::Mat &disparity, const cv::Mat &Q);
        /**
         * @brief Directly assign a point cloud to the object point cloud.
         * @note The matrix need to be a 32F3C where each point has the xyz coordinates
         * @throws runtime_error when a invalid point cloud is recieved
         * @param[in] cv::Mat points_matrix Matrix containing the point cloud
         * @param[in] cv::Mat disparity Disparity map of the new projection
         */
        void manualProjectedPoints(const cv::Mat &points_matrix, const cv::Mat &disparity);
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
        void rotatePoints(const float angle, const axis_index axis);

        /**
         *  @brief Eliminate all points starting from a provided point in a provided direction
         *  @note Uses the same flags from rotatePoints to define the axis
         *  @throws runtime_error If the pointcloud becomes empty after removal, an runtime_error exception is thrown
         *  @param[in] cv::Point3f the point form which the deletion will start
         *  @param[in] axis_index the direction to which the deletion spreads
         */
        void removeAllFromPoint(const cv::Point3f &start_pt, const axis_index axis);

        /**
         * @brief Calculates the needed rotaion matrix to rotate the space by the specified angle on the specified axis
         * @param[in] angle The amount to be rotated
         * @param[in] axis Over which axis the point cloud will be spinning
         * @param[out] rotation_matrix The matrix to be used when rotatin the point cloud
         */
        void getRotationMatrix(const float angle, const axis_index axis, cv::Mat &rotation_matrix);
        /**
         * @brief Applay a 3x3 rotation matrix on every point of the point cloud
         * @param[in] rotation_matrix The matrix customly crafted or calculated with getRotationMatrix
         */
        void applyRotationMatrix(const cv::Mat &rotation_matrix);
        /**
         * @brief Applay a 3x3 rotation matrix on 1 point of the point cloud based on the 2d axis
         * @param[in] point point on 2d image to do the rotation
         * @param[in] rotation_matrix The matrix customly crafted or calculated with getRotationMatrix
         * @param[in] on_computed_points use the point in computed_points or projected_points
         *
         * @return cv::Point3f axis info in 3d after rotation
         */
        cv::Point3f applyRotationMatrix(const cv::Point& point, const cv::Mat &rotation_matrix, bool on_computed_points = true);
        /**
         * @brief Applay a 3x3 rotation matrix on 1 point of the point cloud
         * @param[in] point point on point cloud to do the rotation
         * @param[in] rotation_matrix The matrix customly crafted or calculated with getRotationMatrix
         *
         * @return cv::Point3f axis info in 3d after rotation
         */
         cv::Point3f applyRotationMatrix(const cv::Point3f& point, const cv::Mat &rotation_matrix);

        /**
         * @brief Translate the pointcloud around the specified axis
         * @param float distance By how many voxels the point will move. The conversion from voxels
         * to meters can be done with the calibration params.
         * @param[in] float distance By how many voxels the point cloud will move
         * @param[in] axis_index axis Over which axis the points will translate
         */
        void translatePoints(const float distance, const axis_index axis);
        /**
         * @brief Cancels all operations over the point cloud returning it to its original state
         */
        void returnToOriginalPoints();
        /**
         * @brief Every point on the object point cloud that is closer than the 
         * specified threshold, it will be eliminated.
         * @note to reduce the amount of processing both the target points and the computed_points inside the object
         * have to be on the same orientation.
         * @throws runtime_error If the pointcloud becomes empty after removal, an runtime_error exception is thrown
         * @param[in] cv::Mat target_points_disp This are the disparity image of the base points from where if points of the cloud
         * are close enough will be removed
         * @param[in] float distance_threshold How much distance to consider that the point is close
         * and prone to removal. 
         */
        void removePointsCloseTo(const cv::Mat &target_points_disp, const float distance_threshold=0.5);

        /**
         * @brief Delete all the points that are not inside the specified enclosement box
         * @note The two points provided correspond to the close-top-right and far-botton-left corners to
         * create the enclosement box
         * @throws runtime_error If the pointcloud becomes empty after removal, an runtime_error exception is thrown
         * @param[in] cv::Point3f top_corner close-top-right point
         * @param[in] cv::Point3f lower_corner far-botton-left point
         */
        void removeOutsideTheBox(const cv::Point3f &close_top_right_corner, const cv::Point3f &far_botton_left_corner);

        /**
         * @brief Implementation of the erode function in a 3d space
         * @note since the 3d image generated by a stereo image is layaled like a pile of paper, a pure translation of
         * the erode algorithm into 3d would end up deleting all the voxels
         * @throws runtime_error if the the whole pointcloud is segmented out 
         * @param[in] int cube_size size of the cube to be considered for erosion
         * @param[in] int minimum_number if there is less than the minimum on the cube, the voxel is eroded
         */
        void erode3d(const int cube_size, const int minimum_number);

        /**
         * @brief Return the highest point regarding one axis of the point cloud
         * @note To find the lowest point on a axis, just need to use its _INV versions
         * @throws runtime_error if the provided an inexistent axis_index
         * @param[in] axis_index axis On which axis the highest point is being searched
         * @return std::vector<cv::Point3f> List of coordinates of the pickable points in 3D
         */
        std::vector<cv::Point3f> getHighestPoint(const axis_index axis = Z_AXIS, const int distance_threshold = 10);

        /**
         * @brief Changes the coordinade origin and sense but not its direction
         * @param[in] std::vector<cv::Point3f> original_points the original points
         * @param[in] cv::Point3f new_origin the new origin position
         * @param[in[ cv::Point3f new_sense the new sense of this origin, this is a 1 or -1 only point
         * @return std::vector<cv::Point3f> the same points in the new coordinate system
         */
        std::vector<cv::Point3f> shiftCoordinates(const std::vector<cv::Point3f> &original_points, const cv::Point3f &new_origin, const cv::Point3f &new_sense);

        //---file handling functions------
        /**
         * @brief Saves the computed point cloud (or the original one in case no operations were made) into a
         * .ply file for the Meshlab to load
         * @throws runtime_error If could not create the file to save the points
         * @param[in] std::string filename Where to save
         * @param[in] cv::Mat rgb_img color image from where the voxel will take its color 
         */
        void saveComputedPoints(const std::string &filename, const cv::Mat &rgb_img);
        /**
         * @brief Alternative interface for the saveComputedPoints that accepts a list o points to paint red on the .ply file
         * @param[in] std::string filename Where to save
         * @param[in] std::vector<cv::Point3f> pointlist list of points belonging to the pickupzone. For debug purpose
         * @param[in] std::vector<cv::Point3f> pointlist List of 3d points to be painted red
         */
        void saveComputedPoints(const std::string &filename, const cv::Mat &rgb_img, const std::vector<cv::Point3f> &pointlist);

        /**
         * @brief Select the biggest connected region from the disparity map. This is done to remove small noises that are bound to appear.
         * @throws runtime_error in case no contour is found. This can happen when a completly black disparity map is provided
         */
        void filterToBiggestRegion();

        /**
         * @brief Return the current disparity map saved on the object. If there were segmentations on the 3d volume, the disparity returned is with these segmentations. 
         * @note The disparity map is on 16SC1, so if going to be used as GUI output, need to convert into 8UC1
         * @return cv::Mat The disparity map as a 16SC1 image
         */
        cv::Mat getDisparityMap();

        /**
         * @brief Return the 3D point from a 2D coodinate (on the disparity) map. 
         * @note No error check is done here. So if a invalid pixel is selected, a point to infinity might be returned
         * @throws runtime_error If the point to be retraced does not hit any existing one on the point cloud
         * @param[in] cv::Point search_point The 2D point on the disparity from where the 3D point will be searched
         * @return cv::Point3f 3D point in float
         */
        cv::Point3f retraceXYZfrom2D(const cv::Point &search_point);

        /**
         * @brief Retrace the 2D point (coordinates on dispairty) from a 3D coordinate
         * @throws runtime_error If the retrace point is does not hit any existing one on the point cloud
         * @param[in] cv::Point3f Search point in the 3D space
         * @return cv::Point The 2D coordinate on the disparity map
         * @excepion throw a runtime_error exception in case of the 3D point having no correspondence in the 2D disparity map
         */
        cv::Point retraceXYfrom3D(const cv::Point3f &search_point);

        /**
         * @brief Retrace the closet 2D valid pixel on the disparity for a 3D point inside the provided range.
         * @throws runtime_error If the retrace point is does not hit (or get closer than range) any existing one on the point cloud
         * @param[in] cv::Point3f Search point in the 3D space
         * @param[in] float range The maximum distance to search for a valid voxel to find its 2D correspondance on the disparity map
         * @return cv::Point The 2D coordinate on the disparity map
         * @excepion throw a runtime_error exception in case of the 3D point having no correspondence in the 2D disparity map
         */
        cv::Point retraceXYfrom3D(const cv::Point3f &search_point, const float range);

        /**
         * @brief Retrace the closet 2D valid pixel on the disparity for a 3D point inside the provided range but ignoring the Z axis
         * @throws runtime_error if the retrace point does not hit (or get closer than range) any existing one on the point cloud
         * @param[in] cv::Point3f Search point in the 3D space
         * @param[in] float range The maximum distance to search for a valid voxel to find its 2D correspondance on the disparity map
         * @return cv::Point The 2D coordinate on the disparity map
         * @excepion throw a runtime_error exception in case of the 3D point having no correspondence in the 2D disparity map
         */
        cv::Point retraceXYfrom3D_Zindependent(const cv::Point3f &search_point, const float range);

        /**
         * @brief Gives the Euclidian distance of 2 points in the 3d space
         * @param[in] cv::Point3f pt1 origin point to measure the distance
         * @param[in] cv::Point3f pt2 destination point to measure the distance
         * @return double The distance between pt1 and pt2
         */
        static double pointDistance(const cv::Point3f &pt1, const cv::Point3f &pt2);

    private:
        /**
         * @brief projected_points is the original 3d space while computed_points are the 3d space projection after segmentation and rotation operations
         */
        cv::Mat projected_points, computed_points;
        /**
         * @brief disparity-to-depth matrix used to project the disparity map into 3D
         */
        cv::Mat Qmatrix;
        cv::Mat orig_disp_img, disp_img;
        int failed_pixel;
        void convertToPointCloud(const cv::Mat &disparity_map, const cv::Mat &Q);

        /**
         * @brief Shifts one point in 3D to a new origin
         */
        cv::Point3f shitftOnePoint(const cv::Point3f &target, const cv::Point3f &new_origin, const cv::Point3f &new_sense);
};

}

#endif
