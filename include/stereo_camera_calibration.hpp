/**
* @file stereo_camera_calibration.hpp
* @brief for stereo camera camera calibration.
*/
#ifndef STEREO_CAMERA_CALIBRATION_HPP_
#define STEREO_CAMERA_CALIBRATION_HPP_
#include <stdint.h>
#include <string>
#include <vector>
#include <opencv2/core/core.hpp>

/**
* @brief namespace for stereo camera calibrator
*/
namespace stereo_tools {

/**
* @class StereoCameraCalibration
* @brief Class for Stereo Camera Calibration
*/
class StereoCameraCalibration {
 public:

  /**
  * @brief constructor
  * @param[in] config_file_path : path to the configuration YAML file
  */
  explicit StereoCameraCalibration(const std::string& config_file_path, const bool triple_set_calib=false);
  /**
  * @brief destructor
  */
  ~StereoCameraCalibration();

  /**
  * @brief read config YAML file
  * @param[in] config_file_path : path to the config YAML file
  * @return none
  * @exception raise exception if config YAML file could not be opened.
  */
  void openConfigFile(const std::string& config_file_path);

  /**
  * @brief execute stereo calibrartion starting from reading stereo image pairs.
  * @param[in] with_view Activates the visual output for checking the chessboard accuracy
  * @return none
  * @exception raise exception if failed.
  */
  void calibrate(const bool with_view = false);
  /**
   * @brief Same es calibrate, but expect a circlegrid as input instead of a chesssboard
   * @param[in] with_view Activates the visual output for checking the chessboard accuracy
   * @exception raise exception if failed.
   */
  void calibrate_circlegrid(const bool with_view = false);

  /**
  * @brief output calibrated result as YAML file.
  * @return none
  * @exception raise exception if YAML could not be saved.
  */
  void writeCalibrationFiles();

  /**
   * @brief Calculate the needed matrixes for remapping the stereo images into 
   * its rectified forms
   */
  void calculateRectificationMatrixes();

  /**
   * @brief this method was develop mainly to test the rectification results.
   * it shows the rectified images side-by-side
   * @note for obvious reasons, this method needs to be excecuted after calibration
   */
  void rectifyImages();


 private:
  /**
  * @brief copy constructor
  */
  StereoCameraCalibration(const StereoCameraCalibration&);
  /**
  * @brief assignment operator
  */
  StereoCameraCalibration& operator=(const StereoCameraCalibration&);

  /**
  * @brief read stereo image pairs
  * @return none 
  * @exception raise exception if failed to open image pairs.
  */
  void readImageFiles();
  /**
   * @brief Used for the 2-step calibration. reads the left images, then the right then the stereo pair
   * @exception raise exception if failed to open image pairs.
   */
  void readTripleImageFiles();

  /**
  * @brief detect corner points in the calibration chart.
  * @param[in] with_view Activates the visual output for checking the chessboard accuracy
  * @return none
  * @exception raise execption if no corner point was detected.
  */
  void findChartCornerPoint(const bool with_view=false);
  /**
   * @brief same as findChartCornerPoint but used on the 2-step calibration. Will find the points for the left camera followed the ones for the right.
   * @param[in] with_view Activates the visual output for checking the chessboard accuracy
   * @exception raise execption if no corner point was detected.
   */
  void findChartCornerPoint_split(const bool with_view=false);

  /**
  * @brief detect corner points in the calibration chart.
  * @param[in] with_view Activates the visual output for checking the chessboard accuracy
  * @return none
  * @exception raise execption if no corner point was detected.
  */
  void findChartCirclePoint(const bool with_view=false);
  /**
   * @brief same as findChartCirclePoint but used on the 2-step calibration. Will find the points for the left camera followed the ones for the right.
   * @param[in] with_view Activates the visual output for checking the chessboard accuracy
   * @exception raise execption if no corner point was detected.
   */
  void findChartCirclePoint_split(const bool with_view=false);
  /**
  * @brief execute stereo calibrartion assuming images are already read.
  * @return none
  * @exception ステレオカメラキャリブレーションに失敗した場合、例外を返す
  */
  void calibStereoCamera();

  /** @brief control if the calibration is going to be done with 3 sets of image or 2 sets */
  bool triple_set;
  /** @brief directory where input stereo image pairs exist */
  std::string image_file_directory_path_;
  /** @brief number of stereo image pairs to be used in the calibration. */
  int image_pair_num_, image_left_num_, image_right_num_;
  /** @brief image format for stereo image input */
  std::string image_format_;
  /** @brief number of the column in the calibration chart キャリブレーションチャートの格子点の数（横）[個] */
  int chart_corner_rows_num_;
  /** @brief number of the rows in the calibration chart キャリブレーションチャートの格子点の数（縦）[個] */
  int chart_corner_cols_num_;
  /** @brief キャリブレーションチャートの1個の格子点の大きさ[mm] */
  float chart_square_size_;
  /** @brief 左画像 */
  std::vector<cv::Mat> left_images_;
  /** @brief 右画像 */
  std::vector<cv::Mat> right_images_;
  /** @brief ステレオカメラ画像 */
  std::vector<cv::Mat> stereo_left_images_;
  std::vector<cv::Mat> stereo_right_images_;
  #ifdef WITH_FISHEYE
  /** @brief 3次元コーナー点 */
  std::vector<std::vector<cv::Point3d> > corner_points_3d_;
  /** @brief 左画像の2次元コーナー点 */
  std::vector<std::vector<cv::Point2d> > stereo_left_corner_points_;
  std::vector<std::vector<cv::Point2d> > left_corner_points_;
  /** @brief 右画像の2次元コーナー点 */
  std::vector<std::vector<cv::Point2d> > stereo_right_corner_points_;
  std::vector<std::vector<cv::Point2d> > right_corner_points_;
  #else
  /** @brief 3次元コーナー点 */
  std::vector<std::vector<cv::Point3f> > corner_points_3d_;
  /** @brief 左画像の2次元コーナー点 */
  std::vector<std::vector<cv::Point2f> > stereo_left_corner_points_;
  std::vector<std::vector<cv::Point2f> > left_corner_points_;
  /** @brief 右画像の2次元コーナー点 */
  std::vector<std::vector<cv::Point2f> > stereo_right_corner_points_;
  std::vector<std::vector<cv::Point2f> > right_corner_points_;
  #endif
  /** @brief intrinsic parameter for left camera */
  cv::Mat left_intrinsic_;
  /** @brief intrinsic parameter for right camera */
  cv::Mat right_intrinsic_;
  /** @brief distortion parameter for left camera */
  cv::Mat left_distortion_;
  /** @brief distortion parameter for right camera */
  cv::Mat right_distortion_;
  /** @brief ステレオカメラの間の外部パラメータ(rotation) */
  cv::Mat extrinsic_rotation_;
  /** @brief ステレオカメラの間の外部パラメータ(position) */
  cv::Mat extrinsic_position_;
  /** @brief Rectification rotation matrix (left) */
  cv::Mat rect_left_rotate_;
  /** @brief Rectification rotation matrix (right) */
  cv::Mat rect_right_rotate_;
  /** @brief Rectification position matrix (left) */
  cv::Mat rect_left_position_;
  /** @brief Rectification position matrix (left) */
  cv::Mat rect_right_position_;
  /** @brief Rectification disparity-to-depth matrix */
  cv::Mat disp_to_depth_;
};

}  // end of namespace stereo_tools

#endif  // STEREO_CAMERA_CALIBRATION_HPP_
