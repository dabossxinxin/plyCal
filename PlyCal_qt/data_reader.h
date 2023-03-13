/**
  ******************************************************************************
  * @file	data_reader.h
  * @author	Nick.Liao
  * @version V1.0.0
  * @date	2017-12-20
  * @brief	data_reader.h
  ******************************************************************************
  * @attention
  *
  * Copyright (C) 2017 Nick.Liao <simplelife_nick@hotmail.com>
  * Distributed under terms of the MIT license.
  ******************************************************************************
  */

#ifndef __DATA_READER_H
#define __DATA_READER_H

#include <stdint.h>

#include <QString>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <opencv2/opencv.hpp>

class DataReader
{
public:
    DataReader(const QString& path);
    bool isValid() {
        return is_valid_;
    }

	// 加载点云数据
    pcl::PointCloud<pcl::PointXYZI>::Ptr getPointcloud();
	// 加载图像数据
    std::shared_ptr<cv::Mat> getImage();

    bool moveNext();
    uint32_t getDatasetSize() {
        return size_;
    }
    uint32_t getCurrentId() {
        return index_;
    }

	// 设置相机内参
    void setCameraK(const Eigen::Matrix3d& k);
	// 设置相机畸变参数
    void setCameraD(const Eigen::Matrix<double,5,1>& d);

private:
    static const QString kPointcloudFolderName;
    static const QString kImageFolderName;
    static const QString kLaserscanFolderName;
    static const QString kTransformFileNmae;

    QString dataset_root_;
    QString image_extension_;	
    bool is_valid_;
    uint8_t flag_kd_;
	uint32_t index_;
	uint32_t size_;
    cv::Mat K_;
    cv::Mat D_;

    uint32_t getDataNum(const QString& path);
};

#endif /* !__DATA_READER_H */


