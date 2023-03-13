/**
  ******************************************************************************
  * @file	calibrator.h
  * @author	Nick.Liao
  * @version V1.0.0
  * @date	2017-12-28
  * @brief	calibrator.h
  ******************************************************************************
  * @attention
  *
  * Copyright (C) 2017 Nick.Liao <simplelife_nick@hotmail.com>
  * Distributed under terms of the MIT license.
  ******************************************************************************
  */

#ifndef __CALIBRATOR_H
#define __CALIBRATOR_H

#include "utils.h"
#include "stdint.h"
#include <string>
#include <memory>

#include <Eigen/Dense>
#include <Eigen/Core>

#include <opencv2/opencv.hpp>
#include <opencv2/core/eigen.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "image_polygon.h"
#include "pointcloud_polygon.h"

#include "json/json.hpp"

namespace lqh
{
class Calibrator
{
public:
	explicit Calibrator(const nlohmann::json& js);

    uint32_t Add(const cv::Mat& img, const pcl::PointCloud<pcl::PointXYZI>& pc,
                 cv::Mat& img_out, pcl::PointCloud<pcl::PointXYZRGB>& pcc);

    bool RefineImage(const cv::Mat& img, cv::Mat& img_out, const std::vector<cv::Point2d>& pts);
	bool RefinePointcloud(const pcl::PointCloud<pcl::PointXYZI>& pc,
		pcl::PointCloud<pcl::PointXYZRGB>& pcc,
		const Eigen::Vector4d& param);

    uint32_t SavePolygonData(const std::string& dir);

    void SetCameraK(const Eigen::Matrix3d& k) {
        K_ = k;
    }

    void SetTranformation(const Eigen::Matrix4d& tf);
	const Eigen::Matrix4d& GetTransformation() {
		return T_;
	}

    bool Remove(uint32_t id);
    bool Remove();

    bool Compute(Eigen::Matrix4d& tf);
    bool Compute() {
        return Compute(T_);
    }

	bool Initialize(Eigen::Matrix4d& tf);
	bool Initialize() {
		return Initialize(T_);
	}

	int GetIterations() {
		return iterations_running_;
	}

	int GetMaxIterations() {
		return max_iterations_;
	}

	double GetReprojectError() {
		return reproject_error_;
	}

    // 判断当前id图像数据不为控
    bool ImageGood(uint32_t id)
    {
        if(id >= polygons_v_.size())
        {
            return false;
        }
        return (polygons_v_[id]->img != nullptr);
    }

    // 判断当前id点云数据不为空
    bool PointcloudGood(uint32_t id)
    {
        if(id >= polygons_v_.size())
        {
            return false;
        }
        return (polygons_v_[id]->pc != nullptr);
    }

	bool Good()
	{
		return is_valid_;
	}

    bool Project(pcl::PointCloud<pcl::PointXYZRGB>& pc, cv::Mat& img)
    {
        Project(pc, img, T_);
        return true;
    }
    bool Project(pcl::PointCloud<pcl::PointXYZRGB>& pc, cv::Mat& img,
                 const Eigen::Matrix4d& tf);
    
    using PC = pcl::PointCloud<pcl::PointXYZ>;
    PC Mat2PC(const Eigen::Matrix3Xd& mat) const
    {
        PC pc;
        pc.points.reserve(mat.cols());
        for (uint32_t i = 0; i < mat.cols(); i++)
        {
            pc.points.emplace_back(mat(0, i), mat(1, i), mat(2, i));
        }
        return pc;
    };

    double Distance(const pcl::PointXYZ& p1, const pcl::PointXYZ& p2) const 
    {
        double dx = p1.x - p2.x;
        double dy = p1.y - p2.y;
        double dz = p1.z - p2.z;
        return std::sqrt(dx * dx + dy * dy + dz * dz);
    }

private:
	struct Polygon
	{
		/**
		 * ids: corroesponences of 2d/3d Polygon
		 * same edge: 3d edge  <---> ids[i] 2d edge
         */
		std::vector<uint32_t> ids;  // id start
		ImagePolygon::Polygon2D::ConstPtr img;
		PointcloudPolygon::Polygon3D::ConstPtr pc;

        Polygon(uint32_t size):img(nullptr), pc(nullptr)
		{
			ids.resize(size);
			for(uint32_t i=0; i<size; i++) {
				ids[i] = i;
			}
		}
	};

	double			reproject_error_;		// 重投影误差；
	double			track_error_threshold_;
	bool			is_valid_;				// 输入点云和图像数据是否有效
	uint32_t		size_;					// 输入多边形模型的边的数量
	uint32_t		img_width_;				// 输入图像数据的宽度
    uint32_t		img_height_;			// 输入图像数据的高度
	uint32_t		iterations_running_;	// 非线性优化迭代次数
	uint32_t		max_iterations_;		// 非线性优化最大迭代次数
	Eigen::Matrix3d K_;						// 输入相机内参
	Eigen::Matrix4d T_;						// 输出传感器外参

	std::unique_ptr<ImagePolygon>		imgply_;		// 输入图像数据
	std::unique_ptr<PointcloudPolygon>	pcply_;			// 输入点云数据
    std::vector<Polygon*>				polygons_v_;	// 输入匹配后的图像-点云数据				
	std::list<Polygon>					polygons_;		// 输入匹配后的图像-点云数据

	//void TrackLines(const Polygon& ply_prev, Polygon& ply) const;
	void MatchLines(Polygon& ply) const;
    void Optimize(Eigen::Matrix4d& tf);
	double ComputeReprojError(const std::vector<Polygon*>& poly, Eigen::Matrix4d& tf);
};
}

#endif /* !__CALIBRATOR_H */
