/**
  ******************************************************************************
  * @file	pointcloud_polygon.h
  * @author	Nick.Liao
  * @version V1.0.0
  * @date	2017-12-26
  * @brief	pointcloud_polygon.h
  ******************************************************************************
  * @attention
  *
  * Copyright (C) 2017 Nick.Liao <simplelife_nick@hotmail.com>
  * Distributed under terms of the MIT license.
  ******************************************************************************
  */

#ifndef __POINTCLOUD_POLYGON_H
#define __POINTCLOUD_POLYGON_H

#include "utils.h"
#include <string>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <Eigen/Dense>
#include "json/json.hpp"

namespace lqh
{
	class PointcloudPolygon
	{
	public:

		struct Edge3D {
			Eigen::Vector3d		p0;			// 直线上一点
			Eigen::Vector3d		coef;		// 直线方向向量
			Eigen::Matrix3Xd	points;		// 直线上的所有点

			Edge3D() :p0(0, 0, 0), coef(0, 0, 0) {}
		};

		struct Polygon3D {
			using ConstPtr = std::shared_ptr<const Polygon3D>;
			using Ptr = std::shared_ptr<Polygon3D>;

			Eigen::Vector4d 				coef;		// ply平面法向量(a,b,c,d)
			Eigen::Matrix3Xd 				inliers;	// ply平面上除端点外的点
			std::vector<Edge3D>				edges;		// ply平面的直线边界信息
			std::vector<Eigen::Vector3d>	vertexs;	// ply平面角点信息

			Polygon3D(uint32_t size) {
				vertexs.resize(size, Eigen::Vector3d::Zero());
				edges.resize(size);
			}

			EIGEN_MAKE_ALIGNED_OPERATOR_NEW
		};

		explicit PointcloudPolygon(const nlohmann::json& js, uint8_t size);

		Polygon3D::ConstPtr Add(const pcl::PointCloud<pcl::PointXYZI>& pc,
			pcl::PointCloud<pcl::PointXYZRGB>& pcc);

		void SetFilterParameters(const Eigen::Vector4d& p);

		Eigen::Vector4d GetFilterParameters() {
			return Eigen::Vector4d(filter_angle_start_, filter_angle_size_,
				filter_distance_, filter_floor_gap_);
		}

		bool SaveMarkedPointcloud(const std::string& fn,
			const pcl::PointCloud<pcl::PointXYZI>& pc,
			Polygon3D::ConstPtr ply) const;

		bool SaveEdgePoints(const std::string& fn_prefix);
	private:
		using PCI = pcl::PointCloud<pcl::PointXYZI>;
		using Indices = std::vector<int>;

		const uint8_t 	size_;
		double 		pc_angle_resolution_;
		double 		filter_angle_start_;
		double 		filter_angle_size_;
		double 		filter_distance_;
		double 		filter_floor_gap_;
		uint32_t 	plane_point_num_min_;
		double 		plane_distance_threshold_;
		uint32_t 	ring_point_num_min_;
		uint32_t 	ring_endpoint_num_;
		uint32_t 	edge_point_num_min_;
		double 		edge_distance_threshold_;
		int			use_angle_filter_;

		Polygon3D::Ptr polygon_;

		void SectorFilter(const PCI& pc, Indices& indices);

		bool ExtractPlane(const PCI& pc, Indices& indices, Eigen::Vector4d& coef);

		bool ExtractRings(const PCI& pc, const Indices& indices,
			Indices& inlier, Indices& left, Indices& right);

		bool ExtractEdges(const PCI& pc, Indices& indices,
			Edge3D& edge0, Edge3D& edge1);

		void ExtractVertex(const Edge3D& edge0, const Edge3D& edge1,
			const Eigen::Vector4d& plane_coef, Eigen::Vector3d& vertex);

		void FitLine(const PCI& pc, const Indices& indices,
			Indices& inlier, Eigen::Vector3d& coef, Eigen::Vector3d& pt);

		void MarkPointcloud(pcl::PointCloud<pcl::PointXYZRGB>& pcc,
			const lqh::utils::color::rgb& color,
			const Indices& indices) const;

		pcl::PointCloud<pcl::PointXYZRGB> MarkPointcloudEdit(const pcl::PointCloud<pcl::PointXYZI>& pc,
			const lqh::utils::color::rgb& color,
			const Indices& indices) const;

		bool SaveMarkedPointcloud(const std::string& fn, const PCI& pc,
			const Indices& indices) const;

		bool SaveMarkedPointcloud(const std::string& fn, Edge3D& edge) const;

		bool ExtractEdgesMems(const PCI& pc, Indices& indices,
			Edge3D& edge0, Edge3D& edge1, Indices& Iedge0, Indices& Iedge1, bool is_left);

		bool ExtractEdgesByTheta(const PCI& pc, const Indices& indices, const Eigen::Vector4d& coef,
			Edge3D& edge0, Edge3D& edge1, Indices& Iedge0, Indices& Iedge1);

		bool ExtractEdgesByPrinciple(const PCI& pc, const Indices& indices,
			Edge3D& edge0, Edge3D& edge1, Indices& Iedge0, Indices& Iedge1);

		double PrincipleComponentAnalysis(const PCI& pc, double& first_value, double& second_value);
	};

}
#endif /* !__POINTCLOUD_POLYGON_H */
