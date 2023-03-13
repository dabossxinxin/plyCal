/**
  ******************************************************************************
  * @file	pointcloud_polygon.cpp
  * @author	Nick.Liao
  * @version V1.0.0
  * @date	2017-12-26
  * @brief	pointcloud_polygon.cpp
  ******************************************************************************
  * @attention
  *
  * Copyright (C) 2017 Nick.Liao <simplelife_nick@hotmail.com>
  * Distributed under terms of the MIT license.
  ******************************************************************************
  */

#include "pointcloud_polygon.h"

#include <cmath>
#include <map>
#include <functional>

#include <pcl/common/io.h>
#include <pcl/common/common.h>
#include <pcl/io/pcd_io.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/sample_consensus/sac_model_line.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>

using namespace lqh;

PointcloudPolygon::PointcloudPolygon(const nlohmann::json& js, uint8_t size):
	size_(size), polygon_(nullptr)
{
	pc_angle_resolution_ = js["angle_resolution"].get<double>();
	filter_angle_start_ = js["filter"]["angle_start"].get<double>();
	filter_angle_size_ 	= js["filter"]["angle_size"].get<double>();
	filter_distance_  	= js["filter"]["distance"].get<double>();
	filter_floor_gap_ 	= js["filter"]["floor_gap"].get<double>();
	use_angle_filter_   = js["filter"]["use_angle_filter"].get<uint32_t>();
	plane_point_num_min_ 		= js["plane"]["point_num_min"].get<uint32_t>();
	plane_distance_threshold_ 	= js["plane"]["distance_threshold"].get<double>();
	ring_point_num_min_ = js["ring"]["point_num_min"].get<uint32_t>();
	ring_endpoint_num_  = js["ring"]["endpoint_num"].get<uint32_t>();
	if(ring_endpoint_num_ > ring_point_num_min_)
	{
		std::cout << "W: ring:endpoint_num_min must <= ring:point_num_min\n"
				  << "   set ring:endpoint_num = ring:point_num_min\n";
		ring_endpoint_num_ = ring_point_num_min_;
	}
	edge_point_num_min_ = js["edge"]["point_num_min"].get<uint32_t>();
	edge_distance_threshold_ = js["edge"]["distance_threshold"].get<double>();
	
}

PointcloudPolygon::Polygon3D::ConstPtr PointcloudPolygon::Add(
	const pcl::PointCloud<pcl::PointXYZI>& pc,
	pcl::PointCloud<pcl::PointXYZRGB>& pcc)
{
	Polygon3D::Ptr ply = std::make_shared<Polygon3D>(size_);
	std::vector<int> indices;
	if(pcc.size() != pc.size())
	{
		pcl::copyPointCloud(pc, pcc);
	}
	for(auto& p : pcc.points)
	{
		p.rgba = 0xffffffff;
	}

	lqh::utils::color::rgbs colors = lqh::utils::color::get_rgbs(7);

	ply->coef.setZero();

	// 1、sector filter
	SectorFilter(pc, indices);
	MarkPointcloud(pcc, colors[0], indices);
	if(indices.size() < plane_point_num_min_) {
		return nullptr;
	}
	SaveMarkedPointcloud("SectorFilter.pcd", pc, indices);

	// 2、提取标定板平面
	bool res = ExtractPlane(pc, indices, ply->coef);
	if(!res) {
		return nullptr;
	}
	SaveMarkedPointcloud("ExtractPlane.pcd", pc, indices);

	// 3、提取标定板的左右边缘
	Indices edge_left, edge_right, inliers;
	ExtractRings(pc, indices, inliers, edge_left, edge_right);

	MarkPointcloud(pcc, colors[2], inliers);
	MarkPointcloud(pcc, colors[3], edge_left);
	MarkPointcloud(pcc, colors[4], edge_right);

	SaveMarkedPointcloud("inlier.pcd", pc, inliers);
	SaveMarkedPointcloud("edge_left.pcd", pc, edge_left);
	SaveMarkedPointcloud("edge_right.pcd", pc, edge_right);

	ply->inliers.resize(3, inliers.size());
	for(uint32_t i=0; i<inliers.size(); ++i) {
		auto& p = pc.points[inliers[i]];
		ply->inliers.col(i) << p.x, p.y, p.z;
	}

	// 4、将标定板的边缘信息区分
	Indices edge0, edge1, edge2, edge3;
	ExtractEdgesByPrinciple(pc, edge_left, ply->edges[0], ply->edges[1], edge0, edge1);
	ExtractEdgesByPrinciple(pc, edge_right, ply->edges[3], ply->edges[2], edge3, edge2);

	ExtractVertex(ply->edges[3], ply->edges[0], ply->coef, ply->vertexs[0]);
	ExtractVertex(ply->edges[0], ply->edges[1], ply->coef, ply->vertexs[1]);
	ExtractVertex(ply->edges[1], ply->edges[2], ply->coef, ply->vertexs[2]);
	ExtractVertex(ply->edges[2], ply->edges[3], ply->coef, ply->vertexs[3]);

	MarkPointcloud(pcc, colors[5], edge0);
	MarkPointcloud(pcc, colors[6], edge1);
	MarkPointcloud(pcc, colors[5], edge2);
	MarkPointcloud(pcc, colors[6], edge3);

	polygon_ = ply;

	SaveMarkedPointcloud("edge0.pcd", ply->edges[0]);
	SaveMarkedPointcloud("edge1.pcd", ply->edges[1]);
	SaveMarkedPointcloud("edge2.pcd", ply->edges[2]);
	SaveMarkedPointcloud("edge3.pcd", ply->edges[3]);

	return ply;
}

void PointcloudPolygon::ExtractVertex(const Edge3D& edge0, const Edge3D& edge1,
	const Eigen::Vector4d& plane_coef, Eigen::Vector3d& vertex)
{
	Eigen::Vector3d plane_normal = plane_coef.head(3);
	Eigen::Vector3d edge_first_normal = plane_normal.cross(edge0.coef);
	Eigen::Vector3d edge_second_normal = plane_normal.cross(edge1.coef);

	Eigen::Matrix3d normal_matrix;
	normal_matrix.block(0, 0, 1, 3) = edge_first_normal.transpose();
	normal_matrix.block(1, 0, 1, 3) = edge_second_normal.transpose();
	normal_matrix.block(2, 0, 1, 3) = plane_normal.transpose();

	Eigen::Vector3d error_vector;
	error_vector(0) = edge_first_normal.dot(edge0.p0);
	error_vector(1) = edge_second_normal.dot(edge1.p0);
	error_vector(2) = -plane_coef(3);

	vertex = normal_matrix.inverse()*error_vector;
}

void PointcloudPolygon::SetFilterParameters(const Eigen::Vector4d& p)
{
    if(p(0) >=0 && p(0) <=360)
    {
        filter_angle_start_ = p(0);
    }
    if(p(1) >=0 && p(1) <= 360)
    {
        filter_angle_size_  = p(1);
    }
    if(p(2) > 0)
    {
        filter_distance_    = p(2);
    }
    if(p(3) > -10)
    {
        filter_floor_gap_   = p(3);
    }
}

void PointcloudPolygon::SectorFilter(const PCI& pc, Indices& indices)
{
	indices.clear();
	indices.reserve(plane_point_num_min_*10);

	Eigen::Vector4f min_pt, max_pt;
	pcl::getMinMax3D(pc, min_pt, max_pt);

	double z_threshold = min_pt(2)+ filter_floor_gap_;

	if (use_angle_filter_) {
		for (uint32_t i = 0; i < pc.points.size(); ++i) {
			auto& p = pc.points[i];
			double theta = std::atan2(p.y, p.x) / MATH_PI * 180;
			if (theta < 0) {
				theta += 360;
			}
			double dis = std::sqrt(p.y*p.y + p.x*p.x);
			double size = theta - filter_angle_start_;
			if (size < 0) {
				size += 360;
			}

			if (p.z > z_threshold && dis < filter_distance_
				&& size < filter_angle_size_) {
				indices.push_back(i);
			}

		}
	}
	else {
		for (uint32_t i = 0; i < pc.points.size(); ++i) {
			indices.push_back(i);
		}
	}
}

bool PointcloudPolygon::ExtractPlane(const PCI& pc, Indices& indices,
									 Eigen::Vector4d& coef)
{
	PCI::Ptr pc_ptr (new PCI);
	pcl::copyPointCloud(pc, *pc_ptr);

	pcl::IndicesPtr indices_ptr(new std::vector<int>);
	indices_ptr->resize(indices.size());
	std::copy(indices.begin(), indices.end(), indices_ptr->begin());

	pcl::ModelCoefficients model_coef;
	pcl::PointIndices ids;
	pcl::SACSegmentation<pcl::PointXYZI> seg;
	seg.setOptimizeCoefficients (false);
	seg.setModelType (pcl::SACMODEL_PLANE);
	seg.setMethodType (pcl::SAC_RANSAC);
	seg.setDistanceThreshold (plane_distance_threshold_);
	seg.setInputCloud(pc_ptr);
	seg.setIndices(indices_ptr);
	seg.segment(ids, model_coef);

	if(ids.indices.size() > plane_point_num_min_ )
	{
		coef << model_coef.values[0], model_coef.values[1],
			 model_coef.values[2], model_coef.values[3];
		std::swap(ids.indices, indices);
		return true;
	}
	else
	{
		std::cerr << "E: Fail to extract plane" << std::endl;
		return false;
	}
}

bool PointcloudPolygon::ExtractRings(const PCI& pc, const Indices& indices,
									 Indices& inlier, Indices& left, Indices& right)
{
	struct Point {
		int8_t pitch;
		uint32_t id;
		double yaw;

		Point(uint32_t pid, int8_t ppitch, double pyaw):
			pitch(ppitch),id(pid),yaw(pyaw) {}
	};

	if(indices.size() < plane_point_num_min_) {
		std::cerr << "E: no enough points to extract edge/inlier\n";
		return false;
	}

	// 将平面点云按照角度划分成多个区域条带
	std::map<int16_t, std::vector<Point>> pts;
	for(auto it:indices) {
		auto& p = pc.points[it];
		double yaw = std::atan2(p.y, p.x);
		double pitch_d = std::atan(p.z/std::sqrt(p.y*p.y+p.x*p.x))/MATH_PI*180;
		int16_t pitch = std::round(pitch_d/pc_angle_resolution_);

		pts[pitch].emplace_back(it, pitch, yaw);
	}

	left.reserve(pts.size()*ring_endpoint_num_);
	right.reserve(pts.size()*ring_endpoint_num_);
	inlier.reserve(indices.size() - pts.size()*ring_endpoint_num_ * 2);

	for (auto& r : pts) {
		auto& ring = r.second;

		if (ring.size() < ring_point_num_min_) {
			continue;
		}

		std::sort(ring.begin(), ring.end(), [](const Point& a, const Point& b) {
			return a.yaw < b.yaw;
		});

		// clean every ring with 3 sigma principle
		// sample the center 1/3
		if (false) {
			double mean = 0;
			double sigma = 0;
			uint32_t num = ring.size() - 1;
			uint32_t num_3 = num / 3;
			std::vector<double> yaw_err(num);
			for (uint32_t i = 0; i < num; i++)
			{
				double tmp = ring[i + 1].yaw - ring[i].yaw;
				yaw_err[i] = tmp;
				if (i >= num_3 && i < num_3 * 2)
				{
					mean += tmp;
					sigma += tmp * tmp;
				}
			}
			mean = mean / (num_3);
			sigma = std::sqrt((sigma - mean * mean) / num_3);
			// from center to both end 	
			// we don't check center point, we think it's valid
			uint32_t center = num / 2;
			for (uint32_t i = center + 1; i < num; i++)
			{
				if (std::abs(yaw_err[i] - mean) > 3 * sigma)
				{
					ring.erase(ring.begin() + i + 1, ring.end());
					break;
				}
			}
			for (int32_t i = center - 1; i >= 0; i--)
			{
				if (std::abs(yaw_err[i] - mean) > 3 * sigma)
				{
					ring.erase(ring.begin(), ring.begin() + i + 1);
					break;
				}
			}
		}

		// 保存每个条带的两边端点作为标定板左右边缘点
		for (uint32_t i = 0; i < ring_endpoint_num_; ++i) {
			uint32_t id_right = ring.size() - 1 - i;
			right.push_back(ring[i].id);
			left.push_back(ring[id_right].id);
		}

		// 条带中除端点外的点为内点
		if (ring.size() > 2 * ring_endpoint_num_) {
			for (uint32_t i = ring_endpoint_num_;
				i < ring.size() - ring_endpoint_num_; ++i) {
				inlier.push_back(ring[i].id);
			}
		}
	}
	return true;
}

bool PointcloudPolygon::ExtractEdges(const PCI& pc, Indices& indices,
									 Edge3D& edge0, Edge3D& edge1)
{
	Indices ids;
	Eigen::Vector3d coef, pt;

	FitLine(pc, indices, ids, coef, pt);
	std::sort(ids.begin(), ids.end(), [&pc](const int a, const int b)
	{
		return pc.points[a].z < pc.points[b].z;
	});

	auto AssembleEdge = [&pc](const Indices& indices, Edge3D& edge)
	{
		edge.points.resize(3, indices.size());
		for(uint32_t i=0; i<indices.size(); i++)
		{
			auto& p = pc.points[indices[i]];
			edge.points.col(i) << p.x, p.y, p.z;
		}
	};

	auto Yaw = [](const pcl::PointXYZI& p)->double
	{
		return std::atan2(p.y, p.x);
	};

	// only one line
	if(ids.size() == indices.size())
	{
		double yaw_b = Yaw(pc.points[ids.front()]);
		double yaw_t = Yaw(pc.points[ids.back()]);
		double yaw_bt = yaw_t-yaw_b;
		if(std::abs(yaw_bt) > MATH_PI)
		{
			yaw_bt = yaw_bt >MATH_PI ? 2*MATH_PI - yaw_bt : 2*MATH_PI+yaw_bt;
		}

		if(yaw_bt > 0)
		{
			edge0.coef = coef;			// the line is first line
			edge0.p0 = pt;
			AssembleEdge(ids, edge0);
		}
		else
		{
			edge1.coef = coef;
			edge1.coef = pt;
			AssembleEdge(ids, edge1);
		}
	}
	else
	{
		// two lines
		// ids_another = indices - ids
		std::vector<int> ids_another;
		std::set_difference(
			indices.begin(), indices.end(),
			ids.begin(), ids.end(),
			std::back_inserter( ids_another )
		);

		double err_dis = 0;
		for(uint32_t i=0; i<ring_endpoint_num_; i++)
		{
			auto& it = pc.points[indices[i]];
			Eigen::Vector3d pt_i(it.x, it.y, it.z);

			err_dis += coef.cross(pt_i - pt).norm();
		}
		err_dis = err_dis/ring_endpoint_num_;

		if(err_dis > edge_distance_threshold_)
		{
			// fitline is the second line
			edge1.coef = coef;
			AssembleEdge(ids, edge1);
			AssembleEdge(ids_another, edge0);
		}
		else
		{
			edge0.coef = coef;
			AssembleEdge(ids, edge0);
			AssembleEdge(ids_another, edge1);
		}
	}
	return true;
}

bool PointcloudPolygon::ExtractEdgesMems(const PCI& pc, Indices& indices,
		Edge3D& edge0, Edge3D& edge1, Indices& Iedge0, Indices& Iedge1, bool is_left)
{
	Indices first, second;
	first.reserve(indices.size()/2);
	second.reserve(indices.size()/2);

	// left  find y-max,  right find y-min
	double y_m = pc.points[indices[0]].y;
	uint32_t id_sep = 0;
	if(is_left)
	{
		uint32_t i = 0;
		for(auto id : indices)
		{
			if(pc.points[id].y < y_m)
			{
				y_m = pc.points[id].y;
				id_sep = i;
			}
			i++;
		}
        if (id_sep == 0)
        {
            id_sep = indices.size()-1;
        }
	}
	else
	{
		uint32_t i = 0;
		for(auto id : indices)
		{
			if(pc.points[id].y >= y_m)
			{
				y_m = pc.points[id].y;
				id_sep = i;
			}
			i++;
		}
        if(id_sep == indices.size()-1)
        {
            id_sep = 0;
        }
	}

	for(uint32_t i=0; i<=id_sep; i++)
	{
		first.push_back(indices[i]);
	}
	for(uint32_t i=id_sep; i<indices.size(); i++)
	{
		second.push_back(indices[i]);
	}


	auto makeEdge = [&pc, this](Indices& indice, Edge3D& edge, Indices& Iedge)
	{
		if(indice.size() > 3)
		{
			Indices inlier;
			FitLine(pc, indice, inlier, edge.coef, edge.p0);
			edge.points.resize(3, inlier.size());
			uint32_t i=0;
			for(auto it: inlier)
			{
				edge.points(0,i)= pc.points[it].x;
				edge.points(1,i)= pc.points[it].y;
				edge.points(2,i)= pc.points[it].z;

				Iedge.push_back(it);
				i++;
			}
		}
	};

	makeEdge(first, edge0, Iedge0);
	makeEdge(second, edge1, Iedge1);

	return true;
};

bool PointcloudPolygon::ExtractEdgesByTheta(const PCI& pc, const Indices& indices, const Eigen::Vector4d& coef,
										Edge3D& edge0, Edge3D& edge1, Indices& Iedge0, Indices& Iedge1)
{
	Indices first, second;
	const int number = indices.size();
	first.reserve(number);
	second.reserve(number);
	const int offset = number / 20;

	double theta_max = -FLT_MAX;
	int id_max = 0;

	const Eigen::Vector3d normal(coef(0), coef(1), coef(2));

	for (int it = offset; it < number - offset; it++)
	{
		double theta_mean = 0.;
		for (int j = 1; j <= offset; j++)
		{
			const auto last = pc.points[indices[it - j]];
			const auto curr = pc.points[indices[it]];
			const auto next = pc.points[indices[it + j]];

			Eigen::Vector3d v_last(last.x - curr.x, last.y - curr.y, last.z - curr.z);
			Eigen::Vector3d v_next(next.x - curr.x, next.y - curr.y, next.z - curr.z);

			v_last -= (normal / normal.norm()) * (v_last.dot(normal) / normal.norm());
			v_next -= (normal / normal.norm()) * (v_next.dot(normal) / normal.norm());

			double theta = acos(((v_last.dot(v_next)) / (v_last.norm() * v_next.norm()))) * 180.0 / M_PI;

			if (theta < 0.) theta += 180.0;
			if (theta > 180.) theta -= 180.;

			theta = 180 - theta;
			theta_mean += theta;
		}
		

		if (theta_mean >= theta_max)
		{
			id_max = it;
			theta_max = theta_mean;
		}
	}

	for (uint32_t i = 0; i <= id_max; i++)
	{
		first.push_back(indices[i]);
	}
	for (uint32_t i = id_max+1; i < number; i++)
	{
		second.push_back(indices[i]);
	}

	auto makeEdge = [&pc, this](Indices& indice, Edge3D& edge, Indices& Iedge)
	{
		if (indice.size() > 3)
		{
			Indices inlier;
			FitLine(pc, indice, inlier, edge.coef, edge.p0);
			edge.points.resize(3, inlier.size());
			uint32_t i = 0;
			for (auto it : inlier)
			{
				edge.points(0, i) = pc.points[it].x;
				edge.points(1, i) = pc.points[it].y;
				edge.points(2, i) = pc.points[it].z;

				Iedge.push_back(it);
				i++;
			}
		}
	};

	makeEdge(first, edge0, Iedge0);
	makeEdge(second, edge1, Iedge1);

	return true;
}

bool PointcloudPolygon::ExtractEdgesByPrinciple(const PCI& pc, const Indices& indices,
											Edge3D& edge0, Edge3D& edge1, Indices& Iedge0, Indices& Iedge1)
{
	Indices first, second;
	const int number = indices.size();
	first.reserve(number*0.5);
	second.reserve(number*0.5);

	int id_sep = 0;
	double threshold = -FLT_MAX;

	for (int i = 3; i < number - 3; ++i) {
		pcl::PointCloud<pcl::PointXYZI>::Ptr pc_first(new pcl::PointCloud<pcl::PointXYZI>);
		pcl::PointCloud<pcl::PointXYZI>::Ptr pc_second(new pcl::PointCloud<pcl::PointXYZI>);

		for (int it = 0; it < i; it++)	pc_first->points.push_back(pc.points[indices[it]]);
		for (int it = i; it < number; it++) pc_second->points.push_back(pc.points[indices[it]]);

		double first_value_first, second_value_first, first_value_second, second_value_second;
		double eigen_value_ratio_first = PrincipleComponentAnalysis(*pc_first, first_value_first, second_value_first);
		double eigen_value_ratio_second = PrincipleComponentAnalysis(*pc_second, first_value_second, second_value_second);

		double eigen_value_plus = eigen_value_ratio_first * eigen_value_ratio_second;
		if (eigen_value_plus >= threshold) {
			id_sep = i;
			threshold = eigen_value_plus;
		}
	}

	for (uint32_t i = 0; i <= id_sep; i++) {
		first.push_back(indices[i]);
	}
	for (uint32_t i = id_sep + 1; i < number; i++) {
		second.push_back(indices[i]);
	}

	auto makeEdge = [&pc, this](Indices& indice, Edge3D& edge, Indices& Iedge)
	{
		if (indice.size() > 3) {
			Indices inlier;
			FitLine(pc, indice, inlier, edge.coef, edge.p0);
			edge.points.resize(3, inlier.size());
			uint32_t i = 0;
			for (auto& it : inlier) {
				edge.points(0, i) = pc.points[it].x;
				edge.points(1, i) = pc.points[it].y;
				edge.points(2, i) = pc.points[it].z;

				Iedge.push_back(it);
				i++;
			}
		}
		else {
			std::cerr << "no enough pts to fit line." << std::endl;
		}
	};

	makeEdge(first, edge0, Iedge0);
	makeEdge(second, edge1, Iedge1);

	return true;
}

double PointcloudPolygon::PrincipleComponentAnalysis(const PCI& pc, double& first_value, double& second_value)
{
	pcl::PointXYZI centroid;
	centroid.x = 0.;
	centroid.y = 0.;
	centroid.z = 0.;

	const int number = pc.points.size();
	float number_inv = 1.0 / float(number);

	double x_tmp_center = 0.;
	double y_tmp_center = 0.;
	double z_tmp_center = 0.;

	for (int it = 0; it < number; ++it) {
		x_tmp_center += pc.points[it].x;
		y_tmp_center += pc.points[it].y;
		z_tmp_center += pc.points[it].z;
	}

	centroid.x = x_tmp_center * number_inv;
	centroid.y = y_tmp_center * number_inv;
	centroid.z = z_tmp_center * number_inv;

	Eigen::Matrix<double, 3, Eigen::Dynamic> covariance(3, number);
	for (int it = 0; it < number; ++it) {
		double x_tmp = pc.points[it].x - centroid.x;
		double y_tmp = pc.points[it].y - centroid.y;
		double z_tmp = pc.points[it].z - centroid.z;

		covariance(0, it) = x_tmp;
		covariance(1, it) = y_tmp;
		covariance(2, it) = z_tmp;
	}
	covariance = (covariance * covariance.transpose())*number_inv;

	Eigen::EigenSolver<Eigen::Matrix3d> es(covariance);
	Eigen::Matrix3d D = es.pseudoEigenvalueMatrix();

	for (int i = 0; i < 2; ++i) {
		for (int j = i + 1; j < 3; ++j) {
			if (D(i, i) < D(j, j)) {
				double tmp = D(i, i);
				D(i, i) = D(j, j);
				D(j, j) = tmp;
			}
		}
	}

	first_value = D(0, 0);
	second_value = D(1, 1);
	
	return first_value / second_value;
}

void PointcloudPolygon::FitLine(const PCI& pc, const Indices& indices,
								Indices& inlier, Eigen::Vector3d& coef, Eigen::Vector3d& pt)
{
	PCI::Ptr pc_ptr(new PCI);
	pc_ptr->points.reserve(indices.size());

	for (auto it : indices) {
		pc_ptr->points.push_back(pc.points[it]);
	}

	pcl::ModelCoefficients model_coef;
	pcl::PointIndices ids;

	pcl::SACSegmentation<pcl::PointXYZI> seg;
	seg.setOptimizeCoefficients (true);
	seg.setModelType (pcl::SACMODEL_LINE);
	seg.setMethodType (pcl::SAC_RANSAC);
	seg.setDistanceThreshold (edge_distance_threshold_);
	seg.setInputCloud(pc_ptr);
	seg.segment(ids, model_coef);

	pt(0) 	= model_coef.values[0];
	pt(1) 	= model_coef.values[1];
	pt(2) 	= model_coef.values[2];
	coef(0) = model_coef.values[3];
	coef(1) = model_coef.values[4];
	coef(2) = model_coef.values[5];
	coef.normalize();

	inlier.reserve(ids.indices.size());
	for (auto it : ids.indices) {
		inlier.push_back(indices[it]);
	}
}

bool PointcloudPolygon::SaveMarkedPointcloud(const std::string& fn, Edge3D& edge) const
{
	pcl::PointCloud<pcl::PointXYZRGB> pcc;
	const int number = edge.points.cols();
	pcc.points.resize(number);

	int i = 0;
	for (auto& it : pcc.points) {
		if (i >= number) break;

		it.rgba = 0xffff0000;
		it.x = edge.points(0, i);
		it.y = edge.points(1, i);
		it.z = edge.points(2, i);

		i++;
	}

	return pcl::io::savePCDFileBinary(fn, pcc);
}

bool PointcloudPolygon::SaveMarkedPointcloud(const std::string& fn, const PCI& pc,
		const Indices& indices) const
{
	pcl::PointCloud<pcl::PointXYZRGB> pcc;
	pcl::copyPointCloud(pc, pcc);

	for(auto& it:pcc.points) {
		it.rgba = 0xffffffff;
	}

	for(auto it: indices) {
		pcc.points[it].rgba = 0xffff0000;
	}

	return pcl::io::savePCDFileBinary(fn, pcc);
}

bool PointcloudPolygon::SaveMarkedPointcloud(const std::string& fn,
		const pcl::PointCloud<pcl::PointXYZI>& pc,
		Polygon3D::ConstPtr ply) const
{
	pcl::PointCloud<pcl::PointXYZRGB> pcc;
	pcc.points.reserve(ply->inliers.cols() + ply->edges[0].points.cols()*
					   ply->edges.size());

	lqh::utils::color::rgbs colors = lqh::utils::color::get_rgbs(size_+1);
	pcl::PointXYZRGB p;

	for (uint32_t i = 0; i < size_; ++i) {
		p.r = colors[i][0];
		p.g = colors[i][1];
		p.b = colors[i][2];

		auto& ps = ply->edges[i].points;
		for (uint32_t j = 0; j < ps.cols(); ++j) {
			p.x = ps(0, j);
			p.y = ps(1, j);
			p.z = ps(2, j);
			pcc.push_back(p);
		}
	}

	p.r = colors.back()[0];
	p.g = colors.back()[1];
	p.b = colors.back()[2];
	for(uint32_t i=0; i<ply->inliers.cols(); i++)
	{
		p.x = ply->inliers(0,i);
		p.y = ply->inliers(1,i);
		p.z = ply->inliers(2,i);
		pcc.push_back(p);
	}

	return pcl::io::savePCDFileBinary(fn, pcc);
}

bool PointcloudPolygon::SaveEdgePoints(const std::string& fn_prefix)
{
	Eigen::IOFormat fmt(Eigen::FullPrecision, 0, ", ", "\n", "", "", "", "");
	for (uint8_t i = 0; i < polygon_->edges.size(); ++i) {
		std::ofstream f(fn_prefix + std::to_string(i) + ".csv");

		if (!f.good()) {
			return false;
		}

		f << polygon_->edges[i].points.transpose().format(fmt);
		f.close();
	}

	return true;
}


void PointcloudPolygon::MarkPointcloud(pcl::PointCloud<pcl::PointXYZRGB>& pcc,
									   const lqh::utils::color::rgb& color,
									   const Indices& indices) const
{
	for(auto it : indices) {
		auto& p = pcc.points[it];
		p.r = color[0];
		p.g = color[1];
		p.b = color[2];
	}
}

pcl::PointCloud<pcl::PointXYZRGB> PointcloudPolygon::MarkPointcloudEdit(const pcl::PointCloud<pcl::PointXYZI>& pc,
																			const lqh::utils::color::rgb& color,
																			const Indices& indices) const
{
	pcl::PointCloud<pcl::PointXYZRGB> pcc;
	const int number = indices.size();
	pcc.points.resize(number);
	
	for (int it = 0; it < number; ++it) {
		pcc.points[it].x = pc.points[indices[it]].x;
		pcc.points[it].y = pc.points[indices[it]].y;
		pcc.points[it].z = pc.points[indices[it]].z;
		
		pcc.points[it].r = color[0];
		pcc.points[it].g = color[1];
		pcc.points[it].b = color[2];
	}

	return pcc;
}
