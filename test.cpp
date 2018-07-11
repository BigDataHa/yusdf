/*************************************************************************
	> File Name: test.cpp
	> Author: yusnows
	> Mail: YusnowsLiang@gmail.com
	> Created Time: Mon 04 Jun 2018 09:02:01 PM CST
    > Copyright@yusnows: All rights reserve!
 ************************************************************************/

#include<iostream>
#include "src/tsdf/sdf.h"
#include <opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <string>
#include "utils.hpp"
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>
#include <sys/time.h>

int main(int argc, char const *argv[])
{

	struct timeval t1,t2;

	std::string data_path = "../data/rgbd-frames";
	int base_frame_idx = 150;
	int first_frame_idx = 150;
	float num_frames = 50;
	// Read base frame camera pose
	float base2world[4 * 4];
	float cam2world[4 * 4];
	float cam2base[4 * 4];

	// Voxel grid parameters (change these to change voxel grid resolution, etc.)
	float voxel_grid_origin_x = -1.5f; // Location of voxel grid origin in base frame camera coordinates
	float voxel_grid_origin_y = -1.5f;
	float voxel_grid_origin_z = 0.5f;
	float voxel_size = 0.008f;
	float trunc_margin = voxel_size * 4;
	int voxel_grid_dim_x = 500;
	int voxel_grid_dim_y = 500;
	int voxel_grid_dim_z = 500;

	sdf::sdf_t tsdf(voxel_grid_dim_x,voxel_grid_dim_y,voxel_grid_dim_z,trunc_margin,trunc_margin);
	tsdf.setCameraIntrins(5.85000000e+02,5.85000000e+02,3.20000000e+02,2.40000000e+02);
	tsdf.setScale(voxel_size,voxel_size,voxel_size);
	tsdf.setOffset(voxel_grid_origin_x,voxel_grid_origin_y,voxel_grid_origin_z);
	tsdf.setIndexOffset(0.5,0.5,0.5);
	// tsdf.setuvStep(2,2);
	std::ostringstream base_frame_prefix;
	base_frame_prefix << std::setw(6) << std::setfill('0') << base_frame_idx;
	std::string base2world_file = data_path + "/frame-" + base_frame_prefix.str() + ".pose.txt";
	std::vector<float> base2world_vec = LoadMatrixFromFile(base2world_file, 4, 4);
	std::copy(base2world_vec.begin(), base2world_vec.end(), base2world);
	// Invert base frame camera pose to get world-to-base frame transform
	float base2world_inv[16] = {0};
	invert_matrix(base2world, base2world_inv);
	for (int frame_idx = first_frame_idx; frame_idx < first_frame_idx + (int)num_frames; ++frame_idx)
	{
		
		std::ostringstream curr_frame_prefix;
		curr_frame_prefix << std::setw(6) << std::setfill('0') << frame_idx;
		// // Read current frame depth
		std::string depth_im_file = data_path + "/frame-" + curr_frame_prefix.str() + ".depth.png";
		cv::Mat depth = cv::imread(depth_im_file,cv::IMREAD_UNCHANGED);
		if (depth.empty()) {
			std::cout << "Error: depth image file not read!" << std::endl;
			cv::waitKey(0);
		}
		// Read base frame camera pose
		std::string cam2world_file = data_path + "/frame-" + curr_frame_prefix.str() + ".pose.txt";
		std::vector<float> cam2world_vec = LoadMatrixFromFile(cam2world_file, 4, 4);
		std::copy(cam2world_vec.begin(), cam2world_vec.end(), cam2world);
		// Compute relative camera pose (camera-to-base frame)
		multiply_matrix(base2world_inv, cam2world, cam2base);
		// double pose[4*4];
		Eigen::Affine3d pose;
		// for(int i=0;i<16;++i)
		// 	pose[i] = cam2base[i];
		for(int i=0;i<4;i++)
			for(int j=0;j<4;j++)
				pose(i,j) = cam2base[i*4+j];
		gettimeofday(&t1,NULL);
		if(frame_idx == first_frame_idx)
			tsdf.IntegrateUV(depth,pose);
		else
			tsdf.IntegrateUV(depth,pose);
		gettimeofday(&t2,NULL);
		double t_gap =(double) ((double)(t2.tv_sec - t1.tv_sec) + (double)(t2.tv_usec - t1.tv_usec)/1000000.0f);
		std::cout << "frame: " << frame_idx<<"; time used: "<<t_gap<<"s " <<std::endl;
	}
	// Compute surface points from TSDF voxel grid and save to point cloud .ply file
	std::cout << "Saving surface point cloud (tsdf.ply)..." << std::endl;
	SaveVoxelGrid2SurfacePointCloud("tsdf.ply", voxel_grid_dim_x, voxel_grid_dim_y, voxel_grid_dim_z, 
									voxel_size, voxel_grid_origin_x, voxel_grid_origin_y, voxel_grid_origin_z,
									tsdf.getVerticeTsdf(), tsdf.getVerticeWeight(), 0.2f, 0.0f);

	// Save TSDF voxel grid and its parameters to disk as binary file (float array)
	std::cout << "Saving TSDF voxel grid values to disk (tsdf.bin)..." << std::endl;
	std::string voxel_grid_saveto_path = "tsdf.bin";
	std::ofstream outFile(voxel_grid_saveto_path, std::ios::binary | std::ios::out);
	float voxel_grid_dim_xf = (float) voxel_grid_dim_x;
	float voxel_grid_dim_yf = (float) voxel_grid_dim_y;
	float voxel_grid_dim_zf = (float) voxel_grid_dim_z;
	outFile.write((char*)&voxel_grid_dim_xf, sizeof(float));
	outFile.write((char*)&voxel_grid_dim_yf, sizeof(float));
	outFile.write((char*)&voxel_grid_dim_zf, sizeof(float));
	outFile.write((char*)&voxel_grid_origin_x, sizeof(float));
	outFile.write((char*)&voxel_grid_origin_y, sizeof(float));
	outFile.write((char*)&voxel_grid_origin_z, sizeof(float));
	outFile.write((char*)&voxel_size, sizeof(float));
	outFile.write((char*)&trunc_margin, sizeof(float));
	for (int i = 0; i < voxel_grid_dim_x * voxel_grid_dim_y * voxel_grid_dim_z; ++i)
	{
		float tmp = (float)tsdf.getVerticeTsdf()[i];
		outFile.write((char*)&tmp, sizeof(float));
	}
	outFile.close();
	return 0;
}



