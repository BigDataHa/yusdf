/*************************************************************************
	> File Name: sdf.cpp
	> Author: yusnows
	> Mail: YusnowsLiang@gmail.com
	> Created Time: Fri 01 Jun 2018 11:33:25 PM CST
    > Copyright@yusnows: All rights reserve!
 ************************************************************************/

#include<iostream>
#include "sdf.h"
#include <opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>

using namespace std;

namespace sdf
{
	int volumme_t::setOffset(const point3d_t &offset)
	{
		return setOffset(offset._x,offset._y,offset._z);
	}
	int volumme_t::setOffset(const double &x,const double &y,const double &z)
	{
		_offset._x=x;
		_offset._y=y;
		_offset._z=z;
		return 0;
	}
	int volumme_t::setIndexOffset(const point3d_t &offset)
	{
		return setIndexOffset(offset._x,offset._y,offset._z);
	}
	int volumme_t::setIndexOffset(const double &x,const double &y,const double &z)
	{
		_index_offset._x=x;
		_index_offset._y=y;
		_index_offset._z=z;
		return 0;
	}
	int volumme_t::setScale(const scale3d_t &scale)
	{
		return setScale(scale._xscale,scale._yscale,scale._zscale);
	}
	int volumme_t::setScale(const double &xscale,const double &yscale,const double &zscale)
	{
		_scale._zscale=zscale;
		_scale._yscale=yscale;
		_scale._xscale=xscale;
		return 0;
	}

	int volumme_t::setCameraIntrins(const cameraIntrin_t &intrins)
	{
		return setCameraIntrins(intrins._fx,intrins._fy,intrins._cx,intrins._cy);
	}
	int volumme_t::setCameraIntrins(const double &fx,const double &fy,const double &cx,const double &cy)
	{
		_intrins._fx=fx;
		_intrins._fy=fy;
		_intrins._cx=cx;
		_intrins._cy=cy;
		return 0;
	}

	const point3d_t &volumme_t::getOffset() const
	{
		return _offset;
	}

	const point3d_t &volumme_t::getIndexOffset() const
	{
		return _index_offset;
	}

	const scale3d_t &volumme_t::getScale() const
	{
		return _scale;
	}

	const cameraIntrin_t &volumme_t::getCameraIntrins()const
	{
		return _intrins;
	}

	double &volumme_t::operator[](const index3d_t &index)
	{
		if(_vertice_tsdf==NULL)
		{
			double *ret = new double(0);
			return *ret;
		}
		int nindex = index3d_2_1d(index);
		if(nindex>=_size)
			return _vertice_tsdf[0];
		return _vertice_tsdf[nindex];
	}
	double &volumme_t::operator[](const point3d_t &point)
	{
		return (this->operator[](point3d2index3d(point)));
	}

	inline int volumme_t::index3d_2_1d(const index3d_t &index)
	{
		int ret=0;
		ret = index._z*_xsize*_ysize + index._y*_xsize +index._x;
		return ret;
	}

	inline void volumme_t::index3d_2_1d(int &ret,const index3d_t &index)
	{
		ret = index._z*_xsize*_ysize + index._y*_xsize +index._x;
	}
	inline index3d_t volumme_t::point3d2index3d(const point3d_t &point)
	{
		point3d_t tmp1 = point;
		point3d_t tmp2 = (tmp1 - _offset)/_scale - _index_offset;
		index3d_t ret;
		ret._x = (int)tmp2._x>0?(int)tmp2._x:0;
		ret._y = (int)tmp2._y>0?(int)tmp2._y:0;
		ret._z = (int)tmp2._z>0?(int)tmp2._z:0;
		return ret;
	}
	inline void volumme_t::point3d2index3d(index3d_t &ret,const point3d_t &point)
	{
		point3d_t tmp1 = point;
		point3d_t tmp2 = (tmp1 - _offset)/_scale - _index_offset;
		// ret._x = (int)tmp2._x;
		// ret._y = (int)tmp2._y;
		// ret._z = (int)tmp2._z;
		ret._x = (int)tmp2._x>0?(int)tmp2._x:0;
		ret._y = (int)tmp2._y>0?(int)tmp2._y:0;
		ret._z = (int)tmp2._z>0?(int)tmp2._z:0;
	}
	inline void volumme_t::index3d2point3d(point3d_t &ret,const index3d_t &index)
	{
		index3d_t tmp = index;
		point3d_t tmp2(tmp._x,tmp._y,tmp._z);
		ret = (_index_offset+tmp2)*_scale + _offset;
	}

	inline void volumme_t::Project2CurCamera(point3d_t &ret,const Eigen::Affine3d &pose, const point3d_t &point)
	{
		// Eigen::Vector3d pointv(point._x,point._y,point._z);
		// Eigen::Vector3d pointv1 = pose.inverse() * pointv;
		double tmp[3]={0};
		tmp[0] = point._x - pose(0,3);
		tmp[1] = point._y - pose(1,3);
		tmp[2] = point._z - pose(2,3);
		ret._x = pose(0,0)*tmp[0] + pose(1,0)*tmp[1] + pose(2,0)*tmp[2];
		ret._y = pose(0,1)*tmp[0] + pose(1,1)*tmp[1] + pose(2,1)*tmp[2];
		ret._z = pose(0,2)*tmp[0] + pose(1,2)*tmp[1] + pose(2,2)*tmp[2];
	}

	inline void volumme_t::Project2CurCamera(point3d_t &ret,const double *pose, const point3d_t &point)
	{
		double tmp[3]={0};
		tmp[0] = point._x - pose[0*4+3];
		tmp[1] = point._y - pose[1*4+3];
		tmp[2] = point._z - pose[2*4+3];

		ret._x = pose[0*4+0]*tmp[0] + pose[1*4+0]*tmp[1] + pose[2*4+0]*tmp[2];
		ret._y = pose[0*4+1]*tmp[0] + pose[1*4+1]*tmp[1] + pose[2*4+1]*tmp[2];
		ret._z = pose[0*4+2]*tmp[0] + pose[1*4+2]*tmp[1] + pose[2*4+2]*tmp[2];
	}

	inline point3d_t volumme_t::Project2BaseCamera(const Eigen::Affine3d &pose, const point3d_t &point)
	{
		point3d_t ret;
		ret._x = pose(0,0)*point._x + pose(0,1)*point._y + pose(0,2)*point._z + pose(0,3);
		ret._y = pose(1,0)*point._x + pose(1,1)*point._y + pose(1,2)*point._z + pose(1,3);
		ret._z = pose(2,0)*point._x + pose(2,1)*point._y + pose(2,2)*point._z + pose(2,3);
		return ret;
	}
	inline point3d_t volumme_t::Project2BaseCamera(const double *pose, const point3d_t &point)
	{
		point3d_t ret;
		ret._x = pose[0*4+0]*point._x + pose[0*4+1]*point._y + pose[0*4+2]*point._z + pose[0*4+3];
		ret._y = pose[1*4+0]*point._x + pose[1*4+1]*point._y + pose[1*4+2]*point._z + pose[1*4+3];
		ret._z = pose[2*4+0]*point._x + pose[2*4+1]*point._y + pose[2*4+2]*point._z + pose[2*4+3];
		return ret;
	}

	inline void volumme_t::Project2Image(index2d_t &ret, const point3d_t & point)
	{
		ret._u = (_intrins._fx*(point._x/point._z)+_intrins._cx);
		ret._v = (_intrins._fy*(point._y/point._z)+_intrins._cy);
	}

	inline point3d_t &volumme_t::Image2Point3d(point3d_t &ret, const index2d_t &index,const double &depth)
	{
		ret._x = depth *((double)(index._u-_intrins._cx)/_intrins._fx);
		ret._y = depth *((double)(index._v - _intrins._cy)/_intrins._fy);
		ret._z = depth;
		return ret;
	}
	inline double volumme_t::getDepthFromIm(cv::Mat &depth,int u,int v)
	{
		return (double)(depth.at<unsigned short>(v,u))/1000.0;
	}
	int sdf_t::Integrate(cv::Mat &depth,const Eigen::Affine3d &pose2base)
	{
		// initialConf(depth);
		point3d_t voxel;
		point3d_t voxel_tmp;
		double depth_d = 0;
		double diff =0;
		index2d_t depthindex;
		double tsdf;
		double weight;
		int nindex;
		for(int k = 0; k<_zsize; ++k)
		{
			voxel._z = (k+_index_offset._z)*(_scale._zscale) + _offset._z;
			voxel_tmp._z = voxel._z - pose2base(2,3);
			for(int j = 0; j<_ysize; ++j)
			{
				voxel._y = (j+_index_offset._y)*(_scale._yscale) + _offset._y;
				voxel_tmp._y = voxel._y - pose2base(1,3);
				for(int i = 0; i<_xsize; ++i)
				{
					voxel._x = (i+_index_offset._x)*(_scale._xscale) + _offset._x;
					voxel_tmp._x = voxel._x - pose2base(0,3);
					voxel._x = pose2base(0,0)*voxel_tmp._x + pose2base(1,0)*voxel_tmp._y + pose2base(2,0)*voxel_tmp._z;
					voxel._y = pose2base(0,1)*voxel_tmp._x + pose2base(1,1)*voxel_tmp._y + pose2base(2,1)*voxel_tmp._z;
					voxel._z = pose2base(0,2)*voxel_tmp._x + pose2base(1,2)*voxel_tmp._y + pose2base(2,2)*voxel_tmp._z;

					if(voxel._z <= 0)
						continue;
					
					Project2Image(depthindex,voxel);
					if(depthindex._u<0 || depthindex._u >=depth.cols || depthindex._v<0 || depthindex._v>=depth.rows)
						continue;

					depth_d = getDepthFromIm(depth,depthindex._u,depthindex._v);
					if(depth_d <=_minDepth || depth_d >=_maxDepth )
						continue;
					diff = depth_d - voxel._z;
					if(diff <= -_delta)
						continue;
					
					index3d_t index(i,j,k);
					nindex = index3d_2_1d(index);
					tsdf = fmin(1.0f,diff / _delta);
					weight =1.0f;
					_vertice_tsdf[nindex] = (_vertice_weight[nindex]*_vertice_tsdf[nindex]+weight*tsdf)/(_vertice_weight[nindex]+weight);
					_vertice_weight[nindex] += weight;
					
				}
				//std::cout<<"j: "<<j<<std::endl;
			}
			// std::cout<<"k: "<<k<<std::endl;
		}
		_frames_intefrated +=1;
		return 0;
	}
/*
	// int sdf_t::Integrate(cv::Mat depth,const Eigen::Affine3d &pose2base)
	// {
	// 	// initialConf(depth);
	// 	point3d_t voxel;
	// 	double depth_d = 0;
	// 	double diff =0;
	// 	index2d_t depthindex;
	// 	double tsdf;
	// 	double weight;
	// 	int nindex;
	// 	for(int k = 0; k<_zsize; ++k)
	// 	{
	// 		for(int j = 0; j<_ysize; ++j)
	// 		{
	// 			for(int i = 0; i<_xsize; ++i)
	// 			{
	// 				index3d_t index(i,j,k);
	// 				index3d2point3d(voxel,index);
	// 				Project2CurCamera(voxel,pose2base,voxel);
	// 				if(voxel._z <= 0)
	// 					continue;
					
	// 				Project2Image(depthindex,voxel);
	// 				if(depthindex._u<0 || depthindex._u >=depth.cols || depthindex._v<0 || depthindex._v>=depth.rows)
	// 					continue;

	// 				depth_d = (double)(depth.at<unsigned short>(depthindex._v,depthindex._u))/1000.0;
	// 				if(depth_d <=0 || depth_d >6 )
	// 					continue;
					
	// 				diff = depth_d - voxel._z;
	// 				if(diff <= -_delta)
	// 					continue;
					
	// 				nindex = index3d_2_1d(index);
	// 				tsdf = fmin(1.0f,diff / _delta);
	// 				weight =1.0f;
	// 				_vertice_tsdf[nindex] = (_vertice_weight[nindex]*_vertice_tsdf[nindex]+weight*tsdf)/(_vertice_weight[nindex]+weight);
	// 				_vertice_weight[nindex] += weight;
	// 			}
	// 			//std::cout<<"j: "<<j<<std::endl;
	// 		}
	// 		// std::cout<<"k: "<<k<<std::endl;
	// 	}
	// 	_frames_intefrated +=1;
	// 	return 0;
	// }
*/
	int sdf_t::Integrate(cv::Mat &depth,const double *pose2base)
	{
		point3d_t voxel;
		point3d_t voxel_tmp;
		double depth_d = 0;
		double diff =0;
		index2d_t depthindex;
		double tsdf;
		double weight;
		int nindex;
		for(int k = 0; k<_zsize; ++k)
		{
			voxel._z = (k+_index_offset._z)*(_scale._zscale) + _offset._z;
			voxel_tmp._z = voxel._z - pose2base[2*4+3];
			for(int j = 0; j<_ysize; ++j)
			{
				voxel._y = (j+_index_offset._y)*(_scale._yscale) + _offset._y;
				voxel_tmp._y = voxel._y - pose2base[1*4+3];
				for(int i = 0; i<_xsize; ++i)
				{
					voxel._x = (i+_index_offset._x)*(_scale._xscale) + _offset._x;
					voxel_tmp._x = voxel._x - pose2base[0*4+3];
					voxel._x = pose2base[0*4+0]*voxel_tmp._x + pose2base[1*4+0]*voxel_tmp._y + pose2base[2*4+0]*voxel_tmp._z;
					voxel._y = pose2base[0*4+1]*voxel_tmp._x + pose2base[1*4+1]*voxel_tmp._y + pose2base[2*4+1]*voxel_tmp._z;
					voxel._z = pose2base[0*4+2]*voxel_tmp._x + pose2base[1*4+2]*voxel_tmp._y + pose2base[2*4+2]*voxel_tmp._z;

					if(voxel._z <= 0)
						continue;
					
					Project2Image(depthindex,voxel);
					if(depthindex._u<0 || depthindex._u >=depth.cols || depthindex._v<0 || depthindex._v>=depth.rows)
						continue;

					depth_d = getDepthFromIm(depth,depthindex._u,depthindex._v);
					if(depth_d <=_minDepth || depth_d >=_maxDepth )
						continue;
					diff = depth_d - voxel._z;
					if(diff <= -_delta)
						continue;
					
					index3d_t index(i,j,k);
					nindex = index3d_2_1d(index);
					tsdf = fmin(1.0f,diff / _delta);
					weight =1.0f;
					_vertice_tsdf[nindex] = (_vertice_weight[nindex]*_vertice_tsdf[nindex]+weight*tsdf)/(_vertice_weight[nindex]+weight);
					_vertice_weight[nindex] += weight;
				}
				//std::cout<<"j: "<<j<<std::endl;
			}
			// std::cout<<"k: "<<k<<std::endl;
		}
		_frames_intefrated +=1;
		return 0;
	}

	int sdf_t::IntegrateOpt(cv::Mat &depth,const Eigen::Affine3d &pose2base)
	{
		point3d_t voxel;
		point3d_t voxel_tmp;
		double depth_d = 0;
		double diff =0;
		index2d_t depthindex;
		double tsdf;
		double weight;
		index3d_t Start, End;
		for(int z=0;z<_zsize;++z)
		{
			//计算x和y的范围
			voxel._z = (z+_index_offset._z)*(_scale._zscale) + _offset._z;
			voxel_tmp._z = voxel._z - pose2base(2,3);
			double Zw = voxel._z;

			if(Zw < 0)
				continue;
			// std::cout<< "Zw: "<<Zw<<std::endl;
			double Xw0 = Zw*((0-_intrins._cx)/_intrins._fx);
			double Xw1 = Zw*((depth.cols-_intrins._cx)/_intrins._fx);
			double Yw0 = Zw*((0-_intrins._cy)/_intrins._fy);
			double Yw1 = Zw*((depth.rows-_intrins._cy)/_intrins._fy);
			// std::cout<< "Zw: "<<Zw<<" Yw0: "<<Yw0<<" Yw1: "<<Yw1<<" Xw0: "<<Xw0<<" Xw1: "<<Xw1<< std::endl;
			point3d_t tmp0(Xw0,Yw0,Zw);
			point3d_t tmp1(Xw1,Yw1,Zw);
			Project2CurCamera(tmp0,pose2base,tmp0);
			Project2CurCamera(tmp1,pose2base,tmp1);
			point3d2index3d(Start,tmp0);
			point3d2index3d(End,tmp1);
			// std::cout<<"start x:"<<Start._x<<" end x:"<< End._x <<endl;
			// std::cout<<"start y:"<<Start._y<<" end y:"<< End._y <<endl;
			for(int y = Start._y;y<End._y&&y<_ysize;++y)
			{
				voxel._y = (y+_index_offset._y)*(_scale._yscale) + _offset._y;
				voxel_tmp._y = voxel._y - pose2base(1,3);
				for(int x = Start._x;x<End._x&&x<_xsize;++x)
				{
					voxel._x = (x+_index_offset._x)*(_scale._xscale) + _offset._x;
					voxel_tmp._x = voxel._x - pose2base(0,3);
					voxel._x = pose2base(0,0)*voxel_tmp._x + pose2base(1,0)*voxel_tmp._y + pose2base(2,0)*voxel_tmp._z;
					voxel._y = pose2base(0,1)*voxel_tmp._x + pose2base(1,1)*voxel_tmp._y + pose2base(2,1)*voxel_tmp._z;
					voxel._z = pose2base(0,2)*voxel_tmp._x + pose2base(1,2)*voxel_tmp._y + pose2base(2,2)*voxel_tmp._z;
					if(voxel._z <= 0)
						continue;
					
					Project2Image(depthindex,voxel);
					if(depthindex._u<0 || depthindex._u >=depth.cols || depthindex._v<0 || depthindex._v>=depth.rows)
						continue;
						// diff = -_delta;
					depth_d = getDepthFromIm(depth,depthindex._u,depthindex._v);
					if(depth_d <=_minDepth || depth_d >_maxDepth )
						continue;
					diff = depth_d - voxel._z;
					if(diff <= -_delta)
						continue;
					index3d_t index(x,y,z);
					int nindex = index3d_2_1d(index);
					tsdf = fmin(1.0f,diff / _delta);
					weight =1.0f;
					_vertice_tsdf[nindex] = (_vertice_weight[nindex]*_vertice_tsdf[nindex]+weight*tsdf)/(_vertice_weight[nindex]+weight);
					_vertice_weight[nindex] += weight;
				}
			}
			// std::cout<<"z: "<<z<<std::endl;
		}
		return 0;
	}

	int sdf_t::IntegrateOpt(cv::Mat &depth,const double *pose2base)
	{
		point3d_t voxel;
		point3d_t voxel_tmp;
		double depth_d = 0;
		double diff =0;
		index2d_t depthindex;
		double tsdf;
		double weight;
		index3d_t Start, End;
		for(int z=0;z<_zsize;++z)
		{
			//计算x和y的范围
			voxel._z = (z+_index_offset._z)*(_scale._zscale) + _offset._z;
			voxel_tmp._z = voxel._z - pose2base[2*4+3];
			double Zw = voxel._z;

			if(Zw < 0)
				continue;
			// std::cout<< "Zw: "<<Zw<<std::endl;
			double Xw0 = Zw*((0-_intrins._cx)/_intrins._fx);
			double Xw1 = Zw*((depth.cols-_intrins._cx)/_intrins._fx);
			double Yw0 = Zw*((0-_intrins._cy)/_intrins._fy);
			double Yw1 = Zw*((depth.rows-_intrins._cy)/_intrins._fy);
			// std::cout<< "Zw: "<<Zw<<" Yw0: "<<Yw0<<" Yw1: "<<Yw1<<" Xw0: "<<Xw0<<" Xw1: "<<Xw1<< std::endl;
			point3d_t tmp0(Xw0,Yw0,Zw);
			point3d_t tmp1(Xw1,Yw1,Zw);
			Project2CurCamera(tmp0,pose2base,tmp0);
			Project2CurCamera(tmp1,pose2base,tmp1);
			point3d2index3d(Start,tmp0);
			point3d2index3d(End,tmp1);
			// std::cout<<"start x:"<<Start._x<<" end x:"<< End._x <<endl;
			// std::cout<<"start y:"<<Start._y<<" end y:"<< End._y <<endl;

			for(int y = Start._y;y<End._y&&y<_ysize;++y)
			{
				voxel._y = (y+_index_offset._y)*(_scale._yscale) + _offset._y;
				voxel_tmp._y = voxel._y - pose2base[1*4+3];
				for(int x = Start._x;x<End._x&&x<_xsize;++x)
				{
					voxel._x = (x+_index_offset._x)*(_scale._xscale) + _offset._x;
					voxel_tmp._x = voxel._x - pose2base[0*4+3];
					voxel._x = pose2base[0*4+0]*voxel_tmp._x + pose2base[1*4+0]*voxel_tmp._y + pose2base[2*4+0]*voxel_tmp._z;
					voxel._y = pose2base[0*4+1]*voxel_tmp._x + pose2base[1*4+1]*voxel_tmp._y + pose2base[2*4+1]*voxel_tmp._z;
					voxel._z = pose2base[0*4+2]*voxel_tmp._x + pose2base[1*4+2]*voxel_tmp._y + pose2base[2*4+2]*voxel_tmp._z;
					if(voxel._z <= 0)
						continue;
					
					Project2Image(depthindex,voxel);
					if(depthindex._u<0 || depthindex._u >=depth.cols || depthindex._v<0 || depthindex._v>=depth.rows)
						continue;
						// diff = -_delta;
					depth_d = getDepthFromIm(depth,depthindex._u,depthindex._v);
					if(depth_d <=_minDepth || depth_d >_maxDepth )
						continue;
					diff = depth_d - voxel._z;
					if(diff <= -_delta)
						continue;
					index3d_t index(x,y,z);
					int nindex = index3d_2_1d(index);
					tsdf = fmin(1.0f,diff / _delta);
					weight =1.0f;
					_vertice_tsdf[nindex] = (_vertice_weight[nindex]*_vertice_tsdf[nindex]+weight*tsdf)/(_vertice_weight[nindex]+weight);
					_vertice_weight[nindex] += weight;
				}
			}
			// std::cout<<"z: "<<z<<std::endl;
		}
		return 0;
	}

	
	int sdf_t::IntegrateUV(cv::Mat &depth,const Eigen::Affine3d &pose2base)
	{
		if(depth.empty())
			return -1;
		int v=0,u=0;
		double vy=0,v1y=0; //u,v对应的在相机坐标系下的值(归一化坐标，后面需要乘上深度)
		double ux=0, u1x=0;
		// point3d_t *uv=new point3d_t, *u1v=new point3d_t,*uv1=new point3d_t,*u1v1=new point3d_t;
		point3d_t uv, u1v1;
		// point3d_t mesh_leftup;//, mesh_rightdown;
		index3d_t mesh_Start, mesh_End;
		double diff,tsdf,weight;
		int nindex=0;
		// int nindex_zbase = 0,nindex_zStep = _ysize*_xsize;
		// int nindex_ybase = 0;
		index3d_t index;
		double depth_d = 0;
		double xStep = (double)((double)_uStep)/_intrins._fx;
		double yStep = (double)((double)_vStep)/_intrins._fy;
		vy = (double)((double)v - (double)_vStep/2 -_intrins._cy)/_intrins._fy -yStep;
		v1y= (double)((double)v + (double)_vStep/2 -_intrins._cy)/_intrins._fy - yStep;
		for(v=0;v<depth.rows;v+=_vStep)
		{
			vy += yStep;
			v1y +=yStep;
			u=0;
			ux = (double)((double)u - (double)_uStep/2 -_intrins._cx)/_intrins._fx - xStep;
			u1x = (double)((double)u + (double)_uStep/2 -_intrins._cx)/_intrins._fx - xStep;
			// vy = (double)((double)v - 0.5 -_intrins._cy)/_intrins._fy;
			// v1y= (double)((double)v+ 0.5 -_intrins._cy)/_intrins._fy;
			for(u=0;u<depth.cols;u+=_uStep) //正向遍历
			{
				// ux = (double)((double)u - 0.5 -_intrins._cx)/_intrins._fx;
				// u1x = (double)((double)u + 0.5 -_intrins._cx)/_intrins._fx;
				ux += xStep;
				u1x +=xStep;
				uv._z = getDepthFromIm(depth,u,v);
				
				u1v1._z = uv._z;//getDepthFromIm(depth,u+1,v+1);
				uv._y = vy*uv._z;
				uv._x = ux*uv._z;
				u1v1._y = v1y*u1v1._z; 
				u1v1._x = u1x*u1v1._z;
				uv = Project2BaseCamera(pose2base,uv);
				u1v1 = Project2BaseCamera(pose2base,u1v1);
				depth_d = uv._z;
				uv._z = uv._z - _delta*1.01;
				u1v1._z = u1v1._z + _delta*1.01;
				point3d2index3d(mesh_Start,uv);
				point3d2index3d(mesh_End,u1v1);
				// uv._z = uv._z + _delta*1.2;
				// uv._z = depth_d;
				// std::cout<<"u: "<<u<<"v: "<<v <<"; Start: "<<"x: "<<mesh_Start._x<<"; y: "<<mesh_Start._y<<"; z:"<<mesh_Start._z<<std::endl;
				// std::cout<<"u: "<<u<<"v: "<<v <<"; End:   "<<"x: "<<mesh_End._x<<"; y: "<<mesh_End._y<<"; z:"<<mesh_End._z<<std::endl;
				if(depth_d>_minDepth && depth_d<_maxDepth)
				{
					diff = (mesh_Start._z -1 +0.5)*_scale._zscale + _offset._z;
					// diff = uv._z - diff;
					diff = depth_d - diff;
					// nindex_zbase = (mesh_Start._z-1)*nindex_zStep;
					for(int z = mesh_Start._z ; z<=mesh_End._z;++z)
					{
						// nindex_zbase += nindex_zStep;
						if(z>=_zsize)
							continue;
						
						diff -= _scale._zscale;
						if(diff <=-_delta || diff >= _delta)
							continue;

						index._z = z;
						// nindex_ybase =nindex_zbase + (mesh_Start._y-1)*_xsize;
						for(int y=mesh_Start._y;y<=mesh_End._y;++y)
						{
							// nindex_ybase += _xsize;
							if(y>=_ysize)
								continue;
							index._y = y;
							// nindex = nindex_ybase + mesh_Start._x -1;
							for(int x=mesh_Start._x;x<=mesh_Start._x;++x)
							{
								// nindex ++;
								if(x>=_xsize)
									continue;
								index._x =x;
								nindex = index3d_2_1d(index);
								tsdf = diff/_delta;//fmin(1.0f,diff/_delta);
								weight = _vertice_weight[nindex];
								_vertice_weight[nindex] += 1.0;
								_vertice_tsdf[nindex] = (weight*_vertice_tsdf[nindex]+tsdf)/(_vertice_weight[nindex]);
							}
						}
					}
				}	
			}	
		}
		return 0;
	}
	//now it desn't work correct.
	int sdf_t::IntegrateUVOpt(cv::Mat &depth,const Eigen::Affine3d &pose2base)
	{
		if(depth.empty())
			return -1;
		int v=0,u=0;
		point3d_t mesh_leftup, mesh_rightdown;
		index3d_t mesh_Start, mesh_End;
		double diff,tsdf,weight;
		int nindex=0;
		index3d_t index;
		point3d_t puv;
		double xhalfL = (double)_uStep/2/_intrins._fx;
		double yhalfL = (double)_vStep/2/_intrins._fy;
		double xhalfLatB[3] = {0};
		double yhalfLatB[3] = {0};
		double puvy=0;
		for(v=0;v<depth.rows-1;v+=_vStep)
		{
			puvy = (double)((double)v -_intrins._cy)/_intrins._fy;
			for(u=0;u<depth.cols-1;u+=_uStep) //正向遍历
			{
				puv._x = (double)((double)u-_intrins._cx)/_intrins._fx;
				// puv._y= (double)((double)v -_intrins._cy)/_intrins._fy;
				puv._z = getDepthFromIm(depth,u,v);
				puv._x = puv._x*puv._z;
				puv._y = puvy*puv._z;
				xhalfL = xhalfL*puv._z;
				yhalfL = yhalfL*puv._z;
				// std::cout<<"puv.z: "<<puv._z<<std::endl;
				
				for(int hid = 0; hid<3; ++hid)
					xhalfLatB[hid] = pose2base(hid,0)*xhalfL;
				for(int hid = 0; hid<3;++hid)
					yhalfLatB[hid] = pose2base(hid,1)*yhalfL;

				puv = Project2BaseCamera(pose2base,puv);
				// ret._x = pose(0,0)*point._x + pose(0,1)*point._y + pose(0,2)*point._z + pose(0,3);
				// ret._y = pose(1,0)*point._x + pose(1,1)*point._y + pose(1,2)*point._z + pose(1,3);
				// ret._z = pose(2,0)*point._x + pose(2,1)*point._y + pose(2,2)*point._z + pose(2,3);
				mesh_leftup._x = puv._x - xhalfLatB[0] - yhalfLatB[0];
				mesh_leftup._y = puv._y - xhalfLatB[1] - yhalfLatB[1];
				mesh_leftup._z = puv._z - xhalfLatB[2] - yhalfLatB[2]- _delta*1.01;

				mesh_rightdown._x = puv._x + xhalfLatB[0] + yhalfLatB[0];
				mesh_rightdown._y = puv._y + xhalfLatB[1] + yhalfLatB[1];
				mesh_rightdown._z = puv._z + xhalfLatB[2] + yhalfLatB[2] + _delta*1.01;
				point3d2index3d(mesh_Start,mesh_leftup);
				point3d2index3d(mesh_End,mesh_rightdown);
				// std::cout<<"u: "<<u<<"v: "<<v <<"; Start: "<<"x: "<<mesh_Start._x<<"; y: "<<mesh_Start._y<<"; z:"<<mesh_Start._z<<std::endl;
				// std::cout<<"u: "<<u<<"v: "<<v <<"; End:   "<<"x: "<<mesh_End._x<<"; y: "<<mesh_End._y<<"; z:"<<mesh_End._z<<std::endl;
				if(puv._z>_minDepth && puv._z<_maxDepth)
				{
					diff = (mesh_Start._z -1 +0.5)*_scale._zscale + _offset._z;
					diff = puv._z - diff;
					// diff = depth_d - diff;
					for(int z = mesh_Start._z ; z<=mesh_End._z;++z)
					{
						if(z>=_zsize)
							continue;
						index._z = z;
						diff -= _scale._zscale;
						if(diff <=-_delta || diff >= _delta)
							continue;
						for(int y=mesh_Start._y;y<=mesh_End._y;++y)
						{
							if(y>=_ysize)
								continue;
							index._y = y;
							for(int x=mesh_Start._x;x<=mesh_Start._x;++x)
							{
								if(x>=_xsize)
									continue;
								index._x =x;
								nindex = index3d_2_1d(index);
								tsdf = diff/_delta;//fmin(1.0f,diff/_delta);
								weight = _vertice_weight[nindex];
								_vertice_weight[nindex] += 1.0;
								_vertice_tsdf[nindex] = (weight*_vertice_tsdf[nindex]+tsdf)/(_vertice_weight[nindex]);
							}
						}
					}
				}
			}
		}
		return 0;
	}


	int sdf_t::IntegrateUV(cv::Mat &depth,const double *pose2base)
	{
		if(depth.empty())
			return -1;
		int v=0,u=0;
		double vy=0,v1y=0; //u,v对应的在相机坐标系下的值(归一化坐标，后面需要乘上深度)
		double ux=0, u1x=0;
		// point3d_t *uv=new point3d_t, *u1v=new point3d_t,*uv1=new point3d_t,*u1v1=new point3d_t;
		point3d_t uv, u1v1;
		// point3d_t mesh_leftup;//, mesh_rightdown;
		index3d_t mesh_Start, mesh_End;
		double diff,tsdf,weight;
		int nindex=0;
		index3d_t index;
		double depth_d = 0;
		double xStep = (double)((double)_uStep)/_intrins._fx;
		double yStep = (double)((double)_vStep)/_intrins._fy;
		vy = (double)((double)v - (double)_vStep/2 -_intrins._cy)/_intrins._fy -yStep;
		v1y= (double)((double)v + (double)_vStep/2 -_intrins._cy)/_intrins._fy - yStep;
		for(v=0;v<depth.rows;v+=_vStep)
		{
			vy += yStep;
			v1y +=yStep;
			u=0;
			ux = (double)((double)u - (double)_uStep/2 -_intrins._cx)/_intrins._fx - xStep;
			u1x = (double)((double)u + (double)_uStep/2 -_intrins._cx)/_intrins._fx - xStep;
			// vy = (double)((double)v - 0.5 -_intrins._cy)/_intrins._fy;
			// v1y= (double)((double)v+ 0.5 -_intrins._cy)/_intrins._fy;
			for(u=0;u<depth.cols;u+=_uStep) //正向遍历
			{
				// ux = (double)((double)u - 0.5 -_intrins._cx)/_intrins._fx;
				// u1x = (double)((double)u + 0.5 -_intrins._cx)/_intrins._fx;
				ux += xStep;
				u1x +=xStep;
				uv._z = getDepthFromIm(depth,u,v);
				
				u1v1._z = uv._z;//getDepthFromIm(depth,u+1,v+1);
				uv._y = vy*uv._z;
				uv._x = ux*uv._z;
				u1v1._y = v1y*u1v1._z; 
				u1v1._x = u1x*u1v1._z;
				uv = Project2BaseCamera(pose2base,uv);
				u1v1 = Project2BaseCamera(pose2base,u1v1);
				depth_d = uv._z;
				uv._z = uv._z - _delta*1.01;
				u1v1._z = u1v1._z + _delta*1.01;
				point3d2index3d(mesh_Start,uv);
				point3d2index3d(mesh_End,u1v1);
				// uv._z = uv._z + _delta*1.2;
				// uv._z = depth_d;
				// std::cout<<"u: "<<u<<"v: "<<v <<"; Start: "<<"x: "<<mesh_Start._x<<"; y: "<<mesh_Start._y<<"; z:"<<mesh_Start._z<<std::endl;
				// std::cout<<"u: "<<u<<"v: "<<v <<"; End:   "<<"x: "<<mesh_End._x<<"; y: "<<mesh_End._y<<"; z:"<<mesh_End._z<<std::endl;
				if(depth_d>_minDepth && depth_d<_maxDepth)
				{
					diff = (mesh_Start._z -1 +0.5)*_scale._zscale + _offset._z;
					// diff = uv._z - diff;
					diff = depth_d - diff;
					for(int z = mesh_Start._z ; z<=mesh_End._z;++z)
					{
						if(z>=_zsize)
							continue;
						index._z = z;
						diff -= _scale._zscale;
						if(diff <=-_delta || diff >= _delta)
							continue;
						for(int y=mesh_Start._y;y<=mesh_End._y;++y)
						{
							if(y>=_ysize)
								continue;
							index._y = y;
							for(int x=mesh_Start._x;x<=mesh_Start._x;++x)
							{
								if(x>=_xsize)
									continue;
								index._x =x;
								nindex = index3d_2_1d(index);
								tsdf = diff/_delta;//fmin(1.0f,diff/_delta);
								weight = _vertice_weight[nindex];
								_vertice_weight[nindex] += 1.0;
								_vertice_tsdf[nindex] = (weight*_vertice_tsdf[nindex]+tsdf)/(_vertice_weight[nindex]);
							}
						}
					}
				}	
			}	
		}
		return 0;
	}

	int sdf_t::IntegrateXYZ(cv::Mat &depth,const Eigen::Affine3d &pose2base)
	{
		point3d_t voxel;
		point3d_t voxel_tmp;
		index3d_t Start, End;
		index3d_t index;
		
		double Uw0 = (double)((double)(0-_intrins._cx)/_intrins._fx);
		double Vw0 = (double)((double)(0-_intrins._cy)/_intrins._fy);
		double Uw1 = (double)((double)(depth.cols-_intrins._cx)/_intrins._fx);
		double Vw1 = (double)((double)(depth.rows-_intrins._cy)/_intrins._fy);

		for(int z=0;z<_zsize;++z)
		{
			voxel._z = (z+_index_offset._z)*(_scale._zscale) + _offset._z;
			voxel_tmp._z = voxel._z - pose2base(2,3);
			double Zw = voxel._z;

			if(Zw < 0)
				continue;
			// std::cout<< "Zw: "<<Zw<<std::endl;
			double Xw0 = Zw*Uw0;
			double Xw1 = Zw*Uw1;
			double Yw0 = Zw*Vw0;
			double Yw1 = Zw*Vw1;
			// std::cout<< "Zw: "<<Zw<<" Yw0: "<<Yw0<<" Yw1: "<<Yw1<<" Xw0: "<<Xw0<<" Xw1: "<<Xw1<< std::endl;
			point3d_t tmp0(Xw0,Yw0,Zw);
			point3d_t tmp1(Xw1,Yw1,Zw);
			Project2CurCamera(tmp0,pose2base,tmp0);
			Project2CurCamera(tmp1,pose2base,tmp1);
			point3d2index3d(Start,tmp0);
			point3d2index3d(End,tmp1);
			index._z =z;
			IntegrateXY(depth,pose2base,voxel,voxel_tmp,index,index2d_t(Start._x,Start._y),index2d_t(End._x,End._y));
		}
		return 0;
	}
	int sdf_t::IntegrateXYZ(cv::Mat &depth,const double *pose2base)
	{
		point3d_t voxel;
		point3d_t voxel_tmp;
		index3d_t Start, End;
		index3d_t index;

		double Uw0 = (double)((double)(0-_intrins._cx)/_intrins._fx);
		double Vw0 = (double)((double)(0-_intrins._cy)/_intrins._fy);
		double Uw1 = (double)((double)(depth.cols-_intrins._cx)/_intrins._fx);
		double Vw1 = (double)((double)(depth.rows-_intrins._cy)/_intrins._fy);

		for(int z=0;z<_zsize;++z)
		{
			voxel._z = (z+_index_offset._z)*(_scale._zscale) + _offset._z;
			voxel_tmp._z = voxel._z - pose2base[2*4+3];
			double Zw = voxel._z;

			if(Zw < 0)
				continue;
			// std::cout<< "Zw: "<<Zw<<std::endl;
			double Xw0 = Zw*Uw0;
			double Xw1 = Zw*Uw1;
			double Yw0 = Zw*Vw0;
			double Yw1 = Zw*Vw1;
			// std::cout<< "Zw: "<<Zw<<" Yw0: "<<Yw0<<" Yw1: "<<Yw1<<" Xw0: "<<Xw0<<" Xw1: "<<Xw1<< std::endl;
			point3d_t tmp0(Xw0,Yw0,Zw);
			point3d_t tmp1(Xw1,Yw1,Zw);
			Project2CurCamera(tmp0,pose2base,tmp0);
			Project2CurCamera(tmp1,pose2base,tmp1);
			point3d2index3d(Start,tmp0);
			point3d2index3d(End,tmp1);
			index._z =z;
			IntegrateXY(depth,pose2base,voxel,voxel_tmp,index,index2d_t(Start._x,Start._y),index2d_t(End._x,End._y));
		}
		return 0;
	}

	inline int sdf_t::IntegrateXY(
		cv::Mat &depth,const Eigen::Affine3d &pose2base, point3d_t &voxel,point3d_t &voxel_tmp,
		index3d_t &index, const index2d_t &StartXY, const index2d_t &EndXY)
	{
		for(int y=StartXY._v;y<EndXY._v&&y<_ysize;++y)
		{
			index._y = y;
			voxel._y = (y+_index_offset._y)*(_scale._yscale) + _offset._y;
			voxel_tmp._y = voxel._y - pose2base(1,3);
			IntegrateX(depth,pose2base,voxel,voxel_tmp,index,StartXY._u,EndXY._u);
		}
		return 0;
	}
	inline int sdf_t::IntegrateXY(
		cv::Mat &depth,const double *pose2base,point3d_t &voxel,point3d_t &voxel_tmp,
		index3d_t &index, const index2d_t &StartXY, const index2d_t &EndXY)
	{
		for(int y=StartXY._v;y<EndXY._v&&y<_ysize;++y)
		{
			index._y = y;
			voxel._y = (y+_index_offset._y)*(_scale._yscale) + _offset._y;
			voxel_tmp._y = voxel._y - pose2base[1*4+3];
			IntegrateX(depth,pose2base,voxel,voxel_tmp,index,StartXY._u,EndXY._u);
		}
		return 0;
	}

	inline int sdf_t::IntegrateX(
		cv::Mat &depth,const Eigen::Affine3d &pose2base,  point3d_t &voxel,point3d_t &voxel_tmp,index3d_t &index,int Start, int End)
	{
		index2d_t depthindex;
		double depth_d=0;
		double diff =0;
		double tsdf = 0, weight =0;
		int nindex;
		index._x = Start-1;
		int nindexbase = index3d_2_1d(index);
		for(int x = Start;x<End&&x<_xsize;++x)
		{
			voxel._x = (x+_index_offset._x)*(_scale._xscale) + _offset._x;
			voxel_tmp._x = voxel._x - pose2base(0,3);
			voxel._x = pose2base(0,0)*voxel_tmp._x + pose2base(1,0)*voxel_tmp._y + pose2base(2,0)*voxel_tmp._z;
			voxel._y = pose2base(0,1)*voxel_tmp._x + pose2base(1,1)*voxel_tmp._y + pose2base(2,1)*voxel_tmp._z;
			voxel._z = pose2base(0,2)*voxel_tmp._x + pose2base(1,2)*voxel_tmp._y + pose2base(2,2)*voxel_tmp._z;
			if(voxel._z <= 0)
				continue;
			Project2Image(depthindex,voxel);
			if(depthindex._u<0 || depthindex._u >=depth.cols || depthindex._v<0 || depthindex._v>=depth.rows)
				continue;
			depth_d = getDepthFromIm(depth,depthindex._u,depthindex._v);
			if(depth_d <=_minDepth || depth_d >_maxDepth )
				continue;
			diff = depth_d - voxel._z;
			if(diff <= -_delta || diff >= _delta)
				continue;
			// index._x = x;
			// nindex = index3d_2_1d(index);
			nindex = nindexbase + x;
			tsdf = (diff / _delta);
			weight =1.0f;
			
			_vertice_tsdf[nindex] = (_vertice_weight[nindex]*_vertice_tsdf[nindex]+weight*tsdf)/(_vertice_weight[nindex]+weight);
			_vertice_weight[nindex] += weight;
		}
		return 0;
	}
	inline int sdf_t::IntegrateX(
		cv::Mat &depth,const double *pose2base, point3d_t &voxel,point3d_t &voxel_tmp,index3d_t &index, int Start, int End)
	{
		index2d_t depthindex;
		double depth_d=0;
		double diff =0;
		double tsdf = 0, weight =0;
		for(int x = Start;x<End&&x<_xsize;++x)
		{
			voxel._x = (x+_index_offset._x)*(_scale._xscale) + _offset._x;
			voxel_tmp._x = voxel._x - pose2base[0*4+3];
			voxel._x = pose2base[0*4+0]*voxel_tmp._x + pose2base[1*4+0]*voxel_tmp._y + pose2base[2*4+0]*voxel_tmp._z;
			voxel._y = pose2base[0*4+1]*voxel_tmp._x + pose2base[1*4+1]*voxel_tmp._y + pose2base[2*4+1]*voxel_tmp._z;
			voxel._z = pose2base[0*4+2]*voxel_tmp._x + pose2base[1*4+2]*voxel_tmp._y + pose2base[2*4+2]*voxel_tmp._z;
			if(voxel._z <= 0)
				continue;
			
			Project2Image(depthindex,voxel);
			if(depthindex._u<0 || depthindex._u >=depth.cols || depthindex._v<0 || depthindex._v>=depth.rows)
				continue;
				// diff = -_delta;
			depth_d = getDepthFromIm(depth,depthindex._u,depthindex._v);
			if(depth_d <=_minDepth || depth_d >_maxDepth )
				continue;
			diff = depth_d - voxel._z;
			if(diff <= -_delta)
				continue;
			index._x = x;
			int nindex = index3d_2_1d(index);
			tsdf = (diff / _delta);
			weight =1.0f;
			_vertice_tsdf[nindex] = (_vertice_weight[nindex]*_vertice_tsdf[nindex]+weight*tsdf)/(_vertice_weight[nindex]+weight);
			_vertice_weight[nindex] += weight;
		}
		return 0;
	}
	
}

