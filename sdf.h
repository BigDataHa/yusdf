/*************************************************************************
	> File Name: sdf.h
	> Author: yusnows
	> Mail: YusnowsLiang@gmail.com
	> Created Time: Fri 01 Jun 2018 11:33:15 PM CST
    > Copyright@yusnows: All rights reserve!
 ************************************************************************/

#ifndef _SDF_H
#define _SDF_H

#include <opencv2/opencv.hpp>
#include <memory.h>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>

namespace sdf
{
	class RGB_t
	{
	public:
		RGB_t():r(0),g(0),b(0)
		{}
		RGB_t(uint8_t red, uint8_t green, uint8_t blue):r(red),g(green),b(blue)
		{}
		~RGB_t(){}
	public:
		uint8_t r, g, b;
	};
	class scale3d_t
	{
	public:
		scale3d_t():_xscale(0.001),_yscale(0.001),_zscale(0.001)
		{}
		scale3d_t(double x,double y,double z):_xscale(x),_yscale(y),_zscale(z)
		{}
		~scale3d_t(){}
	public:
		double _xscale, _yscale, _zscale; //各个维度的缩放尺度
	};

	class point3d_t
	{
	public:
		point3d_t():_x(-1.5),_y(-1.5),_z(0.5)
		{}
		point3d_t(double x,double y,double z):_x(x),_y(y),_z(z)
		{}
		~point3d_t(){}
		point3d_t operator+(const point3d_t &r)
		{
			point3d_t ret((this->_x + r._x),(this->_y + r._y), (this->_z + r._z));
			return ret;
		}
		point3d_t operator-(const point3d_t &r)
		{
			point3d_t ret((this->_x - r._x),(this->_y - r._y), (this->_z - r._z));
			return ret;
		}
		point3d_t operator/(double div)
		{
			point3d_t ret((this->_x/div),(this->_y/div),(this->_z/div));
			return ret;
		}
		point3d_t operator/(scale3d_t &scale)
		{
			point3d_t ret((this->_x/scale._xscale),(this->_y/scale._yscale),(this->_z/scale._zscale));
			return ret;
		}
		point3d_t operator*(scale3d_t &scale)
		{
			point3d_t ret((this->_x * scale._xscale),(this->_y * scale._yscale), (this->_z * scale._zscale));
			return ret;
		}
	public:
		double _x, _y, _z; 
	};

	class index3d_t
	{
	public:
		index3d_t(int x=0,int y=0, int z=0):_x(x),_y(y),_z(z)
		{}
		~index3d_t()
		{}

		point3d_t operator*(scale3d_t &scale)
		{
			point3d_t ret;
			ret._x = ((double)this->_x)*scale._xscale;
			ret._y = ((double)this->_y)*scale._yscale;
			ret._z = ((double)this->_z)*scale._zscale;
			return ret;
		}
	public:
		int _x, _y, _z;
	};

	class index2d_t
	{
	public:
		index2d_t(int x=0,int y=0):_u(x),_v(y)
		{}
		~index2d_t()
		{}
	public:
		int _u,_v;
	};

	class vertex_t
	{
	public:
		vertex_t(double vertex = 1,double weight=0, bool hascolor=false)
		:_vertex(vertex),_weight(weight),_hascolor(hascolor)
		{}
		~vertex_t()
		{}
	public:
		double _vertex;		//存储vertex的值。_vertex = D(V) - Vz
		double _weight;		//存储vertex对应的权重。
		bool _hascolor;
		RGB_t _color;		//存储每一个vetex的颜色，为了显示效果需要，默认不开启。
	};

	class cameraIntrin_t
	{
	public:
		cameraIntrin_t(double fx=906.3987,double fy=903.4451,double cx=597.1193,double cy=346.2206):_fx(fx),_fy(fy),_cx(cx),_cy(cy)
		{}
		~cameraIntrin_t()
		{}
	public:
		double _fx,_fy;
		double _cx,_cy;
	};

	//use a 1 dim array to represent 3 dim array.
	class volumme_t
	{
	public:
		volumme_t()
		{
			_vertice_tsdf=NULL;
			_vertice_weight=NULL;
			_hascolor=false;
			_vertice_color=NULL;
		}
		volumme_t(int xsize ,int ysize  ,int zsize, bool hascolor = false)
		:_xsize(xsize),_ysize(ysize),_zsize(zsize),_size(xsize*ysize*zsize),_hascolor(hascolor)//,_unit(scale3d_t(1,1,0.0001))
		{
			_vertice_tsdf =new double[_size];
			for(int i=0;i<_size;++i)
				_vertice_tsdf[i] = 1.0f;
			_vertice_weight = new double[_size];
			memset(_vertice_weight,0,_size*sizeof(double));
			if(_hascolor)
				_vertice_color = new RGB_t[_size];
			else
				_vertice_color=NULL;
		}
		~volumme_t()
		{
			if(_vertice_color != NULL)
				delete[] _vertice_color;
			if(_vertice_tsdf != NULL)
				delete[] _vertice_tsdf;
			if(_vertice_weight != NULL)
				delete[] _vertice_weight;
		}
		int setOffset(const point3d_t &offset);
		int setOffset(const double &x,const double &y,const double &z);
		int setScale(const scale3d_t &scale);
		int setScale(const double &xscale,const double &yscale,const double &zscale);
		// int setUnit(const scale3d_t &unit);
		// int setUnit(const double &xunit,const double &yunit,const double &zunit);
		int setCameraIntrins(const cameraIntrin_t &intrins);
		int setCameraIntrins(const double &fx,const double &fy,const double &cx,const double &cy);
		const point3d_t &getOffset() const;
		const scale3d_t &getScale() const;
		// const scale3d_t &getUnit()const;
		const cameraIntrin_t &getCameraIntrins()const;
		double &operator[](const index3d_t &index);
		double &operator[](const point3d_t &point);
		// const vertex_t &getVertex(const point3d_t &point);
		// const vertex_t &getVertex(const index3d_t &index);
		// const vertex_t &getVertex(const int x,const int y,const int z);
		double *getVerticeTsdf()
		{
			return _vertice_tsdf;
		}
		double *getVerticeWeight()
		{
			return _vertice_weight;
		}

		// int calVertex(double &tsdf, double &weight, const double &diff, const double &delta, const double &eta);

	protected:
		inline int index3d_2_1d(const index3d_t &index);
		inline void index3d_2_1d(int &ret,const index3d_t &index);
		inline index3d_t point3d2index3d(const point3d_t &point);
		inline void point3d2index3d(index3d_t &ret,const point3d_t &point);
		inline void index3d2point3d(point3d_t &ret,const index3d_t &index);
		inline void Project2CurCamera(point3d_t &ret,const Eigen::Affine3d &pose, const point3d_t &point);
		inline void Project2CurCamera(point3d_t &ret,const double *pose, const point3d_t &point);
		inline point3d_t Project2BaseCamera(const Eigen::Affine3d &pose, const point3d_t &point);
		inline point3d_t Project2BaseCamera(const double *pose, const point3d_t &point);
		inline void Project2Image(index2d_t &ret, const point3d_t & point);

		inline point3d_t &Image2Point3d(point3d_t &ret, const index2d_t &index,const double &depth);

		inline double getDepthFromIm(cv::Mat &depth,int u,int v);

	protected:
		int _xsize, _ysize, _zsize;		//各个维度的大小，即格子的数量
		int _size;		//数据总量的大小
		scale3d_t _scale; //缩放倍数，可能>1或<1,单位为：米
		// scale3d_t _unit; 
		point3d_t _offset;
		cameraIntrin_t _intrins;
		// vertex_t *_vertice;
		double *_vertice_tsdf;
		double *_vertice_weight;
		bool _hascolor;
		RGB_t *_vertice_color;
	};

	class sdf_t:public volumme_t
	{
	public:
		sdf_t()
		:volumme_t(512,512,512),_delta(0.03),_eta(0.03),_maxDepth(6),_minDepth(0),_uStep(2),_vStep(2),_frames_intefrated(0)
		{
			// _verticeReach = new unsigned char[_size];
			// memset(_verticeReach,0,_size*sizeof(unsigned char));
		}
		sdf_t(int xsize,int ysize,int zsize,double delta = 0.03,double eta =0.03)
		:volumme_t(xsize,ysize,zsize), _delta(delta),_eta(eta),_maxDepth(6),_minDepth(0),_uStep(2),_vStep(2),_frames_intefrated(0)
		{
			// _verticeReach = new unsigned char[_size];
			// memset(_verticeReach,0,_size*sizeof(unsigned char));
		}
		~sdf_t()
		{
			// if(_verticeReach !=NULL)
			// 	delete[] _verticeReach;
		}
		int setDelta(const double &delta)
		{
			_delta = delta;
			return 0;
		}
		int setEta(const double &eta)
		{
			_eta = eta;
			return 0;
		}
		int setMaxMinDepth(const double &max, const double &min)
		{
			_maxDepth = max;
			_minDepth = min;
			return 0;
		}
		int setuvStep(const int ustep,const int vstep)
		{
			_uStep = ustep;
			_vStep = vstep;
			return 0;
		}
		double getDelta()
		{
			return _delta;
		}
		double getEta()
		{
			return _eta;
		}
		double getMaxDepth()
		{
			return _maxDepth;
		}
		double getMinDepth()
		{
			return _minDepth;
		}
		int Integrate(cv::Mat &depth,const Eigen::Affine3d &pose2base);
		int Integrate(cv::Mat &depth,const double *pose2base);
		int IntegrateOpt(cv::Mat &depth,const Eigen::Affine3d &pose2base);
		int IntegrateOpt(cv::Mat &depth,const double *pose2base);

		int IntegrateXYZ(cv::Mat &depth,const Eigen::Affine3d &pose2base);
		int IntegrateXYZ(cv::Mat &depth,const double *pose2base);

		int IntegrateUV(cv::Mat &depth,const Eigen::Affine3d &pose2base);
		int IntegrateUV(cv::Mat &depth,const double *pose2base);

		int IntegrateUVOpt(cv::Mat &depth,const Eigen::Affine3d &pose2base);

	protected:
		//index2d_t: u is for Z, and v is for Y
		inline int IntegrateXY(cv::Mat &depth,const Eigen::Affine3d &pose2base,point3d_t &voxel,point3d_t &voxel_tmp,index3d_t &num,  const index2d_t &StartXY, const index2d_t &EndXY);
		inline int IntegrateXY(cv::Mat &depth,const double *pose2base,point3d_t &voxel,point3d_t &voxel_tmp,index3d_t &num,  const index2d_t &StartXY, const index2d_t &EndXY);

		inline int IntegrateX(cv::Mat &depth,const Eigen::Affine3d &pose2base, point3d_t &voxel,point3d_t &voxel_tmp,index3d_t &num, int Start, int End);
		inline int IntegrateX(cv::Mat &depth,const double *pose2base, point3d_t &voxel,point3d_t &voxel_tmp,index3d_t &num,int Start, int End);

	private:
		double _delta; //相机精度相关
		double _eta;
		double _maxDepth;
		double _minDepth;
		// unsigned char *_verticeReach;
		int _uStep,_vStep;
		int _frames_intefrated;
		// double tmp;
		// int tmp1;
		// double _depth_d;
		//volumme_t _volumme;
	};

}

#endif
