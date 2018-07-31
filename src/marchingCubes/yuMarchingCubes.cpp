/*************************************************************************
	> File Name: yuMarchingCubes.cpp
	> Author: yusnows
	> Mail: YusnowsLiang@gmail.com
	> Created Time: Wed 11 Jul 2018 06:09:17 PM CST
    > Copyright@yusnows: All rights reserve!
 ************************************************************************/
#include "yuMarchingCubes.h"

namespace sdf
{

	int yuMCubes_t::saveIsoPointCloud(const char *filename)
	{	
		if(_isoPointCloud.size()<=0)
			return -1;
		FILE *fp = fopen(filename, "w");
		fprintf(fp, "ply\n");
		fprintf(fp, "format binary_little_endian 1.0\n");
		fprintf(fp, "element vertex %d\n", (int)_isoPointCloud.size());
		fprintf(fp, "property double x\n");
		fprintf(fp, "property double y\n");
		fprintf(fp, "property double z\n");
		fprintf(fp, "end_header\n");
		for(int i=0;i<_isoPointCloud.size();++i)
		{
			fwrite(&_isoPointCloud[i]._x, sizeof(double), 1, fp);
			fwrite(&_isoPointCloud[i]._y, sizeof(double), 1, fp);
			fwrite(&_isoPointCloud[i]._z, sizeof(double), 1, fp);
		}
		return 0;
	}

	int yuMCubes_t::calIsoPointCloud(bool recal)
	{
		if(recal)
			_isoPointCloud.clear();
		int edgeindex;
		for(int i=0;i<_xsize;++i)
		{
			for(int j=0;j<_ysize;++j)
			{
				for(int k=0;k<_zsize;++k)
				{
					int i1 = min(i+1, _xsize), j1 = min(j+1, _ysize), k1 = min(k+1, _zsize);
					edgeindex = calEdgeTabIndex(i,j,k);
					if(edgeindex ==0)
						continue;
					if(edgeTable[edgeindex]&1) 
						calInterpVertex(i,j,k, i1,j,k,_verticeList[0]);
					if(edgeTable[edgeindex]&2) 
						calInterpVertex(i1,j,k, i1,j1,k,_verticeList[1]);
					if(edgeTable[edgeindex]&4)
						calInterpVertex(i1,j1,k, i,j1,k,_verticeList[2]);
					if(edgeTable[edgeindex]&8) 
						calInterpVertex(i,j1,k, i,j,k,_verticeList[3]);
					if(edgeTable[edgeindex]&16) 
						calInterpVertex(i,j,k1, i1,j,k1,_verticeList[4]);
					if(edgeTable[edgeindex]&32) 
						calInterpVertex(i1,j,k1, i1,j1,k1,_verticeList[5]);
					if(edgeTable[edgeindex]&64) 
						calInterpVertex(i1,j1,k1, i,j1,k1,_verticeList[6]);
					if(edgeTable[edgeindex]&128) 
						calInterpVertex(i,j1,k1, i,j,k1,_verticeList[7]);
					if(edgeTable[edgeindex]&256) 
						calInterpVertex(i,j,k, i,j,k1,_verticeList[8]);
					if(edgeTable[edgeindex]&512) 
						calInterpVertex( i1,j,k, i1,j,k1,_verticeList[9]);
					if(edgeTable[edgeindex]&1024) 
						calInterpVertex(i1,j1,k, i1,j1,k1,_verticeList[10]);
					if(edgeTable[edgeindex]&2048) 
						calInterpVertex(i,j1,k, i,j1,k1,_verticeList[11]);
				}
			}
		}
		for(int i=0;triTable[edgeindex][i]!=-1;i+=3)
		{
			_isoPointCloud.push_back(_verticeList[triTable[edgeindex][i]]);
			_isoPointCloud.push_back(_verticeList[triTable[edgeindex][i+1]]);
			_isoPointCloud.push_back(_verticeList[triTable[edgeindex][i+2]]);
		}
		std::cout<<_isoPointCloud.size() << std::endl;
		return 0;
	}

	int yuMCubes_t::setIsoleval(const double &isoleval)
	{
		_isoleval = isoleval;
		return 0;
	}
	int yuMCubes_t::setThreshold(const double &threshold)
	{
		_threshold = threshold;
		return 0;
	}
	double yuMCubes_t::getIsoleval() const
	{
		return _isoleval;
	}
	double yuMCubes_t::getThreshold() const
	{
		return _threshold;
	}
	inline int yuMCubes_t::calEdgeTabIndex(int i,int j, int k)
	{
		int ret = 0;
		int i1 = min(i+1, _xsize), j1 = min(j+1, _ysize), k1 = min(k+1, _zsize);
		// ret |= getVerticeTsdfval(i,j,k)>_isoleval? (getVerticeWeightval(i,j,k)>0?1:0):0;
		// ret |= getVerticeTsdfval(i1,j,k)>_isoleval? (getVerticeWeightval(i1,j,k)>0?2:0):0;
		// ret |= getVerticeTsdfval(i1,j1,k)>_isoleval? (getVerticeWeightval(i1,j1,k)>0?4:0):0;
		// ret |= getVerticeTsdfval(i,j1,k)>_isoleval? (getVerticeWeightval(i,j1,k)>0?8:0):0;
		// ret |= getVerticeTsdfval(i,j,k1)>_isoleval? (getVerticeWeightval(i,j,k1)>0?16:0):0;
		// ret |= getVerticeTsdfval(i1,j,k1)>_isoleval? (getVerticeWeightval(i1,j,k1)>0?32:0):0;
		// ret |= getVerticeTsdfval(i1,j1,k1)>_isoleval? (getVerticeWeightval(i1,j1,k1)>0?64:0):0;
		// ret |= getVerticeTsdfval(i,j1,k1) >_isoleval? (getVerticeWeightval(i,j1,k1) >0?128:0):0;
		ret |= getVerticeTsdfval(i,j,k)>_isoleval?1:0;
		ret |= getVerticeTsdfval(i1,j,k)>_isoleval?2:0;
		ret |= getVerticeTsdfval(i1,j1,k)>_isoleval?4:0;
		ret |= getVerticeTsdfval(i,j1,k)>_isoleval?8:0;
		ret |= getVerticeTsdfval(i,j,k1)>_isoleval?16:0;
		ret |= getVerticeTsdfval(i1,j,k1)>_isoleval?32:0;
		ret |= getVerticeTsdfval(i1,j1,k1)>_isoleval?64:0;
		ret |= getVerticeTsdfval(i,j1,k1) >_isoleval?128:0;
		return ret;
	}
	inline int yuMCubes_t::calEdgeTabIndex(index3d_t &index)
	{
		return calEdgeTabIndex(index._x,index._y,index._z);
	}

	inline int yuMCubes_t::calInterpVertex(int i,int j,int k,int i1,int j1,int k1, point3d_t &p)
	{
		point3d_t p1,p2;
		index3d2point3d(p1,index3d_t(i,j,k));
		index3d2point3d(p2,index3d_t(i1,j1,k1));
		double v1 = getVerticeTsdfval(i,j,k);
		double v2 = getVerticeTsdfval(i1,j1,k1);
		if(abs(v1-_isoleval)<_scale._zscale/20)
		{
			p = p1;
			return 0;
		}
		if(abs(v2-_isoleval)<_scale._zscale/20)
		{
			p = p2;
			return 0;
		}
		if(abs(v2-v1)<_scale._zscale/20)
		{
			p-p1;
			return 0;
		}
		double t = (_isoleval - v1)/(v2-v1);
		p = p1+(p2-p1)*t;
		return 0;
	}
	
}

