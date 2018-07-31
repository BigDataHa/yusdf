/*************************************************************************
	> File Name: yuMarchingCubes.h
	> Author: yusnows
	> Mail: YusnowsLiang@gmail.com
	> Created Time: Wed 11 Jul 2018 06:09:30 PM CST
    > Copyright@yusnows: All rights reserve!
 ************************************************************************/

#ifndef _YUMARCHINGCUBES_H
#define _YUMARCHINGCUBES_H
#include "mcTables.h"
#include "../tsdf/sdf.h"
#include <vector>

namespace sdf
{
	class yuMCubes_t :public sdf_t
	{
	public:
		yuMCubes_t():_isoleval(0),_threshold(0)
		{}
		~yuMCubes_t(){}


		
		int saveIsoPointCloud(const char *filename);

		int calIsoPointCloud(bool recal);
		int setIsoleval(const double &isoleval);
		int setThreshold(const double &threshold);
		double getIsoleval() const;
		double getThreshold() const;

	protected:
		inline int calEdgeTabIndex(int i,int j, int k);
		inline int calEdgeTabIndex(index3d_t &index);
		inline int calInterpVertex(int i,int j,int k,int i1,int j1,int k1, point3d_t &p);

	private:
		inline int min(int a, int b)
		{
			return ((a>b)?b:a);
		}

	protected:
		double _isoleval;
		double _threshold;
		point3d_t _verticeList[12];
		std::vector<point3d_t> _isoPointCloud;
	};
}


#endif
