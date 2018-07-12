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

namespace sdf
{
	class yuMCubes_t :public sdf_t
	{
	public:
		yuMCubes_t(){}
		~yuMCubes_t(){}


		int setIsoleval(const double &isoleval);
		int setThreshold(const double &threshold);
		double getIsoleval() const;
		double getThreshold() const;
	protected:
		double _isoleval;
		double _threshold;
		
	};
}


#endif
