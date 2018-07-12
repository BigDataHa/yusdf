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

	
}

