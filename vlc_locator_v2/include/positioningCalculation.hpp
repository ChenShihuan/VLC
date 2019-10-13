#ifndef positioningCalculation_hpp_
#define positioningCalculation_hpp_

#include <vlcCommonInclude.hpp>
	
// struct position{// LED的位置，对应不同位置的灯具
// 	int max;	// ID_max,最大条纹数目 	
// 	int min;	// ID_min，最小条纹数目
// 	double X;	// LED灯具的真实位置,x坐标
// 	double Y;	// LED灯具的真实位置,y坐标
// 	};

struct XYZ double_LED(double f,double Center_X, double Center_Y, struct LED A,struct LED B);
struct XYZ three_LED(double f, double Center_X, double Center_Y, struct LED A,struct LED B,struct LED C);

#endif
