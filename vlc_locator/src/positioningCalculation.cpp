//-----------------------------------【头文件包含部分】---------------------------------------  
//      描述：包含程序所依赖的头文件
//----------------------------------------------------------------------------------------------  
#include <positioningCalculation.hpp> 

//-----------------------------------------------------------------------------------------------
//**********************************************************************************************
//
//      *********************             【定位计算函数定义部分】              *******************
//
//**********************************************************************************************
//-----------------------------------------------------------------------------------------------


struct XYZ double_LED(double f,double Center_X, double Center_Y, struct LED A,struct LED B)
{
	double ImgX1;
	double ImgY1;
	double ImgX2;
	double ImgY2;
	double x1;
	double y1;
	double x2;
	double y2;

	if (A.Y>B.Y){
		ImgX1 = A.imgLocalX;
		ImgY1 = A.imgLocalY;
		ImgX2 = B.imgLocalX;
		ImgY2 = B.imgLocalY;
		x1 = A.X;
		y1 = A.Y;
		x2 = B.X;
		y2 = B.Y;
	}

	else
	{
		ImgX1 = B.imgLocalX;
		ImgY1 = B.imgLocalY;
		ImgX2 = A.imgLocalX;
		ImgY2 = A.imgLocalY;
		x1 = B.X;
		y1 = B.Y;
		x2 = A.X;
		y2 = A.Y;
	}

	double alpha;

	if (x1>x2){
		alpha = -(3*pi/4);
	}

	else
	{
		alpha = -(pi/4);
	}
	cout << "alpha=" << alpha << '\n';


	double d_12 = sqrt(pow((ImgX1 - ImgX2),2) + pow((ImgY1 - ImgY2),2))*3.2e-3;
	double D_12 = sqrt(pow((x1 - x2),2) + pow((y1 - y2),2));
	double H = D_12 / d_12*f;
	double X_r = ((ImgX1 + ImgX2) / 2 - Center_X)*3.2e-3*H / f;
	double Y_r = ((ImgY1 + ImgY2) / 2 - Center_Y)*3.2e-3*H / f;
	double X_c = (x1 + x2) / 2;
	double Y_c = (y1 + y2) / 2;
	double X = X_r;
	double Y = Y_r;

	// cout << "H=" << H << '\n';

	// 计算角度
	// cout << "ImgY2=" << ImgY2 << '\n';
	// cout << "ImgY1=" << ImgY1 << '\n';
	// cout << "ImgX2 =" << ImgX2 << '\n';
	// cout << "ImgX1=" << ImgX1 << '\n';
	double K1 = abs((ImgY2 - ImgY1) / (ImgX2 - ImgX1));
	// cout << "K1=" << K1  << '\n';
	double angle = atan(K1);
	// cout << "angle1=" << angle / pi * 180 << '\n';

	//由于对称性，要对角度做进一步处理
	bool ABC = ImgY2 < ImgY1;
	bool EFG = ImgX2 > ImgX1;
	int ABCD = ABC * 2 + EFG;
	// ABCD = 3;
	// cout << "ABCD=" << ABCD << '\n';

	switch (ABCD)
	{
	case 0:
		angle = angle + alpha;
		break;
	case 1:
		angle = pi - angle + alpha;
		break;
	case 2:
		angle = 2 * pi - angle + alpha;
		break;
	case 3:
		angle = angle + pi + alpha;
		break;
	}
	// cout << "angle=" << angle / pi * 180 << '\n';
		
	double XX = X*cos(angle) - Y*sin(angle);
	double YY = X*sin(angle) + Y*cos(angle);

	XX = XX + X_c;
	YY = YY + Y_c;

	double xx = XX / 10;
	double yy = YY / 10;
	double zz = 150 - H / 10;

	// imshow("test time", grayImage);

	struct XYZ pose;
	pose.x=xx;
	pose.y=yy;
	pose.z=zz;

}

struct XYZ three_LED(double f, double Center_X, double Center_Y, struct LED A,struct LED B,struct LED C)
{
	double ImgX1 = A.imgLocalX;
	double ImgY1 = A.imgLocalY;
	double ImgX2 = B.imgLocalX;
	double ImgY2 = B.imgLocalY;
	double ImgX3 = C.imgLocalX;
	double ImgY3 = C.imgLocalY;
	double x1 = A.X;
	double y1 = A.Y;
	double x2 = B.X;
	double y2 = B.Y;
	double x3 = C.X;
	double y3 = C.Y;

	//三灯定位
	double d_12 = sqrt(pow((ImgX1 - ImgX2), 2) + pow((ImgY1 - ImgY2), 2))*3.2e-3;
	double d_13 = sqrt(pow((ImgX1 - ImgX3), 2) + pow((ImgY1 - ImgY3), 2))*3.2e-3;
	double d_23 = sqrt(pow((ImgX2 - ImgX3), 2) + pow((ImgY2 - ImgY3), 2))*3.2e-3;
	double D_12 = sqrt(pow((x1 - x2), 2) + pow((y1 - y2), 2));
	double D_13 = sqrt(pow((x1 - x3), 2) + pow((y1 - y3), 2));
	double D_23 = sqrt(pow((x2 - x3), 2) + pow((y2 - y3), 2));
	double H = (D_12 / d_12*f + D_13 / d_13*f + D_23 / d_23*f) / 3;//计算出高度
	// cout << "H=" << H << '\n';

	//计算水平方向上摄像头到3个LED的距离
	double d_1 = sqrt(pow((ImgX1 - Center_X), 2) + pow((ImgY1 - Center_Y), 2))*3.2e-3;
	double d_2 = sqrt(pow((ImgX2 - Center_X), 2) + pow((ImgY2 - Center_Y), 2))*3.2e-3;
	double d_3 = sqrt(pow((ImgX3 - Center_X), 2) + pow((ImgY3 - Center_Y), 2))*3.2e-3;

	//对应真实的距离
	double D_1 = H / f*d_1;
	double D_2 = H / f*d_2;
	double D_3 = H / f*d_3;

	double r1 = pow(D_1, 2);
	double r2 = pow(D_2, 2);
	double r3 = pow(D_3, 2);

	// double rr1 = pow(d_1, 2);
	// double rr2 = pow(d_2, 2);
	// double rr3 = pow(d_3, 2);

	//解出终端的位置坐标
	double a1 = 2 * (x1 - x3);
	double b1 = 2 * (y1 - y3);
	double c1 = pow(x3, 2) - pow(x1, 2) + pow(y3, 2) - pow(y1, 2) - r3 + r1;
	double a2 = 2 * (x2 - x3);
	double b2 = 2 * (y2 - y3);
	double c2 = pow(x3, 2) - pow(x2, 2) + pow(y3, 2) - pow(y2, 2) - r3 + r2;

	// double a1 = 2 * (ImgX1 - ImgX3);
	// double b1 = 2 * (ImgY1 - ImgY3);
	// double c1 = pow(ImgX3, 2) - pow(ImgX1, 2) + pow(ImgY3, 2) - pow(ImgY1, 2) - rr3 + rr1;
	// double a2 = 2 * (ImgX2 - ImgX3);
	// double b2 = 2 * (ImgY2 - ImgY3);
	// double c2 = pow(ImgX3, 2) - pow(ImgX2, 2) + pow(ImgY3, 2) - pow(ImgY2, 2) - rr3 + rr2;

	double XX = (c2 * b1 - c1 * b2) / (a1*b2 - a2 * b1);
	double YY = (c2 * a1 - c1 * a2) / (a2*b1 - a1 * b2);

	double xx = XX / 10;
	double yy = YY / 10;
	// xx = xx*(f / H);
	// yy = xx*(f / H);
	double zz = 150 - H / 10;

	struct XYZ pose;
	pose.x=xx;
	pose.y=yy;
	pose.z=zz;
	return pose;

}

