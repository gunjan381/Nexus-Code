#include "opencv2/highgui/highgui.hpp"
#include "opencv2/core/core.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/opencv.hpp"
#include<iostream>
using namespace std;
using namespace cv;
#include<vector>
#include <unistd.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <termios.h>
#include <math.h>

int main()
{
	

	int allign=0;
	
	namedWindow("window1",WINDOW_NORMAL);
	namedWindow("window2",WINDOW_NORMAL);
	namedWindow("window3",WINDOW_NORMAL);
	namedWindow("window4",WINDOW_NORMAL);
	namedWindow("window5",WINDOW_NORMAL);

	
		Mat a=imread("bar.jpg",1);
		Mat b(a.rows,a.cols,CV_8UC3,Scalar(0,0,0));
		Mat c(a.rows,a.cols,CV_8UC1,Scalar(0));
		Mat squareimage(a.rows,a.cols,CV_8UC1,Scalar(0));
		Mat d(a.rows,a.cols,CV_8UC1,Scalar(0));
		int count=0;

   		cvtColor(a,b,CV_BGR2GRAY);
   		Mat element=getStructuringElement(MORPH_RECT,Size(3,3),Point(1,1));
   		erode(b,b,element);
		dilate(b,b,element);
		dilate(b,b,element);
		erode(b,b,element);
   		GaussianBlur(b,b,Size(5,5),0,255,0);
   		threshold(b,b,50,255,0);
   		Canny(b,c,20,100,3);

   		vector<vector<Point>> contours;
		vector<Vec4i> hierarchy;
   		findContours(c,contours,hierarchy,CV_RETR_TREE,CV_CHAIN_APPROX_SIMPLE,Point(0,0));

   		if(contours.empty()!=1)
  		{
      	    vector<vector<Point> > contours_poly( contours.size() );
    	    vector<Rect> boundRect( contours.size() );
      		vector<Vec2f>sides;
      int array[contours.size()];		
      for( size_t i = 0; i< contours.size(); i++ )
      { 
      	approxPolyDP( contours[i], contours_poly[i], arcLength(Mat(contours[i]), true)*0.01, true );
      	array[i]=0;
      	count++;
        //boundRect[i] = boundingRect( contours_poly[i] );
      }
      for(int j=0;j<contours_poly.size();j++)
      if (contours_poly[j].size()==4)
      {
        drawContours(squareimage, contours_poly,j,Scalar(255),1,8);
        array[j]=1;
      }
      if(count>1)
      {
      	int lar;
      	int area=0;
      	printf("%d\n",contours_poly.size() );
			for( int i=0;i<contours_poly.size();i++)
			{
				
				if(contourArea(contours_poly[i],false)>area)
				{
					lar=i;
					area=contourArea(contours_poly[i],false);
				}
			}
		Point min1=Point(a.cols,0),min2=Point(a.cols,0),max1=Point(0,0),max2=Point(0,0);
		int c,v;
		for( int i=0;i<4;i++)
		{
			printf("%d\n",contours_poly[lar][i].x );
			if(contours_poly[lar][i].x<min1.x)
			{
				c=i;
				min1=contours_poly[lar][i];
			}
			
			if(contours_poly[lar][i].x>max1.x)
			{
				v=i;
				max1=contours_poly[lar][i];
			}
			
		}

		for( int i=0;i<4;i++)
		{
			if(contours_poly[lar][i].x<min2.x && i!=c)
				min2=contours_poly[lar][i];
			if(contours_poly[lar][i].x>max2.x && i!=v)
				max2=contours_poly[lar][i];
		}
		Point swap;
		if(min1.y>min2.y)
		{
			swap=min1;
			min1=min2;
			min2=swap;
		}
		if(max1.y>max2.y)
		{
			swap=max1;
			max1=max2;
			max2=swap;
		}
		vector<Point2f> pts_a;
		vector<Point2f> pts_b;
		printf("%d %d\n", max1.x,max1.y);
		printf("%d %d\n", max2.x,max2.y);
		printf("%d %d\n", min1.x,min1.y);
		printf("%d %d\n", min2.x,min2.y);
		printf("%d \n", contours_poly[lar].size());
		pts_a.push_back(min1);
			pts_a.push_back(max1);
			pts_a.push_back(min2);
			pts_a.push_back(max2);
			pts_b.push_back(Point2f(0, 0));
	    	pts_b.push_back(Point2f(a.cols,0));
   			pts_b.push_back(Point2f(0, a.rows-1));
   			pts_b.push_back(Point2f(a.cols-1, a.rows-1));
   			Mat h = findHomography(pts_a, pts_b);
   			warpPerspective(b, d, h, b.size());
   		vector<int> location;
   			int loc;
   			if(d.at<uchar>(0,a.rows/2)==255)
	   		{
   				loc=0;
   				location.push_back(0);
   			}
	   		else
   				loc=1;
   			for (int m = 1; m < a.cols; ++m)
   			{
   				if(loc==0 && d.at<uchar>(m,a.rows/2)==0)
   				{
   					loc=1;
   					location.push_back(m);
	   			}
   				if(loc==1 && d.at<uchar>(m,a.rows/2)==255)
   				{
   					loc==0;
   					location.push_back(m);
   				}
   			}
   			int j=1;
	   		vector<int> number;
   			for (int m = 1; m < location.size(); ++m)
   			{
   				if(fabs(location[m]-location[m-j])<a.cols/20)
   					j++;
   				else if(fabs(location[m]-location[m-j])<3*a.cols/20)
   				{	
   					number.push_back(0);
	   				j=1;
   				}
   				else if(fabs(location[m]-location[m-j])<5*a.cols/20)
   				{
   					j=1;
	   				number.push_back(1);
   				}
   			}
   				if(number.empty())
   					printf("no bar code detected\n"); 	
	   			else
	   			for(int m=0;m<number.size();m++)	
	   				printf("%d\n",number[m]);	

  		}
  	}
  	imshow("window1",a);
	//waitKey(0);
	imshow("window2",b);
	//waitKey(0);
	imshow("window5",c);
	//waitKey(0);
	imshow("window3",squareimage);
 	//waitKey(0);
	imshow("window4",d);
	waitKey(0);
  	}