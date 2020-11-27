#include "opencv2/highgui/highgui.hpp"
#include "opencv2/core/core.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include<iostream>
using namespace std;
using namespace cv;
#include<vector>
#include<bits/stdc++.h>
#include "opencv2/calib3d/calib3d.hpp"

#include <unistd.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <termios.h>

int fd;
void settings(const char *abc)
{
  	fd = open(abc,O_RDWR | O_NOCTTY); /* ttyUSB0 is the FT232 based USB2SERIAL Converter   */
  	usleep(3500000);
                                	/* O_RDWR Read/Write access to serial port       	*/
                                	/* O_NOCTTY - No terminal will control the process   */
                                	/* O_NDELAY -Non Blocking Mode,Does not care about-  */
                                	/* -the status of DCD line,Open() returns immediatly */

        	if(fd == -1)                    	/* Error Checking */
               	printf("\n  Error! in Opening ttyUSB0  ");
        	else
               	printf("\n  ttyUSB0 Opened Successfully ");
   	struct termios toptions;     	/* get current serial port settings */
   	tcgetattr(fd, &toptions);    	/* set 9600 baud both ways */
   	cfsetispeed(&toptions, B9600);
   	cfsetospeed(&toptions, B9600);   /* 8 bits, no parity, no stop bits */
   	toptions.c_cflag &= ~PARENB;
   	toptions.c_cflag &= ~CSTOPB;
   	toptions.c_cflag &= ~CSIZE;
   	toptions.c_cflag |= CS8;     	/* Canonical mode */
   	toptions.c_lflag |= ICANON;   	/* commit the serial port settings */
   	tcsetattr(fd, TCSANOW, &toptions);
}
void sendCommand(const char *abc)
{
   write(fd, abc, 1);
}
/*vector <Point2f> a1,b1;
void mouse(int event,int x,int y,int,void*)
{
  if  ( event == EVENT_LBUTTONDOWN )
    a1.push_back(Point(x,y));
  if( event == EVENT_RBUTTONDOWN )
    b1.push_back(Point(x,y));*/
//}


int main(int ,char **)

{
	VideoCapture cap(0);
		if(!cap.isOpened())
		return -1;

	Mat img;
	cap>>img;


	cvtColor(img,img,COLOR_BGR2YCrCb);
	int i,j,rmin=0,bmin=0,gmin=0,rmax=255,bmax=255,gmax=255,can=729;\
  int align=0;

	namedWindow("window1",WINDOW_NORMAL);
	namedWindow("window2",WINDOW_NORMAL);
	namedWindow("window3",WINDOW_NORMAL);
  namedWindow("window4",WINDOW_NORMAL);
	createTrackbar("can","window2",&can,1000);
	createTrackbar("rmin","window1",&rmin,255);
	createTrackbar("rmax","window1",&rmax,255);
	createTrackbar("bmin","window1",&bmin,255);
	createTrackbar("bmax","window1",&bmax,255);
	createTrackbar("gmin","window1",&gmin,255);
	createTrackbar("gmax","window1",&gmax,255);

	createTrackbar("align","window4",&align,img.cols);

	while(1)
	{	Mat img;
		cap>>img;
		Mat a(img.rows,img.cols,CV_8UC1,Scalar(255));
		Mat q(img.rows,img.cols,CV_8UC3,Scalar(255,255,255));
		for(i=0;i<img.rows;++i)
		{
			for(j=0;j<img.cols;++j)
			{
				if(img.at<Vec3b>(i,j)[0]>bmin&&img.at<Vec3b>(i,j)[1]>gmin&&img.at<Vec3b>(i,j)[2]>rmin&&img.at<Vec3b>(i,j)[0]<bmax&&img.at<Vec3b>(i,j)[1]<gmax&&img.at<Vec3b>(i,j)[2]<rmax)
				a.at<uchar>(i,j) = 255;
				else
					a.at<uchar>(i,j) = 0;

			}
		}

		for(i=0;i<img.rows;++i)
		{
			for(j=0;j<img.cols;++j)
			{
				if(img.at<Vec3b>(i,j)[0]>bmin&&img.at<Vec3b>(i,j)[1]>gmin&&img.at<Vec3b>(i,j)[2]>rmin&&img.at<Vec3b>(i,j)[0]<bmax&&img.at<Vec3b>(i,j)[1]<gmax&&img.at<Vec3b>(i,j)[2]<rmax)
				q.at<Vec3b>(i,j)=img.at<Vec3b>(i,j);
				else
					q.at<Vec3b>(i,j)=0;

			}
		}

Mat l(img.rows,img.cols,CV_8UC3,Scalar(0,0,0));
    vector<Point2f>c1;
    c1.push_back(Point2f(0,0));
    c1.push_back(Point2f(img.cols, 0));
    c1.push_back(Point2f(img.cols, img.rows));
    c1.push_back(Point2f(0,img.rows));

    //setMouseCallback("window3",mouse,0);

  vector<Point2f>a1;
    a1.push_back(Point2f(0,0));
    a1.push_back(Point2f(img.cols,0));
    a1.push_back(Point2f(img.cols-align,img.rows));
    a1.push_back(Point2f(align,img.rows));

  if (a1.size()==4&& c1.size()==4)
  {
    Mat h = findHomography(c1,a1);

      warpPerspective(q,l,h,l.size());}

      imshow("window3",q);
      waitKey(1);
      imshow("window4",l);
		  waitKey(1);



      Mat   src_gray;
      Mat src(img.rows/2,img.cols,CV_8UC3,scalar(0,0,0));

      for(int i=img.rows/2;i<img.rows;i++)
      {
        for(int j = 0; j<img.cols;i++)
        {
          src.at<Vec3b>(i-img.rows/2,j)=l.at<Vec3b>(i,j)
        }
      }
      Mat circleimage(img.rows/2,img.cols,CV_8UC3,scalar(0,0,0));
      /// Convert it to gray
  cvtColor( src, src_gray, CV_BGR2GRAY );

  /// Reduce the noise so we avoid false circle detection
  GaussianBlur( src_gray, src_gray, Size(9, 9), 2, 2 );

  vector<Vec3f> circles;

  /// Apply the Hough Transform to find the circles
  HoughCircles( src_gray, circles, CV_HOUGH_GRADIENT, 1, src_gray.rows/8, 200, 100, 0, 0 );

  /// Draw the circles detected
  for( size_t i = 0; i < circles.size(); i++ )
  {
      Point center(cvRound(circles[i][0]), cvRound(circles[i][1]));
      int radius = cvRound(circles[i][2]);
      // circle center
      circle( circleimage, center, 3, Scalar(0,0,0), -1, 8, 0 );
      // circle outline
      circle( circleimage, center, radius, Scalar(255,255,255), 3, 8, 0 );
   }

  /// Show your results
  namedWindow( "Hough Circle Transform Demo", CV_WINDOW_AUTOSIZE );
  imshow( "Hough Circle Transform Demo", circleimage);

  waitKey(1);




		Mat b(a.rows,a.cols,CV_8UC1,Scalar(0));

		//BINARYIMAGE
	for(int i=0;i<a.rows;i++)
	{
		for(int j=0;j<a.cols;j++)
		{
			if(a.at<uchar>(i,j)>=127)
				b.at<uchar>(i,j)=255;
			else
				b.at<uchar>(i,j)=0;
		}
	}

	//EROSION AND DILATION
	Mat c(b.rows,b.cols,CV_8UC1,Scalar(255));
	Mat element=getStructuringElement(MORPH_RECT,Size(7,7),Point(-1,-1));
	erode(b,c,element);
	dilate(c,c,element);

	b=c.clone();



	Canny(b,c,can,2*can,3);

	vector<vector<Point>>contours;
	vector<Vec4i> hierarchy;

	findContours(c,contours,hierarchy,CV_RETR_TREE,CV_CHAIN_APPROX_SIMPLE,Point(0,0));

	int i=0;
	double area;
	while(i<contours.size()&&contours.empty()!=1)
	{

		if(area<contourArea(contours[i]));
			{area=contourArea(contours[i]);
			j=i;

			}

		i++;

	}

	drawContours(c,contours,j,255,2,8);

	if(contours.empty()!=1)
	{
	Moments m = moments(c,true);
	Point p(m.m10/m.m00, m.m01/m.m00);

	int sight1=(c.rows*c.cols)/5;
	int sight2=(c.rows*c.cols)/15;


	if(area<sight1)
	{

		sendCommand("W");
	}
	else
	if(area>=sight2)
	{
	sendCommand("S");}
	else
	if(area<sight2&&area>=sight1)
	{if(p.x<a.cols/3)

		{sendCommand("A");

	}

	else
	if(p.x<a.cols*2/3)
		sendCommand("W");
	else
	if(p.x>a.cols*2/3)
	sendCommand("D");

	}

	}


	imshow("window2",c);
	imshow("window1",b);
	waitKey(1);



	}
}
