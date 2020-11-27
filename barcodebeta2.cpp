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
#include <chrono>
using namespace std::chrono;
int fd;
void settings(const char *abc)
{
      fd = open(abc,O_RDWR | O_NOCTTY); /* ttyUSB0 is the FT232 based USB2SERIAL Converter   */
      usleep(3500000);
                                    /* O_RDWR Read/Write access to serial port           */
                                    /* O_NOCTTY - No terminal will control the process   */
                                    /* O_NDELAY -Non Blocking Mode,Does not care about-  */
                                    /* -the status of DCD line,Open() returns immediatly */

            if(fd == -1)                        /* Error Checking */
                   printf("\n  Error! in Opening ttyUSB0  ");
            else
                   printf("\n  ttyUSB0 Opened Successfully ");
       struct termios toptions;         /* get current serial port settings */
       tcgetattr(fd, &toptions);        /* set 9600 baud both ways */
       cfsetispeed(&toptions, B9600);
       cfsetospeed(&toptions, B9600);   /* 8 bits, no parity, no stop bits */
       toptions.c_cflag &= ~PARENB;
       toptions.c_cflag &= ~CSTOPB;
       toptions.c_cflag &= ~CSIZE;
       toptions.c_cflag |= CS8;         /* Canonical mode */
       toptions.c_lflag |= ICANON;       /* commit the serial port settings */
       tcsetattr(fd, TCSANOW, &toptions);
}
void sendCommand(const char *abc)
{
   write(fd, abc, 1);
}

int main()
{

	//settings("/dev/ttyACM0");
	VideoCapture cap(0);
		if(!cap.isOpened())
		return -1;

		Mat IMAGE;
	  cap>>IMAGE;

	int can=0;
	int rCbMin=0,ryMin=0,rCrMin=0,rCbMax=255,ryMax=255,rCrMax=255;
	int gCbMin=0,gyMin=0,gCrMin=0,gCbMax=255,gyMax=255,gCrMax=255;
	int bCbMin=0,byMin=0,bCrMin=0,bCbMax=255,byMax=255,bCrMax=255;

	int align=239;
	int threshold_binary=50;
	int sum=0;

	namedWindow("window1",WINDOW_NORMAL);
	namedWindow("window2",WINDOW_NORMAL);
	namedWindow("window3",WINDOW_NORMAL);
	namedWindow("window4",WINDOW_NORMAL);
	namedWindow("window5",WINDOW_NORMAL);

	namedWindow("Green Circle",WINDOW_NORMAL);
	namedWindow("Red Circle",WINDOW_NORMAL);
	namedWindow("Blue Circle",WINDOW_NORMAL);
	namedWindow("Bar Code",WINDOW_NORMAL);

	namedWindow("TriangleWindow",CV_WINDOW_NORMAL);
	namedWindow("SquareWindow",CV_WINDOW_NORMAL);

	createTrackbar("threshold","barcode",&threshold_binary,255);
	createTrackbar("align","Bar Code",&align,IMAGE.cols/2);

	createTrackbar("CbMin","Green Circle",&gCbMin,255);
	createTrackbar("CbMax","Green Circle",&gCbMax,255);
	createTrackbar("yMin","Green Circle",&gyMin,255);
	createTrackbar("yMax","Green Circle",&gyMax,255);
	createTrackbar("CrMin","Green Circle",&gCrMin,255);
	createTrackbar("CrMax","Green Circle",&gCrMax,255);

	createTrackbar("CbMin","Red Circle",&rCbMin,255);
	createTrackbar("CbMax","Red Circle",&rCbMax,255);
	createTrackbar("yMin","Red Circle",&ryMin,255);
	createTrackbar("yMax","Red Circle",&ryMax,255);
	createTrackbar("CrMin","Red Circle",&rCrMin,255);
	createTrackbar("CrMax","Red Circle",&rCrMax,255);

	createTrackbar("CbMin","Blue Circle",&bCbMin,255);
	createTrackbar("CbMax","Blue Circle",&bCbMax,255);
	createTrackbar("yMin","Blue Circle",&byMin,255);
	createTrackbar("yMax","Blue Circle",&byMax,255);
	createTrackbar("CrMin","Blue Circle",&bCrMin,255);
	createTrackbar("CrMax","Blue Circle",&bCrMax,255);

	auto time1 = high_resolution_clock::now();
	auto time2 = high_resolution_clock::now();
	auto time3 = high_resolution_clock::now();
	auto time4 = high_resolution_clock::now();
while(1)
{
	cap>>IMAGE;
	char send;


	//COORDINATES TO BE WARPED
	vector<Point2f>c1;
		c1.push_back(Point2f(0,0));
		c1.push_back(Point2f(IMAGE.cols, 0));
		c1.push_back(Point2f(IMAGE.cols, IMAGE.rows));
		c1.push_back(Point2f(0,IMAGE.rows));

	//FINAL COORDINATES
	vector<Point2f>a1;
		a1.push_back(Point2f(0,0));
		a1.push_back(Point2f(IMAGE.cols,0));
		a1.push_back(Point2f(IMAGE.cols-align,IMAGE.rows));
		a1.push_back(Point2f(align,IMAGE.rows));

	//HOMOGRAPHY MATRIX
		Mat h = findHomography(c1,a1);
		Mat tempo;
		warpPerspective(IMAGE,tempo,h,IMAGE.size());
		IMAGE=tempo.clone();

	//PERSPECTIVE LIMITING
		Mat temp(IMAGE.rows/2, IMAGE.cols ,CV_8UC3,Scalar(0,0,0));

		for(int i= IMAGE.rows/2 ; i<IMAGE.rows ; i++)
		{
			for(int j = 0; j<IMAGE.cols ; j++)
			{
				temp.at<Vec3b>(i-IMAGE.rows/2,j)=IMAGE.at<Vec3b>(i,j);
			}
		}
	Mat img=temp.clone();

	Mat bar_code=img.clone();
	cvtColor(bar_code,bar_code,CV_BGR2GRAY);


	cvtColor(img,img,COLOR_BGR2YCrCb);
	Mat green_circle(img.rows,img.cols,CV_8UC1,Scalar(255));
  Mat blue_circle(img.rows,img.cols,CV_8UC1,Scalar(255));
  Mat red_circle(img.rows,img.cols,CV_8UC1,Scalar(255));

//GREEN CIRCLE
  for(int i=0;i<img.rows;++i)
  {
    for(int j=0;j<img.cols;++j)
    {
      if(img.at<Vec3b>(i,j)[0]>=gyMin&&img.at<Vec3b>(i,j)[1]>=gCrMin&&img.at<Vec3b>(i,j)[2]>=gCbMin&&img.at<Vec3b>(i,j)[0]<=gyMax&&img.at<Vec3b>(i,j)[1]<=gCrMax&&img.at<Vec3b>(i,j)[2]<=gCbMax)
        green_circle.at<uchar>(i,j)=255;
      else
        green_circle.at<uchar>(i,j)=0;
    }
  }

//RED CIRCLE
  for(int i=0;i<img.rows;++i)
  {
    for(int j=0;j<img.cols;++j)
    {
      if(img.at<Vec3b>(i,j)[0]>=ryMin&&img.at<Vec3b>(i,j)[1]>=rCrMin&&img.at<Vec3b>(i,j)[2]>=rCbMin&&img.at<Vec3b>(i,j)[0]<=ryMax&&img.at<Vec3b>(i,j)[1]<=rCrMax&&img.at<Vec3b>(i,j)[2]<=rCbMax)
        red_circle.at<uchar>(i,j)=255;
      else
        red_circle.at<uchar>(i,j)=0;
    }
  }
	//BLUE CIRCLE
	  for(int i=0;i<img.rows;++i)
	  {
	    for(int j=0;j<img.cols;++j)
	    {
	      if(img.at<Vec3b>(i,j)[0]>=byMin&&img.at<Vec3b>(i,j)[1]>=bCrMin&&img.at<Vec3b>(i,j)[2]>=bCbMin&&img.at<Vec3b>(i,j)[0]<=byMax&&img.at<Vec3b>(i,j)[1]<=bCrMax&&img.at<Vec3b>(i,j)[2]<=bCbMax)
	        blue_circle.at<uchar>(i,j)=255;
	      else
	        blue_circle.at<uchar>(i,j)=0;
	    }
	  }

	imshow("Green Circle",green_circle);
	waitKey(1);
	imshow("Red Circle",red_circle);
	waitKey(1);
	imshow("Blue Circle",blue_circle);
	waitKey(1);
	imshow("Bar Code",bar_code);
	waitKey(1);

	Mat element=getStructuringElement(MORPH_RECT,Size(3,3),Point(-1,-1));
	//NOISE REMOVED FROM GREEN CIRCLE IMAGE
	erode(green_circle,green_circle,element);
	dilate(green_circle,green_circle,element);
	dilate(green_circle,green_circle,element);
	erode(green_circle,green_circle,element);

	//NOISE REMOVED FROM RED CIRCLE IMAGE
	erode(red_circle,red_circle,element);
	dilate(red_circle,red_circle,element);
	dilate(red_circle,red_circle,element);
	erode(red_circle,red_circle,element);


	//NOISE REMOVED FROM BLUE CIRCLE IMAGE
	erode(blue_circle,blue_circle,element);
	dilate(blue_circle,blue_circle,element);
	dilate(blue_circle,blue_circle,element);
	erode(blue_circle,blue_circle,element);

	//NOISE REMOVED FROM BARCODE IMAGE
	erode(bar_code,bar_code,element);
	dilate(bar_code,bar_code,element);
	dilate(bar_code,bar_code,element);
	erode(bar_code,bar_code,element);


	GaussianBlur(green_circle,green_circle,Size(9, 9),2,2);
  GaussianBlur(red_circle,red_circle,Size(9, 9),2,2);
  GaussianBlur(blue_circle,blue_circle,Size(9, 9),2,2);
	GaussianBlur(bar_code,bar_code,Size(5,5),0,255,0);

	//BINARY IMAGE OF BARCODE
	threshold(bar_code,bar_code,threshold_binary,255,THRESH_BINARY);


	Mat green_contour_image;
	Mat red_contour_image;
	Mat blue_contour_image;
	Mat barcode_contour_image;

	Canny(green_circle,green_circle,can,2*can,3);
	Canny(red_circle,red_circle,can,2*can,3);
	Canny(blue_circle,blue_circle,can,2*can,3);
	Canny(bar_code,bar_code,can,2*can,3);

	vector<vector<Point>>greencontours,redcontours,bluecontours,contours;
	vector<Vec4i>greenhierarchy,redhierarchy,bluehierarchy,hierarchy;

	findContours(bar_code,contours,hierarchy,CV_RETR_TREE,CV_CHAIN_APPROX_SIMPLE,Point(0,0));
	findContours(green_circle,greencontours,greenhierarchy,CV_RETR_TREE,CV_CHAIN_APPROX_SIMPLE,Point(0,0));
	findContours(red_circle,redcontours,redhierarchy,CV_RETR_TREE,CV_CHAIN_APPROX_SIMPLE,Point(0,0));
	findContours(blue_circle,bluecontours,bluehierarchy,CV_RETR_TREE,CV_CHAIN_APPROX_SIMPLE,Point(0,0));

	Point2f greencenter,redcenter,bluecenter;
	greencenter.x=-1;
	greencenter.y=-1;
	redcenter.x=-1;
	redcenter.y=-1;
	bluecenter.x=-1;
	bluecenter.y=-1;

	//GREEN CIRCLES DRAWING
	if(greencontours.empty()!=1)
	{
    vector<vector<Point> > ccontours_poly( greencontours.size() );
    vector<Point2f>centers( greencontours.size() );
    vector<float>radius( greencontours.size() );

    for( size_t i = 0; i< greencontours.size(); i++ )
    {   approxPolyDP( greencontours[i], ccontours_poly[i], arcLength(Mat(greencontours[i]), true)*0.05, true );
        minEnclosingCircle( ccontours_poly[i], centers[i], radius[i] );
    }

		int maximum=0;
		int cj;

    for(int i=0;i<greencontours.size();i++)
      {
        if(int(centers[i].y)>maximum)
          {
            maximum=centers[i].y;
            cj=i;
          }
      }
    greencenter=centers[cj];
  	//drawContours(,greencontours,cj,Scalar(255),5,8);
		circle(green_contour_image, centers[cj], (int)radius[cj], Scalar(255), 2);
	}


//RED CIRCLE DRAWING
	if(redcontours.empty()!=1)
  {
    vector<vector<Point> > ccontours_poly( redcontours.size() );
    vector<Point2f>centers( redcontours.size() );
    vector<float>radius( redcontours.size() );

    for( size_t i = 0; i< redcontours.size(); i++ )
    {   approxPolyDP( redcontours[i], ccontours_poly[i], arcLength(Mat(redcontours[i]), true)*0.05, true );
        minEnclosingCircle( ccontours_poly[i], centers[i], radius[i] );
    }

		int maximum=0;
		int cj=0;

    for(int i=0;i<redcontours.size();i++)
      {
        if(int(centers[i].y)>maximum)
          {
            maximum=centers[i].y;
            cj=i;
          }
      }
    redcenter=centers[cj];
		//drawContours(ccontourimage,redcontours,cj,Scalar(255),5,8);
		circle(red_contour_image,centers[cj],(int)radius[cj],Scalar(255), 2);

  }

//BlUE CIRCLE DRAWING
	if(bluecontours.empty()!=1)
  {
    vector<vector<Point> > ccontours_poly( bluecontours.size() );
    vector<Point2f>centers( bluecontours.size() );
    vector<float>radius( bluecontours.size() );

    for( size_t i = 0; i< bluecontours.size(); i++ )
    {   approxPolyDP( bluecontours[i], ccontours_poly[i], arcLength(Mat(bluecontours[i]), true)*0.05, true );
        minEnclosingCircle( ccontours_poly[i], centers[i], radius[i] );
    }

		int maximum=0,cj=0;
    for(int i=0;i<bluecontours.size();i++)
      {
        if(int(centers[i].y)>maximum)
          {
            maximum=centers[i].y;
            cj=i;
          }
      }
    bluecenter=centers[cj];
    //drawContours(ccontourimage,bluecontours,cj,Scalar(255),5,8);
	  circle( blue_contour_image, centers[cj], (int)radius[cj], Scalar(255), 2);

  }

//BARCODE CODE


	int check=0;
	int count=0;
	int code=-1;
	int area=0;
	Mat squareimage(img.rows,img.cols,CV_8UC1,Scalar(0));
	Mat d(img.rows,img.cols,CV_8UC1,Scalar(0));
	bool is_side_4=false;
	Moments m;

	if(contours.empty()!=1)
	{
		vector<vector<Point>>contours_poly( contours.size() );
		vector<Rect> boundRect( contours.size() );
		vector<Vec2f>sides;

		for( size_t i = 0; i< contours.size(); i++ )
		{
			approxPolyDP( contours[i], contours_poly[i], arcLength(Mat(contours[i]), true)*0.01, true );
			//boundRect[i] = boundingRect( contours_poly[i] );
	  }

		int lar;
		for( int i=0;i<contours_poly.size();i++)
		{

			if(contourArea(contours[i])>area)
			{
				lar=i;
				area=contourArea(contours[i]);
			}
		}

		if (contours_poly[lar].size()==4)
		{
			drawContours(squareimage,contours,lar,Scalar(255),1,8);
			m=moments(squareimage,true);

		}

		check=m.m01/m.m00;



		if(contours_poly[lar].size()==4&&area>18000)
		{
			Point min1=Point(img.cols,0),min2=Point(img.cols,0),max1=Point(0,0),max2=Point(0,0);
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
			pts_a.push_back(min1);
			pts_a.push_back(max1);
			pts_a.push_back(min2);
			pts_a.push_back(max2);
			pts_b.push_back(Point2f(0, 0));
			pts_b.push_back(Point2f(img.cols,0));
			pts_b.push_back(Point2f(0, img.rows-1));
			pts_b.push_back(Point2f(img.cols-1, img.rows-1));
			Mat h = findHomography(pts_a, pts_b);
			warpPerspective(squareimage, d, h, squareimage.size());
			threshold(d,d,50,255,THRESH_BINARY);
			vector<int> location;

					int loc;
					if(d.at<uchar>(img.rows/2,0)==255)
					{
						loc=0;
						location.push_back(0);
					}
					else
						loc=1;
					for (int m = 1; m < img.cols; ++m)
					{
						if(loc==0 && d.at<uchar>(img.rows/2,m)==0)
						{
							loc=1;
							location.push_back(m);
						}
						if(loc==1 && d.at<uchar>(img.rows/2,m)==255)
						{
							loc=0;
							location.push_back(m);
						}
					}
					int j=1;
					vector<int> number;
					for (int m = 1; m < location.size(); m=m+2)
					{
						if(fabs(location[m]-location[m-j])<img.cols/20)
							j++;
						else if(fabs(location[m]-location[m-j])<3*img.cols/20)
						{
							number.push_back(0);
							j=1;
						}
						else if(fabs(location[m]-location[m-j])<5*img.cols/20)
						{
							j=1;
							number.push_back(1);
						}
					}
						if(number.empty())
							printf("no bar code detected\n");
						else
						code=8*number[0]+4*number[1]+2*number[2]+number[3];
			}


	}

		Point2f center;
		int maximum=greencenter.y;
				center=greencenter;

			if(maximum<redcenter.y)
				{	maximum=redcenter.y;
					center=redcenter;
				}
			if(maximum<bluecenter.y)
			{	maximum=bluecenter.y;
				center=bluecenter;
			}
		auto duration = duration_cast<milliseconds>(high_resolution_clock::now()-time1);


		if(maximum>check&&(greencontours.empty()!=1||redcontours.empty()!=1||bluecontours.empty()!=1)&&code==-1)
		{
			cout<<"CIRCLE BELOW SQUARE "<<endl;
	    if(center.x<3*img.cols/8)
	      send='A';

	    if(center.x>5*img.cols/8)
	      send='D';
	    if(center.x>3*img.cols/8 && center.x<5*img.cols/8)
	      send='W';
	    sendCommand(&send);
	    printf("%c\n",send);

		}
		else
		if(code!=-1&&check>maximum&&duration.count()>1000)
		{		time1=high_resolution_clock::now();
				sum+=code;

				if(code%3==0)
				{
					//
				}
				else
				if(code%3==1)
				{
					//
				}
				else
				{
					//
				}

		}
		else
		{
			send='X';
      sendCommand(&send);
		}

	}



  //	imshow("window1",img);
	//waitKey(0);
//	imshow("window2",b);
	//waitKey(0);
//	imshow("window5",c);
	//waitKey(0);
//	imshow("window3",squareimage);
 	//waitKey(0);
//	imshow("window4",d);
//	waitKey(0);
  	}
