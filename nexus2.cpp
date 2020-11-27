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
#include <math.h>
#include <termios.h>
#include <math.h>

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
  settings("/dev/ttyACM0");
	VideoCapture cap(1);
		if(!cap.isOpened())
		return -1;


	Mat img;
  img=imread("images.jpeg",1);
  cap>>img;

  //YCrCb IMAGE SCHEME;
  cvtColor(img,img,COLOR_BGR2YCrCb);

	int i,j,gCbMin=0,gyMin=0,gCrMin=152,gCbMax=149,gyMax=255,gCrMax=255,can=0,bCbMin=0,byMin=0,bCrMin=152,bCbMax=149,byMax=255,bCrMax=255,rCbMin=0,ryMin=0,rCrMin=152,rCbMax=149,ryMax=255,rCrMax=255;
  int tCbMin=116,tyMin=48,tCrMin=70,tCbMax=184,tyMax=118,tCrMax=121;

  int align=239;
  int thresh=0;
  namedWindow("CIRCLEORIGINAL",WINDOW_NORMAL);

	namedWindow("window1",WINDOW_NORMAL);
  namedWindow("window2",WINDOW_NORMAL);
  namedWindow("window3",WINDOW_NORMAL);
	namedWindow("ColorImageWindow",WINDOW_NORMAL);

  namedWindow( "circlewindow", CV_WINDOW_NORMAL );
  namedWindow("TriangleWindow",CV_WINDOW_NORMAL);
  namedWindow("SquareWindow",CV_WINDOW_NORMAL);
	createTrackbar("gCbMin","window1",&gCbMin,255);
	createTrackbar("gCbMax","window1",&gCbMax,255);
	createTrackbar("gyMin","window1",&gyMin,255);
	createTrackbar("gyMax","window1",&gyMax,255);
	createTrackbar("gCrMin","window1",&gCrMin,255);
	createTrackbar("gCrMax","window1",&gCrMax,255);
  createTrackbar("bCbMin","window2",&btCbMin,255);
	createTrackbar("btCbMax","window2",&btCbMax,255);
	createTrackbar("btyMin","window2",&btyMin,255);
	createTrackbar("btyMax","window2",&btyMax,255);
	createTrackbar("btCrMin","window2",&btCrMin,255);
	createTrackbar("btCrMax","window2",&btCrMax,255);
  createTrackbar("rCbMin","window3",&rCbMin,255);
  createTrackbar("rCbMax","window3",&rCbMax,255);
  createTrackbar("ryMin","window3",&ryMin,255);
  createTrackbar("ryMax","window3",&ryMax,255);
  createTrackbar("rCrMin","window3",&rCrMin,255);
  createTrackbar("rCrMax","window3",&rCrMax,255);

  createTrackbar("can","window1",&can,1000);
	createTrackbar("align","window1",&align,img.cols);
  createTrackbar("threshold","TriangleWindow",&thresh,300);

  bool drawcircle;
  int barcode=0;
	while(1)
	{	
    int check=0;
    Point2f center;
    Mat img;
    //IMAGE RECEIVED FROM CAMERA
    cap>>img;
    char send;
    //YCrCb IMAGE SCHEME;
    cvtColor(img,img,COLOR_BGR2YCrCb);

		Mat blackimage(img.rows,img.cols,CV_8UC1,Scalar(255));
		Mat colorimage(img.rows,img.cols,CV_8UC3,Scalar(255,255,255));
    Mat circleorig(img.rows,img.cols,CV_8UC3,Scalar(255,255,255));

    //SELECTED ITEM WHITE
    for(i=0;i<img.rows;++i)
		{
			for(j=0;j<img.cols;++j)
			{

				if(barcode%3==0 && img.at<Vec3b>(i,j)[0]>=gyMin && img.at<Vec3b>(i,j)[1]>=gCrMin && img.at<Vec3b>(i,j)[2]>=gCbMin && img.at<Vec3b>(i,j)[0]<=gyMax && img.at<Vec3b>(i,j)[1]<=gCrMax && img.at<Vec3b>(i,j)[2]<=gCbMax)
				blackimage.at<uchar>(i,j) = 255;
				else if(barcode%3==0)
					blackimage.at<uchar>(i,j) = 0;
        if(barcode%3==1 && img.at<Vec3b>(i,j)[0]>=byMin && img.at<Vec3b>(i,j)[1]>=bCrMin && img.at<Vec3b>(i,j)[2]>=bCbMin && img.at<Vec3b>(i,j)[0]<=byMax && img.at<Vec3b>(i,j)[1]<=bCrMax && img.at<Vec3b>(i,j)[2]<=bCbMax)
        blackimage.at<uchar>(i,j) = 255;
        else if(barcode%3==1)
          blackimage.at<uchar>(i,j) = 0;
        if(barcode%3==2 && img.at<Vec3b>(i,j)[0]>=ryMin && img.at<Vec3b>(i,j)[1]>=rCrMin && img.at<Vec3b>(i,j)[2]>=rCbMin && img.at<Vec3b>(i,j)[0]<=ryMax && img.at<Vec3b>(i,j)[1]<=rCrMax && img.at<Vec3b>(i,j)[2]<=rCbMax)
        blackimage.at<uchar>(i,j) = 255;
        else if(barcode%3==2)
          blackimage.at<uchar>(i,j) = 0;

			}
		}




		for(i=0;i<img.rows;++i)
		{
			for(j=0;j<img.cols;++j)
			{
          if(barcode%3==0 && img.at<Vec3b>(i,j)[0]>=gyMin && img.at<Vec3b>(i,j)[1]>=gCrMin && img.at<Vec3b>(i,j)[2]>=gCbMin && img.at<Vec3b>(i,j)[0]<=gyMax && img.at<Vec3b>(i,j)[1]<=gCrMax && img.at<Vec3b>(i,j)[2]<=gCbMax)
        circleorig.at<Vec3b>(i,j) = 255;
        else if(barcode%3==0)
          circleorig.at<Vec3b>(i,j) = 0;
        if(barcode%3==1 && img.at<Vec3b>(i,j)[0]>=byMin && img.at<Vec3b>(i,j)[1]>=bCrMin && img.at<Vec3b>(i,j)[2]>=bCbMin && img.at<Vec3b>(i,j)[0]<=byMax && img.at<Vec3b>(i,j)[1]<=bCrMax && img.at<Vec3b>(i,j)[2]<=bCbMax)
        circleorig.at<Vec3br>(i,j) = 255;
        else if(barcode%3==1)
          circleorig.at<Vec3b>(i,j) = 0;
        if(barcode%3==2 && img.at<Vec3b>(i,j)[0]>=ryMin && img.at<Vec3b>(i,j)[1]>=rCrMin && img.at<Vec3b>(i,j)[2]>=rCbMin && img.at<Vec3b>(i,j)[0]<=ryMax && img.at<Vec3b>(i,j)[1]<=rCrMax && img.at<Vec3b>(i,j)[2]<=rCbMax)
        circleorig.at<Vec3b>(i,j) = 255;
        else if(barcode%3==2)
          circleorig.at<Vec3b>(i,j) = 0;
				

			}
		}



   // for(i=0;i<img.rows;++i)
		//{
		//	for(j=0;j<img.cols;++j)
		//	{
    //    if(img.at<Vec3b>(i,j)[0]>=tyMin&&img.at<Vec3b>(i,j)[1]>=tCrMin&&img.at<Vec3b>(i,j)[2]>=tCbMin&&img.at<Vec3b>(i,j)[0]<=tyMax&&img.at<Vec3b>(i,j)[1]<=tCrMax&&img.at<Vec3b>(i,j)[2]<=tCbMax)
   //       circleorig.at<Vec3b>(i,j)=img.at<Vec3b>(i,j);
   //     else
   //       circleorig.at<Vec3b>(i,j)=0;
	//		}
	//	}
    cap>>a;
    Mat b(a.rows,a.cols,CV_8UC3,Scalar(0,0,0));
    //Mat c(a.rows,a.cols,CV_8UC1,Scalar(0));
    Mat d(a.rows,a.cols,CV_8UC1,Scalar(0));
    int count=0;

      cvtColor(a,b,CV_BGR2GRAY);
      Mat element=getStructuringElement(MORPH_RECT,Size(3,3),Point(1,1));
      erode(b,b,element);
    dilate(b,b,element);
    dilate(b,b,element);
    erode(b,b,element);
      GaussianBlur(b,b,Size(5,5),0,255,0);
      threshold(b,colorimage,10,255,0);
      // Canny(b,colorimage,20,100,3);




    imshow("CIRCLEORIGINAL",circleorig);
    waitKey(1);

  //warped Mat warpedimage declared
  Mat warpedimage(img.rows,img.cols,CV_8UC3,Scalar(0,0,0));
  Mat cwarpedimage(img.rows,img.cols,CV_8UC3,Scalar(0,0,0));

  //COORDINATES TO BE WARPED
  vector<Point2f>c1;
    c1.push_back(Point2f(0,0));
    c1.push_back(Point2f(img.cols, 0));
    c1.push_back(Point2f(img.cols, img.rows));
    c1.push_back(Point2f(0,img.rows));

  //FINAL COORDINATES
  vector<Point2f>a1;
    a1.push_back(Point2f(0,0));
    a1.push_back(Point2f(img.cols,0));
    a1.push_back(Point2f(img.cols-align,img.rows));
    a1.push_back(Point2f(align,img.rows));


  //HOMOGRAPHY MATRIX
  Mat h = findHomography(c1,a1);

  //IMAGE WARPED
  warpPerspective(colorimage,warpedimage,h,warpedimage.size());

  warpPerspective(circleorig,cwarpedimage,h,warpedimage.size());


  Mat src(img.rows/2,img.cols,CV_8UC3,Scalar(0,0,0));
  Mat csrc(img.rows/2,img.cols,CV_8UC3,Scalar(0,0,0));


  for(int i= img.rows/2 ; i<img.rows ; i++)
  {
    for(int j = 0; j<img.cols ; j++)
    {
      src.at<uchar>(i-img.rows/2,j)=warpedimage.at<uchar>(i,j);
    }
  }

  for(int i= img.rows/2 ; i<img.rows ; i++)
  {
    for(int j = 0; j<img.cols ; j++)
    {
      csrc.at<Vec3b>(i-img.rows/2,j)=cwarpedimage.at<Vec3b>(i,j);
    }
  }




  Mat   src_gray=src.clone();
  Mat  csrc_gray;
  //cvtColor(src,src_gray,CV_BGR2GRAY);
  cvtColor(csrc,csrc_gray,CV_BGR2GRAY);

  //	BINARYIMAGE
  for(int i=0;i<src_gray.rows;i++)
  {
  	for(int j=0;j<src_gray.cols;j++)
  	{
  		if(src_gray.at<uchar>(i,j)>=80)
  			src_gray.at<uchar>(i,j)=255;
  		else
  			src_gray.at<uchar>(i,j)=0;
  	}
  }


  for(int i=0;i<csrc_gray.rows;i++)
  {
    for(int j=0;j<csrc_gray.cols;j++)
    {
      if(csrc_gray.at<uchar>(i,j)>=10)
        csrc_gray.at<uchar>(i,j)=255;
      else
        csrc_gray.at<uchar>(i,j)=0;
    }
  }

  //NOISE REMOVED USING EROSION AND DILATION
  Mat element=getStructuringElement(MORPH_RECT,Size(7,7),Point(-1,-1));
  //Mat temp,temp1;
  //erode(src_gray,temp,element);
  //dilate(temp,temp1,element);
  //src_gray=temp1.clone();

  Mat temp2,temp3;
  erode(csrc_gray,temp2,element);
  dilate(temp2,temp3,element);
  csrc_gray=temp3.clone();

  // Reduce the noise so we avoid false circle detection
 // GaussianBlur( src_gray, src_gray, Size(9, 9), 2, 2 );
  GaussianBlur( csrc_gray, csrc_gray, Size(9, 9), 2, 2 );

  Mat contourimage(src_gray.rows,src_gray.cols,CV_8UC3,Scalar(0,0,0));
  Mat ccontourimage(src_gray.rows,src_gray.cols,CV_8UC3,Scalar(0,0,0));

	Canny(src_gray,contourimage,can,2*can,3);
  Canny(csrc_gray,ccontourimage,can,2*can,3);

	vector<vector<Point>>contours;
	vector<Vec4i> hierarchy;

  vector<vector<Point>>ccontours;
	vector<Vec4i> chierarchy;


	findContours(contourimage,contours,hierarchy,CV_RETR_LIST,CV_CHAIN_APPROX_SIMPLE,Point(0,0));
  findContours(ccontourimage,ccontours,chierarchy,CV_RETR_TREE,CV_CHAIN_APPROX_SIMPLE,Point(0,0));
  j=0;
  int cj=0;
	int i=0,ci=0;
	double area=0;
	while(i<contours.size()&&contours.empty()!=1)
	{
	   if(area<contourArea(contours[i]));
		{  area=contourArea(contours[i]);
		   j=i;
    }


		 i++;
  }

  Mat triangleimage(img.rows/2,img.cols,CV_8UC1,Scalar(0,0,0));
  Mat squareimage(img.rows/2,img.cols,CV_8UC1,Scalar(0,0,0));
  Mat circleimage(img.rows/2,img.cols,CV_8UC1,Scalar(0,0,0));


	int maximum=0;
  int polyarea=0;
  if(ccontours.empty()!=1)
  {
    vector<vector<Point> > ccontours_poly( ccontours.size() );
    vector<Point2f>centers( ccontours.size() );
    vector<float>centerx;
    vector<float>radius( ccontours.size() );

    for( size_t i = 0; i< ccontours.size(); i++ )
    {   approxPolyDP( ccontours[i], ccontours_poly[i], arcLength(Mat(ccontours[i]), true)*0.05, true );
        minEnclosingCircle( ccontours_poly[i], centers[i], radius[i] );
    }




    for(int i=0;i<ccontours.size();i++)
      {
        if(int(centers[i].y)>maximum)
          {
            maximum=centers[i].y;
            cj=i;
          }
      }
      center=centers[cj];


      drawContours(ccontourimage,ccontours,cj,Scalar(255),5,8);

      circle( circleimage, centers[cj], (int)radius[cj], Scalar(255), 2);

  }
  else  drawcircle=false;

  int side=0;


  if(contours.empty()!=1)
  {
      vector<vector<Point> > contours_poly( contours.size() );
      vector<Rect> boundRect( contours.size() );
      vector<Vec2f>sides;

      for( size_t i = 0; i< contours.size(); i++ )
      { approxPolyDP( contours[i], contours_poly[i], arcLength(Mat(contours[i]), true)*0.05, true );
        boundRect[i] = boundingRect( contours_poly[i] );
      }



      Moments m;
      if (contours_poly[j].size()==4)
      {
        rectangle( squareimage, boundRect[j].tl(), boundRect[j].br(),  Scalar(255), 5);
        m = moments(squareimage,true);
        side=4;
      }
      else
      if(contours_poly[j].size()==3)
      {
          drawContours(triangleimage,contours,j,Scalar(255),5,8);
          m = moments(triangleimage,true);
          side=3;
      }
      check=m.m01/m.m00;


  }


	if(maximum>check&&ccontours.empty()!=1)
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
	if(contours.empty()!=1)
		{ 
      cout<<"SQUARE BELOW CIRCLE"<<endl;
      //if(side==3&&area>4000&&area<15000)
      //{
      // send='Q';
      //  sendCommand(&send);
      //  printf("%c\n",send);
      //  send='L';
       // sendCommand(&send);
      //  printf("%c\n",send);
      //  send='W';
      //  sendCommand(&send);
      //  waitKey(10);
     // }
      if(side==4&&area>18000)
      {




       int array[contours.size()];

      for (int m = 0; m < contours.size(); ++m)
        array[m]=0;

      for (int m = 0; m < contours.size(); ++m)
      if(contours[m].size()==4)
      {
        array[m]=1;
        count++;  
      }
      Point min1=Point(a.rows-1,0),min2=Point(a.cols-1,0),max1=Point(0,0),max2=Point(0,0);
      vector<Point2f> pts_a;
    vector<Point2f> pts_b;
    if(count==5)
    {
      int rem;
      for( int m=0;m<contours.size();m++)
      {
        int area=0;
        if(contourArea(contours[m],false)>area)
          rem=m;
      }
      array[rem]=0;
      for (int k = 0; k < 2; ++k)
      {
        int min=a.cols,max=0;
        for (int m = 0; m < contours.size(); ++m)
        for (int j = 0; j < 4; ++j)
        {
          if(array[m]==1)
          {
              if(k==0 && contours[m][j].x<min)
              {
                min=contours[m][j].x;
                min1=contours[m][j];
              }
              if(k==1 && contours[m][j].x<min && contours[m][j]!=min1)
              { 
                min=contours[m][j].x;
                min2=contours[m][j];
              }

              if(k==0 && contours[m][j].x>max)
              {
                max=contours[m][j].x;
                max1=contours[m][j];
              }
              if(k==1 && contours[m][j].x>max && contours[m][j]!=max1)
              { 
                max=contours[m][j].x;
                max2=contours[m][j];
              }
          }
        }
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

      pts_a.push_back(min1);
      pts_a.push_back(max1);
      pts_a.push_back(min2);
      pts_a.push_back(max2);
      pts_b.push_back(Point2f(0, 0));
        pts_b.push_back(Point2f(a.cols,0));
        pts_b.push_back(Point2f(0, a.rows-1));
        pts_b.push_back(Point2f(a.cols-1, a.rows-1));
        Mat h = findHomography(pts_a, pts_b);
        warpPerspective(c, d, h, b.size());
        }
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
          if(number.empty())
            printf("no bar code detected\n");   
          else
          for(int m=0;m<number.size();m++)  
            printf("%d\n",number[m]);
        
      }
      barcode=8*number[number.size()-4]+4*number[number.size()-3]+2*number[number.size()-2]+number[number.size()-1]










        //send='E';
        //sendCommand(&send);
        //  printf("%c\n",send);
        //send='l';
       // sendCommand(&send);
       //   printf("%c\n",send);
      }

    }
    else
    {
      send='X';
      sendCommand(&send);
    }



//namedWindow("grayimage",WINDOW_NORMAL);
//imshow("grayimage",csrc_gray);
//waitKey(1);
imshow("TriangleWindow",triangleimage);
waitKey(1);
imshow("SquareWindow",squareimage);
waitKey(1);
imshow( "circlewindow",circleimage);
waitKey(1);


  //imshow("grayimage",src_gray);
  //waitKey(1);
  namedWindow("ccontourwindow",WINDOW_NORMAL);
  imshow("ccontourwindow",ccontourimage);
  waitKey(1);

  imshow("ColorImageWindow",colorimage);
  waitKey(1);
  imshow("window1",circleorig);
  waitKey(1);



	}
}