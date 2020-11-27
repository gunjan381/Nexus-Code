#include "opencv2/highgui/highgui.hpp"
#include "opencv2/core/core.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include<iostream>
using namespace std;
using namespace cv;
#include<vector>
#include<bits/stdc++.h>
#include "opencv2/calib3d/calib3d.hpp"

int main()

{
	VideoCapture cap(0);
		if(!cap.isOpened())
		return -1;


	Mat img;
  img=imread("images.jpeg",1);
  cap>>img;

  //YCrCb IMAGE SCHEME;
  cvtColor(img,img,COLOR_BGR2YCrCb);

	int i,j,CbMin=0,yMin=0,CrMin=0,CbMax=110,yMax=255,CrMax=120,can=0;
  int tCbMin=0,tyMin=0,tCrMin=0,tCbMax=255,tyMax=255,tCrMax=255;

  int align=0;
  int thresh=0;
  namedWindow("CIRCLEORIGINAL",WINDOW_NORMAL);

	namedWindow("window1",WINDOW_NORMAL);
	namedWindow("ColorImageWindow",WINDOW_NORMAL);

  namedWindow( "circlewindow", CV_WINDOW_NORMAL );
  namedWindow("TriangleWindow",CV_WINDOW_NORMAL);
  namedWindow("SquareWindow",CV_WINDOW_NORMAL);
	createTrackbar("CbMin","window1",&CbMin,255);
	createTrackbar("CbMax","window1",&CbMax,255);
	createTrackbar("yMin","window1",&yMin,255);
	createTrackbar("yMax","window1",&yMax,255);
	createTrackbar("CrMin","window1",&CrMin,255);
	createTrackbar("CrMax","window1",&CrMax,255);
  createTrackbar("tCbMin","CIRCLEORIGINAL",&tCbMin,255);
	createTrackbar("tCbMax","CIRCLEORIGINAL",&tCbMax,255);
	createTrackbar("tyMin","CIRCLEORIGINAL",&tyMin,255);
	createTrackbar("tyMax","CIRCLEORIGINAL",&tyMax,255);
	createTrackbar("tCrMin","CIRCLEORIGINAL",&tCrMin,255);
	createTrackbar("tCrMax","CIRCLEORIGINAL",&tCrMax,255);

  createTrackbar("can","window1",&can,1000);
	createTrackbar("align","window1",&align,img.cols);
  createTrackbar("threshold","TriangleWindow",&thresh,300);

  bool drawcircle;
	while(1)
	{	int check=0;

    Mat img;
    //IMAGE RECEIVED FROM CAMERA
    cap>>img;

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
				if(img.at<Vec3b>(i,j)[0]>=yMin&&img.at<Vec3b>(i,j)[1]>=CrMin&&img.at<Vec3b>(i,j)[2]>=CbMin&&img.at<Vec3b>(i,j)[0]<=yMax&&img.at<Vec3b>(i,j)[1]<=CrMax&&img.at<Vec3b>(i,j)[2]<=CbMax)
				blackimage.at<uchar>(i,j) = 255;
				else
					blackimage.at<uchar>(i,j) = 0;

			}
		}




		for(i=0;i<img.rows;++i)
		{
			for(j=0;j<img.cols;++j)
			{
				if(img.at<Vec3b>(i,j)[0]>=yMin&&img.at<Vec3b>(i,j)[1]>=CrMin&&img.at<Vec3b>(i,j)[2]>=CbMin&&img.at<Vec3b>(i,j)[0]<=yMax&&img.at<Vec3b>(i,j)[1]<=CrMax&&img.at<Vec3b>(i,j)[2]<=CbMax)
				  colorimage.at<Vec3b>(i,j)=img.at<Vec3b>(i,j);
				else
					colorimage.at<Vec3b>(i,j)=0;

			}
		}



    for(i=0;i<img.rows;++i)
		{
			for(j=0;j<img.cols;++j)
			{
        if(img.at<Vec3b>(i,j)[0]>=tyMin&&img.at<Vec3b>(i,j)[1]>=tCrMin&&img.at<Vec3b>(i,j)[2]>=tCbMin&&img.at<Vec3b>(i,j)[0]<=tyMax&&img.at<Vec3b>(i,j)[1]<=tCrMax&&img.at<Vec3b>(i,j)[2]<=tCbMax)
          circleorig.at<Vec3b>(i,j)=img.at<Vec3b>(i,j);
        else
          circleorig.at<Vec3b>(i,j)=0;
			}
		}

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
      src.at<Vec3b>(i-img.rows/2,j)=warpedimage.at<Vec3b>(i,j);
    }
  }

  for(int i= img.rows/2 ; i<img.rows ; i++)
  {
    for(int j = 0; j<img.cols ; j++)
    {
      csrc.at<Vec3b>(i-img.rows/2,j)=cwarpedimage.at<Vec3b>(i,j);
    }
  }




  Mat   src_gray;
  Mat  csrc_gray;
  cvtColor(src,src_gray,CV_BGR2GRAY);
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
  Mat temp,temp1;
  erode(src_gray,temp,element);
  dilate(temp,temp1,element);
  src_gray=temp1.clone();

  Mat temp2,temp3;
  erode(csrc_gray,temp2,element);
  dilate(temp2,temp3,element);
  csrc_gray=temp3.clone();

  // Reduce the noise so we avoid false circle detection
  GaussianBlur( src_gray, src_gray, Size(9, 9), 2, 2 );
  GaussianBlur( csrc_gray, csrc_gray, Size(9, 9), 2, 2 );

  Mat contourimage(src_gray.rows,src_gray.cols,CV_8UC3,Scalar(0,0,0));
  Mat ccontourimage(src_gray.rows,src_gray.cols,CV_8UC3,Scalar(0,0,0));

	Canny(src_gray,contourimage,can,2*can,3);
  Canny(csrc_gray,ccontourimage,can,2*can,3);

	vector<vector<Point>>contours;
	vector<Vec4i> hierarchy;

  vector<vector<Point>>ccontours;
	vector<Vec4i> chierarchy;


	findContours(contourimage,contours,hierarchy,CV_RETR_TREE,CV_CHAIN_APPROX_SIMPLE,Point(0,0));
  findContours(ccontourimage,ccontours,chierarchy,CV_RETR_TREE,CV_CHAIN_APPROX_SIMPLE,Point(0,0));
  j=0;
  int cj=0;
	int i=0,ci=0;
	double area,carea;
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
        if(int(centers[i].x)>maximum)
          {
            maximum=centers[i].x;
            cj=i;
          }
      }


      drawContours(ccontourimage,ccontours,cj,Scalar(255),5,8);

      circle( circleimage, centers[cj], (int)radius[cj], Scalar(255), 2);

  }
  else  drawcircle=false;


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
      }
      else
      if(contours_poly[j].size()==3)
      {
          drawContours(triangleimage,contours,j,Scalar(255),5,8);
          m = moments(triangleimage,true);
      }
      check=m.m10/m.m00;

  }


	if(maximum>check&&contours.empty()!=1&&ccontours.empty()!=1)
	{
		cout<<"CIRCLE BELOW SQUARE "<<endl;
	}
	else
	if(maximum<check&&contours.empty()!=1&&ccontours.empty()!=1)
		cout<<"SQUARE BELOW CIRCLE"<<endl;



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
