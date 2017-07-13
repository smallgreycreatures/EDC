//----------------------------------------------------------------------------
//  Copyright (C) 2004-2017 by EMGU Corporation. All rights reserved.       
//----------------------------------------------------------------------------

using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Data;
using System.Drawing;
using System.Text;
using System.Windows.Forms;
using Emgu.CV;
using Emgu.CV.CvEnum;
using Emgu.CV.Structure;
using System.Diagnostics;
using Emgu.CV.Util;
using Emgu.CV.Cvb;
using Model.Measure.LowLevel;
using MathNet.Numerics;
using System.Linq;

namespace ShapeDetection
{
    public partial class MainForm : Form
    {
        public MainForm()
        {
            InitializeComponent();

        }
        const int HORIZONTAL = 255;
        const int VERTICAL = 0;
		const int UP = 1;
		const int RIGHT = 2;
		const int DOWN = 3;
		const int LEFT = 4;
		//const int TryTime     6;
		//const int SkipEdgePoint 2;
		const int MIN_LINE_LENGTH = 6;
        public void PerformShapeDetection()
        {
			Image<Bgr, Byte> img =
				new Image<Bgr, byte>("pic3.png");
			edLineDetector (img.Convert<Gray,byte>());
        }
  
		class EdgeSegment {
			
			List<Point> pointList = new List<Point>();

			public void addPoint(Point p){
				pointList.Add (p);
				
			}
			
		}
		class EdgeChain {
			List<int> xCoordinates;//all the x coordinates of edge points
			List<int> yCoordinates;//all the y coordinates of edge points
			List<int> startID; //the start index of each edge in the coordinate arrays
			int numberOfEdges;//the number of edges whose length are larger than minLineLen; numOfEdges < sId.size;

		}

		struct LineChains {
			List<int> xCoordinates;//all the x coordinates of line points
			List<int> yCoordinates;//all the y coordinates of line points
			List<int> startID; //the start index of each line in the coordinate arrays
			int numberOfLines;//the number of lines whose length are larger than minLineLen; numOfEdges < sId.size;

		}

		int imageWidth;
		int imageHeight;

		/*
		public int EdLineDetection(Image<Gray, byte> img) {
			EdgeChains edges;

			if(!EdgeDrawing(img, edges)){
				Debug.Write ("Fail");
				return -1;
			}
			return 0;
		}

		public int EdgeDrawing(Image<Gray, Byte> img, EdgeChains edges)
		{
			
			int threshold = 180;
			int pixelsToJump = 4;
			int anchorThreshold = 50;
			int scanIntervals = 1; //how many anchor points to jump over
			imageWidth = img.Cols;
			imageHeight = img.Rows;
			int pixelNum = imageWidth*imageHeight;
			 int edgePixelArraySize = pixelNum/5;
			 int maxNumOfEdge = edgePixelArraySize/20;

			Image<Gray, short> blurredImg = img.SmoothGaussian (5).Convert<Gray, short>();
			Image<Gray, short> dX = blurredImg.Sobel (1, 0, 3);
			Image<Gray, short> dY = blurredImg.Sobel (0, 1, 3);
			Image<Gray, byte> directionImage = blurredImg.CopyBlank().Convert<Gray, byte>();
			Image<Gray, short> sumdXdY = blurredImg.CopyBlank ();
			Image<Gray, byte> edgeImage = blurredImg.CopyBlank ().Convert<Gray, Byte>();
			Image<Gray, short> dXAbs = dX.AbsDiff(sumdXdY); //Calc the abs diff between the empty image sumdXdY and dX to get abs(dx)
			Image<Gray, short> dYAbs = dY.AbsDiff(sumdXdY); // same same
			Image<Gray, short> gradientThresholdImg = blurredImg.CopyBlank();


			int[] pFirstPartEdgeX = new int[edgePixelArraySize];
			int[] pFirstPartEdgeY = new int[edgePixelArraySize];
			int[] pSecondPartEdgeX = new int[edgePixelArraySize];
			int[] pSecondPartEdgeY = new int[edgePixelArraySize];
			int[] pFirstPartEdgeS = new int[maxNumOfEdge];
			int[] pSecondPartEdgeS = new int[maxNumOfEdge];
			int[] pAnchorX = new int[edgePixelArraySize];
			int[] pAnchorY = new int[edgePixelArraySize];

			CvInvoke.Add(dXAbs, dYAbs, sumdXdY);
			Debug.Write("HJEJA2");
			//calc direction 0=horizontal, 255=vertical
			CvInvoke.Compare(dXAbs, dYAbs, directionImage, CmpType.LessThan);
			/*for (int i = 0; i < img.Width; i++)
			{
				for (int j = 0; j < img.Height; j++)
				{
					if (dYAbs.Data[j,i,0] > dXAbs.Data[j,i,0])
					{
						directionImage.Data[j, i, 0] = 255;
					}
				}
			}
			//Threshold image and set weak pixels to zero
			CvInvoke.Threshold(sumdXdY, gradientThresholdImg, threshold, 255, ThresholdType.ToZero);
			CvInvoke.Imshow ("thresholdimg", gradientThresholdImg);
			CvInvoke.WaitKey ();


			//Extract the anchor points
			int[] pDirectionImage = directionImage.Bytes;
			byte[] pGradientThresholdImage = gradientThresholdImg.Bytes;
			

			 int anchorsSize = 0;
			int indexInArray;
			 char gValue1, gValue2, gValue3;
			for(int w=1; w<imageWidth-1; w=w+scanIntervals){
				for(int h=1; h<imageHeight-1; h=h+scanIntervals){
					indexInArray = h*imageWidth+w;
					//gValue1 = pdirImg[indexInArray];
					if(pDirectionImage[indexInArray]==HORIZONTAL){//if the direction of pixel is horizontal, then compare with up and down
						//gValue2 = pgImg[indexInArray];
						if(pGradientThresholdImage[indexInArray]>=pGradientThresholdImage[indexInArray-imageWidth]+anchorThreshold
							&&pGradientThresholdImage[indexInArray]>=pGradientThresholdImage[indexInArray+imageWidth]+anchorThreshold){// (w,h) is accepted as an anchor
							pAnchorX[anchorsSize]   = w;
							pAnchorY[anchorsSize++] = h;
						}
					}else{// if(pdirImg[indexInArray]==Vertical){//it is vertical edge, should be compared with left and right
						//gValue2 = pgImg[indexInArray];
						if(pGradientThresholdImage[indexInArray]>=pGradientThresholdImage[indexInArray-1]+anchorThreshold
							&&pGradientThresholdImage[indexInArray]>=pGradientThresholdImage[indexInArray+1]+anchorThreshold){// (w,h) is accepted as an anchor
							pAnchorX[anchorsSize]   = w;
							pAnchorY[anchorsSize++] = h;
						}
					}
				}
			}
			if(anchorsSize>edgePixelArraySize){
				Debug.WriteLine("anchor size is larger than its maximal size. anchorsSize=" + anchorsSize
					+ ", maximal size = " +edgePixelArraySize);
				return -1;
			}

			//link the anchors by smart routing
			byte* pEdgeImg = edgeImage.Data;

			int offsetPFirst=0, offsetPSecond=0;
			int offsetPS=0;


			 int x, y;
			 int lastX, lastY;
			 char lastDirection;//up = 1, right = 2, down = 3, left = 4;
			 char shouldGoDirection;//up = 1, right = 2, down = 3, left = 4;
			int edgeLenFirst, edgeLenSecond;

		}

		private void plot_points(List<Point> points)
		{
			Bitmap bmp = new Bitmap(400, 400);
			Graphics g = Graphics.FromImage(bmp);
			foreach (Point p in points)
			{
				g.DrawLine(new Pen(Color.White), p, new Point(p.X + 1, p.Y + 1));
			}

			Image<Bgr, Byte> graph_img = new Image<Bgr, byte>(bmp).Resize(400, 400, Emgu.CV.CvEnum.Inter.Linear, true);
		}

		*/


      private void textBox1_TextChanged(object sender, EventArgs e)
      {
         PerformShapeDetection();
      }

      private void loadImageButton_Click(object sender, EventArgs e)
      {

      }


        bool isAnchorPoint(int x, int y, Image<Gray, float> gradientImage, Image<Gray, byte> directionImage, int anchorThreshold)
        {

            if (directionImage.Data[y, x, 0] == HORIZONTAL)
            {

                if (gradientImage.Data[y, x, 0] - gradientImage.Data[y - 1, x, 0] >= anchorThreshold &&
                gradientImage.Data[y, x, 0] - gradientImage.Data[y + 1, x, 0] >= anchorThreshold)
                {
                    return true;
                }
            }
            else //It's vertical
            {
                if (gradientImage.Data[y, x, 0] - gradientImage.Data[y, x - 1, 0] >= anchorThreshold &&
                    gradientImage.Data[y, x, 0] - gradientImage.Data[y, x + 1, 0] >= anchorThreshold)
                {
                    return true;
                }
            }
            return false;
        }

		bool goLeft(int x, int y, Image<Gray, float> gradientImage, Image<Gray, byte> directionImage, Image<Gray, float> edgeImage, List<Point> edgeSegment, Tuple<List<int>, List<int>> edgeSegmentTuple)
        {
            if (x > gradientImage.Width || y > gradientImage.Height)
                return;
            while (gradientImage.Data[y, x, 0] > 0 && edgeImage.Data[y, x, 0] == 0 && directionImage.Data[y, x, 0] == HORIZONTAL)
            {
                edgeImage.Data[y, x, 0] = 255; //Save as edge
				edgeSegment.Add(new Point(x,y));
                //look at three neightbors to left and pick the one with largest gradient

            if (gradientImage.Data[y-1, x-1,0] >gradientImage.Data[y,x-1,0] &&
                    gradientImage.Data[y-1, x-1,0] > gradientImage.Data[y+1, x-1,0] )
                {
                    x = x - 1;
                    y = y - 1;
                } else if (gradientImage.Data[y+1, x-1,0] >gradientImage.Data[y, x-1,0] &&
                    gradientImage.Data[y+1, x-1,0] > gradientImage.Data[y-1, x-1, 0])
                {
                    x = x - 1;
                    y = y + 1;
                } else
                {
                    x = x - 1;
                }
            }
			if (directionImage.Data [y, x, 0] == VERTICAL)
				return true;
			else
				return false;
           /* if (directionImage.Data[y, x, 0] == VERTICAL)
            {
                int x_temp = x;
                int y_temp = y;
                goUp(x, y, gradientImage, directionImage, edgeImage);
                goLeft(x_temp, y_temp, gradientImage, directionImage, edgeImage);
            }*/
        }
		bool goRight(int x, int y, Image<Gray, float> gradientImage, Image<Gray, byte> directionImage, Image<Gray, float> edgeImage, List<Point> edgeSegment, Tuple<List<int>, List<int>> edgeSegmentTuple)
        {
            if (x > gradientImage.Width || y > gradientImage.Height)
                return;
            while (gradientImage.Data[y, x, 0] > 0 && edgeImage.Data[y, x, 0] == 0 && directionImage.Data[y, x, 0] == HORIZONTAL)
            {
                edgeImage.Data[y, x, 0] = 255; //Save as edge
				edgeSegment.Add(new Point(x,y));
                //look at three neightbors to right and pick the one with largest gradient

                if (gradientImage.Data[y - 1, x + 1, 0] > gradientImage.Data[y, x + 1, 0] &&
                        gradientImage.Data[y - 1, x + 1, 0] > gradientImage.Data[y + 1, x + 1, 0])
                {
                    x = x + 1;
                    y = y - 1;
                }
                else if (gradientImage.Data[y + 1, x + 1, 0] > gradientImage.Data[y, x + 1, 0] &&
                  gradientImage.Data[y + 1, x + 1, 0] > gradientImage.Data[y -1, x + 1, 0])
                {
                    x = x + 1;
                    y = y + 1;
                }
                else
                {
                    x = x + 1;
                }
            }
			if (directionImage.Data [y, x, 0] == VERTICAL)
				return true;
			else
				return false;
            /*if (directionImage.Data[y, x, 0] == VERTICAL)
            {
                int x_temp = x;
                int y_temp = y;
                goUp(x, y, gradientImage, directionImage, edgeImage);
                goLeft(x_temp, y_temp, gradientImage, directionImage, edgeImage);
            }*/
        }
		bool goDown(int x, int y, Image<Gray, float> gradientImage, Image<Gray, byte> directionImage, Image<Gray, float> edgeImage, List<Point> edgeSegment, Tuple<List<int>, List<int>> edgeSegmentTuple)
        {
            if (x > gradientImage.Width || y > gradientImage.Height)
                return;
            while (gradientImage.Data[y, x, 0] > 0 && edgeImage.Data[y, x, 0] == 0 && directionImage.Data[y, x, 0] == VERTICAL)
            {
                edgeImage.Data[y, x, 0] = 255; //Save as edge
				edgeSegment.Add(new Point(x,y));
                //look at three neightbors to up and pick the one with largest gradient

                if (gradientImage.Data[y + 1, x - 1, 0] > gradientImage.Data[y +1, x, 0] &&
                        gradientImage.Data[y + 1, x - 1, 0] > gradientImage.Data[y + 1, x + 1, 0])
                {
                    x = x - 1;
                    y = y + 1;
                }
                else if (gradientImage.Data[y + 1, x + 1, 0] > gradientImage.Data[y + 1, x - 1, 0] &&
                  gradientImage.Data[y + 1, x + 1, 0] > gradientImage.Data[y + 1, x, 0])
                {
                    x = x + 1;
                    y = y + 1;
                }
                else
                {
                    y = y + 1;
                }
            }
			if (directionImage.Data [y, x, 0] == HORIZONTAL)
				return true;
			else
				return false;
            /*if (directionImage.Data[y, x, 0] == HORIZONTAL)
            {
                int x_temp = x;
                int y_temp = y;
                goLeft(x, y, gradientImage, directionImage, edgeImage);
                goRight(x_temp, y_temp, gradientImage, directionImage, edgeImage);
            }*/
        }

		bool goUp(int x, int y, Image<Gray, float> gradientImage, Image<Gray, byte> directionImage, Image<Gray, float> edgeImage, List<Point> edgeSegment, Tuple<List<int>, List<int>> edgeSegmentTuple)
        {
            if (x > gradientImage.Width || y > gradientImage.Height)
                return;
            while (gradientImage.Data[y, x, 0] > 0 && edgeImage.Data[y, x, 0] == 0 && directionImage.Data[y, x, 0] == VERTICAL)
            {
                edgeImage.Data[y, x, 0] = 255; //Save as edge
				edgeSegment.Add(new Point(x,y));
                //look at three neightbors to down and pick the one with largest gradient

                if (gradientImage.Data[y - 1, x - 1, 0] > gradientImage.Data[y - 1, x, 0] &&
                        gradientImage.Data[y - 1, x - 1, 0] > gradientImage.Data[y - 1, x + 1, 0])
                {
                    x = x - 1;
                    y = y - 1;
                }
                else if (gradientImage.Data[y - 1, x + 1, 0] > gradientImage.Data[y - 1, x - 1, 0] &&
                  gradientImage.Data[y - 1, x + 1, 0] > gradientImage.Data[y - 1, x, 0])
                {
                    x = x + 1;
                    y = y - 1;
                }
                else
                {
                    y = y - 1;
                }
            }

			if (directionImage.Data [y, x, 0] == HORIZONTAL)
				return true;
			else
				return false;
         /*   if (directionImage.Data[y,x,0] == HORIZONTAL)
            {
                int x_temp = x;
                int y_temp = y;
                goLeft(x, y, gradientImage, directionImage, edgeImage);
                goRight(x_temp, y_temp, gradientImage, directionImage, edgeImage);
            }*/
        }
        public void edLineDetector(Image<Gray, Byte> img)
        {
			//EdgeChains edgeChains;

            int pixelsToJump = 4;
            int anchorThreshold = 50;
            Image<Gray, float> dX = img.Sobel(1, 0, 3);
            Image<Gray, float> dY = img.Sobel(0, 1, 3);
			Image<Gray, byte> directionImage = img.CopyBlank ();
            Image<Gray, float> sumdXdY = img.CopyBlank().Convert<Gray, float>();
            Image<Gray, float> thresholdImage = img.CopyBlank().Convert<Gray, float>();
            Image<Gray, float> edgeImage = img.CopyBlank().Convert<Gray, float>();
            Image<Gray, float> dXAbs = dX.AbsDiff(sumdXdY); //Calc the abs diff between the empty image sumdXdY and dX to get abs(dx)
            Image<Gray, float> dYAbs = dY.AbsDiff(sumdXdY); // same same
            CvInvoke.Add(dXAbs, dYAbs, sumdXdY);
            Debug.Write("HJEJA2");
            //calc direction 255=horizontal, 0=vertical
            //CvInvoke.Compare(dXAbs, dYAbs, directionImage, CmpType.LessThan);
			directionImage = dXAbs.Cmp (dYAbs, CmpType.LessThan);
			CvInvoke.Imshow ("dir", directionImage);
			CvInvoke.WaitKey ();
           /* for (int i = 0; i < img.Width; i++)
            {
                for (int j = 0; j < img.Height; j++)
                {
                    if (dYAbs.Data[j,i,0] > dXAbs.Data[j,i,0])
                    {
                        directionImage.Data[j, i, 0] = 255;
                    }
                }
            }*/
            //Threshold image and set weak pixels to zero
            CvInvoke.Threshold(sumdXdY, thresholdImage, 36, 255, ThresholdType.ToZero);

            //Extract the anchor points
            List<Point> anchorPoints = new List<Point>();
            for (int x = 1; x < img.Width-1; x += pixelsToJump)
            {
                for (int y = 1; y < img.Height-1; y += pixelsToJump)
                {

                    bool isAnchor = isAnchorPoint(x, y, thresholdImage, directionImage, anchorThreshold);
                    if (isAnchor)
                    {
                        anchorPoints.Add(new Point(x, y));

                    }
                }
            }

			List<List<Point>> edgeSegments = new List<List<Point>> ();
			List<Tuple<List<int>, List<int>>> edgeSegments2 = new List<Tuple<List<int>, List<int>>>();

			//Do the smart routing
            foreach (Point anchorPoint in anchorPoints)
            {
				List<Point> edgeSegment = new List<Point> ();
				Tuple<List<int>, List<int>> edgeSegmentTuple = new Tuple<List<int>, List<int>> ();
				List<int> xData = new List<int> ();
				List<int> yData = new List<int> ();
				edgeSegmentTuple.Item1 = xData;
				edgeSegmentTuple.Item2 = yData;
				int x = anchorPoint.X;
				int y = anchorPoint.Y;
				bool adjacentPoints = true;
				while (adjacentPoints) {
					x = anchorPoint.X;
					y = anchorPoint.Y;
                 
					if (directionImage.Data [y, x, 0] == HORIZONTAL) {
						adjacentPoints = goRight (x, y, thresholdImage, directionImage, edgeImage, edgeSegment, edgeSegmentTuple);
						x = anchorPoint.X;
						y = anchorPoint.Y;
						adjacentPoints = goLeft (x, y, thresholdImage, directionImage, edgeImage, edgeSegment, edgeSegmentTuple);




					} else {
						adjacentPoints = goDown (x, y, thresholdImage, directionImage, edgeImage, edgeSegment, edgeSegmentTuple);
						x = anchorPoint.X;
						y = anchorPoint.Y;
						adjacentPoints = goUp (x, y, thresholdImage, directionImage, edgeImage, edgeSegment, edgeSegmentTuple);
					}
				}
				edgeSegments.Add (edgeSegment);
				edgeSegments2.Add (edgeSegmentTuple);
            }

			List<List<Point>> candidates = new List<List<Point>> ();

			//Extract found circles
			foreach (List<Point> edgeSegment in edgeSegments) {
				Point startPoint = edgeSegment [0];
				Point endPoint = edgeSegment [edgeSegment.Count];
				int startX = startPoint.X;
				int startY = startPoint.Y;
				int endX = endPoint.X;
				int endY = endPoint.Y;

				if ((startX-1 == endX && startY+1 == endY) || (startX-1 == endX && startY == endY) ||
					(startX-1==endX && startY-1==endY) || (startX==endX && startY+1==endY) ||
					(startX==endX && startY==endY) || (startX==endX && startY-1 == endY) ||
						(startX+1 == endX && startY+1==endY) || (startX+1==endX && startY ==endY) ||
						(startX+1 == endX && startY-1==endY)) {
					candidates.Add (edgeSegment);
					}
				Ellipse ellipse = new Ellipse();
				ellipse = PointCollection.EllipseLeastSquareFitting(pointList.ToArray());

				//calc for each point x^2/a^2+y^2/b^2 -1 = err. Sum and take mean 



			}

			//Create edges
			int MIN_LINE_LENGTH = 6;
			foreach (Tuple<List<int>,List<int>> edgeSegment in edgeSegments2) {
				
			List<int> xData = new List<int> ();
				xData = edgeSegment.Item1;
			List<int> yData = new List<int> ();
				yData = edgeSegment.Item2;
			
				LineFit (edgeSegment);
			
			}
			/*
            foreach(Point anchorPoint in anchorPoints)
            {
                int x = anchorPoint.X;
                int y = anchorPoint.Y;

                bool doneLeft = true;
                bool doneRight = true;
                bool doneUp = true;
                bool doneDown = true;

                while(doneLeft && doneRight && doneDown && doneUp)
                {
                    if (directionImage.Data[y, x, 0] == HORIZONTAL)
                    {

                    }
                }

            }
			List<Point> connectedSegmentIndexes = new List<Point> ();
			int counter = 0;
			int counter2 = 0;
			foreach (List<Point> edgeSegment in edgeSegments) {
				Point startPoint = edgeSegment [0];
				Point endPoint = edgeSegment [edgeSegment.Count];
				int startX = startPoint.X;
				int startY = startPoint.Y;
				int endX = endPoint.X;
				int endY = endPoint.Y;

				foreach(List<Point> edgeSegment2 in edgeSegments) {


					Point startPoint2 = edgeSegment2 [0];
					Point endPoint2 = edgeSegment2 [edgeSegment2.Count];
					int startX2 = startPoint2.X;
					int startY2 = startPoint2.Y;
					int endX2 = endPoint2.X;
					int endY2 = endPoint2.Y;

				
					if ((startX == startX2 && startY == startY2) || (startX - 1 == startX2 && startY == startY2) ||
					    (startX - 1 == startX2 && startY - 1 == startY2) || (startX - 1 == startX2 && startY + 1 == startY2) ||
					    (startX == startX2 && startY - 1 == startY2) || (startX == startX2 && startY + 1 == startY2) ||
					    (startX + 1 == startX2 && startY - 1 == startY2) || (startX + 1 == startX2 && startY == startY2) ||
					    (startX + 1 == startX2 && startY + 1 == startY2)) {
					
						//Connection is found
						connectedSegmentIndexes.Add(new Point(counter, counter2));
					
					}

					counter2++;
				}
				counter2 = 0;
				counter++;
			}*/


        }

						public double ComputePointDistance2Line(double a, double b, double c, int x, int y){
			Math.Abs (a * x + b * y + c) / Math.Sqrt (Math.Pow (a, 2) + Math.Pow (b, 2));
		}


		public void LineFit(Tuple<List<int>,List<int>> edgeSegment ) {
			List<int> xData = new List<int> ();
			xData = edgeSegment.Item1;
			List<int> yData = new List<int> ();
			yData = edgeSegment.Item2;

			double lineFitError = Double.MaxValue;
			int numberOfPixels = xData.Count;

			lineFitError = Double.MaxValue; // current line fit error

			Tuple<double, double> line; // y = ax+b OR x = ay+b
			double a = 0;
			double b = 0;

			while (numberOfPixels > MIN_LINE_LENGTH) {

				Tuple<double, double> line2 = Fit.Line (xData.ToArray (), yData.ToArray ());

				a = line2.Item1;
				b = line2.Item2;
				GoodnessOfFit.RSquared (xData.Select (x => a + b * x), yData.ToArray ());
				if (lineFitError <= 1.0)
					break; // OK. An initial line segment detected

				// Skip the first pixel & try with the remaining pixels
				xData.RemoveAt (xData.Count);
				yData.RemoveAt (yData.Count);

				numberOfPixels--; // One less pixel

			} // end-while



			if (lineFitError > 1.0)
				return; // no initial line segment. Done.



			// An initial line segment detected. Try to extend this line segment

			int lineLen = MIN_LINE_LENGTH;
			List<int> xDataExtended = new List<int> ();
			List<int> yDataExtended = new List<int> ();
			for (int i = 0; i < MIN_LINE_LENGTH; i++) {
				xDataExtended [i] = xData [i];
				yDataExtended [i] = yData [i];
			}

			while (lineLen < numberOfPixels) {
				int x = edgeSegment.Item1 [lineLen];
				int y = edgeSegment.Item2 [lineLen];


				double d = ComputePointDistance2Line (a, -1, b, x, y);

				if (d > 1.0)
					break;

				lineLen++;

			} //end-while



			// End of the current line segment. Compute the final line equation & output it.

			Tuple<double,double> line3 = Fit.Line (xDataExtended.ToArray (), yDataExtended.ToArray ());





			// Extract line segments from the remaining pixels
			for (int i = lineLen; i < edgeSegment.Item1.Count; i++) {
				xData [i - lineLen] = edgeSegment.Item1 [i];
				yData [i - lineLen] = edgeSegment.Item2 [i];
					
			
			}
			for (int i = 0; i < lineLen; i++) {
				edgeSegment.Item1.RemoveAt (i);
				edgeSegment.Item2.RemoveAt (i);
			}
			LineFit (xData, yData, edgeSegment);
		}



    public class NoEllipsesFoundException : Exception
    {
        public NoEllipsesFoundException(string message) : base(message)
        {

        }
    }
    public class ToManyEllipsesFound : Exception
    {
        public ToManyEllipsesFound(string message) : base(message)
        {

        }
	
		}  
    }
}
