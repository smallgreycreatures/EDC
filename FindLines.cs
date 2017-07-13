using Emgu.CV;
using Emgu.CV.Structure;
using MathNet.Numerics;
using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Drawing;
using System.Linq;
using System.Text;

namespace Model.Measure.LowLevel
{
    public class Line
    {
        public Line(IEnumerable<Point> edgePoints)
        {
            var xArray = new double[edgePoints.Count()];
            var yArray = new double[edgePoints.Count()];
            double xTot = 0.0;
            for (int i = 0; i < edgePoints.Count(); i++)
            {
                xArray[i] = edgePoints.ElementAt(i).X;
                yArray[i] = edgePoints.ElementAt(i).Y;
                xTot += edgePoints.ElementAt(i).X;
            }
            X = xTot / edgePoints.Count();
            Tuple<double, double> l0 = Fit.Line(xArray, yArray);
            M = l0.Item1;
            K = l0.Item2;

        }

        public double M;
        public double K;
        public double X;
        public bool Reverse;
        public double RadianAngle
        {
            get
            {
                var angle = Math.Atan2(K, 1);
                angle = (double.IsNaN(angle) ? Math.PI / 2 : angle);

                while (angle < 0) angle += Math.PI * 2;
                while (angle > Math.PI * 2) angle -= Math.PI * 2;

                if (Reverse)
                    return angle + Math.PI;
                else
                    return angle;
            }
        }
        public double Angle
        {
            get
            {
                var angle = 180 * RadianAngle / Math.PI;
                return angle;
            }
        }

        public double CalcX(double y)
        {
            return (y - M) / K;
        }

        public double CalcY(double x)
        {
            return K * x + M;
        }
    }

    public class FindLines
    {
        /// <summary>
        /// finds all the lines in the image
        /// </summary>
        /// <param name="img">the image to find lines on</param>
        /// <returns></returns>
        public static Line GetLine(Image<Gray, byte> gray, Rectangle a1)
        {
            Image<Gray, Byte> edgeImage;
            var edges = FindEdges(gray, a1, out edgeImage);
            IEnumerable<Point> longestEdge = null;
            foreach (var edge in edges)
                if (longestEdge == null || longestEdge.Count() < edge.Count())
                    longestEdge = edge;

            // detect darkside.
            if (longestEdge == null) return null;
            var pix = longestEdge.First();
            var returnData = new Line(longestEdge);

            var angle = returnData.RadianAngle + Math.PI / 2;
            double pSide = 0.0;
            double mSide = 0.0;

            var dX = Math.Cos(angle);
            var dY = Math.Sin(angle);

            for (int trig = 1; trig < 10; trig++)
            {
                var xP = (int)(pix.X + dX * trig * 2);
                var yP = (int)(pix.Y + dY * trig * 2);
                var xM = (int)(pix.X - dX * trig * 2);
                var yM = (int)(pix.Y - dY * trig * 2);

                var pSideValue = gray[yP, xP].Intensity;
                var mSideValue = gray[yM, xM].Intensity;
                pSide += pSideValue;
                mSide += mSideValue;
            }

            returnData.Reverse = mSide < pSide;

            return returnData;
        }

        public static void DrawLine(Line line, Rectangle area, ref Image<Gray, byte> lineImage)
        { 
            Point? FirstPoint = null;
            Point? LastPoint = null;
            for (int x = area.X; x < area.X + area.Width; x++)
            {
                var y = line.CalcY(x);
                if (y < area.Y + area.Height && area.Y < y)
                {
                    if (FirstPoint == null) FirstPoint = new Point(x, (int)y);
                    LastPoint = new Point(x, (int)y);
                }
            }

            var img = lineImage.CopyBlank();
            if (FirstPoint == null || LastPoint == null)
            {
                FirstPoint = new Point((int)line.X, area.Y);
                LastPoint = new Point((int)line.X, area.Y + area.Height);
                if (line.Reverse)
                    CvInvoke.ArrowedLine(img, (Point)LastPoint, (Point)FirstPoint, new MCvScalar(255), 1);
                else
                    CvInvoke.ArrowedLine(img, (Point)FirstPoint, (Point)LastPoint, new MCvScalar(255), 1);
            }
            else
            {
                if (line.Reverse)
                    CvInvoke.ArrowedLine(img, (Point)LastPoint, (Point)FirstPoint, new MCvScalar(255), 1);
                else
                    CvInvoke.ArrowedLine(img, (Point)FirstPoint, (Point)LastPoint, new MCvScalar(255), 1);
            }
            lineImage.SetValue(new Gray(255), img);
        }

        public static IEnumerable<IEnumerable<Point>> FindEdges(Image<Gray, byte> grayImage, Rectangle workArea, out Image<Gray, byte> edgeImage)
        {
            Stopwatch watch = Stopwatch.StartNew();
            watch.Restart();
            edgeImage = grayImage.CopyBlank();


            double maxPixelValue = 0;
            double minPixelValue = 255;
            // limit search area to inside the image.
            int xMin = Math.Max(workArea.X, 0);
            int yMin = Math.Max(workArea.Y, 0);
            int yMax = Math.Min(yMin + workArea.Height, grayImage.Height);
            int xMax = Math.Min(xMin + workArea.Width, grayImage.Width);


            Boolean quadraticImage = false;
  
            for (int x = xMin; x < xMax; x += 10)
            {
                for (int y = yMin; y < yMax; y += 10) //Optimering???
                {
                    var pixelValue = grayImage[y, x].Intensity;
                    maxPixelValue = Math.Max(maxPixelValue, pixelValue);
                    minPixelValue = Math.Min(minPixelValue, pixelValue);
                }
            }
            long l1 = watch.ElapsedMilliseconds;
            double threshold = minPixelValue + maxPixelValue;
            threshold /= 2;

            bool lastValueX;
            bool thisPixelX;
            bool lastValueY;
            bool thisPixelY;

            /*      if (workArea.Width == workArea.Height && false)
                  {
                      long l2 = watch.ElapsedMilliseconds;
                      //edgeImage.Draw(workArea, new Gray(255), -1);
                      //edgeImage.ROI = workArea;
                      edgeImage.Save("c:\\Users\\bk\\Documents\\edge_rect.bmp");
                      Image<Gray, Byte> smallImg;// = new Image<Gray, Byte>(new Size(workArea.Width, workArea.Height));
                      long l3 = watch.ElapsedMilliseconds;
                      smallImg = grayImage.Copy(workArea);
                      long l4 = watch.ElapsedMilliseconds;
                      smallImg.Save("c:\\Users\\bk\\Documents\\smallImg.bmp");
                      long l5 = watch.ElapsedMilliseconds;

                      for (int x = xMin; x < xMax; x++)
                      {
                          lastValueX = grayImage[yMin, x].Intensity < threshold;
                          lastValueY = grayImage[x, xMin].Intensity < threshold;

                          for (int y = yMin; y < yMax; y++)
                          {

                              thisPixelX = grayImage[y, x].Intensity < threshold;
                              thisPixelY = grayImage[x, y].Intensity < threshold;

                              if (thisPixelX ^ lastValueX) /* xor */
            /*    edgeImage[y, x] = new Gray(255);
            lastValueX = thisPixelX;


            if (thisPixelY ^ lastValueY) /* xor */
            /*   edgeImage[x, y] = new Gray(255);
           lastValueY = thisPixelY;
       }
   }
   long l6 = watch.ElapsedMilliseconds;
   Debug.Write("hej");
}

else
{*/



            // Y axis
             for (int x = xMin; x < xMax; x++)
               {
                   lastValueX = grayImage[yMin, x].Intensity < threshold;

                   for (int y = yMin; y < yMax; y++)
                   {
                       thisPixelX = grayImage[y, x].Intensity < threshold;
                       if (thisPixelX ^ lastValueX) /* xor */
                    edgeImage[y, x] = new Gray(255);
                lastValueX = thisPixelX;
            }
        }

        // x axis threshold
        for (int y = yMin; y < yMax; y++)
        {
            lastValueY = grayImage[y, xMin].Intensity < threshold;
            for (int x = xMin; x < xMax; x++)
            {
                thisPixelY = grayImage[y, x].Intensity < threshold;
               if (thisPixelY ^ lastValueY) /* xor */
                 edgeImage[y, x] = new Gray(255);
             lastValueY = thisPixelY;
         }
     }
 //}    

            
            long l = watch.ElapsedMilliseconds;
            watch.Stop();
             edgeImage.Save("c:\\Users\\bk\\Documents\\edge_imgae_findlines.bmp");
            return findEdges(edgeImage, workArea);
        }

        private static void changeLineIndex(ref Dictionary<int, int> pointToEdge, ref Dictionary<int, List<int>> edgeToPoint, int oldIndex, int newIndex)
        {
            if (oldIndex == newIndex)
                return;

            foreach (var point in edgeToPoint[oldIndex])
            {
                pointToEdge[point] = newIndex;
                edgeToPoint[newIndex].Add(point);
            }
            edgeToPoint.Remove(oldIndex);
        }

        private static IEnumerable<IEnumerable<Point>> findEdges(Image<Gray, byte> edgeImage, Rectangle workArea)
        {
            // we need 8 way cognetivity
            Stopwatch watch = Stopwatch.StartNew();
            watch.Restart();
            int LineIndex = 1;
            Dictionary<int, int> pointToEdge = new Dictionary<int, int>();
            Dictionary<int, List<int>> edgeToPoint = new Dictionary<int, List<int>>();
            Dictionary<int, Point> points = new Dictionary<int, Point>();

            var minX = Math.Max(1, workArea.X);
            var maxX = Math.Max(edgeImage.Size.Width - 1, workArea.X + workArea.Width);
            var minY = Math.Max(1, workArea.Y);
            var maxY = Math.Min(edgeImage.Size.Height - 1, workArea.Y + workArea.Height);

            for (int y = minY; y < maxY; y++)
            {
                for (int x = minX; x < maxX; x++)
                {
                    var point = x * 10000 + y;
                    points.Add(point, new Point(x, y));
                    if (edgeImage[y, x].Intensity == 0)
                    {

                        pointToEdge.Add(point, 0);
                        if (!edgeToPoint.ContainsKey(0))
                            edgeToPoint.Add(0, new List<int>());
                        edgeToPoint[0].Add(point);
                        continue;
                    }

                    var diffFromZero = new List<int>();
                    var a = (x - 1) * 10000 + (y - 1);
                    int aboveLeft = (pointToEdge.ContainsKey(a) ? pointToEdge[a] : 0);
                    a = (x) * 10000 + (y - 1);
                    int aboveMiddle = (pointToEdge.ContainsKey(a) ? pointToEdge[a] : 0);
                    a = (x + 1) * 10000 + (y - 1);
                    int aboveRight = (pointToEdge.ContainsKey(a) ? pointToEdge[a] : 0);
                    a = (x - 1) * 10000 + (y);
                    int left = (pointToEdge.ContainsKey(a) ? pointToEdge[a] : 0);

                    if (aboveLeft != 0)
                        diffFromZero.Add(aboveLeft);
                    if (aboveMiddle != 0)
                        diffFromZero.Add(aboveMiddle);
                    if (aboveRight != 0)
                        diffFromZero.Add(aboveRight);
                    if (left != 0)
                        diffFromZero.Add(left);

                    if (diffFromZero.Count() == 0)
                    {
                        pointToEdge.Add(point, LineIndex);
                        if (!edgeToPoint.ContainsKey(LineIndex))
                            edgeToPoint.Add(LineIndex, new List<int>());
                        edgeToPoint[LineIndex].Add(point);
                        LineIndex++;
                    }
                    else if (diffFromZero.Count() >= 1)
                    {

                        var lineId = diffFromZero.First();
                        pointToEdge.Add(point, lineId);
                        if (!edgeToPoint.ContainsKey(lineId))
                            edgeToPoint.Add(lineId, new List<int>());
                        edgeToPoint[lineId].Add(point);
                    }

                    if (diffFromZero.Count() > 1)
                    {
                        var newValue = diffFromZero.First();
                        foreach (var item in diffFromZero)
                        {
                            if (item != newValue &&
                                edgeToPoint.ContainsKey(item))
                                changeLineIndex(ref pointToEdge, ref edgeToPoint, item, newValue);
                        }
                    }
                }
            }
            long l = watch.ElapsedMilliseconds;
            watch.Stop();

            var v0 = edgeToPoint.OrderBy(o => o.Value.Count).Reverse();
            var v2 = new List<List<Point>>();
            foreach (var item0 in v0)
            {
                if (item0.Key == 0)
                    continue;
                var list = new List<Point>();
                foreach (var item1 in item0.Value)
                {
                    list.Add(points[item1]);
                }
                v2.Add(list);
            }

            return v2;
        }
    }
}
