    /// <summary>
        /// Finds the ellipse/circle that best fits the input attributes.
        /// </summary>
        /// <param name="ellipseList">List of ellipses</param>
        /// <param name="minWidth">Minimum width of ellips. Width >= Height always</param>
        /// <param name="maxWidth">Maximum width of ellips. Width >= Height always</param>
        /// <param name="minHeight">Minimum height of ellips. Width >= Height always</param>
        /// <param name="maxHeight">Maximum height of ellips. Width >= Height always</param>
        /// <param name="error">Maximum error that is allowed from the given proportions</param>
        private Ellipse findSpecificEllipse(List<Ellipse> ellipseList, int minWidth, int maxWidth, int minHeight, int maxHeight, double error)
        {
            //Calculates the proportions of the ellips. If the relation are 1 we want a circle
            //TODO: Give some error space
            double circleFactorMin = maxWidth / maxHeight - error;
            double circleFactorMax = maxHeight / maxWidth + error;

            List<Ellipse> chosenEllips = new List<Ellipse>();
            foreach (Ellipse ellipse in ellipseList)
            {
                float xc = ellipse.RotatedRect.Center.X;
                float yc = ellipse.RotatedRect.Center.Y;


                //a and b are axises for ellipse
                float a = ellipse.RotatedRect.Size.Width / 2;
                float b = ellipse.RotatedRect.Size.Height / 2;

                // a/b=1 for perfect circles
                if (a / b >= circleFactorMin && a / b <= circleFactorMax && a >= minWidth && a <= maxWidth && b >= minHeight && b <= maxHeight)
                {

                    chosenEllips.Add(ellipse);

                }
            }

            if (chosenEllips.Count == 0)
            {
                throw new NoEllipsesFoundException("No Ellipses found. Try other parameters.");

            }
            else if (chosenEllips.Count > 1)
            {
                string ellipseData = "";
                int count = 0;
                foreach (Ellipse ellipse in chosenEllips)
                {
                    ellipseData += "Ellips" + count + ": x_c=" + ellipse.RotatedRect.Center.X + ", y_c=" + ellipse.RotatedRect.Center.Y + ", width=" +
                        (ellipse.RotatedRect.Size.Width / 2) + ", height" + (ellipse.RotatedRect.Size.Height / 2) +
                        ", angle=" + ellipse.RotatedRect.Angle + "\n";
                }
                throw new ToManyEllipsesFound("Not precise instructions enough.\n" + ellipseData);
            }
            Ellipse eli = chosenEllips[0];

            return eli;
        }

        /// <summary>
        /// Searches an image for ellipses or circles 
        /// </summary>
        /// <param name="inputImage">Image to be scanned for ellipses</param>
        private List<Ellipse> findEllipsesInImage(Image<Gray, Byte> inputImage)
        {
            


            Image<Gray, Byte> edgeImage = inputImage.Copy();
            Rectangle rect = new Rectangle(Point.Empty, inputImage.Size);
            IEnumerable<IEnumerable<Point>> edgeSets = FindLines.FindEdges(inputImage, rect, out edgeImage);

            List<Ellipse> ellipseList = new List<Ellipse>();
            List<PointF> pointList = new List<PointF>();

            foreach (IEnumerable<Point> ie in edgeSets)
            {

                foreach (Point p in ie)
                {
                    //Must convert to PointF for Ellipse Least Sqaure Fitting
                    PointF pf = new PointF(p.X, p.Y);
                    pointList.Add(pf);


                }
                //TODO: Decide what is crap and can be thrown away. This will be a suitable place
                if (pointList.Count > 5) //Min 5 points for Least Square 
                {
                    Ellipse ellipse = new Ellipse();
                    ellipse = PointCollection.EllipseLeastSquareFitting(pointList.ToArray());
                    ellipseList.Add(ellipse);
                }
                pointList.Clear();

            }

            return ellipseList;
        }