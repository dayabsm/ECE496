// (c) Copyright Microsoft Corporation.
// This source is subject to the Microsoft Public License (Ms-PL).
// Please see http://go.microsoft.com/fwlink/?LinkID=131993 for details.
// All other rights reserved.

using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Windows;
using System.Windows.Controls;
using System.Windows.Data;
using System.Windows.Documents;
using System.Windows.Input;
using System.Windows.Media;
using System.Windows.Media.Imaging;
using System.Windows.Navigation;
using System.Windows.Shapes;
using Microsoft.Kinect;
using System.Diagnostics;

using System.Drawing.Drawing2D;

namespace WorkingWithDepthData
{
        public struct position
    {
        public float x;
        public float y;

        public position(float xVal, float yVal)
        {
            x = xVal;
            y = yVal;
        }
    }

    /// <summary>
    /// Interaction logic for MainWindow.xaml
    /// </summary>
    ///
    public partial class MainWindow : Window
    {
        //-- added feb 18
        
        Queue<double> right_degrees = new Queue<double>();
        Queue<double> left_degrees = new Queue<double>();

        const int depthChangeToRegister = 20;
        const double MAX_DEGREE_DEVIATION = 75.0;   //maybe a percentage is more flexible
        const int MAX_QUEUE = 5;    //number of previous degrees to take average
        /*
        double[] right_degrees = new double[MAX_QUEUE];
        double[] left_degrees = new double[MAX_QUEUE];
        */
        //--

        public MainWindow()
        {
            InitializeComponent();
        }

        public static int angleRight = -1;
        public static int angleLeft = -1;

        int numCol;
        int numRows;
        int rightHandArea;
        int leftHandArea;

        const int skeletonCount = 6;
        Skeleton[] allSkeletons = new Skeleton[skeletonCount];
        position rightWrist;
        position leftWrist;
        position head;

        position rightElbow;
        
        position rightHand;
        int minDepthLeft = int.MaxValue;
        int minDepthRight = int.MaxValue;
        int prevMinDepthLeft = int.MaxValue;
        int prevMinDepthRight = int.MaxValue;
        int minDepth;

        bool isLeftHandOpen = true;
        bool isRightHandOpen = true;

        const float MaxDepthDistance = 4095; // max value returned
        const float MinDepthDistance = 850; // min value returned
        const float MaxDepthDistanceOffset = MaxDepthDistance - MinDepthDistance;

        Queue<int> leftHandAreaQ = new Queue<int>();
        Queue<int> rightHandAreaQ = new Queue<int>();
        private void Window_Loaded(object sender, RoutedEventArgs e)
        {
            kinectSensorChooser1.KinectSensorChanged += new DependencyPropertyChangedEventHandler(kinectSensorChooser1_KinectSensorChanged);
        }

        void kinectSensorChooser1_KinectSensorChanged(object sender, DependencyPropertyChangedEventArgs e)
        {

            var oldSensor = (KinectSensor)e.OldValue;

            //stop the old sensor
            if (oldSensor != null)
            {
                oldSensor.Stop();
                oldSensor.AudioSource.Stop();
            }

            // get the new sensor
            var newSensor = (KinectSensor)e.NewValue;
            if (newSensor == null)
            {
                return;
            }

            // turn on features that you need
            newSensor.DepthStream.Enable(DepthImageFormat.Resolution320x240Fps30); // changed the res here

            /// CHANGE CODE HERE IF NEEDED!!!!!!
            newSensor.SkeletonStream.Enable(/*new TransformSmoothParameters()
            {
                Smoothing = 0.75f,
                Correction = 0.0f,
                Prediction = 0.0f,
                JitterRadius = 0.05f,
                MaxDeviationRadius = 0.04f
            }*/);

            newSensor.AllFramesReady += new EventHandler<AllFramesReadyEventArgs>(newSensor_AllFramesReady);


            try
            {
                newSensor.Start();
                //newSensor.ElevationAngle = newSensor.MaxElevationAngle;
            }
            catch (System.IO.IOException)
            {
                // this happens if another app is using the Kinect
                kinectSensorChooser1.AppConflictOccurred();
            }
        }

        void newSensor_AllFramesReady(object sender, AllFramesReadyEventArgs e)
        {
            Skeleton first = GetFirstSkeleton(e);

            if (first == null)
            {
                return;
            }

            position rightFingerTips;
            position leftFingerTips;
            using (DepthImageFrame depthFrame = e.OpenDepthImageFrame())
            {
                if (depthFrame == null)
                {
                    return; 
                }

                DepthImagePoint rightWristDepthPoint = depthFrame.MapFromSkeletonPoint(first.Joints[JointType.WristRight].Position);
                DepthImagePoint leftWristDepthPoint = depthFrame.MapFromSkeletonPoint(first.Joints[JointType.WristLeft].Position);

                DepthImagePoint headDepthPoint = depthFrame.MapFromSkeletonPoint(first.Joints[JointType.Head].Position);

                DepthImagePoint rightHandDepthPoint = depthFrame.MapFromSkeletonPoint(first.Joints[JointType.HandRight].Position);
                DepthImagePoint rightElbowDepthPoint = depthFrame.MapFromSkeletonPoint(first.Joints[JointType.ElbowRight].Position);

                rightElbow.x = rightElbowDepthPoint.X;
                rightElbow.y = rightElbowDepthPoint.Y;

                head.x = headDepthPoint.X;
                head.y = headDepthPoint.Y;

                rightHand.x = rightHandDepthPoint.X;
                rightHand.y = rightHandDepthPoint.Y;

                byte[] pixels = GenerateColoredBytes(depthFrame, out leftFingerTips, out rightFingerTips);

                //number of bytes per row width * 4 (B,G,R,Empty)
                int stride = depthFrame.Width * 4;

                // Right Hand

                float xRight = (rightWrist.x - rightFingerTips.x);
                float yRight = (rightWrist.y - rightFingerTips.y);
               
                // TO USE Hand Skeletal Node INSTEAD - UNCOMMENT
                //float xRight = (rightWrist.x - rightHand.x);
                //float yRight = (rightWrist.y - rightHand.y);

                // TO USE ELBOW INSTEAD - UNCOMMENT
                //float xRight = (rightElbow.x - rightWrist.x);
                //float yRight = (rightElbow.y - rightWrist.y);

                double radiansRight = Math.Atan(yRight / xRight);
                int angleValRight = (int)(radiansRight * (180 / Math.PI));
                if (angleValRight < 0)
                {
                    angleValRight += 180;
                }

                // Left Hand
                float xLeft = (leftFingerTips.x - leftWrist.x);
                float yLeft = (leftFingerTips.y - leftWrist.y);

                double radiansLeft = Math.Atan(yLeft / xLeft);
                int angleValLeft = (int)(radiansLeft * (180 / Math.PI));
                if (angleValLeft < 0)
                {
                    angleValLeft += 180;
                }

                rightWrist.x = rightWristDepthPoint.X;
                rightWrist.y = rightWristDepthPoint.Y;

                leftWrist.x = leftWristDepthPoint.X;
                leftWrist.y = leftWristDepthPoint.Y;

                // Rotate Left Dial
                int position = 0;
                if (leftWrist.y < head.y)
                {
                    // Top Quadrant
                    position = 3;
                }

                else
                {
                    // Neutral
                    position = 1;
                }

                double angle = angleValLeft;

                textBoxLeft.Text = angle.ToString("#.##") + "°";
                //offset -> need to adjust probably
                angle = angle - 90;
                Rotate(cursorLeft, wheelLeft, angle, position);

                if (rightWrist.y < head.y)
                {
                    position = 4;
                }

                else
                {
                    position = 2;
                }

                angle = angleValRight;
                textBoxRight.Text = angle.ToString("#.##") + "°";
                //offset -> need to adjust probably
                angle = angle - 90;
                Rotate(cursorRight, wheelRight, angle, position);

                minDepthBox.Text = "Left Angle: " + angleValLeft + " Right Angle: " + angleValRight;

                //create image
                image1.Source =
                    BitmapSource.Create(depthFrame.Width, depthFrame.Height,
                    96, 96, PixelFormats.Bgr32, null, pixels, stride); 
            }
        }

        Skeleton GetFirstSkeleton(AllFramesReadyEventArgs e)
        {
            using (SkeletonFrame skeletonFrameData = e.OpenSkeletonFrame())
            {
                if (skeletonFrameData == null)
                {
                    return null;
                }

                skeletonFrameData.CopySkeletonDataTo(allSkeletons);

                // get the first tracked skeleton
                Skeleton first = (from s in allSkeletons
                                  where s.TrackingState == SkeletonTrackingState.Tracked
                                  select s).FirstOrDefault();
                return first;

            }
        }

        byte[] GenerateColoredBytes(DepthImageFrame depthFrame, out position leftCoords, out position rightCoords)
        {

            // Find Vector from Hand to Wrist -- Vector may not be correct tho.
            // You know which direction Hand is facing
            // Only select points that are at the correct side of the contour

            //get the raw data from kinect with the depth for every pixel
            short[] rawDepthData = new short[depthFrame.PixelDataLength];
            depthFrame.CopyPixelDataTo(rawDepthData);

            numRows = depthFrame.Height;
            numCol = depthFrame.Width;

            position rightFingerTip;
            position leftFingerTip;
            rightFingerTip.x = 0;
            rightFingerTip.y = 0;
            leftFingerTip.x = 0;
            leftFingerTip.y = 0;

            //use depthFrame to create the image to display on-screen
            //depthFrame contains color information for all pixels in image
            //Height x Width x 4 (Red, Green, Blue, empty byte)
            Byte[] pixels = new byte[depthFrame.Height * depthFrame.Width * 4];

            //hardcoded locations to Blue, Green, Red (BGR) index positions       
            const int BlueIndex = 0;
            const int GreenIndex = 1;
            const int RedIndex = 2;

            minDepthLeft = int.MaxValue;
            minDepthRight = int.MaxValue;
            //loop through all distances
            //pick a RGB color based on distance
            for (int depthIndex = 0, colorIndex = 0; 
                depthIndex < rawDepthData.Length && colorIndex < pixels.Length; 
                depthIndex++, colorIndex += 4)
            {
                //get the player (requires skeleton tracking enabled for values)
                int player = rawDepthData[depthIndex] & DepthImageFrame.PlayerIndexBitmask;

                //gets the depth value
                int depth = rawDepthData[depthIndex] >> DepthImageFrame.PlayerIndexBitmaskWidth;
                position pixel;
                pixel.x = depthIndex % numCol;
                pixel.y = (depthIndex - pixel.x) / numCol;

                if (depth > 0)
                {

                    if ( pixel.x < head.x)
                    {
                        // Finds the closest depth to sensor - Left
                        minDepthLeft = Math.Min(minDepthLeft, depth);
                    }

                    else
                    {
                        // Right
                        minDepthRight = Math.Min(minDepthRight, depth);
                    }
                }
            }

            /*if (Math.Abs(minDepthLeft - prevMinDepthLeft) > depthChangeToRegister)
            {
                if (minDepthBox.Foreground == Brushes.Red)
                {
                    minDepthBox.Foreground = Brushes.Green;
                }
                else
                {
                    minDepthBox.Foreground = Brushes.Red;
                }
            }*/

            prevMinDepthLeft = minDepthLeft;
            prevMinDepthRight = minDepthRight;

            //int rightHandDepthIndex = (int)(rightHand.y * numCol + rightHand.x);
            //if (rightHandDepthIndex < 76800)
            //{
            //    minDepthRight = rawDepthData[(int)(rightHand.y * numCol + rightHand.x)] >> DepthImageFrame.PlayerIndexBitmaskWidth;
            //}
            /*else
             * {
             * return???!
             * }
             */
            List<position> contourLeft = new List<position>();
            List<position> contourRight = new List<position>();
            leftHandArea = 0;
            rightHandArea = 0;
            for (int depthIndex = 0, colorIndex = 0;
                depthIndex < rawDepthData.Length && colorIndex < pixels.Length;
                depthIndex++, colorIndex += 4)
            {
                //get the player (requires skeleton tracking enabled for values)
                int player = rawDepthData[depthIndex] & DepthImageFrame.PlayerIndexBitmask;

                //gets the depth value
                int depth = rawDepthData[depthIndex] >> DepthImageFrame.PlayerIndexBitmaskWidth;

                position pixel;
                pixel.x = depthIndex % numCol;
                pixel.y = (depthIndex - pixel.x) / numCol;

                // Find minDepth for Left Side
                if (pixel.x < head.x)
                {
                    minDepth = minDepthLeft;
                    //pixels[colorIndex + BlueIndex] = 0;
                    //pixels[colorIndex + GreenIndex] = 0;
                    //pixels[colorIndex + RedIndex] = 0;
                }

                else
                {
                    // Find minDepth for Right Side
                    minDepth = minDepthRight;
                }

                // Give margin for hand - Make sure you copy any changes to 100 to Hand Contour
                if (((minDepth+ 60) > depth) && (depth >= (minDepth)))
                {
                    // the hand should be blue
                    pixels[colorIndex + BlueIndex] = 255;
                    pixels[colorIndex + GreenIndex] = 0;
                    pixels[colorIndex + RedIndex] = 0;

                    // Record Area of Hand
                    if (pixel.x < head.x)
                    {
                        leftHandArea++;
                        if (pixelIsOnContour(pixel, rawDepthData))
                            contourLeft.Add(pixel);
                    }
                    else
                    {
                        rightHandArea++;
                        if (pixelIsOnContour(pixel, rawDepthData))
                            contourRight.Add(pixel);
                    }
                    /*
                    // Calculate distance between pixel's x and y and wrist position
                    if (pixel.x < head.x)
                    {
                        // First Condition checks whether the pixel the pixel being checked is over the wrist (lower y coordinate)
                        if (pixel.y < leftWrist.y && findDistance(pixel, leftWrist) > distanceLeft)
                        {
                            // Left Hand
                            leftFingerTip.x = pixel.x;
                            leftFingerTip.y = pixel.y;
                            distanceLeft = findDistance(pixel, leftWrist);
                            pixels[colorIndex + BlueIndex] = 0;
                            pixels[colorIndex + GreenIndex] = 0;
                            pixels[colorIndex + RedIndex] = 255;
                        }
                    }

                    else
                    {
                        if (pixel.y < rightWrist.y && findDistance(pixel, rightWrist) > distanceRight)
                        {
                            // Right Hand
                            rightFingerTip.x = pixel.x;
                            rightFingerTip.y = pixel.y;
                            distanceRight = findDistance(pixel, rightWrist);
                            pixels[colorIndex + BlueIndex] = 0;
                            pixels[colorIndex + GreenIndex] = 0;
                            pixels[colorIndex + RedIndex] = 255;
                        }
                    }

                */    
                }   
            }

            leftHandAreaQ.Enqueue(leftHandArea);
            rightHandAreaQ.Enqueue(rightHandArea);

            // If hand area reduces by 1.5, we have a closed fist
            if ((isLeftHandOpen && (leftHandAreaQ.Peek()/leftHandArea > 1.5)) || !isLeftHandOpen)
            {
                minDepthBox.Foreground = Brushes.Green;
                isLeftHandOpen = false;
            }

            // Closed Hand -> Open
            if (!isLeftHandOpen && (leftHandAreaQ.Peek()/leftHandArea < 0.6) || isLeftHandOpen)
            {
                minDepthBox.Foreground = Brushes.Red;
                isLeftHandOpen = true;
            }

            if (leftHandAreaQ.Count == 30)
            {
                leftHandAreaQ.Dequeue();
            }

            // If hand area reduces by 1.5, we have a closed fist
            if ((isRightHandOpen && (rightHandAreaQ.Peek() / rightHandArea > 1.5)) || !isRightHandOpen)
            {
                minDepthBox.Foreground = Brushes.Green;
                isRightHandOpen = false;
            }

            // Closed Hand -> Open
            if (!isRightHandOpen && (rightHandAreaQ.Peek() / rightHandArea < 0.6) || isRightHandOpen)
            {
                minDepthBox.Foreground = Brushes.Red;
                isRightHandOpen = true;
            }

            if (rightHandAreaQ.Count == 30)
            {
                rightHandAreaQ.Dequeue();
            }

            // FIX THIS CRAP
            findLongestDistance(contourLeft, out leftWrist, out leftFingerTip);
            findLongestDistance(contourRight, out rightWrist, out rightFingerTip);

            // wrist is white
            int colorIndexD = (int)(rightWrist.y * numCol + rightWrist.x) * 4;
            try
            {
                pixels[colorIndexD + BlueIndex] = 255;
                pixels[colorIndexD + GreenIndex] = 255;
                pixels[colorIndexD + RedIndex] = 255;

                // Right Finger Tip is green
                colorIndexD = (int)(rightFingerTip.y * numCol + rightFingerTip.x) * 4;
                pixels[colorIndexD + BlueIndex] = 0;
                pixels[colorIndexD + GreenIndex] = 255;
                pixels[colorIndexD + RedIndex] = 0;
            }
            catch (Exception e)
            {
            }


            leftCoords.x = leftFingerTip.x;
            leftCoords.y = leftFingerTip.y;
            rightCoords.x = rightFingerTip.x;
            rightCoords.y = rightFingerTip.y;

          
            return pixels;
        }

        double findDistance(position a, position wrist)
        {
            return Math.Sqrt(Math.Pow((a.x - wrist.x), 2) + Math.Pow((a.y - wrist.y), 2));
        }

        bool pixelIsOnContour(position pixel, short[] depthData)
        {
            // At border, return false
            if (pixel.x >= 319 || pixel.x <= 0 || pixel.y >= 239 || pixel.y <= 0)
            {
                return false;
            }
            position pixelTop = new position(pixel.x, pixel.y - 1);
            position pixelTopLeft = new position(pixel.x -1, pixel.y - 1);
            position pixelTopRight= new position(pixel.x+1, pixel.y - 1);
            
            position pixelLeft= new position(pixel.x-1, pixel.y);
            position pixelRight= new position(pixel.x+1, pixel.y);

            position pixelBottom= new position(pixel.x, pixel.y + 1);
            position pixelBottomLeft=new position(pixel.x-1, pixel.y + 1);
            position pixelBottomRight=new position(pixel.x+1, pixel.y + 1);

            if (CheckBorder(pixelTop, depthData) || CheckBorder(pixelTopLeft, depthData) || CheckBorder(pixelTopRight, depthData) 
                || CheckBorder(pixelBottom, depthData) || CheckBorder(pixelBottomLeft, depthData) || CheckBorder(pixelBottomRight, depthData)
                || CheckBorder(pixelLeft, depthData) || CheckBorder(pixelRight, depthData))
            {
                return true;
            }

            return false;
        }

        // Returns true if pixel is NOT on hand
        bool CheckBorder(position pixel, short [] rawDepthData)
        {
            int threshold = 70;
            // Convert pixel's neighbor locations
            int neighborDepthIndex;
            neighborDepthIndex = (int) (pixel.y*numCol + pixel.x); // Add modifications to convert this to depth index


            int depth = rawDepthData[neighborDepthIndex] >> DepthImageFrame.PlayerIndexBitmaskWidth;
            
            if (pixel.x < head.x)
            {
                minDepth = minDepthLeft;
            }

            else
            {
                minDepth = minDepthRight;
            }

            if (depth - minDepth > threshold)
            {
                // The current point is NOT on the hand
                return true;
            }
            return false;
        }

        void findLongestDistance(List<position> contour, out position wrist, out position fingertip)
        {
            double distance = 0;

            wrist.x = -1;
            wrist.y = -1;

            fingertip.x = -1;
            fingertip.y = -1;
            
            for (int i=0; i < contour.Count; i++)
            {
                for (int j = 0; j < contour.Count; j++)
                {
                    double testDistance = findDistance(contour[i], contour[j]);
                    if (distance < testDistance)
                    {
                        distance = testDistance;
                        if (contour[i].y < contour[j].y)
                        {
                            wrist = contour[j];
                            fingertip = contour[i];
                        }
                        else
                        {
                            wrist = contour[i];
                            fingertip = contour[j];
                        }
                    }
                }
            }
        }

        public static byte CalculateIntensityFromDepth(int distance)
        {
            //formula for calculating monochrome intensity for histogram
            return (byte)(255 - (255 * Math.Max(distance - MinDepthDistance, 0) 
                / (MaxDepthDistanceOffset)));
        }

        private void Window_Closing(object sender, System.ComponentModel.CancelEventArgs e)
        {
            StopKinect(kinectSensorChooser1.Kinect); 
        }

        private void StopKinect(KinectSensor sensor)
        {
            if (sensor != null)
            {
                if (sensor.IsRunning)
                {
                    //stop sensor 
                    sensor.Stop();

                    //stop audio if not null
                    if (sensor.AudioSource != null)
                    {
                        sensor.AudioSource.Stop();
                    }


                }
            }
        }

        private void minDepthBox_TextChanged(object sender, TextChangedEventArgs e)
        {
           
        }

        private void sliderLeft_ValueChanged(object sender, RoutedPropertyChangedEventArgs<double> e)
        {
            int position = 0;
            if (radioButtonLeft1.IsChecked == true)
                position = 1;
            if (radioButtonLeft2.IsChecked == true)
                position = 3;
            double angle = sliderLeft.Value;

            textBoxLeft.Text = angle.ToString("#.##") + "°";
            //offset -> need to adjust probably
            angle = angle - 90;
            
            Rotate(cursorLeft, wheelLeft, angle, position);
        }

        private void sliderRight_ValueChanged_1(object sender, RoutedPropertyChangedEventArgs<double> e)
        {
            int position = 0;
            if (radioButtonLeft1.IsChecked == true)
                position = 2;
            if (radioButtonLeft2.IsChecked == true)
                position = 4;
            double angle = sliderRight.Value;

            textBoxRight.Text = angle.ToString("#.##") + "°";
            //offset -> need to adjust probably
            angle = angle - 90;

            Rotate(cursorRight, wheelRight, angle, position);
        }

        //last change feb 18
        private void Rotate(Image cursor, Image wheel, double angle, int position)
        {
            // !! Use isLeftHandOpen and isRightHandOpen to detect closed fists!!
            BitmapImage newWheel = new BitmapImage();
            newWheel.BeginInit();
            if (position == 1)
                newWheel.UriSource = new Uri(@"pack://application:,,,/images/layout1.png", UriKind.Absolute);
            else if (position == 2)
                newWheel.UriSource = new Uri(@"pack://application:,,,/images/layout2.png", UriKind.Absolute);
            else if (position == 3)
                newWheel.UriSource = new Uri(@"pack://application:,,,/images/layout3.png", UriKind.Absolute);
            else if (position == 4)
                newWheel.UriSource = new Uri(@"pack://application:,,,/images/layout4.png", UriKind.Absolute);
            else
                newWheel.UriSource = new Uri(@"pack://application:,,,/images/layout1.png", UriKind.Absolute);
            newWheel.CacheOption = BitmapCacheOption.OnLoad;
            newWheel.EndInit();
            wheel.Source = newWheel;
            //wheel.RenderTransform = new RotateTransform(angle);   //rotate wheel instead   

            double new_angle = check_angles(angle, position);
            if (new_angle != -999.999)
            {
                double rounded_angle = Math.Round(new_angle, 0);
                cursor.RenderTransform = new RotateTransform(rounded_angle);
            }
            
        }

        //last change feb 18
        /*
        private double check_angles(double angle, int position)
        {
            if (right_degrees[0] == -999.999) //empty
            {
                right_degrees[0] = angle;
                return angle;
            }
            else
            {
                //shift array by 1
                for (x = 0; x < MAX_QUEUE-1; x ++)
            }
        }
         */
        
        private double check_angles(double angle, int position)
        {
            //return angle;
            if (position % 2 == 0)  //even, position 2,4
            {
                //if there is MAX_QUEUE or more queues, dequeue 1, and enqueue the current angle                
                if (right_degrees.Count >= MAX_QUEUE)
                {
                    //check the current average of the previous degrees
                    double current_average = 0.0;
                    Queue<double> copyQueueR1 = new Queue<double>(right_degrees.ToArray());
                    foreach (double current in copyQueueR1)
                    {
                        current_average = current_average + current;
                    }
                    current_average = current_average / right_degrees.Count;

                    //check range of the current average and compare with the given angle
                    if (angle < current_average + MAX_DEGREE_DEVIATION && angle > current_average - MAX_DEGREE_DEVIATION)
                    {
                        right_degrees.Dequeue();
                        right_degrees.Enqueue(angle);
                    }
                    else
                    {
                        return -999.999;
                    }                        
                }
                else //else there is less than MAX_QUEUE, so enqueue current angle
                {
                    right_degrees.Enqueue(angle);
                }
                //calculate the average degree and returns
                double current_degree = 0.0;
                Queue<double> copyQueueR2 = new Queue<double>(right_degrees.ToArray());
                foreach (double number in copyQueueR2)
                {
                    current_degree = current_degree + number;
                }
                double new_average = current_degree / right_degrees.Count;
                return new_average;
            }
            else if (position % 2 == 1) //position 1,3
            {
                //if there is MAX_QUEUE or more queues, dequeue 1, and enqueue the current angle                
                if (left_degrees.Count >= MAX_QUEUE)
                {
                    //check the current average of the previous degrees
                    double current_average = 0.0;
                    Queue<double> copyQueueL1 = new Queue<double>(left_degrees.ToArray());
                    foreach (double current in copyQueueL1)
                    {
                        current_average = current_average + current;
                    }
                    current_average = current_average / left_degrees.Count;

                    //check range of the current average and compare with the given angle
                    if (angle < current_average + MAX_DEGREE_DEVIATION && angle > current_average - MAX_DEGREE_DEVIATION)
                    {
                        left_degrees.Dequeue();
                        left_degrees.Enqueue(angle);
                    }
                    else
                    {
                        return -999.999;
                    }
                }
                else //else there is less than MAX_QUEUE, so enqueue current angle
                {
                    left_degrees.Enqueue(angle);
                }
                //calculate the average degree and returns
                double current_degree = 0.0;
                Queue<double> copyQueueL2 = new Queue<double>(left_degrees.ToArray());
                foreach (double number in copyQueueL2)
                {
                    current_degree = current_degree + number;
                }
                double new_average = current_degree / left_degrees.Count;
                return new_average;
            }
            else
            {
                return -999.999;
            }
        }
        
        private void enableTestUI_Click(object sender, RoutedEventArgs e)
        {
            if (enableTestUI.IsChecked == true)
            {
                testUI.Visibility = Visibility.Visible;
                actualUI.Visibility = Visibility.Hidden;
            }
            else
            {
                testUI.Visibility = Visibility.Hidden;
                actualUI.Visibility = Visibility.Visible;
            }
        }
    }
}

