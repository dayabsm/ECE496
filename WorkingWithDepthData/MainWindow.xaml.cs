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
using System.Media;

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
        Queue<double> right_degrees = new Queue<double>();
        Queue<double> left_degrees = new Queue<double>();

        const int depthChangeToRegister = 20;
        const double MAX_DEGREE_DEVIATION = 75.0;   //maybe a percentage is more flexible
        const int MAX_QUEUE = 5;    //number of previous degrees to take average

        Queue<bool> right_palm = new Queue<bool>();
        Queue<bool> left_palm = new Queue<bool>();

        const int MAX_BOOL = 8;     //look for 4 closed palms, then 4 open palms

        const int OFFSET = 5;

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
        position leftHand;

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
                DepthImagePoint leftHandDepthPoint = depthFrame.MapFromSkeletonPoint(first.Joints[JointType.HandLeft].Position);

                DepthImagePoint rightElbowDepthPoint = depthFrame.MapFromSkeletonPoint(first.Joints[JointType.ElbowRight].Position);

                rightElbow.x = rightElbowDepthPoint.X;
                rightElbow.y = rightElbowDepthPoint.Y;

                head.x = headDepthPoint.X;
                head.y = headDepthPoint.Y;

                rightHand.x = rightHandDepthPoint.X;
                rightHand.y = rightHandDepthPoint.Y;
                leftHand.x = leftHandDepthPoint.X;
                leftHand.y = leftHandDepthPoint.Y;

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

                if (first.Joints[JointType.HandLeft].TrackingState == JointTrackingState.Inferred)
                {
                    // TODO: Handle the case where hand is not being used - what do we want to show in GUI?
                }

                else
                {
                    Rotate(cursorLeft, wheelLeft, angle, position);
                }

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

                

                if (first.Joints[JointType.HandRight].TrackingState == JointTrackingState.Inferred)
                {
                    // TODO: Handle the case where hand is not being used - what do we want to show in GUI?
                    minDepthBox.Text = "Hidden";
                }

                else
                {
                    minDepthBox.Text = "Left Angle: " + angleValLeft + " Right Angle: " + angleValRight;
                    Rotate(cursorRight, wheelRight, angle, position);
                }

                


               

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
                    
                    //if ( pixel.x < head.x)
                    if (Math.Abs(pixel.x - leftHand.x) < Math.Abs(pixel.x - rightHand.x))
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

            // Variables to remove body blob
            bool leftBlobFound = false;
            bool rightBlobFound = false;
            bool leftCaptureDone = false;
            bool rightCaptureDone = false;
        
            float lastYIndex_left = 0;
            float lastYIndex_right = 0;


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

               
                if (Math.Abs(pixel.x - leftHand.x) < Math.Abs(pixel.x - rightHand.x))
                // Depending on which side it is, choose the appropriate minDepth
                {
                    minDepth = minDepthLeft;
                }

                else
                {
                    // Find minDepth for Right Side
                    minDepth = minDepthRight;
                }

                // Give margin for hand - Make sure you copy any changes to 100 to Hand Contour
                // Need to also make sure that all points are in the same blob as the hand node
                if (((minDepth+ 60) > depth) && (depth >= (minDepth)))
                {
                    // the hand should be blue
                    pixels[colorIndex + BlueIndex] = 255;
                    pixels[colorIndex + GreenIndex] = 0;
                    pixels[colorIndex + RedIndex] = 0;

                    // Record Area of Hand
                    if (Math.Abs(pixel.x - leftHand.x) < Math.Abs(pixel.x - rightHand.x))
                    {
                        // We've found the first instance of the left hand
                        leftBlobFound = true;

                        if (!leftCaptureDone)
                        {
                            pixels[colorIndex + BlueIndex] = 255;
                            pixels[colorIndex + GreenIndex] = 0;
                            pixels[colorIndex + RedIndex] = 255;

                            leftHandArea++;
                            lastYIndex_left = pixel.y;
                            if (pixelIsOnContour(pixel, rawDepthData))
                            {
                                leftBlobFound = true;
                                contourLeft.Add(pixel);
                            }

                            
                            //// else if an entire row has no hand pixels, we have to stop
                            //if (leftBlobFound && (pixel.y > lastYIndex_left))
                            //{
                            //    // we are done capturing left blob
                            //    leftCaptureDone = true;
                            //}
                        }

                    }
                    else
                    {
                        if (!rightCaptureDone)
                        {
                            pixels[colorIndex + BlueIndex] = 255;
                            pixels[colorIndex + GreenIndex] = 0;
                            pixels[colorIndex + RedIndex] = 255;
                            rightHandArea++;
                            lastYIndex_right = pixel.y;
                            if (pixelIsOnContour(pixel, rawDepthData))
                            {
                                // Keep track of largest minimum value
                                rightBlobFound = true;
                                contourRight.Add(pixel);
                                
                            }
                            //// we have reached a pixel which is 10 indices away from the previous contour point
                            //else if (rightBlobFound && (pixel.y > lastYIndex_right + 10))
                            //{
                            //    // we are done capturing right blob
                            //    rightCaptureDone = true;
                            //}
                        }
                    }
                }

                // Once we reach the end of every row
                if (pixel.x >= numCol - 1)
                {
                    //Check if we found any new hand points in this row
                    if (leftBlobFound && lastYIndex_left < pixel.y)
                        leftCaptureDone = true;
                    if (rightBlobFound && lastYIndex_right < pixel.y)
                        rightCaptureDone = true;
                }   
            }

            leftHandAreaQ.Enqueue(leftHandArea);
            rightHandAreaQ.Enqueue(rightHandArea);

            // If hand area reduces by 1.5, we have a closed fist
            try
            {
                if ((isLeftHandOpen && (leftHandAreaQ.Peek() / leftHandArea > 1.5)) || !isLeftHandOpen)
                {
                    //minDepthBox.Foreground = Brushes.Green;
                    isLeftHandOpen = false;
                }

                // Closed Hand -> Open
                if ((!isLeftHandOpen && (leftHandAreaQ.Peek() / leftHandArea < 0.6)) || isLeftHandOpen)
                {
                    //minDepthBox.Foreground = Brushes.Red;
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
                if ((!isRightHandOpen && (rightHandAreaQ.Peek() / rightHandArea < 0.6)) || isRightHandOpen)
                {
                    minDepthBox.Foreground = Brushes.Red;
                    isRightHandOpen = true;
                }

                if (rightHandAreaQ.Count == 30)
                {
                    rightHandAreaQ.Dequeue();
                }
            }

            catch (DivideByZeroException e)
            {
                // New Area is equal to zero
                // Make no changes
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
            
            //if (pixel.x < head.x)
            if (Math.Abs(pixel.x - leftHand.x) < Math.Abs(pixel.x - rightHand.x))
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

        private bool check_palms(int position)
        {
            if (position % 2 == 0)    //right palm
            {
                if (right_palm.Count >= MAX_BOOL)
                {
                    right_palm.Dequeue();
                    right_palm.Enqueue(isRightHandOpen);
                }
                else
                {
                    right_palm.Enqueue(isRightHandOpen);
                    return false;
                }
                               
                if (right_palm.Count >= MAX_BOOL)
                {
                    int i = 1;
                    int flag = 0;
                    Queue<bool> copyRightPalm = new Queue<bool>(right_palm.ToArray());
                    foreach (bool current in copyRightPalm)      //looking for pattern of 4 false (palm closed), then 4 true (palm open)
                    {
                        if ((i == 1) && (current == false))
                        {
                            flag = flag + 1;
                        }
                        else if ((i == 2) && (current == false))
                        {
                            flag = flag + 2;
                        }
                        else if ((i == 3) && (current == false))
                        {
                            flag = flag + 3;
                        }
                        else if ((i == 4) && (current == false))
                        {
                            flag = flag + 4;
                        }
                        else if ((i == 5) && (current == true))
                        {
                            flag = flag + 5;
                        }
                        else if ((i == 6) && (current == true))
                        {
                            flag = flag + 6;
                        }
                        else if ((i == 7) && (current == true))
                        {
                            flag = flag + 7;
                        }
                        else if ((i == 8) && (current == true))
                        {
                            flag = flag + 8;
                        }
                        else
                        {
                            flag = flag + 0;
                        }
                        i = i + 1;
                    }

                    if (flag == 36) //only if every condition is met, the total sum must be 36, which means we detected a closing on opening of the palm
                    {
                        return true;
                    }
                    else
                    {
                        return false;

                    }

                }
            }
            else if (position % 2 == 1)    //left palm
            {
                if (left_palm.Count >= MAX_BOOL)
                {
                    left_palm.Dequeue();
                    left_palm.Enqueue(isLeftHandOpen);
                }
                else
                {
                    left_palm.Enqueue(isLeftHandOpen);
                    return false;
                }

                if (left_palm.Count >= MAX_BOOL)
                {
                    int i = 1;
                    int flag = 0;
                    Queue<bool> copyLeftPalm = new Queue<bool>(left_palm.ToArray());
                    foreach (bool current in copyLeftPalm)      //looking for pattern of 4 false (palm closed), then 4 true (palm open)
                    {
                        if ((i == 1) && (current == false))
                        {
                            flag = flag + 1;
                        }
                        else if ((i == 2) && (current == false))
                        {
                            flag = flag + 2;
                        }
                        else if ((i == 3) && (current == false))
                        {
                            flag = flag + 3;
                        }
                        else if ((i == 4) && (current == false))
                        {
                            flag = flag + 4;
                        }
                        else if ((i == 5) && (current == true))
                        {
                            flag = flag + 5;
                        }
                        else if ((i == 6) && (current == true))
                        {
                            flag = flag + 6;
                        }
                        else if ((i == 7) && (current == true))
                        {
                            flag = flag + 7;
                        }
                        else if ((i == 8) && (current == true))
                        {
                            flag = flag + 8;
                        }
                        else
                        {
                            flag = flag + 0;
                        }
                        i = i + 1;
                    }

                    if (flag == 36) //only if every condition is met, the total sum must be 36, which means we detected a closing on opening of the palm
                    {
                        return true;
                    }
                    else
                    {
                        return false;

                    }

                }

            }

            return false;   //error?
        }

        private void playAudio(int position, int angle)
        {
            int interval = 13;  //every letter has 13 degrees of seperation

            //==============Left Hand Middle Start==================//
            if ( (position==1)&&(angle>=OFFSET && angle<OFFSET+interval*1) )
            {
                try
                {
                    SoundPlayer audio = new SoundPlayer(Properties.Resources.z);
                    audio.Play();
                }
                catch (Exception ex)
                {
                    //do nothing
                }
            }
            if ( (position==1)&&(angle>=OFFSET+interval*1 && OFFSET+angle<interval*2) )
            {
                try
                {
                    SoundPlayer audio = new SoundPlayer(Properties.Resources.k);
                    audio.Play();
                }
                catch (Exception ex)
                {
                    //do nothing
                }
            }
            if ( (position==1)&&(angle>=OFFSET+interval*2 && OFFSET+angle<interval*3) )
            {
                try
                {
                    SoundPlayer audio = new SoundPlayer(Properties.Resources.y);
                    audio.Play();
                }
                catch (Exception ex)
                {
                    //do nothing
                }
            }
            if ( (position==1)&&(angle>=OFFSET+interval*3 && angle<OFFSET+interval*4) )
            {
                try
                {
                    SoundPlayer audio = new SoundPlayer(Properties.Resources.m);
                    audio.Play();
                }
                catch (Exception ex)
                {
                    //do nothing
                }
            }
            if ( (position==1)&&(angle>=OFFSET+interval*4 && angle<OFFSET+interval*5) )
            {
                try
                {
                    SoundPlayer audio = new SoundPlayer(Properties.Resources.d);
                    audio.Play();
                }
                catch (Exception ex)
                {
                    //do nothing
                }
            }
            if ( (position==1)&&(angle>=OFFSET+interval*5 && angle<OFFSET+interval*6) )
            {
                try
                {
                    SoundPlayer audio = new SoundPlayer(Properties.Resources.n);
                    audio.Play();
                }
                catch (Exception ex)
                {
                    //do nothing
                }
            }
            if ( (position==1)&&(angle>=OFFSET+interval*6 && angle<OFFSET+interval*7) )
            {
                try
                {
                    SoundPlayer audio = new SoundPlayer(Properties.Resources.e);
                    audio.Play();
                }
                catch (Exception ex)
                {
                    //do nothing
                }
            }
            if ( (position==1)&&(angle>=OFFSET+interval*7 && angle<OFFSET+interval*8) )
            {
                try
                {
                    SoundPlayer audio = new SoundPlayer(Properties.Resources.o);
                    audio.Play();
                }
                catch (Exception ex)
                {
                    //do nothing
                }
            }
            if ( (position==1)&&(angle>=OFFSET+interval*8 && angle<OFFSET+interval*9) )
            {
                try
                {
                    SoundPlayer audio = new SoundPlayer(Properties.Resources.h);
                    audio.Play();
                }
                catch (Exception ex)
                {
                    //do nothing
                }
            }
            if ( (position==1)&&(angle>=OFFSET+interval*9 && angle<OFFSET+interval*10) )
            {
                try
                {
                    SoundPlayer audio = new SoundPlayer(Properties.Resources.u);
                    audio.Play();
                }
                catch (Exception ex)
                {
                    //do nothing
                }
            }
            if ( (position==1)&&(angle>=OFFSET+interval*10 && angle<OFFSET+interval*11) )
            {
                try
                {
                    SoundPlayer audio = new SoundPlayer(Properties.Resources.f);
                    audio.Play();
                }
                catch (Exception ex)
                {
                    //do nothing
                }
            }
            if ( (position==1)&&(angle>=OFFSET+interval*11 && angle<OFFSET+interval*12) )
            {
                try
                {
                    SoundPlayer audio = new SoundPlayer(Properties.Resources.b);
                    audio.Play();
                }
                catch (Exception ex)
                {
                    //do nothing
                }
            }
            if ( (position==1)&&(angle>=OFFSET+interval*12 && angle<OFFSET+interval*13) )
            {
                try
                {
                    SoundPlayer audio = new SoundPlayer(Properties.Resources.x);
                    audio.Play();
                }
                catch (Exception ex)
                {
                    //do nothing
                }
            }
            //==============Left Hand Middle End==================//

            //==============Right Hand Middle Start==================//
            if ( (position==2)&&(angle>=OFFSET && angle<OFFSET+interval*1) )
            {
                try
                {
                    SoundPlayer audio = new SoundPlayer(Properties.Resources.q);
                    audio.Play();
                }
                catch (Exception ex)
                {
                    //do nothing
                }
            }
            if ( (position==2)&&(angle>=OFFSET+interval*1 && OFFSET+angle<interval*2) )
            {
                try
                {
                    SoundPlayer audio = new SoundPlayer(Properties.Resources.v);
                    audio.Play();
                }
                catch (Exception ex)
                {
                    //do nothing
                }
            }
            if ( (position==2)&&(angle>=OFFSET+interval*2 && OFFSET+angle<interval*3) )
            {
                try
                {
                    SoundPlayer audio = new SoundPlayer(Properties.Resources.g);
                    audio.Play();
                }
                catch (Exception ex)
                {
                    //do nothing
                }
            }
            if ( (position==2)&&(angle>=OFFSET+interval*3 && angle<OFFSET+interval*4) )
            {
                try
                {
                    SoundPlayer audio = new SoundPlayer(Properties.Resources.c);
                    audio.Play();
                }
                catch (Exception ex)
                {
                    //do nothing
                }
            }
            if ( (position==2)&&(angle>=OFFSET+interval*4 && angle<OFFSET+interval*5) )
            {
                try
                {
                    SoundPlayer audio = new SoundPlayer(Properties.Resources.r);
                    audio.Play();
                }
                catch (Exception ex)
                {
                    //do nothing
                }
            }
            if ( (position==2)&&(angle>=OFFSET+interval*5 && angle<OFFSET+interval*6) )
            {
                try
                {
                    SoundPlayer audio = new SoundPlayer(Properties.Resources.i);
                    audio.Play();
                }
                catch (Exception ex)
                {
                    //do nothing
                }
            }
            if ( (position==2)&&(angle>=OFFSET+interval*6 && angle<OFFSET+interval*7) )
            {
                try
                {
                    SoundPlayer audio = new SoundPlayer(Properties.Resources.t);
                    audio.Play();
                }
                catch (Exception ex)
                {
                    //do nothing
                }
            }
            if ( (position==2)&&(angle>=OFFSET+interval*7 && angle<OFFSET+interval*8) )
            {
                try
                {
                    SoundPlayer audio = new SoundPlayer(Properties.Resources.a);
                    audio.Play();
                }
                catch (Exception ex)
                {
                    //do nothing
                }
            }
            if ( (position==2)&&(angle>=OFFSET+interval*8 && angle<OFFSET+interval*9) )
            {
                try
                {
                    SoundPlayer audio = new SoundPlayer(Properties.Resources.s);
                    audio.Play();
                }
                catch (Exception ex)
                {
                    //do nothing
                }
            }
            if ( (position==2)&&(angle>=OFFSET+interval*9 && angle<OFFSET+interval*10) )
            {
                try
                {
                    SoundPlayer audio = new SoundPlayer(Properties.Resources.l);
                    audio.Play();
                }
                catch (Exception ex)
                {
                    //do nothing
                }
            }
            if ( (position==2)&&(angle>=OFFSET+interval*10 && angle<OFFSET+interval*11) )
            {
                try
                {
                    SoundPlayer audio = new SoundPlayer(Properties.Resources.w);
                    audio.Play();
                }
                catch (Exception ex)
                {
                    //do nothing
                }
            }
            if ( (position==2)&&(angle>=OFFSET+interval*11 && angle<OFFSET+interval*12) )
            {
                try
                {
                    SoundPlayer audio = new SoundPlayer(Properties.Resources.p);
                    audio.Play();
                }
                catch (Exception ex)
                {
                    //do nothing
                }
            }
            if ( (position==2)&&(angle>=OFFSET+interval*12 && angle<OFFSET+interval*13) )
            {
                try
                {
                    SoundPlayer audio = new SoundPlayer(Properties.Resources.j);
                    audio.Play();
                }
                catch (Exception ex)
                {
                    //do nothing
                }
            }
            //==============Right Hand Middle End==================//

            //==============Left Hand Low Start==================//
            //insert in here when need to change
            //==============Left Hand Low End==================//

            //==============Right Hand Low Start==================//
            //insert in here when need to change
            //==============Right Hand Low End==================//

            //==============Left Hand High Start==================//
            //insert in here when need to change
            //==============Left Hand High End==================//

            //==============Right Hand High Start==================//
            //insert in here when need to change
            //==============Right Hand High End==================//

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

