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
using System.Windows.Forms;

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

        bool right_pressed = false;
        bool left_pressed = false;

        Queue<int> previous_right_degrees = new Queue<int>();
        Queue<int> previous_left_degrees = new Queue<int>();

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

                bool leftHandHidden = false;
                bool rightHandHidden = false;

                if ((first.Joints[JointType.HandLeft].TrackingState == JointTrackingState.Inferred) || (first.Joints[JointType.HandLeft].TrackingState == JointTrackingState.NotTracked))
                {
                    leftHandHidden = true;
                }

                else
                {
                    leftHandHidden = false;
                }

                if ((first.Joints[JointType.HandRight].TrackingState == JointTrackingState.Inferred) || (first.Joints[JointType.HandRight].TrackingState == JointTrackingState.NotTracked))
                {
                    rightHandHidden = true;
                }

                else
                {
                    rightHandHidden = false;
                }

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

                //textBoxLeft.Text = angle.ToString("#.##") + "°";
                //offset -> need to adjust probably
                angle = angle - 90;


                if (rightHandHidden && leftHandHidden)
                {
                    minDepthBox.Text = "Both Hands Hidden";
                }

                else if (rightHandHidden)
                {
                    minDepthBox.Text = "Right Hand Hidden";
                }

                else if (leftHandHidden)
                {
                    minDepthBox.Text = "Left Hand Hidden";
                }
                
                //if (!leftHandHidden)
                //{
                //    Rotate(cursorLeft, wheelLeft, angle, position);
                //}
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
                //textBoxRight.Text = angle.ToString("#.##") + "°";
                //offset -> need to adjust probably
                angle = angle - 90;

                if (!rightHandHidden)
                {
                    Rotate(cursorRight, wheelRight, angle, position);
                }


                if (!rightHandHidden && !leftHandHidden)
                {
                    minDepthBox.Text = "Left: " + angleValLeft + ", Right: " + angleValRight;
                }

               

                //create image
                image1.Source =
                    BitmapSource.Create(depthFrame.Width, depthFrame.Height,
                    96, 96, PixelFormats.Bgr32, null, pixels, stride);
                //create image
                image2.Source =
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
                                // change back to true if we want trosoReduction (TM)
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
                                // change back to true if we want trosoReduction (TM)
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

                // UNCOMMENT THIS FOR TORSO-REDUCTOMETER
                // Once we reach the end of every row
                /*if (pixel.x >= numCol - 1)
                {
                    //Check if we found any new hand points in this row
                    if (leftBlobFound && lastYIndex_left < pixel.y)
                        leftCaptureDone = true;
                    if (rightBlobFound && lastYIndex_right < pixel.y)
                        rightCaptureDone = true;
                }*/
            }

            string toWrite = " " + rightHandArea + "\n";

            //Append new text to an existing file 
            /*using (System.IO.StreamWriter file = new System.IO.StreamWriter(@"C:\Users\Daya\Desktop\areaTrace.txt", true))
            {
                file.WriteLine(toWrite);
            } */
            

            //--------------------FOR AREA UNCOMMENT FROM HERE ----------------------------------

            //leftHandAreaQ.Enqueue(leftHandArea);
            //rightHandAreaQ.Enqueue(rightHandArea);

            //// If hand area reduces by 1.5, we have a closed fist
            //try
            //{
            //    if ((isLeftHandOpen && (leftHandAreaQ.Peek() / leftHandArea > 1.3) /*&& (leftHandAreaQ.Peek() / leftHandArea < 100)*/) || !isLeftHandOpen)
            //    {
            //        //minDepthBox.Foreground = Brushes.Green;
            //        isLeftHandOpen = false;
            //    }

            //    // Closed Hand -> Open
            //    if ((!isLeftHandOpen && (leftHandAreaQ.Peek() / leftHandArea < 0.6) /*&& (leftHandAreaQ.Peek() / leftHandArea > 0.2)*/) || isLeftHandOpen)
            //    {
            //        //minDepthBox.Foreground = Brushes.Red;
            //        isLeftHandOpen = true;
            //    }

            //    // Once it fills to 30 frames, we are dequeuing, this might be the problem?
            //    if (leftHandAreaQ.Count == 30)
            //    {
            //        leftHandAreaQ.Dequeue();
            //    }

            //    // If hand area reduces by 1.5, we have a closed fist
            //    // PROBLEM MIGHT BE WITH THE SECOND CONDITION - NOT SURE
            //    if ((isRightHandOpen && (rightHandAreaQ.Peek() / rightHandArea > 1.5)/* && (rightHandAreaQ.Peek() / rightHandArea < 100)*/) || !isRightHandOpen)
            //    {
            //        minDepthBox.Foreground = Brushes.Green;
            //        isRightHandOpen = false;
            //    }

            //    // Closed Hand -> Open
            //    if ((!isRightHandOpen && (rightHandAreaQ.Peek() / rightHandArea < 0.6) /*&& (rightHandAreaQ.Peek() / rightHandArea > 0.2)*/) || isRightHandOpen)
            //    {
            //        minDepthBox.Foreground = Brushes.Red;
            //        isRightHandOpen = true;
            //    }

            //    if (rightHandAreaQ.Count == 30)
            //    {
            //        rightHandAreaQ.Dequeue();
            //    }
            //}

            //catch (DivideByZeroException e)
            //{
            //    // New Area is equal to zero
            //    // Make no changes
            //}

            //--------------------FOR AREA UNCOMMENT TO HERE ----------------------------------
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
            if (radioButtonLeft3.IsChecked == true)
                position = 5;
            double angle = sliderLeft.Value;

            //textBoxLeft.Text = angle.ToString("#.##") + "°";
            //offset -> need to adjust probably
            angle = angle - 90;

            Rotate(cursorLeft, wheelLeft, angle, position);
        }

        private void sliderRight_ValueChanged_1(object sender, RoutedPropertyChangedEventArgs<double> e)
        {
            int position = 0;
            if (radioButtonRight1.IsChecked == true)
                position = 2;
            if (radioButtonRight2.IsChecked == true)
                position = 4;
            if (radioButtonRight3.IsChecked == true)
                position = 6;
            double angle = sliderRight.Value;

            //textBoxRight.Text = angle.ToString("#.##") + "°";
            //offset -> need to adjust probably
            angle = angle - 90;

            Rotate(cursorRight, wheelRight, angle, position);
        }

        private void Rotate(Image cursor, Image wheel, double angle, int position)
        {
            BitmapImage newWheel = new BitmapImage();
            newWheel.BeginInit();
            if (position == 1)
                newWheel.UriSource = new Uri(@"pack://application:,,,/images/left_med.png", UriKind.Absolute);
            else if (position == 2)
                newWheel.UriSource = new Uri(@"pack://application:,,,/images/right_med.png", UriKind.Absolute);
            else if (position == 3)
                newWheel.UriSource = new Uri(@"pack://application:,,,/images/left_low.png", UriKind.Absolute);
            else if (position == 4)
                newWheel.UriSource = new Uri(@"pack://application:,,,/images/right_low.png", UriKind.Absolute);
            else if (position == 5)
                newWheel.UriSource = new Uri(@"pack://application:,,,/images/left_high.png", UriKind.Absolute);
            else if (position == 6)
                newWheel.UriSource = new Uri(@"pack://application:,,,/images/right_high.png", UriKind.Absolute);
            else
                newWheel.UriSource = new Uri(@"pack://application:,,,/images/left_med.png", UriKind.Absolute);  //make a empty layout
            newWheel.CacheOption = BitmapCacheOption.OnLoad;
            newWheel.EndInit();
            wheel.Source = newWheel;

            /*
            Bitmap newWheel = new Bitmap(Properties.Resources.left_med);
            if (position == 1)
                newWheel = new Bitmap(Properties.Resources.left_med);
            else if (position == 2)
                newWheel = new Bitmap(Properties.Resources.right_med);
            else if (position == 3)
                newWheel = new Bitmap(Properties.Resources.left_low);
            else if (position == 4)
                newWheel = new Bitmap(Properties.Resources.right_low);
            else if (position == 5)
                newWheel = new Bitmap(Properties.Resources.left_high);
            else if (position == 6)
                newWheel = new Bitmap(Properties.Resources.right_high);
            */

            bool new_palm = check_palms(position);

            bool current_palm = false;
            if (position % 2 == 0)
                current_palm = isRightHandOpen;
            else
                current_palm = isLeftHandOpen;

            if (current_palm == true)   //if palm is open, check the angle and rotate
            {
                double new_angle = check_angles(angle, position);
                if (new_angle != -999.999)
                {
                    double rounded_angle = Math.Round(new_angle, 0);
                    cursor.RenderTransform = new RotateTransform(rounded_angle);

                    if (position % 2 == 0)
                    {
                        if (previous_right_degrees.Count >= MAX_BOOL)
                        {
                            previous_right_degrees.Dequeue();
                            previous_right_degrees.Enqueue((int)rounded_angle + 90);
                        }
                        else
                        {
                            previous_right_degrees.Enqueue((int)rounded_angle + 90);
                        }
                    }
                    else
                    {
                        if (previous_left_degrees.Count >= MAX_BOOL)
                        {
                            previous_left_degrees.Dequeue();
                            previous_left_degrees.Enqueue((int)rounded_angle + 90);
                        }
                        else
                        {
                            previous_left_degrees.Enqueue((int)rounded_angle + 90);
                        }
                    }

                    //playAudio(position, rounded_angle);
                    //playAudio(position, (int)rounded_angle + 90);        
                    if (position % 2 == 0)
                    {
                        if (previous_right_degrees.Count > 0)
                            playAudio(position, previous_right_degrees.Peek());
                        else
                            playAudio(position, (int)rounded_angle + 90);

                        if (right_pressed == true)
                        {
                            //insert letter
                            textBox1.Text = textBox1.Text + textBoxRight.Text;
                            right_pressed = false;
                        }
                    }
                    else
                    {
                        if (previous_left_degrees.Count > 0)
                            playAudio(position, previous_left_degrees.Peek());
                        else
                            playAudio(position, (int)rounded_angle + 90);

                        if (left_pressed == true)
                        {
                            //insert letter
                            textBox1.Text = textBox1.Text + textBoxLeft.Text;
                            left_pressed = false;
                        }
                    }
                }
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
            //GUI display
            try
            {
                if (position % 2 == 0)
                {
                    labelRight.Content = isRightHandOpen.ToString();
                }
                else
                {
                    labelLeft.Content = isLeftHandOpen.ToString();
                }
            }
            catch
            {
                //do nothing
            }

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
                    labelRightFlag.Content = flag.ToString();
                    if (flag == 10) //only if every condition is met, the total sum must be 36, which means we detected a closing on opening of the palm
                    {
                        
                        SoundPlayer audio = new SoundPlayer(Properties.Resources.confirm);
                        audio.Play();
                        right_pressed = true;
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
                    labelLeftFlag.Content = flag.ToString();

                    if (flag == 10) //only if every condition is met, the total sum must be 36, which means we detected a closing on opening of the palm
                    {
                        SoundPlayer audio = new SoundPlayer(Properties.Resources.confirm);
                        audio.Play();
                        left_pressed = true;
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

        private string playAudio(int position, int angle)
        {
            int interval = 13;  //every letter has 13 degrees of seperation
            SoundPlayer audio;
            string return_letter = "";
            audio = new SoundPlayer(Properties.Resources.confirm);  //set to something to just make it compile
            //==============Left Hand Middle Start==================//
            if ((position == 1) && (angle >= OFFSET && angle < OFFSET + interval * 1))
            {
                textBoxLeft.Text = "z";
                audio = new SoundPlayer(Properties.Resources.z);
                return_letter = "z";
            }
            if ((position == 1) && ((angle >= OFFSET + interval * 1) && angle < OFFSET + interval * 2))
            {
                textBoxLeft.Text = "z";
                audio = new SoundPlayer(Properties.Resources.k);
                return_letter = "k";                
            }
            if ((position == 1) && (angle >= OFFSET + interval * 2 && angle < OFFSET + interval * 3))
            {
                textBoxLeft.Text = "y";
                audio = new SoundPlayer(Properties.Resources.y);
                return_letter = "y";                
            }
            if ((position == 1) && (angle >= OFFSET + interval * 3 && angle < OFFSET + interval * 4))
            {
                textBoxLeft.Text = "y";
                audio = new SoundPlayer(Properties.Resources.m);
                return_letter = "m";                 
            }
            if ((position == 1) && (angle >= OFFSET + interval * 4 && angle < OFFSET + interval * 5))
            {
                textBoxLeft.Text = "d";
                audio = new SoundPlayer(Properties.Resources.d);
                return_letter = ""; 
            }
            if ((position == 1) && (angle >= OFFSET + interval * 5 && angle < OFFSET + interval * 6))
            {
                textBoxLeft.Text = "d";
                audio = new SoundPlayer(Properties.Resources.n);
                return_letter = "";
            }
            if ((position == 1) && (angle >= OFFSET + interval * 6 && angle < OFFSET + interval * 7))
            {
                textBoxLeft.Text = "e";
                audio = new SoundPlayer(Properties.Resources.e);
                return_letter = "";
            }
            if ((position == 1) && (angle >= OFFSET + interval * 7 && angle < OFFSET + interval * 8))
            {
                textBoxLeft.Text = "e";
                audio = new SoundPlayer(Properties.Resources.o);
                return_letter = "";
            }
            if ((position == 1) && (angle >= OFFSET + interval * 8 && angle < OFFSET + interval * 9))
            {
                textBoxLeft.Text = "h";
                audio = new SoundPlayer(Properties.Resources.h);
                return_letter = "";
            }
            if ((position == 1) && (angle >= OFFSET + interval * 9 && angle < OFFSET + interval * 10))
            {
                textBoxLeft.Text = "h";
                audio = new SoundPlayer(Properties.Resources.u);
                return_letter = "";
            }
            if ((position == 1) && (angle >= OFFSET + interval * 10 && angle < OFFSET + interval * 11))
            {
                textBoxLeft.Text = "f";
                audio = new SoundPlayer(Properties.Resources.f);
                return_letter = "";
            }
            if ((position == 1) && (angle >= OFFSET + interval * 11 && angle < OFFSET + interval * 12))
            {
                textBoxLeft.Text = "f";
                audio = new SoundPlayer(Properties.Resources.b);
                return_letter = "";
            }
            if ((position == 1) && (angle >= OFFSET + interval * 12 && angle < OFFSET + interval * 13))
            {
                textBoxLeft.Text = "z";
                audio = new SoundPlayer(Properties.Resources.x);
                return_letter = "";
            }
            //==============Left Hand Middle End==================//

            //==============Right Hand Middle Start==================//
            if ((position == 2) && (angle >= OFFSET && angle < OFFSET + interval * 1))
            {
                textBoxRight.Text = "q";
                audio = new SoundPlayer(Properties.Resources.q);
                return_letter = "";
            }
            if ((position == 2) && (angle >= OFFSET + interval * 1 &&  angle < OFFSET + interval * 2))
            {
                textBoxRight.Text = "v";
                audio = new SoundPlayer(Properties.Resources.v);
                return_letter = "";
            }
            if ((position == 2) && (angle >= OFFSET + interval * 2 && angle < OFFSET + interval * 3))
            {
                textBoxRight.Text = "g";
                audio = new SoundPlayer(Properties.Resources.g);
                return_letter = "";
            }
            if ((position == 2) && (angle >= OFFSET + interval * 3 && angle < OFFSET + interval * 4))
            {
                textBoxRight.Text = "c";
                audio = new SoundPlayer(Properties.Resources.c);
                return_letter = "";
            }
            if ((position == 2) && (angle >= OFFSET + interval * 4 && angle < OFFSET + interval * 5))
            {
                textBoxRight.Text = "r";
                audio = new SoundPlayer(Properties.Resources.r);
                return_letter = "";
            }
            if ((position == 2) && (angle >= OFFSET + interval * 5 && angle < OFFSET + interval * 6))
            {
                textBoxRight.Text = "i";
                audio = new SoundPlayer(Properties.Resources.i);
                return_letter = "";
            }
            if ((position == 2) && (angle >= OFFSET + interval * 6 && angle < OFFSET + interval * 7))
            {
                textBoxRight.Text = "t";
                audio = new SoundPlayer(Properties.Resources.t);
                return_letter = "";
            }
            if ((position == 2) && (angle >= OFFSET + interval * 7 && angle < OFFSET + interval * 8))
            {
                textBoxRight.Text = "a";
                audio = new SoundPlayer(Properties.Resources.a);
                return_letter = "";
            }
            if ((position == 2) && (angle >= OFFSET + interval * 8 && angle < OFFSET + interval * 9))
            {
                textBoxRight.Text = "s";
                audio = new SoundPlayer(Properties.Resources.s);
                return_letter = "";
            }
            if ((position == 2) && (angle >= OFFSET + interval * 9 && angle < OFFSET + interval * 10))
            {
                textBoxRight.Text = "l";
                audio = new SoundPlayer(Properties.Resources.l);
                return_letter = "";
            }
            if ((position == 2) && (angle >= OFFSET + interval * 10 && angle < OFFSET + interval * 11))
            {
                textBoxRight.Text = "w";
                audio = new SoundPlayer(Properties.Resources.w);
                return_letter = "";
            }
            if ((position == 2) && (angle >= OFFSET + interval * 11 && angle < OFFSET + interval * 12))
            {
                textBoxRight.Text = "p";
                audio = new SoundPlayer(Properties.Resources.p);
                return_letter = "";
            }
            if ((position == 2) && (angle >= OFFSET + interval * 12 && angle < OFFSET + interval * 13))
            {
                textBoxRight.Text = "j";
                audio = new SoundPlayer(Properties.Resources.j);
                return_letter = "";
            }
            //==============Right Hand Middle End==================//

            //==============Left Hand Low Start==================//
            if ((position == 3) && (angle >= OFFSET && angle < OFFSET + interval * 1))
            {
                textBoxLeft.Text = "(";
                audio = new SoundPlayer(Properties.Resources.open_bracket);
                return_letter = "";
            }
            if ((position == 3) && (angle >= OFFSET + interval * 1 && angle < OFFSET + interval * 2))
            {
                textBoxLeft.Text = "0";
                audio = new SoundPlayer(Properties.Resources._0);
                return_letter = "";
            }
            if ((position == 3) && (angle >= OFFSET + interval * 2 && angle < OFFSET + interval * 3))
            {
                textBoxLeft.Text = "1";
                audio = new SoundPlayer(Properties.Resources._1);
                return_letter = "";
            }
            if ((position == 3) && (angle >= OFFSET + interval * 3 && angle < OFFSET + interval * 4))
            {
                textBoxLeft.Text = "2";
                audio = new SoundPlayer(Properties.Resources._2);
                return_letter = "";
            }
            if ((position == 3) && (angle >= OFFSET + interval * 4 && angle < OFFSET + interval * 5))
            {
                textBoxLeft.Text = "3";
                audio = new SoundPlayer(Properties.Resources._3);
                return_letter = "";
            }
            if ((position == 3) && (angle >= OFFSET + interval * 5 && angle < OFFSET + interval * 6))
            {
                textBoxLeft.Text = "4";
                audio = new SoundPlayer(Properties.Resources._4);
                return_letter = "";
            }
            if ((position == 3) && (angle >= OFFSET + interval * 6 && angle < OFFSET + interval * 7))
            {
                textBoxLeft.Text = "space";
                audio = new SoundPlayer(Properties.Resources.space);
                return_letter = "";
            }
            if ((position == 3) && (angle >= OFFSET + interval * 7 && angle < OFFSET + interval * 8))
            {
                textBoxLeft.Text = "5";
                audio = new SoundPlayer(Properties.Resources._5);
                return_letter = "";
            }
            if ((position == 3) && (angle >= OFFSET + interval * 8 && angle < OFFSET + interval * 9))
            {
                textBoxLeft.Text = "6";
                audio = new SoundPlayer(Properties.Resources._6);
                return_letter = "";
            }
            if ((position == 3) && (angle >= OFFSET + interval * 9 && angle < OFFSET + interval * 10))
            {
                textBoxLeft.Text = "7";
                audio = new SoundPlayer(Properties.Resources._7);
                return_letter = "";
            }
            if ((position == 3) && (angle >= OFFSET + interval * 10 && angle < OFFSET + interval * 11))
            {
                textBoxLeft.Text = "8";
                audio = new SoundPlayer(Properties.Resources._8);
                return_letter = "";
            }
            if ((position == 3) && (angle >= OFFSET + interval * 11 && angle < OFFSET + interval * 12))
            {
                textBoxLeft.Text = "9";
                audio = new SoundPlayer(Properties.Resources._9);
                return_letter = "";
            }
            if ((position == 3) && (angle >= OFFSET + interval * 12 && angle < OFFSET + interval * 13))
            {
                textBoxLeft.Text = ")";
                audio = new SoundPlayer(Properties.Resources.close_bracket);
                return_letter = "";
            }
            //==============Left Hand Low End==================//

            //==============Right Hand Low Start==================//
            if ((position == 4) && (angle >= OFFSET && angle < OFFSET + interval * 1))
            {
                textBoxRight.Text = "`";
                audio = new SoundPlayer(Properties.Resources.graveaccent);
                return_letter = "";
            }
            if ((position == 4) && (angle >= OFFSET + interval * 1 && angle < OFFSET + interval * 2))
            {
                textBoxRight.Text = "&";
                audio = new SoundPlayer(Properties.Resources.Ampersand);
                return_letter = "";
            }
            if ((position == 4) && (angle >= OFFSET + interval * 2 && angle < OFFSET + interval * 3))
            {
                textBoxRight.Text = "$";
                audio = new SoundPlayer(Properties.Resources.money);
                return_letter = "";
            }
            if ((position == 4) && (angle >= OFFSET + interval * 3 && angle < OFFSET + interval * 4))
            {
                textBoxRight.Text = "!";
                audio = new SoundPlayer(Properties.Resources.exclamation);
                return_letter = "";
            }
            if ((position == 4) && (angle >= OFFSET + interval * 4 && angle < OFFSET + interval * 5))
            {
                textBoxRight.Text = ":";
                audio = new SoundPlayer(Properties.Resources.colon);
                return_letter = "";
            }
            if ((position == 4) && (angle >= OFFSET + interval * 5 && angle < OFFSET + interval * 6))
            {
                textBoxRight.Text = "@";
                audio = new SoundPlayer(Properties.Resources.at);
                return_letter = "";
            }
            if ((position == 4) && (angle >= OFFSET + interval * 6 && angle < OFFSET + interval * 7))
            {
                textBoxRight.Text = "Enter";
                audio = new SoundPlayer(Properties.Resources.enter);
                return_letter = "";
            }
            if ((position == 4) && (angle >= OFFSET + interval * 7 && angle < OFFSET + interval * 8))
            {
                textBoxRight.Text = "←";
                audio = new SoundPlayer(Properties.Resources.backspace);
                return_letter = "";
            }
            if ((position == 4) && (angle >= OFFSET + interval * 8 && angle < OFFSET + interval * 9))
            {
                textBoxRight.Text = ",";
                audio = new SoundPlayer(Properties.Resources.comma);
                return_letter = "";
            }
            if ((position == 4) && (angle >= OFFSET + interval * 9 && angle < OFFSET + interval * 10))
            {
                textBoxRight.Text = "#";
                audio = new SoundPlayer(Properties.Resources.pound);
                return_letter = "";
            }
            if ((position == 4) && (angle >= OFFSET + interval * 10 && angle < OFFSET + interval * 11))
            {
                textBoxRight.Text = "%";
                audio = new SoundPlayer(Properties.Resources.percent);
                return_letter = "";
            }
            if ((position == 4) && (angle >= OFFSET + interval * 11 && angle < OFFSET + interval * 12))
            {
                textBoxRight.Text = "-";
                audio = new SoundPlayer(Properties.Resources.minus);
                return_letter = "";
            }
            if ((position == 4) && (angle >= OFFSET + interval * 12 && angle < OFFSET + interval * 13))
            {
                textBoxRight.Text = "/";
                audio = new SoundPlayer(Properties.Resources.backslash);
                return_letter = "";
            }
            //==============Right Hand Low End==================//

            //==============Left Hand High Start==================//
            if ((position == 5) && (angle >= OFFSET && angle < OFFSET + interval * 1))
            {
                textBoxLeft.Text = "~";
                audio = new SoundPlayer(Properties.Resources.tilde);
                return_letter = "";
            }
            if ((position == 5) && (angle >= OFFSET + interval * 1 && angle < OFFSET + interval * 2))
            {
                textBoxLeft.Text = "^";
                audio = new SoundPlayer(Properties.Resources.carrot);
                return_letter = "";
            }
            if ((position == 5) && (angle >= OFFSET + interval * 2 && angle < OFFSET + interval * 3))
            {
                textBoxLeft.Text = "*";
                audio = new SoundPlayer(Properties.Resources.star);
                return_letter = "";
            }
            if ((position == 5) && (angle >= OFFSET + interval * 3 && angle < OFFSET + interval * 4))
            {
                textBoxLeft.Text = "_";
                audio = new SoundPlayer(Properties.Resources.underscore);
                return_letter = "";
            }
            if ((position == 5) && (angle >= OFFSET + interval * 4 && angle < OFFSET + interval * 5))
            {
                textBoxLeft.Text = "Left";
                audio = new SoundPlayer(Properties.Resources.left);
                return_letter = "";
            }
            if ((position == 5) && (angle >= OFFSET + interval * 5 && angle < OFFSET + interval * 6))
            {
                textBoxLeft.Text = "Up";
                audio = new SoundPlayer(Properties.Resources.up);
                return_letter = "";
            }
            if ((position == 5) && (angle >= OFFSET + interval * 6 && angle < OFFSET + interval * 7))
            {
                textBoxLeft.Text = "Space";
                audio = new SoundPlayer(Properties.Resources.space);
                return_letter = "";
            }
            if ((position == 5) && (angle >= OFFSET + interval * 7 && angle < OFFSET + interval * 8))
            {
                textBoxLeft.Text = "Down";
                audio = new SoundPlayer(Properties.Resources.down);
                return_letter = "";
            }
            if ((position == 5) && (angle >= OFFSET + interval * 8 && angle < OFFSET + interval * 9))
            {
                textBoxLeft.Text = "Right";
                audio = new SoundPlayer(Properties.Resources.right);
                return_letter = "";
            }
            if ((position == 5) && (angle >= OFFSET + interval * 9 && angle < OFFSET + interval * 10))
            {
                textBoxLeft.Text = "+";
                audio = new SoundPlayer(Properties.Resources.plus);
                return_letter = "";
            }
            if ((position == 5) && (angle >= OFFSET + interval * 10 && angle < OFFSET + interval * 11))
            {
                textBoxLeft.Text = "=";
                audio = new SoundPlayer(Properties.Resources.equal);
                return_letter = "";
            }
            if ((position == 5) && (angle >= OFFSET + interval * 11 && angle < OFFSET + interval * 12))
            {
                textBoxLeft.Text = ";";
                audio = new SoundPlayer(Properties.Resources.semicolon);
                return_letter = "";
            }
            if ((position == 5) && (angle >= OFFSET + interval * 12 && angle < OFFSET + interval * 13))
            {
                textBoxLeft.Text = "\\";
                audio = new SoundPlayer(Properties.Resources.forwardslash);
                return_letter = "";
            }
            //==============Left Hand High End==================//

            //==============Right Hand High Start==================//
            if ((position == 6) && (angle >= OFFSET && angle < OFFSET + interval * 1))
            {
                textBoxRight.Text = "[";
                audio = new SoundPlayer(Properties.Resources.squarebracket);
                return_letter = "";
            }
            if ((position == 6) && (angle >= OFFSET + interval * 1 && angle < OFFSET + interval * 2))
            {
                textBoxRight.Text = "{";
                audio = new SoundPlayer(Properties.Resources.curlybracket);
                return_letter = "";
            }
            if ((position == 6) && (angle >= OFFSET + interval * 2 && angle < OFFSET + interval * 3))
            {
                textBoxRight.Text = "<";
                audio = new SoundPlayer(Properties.Resources.lessthan);
                return_letter = "";
            }
            if ((position == 6) && (angle >= OFFSET + interval * 3 && angle < OFFSET + interval * 4))
            {
                textBoxRight.Text = "\"";
                audio = new SoundPlayer(Properties.Resources.quote);
                return_letter = "";
            }
            if ((position == 6) && (angle >= OFFSET + interval * 4 && angle < OFFSET + interval * 5))
            {
                textBoxRight.Text = "Home";
                audio = new SoundPlayer(Properties.Resources.home);
                return_letter = "";
            }
            if ((position == 6) && (angle >= OFFSET + interval * 5 && angle < OFFSET + interval * 6))
            {
                textBoxRight.Text = "PgUp";
                audio = new SoundPlayer(Properties.Resources.pgup);
                return_letter = "";
            }
            if ((position == 6) && (angle >= OFFSET + interval * 6 && angle < OFFSET + interval * 7))
            {
                textBoxRight.Text = "Enter";
                audio = new SoundPlayer(Properties.Resources.enter);
                return_letter = "";
            }
            if ((position == 6) && (angle >= OFFSET + interval * 7 && angle < OFFSET + interval * 8))
            {
                textBoxRight.Text = "PgDn";
                audio = new SoundPlayer(Properties.Resources.pgdn);
                return_letter = "";
            }
            if ((position == 6) && (angle >= OFFSET + interval * 8 && angle < OFFSET + interval * 9))
            {
                textBoxRight.Text = "End";
                audio = new SoundPlayer(Properties.Resources.end);
                return_letter = "";
            }
            if ((position == 6) && (angle >= OFFSET + interval * 9 && angle < OFFSET + interval * 10))
            {
                textBoxRight.Text = "?";
                audio = new SoundPlayer(Properties.Resources.question_mark);
                return_letter = "";
            }
            if ((position == 6) && (angle >= OFFSET + interval * 10 && angle < OFFSET + interval * 11))
            {
                textBoxRight.Text = ">";
                audio = new SoundPlayer(Properties.Resources.greaterthan);
                return_letter = "";
            }
            if ((position == 6) && (angle >= OFFSET + interval * 11 && angle < OFFSET + interval * 12))
            {
                textBoxRight.Text = "}";
                audio = new SoundPlayer(Properties.Resources.curlybracket);
                return_letter = "";
            }
            if ((position == 6) && (angle >= OFFSET + interval * 12 && angle < OFFSET + interval * 13))
            {
                textBoxRight.Text = "]";
                audio = new SoundPlayer(Properties.Resources.squarebracket);
                return_letter = "";
            }
            //==============Right Hand High End==================//
            /*
            try
            {
                if (checkBoxSound.IsChecked == true)
                    audio.Play();
            }
            catch
            {
                //do nothing
            }*/
            return return_letter;
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

        private void grid1_Loaded(object sender, RoutedEventArgs e)
        {
            // Create the interop host control.
            System.Windows.Forms.Integration.WindowsFormsHost host =
                new System.Windows.Forms.Integration.WindowsFormsHost();

            // Create the MaskedTextBox control.
            MaskedTextBox mtbDate = new MaskedTextBox("00/00/0000");

            var source = new AutoCompleteStringCollection();
            string[] dictionary = System.IO.File.ReadAllLines("c:\\Share\\words.txt");
            source.AddRange(dictionary);

            var textbox = new System.Windows.Forms.TextBox
            {
                AutoCompleteCustomSource = source,
                AutoCompleteMode = AutoCompleteMode.Append,
                AutoCompleteSource = AutoCompleteSource.CustomSource
            };

            // Assign the MaskedTextBox control as the host control's child.
            host.Child = textbox;

            // Add the interop host control to the Grid 
            // control's collection of child controls. 
            this.grid1.Children.Add(host);
        }
    }
}

