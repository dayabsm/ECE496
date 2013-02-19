using System;
using System.Collections;
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
using System.Media;
using System.Drawing.Drawing2D;
using System.Resources;
using System.Reflection;

namespace PrototypeUI
{
    /// <summary>
    /// Interaction logic for MainWindow.xaml
    /// </summary>
    public partial class MainWindow : Window
    {
        //-- added feb 18
        Queue<double> right_degrees = new Queue<double>();
        Queue<double> left_degrees = new Queue<double>();

        const double MAX_DEGREE_DEVIATION = 10.0;   //maybe a percentage is more flexible
        const int MAX_QUEUE = 5;    //number of previous degrees to take average
        //--

        public MainWindow()
        {
            InitializeComponent();
        }
        
        public void MainLoop()
        {
            while(true)
            {
                int position = 0;

                if (radioButtonLeft1.IsChecked == true)
                    position = 2;
                if (radioButtonLeft2.IsChecked == true)
                    position = 4;
                double angle = WorkingWithDepthData.MainWindow.angleRight;

                textBoxRight.Text = angle.ToString("#.##") + "°";
                //offset -> need to adjust probably
                angle = angle - 90;

                Rotate(cursorRight, wheelRight, angle, position);
            }
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
        private double check_angles(double angle, int position)
        {
            if (position % 2 == 0)  //even, position 2,4
            {
                //if there is 3 or more queues, dequeue 1, and enqueue the current angle
                //so there is always 3 or less queues
                if (right_degrees.Count >= MAX_QUEUE)
                {
                    //check the current average of the previous degrees
                    double current_average = 0.0;
                    foreach (double current in right_degrees)
                    {
                        current_average = current_average + current;
                    }
                    //check the bounds of the angle with the current average, 
                    //if the angle is greater of less than current avg is (+/-)40 degrees, discard
                    if (angle > current_average + MAX_DEGREE_DEVIATION || angle < current_average - MAX_DEGREE_DEVIATION)
                    {
                        return -999.999;
                    }
                    //if it's not noise, dequeue and add to queue
                    right_degrees.Dequeue();
                    right_degrees.Enqueue(angle);
                }
                else //else there is less than 3, so enqueue current angle
                {
                    right_degrees.Enqueue(angle);
                }
                //calculate the average degree and returns
                double current_degree = 0.0;
                foreach (double number in right_degrees)
                {
                    current_degree = current_degree + number;
                }
                double new_average = current_degree / right_degrees.Count;
                return new_average;
            }
            else if (position % 2 == 1) //position 1,3
            {
                if (left_degrees.Count >= MAX_QUEUE)
                {
                    double current_average = 0.0;
                    foreach (double current in left_degrees)
                    {
                        current_average = current_average + current;
                    }

                    if (angle > current_average + MAX_DEGREE_DEVIATION || angle < current_average - MAX_DEGREE_DEVIATION)
                    {
                        return -999.999;
                    }

                    left_degrees.Dequeue();
                    left_degrees.Enqueue(angle);
                }
                else
                {
                    left_degrees.Enqueue(angle);
                }
                double current_degree = 0.0;
                foreach (double number in right_degrees)
                {
                    current_degree = current_degree + number;
                }
                double new_average = current_degree / right_degrees.Count;
                return new_average;
            }
            else
            {
                return -999.999;
            }
        }

        private void PlayLetter(char letter)
        {
            /* not in use right now
            Assembly assembly;
            SoundPlayer sp;
            assembly = Assembly.GetExecutingAssembly();
            sp = new SoundPlayer(assembly.GetManifestResourceStream("test.wav"));
            sp.Play();            
             */
        }

    }
}
