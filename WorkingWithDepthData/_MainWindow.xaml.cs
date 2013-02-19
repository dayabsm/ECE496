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

using System.Drawing.Drawing2D;

namespace PrototypeUI
{
    /// <summary>
    /// Interaction logic for MainWindow.xaml
    /// </summary>
    public partial class MainWindow : Window
    {
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

            cursor.RenderTransform = new RotateTransform(angle);
        }
    }
}
