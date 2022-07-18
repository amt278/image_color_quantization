using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Data;
using System.Drawing;
using System.Text;
using System.Windows.Forms;
using System.Diagnostics;


namespace ImageQuantization
{
    public partial class MainForm : Form
    {
        public MainForm()
        {
            InitializeComponent();
        }

        RGBPixel[,] ImageMatrix;

        private void btnOpen_Click(object sender, EventArgs e)
        {

 
            OpenFileDialog openFileDialog1 = new OpenFileDialog();
            if (openFileDialog1.ShowDialog() == DialogResult.OK)
            {
                //Open the browsed image and display it
                string OpenedFilePath = openFileDialog1.FileName;
                ImageMatrix = ImageOperations.OpenImage(OpenedFilePath);
                ImageOperations.DisplayImage(ImageMatrix, pictureBox1);
            }
            txtWidth.Text = ImageOperations.GetWidth(ImageMatrix).ToString();
            txtHeight.Text = ImageOperations.GetHeight(ImageMatrix).ToString();

            

        }

        private void btnGaussSmooth_Click(object sender, EventArgs e)
        {
            Stopwatch time_taken = new Stopwatch();
            time_taken.Start();

            double sigma = double.Parse(txtGaussSigma.Text);
            int maskSize = (int)nudMaskSize.Value;

            List<int> L = ImageOperations.DistinctPixels(ImageMatrix);
          textBox1.Text= L.Count.ToString();
              //ImageOperations.GraphConstruct();

            double total_cost = ImageOperations.MST();
            textBox2.Text=total_cost.ToString();

          
            ImageOperations.num_of_clusters = maskSize;
            ImageOperations.BeginCLustering();

          // ImageOperations.Quantize(ref ImageMatrix);

           // ImageMatrix = ImageOperations.GaussianFilter1D(ImageMatrix, maskSize, sigma);
            ImageOperations.DisplayImage2(ImageMatrix, pictureBox2);

            time_taken.Stop();
            textBox3.Text = time_taken.Elapsed.ToString();
        }



    }
}