using System;
using System.Collections.Generic;
using System.Drawing;
using System.Drawing.Imaging;
using System.Linq;
using System.Windows.Forms;


///Algorithms Project
///Intelligent Scissors
///

namespace ImageQuantization
{
    /// <summary>
    /// Holds the pixel color in 3 byte values: red, green and blue
    /// </summary>
    public struct RGBPixel
    {
        public byte red, green, blue;
        public RGBPixel(double R, double G, double B)
        {
            red = (byte)R;
            green = (byte)G;
            blue = (byte)B;
        }
    }
    public struct edges
    {
        public int V1, V2;
        public double wieght;
    }
    public struct RGBPixelD
    {
        public double red, green, blue;
      
    }


    /// <summary>
    /// Library of static functions that deal with images
    /// </summary>
    public class ImageOperations
    {
        /// <summary>
        /// Open an image and load it into 2D array of colors (size: Height x Width)
        /// </summary>
        /// <param name="ImagePath">Image file path</param>
        /// <returns>2D array of colors</returns>
        /// 

        public static RGBPixel[,] OpenImage(string ImagePath)
        {
            Bitmap original_bm = new Bitmap(ImagePath);
            int Height = original_bm.Height;
            int Width = original_bm.Width;

            RGBPixel[,] Buffer = new RGBPixel[Height, Width];

            unsafe
            {
                BitmapData bmd = original_bm.LockBits(new Rectangle(0, 0, Width, Height), ImageLockMode.ReadWrite, original_bm.PixelFormat);
                int x, y;
                int nWidth = 0;
                bool Format32 = false;
                bool Format24 = false;
                bool Format8 = false;

                if (original_bm.PixelFormat == PixelFormat.Format24bppRgb)
                {
                    Format24 = true;
                    nWidth = Width * 3;
                }
                else if (original_bm.PixelFormat == PixelFormat.Format32bppArgb || original_bm.PixelFormat == PixelFormat.Format32bppRgb || original_bm.PixelFormat == PixelFormat.Format32bppPArgb)
                {
                    Format32 = true;
                    nWidth = Width * 4;
                }
                else if (original_bm.PixelFormat == PixelFormat.Format8bppIndexed)
                {
                    Format8 = true;
                    nWidth = Width;
                }
                int nOffset = bmd.Stride - nWidth;
                byte* p = (byte*)bmd.Scan0;
                for (y = 0; y < Height; y++)
                {
                    for (x = 0; x < Width; x++)
                    {
                        if (Format8)
                        {
                            Buffer[y, x].red = Buffer[y, x].green = Buffer[y, x].blue = p[0];
                            p++;
                        }
                        else
                        {
                            Buffer[y, x].red = p[2];
                            Buffer[y, x].green = p[1];
                            Buffer[y, x].blue = p[0];
                            if (Format24) p += 3;
                            else if (Format32) p += 4;
                        }
                    }
                    p += nOffset;
                }
                original_bm.UnlockBits(bmd);
            }

            return Buffer;
        }
        
        /// <summary>
        /// Get the height of the image 
        /// </summary>
        /// <param name="ImageMatrix">2D array that contains the image</param>
        /// <returns>Image Height</returns>
        public static int GetHeight(RGBPixel[,] ImageMatrix)
        {
            return ImageMatrix.GetLength(0);
        }

        /// <summary>
        /// Get the width of the image 
        /// </summary>
        /// <param name="ImageMatrix">2D array that contains the image</param>
        /// <returns>Image Width</returns>
        public static int GetWidth(RGBPixel[,] ImageMatrix)
        {
            return ImageMatrix.GetLength(1);
        }
        /*
         //////////////////////////////////////////////////////////////////////////////////////
         //////////////////////////////////////////////////////////////////////////////////////
         /////////////////////////////////////////////////////////////////////////////////////////
         /////////////////////////////////////////////////////////////////////////////////////////
         /////////////////////////////////////////////////////////////////////////////////////////
         /////////////////////////////////////////////////////////////////////////////////////////
         ///
        */

        public static List<int> listOFcolors;
        public static List<int> DistinctPixels(RGBPixel[,]ImageMatrix)
        {
            int width = ImageMatrix.GetLength(1);
            int height = ImageMatrix.GetLength(0);
            int r, g, b;
            HashSet<int> set = new HashSet<int>();
            
            for (int y = 0; y < height; y++)
            {
                for (int x = 0; x < width; x++)
                {
                    r = ImageMatrix[y, x].red;
                    g = ImageMatrix[y, x].green;
                    b = ImageMatrix[y, x].blue;
                    set.Add((r << 16) + (g << 8) + b);
                }
            }
            listOFcolors = set.ToList();
            return listOFcolors;
        }
        public static Dictionary<int, List<float>> Graph_Wieghts;
        public static SortedDictionary<double, edges> edges_wieght;
        public static int counter = 0;
        public static edges minimum_wieght;
        
        public static void GraphConstruct()
        {
            minimum_wieght.wieght = double.MaxValue;
            byte r1,r2,g1,g2,b1,b2;
            double distance;


            Graph_Wieghts = new Dictionary<int, List<float>>(listOFcolors.Count);
            for (int i = 0 ; i < listOFcolors.Count;i++)
                {
                Graph_Wieghts.Add(i, new List<float>());
                r1 = (byte)(listOFcolors[i] >> 16);
                g1 = (byte)(listOFcolors[i] >> 8);
                b1 = (byte)(listOFcolors[i]);
                for (int j = i; j < listOFcolors.Count; j++)
                {
                        r2 = (byte)(listOFcolors[j] >> 16);
                        g2 = (byte)(listOFcolors[j] >> 8);
                        b2 = (byte)(listOFcolors[j]);

                    distance = Math.Sqrt((r1 - r2) * (r1 - r2) + (g1 - g2) * (g1 - g2) + (b1 - b2) * (b1 - b2));

                    if (distance < minimum_wieght.wieght && distance!=0)
                    {
                        minimum_wieght.V1 =i ;
                        minimum_wieght.V2 = j;
                          minimum_wieght.wieght = distance;
                    }

                    Graph_Wieghts[i].Add((float)(distance));

                }
            }
           

        }

        public static List<edges> graph_path;
        public static List<bool> check;
        public static double MST_value;
        public static double MST() // find mst
        {

            byte r1, r2, g1, g2, b1, b2;

        
            graph_path = new List<edges>(listOFcolors.Count);
            List<double> edges_value;
            List<int> parent;
            edges_value = Enumerable.Repeat(double.MaxValue, listOFcolors.Count).ToList();
            check = Enumerable.Repeat(false, listOFcolors.Count).ToList();
            parent = Enumerable.Repeat(-1, listOFcolors.Count).ToList();
            edges_value[0] = 0;
            int nextnode = 0;
            double Cost;
            for (int i = 0; i < listOFcolors.Count - 1; i++)  // o(n)
            {
                int idx = nextnode;
                check[idx] = true;
                double mininode = double.MaxValue;
                
                for (int j = 0; j < listOFcolors.Count; j++) // o(N)
                {
                    r2 = (byte)(listOFcolors[j] >> 16);
                    g2 = (byte)(listOFcolors[j] >> 8);
                    b2 = (byte)(listOFcolors[j]);
                    r1 = (byte)(listOFcolors[idx] >> 16);
                    g1 = (byte)(listOFcolors[idx] >> 8);
                    b1 = (byte)(listOFcolors[idx]);

                    if (!check[j])
                    {

                        Cost = Math.Sqrt((r1 - r2) * (r1 - r2) + (g1 - g2) * (g1 - g2) + (b1 - b2) * (b1 - b2));
                         
                          // Get Min  IDX in Nextnode
                        if (Cost < mininode)
                        {
                            mininode = Cost;
                            nextnode = j;
                        }
                        // set Idx for Min Node the Current Node
                        if (edges_value[j] < mininode)
                        {
                            mininode = edges_value[j];
                            nextnode = j;
                        }
                        // Update Value of the Edge with new Min val
                        if (Cost < edges_value[j])
                        {
                            parent[j] = idx;
                           edges_value[j] = Cost;
                        }
                    }
                }

            }
            MST_value = 0;
            for (int i = 1; i < parent.Count; i++)
            {
                edges Node;
                Node.V1 = parent[i];
                Node.V2 = i;
                Node.wieght = edges_value[i];
                graph_path.Add(Node);
                MST_value += edges_value[i];
            }

            return MST_value;
        }


        public static List<List<RGBPixel>> Clustering; // OutPutClusters
        public static List<List<int>> Adjacent;    // Bfs 
        public static SortedSet<int> Big;
        public static int num_of_clusters;


       //get niehbours of maximum nodes using bfs algorithm 
        public static List<RGBPixel> Clustering_BFS(int s) // o(V^2)
        {

            List<RGBPixel> ret = new List<RGBPixel>();
            byte r1, g1, b1;

            r1 = (byte)(listOFcolors[s] >> 16);
            g1 = (byte)(listOFcolors[s] >> 8);
            b1 = (byte)(listOFcolors[s]);

            RGBPixel save;

            save.blue = b1;
            save.red = r1;
            save.green = g1;


            ret.Add(save); // add him in cluster

            Queue<int> q = new Queue<int>();
            q.Enqueue(s);
            while (q.Count != 0) // o(E) worst O(V^2) ~=  o(D)
            {
                int p = q.Dequeue();
                if (!check[p])
                {
                    check[p] = true;   // Just Marked Him as true
                    int count = Adjacent[p].Count;
                    for (int i = 0; i <count; i++) //  Loop to push his childs : worst o(V)             
                        if (!check[Adjacent[p][i]])
                        {
                            r1 = (byte)(listOFcolors[Adjacent[p][i]] >> 16);
                            g1 = (byte)(listOFcolors[Adjacent[p][i]] >> 8);
                            b1 = (byte)(listOFcolors[Adjacent[p][i]]);


                            save.blue = b1;
                            save.red = r1;
                            save.green = g1;
                            ret.Add(save); // add him in cluster
                            q.Enqueue(Adjacent[p][i]);
                        }
                }
            }
            return ret;
        }


        public static void BeginCLustering()
        {
           Clustering = new List<List<RGBPixel>>();
            graph_path = graph_path.OrderBy(edges => edges.wieght).ToList();      // Sort Quick Sort NlogN
          
            check = Enumerable.Repeat(false, listOFcolors.Count).ToList();  // Re Use to Check if Take it in Clustering or Not
            Adjacent = new List<List<int>>(num_of_clusters);

            for (int i = 0; i < listOFcolors.Count; i++) // memset Adjacent
                Adjacent.Add( new List<int>());
            
            Big  = new SortedSet<int>();
            for (int i = 0; i < num_of_clusters-1 ; i++) // O(C*C.Length) ~= o(N^2)
            {
                double max = -1;
                int indx = 0;
                for (int ii = 0; ii < graph_path.Count; ii++)
                {
                    if (max < graph_path[ii].wieght)
                    {
                        max = graph_path[ii].wieght;
                        indx = ii;
                    }
                }
                edges e;
                e.V1 = graph_path[indx].V1;
                e.V2 = graph_path[indx].V2;
                e.wieght = -1;

                graph_path[indx] = e;

                Big.Add(graph_path[indx].V1);
                Big.Add(graph_path[indx].V2);
            }

            //creat new adj list

            for (int i = 0;i< graph_path.Count; i++) // o(C) ~= O(N)
            {
                if (graph_path[i].wieght != -1)
                {

                    Adjacent[graph_path[i].V1].Add(graph_path[i].V2);
                    Adjacent[graph_path[i].V2].Add(graph_path[i].V1);

                }
            }
            foreach(int i in  Big) // o(C) * O(E) ~= o(C*E) : E in worst = D then O(C*D)
                if (!check[i])
                    Clustering.Add(new List<RGBPixel>( Clustering_BFS(i)));
                
            setClusters(); // Change Values o(V^2)
        }


        public static List<RGBPixel> Pallete;


        public static RGBPixel[,,] RGB = new RGBPixel[256, 256, 256];

        // generate pallett and set average or centriod piont for every color
        public static void setClusters() // O(K*D)
        {
            Pallete = new List<RGBPixel>();
            for (int i = 0; i < Clustering.Count; i++) // o(K*D)
            {
                RGBPixel res = Avrge(i);
                Pallete.Add(res);
                for (int j = 0; j < Clustering[i].Count; j++)
                    RGB[Clustering[i][j].red, Clustering[i][j].green, Clustering[i][j].blue] = res;

            }

        }


        public static RGBPixel Avrge(int cluster_Number) // O(C)
        {

            double R = 0, G = 0, B = 0;

            byte r, g, b;
            HashSet<int> key = new HashSet<int>();
            List<int> save_value = new List<int>();
            for (int i = 0; i < Clustering[cluster_Number].Count; i++) // o(C.length)
            {
                R += Clustering[cluster_Number][i].red;
                G += Clustering[cluster_Number][i].green;
                B += Clustering[cluster_Number][i].blue;
            }
            R /= Clustering[cluster_Number].Count;
            G /= Clustering[cluster_Number].Count;
            B /= Clustering[cluster_Number].Count;
        
            return new RGBPixel(R, G, B);
        }

       /*public static void Quantize(ref RGBPixel[,] ImageMatrix)
        {
            int width = ImageMatrix.GetLength(1);
            int height = ImageMatrix.GetLength(0);
            byte r, g, b;
            int key = 0;
            for (int y = 0; y < height; y++)
            {
                for (int x = 0; x < width; x++)
                {
                    r = ImageMatrix[y, x].red;
                    g = ImageMatrix[y, x].green;
                    b = ImageMatrix[y, x].blue;
                    RGBPixel save = RGB[r, g, b];



                    ImageMatrix[y, x].red = save.red;
                    ImageMatrix[y, x].green =save.green;
                    ImageMatrix[y, x].blue = save.blue;
                }
            }
      
        }
        */

        /// <summary>
        /// Display the given image on the given PictureBox object
        /// </summary>
        /// <param name="ImageMatrix">2D array that contains the image</param>
        /// <param name="PicBox">PictureBox object to display the image on it</param>
        public static void DisplayImage(RGBPixel[,] ImageMatrix, PictureBox PicBox)
        {
            // Create Image:
            //==============
            int Height = ImageMatrix.GetLength(0);
            int Width = ImageMatrix.GetLength(1);
            Bitmap ImageBMP = new Bitmap(Width, Height, PixelFormat.Format24bppRgb);

            unsafe
            {
                BitmapData bmd = ImageBMP.LockBits(new Rectangle(0, 0, Width, Height), ImageLockMode.ReadWrite, ImageBMP.PixelFormat);
                int nWidth = 0;
                nWidth = Width * 3;
                int nOffset = bmd.Stride - nWidth;
                byte* p = (byte*)bmd.Scan0;
                for (int i = 0; i < Height; i++)
                {
                    for (int j = 0; j < Width; j++)
                    {
                        p[2] = ImageMatrix[i, j].red;
                        p[1] = ImageMatrix[i, j].green;
                        p[0] = ImageMatrix[i, j].blue;
                        p += 3;
                    }

                    p += nOffset;
                }
                ImageBMP.UnlockBits(bmd);
            }
            PicBox.Image = ImageBMP;
        }

        public static void DisplayImage2(RGBPixel[,] ImageMatrix, PictureBox PicBox)
        {
            // Create Image:
            //==============
            RGBPixel ne;

            int Height = ImageMatrix.GetLength(0);
            int Width = ImageMatrix.GetLength(1);
            Bitmap ImageBMP = new Bitmap(Width, Height, PixelFormat.Format24bppRgb);

            unsafe
            {
                BitmapData bmd = ImageBMP.LockBits(new Rectangle(0, 0, Width, Height), ImageLockMode.ReadWrite, ImageBMP.PixelFormat);
                int nWidth = 0;
                nWidth = Width * 3;
                int nOffset = bmd.Stride - nWidth;
                byte* p = (byte*)bmd.Scan0;
                for (int i = 0; i < Height; i++)
                {
                    for (int j = 0; j < Width; j++)
                    {
                        ne = RGB[ImageMatrix[i, j].red, ImageMatrix[i, j].green, ImageMatrix[i, j].blue];
                        p[2] = ne.red;
                        p[1] = ne.green;
                        p[0] = ne.blue;
                        p += 3;
                    }

                    p += nOffset;
                }
                ImageBMP.UnlockBits(bmd);
            }
            PicBox.Image = ImageBMP;
        }


        /// <summary>
        /// Apply Gaussian smoothing filter to enhance the edge detection 
        /// </summary>
        /// <param name="ImageMatrix">Colored image matrix</param>
        /// <param name="filterSize">Gaussian mask size</param>
        /// <param name="sigma">Gaussian sigma</param>
        /// <returns>smoothed color image</returns>
        public static RGBPixel[,] GaussianFilter1D(RGBPixel[,] ImageMatrix, int filterSize, double sigma)
        {
            int Height = GetHeight(ImageMatrix);
            int Width = GetWidth(ImageMatrix);

            RGBPixelD[,] VerFiltered = new RGBPixelD[Height, Width];
            RGBPixel[,] Filtered = new RGBPixel[Height, Width];

           
            // Create Filter in Spatial Domain:
            //=================================
            //make the filter ODD size
            if (filterSize % 2 == 0) filterSize++;

            double[] Filter = new double[filterSize];

            //Compute Filter in Spatial Domain :
            //==================================
            double Sum1 = 0;
            int HalfSize = filterSize / 2;
            for (int y = -HalfSize; y <= HalfSize; y++)
            {
                //Filter[y+HalfSize] = (1.0 / (Math.Sqrt(2 * 22.0/7.0) * Segma)) * Math.Exp(-(double)(y*y) / (double)(2 * Segma * Segma)) ;
                Filter[y + HalfSize] = Math.Exp(-(double)(y * y) / (double)(2 * sigma * sigma));
                Sum1 += Filter[y + HalfSize];
            }
            for (int y = -HalfSize; y <= HalfSize; y++)
            {
                Filter[y + HalfSize] /= Sum1;
            }

            //Filter Original Image Vertically:
            //=================================
            int ii, jj;
            RGBPixelD Sum;
            RGBPixel Item1;
            RGBPixelD Item2;

            for (int j = 0; j < Width; j++)
                for (int i = 0; i < Height; i++)
                {
                    Sum.red = 0;
                    Sum.green = 0;
                    Sum.blue = 0;
                    for (int y = -HalfSize; y <= HalfSize; y++)
                    {
                        ii = i + y;
                        if (ii >= 0 && ii < Height)
                        {
                            Item1 = ImageMatrix[ii, j];
                            Sum.red += Filter[y + HalfSize] * Item1.red;
                            Sum.green += Filter[y + HalfSize] * Item1.green;
                            Sum.blue += Filter[y + HalfSize] * Item1.blue;
                        }
                    }
                    VerFiltered[i, j] = Sum;
                }

            //Filter Resulting Image Horizontally:
            //===================================
            for (int i = 0; i < Height; i++)
                for (int j = 0; j < Width; j++)
                {
                    Sum.red = 0;
                    Sum.green = 0;
                    Sum.blue = 0;
                    for (int x = -HalfSize; x <= HalfSize; x++)
                    {
                        jj = j + x;
                        if (jj >= 0 && jj < Width)
                        {
                            Item2 = VerFiltered[i, jj];
                            Sum.red += Filter[x + HalfSize] * Item2.red;
                            Sum.green += Filter[x + HalfSize] * Item2.green;
                            Sum.blue += Filter[x + HalfSize] * Item2.blue;
                        }
                    }
                    Filtered[i, j].red = (byte)Sum.red;
                    Filtered[i, j].green = (byte)Sum.green;
                    Filtered[i, j].blue = (byte)Sum.blue;
                }

            return Filtered;
        }


    }
}
