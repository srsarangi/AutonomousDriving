using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System.IO;
using System.Linq;
using Emgu;
using System.Drawing;
using Emgu.CV;
using Emgu.CV.Structure;

namespace CarPhysics
{
    public class LIDARSIM : MonoBehaviour
    {
        public float upperFOV = 5f;
        public float lowerFOV = -20f;
        public int numChannels = 20;
        // string path;
        //StreamWriter outputfile;
        public int numRays = 36;
        private Vector3 origin;
        private RaycastHit outhit;
        private bool flag;
        Vector3[,] dirs;
        private string path;
        public Quaternion horQuat;
        private static Vector3[] points;
        public volatile int points_idx;
        public volatile Vector3[] pointCloud;
        private static bool isPointCloudReady = false;
        private StreamWriter outputfile;
        
        // Start is called before the first frame update
        void Start()
        {
            points_idx = 0;
            path = @"./Assets/Scripts/carPhysics";
            //outputfile = new StreamWriter(Path.Combine(path, "outputfilelinux.txt"), true);
            points = new Vector3[14400];
            
            Quaternion vert_angle_up = Quaternion.AngleAxis(1f, transform.right);
            Quaternion vert_angle_down = Quaternion.AngleAxis(-1f, transform.right);
            dirs = new Vector3[numRays, 2 * numChannels];
            horQuat = Quaternion.AngleAxis(1, transform.up);
            dirs[0, 0] = vert_angle_up * transform.forward;
            for (int i = 1; i < numChannels; i++)
            {
                dirs[0, i] = vert_angle_up * dirs[0, i - 1];
            }

            dirs[0, numChannels] = vert_angle_down * transform.forward;
            for (int i = numChannels + 1; i < 2 * numChannels; i++)
            {
                dirs[0, i] = vert_angle_down * dirs[0, i - 1];
            }

            float tempangle = 360 / numRays;
            for (int i = 1; i < numRays; i++)
            {
                Quaternion temp = Quaternion.AngleAxis(tempangle, transform.up);
                for (int j = 0; j < 2 * numChannels; j++)
                {
                    dirs[i, j] = temp * dirs[i - 1, j];
                }
            }
        }

        // Update is called once per frame
        void Update()
        {
            if (!isPointCloudReady)
            {
                //points_idx = 0;
                origin = transform.position + transform.up;
                // var results = new NativeArray<RaycastHit>(numRays * 2 * numChannels, Allocator.Temp);

                // var commands = new NativeArray<RaycastCommand>(numRays * 2 * numChannels, Allocator.Temp);
                // outputFile.WriteLine("Fourth Line");
                for (int j = 0; j < numRays; j++)
                {
                    for (int i = 0; i < 2 * numChannels; i++)
                    {
                        // Debug.Log(points_idx.ToString());
                        flag = Physics.Raycast(origin, dirs[j, i], out outhit, 20f);
                        if (flag)
                        {
                            // Debug.DrawLine(origin, outhit.point, Color.red);
                            Vector3 tmp = outhit.point - origin;
                            //outputfile.WriteLine(tmp[0] + " " + tmp[1] + " " + tmp[2]);

                            points[points_idx] = tmp;
                            points_idx = (points_idx + 1) % points.Length;
                        }

                        // commands[j*numRays+i] = new RaycastCommand(origin, dirs[j, i]);
                        dirs[j, i] = horQuat * dirs[j, i];
                        // cnout = j;
                    }
                }

                isPointCloudReady = true;

                /*JobHandle handle = RaycastCommand.ScheduleBatch(commands, results,1 , default(JobHandle));
                handle.Complete();
                int n = numRays * 2 * numChannels;
                for(int i=0; i<n; i++)
                {
                    if (results[i].collider != null)
                    {
                        Debug.DrawLine(origin, results[i].point, Color.red);
                    }
                    else
                    {
                        break;
                    }
                }*/
            }
        }

        public static Vector3[] GETLatestPointCloud()
        {
            if (isPointCloudReady)
            {
                isPointCloudReady = false;
                return points;
            }
            return null;
        }

        private static readonly bool[][] Selem = new bool[][]
        {
            new bool[] {true, true, true},
            new bool[] {true, true, true},
            new bool[] {true, true, true}
        };

       /* public List<PointF[]> FindBb(Vector3[] points, float res = 0.5f, float floor = -2.5f, int gridX = 80, int gridY = 80,
            int gridZ = 20)
        {
            int count = 2;
            points = points.Where(p => (p.z > floor)).ToArray();

            for (int i = 0; i < points.Length; i++)
            {
                points[i].x /= res;
                points[i].y /= res;
                points[i].z /= res;
            }

            bool[,,] voxelGrid = new bool[gridX, gridY, gridZ];
            int offsetX = gridX / 2;
            int offsetY = gridY / 2;

            foreach (var vector3 in points)
            {
                voxelGrid[(int) vector3.x + offsetX, (int) vector3.y + offsetY, (int) vector3.z + 3] = true;
            }

            bool[,] img = new bool[gridX, gridY];
            for (int i = 0; i < gridX; i++)
            {
                for (int j = 0; j < gridY; j++)
                {
                    for (int k = 0; k < gridZ; k++)
                    {
                        bool f = false;
                        if (voxelGrid[i, j, k])
                        {
                            f = true;
                            break;
                        }

                        if (f) img[i, j] = true;
                    }
                }
            }

            img = BinaryClosing(img, LIDARSIM.Selem);
            List<int> xn = new List<int>();
            List<int> yn = new List<int>();

            int[,] img2 = new int[gridX, gridY];

            for (int i = 0; i < gridX; i++)
            {
                for (int j = 0; j < gridY; j++)
                {
                    if (img[i, j])
                    {
                        img2[i, j] = 1;
                        xn.Add(i);
                        yn.Add(j);
                    }
                }
            }

            List<int> minArrayX = new List<int>();
            List<int> minArrayY = new List<int>();
            List<int> maxArrayX = new List<int>();
            List<int> maxArrayY = new List<int>();

            for (int i = 0; i < xn.Capacity; i++)
            {
                int xni = xn[i], yni = yn[i];
                if (img2[xni, yni] == 1)
                {
                    Stack<Tuple<int, int>> qu = new Stack<Tuple<int, int>>();
                    qu.Push(Tuple.Create(xni, yni));

                    img2[yni, xni] = count;
                    minArrayX.Add(xni);
                    minArrayY.Add(yni);
                    maxArrayX.Add(xni);
                    maxArrayY.Add(yni);
                    int ind = count - 2;

                    while (qu.Any())
                    {
                        Tuple<int, int> curr = qu.Pop();
                        int p = curr.Item1 + 1;
                        int q = curr.Item2 + 1;
                        if (p < gridY && q < gridX && img2[q, p] == 1)
                        {
                            img2[q, p] = count;
                            qu.Push(Tuple.Create(p, q));
                            maxArrayX[ind] = Math.Max(p, maxArrayX[ind]);
                            maxArrayY[ind] = Math.Max(q, maxArrayY[ind]);
                            maxArrayX[ind] = Math.Min(p, minArrayX[ind]);
                            minArrayY[ind] = Math.Min(q, minArrayY[ind]);
                        }

                        p = curr.Item1 - 1;
                        q = curr.Item2 - 1;
                        if (p >= 0 && q >= 0 && img2[q, p] == 1)
                        {
                            img2[q, p] = count;
                            qu.Push(Tuple.Create(p, q));
                            maxArrayX[ind] = Math.Max(p, maxArrayX[ind]);
                            maxArrayY[ind] = Math.Max(q, maxArrayY[ind]);
                            maxArrayX[ind] = Math.Min(p, minArrayX[ind]);
                            minArrayY[ind] = Math.Min(q, minArrayY[ind]);
                        }

                        q = curr.Item2 - 1;
                        if (q >= 0 && img2[q, curr.Item1] == 1)
                        {
                            img2[q, curr.Item1] = count;
                            qu.Push(Tuple.Create(curr.Item1, q));
                            maxArrayX[ind] = Math.Max(curr.Item1, maxArrayX[ind]);
                            maxArrayY[ind] = Math.Max(q, maxArrayY[ind]);
                            maxArrayX[ind] = Math.Min(curr.Item1, minArrayX[ind]);
                            minArrayY[ind] = Math.Min(q, minArrayY[ind]);
                        }

                        p = curr.Item1 - 1;
                        if (p >= 0 && img2[curr.Item2, p] == 1)
                        {
                            img2[curr.Item2, p] = count;
                            qu.Push(Tuple.Create(p, curr.Item2));
                            maxArrayX[ind] = Math.Max(p, maxArrayX[ind]);
                            maxArrayY[ind] = Math.Max(curr.Item2, maxArrayY[ind]);
                            maxArrayX[ind] = Math.Min(p, minArrayX[ind]);
                            minArrayY[ind] = Math.Min(curr.Item2, minArrayY[ind]);
                        }

                        p = curr.Item1 + 1;
                        q = curr.Item2 - 1;
                        if (p < gridY && q >= 0 && img2[q, p] == 1)
                        {
                            img2[q, p] = count;
                            qu.Push(Tuple.Create(p, q));
                            maxArrayX[ind] = Math.Max(p, maxArrayX[ind]);
                            maxArrayY[ind] = Math.Max(q, maxArrayY[ind]);
                            maxArrayX[ind] = Math.Min(p, minArrayX[ind]);
                            minArrayY[ind] = Math.Min(q, minArrayY[ind]);
                        }

                        p = curr.Item1 - 1;
                        q = curr.Item2 + 1;
                        if (p >= 0 && q < gridX && img2[q, p] == 1)
                        {
                            img2[q, p] = count;
                            qu.Push(Tuple.Create(p, q));
                            maxArrayX[ind] = Math.Max(p, maxArrayX[ind]);
                            maxArrayY[ind] = Math.Max(q, maxArrayY[ind]);
                            maxArrayX[ind] = Math.Min(p, minArrayX[ind]);
                            minArrayY[ind] = Math.Min(q, minArrayY[ind]);
                        }

                        p = curr.Item1 + 1;
                        if (p < gridY && img2[curr.Item2, p] == 1)
                        {
                            img2[curr.Item2, p] = count;
                            qu.Push(Tuple.Create(p, curr.Item2));
                            maxArrayX[ind] = Math.Max(p, maxArrayX[ind]);
                            maxArrayY[ind] = Math.Max(curr.Item2, maxArrayY[ind]);
                            maxArrayX[ind] = Math.Min(p, minArrayX[ind]);
                            minArrayY[ind] = Math.Min(curr.Item2, minArrayY[ind]);
                        }

                        q = curr.Item2 + 1;
                        if (q < gridX && img2[q, curr.Item1] == 1)
                        {
                            img2[q, curr.Item1] = count;
                            qu.Push(Tuple.Create(curr.Item1, q));
                            maxArrayX[ind] = Math.Max(curr.Item1, maxArrayX[ind]);
                            maxArrayY[ind] = Math.Max(q, maxArrayY[ind]);
                            maxArrayX[ind] = Math.Min(curr.Item1, minArrayX[ind]);
                            minArrayY[ind] = Math.Min(q, minArrayY[ind]);
                        }
                    }

                    count++;
                }
            }

            int l = count - 3;

            List<PointF[]> bboxes = new List<PointF[]>();
            while (l >= 0)
            {
                Tuple<List<int>, List<int>> yxp =
                    NonZero(img2.Slice(minArrayY[l], maxArrayX[l] + 1, minArrayX[l], maxArrayX[l] + 1));
                List<int> xp = yxp.Item1;
                List<int> yp = yxp.Item2;
                xp.ForEach(x => x += minArrayY[l]);
                yp.ForEach(y => y += minArrayY[l]);

                PointF[] cnt = new PointF[xp.Count];
                for (int i = 0; i < xp.Count; i++)
                {
                    cnt[i] = new PointF(xp[i], yp[i]);
                }

                bboxes.Add(CvInvoke.BoxPoints(CvInvoke.MinAreaRect(cnt)));
                l--;
            }
            
            for (int i = 0; i < bboxes.Count; i++)
            {
                for (int j = 0; j < 4; j++)
                {
                    bboxes[i][j].X -= (float)gridX / 2;
                    bboxes[i][j].Y -= (float)gridX / 2;
                }
            }

            return bboxes;
        }*/

        private bool[,] BinaryClosing(bool[,] img, bool[][]  salem)
        {
            return null;
        }
        
        public static Tuple<List<int>, List<int>> NonZero(int[,] source)
        {
            List<int> x = new List<int>();
            List<int> y = new List<int>();
            for (int i = 0; i < source.GetLength(0); i++)
            {
                for (int j = 0; j < source.GetLength(1); j++)
                {
                    if (source[i, j] != 0)
                    {
                        x.Add(i);
                        y.Add(j);
                    }
                }
            }
            return new Tuple<List<int>, List<int>>(x, y);
        }
        
        
        
        
    }
}