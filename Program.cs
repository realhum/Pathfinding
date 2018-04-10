using System;
using System.Collections.Generic;
using System.Globalization;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Xml;


namespace GraphOpenStreetMap
{
    
    class Deq<T>
    {
        T[] array;

        public Deq()
        {
            array = new T[0];
        }
        public int Count
        {
            get
            {
                return array.Length;
            }
        }
        public bool Empty
        {
            get
            {
                return array.Length > 0;
            }
        }
        public void PushBack(T item)
        {
            Array.Resize(ref array, array.Length + 1);
            array[array.Length - 1] = item;
        }
        public void PushFront(T item)
        {
            Array.Resize(ref array, array.Length + 1);
            for (int i = array.Length - 1; i > 0; i--)
                array[i] = array[i - 1];
            array[0] = item;
        }
        public T PopBack()
        {
            T item = array[array.Length - 1];
            Array.Resize(ref array, array.Length - 1);
            return item;
        }
        public T PopFront()
        {
            T item = array[0];
            for (int i = 0; i < array.Length - 1; i++)
                array[i] = array[i + 1];
            Array.Resize(ref array, array.Length - 1);
            return item;
        }
        public T Front
        {
            get
            {
                return array[0];
            }
        }
        public T Back
        {
            get
            {
                return array[array.Length - 1];
            }
        }
    }


    class Program
    {

        private static readonly double major = 6378137.0;
        private static readonly double minor = 6356752.3142;
        private static readonly double ratio = minor / major;
        private static readonly double e = Math.Sqrt(1.0 - (ratio * ratio));
        private static readonly double com = 0.5 * e;
        private static readonly double degToRad = Math.PI / 180.0;

        struct coord
        {
            public double lat;
            public double lon;
        }

        struct pointDijk
        {
            public long id;
            public double x;
            public double y;
            public double weight;
            public long parent;
            public bool isVisited;
        }

        struct pointLevit
        {
            public int M;
            public long id;
            public double x;
            public double y;
            public double weight;
            public long parent;
        }

        private static double minlon;
        private static double maxlat;

        private static SortedDictionary<long, coord> Nodes = new SortedDictionary<long, coord>();
        private static SortedDictionary<long, List<long>> AddjestedList = new SortedDictionary<long, List<long>>();
        private static List<string> Valid = new List<string>() {"motorway", "motorway_link", "trunk", "trunk_link", "primary", "primary_link", "secondary",
                                            "secondary_link", "tertiary", "tertiary_link", "unclassified", "road", "service", "living_street", "residential" };


        private static List<long>[] DijkstraRoute = new List<long>[10];
        private static List<long>[] LevitRoute = new List<long>[10];
        private static pointDijk[] DijkstranPoint;
        private static pointLevit[] LevitPoint;

        static void Osm(string path)
        {

            XmlDocument xDoc = new XmlDocument();
            xDoc.Load(path);
            XmlElement xRoot = xDoc.DocumentElement;
            XmlNodeList nodes = xRoot.SelectNodes("node");
            maxlat = double.Parse(xRoot.SelectSingleNode("bounds").Attributes["maxlat"].Value, CultureInfo.InvariantCulture);
            minlon = double.Parse(xRoot.SelectSingleNode("bounds").Attributes["minlon"].Value, CultureInfo.InvariantCulture);
            foreach (XmlNode n in nodes)
            {
                long id = long.Parse(n.SelectSingleNode("@id").Value);
                double lat = double.Parse(n.SelectSingleNode("@lat").Value, CultureInfo.InvariantCulture);
                double lon = double.Parse(n.SelectSingleNode("@lon").Value, CultureInfo.InvariantCulture);
                coord Node_coord;
                Node_coord.lat = lat;
                Node_coord.lon = lon;
                Nodes.Add(id, Node_coord);
            }
            Valid.Sort();
            XmlNodeList ways = xRoot.SelectNodes("//way[.//tag[@k = 'highway']]");
            foreach (XmlNode n in ways)
            {
                string validway = n.SelectSingleNode("tag[@k = 'highway']").Attributes["v"].Value;
                if (Valid.BinarySearch(validway) >= 0)
                {
                    XmlNodeList nd = n.SelectNodes("nd");
                    List<long> nodes_list_id = new List<long>();
                    foreach (XmlNode m in nd)
                    {
                        long id = long.Parse(m.SelectSingleNode("@ref").Value);
                        nodes_list_id.Add(id);
                    }
                    for (int i = 0; i < nodes_list_id.Count(); ++i)
                    {
                        if (i < nodes_list_id.Count() - 1)
                        {
                            if (AddjestedList.ContainsKey(nodes_list_id[i]))
                            {
                                AddjestedList[nodes_list_id[i]].Add(nodes_list_id[i + 1]);
                            }
                            else
                            {
                                AddjestedList.Add(nodes_list_id[i], new List<long>());
                                AddjestedList[nodes_list_id[i]].Add(nodes_list_id[i + 1]);
                            }
                        }
                        if (i >= 1)
                        {
                            if (AddjestedList.ContainsKey(nodes_list_id[i]))
                            {
                                AddjestedList[nodes_list_id[i]].Add(nodes_list_id[i - 1]);
                            }
                            else
                            {
                                AddjestedList.Add(nodes_list_id[i], new List<long>());
                                AddjestedList[nodes_list_id[i]].Add(nodes_list_id[i - 1]);
                            }
                        }
                    }
                }
            }
        }

        private static double DegToRad(double deg)
        {
            return deg * degToRad;
        }

        public static double lonToX(double lon)
        {
            return major * DegToRad(lon) * 0.1;
        }

        public static double latToY(double lat)
        {
            lat = Math.Min(89.5, Math.Max(lat, -89.5));
            double phi = DegToRad(lat);
            double sinphi = Math.Sin(phi);
            double con = e * sinphi;
            con = Math.Pow(((1.0 - con) / (1.0 + con)), com);
            double ts = Math.Tan(0.5 * ((Math.PI * 0.5) - phi)) / con;
            return 0 - major * Math.Log(ts) * 0.1;
        }


        static void Dijkstra(long begin_node)
        {
            //Создание списка вершин
            ICollection<long> keys = AddjestedList.Keys;
            DijkstranPoint = new pointDijk[keys.Count];
            int k = 0;
            DijkstranPoint[k].id = begin_node;
            DijkstranPoint[k].x = lonToX(Nodes[begin_node].lon);
            DijkstranPoint[k].y = latToY(Nodes[begin_node].lat);
            DijkstranPoint[k].weight = 0;
            DijkstranPoint[k].parent = 0;
            DijkstranPoint[k].isVisited = false;
            k++;
            foreach (long i in keys)
            {
                if (i != begin_node)
                {
                    DijkstranPoint[k].id = i;
                    DijkstranPoint[k].x = lonToX(Nodes[i].lon);
                    DijkstranPoint[k].y = latToY(Nodes[i].lat);
                    DijkstranPoint[k].weight = Double.PositiveInfinity;
                    DijkstranPoint[k].parent = 0;
                    DijkstranPoint[k].isVisited = false;
                    k++;
                }
            }

            for (int j = 0; j < keys.Count(); ++j)
            {
                double minWeight = Double.PositiveInfinity;
                long minKey = 0;
                int index_minKey = 0;
                for (int i = 0; i < keys.Count; ++i)
                {
                    if (DijkstranPoint[i].weight != Double.PositiveInfinity && DijkstranPoint[i].weight < minWeight && !DijkstranPoint[i].isVisited)
                    {
                        minWeight = DijkstranPoint[i].weight;
                        minKey = DijkstranPoint[i].id;
                        index_minKey = i;
                    }
                }
                if (minKey != 0)
                {
                    for (int i = 0; i < AddjestedList[minKey].Count(); ++i)
                    {
                        long neighbourPoint = AddjestedList[minKey][i];
                        int index_neighbour = 0;
                        for (int p = 1; p < keys.Count; ++p)
                        {
                            if (DijkstranPoint[p].id == neighbourPoint)
                            {
                                index_neighbour = p;
                                break;
                            }
                        }
                        if (!DijkstranPoint[index_neighbour].isVisited)
                        {
                            double weightCurrEdge = Math.Sqrt(Math.Pow(DijkstranPoint[index_minKey].x - DijkstranPoint[index_neighbour].x, 2.0) + Math.Pow(DijkstranPoint[index_minKey].y - DijkstranPoint[index_neighbour].y, 2.0));
                            if (DijkstranPoint[index_neighbour].weight > DijkstranPoint[index_minKey].weight + weightCurrEdge)
                            {
                                DijkstranPoint[index_neighbour].weight = DijkstranPoint[index_minKey].weight + weightCurrEdge;
                                DijkstranPoint[index_neighbour].parent = index_minKey;
                            }
                        }
                    }
                    DijkstranPoint[index_minKey].isVisited = true;
                }
                else
                {
                    break;
                }
            }
            for(int i = 0; i < 10; i++) 
            {
                DijkstraRoute[i] = new List<long>();
            }
            long idEndPoint = 4119187711;
            FindDijkstraRoute(0, idEndPoint, keys.Count);
        }

        static void FindDijkstraRoute(int numbRoute, long endPoint, int N)
        {
            long index_endPoint = -1;
            double weight = -1;
            for (int i = 0; i < N; ++i)
                if (DijkstranPoint[i].id == endPoint)
                {
                    index_endPoint = i;
                    weight = DijkstranPoint[i].weight;
                    break;
                }
            if (index_endPoint == -1)
            {
                Console.WriteLine("No way to the {0} point!", numbRoute);
                return;
            }
            while (index_endPoint != 0)
            {
                DijkstraRoute[numbRoute].Add(DijkstranPoint[index_endPoint].id);
                index_endPoint = DijkstranPoint[index_endPoint].parent;
            }
            DijkstraRoute[numbRoute].Add(DijkstranPoint[index_endPoint].id);
            foreach (long i in DijkstraRoute[numbRoute])
            {
                Console.WriteLine(i);
            }
            Console.WriteLine("Weight {0}", weight);
        }

        static void Levit(long begin_node)
        {
            Deq<long> M1 = new Deq<long>();
            ICollection<long> keys = AddjestedList.Keys;
            LevitPoint = new pointLevit[keys.Count];
            int k = 0;
            LevitPoint[k].M = 1;
            LevitPoint[k].id = begin_node;
            LevitPoint[k].x = lonToX(Nodes[begin_node].lon);
            LevitPoint[k].y = latToY(Nodes[begin_node].lat);
            LevitPoint[k].weight = 0;
            LevitPoint[k].parent = 0;
            k++;
            M1.PushFront(begin_node);
            foreach (long i in keys)
            {
                if (i != begin_node)
                {
                    LevitPoint[k].M = 2;
                    LevitPoint[k].id = i;
                    LevitPoint[k].x = lonToX(Nodes[i].lon);
                    LevitPoint[k].y = latToY(Nodes[i].lat);
                    LevitPoint[k].weight = Double.PositiveInfinity;
                    LevitPoint[k].parent = 0;
                    k++;
                }
            }

            while(M1.Empty)
            {
                long currId = M1.PopFront();
                int index_currPoint = -1;
                for (int i = 0; i < keys.Count; ++i)
                {
                    if (LevitPoint[i].id == currId)
                    {
                        index_currPoint = i;
                        LevitPoint[i].M = 0;
                    }
                }
                if (index_currPoint != -1)
                {
                    for (int i = 0; i < AddjestedList[currId].Count(); ++i)
                    {
                        long neighbourPoint = AddjestedList[currId][i];
                        int index_neighbour = 0;
                        for (int p = 1; p < keys.Count; ++p)
                        {
                            if (LevitPoint[p].id == neighbourPoint)
                            {
                                index_neighbour = p;
                                break;
                            }
                        }

                        if(LevitPoint[index_neighbour].M == 2)
                        {
                            double weightCurrEdge = Math.Sqrt(Math.Pow(LevitPoint[index_currPoint].x - LevitPoint[index_neighbour].x, 2.0) + Math.Pow(LevitPoint[index_currPoint].y - LevitPoint[index_neighbour].y, 2.0));
                            LevitPoint[index_neighbour].M = 1;
                            LevitPoint[index_neighbour].weight = LevitPoint[index_currPoint].weight + weightCurrEdge;
                            LevitPoint[index_neighbour].parent = index_currPoint;
                            M1.PushBack(LevitPoint[index_neighbour].id);
                        }
                        if (LevitPoint[index_neighbour].M == 1)
                        {
                            double weightCurrEdge = Math.Sqrt(Math.Pow(LevitPoint[index_currPoint].x - LevitPoint[index_neighbour].x, 2.0) + Math.Pow(LevitPoint[index_currPoint].y - LevitPoint[index_neighbour].y, 2.0)); if (LevitPoint[index_neighbour].weight > LevitPoint[index_currPoint].weight + weightCurrEdge)
                            {
                                LevitPoint[index_neighbour].weight = LevitPoint[index_currPoint].weight + weightCurrEdge;
                                LevitPoint[index_neighbour].parent = index_currPoint;
                            }
                        }
                        if (LevitPoint[index_neighbour].M == 0)
                        {
                            double weightCurrEdge = Math.Sqrt(Math.Pow(LevitPoint[index_currPoint].x - LevitPoint[index_neighbour].x, 2.0) + Math.Pow(LevitPoint[index_currPoint].y - LevitPoint[index_neighbour].y, 2.0)); if (LevitPoint[index_neighbour].weight > LevitPoint[index_currPoint].weight + weightCurrEdge)
                            {
                                LevitPoint[index_neighbour].weight = LevitPoint[index_currPoint].weight + weightCurrEdge;
                                LevitPoint[index_neighbour].parent = index_currPoint;
                                M1.PushFront(LevitPoint[index_neighbour].id);
                            }
                        }
                    }
                    
                }
                else
                {
                    break;
                }
            }
            for (int i = 0; i < 10; i++)
            {
                LevitRoute[i] = new List<long>();
            }
            long idEndPoint = 4119187711;
            FindLevitRoute(0, idEndPoint, keys.Count);
        }

        static void FindLevitRoute(int numbRoute, long endPoint, int N)
        {
            long index_endPoint = -1;
            double weight = -1;
            for (int i = 0; i < N; ++i)
                if (LevitPoint[i].id == endPoint)
                {
                    index_endPoint = i;
                    weight = LevitPoint[i].weight;
                    break;
                }
            if (index_endPoint == -1)
            {
                Console.WriteLine("No way to the {0} point!", numbRoute);
                return;
            }
            while (index_endPoint != 0)
            {
                LevitRoute[numbRoute].Add(LevitPoint[index_endPoint].id);
                index_endPoint = LevitPoint[index_endPoint].parent;
            }
            LevitRoute[numbRoute].Add(LevitPoint[index_endPoint].id);
            foreach (long i in LevitRoute[numbRoute])
            {
                Console.WriteLine(i);
            }
            Console.WriteLine("Weight {0}", weight);
        }

        static void Main(string[] args)
        {
            string path;
            Console.WriteLine("Insert full way to osm file:");
            path = Console.ReadLine();
            Osm(path);
            Dijkstra(243340719);
            Levit(243340719);
            Console.ReadLine();
        }
    }
}
