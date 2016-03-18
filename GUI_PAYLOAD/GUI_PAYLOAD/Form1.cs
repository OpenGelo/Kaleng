//*************************************************************************************************************
//                                      ARIFAS Data Interface Version 1
//*************************************************************************************************************
//  Welcome to the source code of ARIFAS Data Interface Version 1. This program is made to competiting in KOMURINDO 2014
//  and still developing. The lack of this program are:
//  1. in serial receiveng, not consistent, parsing data still lack, etc. Try serial1.read() --> also using bytes[], bytes, serial1.BytesToRead() 
//  2. Texture of 3D Model
//  3. Can't choose map provider, and another 'little' errors in GMapNet
//  4. Image Processing under 60 seconds but still take a long time, make it under 24 seconds!
//  5. the data of image is JPEG, it's because division of system won't to take time so long. So next, change to RGB! 
//     actually, I have made it. 
//  6. Make it in one tab
//  7. Change the 2D interface with your own!
//  8. Use WPF not Windows Form *(optional)
//  9. Target is using Mavlink Protocol!
//  10. Make it for UAV APTRG!  
//  11. Use MySql or Ms. Excell to make database!
//
//*************************************************************************************************************
//                                                  READ ME!
//*************************************************************************************************************
//  for all friends in APTRG,
//
//  To Understand my program, just see in the Design of this GUI, the regions, the functions, the events and the comments
//  Click the plus (+) sign in the left side of all functions & regions to see their content
//  Search and trace its correlation with others
//  I suggest you to print this program, and do some researches, make some notes of the correlation of function, 
//  Make a block diagram to see how this program works
//
//  Keep Developing!
//                                                                              Regards, Muhamad Fadhil Abdullah
//
//************************************************************************************************************
//                    Aeromodelling and Payload Telemetry Research Group (APTRG)
//************************************************************************************************************
//  This program made by Muhamad Fadhil Abdullah (Telkom University) for 
//  Kompetisi Muatan Roket Indonesia (KOMURIDNO), a payload of sattelite competition in Indonesia (Can Sattelite)  
//  and Kompetisi Balon Atmosfer (KOMBAT) in 2014
//
//  I use external libraries for some task, such as 3D model with SharpGL, ZedGraph to make real-time graphic, and
//  GMapNet to make a map and route our Payload.
//
//  The references of this program come from Multiwii Windows GUI & PCCom.SerialCommunication Version 1.0.0.0, 
//  stackoverflow, codeproject, sourcefrog, github, wangready.wrodpress.com, youtube, google, some personal blogs,
//  my masters: Swadexi I., Subkhan Avesina, Ahmad Mubarak, Eka Puji W., Iman D. P., my seniors: Nurmajid & Riyadhi,
//  my partners: Anggara &  Ramtsal, All my friends in APTRG that have trusted me to take this competition, etc.  
//  
//                                                                            Regards, Muhamad Fadhil Abdullah
//
//************************************************************************************************************


using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Data;
using System.Drawing;
using System.Drawing.Imaging;
using System.Linq;
using System.Text;
using System.Windows.Forms;
using System.IO.Ports;
using System.IO;
using System.Text.RegularExpressions;
using System.Runtime.InteropServices;
using System.Configuration;
using System.Drawing.Drawing2D;
using System.Threading;
using System.Security.Cryptography.X509Certificates;
using System.Net;
using System.Net.Sockets;
using System.Xml; // config file
using System.Runtime.InteropServices; // dll imports
using System.Reflection;

using System.Net.NetworkInformation;
using System.Globalization;

using GUI_PAYLOAD.Properties;

using ZedGraph;

using GMap.NET;
using GMap.NET.WindowsForms;
using GMap.NET.WindowsForms.Markers;
using GMap.NET.MapProviders;

using SharpGL;
using SharpGL.SceneGraph;
using SharpGL.SceneGraph.Cameras;
using SharpGL.SceneGraph.Collections;
using SharpGL.SceneGraph.Primitives;
using SharpGL.Serialization;
using SharpGL.SceneGraph.Core;
using SharpGL.Enumerations;
using SharpGL.SceneGraph.Assets;

using PCComm;

namespace GUI_PAYLOAD
{
    #region Public Enumerations
    public enum DataMode { Text, Hex }
    public enum LogMsgType { Incoming, Outgoing, Normal, Warning, Error };
    // Various colors for logging info
    //private Color[] LogMsgTypeColor = { Color.Blue, Color.Green, Color.Black, Color.Orange, Color.Red };
    #endregion

    public partial class Form1 : Form
    {
        #region Public Variables
        CommunicationManager comm = new CommunicationManager();
        CommunicationManager commAT = new CommunicationManager();
        
        static GUI_settings gui_settings;

        static LineItem curve_acc_roll, curve_acc_pitch, curve_acc_z;
        static LineItem curve_gyro_roll, curve_gyro_pitch, curve_gyro_yaw, curve_thermo;
        static LineItem curve_alt, curve_head;

        static RollingPointPairList list_acc_roll, list_acc_pitch, list_acc_z;
        static RollingPointPairList list_gyro_roll, list_gyro_pitch, list_gyro_yaw;
        static RollingPointPairList list_alt, list_thermo;

        static double xTimeStamp = 0;
        string data_x, data_y, data_z, data_r, data_p, data_w, data_a, data_h, data_t, c1, c2, c3, c4, c5, c6;
        double roll, pitch, yaw, a, t, bearing = 000.0;
        float x, y, z, r, p, h;
        //int t, a;
        int head,tinggi;
        bool antena = false;

        static Pen drawPen;
        static System.Drawing.SolidBrush drawBrush;
        static System.Drawing.Font drawFont;

        //For logging
        /*StreamWriter wLogStream;
        StreamWriter wKMLLogStream;
        static bool bLogRunning = false;
        static bool bKMLLogRunning = false;
        static UInt32 last_mode_flags;*/
        //Contains the mode flags from the pervious log write tick

        static Int16 nav_lat, nav_lon;
        static int GPS_lat_old, GPS_lon_old;
        static bool GPSPresent = true;
        static int iWindLat = 0;
        static int iWindLon = 0;
        static int iAngleLat = 0;
        static int iAngleLon = 0;
        static double SpeedLat = 0;
        static double SpeedLon = 0;


        // static int GPS_lat_old, GPS_lon_old;

        //Routes on Map
        static GMapRoute GMRouteFlightPath;
        //static GMapRoute GMRouteMission;

        //Map Overlays
        static GMapOverlay GMOverlayFlightPath;// static so can update from gcs
        static GMapOverlay GMOverlayWaypoints;
        static GMapOverlay GMOverlayMission;
        static GMapOverlay GMOverlayLiveData;
        //static GMapOverlay GMOverlayPOI;

        static GMapProvider[] mapProviders;
        //static PointLatLng copterPos = new PointLatLng(47.402489, 19.071558);       //Just the corrds of my flying place
        static PointLatLng copterPos = new PointLatLng(-6.976957, 107.630241);
        //static PointLatLng copterPos;
        static bool isMouseDown = false;
        static bool isMouseDraging = false;

        static bool bPosholdRecorded = false;
        static bool bHomeRecorded = false;

        // markers
        GMarkerGoogle currentMarker;
        GMapMarkerRect CurentRectMarker = null;
        GMapMarker center;
        GMapMarker markerGoToClick = new GMarkerGoogle(new PointLatLng(0.0, 0.0), GMarkerGoogleType.lightblue);

        List<PointLatLng> points = new List<PointLatLng>();

        PointLatLng GPS_pos, GPS_pos_old;
        PointLatLng Home;
        PointLatLng end;
        PointLatLng start;
#endregion

        #region Local Properties
        private DataMode CurrentDataMode
        {
            get
            {
                if (radioButton1.Checked) return DataMode.Hex;
                else return DataMode.Text;
            }
            set
            {
                if (value == DataMode.Text) radioButton1.Checked = true;
                else radioButton2.Checked = true;
            }
        }
        #endregion

        public Form1()
        {
            InitializeComponent();
            //  Get the OpenGL object, for quick access.
            SharpGL.OpenGL gl = this.openGLControl1.OpenGL;
            timer1.Tick += new EventHandler(timer1_Tick);
            gl.Enable(OpenGL.GL_TEXTURE_2D);
            //backgroundWorker1.WorkerSupportsCancellation = true;
            //backgroundWorker1.DoWork += backgroundWorker1_DoWork;
            //worker.RunWorkerCompleted += worker_RunWorkerCompleted;

            //GPS_pos.Lat = 47.402489;
            //GPS_pos.Lng = 19.071558;
            #region map_setup
            // config map             
            MainMap.MinZoom = 1;
            MainMap.MaxZoom = 20;
            MainMap.CacheLocation = Path.GetDirectoryName(Application.ExecutablePath) + "/mapcache/";

            mapProviders = new GMapProvider[7];
            mapProviders[0] = GMapProviders.BingHybridMap;
            mapProviders[1] = GMapProviders.BingSatelliteMap;
            mapProviders[2] = GMapProviders.GoogleSatelliteMap;
            mapProviders[3] = GMapProviders.GoogleHybridMap;
            mapProviders[4] = GMapProviders.OviSatelliteMap;
            mapProviders[5] = GMapProviders.OviHybridMap;

            /*mapProviders = new GMapProvider[9];
            mapProviders[0] = BingHybridMapProvider.Instance;
            mapProviders[1] = BingSatelliteMapProvider.Instance;
            mapProviders[2] = GoogleSatelliteMapProvider.Instance;
            mapProviders[3] = GoogleHybridMapProvider.Instance;
            mapProviders[4] = OviMapProvider.Instance;
            mapProviders[5] = OviHybridMapProvider.Instance;
            mapProviders[6] = OpenStreetMapProvider.Instance;
            mapProviders[7] = ArcGIS_World_Street_MapProvider.Instance;
            mapProviders[8] = ArcGIS_DarbAE_Q2_2011_NAVTQ_Eng_V5_MapProvider.Instance;*/

            for (int i = 0; i < 6; i++)
            {
                cbMapProviders.Items.Add(mapProviders[i]);
            }

            // map events

            MainMap.OnPositionChanged += new PositionChanged(MainMap_OnCurrentPositionChanged);
            //MainMap.OnMarkerClick += new MarkerClick(MainMap_OnMarkerClick);
            MainMap.OnMapZoomChanged += new MapZoomChanged(MainMap_OnMapZoomChanged);
            MainMap.MouseMove += new MouseEventHandler(MainMap_MouseMove);
            MainMap.MouseDown += new MouseEventHandler(MainMap_MouseDown);
            MainMap.MouseUp += new MouseEventHandler(MainMap_MouseUp);
            MainMap.OnMarkerEnter += new MarkerEnter(MainMap_OnMarkerEnter);
            MainMap.OnMarkerLeave += new MarkerLeave(MainMap_OnMarkerLeave);

            currentMarker = new GMarkerGoogle(MainMap.Position, GMarkerGoogleType.red);
            //MainMap.MapScaleInfoEnabled = true;

            MainMap.ForceDoubleBuffer = true;
            MainMap.Manager.Mode = AccessMode.ServerAndCache;

            MainMap.Position = copterPos;

            Pen penRoute = new Pen(Color.Yellow, 3);
            Pen penScale = new Pen(Color.Blue, 3);

            MainMap.ScalePen = penScale;

            GMOverlayFlightPath = new GMapOverlay("flightpath");
            MainMap.Overlays.Add(GMOverlayFlightPath);

            GMOverlayMission = new GMapOverlay("missionroute");
            MainMap.Overlays.Add(GMOverlayMission);

            GMOverlayWaypoints = new GMapOverlay("waypoints");
            MainMap.Overlays.Add(GMOverlayWaypoints);


            GMOverlayLiveData = new GMapOverlay("livedata");
            MainMap.Overlays.Add(GMOverlayLiveData);

            GMOverlayLiveData.Markers.Clear();
            GMOverlayLiveData.Markers.Add(new GMapMarkerCopter(copterPos, 0, 0, 0));

            GMRouteFlightPath = new GMapRoute(points, "flightpath");
            GMRouteFlightPath.Stroke = penRoute;
            GMOverlayFlightPath.Routes.Add(GMRouteFlightPath);

            center = new GMarkerGoogle(MainMap.Position, GMarkerGoogleType.blue_dot);
            //center = new GMapMarkerCross(MainMap.Position);

            MainMap.Invalidate(false);
            //MainMap.Refresh();
            #endregion

           // comm.gambar = 0;
        }

        private void openGLControl1_OpenGLDraw(object sender, PaintEventArgs e)
        {

            //  The texture identifier.
            /*Texture texture = new Texture(); */

            //  Get the OpenGL object, for quick access.
            SharpGL.OpenGL gl = this.openGLControl1.OpenGL;

            //  Clear and load the identity.
            gl.Clear(OpenGL.GL_COLOR_BUFFER_BIT | OpenGL.GL_DEPTH_BUFFER_BIT);
            gl.LoadIdentity();

            //  Bind the texture.
            texture.Bind(gl);

            //  View from a bit away the y axis and a few units above the ground.
            gl.LookAt(-10, -15, 0, 0, 0, 0, 0, 1, 0);


            //  Rotate the objects every cycle.
            // gl.Rotate(rotate, 0.0f, 0.0f, 1.0f);
            //gl.Rotate(float.Parse(data_r), float.Parse(data_p), float.Parse(data_h));

            //  Move the objects down a bit so that they fit in the screen better.
            gl.Translate(0, 0, 0);

            //  Draw every polygon in the collection.
            foreach (Polygon polygon in polygons)
            {
                polygon.PushObjectSpace(gl);
                polygon.Render(gl, SharpGL.SceneGraph.Core.RenderMode.Render);
                polygon.PopObjectSpace(gl);
            }
            //  Rotate a bit more each cycle.
            rotate += 1.0f;
        }   //function to drawing 3D model


        float rotate = 0;

        //  A set of polygons to draw.
        List<Polygon> polygons = new List<Polygon>();

        //  The camera.
        SharpGL.SceneGraph.Cameras.PerspectiveCamera camera = new SharpGL.SceneGraph.Cameras.PerspectiveCamera();

        /// <summary>
        /// Handles the Click event of the importPolygonToolStripMenuItem control.
        /// </summary>
        /// <param name="sender">The source of the event.</param>
        /// <param name="e">The <see cref="System.EventArgs"/> instance containing the event data.</param>

        private void button1_Click(object sender, EventArgs e)
        {
            if (comm.isOpen() == true)
            {
                button1.Text = "Connect";
                button1.Image = Properties.Resources.connect;

                //backgroundWorker1.CancelAsync();
                System.Threading.Thread.Sleep(100);
                comm.ClosePort();

                //timer1.Stop();
                timer1.Enabled = false;
            }
            else
            {
                if (comboBox1.Text == "") { return; }  //if no port selected then do nothin' at connect


                comm.Parity = "None";
                comm.StopBits = "One";
                comm.DataBits = "8";
                comm.BaudRate = comboBox2.Text;
                comm.DisplayWindow = richTextBox1;
                comm.DisplayWindowforGambar = richTextBox2;
                comm.DisplayGambar = HasilKamera;
                comm.PortName = comboBox1.Text;
                comm.OpenPort();

                //_fasttimer = new Timer();         // Set up the timer for 3 seconds
                //timer1.Tick += new EventHandler(FastTimerEventProcessor);
                //timer1.Interval = 100;
                //timer1.Enabled = true;
                //timer1.Start();
                button1.Text = "DC";
                button1.Image = Properties.Resources.disconnect;
                
               
            }
        }

        private void Form1_Load(object sender, EventArgs e)
        {
            //—menset event handler untuk DataReceived event—
            // serialPort1.DataReceived += new System.IO.Ports.SerialDataReceivedEventHandler(serialPort1_DataReceived);

            //headingIndicatorInstrumentControl1.
            //GMarkerGoogle marker = new GMarkerGoogle(copterPos,GUI_PAYLOAD.Properties.Resources.home);
            //Set up zgMonitor control for real time monitoring
            GraphPane myPane = zedGraphControl1.GraphPane;
            myPane.Title.Text = "";
            myPane.XAxis.Title.Text = "";
            myPane.YAxis.Title.Text = "";

            

            // Save 1200 points.  At 50 ms sample rate, this is one minute
            // The RollingPointPairList is an efficient storage class that always
            // keeps a rolling set of point data without needing to shift any data values
            //RollingPointPairList list = new RollingPointPairList(1200);

            // Initially, a curve is added with no data points (list is empty)
            // Color is blue, and there will be no symbols
            //Set up pointlists and curves
            list_acc_roll = new RollingPointPairList(300);
            curve_acc_roll = myPane.AddCurve("acc_roll", list_acc_roll, Color.Red, SymbolType.None);

            list_acc_pitch = new RollingPointPairList(300);
            curve_acc_pitch = myPane.AddCurve("acc_pitch", list_acc_pitch, Color.Yellow, SymbolType.None);

            list_acc_z = new RollingPointPairList(300);
            curve_acc_z = myPane.AddCurve("acc_z", list_acc_z, Color.Blue, SymbolType.None);

            list_gyro_roll = new RollingPointPairList(300);
            curve_gyro_roll = myPane.AddCurve("gyro_roll", list_gyro_roll, Color.Khaki, SymbolType.None);

            list_gyro_pitch = new RollingPointPairList(300);
            curve_gyro_pitch = myPane.AddCurve("gyro_pitch", list_gyro_pitch, Color.Cyan, SymbolType.None);

            list_gyro_yaw = new RollingPointPairList(300);
            curve_gyro_yaw = myPane.AddCurve("gyro_yaw", list_gyro_yaw, Color.Magenta, SymbolType.None);

            list_alt = new RollingPointPairList(300);
            curve_alt = myPane.AddCurve("altitude", list_alt, Color.Green, SymbolType.None);

            list_thermo = new RollingPointPairList(300);
            curve_thermo = myPane.AddCurve("thermo", list_thermo, Color.DarkOliveGreen, SymbolType.None);

            // Sample at 50ms intervals
            //timer1.Interval = 50;
            //timer1.Enabled = true;
            //timer1.Start();

            // Just manually control the X axis range so it scrolls continuously
            // instead of discrete step-sized jumps
            myPane.XAxis.Scale.Min = 0;
            myPane.XAxis.Scale.Max = 30;
            myPane.XAxis.Scale.MinorStep = 1;
            myPane.XAxis.Scale.MajorStep = 5;

            // Show the x axis grid
            myPane.XAxis.MajorGrid.IsVisible = true;
            myPane.YAxis.MajorGrid.IsVisible = true;
            myPane.XAxis.MajorGrid.Color = Color.DarkGray;
            myPane.YAxis.MajorGrid.Color = Color.DarkGray;

            //myPane.Border.Color = Color.FromArgb(64, 64, 64);

            myPane.Chart.Fill = new Fill(Color.Black, Color.Black, 45.0f);
            //myPane.Fill = new Fill(Color.FromArgb(64, 64, 64), Color.FromArgb(64, 64, 64), 45.0f);
            myPane.Legend.IsVisible = false;
            myPane.XAxis.Scale.IsVisible = true;
            myPane.YAxis.Scale.IsVisible = true;

            foreach (ZedGraph.LineItem li in myPane.CurveList)
            {
                li.Line.Width = 3;
            }

            // Scale the axes
            zedGraphControl1.AxisChange();

            // Save the beginning time for reference
            //tickStart = Environment.TickCount;
            gui_settings = new GUI_settings();

            MainMap.Manager.Mode = AccessMode.ServerAndCache;
            if (!Stuff.PingNetwork("pingtest.com"))
            {
                MainMap.Manager.Mode = AccessMode.CacheOnly;
                MessageBox.Show("No internet connection available, going to CacheOnly mode.", "GMap.NET - Demo.WindowsForms", MessageBoxButtons.OK, MessageBoxIcon.Warning);
            }

            cbMapProviders.SelectedIndex = gui_settings.iMapProviderSelectedIndex;
            MainMap.MapProvider = mapProviders[gui_settings.iMapProviderSelectedIndex];
            //MainMap.MapProvider = BingHybridMapProvider.Instance;
            MainMap.Zoom = 18;
            MainMap.Invalidate(false);

            int w = MainMap.Size.Width;
            MainMap.Width = w + 1;
            MainMap.Width = w;
            MainMap.ShowCenter = false;


        }

        private void timer1_Tick(object sender, EventArgs e)
        {
            update_gui();
        }

        private void comboBox1_DropDown(object sender, EventArgs e)
        {
            //string[] ports = SerialPort.GetPortNames(); //nyari port/terminal yang tersedia, dimasukin ke array of string

            /*public List<string> GetAllPorts()
            {            List<String> allPorts = new List<String>();
            foreach (String portName in System.IO.Ports.SerialPort.GetPortNames())
            {
                allPorts.Add(portName);
            }
            return allPorts;
            //comboBox1.Items.Add(allPorts);

             }*/
            /*if (intlen != ports.Length)
            {
                intlen = ports.Length;
                comboBox1.Items.Clear();
                for (int j = 0; j < intlen; j++)
                {
                    comboBox1.Items.Add(ports[j]);
                }
                comboBox1.Text = ports[0];
            //}*/

            //hapus tex yg ada di combo box, karena bekas yang lama 
            comboBox1.Items.Clear();
            comm.SetPortNameValues(comboBox1);

            /*foreach (string port in ports)
            {
                comboBox1.Items.Add(port);
            }*/

            //setiap port yang terbuka, masukin namanya ke combobox
            /*foreach (String portName in ports)
            {
                comboBox1.Items.Add(portName);
            }*/
        }

        string datagambar;
        string lat, lon, data_l;
        
        string[] words = new string[15];
        private void FastTimerEventProcessor(Object myObject,
                                       EventArgs myEventArgs)
        {
            int i;
            //comm.DisplayWindow.Clear();
            // Create a string array and store the contents of the Lines property.
            string[] tempArray = comm.DisplayWindow.Lines;
            string line = tempArray[tempArray.Length - 2];
            //string line = tempArray[0];
            if (line == "")
                return;

            string[] words = Regex.Split(line, " ");
            
            //if (values.Length < 5)
            
            if (comm.gambar == 0)
            {
                c1 = words[0];
                if (c1 == "001" && line.Length == 24)
                {
                    //textBox1.Text = line;
                    //copterPos.Lat = Convert.ToDouble(words[1]);
                    //copterPos.Lng = Convert.ToDouble(words[2]);
                }

                if (c1 == "005" && words.Length == 11 )
                {
                    data_x = words[1];
                    data_y = words[2];
                    data_z = words[3];
                    data_r = words[4];
                    data_p = words[5];
                    data_h = words[6];
                    data_a = words[7];
                    data_t = words[8];
                    //data_l = words[9];
                    lat = words[9];
                    lon = words[10];

                    x = float.Parse(words[1]);
                    y = float.Parse(words[2]);
                    z = float.Parse(words[3]);
                    roll = double.Parse(words[4]);
                    pitch = double.Parse(words[5]);
                    yaw = double.Parse(words[6]);

                    r = float.Parse(words[4]);
                    p = float.Parse(words[5]);
                    h = float.Parse(words[6]);

                    head = int.Parse(words[6]);
                    //tinggi = int.Parse(words[7]);

                    #region GUIPages.mission

                    //Map update should be continous


                    GPS_pos.Lat = Convert.ToDouble(lat);
                    GPS_pos.Lng = Convert.ToDouble(lon);
               
                    GMRouteFlightPath.Points.Add(GPS_pos);
                   
                    bearing = MainMap.MapProvider.Projection.GetBearing(copterPos,GPS_pos);
                    if (270.0 > bearing && bearing >= 180.0) { bearing = 180; }
                    else if (270 <= bearing && bearing < 360) { bearing = 0; }
                    //MainMap.MapProvider.Projection.
                    
                    label21.Text = lat;
                    label22.Text = lon;


                    #endregion

                    GMOverlayLiveData.Markers.Clear();
                    MainMap.Position = GPS_pos;
                    GMOverlayLiveData.Markers.Add(new GMapMarkerCopter(GPS_pos, float.Parse(data_h), 0, 0));
                    GMOverlayLiveData.Markers.Add(new GMapMarkerlain(copterPos, 0, 0, 0));
                    MainMap.Invalidate(false);

                    double distance = MainMap.MapProvider.Projection.GetDistance(copterPos,GPS_pos);
                    distance = distance * 1000; //convert it to meters;
                    
                    double speed = distance * 10;

                    double suduttinggi = Math.Atan2(Convert.ToDouble(data_a), distance);
                    int elevasi = Convert.ToInt32(suduttinggi);
                    //center = new GMarkerGoogle(copterPos, GMarkerGoogleType.green);

                    richTextBox1.Invoke(new EventHandler(delegate
                    {
                        textBox3.Text = data_x;
                        textBox4.Text = data_y;
                        textBox5.Text = data_z;
                        textBox6.Text = data_r;
                        textBox7.Text = data_p;
                        textBox8.Text = data_t;
                        label5.Text = data_x;
                        label6.Text = data_y;
                        label7.Text = data_z;
                        label8.Text = data_p;
                        label9.Text = data_r;
                        label10.Text = data_h;
                        label11.Text = data_a;
                        label12.Text = data_t;
                        label14.Text = Convert.ToString(Convert.ToInt32(bearing));
                        label4.Text = Convert.ToString(distance);
                        //label27.Text = Convert.ToString(copterPos.Lat);
                        //label28.Text = Convert.ToString(copterPos.Lng);
                        label3.Text = Convert.ToString(elevasi);
                    }));

                    if (antena)
                    {
                        //commAT.WriteData("005 " + Convert.ToString(bearing) + " " + label11.Text);
                        commAT.WriteData("P" + label11.Text + "Y" + Convert.ToString(Convert.ToInt32(bearing)) + "L");
                    }
                }
            }
            

            /*if (comm.gambar == 2)
            {
                //comm.DisplayWindowforGambar.Clear();
                richTextBox2.Invoke(new EventHandler(delegate
                {
                    richTextBox2.AppendText(words[0]);
                }));
                if (richTextBox2.Text.IndexOf("FFD9") == 0)
                {
                    //ajajaja();
                    comm.WriteData("x");
                    comm.gambar = 0;
                    //ajajaja();
                    //MessageBox.Show("no");
                }
                else
                {
                    
                    
                }
                //ajajaja();
            }*/
            /*if (c1 == "ff")
            { 
                datagambar = words[1];
                richTextBox2.Invoke(new EventHandler(delegate
                {
                   richTextBox2.AppendText(datagambar);
                }));
                if (datagambar.IndexOf("FFD9") != -1)
                {
                    comm.WriteData("x");
                    byte[] v = stringtobyte(richTextBox2.Text);
                    pictureBox1.Image = byteArrayToImage(v);
                }
                
            
            }

            if (line == "005FF")
            {
                richTextBox1.Invoke(new EventHandler(delegate
                {
                    //richTextBox2.AppendText(line);
                }));
            }*/
            //Double seconds_elapsed = Double.Parse(values[7]);

            textBox1.Text = line; 
        }   //main process, receiving data, send value to GUI

        private byte[] stringtobyte(string strinput)
        {
            int i = 0;
            int x = 0;
            byte[] bytes = new byte[(strinput.Length) / 2];
            while (strinput.Length > i + 1)
            {
                long lgndecimal = Convert.ToInt32(strinput.Substring(i, 2), 16);
                bytes[x] = Convert.ToByte(lgndecimal);
                i = i + 2;
                ++x;
            }
            return bytes;
        }

        public Image byteArrayToImage(byte[] byteArrayIn)
        {
            MemoryStream ms = new MemoryStream(byteArrayIn);
            Image returnImage = Image.FromStream(ms);
            return returnImage;
        }

        /// <summary> Converts an array of bytes into a formatted string of hex digits (ex: E4 CA B2)</summary>
        /// <param name="data"> The array of bytes to be translated into a string of hex digits. </param>
        /// <returns> Returns a well formatted string of hex digits with spacing. </returns>
        private string ByteArrayToHexString(byte[] data)
        {
            StringBuilder sb = new StringBuilder(data.Length * 3);
            foreach (byte b in data)
                sb.Append(Convert.ToString(b, 16).PadLeft(2, '0').PadRight(3, ' '));
            return sb.ToString().ToUpper();
        }

        private void Log(LogMsgType msgtype, string msg)
        {
            richTextBox1.Invoke(new EventHandler(delegate
            {
                richTextBox1.SelectedText = string.Empty;
                //rtfTerminal.SelectionFont = new Font(rtfTerminal.SelectionFont, FontStyle.Bold);
                //rtfTerminal.SelectionColor = LogMsgTypeColor[(int)msgtype];
                richTextBox1.AppendText(msg);
                richTextBox1.ScrollToCaret();
                //data_x = richTextBox1.Text.Substring(6, 9);
                //textBox2.Text = data_x;
            }));
        }

        //—Delegate and subroutine untuk ditampilkan pada TextBox control—
        public delegate void myDelegate();

        public void updateTextBox()
        {
            //—menambahkan data pada TextBox control—
            textBox2.Text = data_x; //kalau mau klik dc, malah error disini
        }

        private void button4_Click(object sender, EventArgs e)
        {
            timer1.Stop();                       //Stop timer(s), whatever it takes
            //backgroundWorker1.CancelAsync();
            //System.Threading.Thread.Sleep(500);         //Wait for 1 cycle to let backgroundworker finish it's last job.
            comm.ClosePort();
            Close();
        }
        

        private void update_gui()
        {
            /*if (tabControl1.SelectedIndex == 2)
            {
                #region GUIPages.mission

                //Map update should be continous

                // if (mw_gui.GPS_latitude != 0)
                //{
                GPS_pos.Lat = Convert.ToDouble(lat) / 1000000;
                GPS_pos.Lng = Convert.ToDouble(lon) / 1000000;
                //
                //     GMRouteFlightPath.Points.Add(GPS_pos);
                // }

                //GPS_pos.Lat = Convert.ToDouble(lat) + 78.074233;
                //GPS_pos.Lng = Convert.ToDouble(lon) - 72.36983;
                //GPS_pos.Lat = Convert.ToDouble(lat) + 0.0;
                //GPS_pos.Lng = Convert.ToDouble(lon) +0.0 ;
                //GPS_pos.Lat = Convert.ToDouble(lat);
                //GPS_pos.Lng = Convert.ToDouble(lon);
                GMRouteFlightPath.Points.Add(GPS_pos);
                //label21.Text = lat;
                //label22.Text = Convert.ToString(Convert.ToDouble(lon)+0.00000001);
                label21.Text = lat;
                label22.Text = lon;

                #endregion

                GMOverlayLiveData.Markers.Clear();
                MainMap.Position = GPS_pos;
                GMOverlayLiveData.Markers.Add(new GMapMarkerCopter(GPS_pos, float.Parse(data_h), 0, 0));
                MainMap.Invalidate(false);
                //GPS_pos.Lat += (0.0000009009 * iWindLat) + (0.0000009009 * SpeedLat);
                //GPS_pos.Lng += (0.0000009009 * iWindLon) + (0.0000009009 * SpeedLon);


                double distance = MainMap.MapProvider.Projection.GetDistance(GPS_pos, GPS_pos_old);
                distance = distance * 1000; //convert it to meters;

                double speed = distance * 10;

                //GMRouteFlightPath.Points.Add(GPS_pos);
                //MainMap.Position = GPS_pos;
                //MainMap.Invalidate(false);

                //----lGPS_lat.Text = Convert.ToString((decimal)mw_gui.GPS_latitude / 10000000);
                //----lGPS_lon.Text = Convert.ToString((decimal)mw_gui.GPS_longitude / 10000000);

                //----PointLatLng GPS_home = new PointLatLng((double)mw_gui.GPS_home_lat / 10000000, (double)mw_gui.GPS_home_lon / 10000000);
                //----GMOverlayLiveData.Markers.Add(new GMapMarkerHome(GPS_home));


                //MainMap.Invalidate(false);
            }*/

               
                if (checkBox1.Checked) { list_acc_roll.Add(xTimeStamp, Convert.ToDouble(data_x)); }
                if (checkBox2.Checked) { list_acc_pitch.Add(xTimeStamp, Convert.ToDouble(data_y)); }
                if (checkBox3.Checked) { list_acc_z.Add(xTimeStamp, Convert.ToDouble(data_z)); }
                if (checkBox4.Checked) { list_gyro_roll.Add(xTimeStamp, Convert.ToDouble(data_r)); }
                if (checkBox5.Checked) { list_gyro_pitch.Add(xTimeStamp, Double.Parse(data_p)); }
                if (checkBox6.Checked) { list_gyro_yaw.Add(xTimeStamp, Double.Parse(data_h)); }
                if (checkBox7.Checked) { list_alt.Add(xTimeStamp, Convert.ToDouble(data_a)); }
                if (checkBox8.Checked) { list_thermo.Add(xTimeStamp, Convert.ToDouble(data_t)); }

                xTimeStamp = xTimeStamp + 1;

                Scale xScale = zedGraphControl1.GraphPane.XAxis.Scale;
                if (xTimeStamp > xScale.Max - xScale.MajorStep)
                {
                    xScale.Max = xTimeStamp + xScale.MajorStep;
                    xScale.Min = xScale.Max - 30.0;
                }

                zedGraphControl1.AxisChange();

                zedGraphControl1.Invalidate();

                curve_acc_roll.IsVisible = checkBox1.Checked;
                curve_acc_pitch.IsVisible = checkBox2.Checked;
                curve_acc_z.IsVisible = checkBox3.Checked;
                curve_gyro_roll.IsVisible = checkBox4.Checked;
                curve_gyro_pitch.IsVisible = checkBox5.Checked;
                curve_gyro_yaw.IsVisible = checkBox6.Checked;
                curve_alt.IsVisible = checkBox7.Checked;
                curve_thermo.IsVisible = checkBox8.Checked;

                attitudeIndicatorInstrumentControl1.SetArtificalHorizon(pitch, roll);

                //axiThermometerX1.Position = Convert.ToDouble(data_t);
                //airSpeedIndicatorInstrumentControl1.SetAirSpeedIndicatorParameters();
                //verticalSpeedIndicatorInstrumentControl1.SetVerticalSpeedIndicatorParameters(

                altitude_meter1.SetAlimeterParameters(tinggi);

                headingIndicatorInstrumentControl1.SetHeadingIndicatorParameters(head);

                turnCoordinatorInstrumentControl1.SetTurnCoordinatorParameters(r,r);

                foreach (Polygon polygon in polygons)
                {
                    polygon.Transformation.RotateX = h;
                    polygon.Transformation.RotateY = r;
                    polygon.Transformation.RotateZ = p;
                }
            
        }

        private void importPolygonToolStripMenuItem_Click(object sender, EventArgs e)
        {
            //  Show a file open dialog.
            OpenFileDialog openDialog = new OpenFileDialog();
            openDialog.Filter = SerializationEngine.Instance.Filter;
            if (openDialog.ShowDialog() == DialogResult.OK)
            {
                Scene scene = SerializationEngine.Instance.LoadScene(openDialog.FileName);
                if (scene != null)
                {
                    foreach (var polygon in scene.SceneContainer.Traverse<Polygon>())
                    {
                        //  Get the bounds of the polygon.
                        BoundingVolume boundingVolume = polygon.BoundingVolume;
                        float[] extent = new float[3];
                        polygon.BoundingVolume.GetBoundDimensions(out extent[0], out extent[1], out extent[2]);

                        //  Get the max extent.
                        float maxExtent = extent.Max();

                        //  Scale so that we are at most 10 units in size.
                        float scaleFactor = maxExtent > 10 ? 10.0f / maxExtent : 1;
                        polygon.Transformation.ScaleX = scaleFactor;
                        polygon.Transformation.ScaleY = scaleFactor;
                        polygon.Transformation.ScaleZ = scaleFactor;
                        polygon.Freeze(openGLControl1.OpenGL);
                        polygons.Add(polygon);
                    }
                }
            }
        }   //import 3D model

        private void solidToolStripMenuItem_Click(object sender, EventArgs e)
        {
            wireframeToolStripMenuItem.Checked = false;
            solidToolStripMenuItem.Checked = true;
            lightedToolStripMenuItem.Checked = false;
            openGLControl1.OpenGL.PolygonMode(FaceMode.FrontAndBack, PolygonMode.Filled);
            openGLControl1.OpenGL.Disable(OpenGL.GL_LIGHTING);
        }   //solid texture

        private void wireframeToolStripMenuItem_Click(object sender, EventArgs e)
        {
            wireframeToolStripMenuItem.Checked = true;
            solidToolStripMenuItem.Checked = false;
            lightedToolStripMenuItem.Checked = false;
            openGLControl1.OpenGL.PolygonMode(FaceMode.FrontAndBack, PolygonMode.Lines);
            openGLControl1.OpenGL.Disable(OpenGL.GL_LIGHTING);
        }  //wire frame texture

        private void lightedToolStripMenuItem_Click(object sender, EventArgs e)
        {
            wireframeToolStripMenuItem.Checked = false;
            solidToolStripMenuItem.Checked = false;
            lightedToolStripMenuItem.Checked = true;
            openGLControl1.OpenGL.PolygonMode(FaceMode.FrontAndBack, PolygonMode.Filled);
            openGLControl1.OpenGL.Enable(OpenGL.GL_LIGHTING);
            openGLControl1.OpenGL.Enable(OpenGL.GL_LIGHT0);
            openGLControl1.OpenGL.Enable(OpenGL.GL_COLOR_MATERIAL);
        }      

        //  The texture identifier.
        Texture texture = new Texture();
        private void importTextureToolStripMenuItem_Click(object sender, EventArgs e)
        {
            //OpenFileDialog openFileDialog1 = new OpenFileDialog();

            //  Show a file open dialog.
            if (openFileDialog1.ShowDialog() == DialogResult.OK)
            {
                //  Destroy the existing texture.
                texture.Destroy(openGLControl1.OpenGL);

                //  Create a new texture.
                texture.Create(openGLControl1.OpenGL, openFileDialog1.FileName);

                //  Redraw.
                openGLControl1.Invalidate();
            }
        }   //import texture, texturing to 3D model still failed

        private void freezeToolStripMenuItem_Click(object sender, EventArgs e)
        {
            foreach (var poly in polygons)
                poly.Freeze(openGLControl1.OpenGL);
        }      //freeze the 3D MODEL

        private void unfreezeToolStripMenuItem_Click(object sender, EventArgs e)
        {
            foreach (var poly in polygons)
                poly.Unfreeze(openGLControl1.OpenGL);
        }

        private void clearToolStripMenuItem_Click(object sender, EventArgs e)
        {
            SharpGL.OpenGL gl = this.openGLControl1.OpenGL;
            polygons.Clear();
            texture.Destroy(gl);
        }

        private void exitToolStripMenuItem_Click(object sender, EventArgs e)
        {
            this.Close();
        }



        // MapZoomChanged
        void MainMap_OnMapZoomChanged()
        {
            if (MainMap.Zoom > 0)
            {
                //tb_mapzoom.Value = (int)(MainMap.Zoom);
                center.Position = MainMap.Position;
            }
        }

        // current point changed
        void MainMap_OnCurrentPositionChanged(PointLatLng point)
        {
            if (point.Lat > 90) { point.Lat = 90; }
            if (point.Lat < -90) { point.Lat = -90; }
            if (point.Lng > 180) { point.Lng = 180; }
            if (point.Lng < -180) { point.Lng = -180; }
            center.Position = point;
            //LMousePos.Text = "Lat:" + String.Format("{0:0.000000}", point.Lat) + " Lon:" + String.Format("{0:0.000000}", point.Lng);

        }   //on current position changed

        void MainMap_OnMarkerLeave(GMapMarker item)
        {
            if (!isMouseDown)
            {
                if (item is GMapMarkerRect)
                {

                    CurentRectMarker = null;

                    GMapMarkerRect rc = item as GMapMarkerRect;
                    rc.Pen.Color = Color.Blue;
                    MainMap.Invalidate(false);
                }
            }
        }

        void MainMap_OnMarkerEnter(GMapMarker item)
        {
            if (!isMouseDown)
            {
                if (item is GMapMarkerRect)
                {
                    GMapMarkerRect rc = item as GMapMarkerRect;
                    rc.Pen.Color = Color.Red;
                    MainMap.Invalidate(false);

                    CurentRectMarker = rc;
                }
            }
        }

        void MainMap_MouseUp(object sender, MouseEventArgs e)
        {
            end = MainMap.FromLocalToLatLng(e.X, e.Y);

            if (isMouseDown) // mouse down on some other object and dragged to here.
            {
                if (e.Button == MouseButtons.Left)
                {
                    isMouseDown = false;
                }
                if (!isMouseDraging)
                {
                    if (CurentRectMarker != null)
                    {
                        // cant add WP in existing rect
                    }
                    else
                    {
                        //addWP("WAYPOINT", 0, currentMarker.Position.Lat, currentMarker.Position.Lng, iDefAlt);
                    }
                }
                else
                {
                    if (CurentRectMarker != null)
                    {
                        //update existing point in datagrid
                    }
                }
            }
            if (comm.isOpen() == true) timer1.Start();
            isMouseDraging = false;
        }

        void MainMap_MouseDown(object sender, MouseEventArgs e)
        {
            start = MainMap.FromLocalToLatLng(e.X, e.Y);

            if (e.Button == MouseButtons.Left && Control.ModifierKeys != Keys.Alt)
            {
                isMouseDown = true;
                isMouseDraging = false;

                if (currentMarker.IsVisible)
                {
                    currentMarker.Position = MainMap.FromLocalToLatLng(e.X, e.Y);
                }
            }
        }

        // move current marker with left holding
        void MainMap_MouseMove(object sender, MouseEventArgs e)
        {

            PointLatLng point = MainMap.FromLocalToLatLng(e.X, e.Y);

            currentMarker.Position = point;
            label23.Text = "Lat:" + String.Format("{0:0.000000}", point.Lat) + " Lon:" + String.Format("{0:0.000000}", point.Lng);

            if (!isMouseDown)
            {

            }


            //draging
            if (e.Button == MouseButtons.Left && isMouseDown)
            {
                isMouseDraging = true;
                if (CurentRectMarker == null) // left click pan
                {
                    double latdif = start.Lat - point.Lat;
                    double lngdif = start.Lng - point.Lng;
                    MainMap.Position = new PointLatLng(center.Position.Lat + latdif, center.Position.Lng + lngdif);
                }
                else
                {
                    if (comm.isOpen() == true) timer1.Stop();
                    PointLatLng pnew = MainMap.FromLocalToLatLng(e.X, e.Y);
                    if (currentMarker.IsVisible)
                    {
                        currentMarker.Position = pnew;
                    }
                    CurentRectMarker.Position = pnew;

                    if (CurentRectMarker.InnerMarker != null)
                    {
                        CurentRectMarker.InnerMarker.Position = pnew;
                        GPS_pos = pnew;
                    }
                }
            }
        }   //when mouse move on map

        private void cbMapProviders_SelectedIndexChanged(object sender, EventArgs e)
        {

            this.Cursor = Cursors.WaitCursor;
            //MainMap.MapProvider = GMapProviders.GoogleSatelliteMap;
            MainMap.MapProvider = (GMapProvider)cbMapProviders.SelectedItem;
            MainMap.MinZoom = 5;
            MainMap.MaxZoom = 20;
            MainMap.Zoom = 18;
            MainMap.Invalidate(false);
            gui_settings.iMapProviderSelectedIndex = cbMapProviders.SelectedIndex;
            //gui_settings.save_to_xml(sGuiSettingsFilename);


            this.Cursor = Cursors.Default;

        }   //choose your map provider in combobox


        /// <summary>
        /// used to override the drawing of the waypoint box bounding
        /// </summary>
        public class GMapMarkerRect : GMapMarker
        {
            public Pen Pen = new Pen(Brushes.White, 2);

            public Color Color { get { return Pen.Color; } set { Pen.Color = value; } }

            public GMapMarker InnerMarker;

            public int wprad = 0;
            public GMapControl MainMap;

            public GMapMarkerRect(PointLatLng p)
                : base(p)
            {
                Pen.DashStyle = DashStyle.Dash;

                // do not forget set Size of the marker
                // if so, you shall have no event on it ;}
                Size = new System.Drawing.Size(50, 50);
                Offset = new System.Drawing.Point(-Size.Width / 2, -Size.Height / 2 - 20);
            }

            public override void OnRender(Graphics g)
            {
                base.OnRender(g);

                if (wprad == 0 || MainMap == null)
                    return;

                // undo autochange in mouse over
                if (Pen.Color == Color.Blue)
                    Pen.Color = Color.White;

                double width = (MainMap.MapProvider.Projection.GetDistance(MainMap.FromLocalToLatLng(0, 0), MainMap.FromLocalToLatLng(MainMap.Width, 0)) * 1000.0);
                double height = (MainMap.MapProvider.Projection.GetDistance(MainMap.FromLocalToLatLng(0, 0), MainMap.FromLocalToLatLng(MainMap.Height, 0)) * 1000.0);
                double m2pixelwidth = MainMap.Width / width;
                double m2pixelheight = MainMap.Height / height;

                GPoint loc = new GPoint((int)(LocalPosition.X - (m2pixelwidth * wprad * 2)), LocalPosition.Y);// MainMap.FromLatLngToLocal(wpradposition);
                g.DrawArc(Pen, new System.Drawing.Rectangle((int)(LocalPosition.X - Offset.X - (Math.Abs(loc.X - LocalPosition.X) / 2)), (int)(LocalPosition.Y - Offset.Y - Math.Abs(loc.X - LocalPosition.X) / 2), (int)(Math.Abs(loc.X - LocalPosition.X)), (int)(Math.Abs(loc.X - LocalPosition.X))), 0, 360);

            }
        }       // default marker

        public class GMapMarkerCopter : GMapMarker
        {
            const float rad2deg = (float)(180 / Math.PI);
            const float deg2rad = (float)(1.0 / rad2deg);

            static readonly System.Drawing.Size SizeSt = new System.Drawing.Size(global::GUI_PAYLOAD.Properties.Resources.marker_quadx.Width, global::GUI_PAYLOAD.Properties.Resources.marker_quadx.Height);
            float heading = 0;
            float cog = -1;
            float target = -1;
            //byte coptertype;

            //public GMapMarkerCopter(PointLatLng p, float heading, float cog, float target, byte coptertype)
            public GMapMarkerCopter(PointLatLng p, float heading, float cog, float target)
                : base(p)
            {
                this.heading = heading;
                this.cog = cog;
                this.target = target;
                //this.coptertype = coptertype;
                Size = SizeSt;
            }

            public override void OnRender(Graphics g)
            {
                System.Drawing.Drawing2D.Matrix temp = g.Transform;
                g.TranslateTransform(LocalPosition.X, LocalPosition.Y);

                Image pic = global::GUI_PAYLOAD.Properties.Resources.marker_quadx;


                int length = 100;
                // anti NaN
                g.DrawLine(new Pen(Color.Red, 2), 0.0f, 0.0f, (float)Math.Cos((heading - 90) * deg2rad) * length, (float)Math.Sin((heading - 90) * deg2rad) * length);
                //g.DrawLine(new Pen(Color.Black, 2), 0.0f, 0.0f, (float)Math.Cos((cog - 90) * deg2rad) * length, (float)Math.Sin((cog - 90) * deg2rad) * length);
                g.DrawLine(new Pen(Color.Orange, 2), 0.0f, 0.0f, (float)Math.Cos((target - 90) * deg2rad) * length, (float)Math.Sin((target - 90) * deg2rad) * length);
                // anti NaN
                g.RotateTransform(heading);
                g.DrawImageUnscaled(pic, pic.Width / -2, pic.Height / -2);
                g.Transform = temp;
            }
        }   //if the marker is a copter

        public class GMapMarkerlain : GMapMarker
        {
            const float rad2deg = (float)(180 / Math.PI);
            const float deg2rad = (float)(1.0 / rad2deg);

            static readonly System.Drawing.Size SizeSt = new System.Drawing.Size(global::GUI_PAYLOAD.Properties.Resources.home.Width, global::GUI_PAYLOAD.Properties.Resources.home.Height);
            //static readonly System.Drawing.Size SizeSt = new System.Drawing.Size(global::GUI_PAYLOAD.Properties.Resources.marker_poi.Width, global::GUI_PAYLOAD.Properties.Resources.marker_poi.Height);
            float heading = 0;
            float cog = -1;
            float target = -1;
            //byte coptertype;

            //public GMapMarkerCopter(PointLatLng p, float heading, float cog, float target, byte coptertype)
            public GMapMarkerlain(PointLatLng p, float heading, float cog, float target)
                : base(p)
            {
                this.heading = heading;
                this.cog = cog;
                this.target = target;
                //this.coptertype = coptertype;
                Size = SizeSt;
            }

            public override void OnRender(Graphics g)
            {
                System.Drawing.Drawing2D.Matrix temp = g.Transform;
                g.TranslateTransform(LocalPosition.X, LocalPosition.Y);

//                Image pic = global::GUI_PAYLOAD.Properties.Resources.marker_poi;
                Image pic = global::GUI_PAYLOAD.Properties.Resources.home;

                g.DrawImageUnscaled(pic, pic.Width / -2, pic.Height / -2);
                g.Transform = temp;

               // int length = 100;
                // anti NaN
               /* g.DrawLine(new Pen(Color.Red, 2), 0.0f, 0.0f, (float)Math.Cos((heading - 90) * deg2rad) * length, (float)Math.Sin((heading - 90) * deg2rad) * length);
                //g.DrawLine(new Pen(Color.Black, 2), 0.0f, 0.0f, (float)Math.Cos((cog - 90) * deg2rad) * length, (float)Math.Sin((cog - 90) * deg2rad) * length);
                g.DrawLine(new Pen(Color.Orange, 2), 0.0f, 0.0f, (float)Math.Cos((target - 90) * deg2rad) * length, (float)Math.Sin((target - 90) * deg2rad) * length);
                // anti NaN
                g.RotateTransform(heading);
                g.DrawImageUnscaled(pic, pic.Width / -2, pic.Height / -2);
                g.Transform = temp;*/
            }
        }   //another marker

        public class GUI_settings
        {
            public int iMapProviderSelectedIndex { get; set; }
            public GUI_settings()
            {
                iMapProviderSelectedIndex = 1;  //Bing Map
            }
        }

        public class Stuff
        {
            public static bool PingNetwork(string hostNameOrAddress)
            {
                bool pingStatus = false;

                using (Ping p = new Ping())
                {
                    byte[] buffer = Encoding.ASCII.GetBytes("aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa");
                    int timeout = 4444; // 4s

                    try
                    {
                        PingReply reply = p.Send(hostNameOrAddress, timeout, buffer);
                        pingStatus = (reply.Status == IPStatus.Success);
                    }
                    catch (Exception)
                    {
                        pingStatus = false;
                    }
                }

                return pingStatus;
            }
        }
        private void button3_Click(object sender, EventArgs e)
        {
            
            timer1.Stop();
            antena = false;
            comm.WriteData("x");
            //System.Threading.Thread.Sleep(10000);
            comm.WriteData("2");
            comm.gambar = 2;
        }

        private void button6_Click(object sender, EventArgs e)
        {
            timer1.Stop();
            antena = false;

            comm.WriteData("x");
            System.Threading.Thread.Sleep(1000);
            //commAT.WriteData("x");
        }

        private void button2_Click(object sender, EventArgs e)
        {
            richTextBox2.Clear();
            timer1.Tick += new EventHandler(FastTimerEventProcessor);
            timer1.Interval = 100;
            timer1.Enabled = true;
            timer1.Start();
            comm.WriteData("1");
            comm.gambar = 0;
        }

        private void button7_Click(object sender, EventArgs e)
        {
            RectLatLng area = MainMap.SelectedArea;
            if (area.IsEmpty)
            {
                DialogResult res = MessageBox.Show("No ripp area defined, ripp displayed on screen?", "Rip", MessageBoxButtons.YesNo);
                if (res == DialogResult.Yes)
                {
                    area = MainMap.ViewArea;
                }
            }

            if (!area.IsEmpty)
            {
                DialogResult res = MessageBox.Show("Ready ripp at Zoom = " + (int)MainMap.Zoom + " ?", "GMap.NET", MessageBoxButtons.YesNo);

                for (int i = 1; i <= MainMap.MaxZoom; i++)
                {
                    if (res == DialogResult.Yes)
                    {
                        TilePrefetcher obj = new TilePrefetcher();
                        obj.ShowCompleteMessage = false;
                        obj.Start(area, i, MainMap.MapProvider, 100, 0);

                    }
                    else if (res == DialogResult.No)
                    {
                        continue;
                    }
                    else if (res == DialogResult.Cancel)
                    {
                        break;
                    }
                }
            }
            else
            {
                MessageBox.Show("Select map area holding ALT", "GMap.NET", MessageBoxButtons.OK, MessageBoxIcon.Exclamation);
            }

        }

        private void button5_Click(object sender, EventArgs e)
        {

        }

        private void button8_Click(object sender, EventArgs e)
        {
            string replacement = Regex.Replace(richTextBox2.Text, @"\t|\n|\r|\s", "");
            richTextBox3.Invoke(new EventHandler(delegate
            {
                richTextBox3.AppendText(replacement);
            }));
            string hapus = Regex.Replace(replacement, "port", string.Empty);
            byte[] v = stringtobyte(replacement);
            HasilKamera.Image = byteArrayToImage(v);   
        }

        private void ajajaja()
        {
            string replacement = Regex.Replace(richTextBox2.Text, @"\t|\n|\r|\s", "");
            richTextBox3.Invoke(new EventHandler(delegate
            {
                richTextBox3.AppendText(replacement);
            }));
            string hapus = Regex.Replace(replacement, "port", string.Empty);
            byte[] v = stringtobyte(replacement);
            HasilKamera.Image = byteArrayToImage(v);
        }  //it's not used

        private void button9_Click(object sender, EventArgs e)
        {
            if (commAT.isOpen() == true)
            {
                button9.Text = "Connect";
                button9.Image = Properties.Resources.connect;

                //backgroundWorker1.CancelAsync();
                System.Threading.Thread.Sleep(100);
                comm.ClosePort();

                //timer2.Stop();
                //timer2.Enabled = false;
            }
            else
            {
                if (comboBox3.Text == "") { return; }  //if no port selected then do nothin' at connect
                //Assume that the selection in the combobox for port is still valid
                //—mengatur beberapa parameter untuk koneksi serialport                                                   
                /*serialPort1.PortName = comboBox1.Text; //nama port/terminal-nya
                serialPort1.BaudRate = int.Parse(comboBox2.Text); //baudrate-kecepatan data, konversi dari string ke integer
                serialPort1.Parity = System.IO.Ports.Parity.None; 
                serialPort1.DataBits = 8; //terima-kirim 8 bit
                serialPort1.StopBits = StopBits.One;
                serialPort1.Handshake = Handshake.None;*/

                commAT.Parity = "None";
                commAT.StopBits = "One";
                commAT.DataBits = "8";
                commAT.BaudRate = comboBox4.Text;
                commAT.DisplayWindow = richTextBox1;
                //commAT.DisplayWindowforGambar = richTextBox2;
                commAT.PortName = comboBox3.Text;
                commAT.OpenPort();

                //_fasttimer = new Timer();         // Set up the timer for 3 seconds
                //timer2.Tick += new EventHandler(FastTimerEventProcessor);
                //timer2.Interval = 100;
                //timer2.Enabled = true;

                button9.Text = "DC";
                button9.Image = Properties.Resources.disconnect;
                timer2.Start();
                /*
                //Run BackgroundWorker
                if (!backgroundWorker1.IsBusy)
                {
                    try
                    {
                        backgroundWorker1.RunWorkerAsync();
                    }
                    catch { MessageBox.Show("ga jalan euy"); }
                }
                    
                System.Threading.Thread.Sleep(1000);*/
            }
        }

        private void comboBox3_DropDown(object sender, EventArgs e)
        {
            comboBox3.Items.Clear();
            commAT.SetPortNameValues(comboBox3);
        }

        private void button10_Click(object sender, EventArgs e)
        {
            //commAT.WriteData("005 "+ Convert.ToString(bearing) +" " + label22.Text);
            //commAT.WriteData("P" + "45"+ "Y" + Convert.ToString(Convert.ToInt32(bearing)) + "L");
            //commAT.WriteData("P" + "45" + "Y" + "000045" + "L");
            antena = true;
        }

        private void button11_Click(object sender, EventArgs e)
        {
            comm.WriteData("h");
            //textBox1.Text = richTextBox1.Text;
            string[] words = Regex.Split(textBox11.Text, " ");
            string head = words[0];
            //if (head == "005" && textBox1.Text.Length == 24)
            if (head == "005")
            {
                /*label27.Text = Convert.ToString(copterPos.Lat);
                label28.Text = Convert.ToString(copterPos.Lng);*/
                label27.Text = words[1];
                label28.Text = words[2];
                copterPos.Lat = Convert.ToDouble(label27.Text);
                copterPos.Lng = Convert.ToDouble(label28.Text);
                center = new GMarkerGoogle(copterPos, GMarkerGoogleType.green);
                GMOverlayLiveData.Markers.Add(new GMapMarkerlain(copterPos, 0, 0, 0));
            }
            
        }
    }
}

