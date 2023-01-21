using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Globalization;
using System.IO;
using System.Net;
using System.Net.Sockets;
using System.Windows.Forms;

namespace FlasherX
{
    public partial class Form1 : Form
    {
        private Socket UDPSocket;

        private IPEndPoint epModule;

        private EndPoint epUDPSender = new IPEndPoint(IPAddress.Any, 0);

        Dictionary<string, byte> hexindex = new Dictionary<string, byte>();
        // Data stream
        private byte[] buffer = new byte[1024];

        public Form1()
        {
            InitializeComponent();

            hexindex.Clear();
            for (int i = 0; i <= 255; i++)
                hexindex.Add(i.ToString("X2"), (byte)i);
            hexindex.Add("::", 0x3a);

            try //udp network
            {
                // Initialise the socket
                UDPSocket = new Socket(AddressFamily.InterNetwork, SocketType.Dgram, ProtocolType.Udp);
                UDPSocket.SetSocketOption(SocketOptionLevel.Socket, SocketOptionName.Broadcast, true);
                UDPSocket.Bind(new IPEndPoint(IPAddress.Any, 9999));
                UDPSocket.BeginReceiveFrom(buffer, 0, buffer.Length, SocketFlags.None, ref epUDPSender, new AsyncCallback(ReceiveDataUDPAsync), null);

                // AgIO sends to this endpoint - usually 192.168.1.255:8888
                epModule = new IPEndPoint(IPAddress.Parse("192.168.5.255"), 6666);

            }
            catch (Exception e)
            {
                //WriteErrorLog("UDP Server" + e);
                MessageBox.Show("Load Error: " + e.Message, "UDP Server", MessageBoxButtons.OK, MessageBoxIcon.Error);
            }
        }

        //sends byte array
        public void SendUDPMessage(byte[] byteData, IPEndPoint endPoint)
        {
            try
            {
                // Send packet to the zero
                if (byteData.Length != 0)
                {
                    UDPSocket.BeginSendTo(byteData, 0, byteData.Length, SocketFlags.None, endPoint, new AsyncCallback(SendDataUDPAsync), null);
                }
            }
            catch (Exception)
            {
                //WriteErrorLog("Sending UDP Message" + e.ToString());
                //MessageBox.Show("Send Error: " + e.Message, "UDP Client", MessageBoxButtons.OK,
                //MessageBoxIcon.Error);
            }
        }

        private void SendDataUDPAsync(IAsyncResult asyncResult)
        {
            try
            {
                UDPSocket.EndSend(asyncResult);
            }
            catch (Exception)
            {
                //WriteErrorLog(" UDP Send Data" + e.ToString());
                //MessageBox.Show("SendData Error: " + e.Message, "UDP Server", MessageBoxButtons.OK,
                //MessageBoxIcon.Error);
            }
        }

        private void ReceiveDataUDPAsync(IAsyncResult asyncResult)
        {
            try
            {
                // Receive all data
                int msgLen = UDPSocket.EndReceiveFrom(asyncResult, ref epUDPSender);

                byte[] localMsg = new byte[msgLen];
                Array.Copy(buffer, localMsg, msgLen);

                // Listen for more connections again...
                UDPSocket.BeginReceiveFrom(buffer, 0, buffer.Length, SocketFlags.None, ref epUDPSender, new AsyncCallback(ReceiveDataUDPAsync), null);

                BeginInvoke((MethodInvoker)(() => ReceiveFromUDP(localMsg)));
            }
            catch (Exception)
            {
                //WriteErrorLog("UDP Recv data " + e.ToString());
                //MessageBox.Show("ReceiveData Error: " + e.Message, "UDP Server", MessageBoxButtons.OK,
                //MessageBoxIcon.Error);
            }
        }

        private void ReceiveFromUDP(byte[] data)
        {
            if (data.Length == 9 && data[0] == 0x4f && data[1] == 0x54 && data[2] == 0x41 && data[3] == 0x55 && data[4] == 0x70)
            {
                int lines = data[5] | (data[6] << 8) | (data[7] << 16) | (data[8] << 24);

                if (totallines == lines)
                    SendUDPMessage(new byte[] { 0x3a, 0x00, 0x00, 0x00, 0x06, 0xFA }, epModule);
                else
                    SendUDPMessage(new byte[] { 0x3a, 0x00, 0x00, 0x00, 0x07, 0xF9 }, epModule);
            }
        }

        private void button1_Click(object sender, EventArgs e)
        {
            //create the dialog instance
            OpenFileDialog ofd = new OpenFileDialog
            {
                //set the filter to text KML only
                Filter = "HEX files (*.HEX)|*.hex"
            };

            //was a file selected
            if (ofd.ShowDialog() == DialogResult.Cancel) return;
            label1.Text = ofd.FileName;
        }

        byte[] OTAUpdate = new byte[] { 0x4f, 0x54, 0x41, 0x55, 0x70, 0x64, 0x61, 0x74, 0x65 };

        private void button3_Click(object sender, EventArgs e)
        {
            SendUDPMessage(OTAUpdate, epModule);
        }

        int totallines = 0;
        private void button2_Click(object sender, EventArgs e)
        {
            if (File.Exists(label1.Text))
            {
                hexindex.Clear();
                for (int i = 0; i <= 255; i++)
                    hexindex.Add(i.ToString("X2"), (byte)i);
                hexindex.Add("::", 0x3a);
                totallines = 0;
                using (StreamReader reader = new StreamReader(label1.Text))
                {
                    try
                    {
                        string line;
                        //read all the lines
                        DateTime prev = DateTime.Now;
                        DateTime start = DateTime.Now;
                        TimeSpan aa = new TimeSpan(TimeSpan.TicksPerMillisecond * 2);
                        int idx = 0;
                        while (true)
                        {
                            if (DateTime.Now - prev > aa)
                            {
                                prev = DateTime.Now;
                                line = "";
                                for (int i = 0; i < 11; i++)
                                {
                                    if (!reader.EndOfStream)
                                    {
                                        line += ":" + reader.ReadLine();
                                        idx++;
                                    }
                                }
                                SendUDPMessage(StrToByteArray(line), epModule);

                                if (reader.EndOfStream)
                                    break;
                            }
                        }

                        Debug.WriteLine(DateTime.Now - start);
                        Debug.WriteLine("finished with " + idx + " lines");
                        totallines = idx;
                    }
                    catch (Exception er)
                    {
                    }
                }
            }
        }

        public byte[] StrToByteArray(string str)
        {
            List<byte> hexres = new List<byte>();
            for (int i = 0; i < str.Length; i += 2)
                hexres.Add(hexindex[str.Substring(i, 2)]);

            return hexres.ToArray();
        }

        private void button4_Click(object sender, EventArgs e)
        {
            SendUDPMessage(new byte[] { 0x3a, 0x00, 0x00, 0x00, 0x07, 0xF9 }, epModule);
        }
    }
}
