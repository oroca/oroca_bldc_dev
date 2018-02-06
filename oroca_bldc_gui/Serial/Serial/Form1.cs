using System;
using System.Drawing;
using System.Windows.Forms;
using System.IO.Ports;
using System.Threading;
using System.Text;
using System.Runtime.InteropServices;

using System.Data;
using System.Data.OleDb;

using MavLink;

namespace Serial
{
    public partial class Form1 : Form
    {
        // Mavlink message object
        Mavlink receivedMsg = new Mavlink();

        public Form1()
        {
            InitializeComponent();

            GetSerialPort();

            cmbBRate.SelectedIndex = 5;
            SerialPort.DataBits = 8;
            SerialPort.Parity = Parity.None;
            SerialPort.StopBits = StopBits.One;

            switch (cmbBRate.SelectedIndex)
            {
                case 0: SerialPort.BaudRate = (int)9600; break;
                case 1: SerialPort.BaudRate = (int)14400; break;
                case 2: SerialPort.BaudRate = (int)19200; break;
                case 3: SerialPort.BaudRate = (int)38400; break;
                case 4: SerialPort.BaudRate = (int)57600; break;
                case 5: SerialPort.BaudRate = (int)115200; break;
                default: SerialPort.BaudRate = (int)19200; break;
            }


            
            // Add my packet receive methods to the event handler
            receivedMsg.PacketReceived += new PacketReceivedEventHandler(this.PrintRecievedPackets);
        }

        protected override void WndProc(ref Message m)
        {
            UInt32 WM_DEVICECHANGE = 0x0219;
            UInt32 DBT_DEVTUP_VOLUME = 0x02;
            UInt32 DBT_DEVICEARRIVAL = 0x8000;
            UInt32 DBT_DEVICEREMOVECOMPLETE = 0x8004;

            if ((m.Msg == WM_DEVICECHANGE) && (m.WParam.ToInt32() == DBT_DEVICEARRIVAL))//디바이스 연결
            {
                //int m_Count = 0;
                int devType = Marshal.ReadInt32(m.LParam, 4);

                if (devType == DBT_DEVTUP_VOLUME)
                {
                    GetSerialPort();
                }
            }

            if ((m.Msg == WM_DEVICECHANGE) && (m.WParam.ToInt32() == DBT_DEVICEREMOVECOMPLETE))  //디바이스 연결 해제
            {
                int devType = Marshal.ReadInt32(m.LParam, 4);
                if (devType == DBT_DEVTUP_VOLUME)
                {
                    GetSerialPort();
                }
            }

            base.WndProc(ref m);
        }

        private void Form1_Load(object sender, EventArgs e)
        {
            CheckForIllegalCrossThreadCalls = false;
#if false
            OleDbConnection ExcelConn = new OleDbConnection(@"Provider=Microsoft.ACE.OLEDB.12.0;
                                                              Data Source='Sample.xlsx';
                                                              Extended Properties=Excel 8.0;
                                                              HDR=YES");

          //  ExcelConn.Open();
            OleDbDataAdapter ExcelCmd = new OleDbDataAdapter("select * from [Sheet1$]", ExcelConn);
            ExcelCmd.TableMappings.Add("Table", "Net-informations.com");

            DataSet DtSet = new DataSet();
            ExcelCmd.Fill(DtSet);
            dataGridView1.DataSource = DtSet.Tables[0];
            ExcelConn.Close();

#endif
        }

        private MavlinkPacket MsgAckHandler;

        public void PrintRecievedPackets(object sender, MavlinkPacket e)
        {
            // Print Message Basic Info
            Console.WriteLine("System ID: {0}", e.SystemId);
            Console.WriteLine("Message: {0}", e.Message.ToString ());
            Console.WriteLine ("Time Stamp: {0}", e.TimeStamp);

            if (e.Message.GetType() == typeof(MavLink.Msg_ack))
            {
                MsgAckHandler = e;
            }
            else if (e.Message.GetType() == typeof(MavLink.Msg_set_velocity))
            {
               // Console.WriteLine("Msg_set_velocity : {0}", (e.Message as MavLink.Msg_set_velocity).ref_angular_velocity);
                rbText.Text += e.TimeStamp; 
                rbText.Text += (e.Message as MavLink.Msg_set_velocity).ref_angular_velocity + "\r\n";
            }
            else if (e.Message.GetType() == typeof(MavLink.Msg_debug_string))
            {
                byte[] temp = (e.Message as MavLink.Msg_debug_string).dbg_str;
                // Console.WriteLine(Encoding.Default.GetString(temp));
                rbText.Text += DateTime.Now +" "+Encoding.Default.GetString(temp);
                rbText.Text += "\r\n";

                rbText.SelectionStart = rbText.TextLength;
                rbText.ScrollToCaret();
                
            }
        }

        private void SP_DataReceived(object sender, SerialDataReceivedEventArgs e)
        {
            if (SerialPort.IsOpen)
            {
                Thread.Sleep(1);

                int num = SerialPort.BytesToRead;
                byte[] receiveByteArray = new byte[num];
                SerialPort.Read(receiveByteArray, 0, num);

               // Console.WriteLine(BitConverter.ToString(receiveByteArray));
                
                receivedMsg.ParseBytes(receiveByteArray);
                
            }
        }

        private void btnOpen_Click(object sender, EventArgs e)
        {
            board_connect();

            if (SerialPort.IsOpen)
            {
                lbStatus.Text = "connected";
                btnOpen.Visible = false;
                btnPortClose.Visible = true;
            }
            else
            {
                lbStatus.Text = "fail";
                lbStatus.ForeColor = Color.Red;
            }
        }

        private void btnPortClose_Click(object sender, EventArgs e)
        {
            board_disconnect();

            lbStatus.Text = "disconnected";
            btnOpen.Visible = true;
            btnPortClose.Visible = false;
        }
        private void cmbPort_SelectedIndexChanged(object sender, EventArgs e)
        {
            SerialPort.PortName = cmbPort.SelectedItem.ToString();

        }

        private void cmbBRate_SelectedIndexChanged(object sender, EventArgs e)
        {
            switch (cmbBRate.SelectedIndex)
            {
                case 0 :                    SerialPort.BaudRate = (int)9600;                    break;
                case 1 :                    SerialPort.BaudRate = (int)14400;                    break;
                case 2 :                    SerialPort.BaudRate = (int)19200;                    break;
                case 3 :                    SerialPort.BaudRate = (int)38400;                    break;
                case 4 :                    SerialPort.BaudRate = (int)57600;                    break;
                case 5 :                    SerialPort.BaudRate = (int)115200;                    break;
                default:                    SerialPort.BaudRate = (int)19200;                    break;
            }
        }


        private void Form1_FormClosed(object sender, FormClosedEventArgs e)
        {
            if (SerialPort.IsOpen)
            {
                SerialPort.Close();
            }
        }

        private void getVersion_Click(object sender, EventArgs e)
        {
            cmd_read_version();
        }
       
        private void getBoardName_Click(object sender, EventArgs e)
        {
            cmd_read_BoardName();
        }
        private void numericUpDown1_ValueChanged(object sender, EventArgs e)
        {
            byte[] sendbuf = new byte[100];

            Msg_set_velocity msg_set_velocity = new Msg_set_velocity();
            msg_set_velocity.ref_angular_velocity = (UInt16)numericUpDown1.Value;

            MavlinkPacket mavlink_packet = new MavlinkPacket();
            mavlink_packet.ComponentId = 121;
            mavlink_packet.SystemId = 9;
            mavlink_packet.Message = msg_set_velocity;

            Mavlink mav_link = new Mavlink();
            sendbuf = mav_link.Send(mavlink_packet);

            // string result = System.Text.Encoding.UTF8.GetString(sendbuf);
            string result = BitConverter.ToString(sendbuf);
            rbText.Text += "\r\n" + "[" + SerialPort.PortName.ToString() + "] " + result;

            if (SerialPort.IsOpen)
            {
                SerialPort.Write(sendbuf, 0, sendbuf.Length);
            }
        }

        private void board_connect()
        {
            SerialPort.Open();

            cmd_read_BoardName();
            cmd_read_version();

        }
        private void board_disconnect()
        {
            SerialPort.Close();
        }

        static class Constants
        {
            public const int OK = 0x0000;

            public const int ERR_TIMEOUT = 0xF020;
            public const int ERR_MISMATCH_ID = 0xF021;
            public const int ERR_SIZE_OVER = 0xF022;
        }

        private bool cmd_read_version()
        {
            byte[] sendbuf = new byte[100];

            Msg_read_version msg_read_version = new Msg_read_version();
            msg_read_version.resp = (byte)1;
            msg_read_version.param = new byte[] { 0, 0, 0, 0, 0, 0, 0, 0 };

            MavlinkPacket mavlink_packet = new MavlinkPacket();
            mavlink_packet.ComponentId = 121;
            mavlink_packet.SystemId = 9;
            mavlink_packet.Message = msg_read_version;

            Mavlink mav_link = new Mavlink();
            sendbuf = mav_link.Send(mavlink_packet);

            if (SerialPort.IsOpen)
            {
                SerialPort.Write(sendbuf, 0, sendbuf.Length);

                DateTime start = DateTime.Now;
                int retrys = 3;

                while (true)
                {
                    if (!(start.AddMilliseconds(200) > DateTime.Now))
                    {
                        if (retrys > 0)
                        {
                            if (SerialPort.IsOpen)
                            {
                                SerialPort.Write(sendbuf, 0, sendbuf.Length);
                            }
                            start = DateTime.Now;
                            retrys--;
                            continue;
                        }
                        return false;
                    }

                    if (MsgAckHandler != null)
                    {
                        if (MsgAckHandler.Message.GetType() == typeof(MavLink.Msg_ack))
                        {
                            byte[] temp = (MsgAckHandler.Message as MavLink.Msg_ack).data;
                            //Console.WriteLine(Encoding.Default.GetString(temp));
                            rbText.Text += "FW version :  " + BitConverter.ToString(temp);
                            rbText.Text += "\r\n";
                            MsgAckHandler = null;
                            return true;
                        }
                    }
                }

            }
            else 
            {
                return false;
            }
        }
        private bool cmd_read_BoardName()
        {
            byte[] sendbuf = new byte[100];

            Msg_read_board_name msg_read_board_name = new Msg_read_board_name();
            msg_read_board_name.resp = (byte)1;
            msg_read_board_name.param = new byte[] { 0, 0, 0, 0, 0, 0, 0, 0 };

            MavlinkPacket mavlink_packet = new MavlinkPacket();
            mavlink_packet.ComponentId = 121;
            mavlink_packet.SystemId = 9;
            mavlink_packet.Message = msg_read_board_name;

            Mavlink mav_link = new Mavlink();
            sendbuf = mav_link.Send(mavlink_packet);

            if (SerialPort.IsOpen)
            {
                SerialPort.Write(sendbuf, 0, sendbuf.Length);

                DateTime start = DateTime.Now;
                int retrys = 3;

                while (true)
                {
                    if (!(start.AddMilliseconds(200) > DateTime.Now))
                    {
                        if (retrys > 0)
                        {
                            if (SerialPort.IsOpen)
                            {
                                SerialPort.Write(sendbuf, 0, sendbuf.Length);
                            }
                            start = DateTime.Now;
                            retrys--;
                            continue;
                        }
                        return false;
                    }
                    if (MsgAckHandler != null)
                    {
                        if (MsgAckHandler.Message.GetType() == typeof(MavLink.Msg_ack))
                        {
                            byte[] temp = (MsgAckHandler.Message as MavLink.Msg_ack).data;
                           // Console.WriteLine(Encoding.Default.GetString(temp));
                            rbText.Text += "Board name :  " + Encoding.Default.GetString(temp);
                            rbText.Text += "\r\n";
                            MsgAckHandler = null;
                            return true;
                        }
                    }
                }

            }
            else
            {
                return false;
            }
        }

        private bool mavlink_packet_gen(MavlinkPacket packet)
        {
            MavlinkPacket mavlink_packet = new MavlinkPacket();
            mavlink_packet.ComponentId = 121;
            mavlink_packet.SystemId = 9;
            mavlink_packet.Message = packet.Message;

            Mavlink mav_link = new Mavlink();
            byte[] sendbuf = mav_link.Send(mavlink_packet);

            return true;
        }

        private bool mavlink_packet_send(byte[] sendbuf)
        {
            if (SerialPort.IsOpen)
            {
                SerialPort.Write(sendbuf, 0, sendbuf.Length);

                DateTime start = DateTime.Now;
                int retrys = 3;

                while (true)
                {
                    if (!(start.AddMilliseconds(200) > DateTime.Now))
                    {
                        if (retrys > 0)
                        {
                            if (SerialPort.IsOpen)
                            {
                                SerialPort.Write(sendbuf, 0, sendbuf.Length);
                            }
                            start = DateTime.Now;
                            retrys--;
                            continue;
                        }
                        return false;
                    }
                    if (MsgAckHandler != null)
                    {
                        if (MsgAckHandler.Message.GetType() == typeof(MavLink.Msg_ack))
                        {
                            byte[] temp = (MsgAckHandler.Message as MavLink.Msg_ack).data;
                            // Console.WriteLine(Encoding.Default.GetString(temp));
                            rbText.Text += "Board name :  " + Encoding.Default.GetString(temp);
                            rbText.Text += "\r\n";
                            MsgAckHandler = null;
                            return true;
                        }
                    }
                }

            }
            else
            {
                return false;
            }
        }

        private void GetSerialPort()
        {
           // cmbPort.BeginUpdate();
           // foreach (string comport in SerialPort.GetPortNames())
           // {
           //     cmbPort.Items.Add(comport);
           // }
           // cmbPort.EndUpdate();

           // cmbPort.SelectedIndex = 0;

            //Port
            cmbPort.Items.Clear();
            try
            {
                foreach (string str in SerialPort.GetPortNames())
                {
                    cmbPort.Items.Add(str);
                }
                if (cmbPort.Items.Count <= 0)
                {
                    cmbPort.Items.Add("연결 장치 없음");
                }
            }
            catch (Exception ex)
            {
                MessageBox.Show(ex.Message);
            }

            cmbPort.SelectedIndex = 0;
        }
    }
}
