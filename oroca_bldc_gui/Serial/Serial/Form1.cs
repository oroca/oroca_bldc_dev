using System;
using System.Drawing;
using System.Windows.Forms;
using System.IO.Ports;
using System.Threading;
using System.Text;

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

            //텍스스 박스 초기화;
            //rbText.ScrollBars = RichTextBoxScrollBars.Vertical;

            //Port
            cmbPort.BeginUpdate();
            foreach (string comport in SerialPort.GetPortNames())
            {
                cmbPort.Items.Add(comport);
            }
            cmbPort.EndUpdate();

            //SerialPort 초기 설정
            //SerialPort.PortName = "COM4";
            //SerialPort.BaudRate = (int)19200;
            //SerialPort.DataBits = (int)8;
            //SerialPort.Parity = Parity.None;
            //SerialPort.StopBits = StopBits.One;
            //SP.ReadTimeout = (int)100;
            //SP.WriteTimeout = (int)100;

            cmbPort.SelectedIndex = 0;
            cmbBRate.SelectedIndex = 5;
            cmbDataBits.SelectedIndex = 0;
            cmbParity.SelectedIndex = 2;
            cmbStopBits.SelectedIndex = 2;

           // SerialPort.PortName = cmbPort.SelectedItem.ToString();

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

            switch (cmbDataBits.SelectedIndex)
            {
                case 0: SerialPort.DataBits = 8; break;
                case 1: SerialPort.DataBits = 7; break;
                default: SerialPort.DataBits = 8; break;
            }

            switch (cmbParity.SelectedIndex)
            {
                case 0: SerialPort.Parity = Parity.Even; break;
                case 1: SerialPort.Parity = Parity.Mark; break;
                case 2: SerialPort.Parity = Parity.None; break;
                case 3: SerialPort.Parity = Parity.Odd; break;
                case 4: SerialPort.Parity = Parity.Space; break;
                default: SerialPort.Parity = Parity.None; break;
            }
            switch (cmbStopBits.SelectedIndex)
            {
                case 0:
                    //SP.StopBits = StopBits.None;
                    MessageBox.Show("이 값은 지원되지 않습니다");
                    break;
                case 1: SerialPort.StopBits = StopBits.One; break;
                case 2: SerialPort.StopBits = StopBits.OnePointFive; break;
                case 3: SerialPort.StopBits = StopBits.Two; break;
                default: SerialPort.StopBits = StopBits.One; break;
            }

            
            // Add my packet receive methods to the event handler
            receivedMsg.PacketReceived += new PacketReceivedEventHandler(this.PrintRecievedPackets);
        }

        private MavlinkPacket MsgHandler;

        public void PrintRecievedPackets(object sender, MavlinkPacket e)
        {
            // Print Message Basic Info
            Console.WriteLine("System ID: {0}", e.SystemId);
            Console.WriteLine("Message: {0}", e.Message.ToString ());
            Console.WriteLine ("Time Stamp: {0}", e.TimeStamp);

            if (e.Message.GetType() == typeof(MavLink.Msg_ack))
            {
                MsgHandler = e;
                //byte[] temp = (e.Message as MavLink.Msg_ack).data;
                //Console.WriteLine(Encoding.Default.GetString(temp));
                //rbText.Text += "[수신된 Data] " + Encoding.Default.GetString(temp) + "\r\n";
            }
            else if (e.Message.GetType() == typeof(MavLink.Msg_set_velocity))
            {
                Console.WriteLine("Msg_set_velocity : {0}", (e.Message as MavLink.Msg_set_velocity).ref_angular_velocity);
                rbText.Text += "[수신된 Data] " + (e.Message as MavLink.Msg_set_velocity).ref_angular_velocity + "\r\n";
            }
        }

        private void SP_DataReceived(object sender, SerialDataReceivedEventArgs e)
        {
            if (SerialPort.IsOpen)
            {
                Thread.Sleep(1);
                //string str = SerialPort.ReadLine();

               //string str = SerialPort.ReadExisting();
               // byte[] receiveByteArray = Encoding.UTF8.GetBytes(str);
                int num = SerialPort.BytesToRead;
                byte[] receiveByteArray = new byte[num];
                SerialPort.Read(receiveByteArray, 0, num);

                Console.WriteLine(BitConverter.ToString(receiveByteArray));
                
                receivedMsg.ParseBytes(receiveByteArray);


                //str = str.Trim().Replace("\r\n", "");
                //lbResult.Text = str;
                //rbText.Text = string.Format("{0}{1}{2}", rbText.Text, "[Received]", str+"\r\n");
                //rbText.SelectionStart = rbText.Text.Length;
                //rbText.ScrollToCaret();
                //rbText.Text += "[전송된 Data] " + str;
                
            }
        }

        private void btnOpen_Click(object sender, EventArgs e)
        {
                SerialPort.Open();
                if (SerialPort.IsOpen)
                {
                    //rbText.Text = string.Format("{0}{1}", rbText.Text, "\r\n[Succed] Port Open!!");
                    rbText.Text = "["+SerialPort.PortName.ToString() +"] Port Open Connect!!";
                    lbStatus.Text = "Connect!!";
                    btnOpen.Visible = false;
                    btnPortClose.Visible = true;
                }
                else
                {
                    //rbText.Text = string.Format("{0}{1}", rbText.Text, "\r\n[Fail] Port Open!!");
                    rbText.Text = "[" + SerialPort.PortName.ToString() + "] Port Open Failed!";
                    lbStatus.Text = "[Fail] Port Open!";
                    lbStatus.ForeColor = Color.Red;
                }
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

        private void cmbDataBits_SelectedIndexChanged(object sender, EventArgs e)
        {
            switch (cmbDataBits.SelectedIndex)
            {
                case 0:                    SerialPort.DataBits = 8;                    break;
                case 1:                    SerialPort.DataBits = 7;                    break;
                default :                    SerialPort.DataBits = 8;                    break;
            }
        }

        private void cmbParity_SelectedIndexChanged(object sender, EventArgs e)
        {
            switch (cmbParity.SelectedIndex)
            {
                case 0:                    SerialPort.Parity = Parity.Even;                    break;
                case 1:                    SerialPort.Parity = Parity.Mark;                    break;
                case 2:                    SerialPort.Parity = Parity.None;                    break;
                case 3:                    SerialPort.Parity = Parity.Odd;                    break;
                case 4:                    SerialPort.Parity = Parity.Space;                    break;
                default:                    SerialPort.Parity = Parity.None;                    break;
            }
        }

        private void cmbStopBits_SelectedIndexChanged(object sender, EventArgs e)
        {
            switch (cmbStopBits.SelectedIndex)
            {
                case 0:
                    //SP.StopBits = StopBits.None;
                    MessageBox.Show("이 값은 지원되지 않습니다");
                    break;
                case 1:                    SerialPort.StopBits = StopBits.One;                    break;
                case 2:                    SerialPort.StopBits = StopBits.OnePointFive;                    break;
                case 3:                    SerialPort.StopBits = StopBits.Two;                    break;
                default:                   SerialPort.StopBits = StopBits.One;                    break;
            }
        }

        private void Form1_Load(object sender, EventArgs e)
        {
            CheckForIllegalCrossThreadCalls = false;
        }

        private void btnPortClose_Click(object sender, EventArgs e)
        {
            SerialPort.Close();
            rbText.Text += "\r\n" + "[" + SerialPort.PortName.ToString() + "] Port Close!!";
            lbStatus.Text = "Not Connect!!";
            btnOpen.Visible = true;
            btnPortClose.Visible = false;
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

                    if (MsgHandler != null)
                    {
                        if (MsgHandler.Message.GetType() == typeof(MavLink.Msg_ack))
                        {
                            byte[] temp = (MsgHandler.Message as MavLink.Msg_ack).data;
                            //Console.WriteLine(Encoding.Default.GetString(temp));
                            rbText.Text += "FW version :  " + BitConverter.ToString(temp);
                            rbText.Text += "\r\n";
                            MsgHandler = null;
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
                    if (MsgHandler != null)
                    {
                        if (MsgHandler.Message.GetType() == typeof(MavLink.Msg_ack))
                        {
                            byte[] temp = (MsgHandler.Message as MavLink.Msg_ack).data;
                           // Console.WriteLine(Encoding.Default.GetString(temp));
                            rbText.Text += "Board name :  " + Encoding.Default.GetString(temp);
                            rbText.Text += "\r\n";
                            MsgHandler = null;
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
                    if (MsgHandler != null)
                    {
                        if (MsgHandler.Message.GetType() == typeof(MavLink.Msg_ack))
                        {
                            byte[] temp = (MsgHandler.Message as MavLink.Msg_ack).data;
                            // Console.WriteLine(Encoding.Default.GetString(temp));
                            rbText.Text += "Board name :  " + Encoding.Default.GetString(temp);
                            rbText.Text += "\r\n";
                            MsgHandler = null;
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
    }
}
