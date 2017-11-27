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

            //cmbPort.SelectedIndex = 0;
            cmbBRate.SelectedIndex = 5;
            cmbDataBits.SelectedIndex = 0;
            cmbParity.SelectedIndex = 2;
            cmbStopBits.SelectedIndex = 2;

            //SerialPort.PortName = cmbPort.SelectedItem.ToString();

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

        public void PrintRecievedPackets(object sender, MavlinkPacket e)
        {
            // Print Message Basic Info
                        Console.WriteLine("System ID: {0}", e.SystemId);
                        Console.WriteLine("Message: {0}", e.Message.ToString ());
                        Console.WriteLine ("Time Stamp: {0}", e.TimeStamp);

            rbText.Text += ".";

            if (e.Message.GetType() == typeof(MavLink.Msg_debug_string))
            {
                Console.WriteLine((e.Message as MavLink.Msg_debug_string).dbg_str);
                rbText.Text += "[수신된 Data] " + (e.Message as MavLink.Msg_debug_string).dbg_str + "\r\n";
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
               // string str = SerialPort.ReadLine();

               // string str = SerialPort.ReadExisting();
                // byte[] receiveByteArray = Encoding.UTF8.GetBytes(str);


                byte[] receiveByteArray = new byte[1024];
                SerialPort.Read(receiveByteArray, 0, SerialPort.BytesToRead);
                receivedMsg.ParseBytes(receiveByteArray);

                //str = str.Trim().Replace("\r\n", "");
                //lbResult.Text = str;
                //rbText.Text = string.Format("{0}{1}{2}", rbText.Text, "[Received]", str+"\r\n");
                //rbText.SelectionStart = rbText.Text.Length;
                //rbText.ScrollToCaret();
                //rbText.Text += "[전송된 Data] " + str;
                Thread.Sleep(1);
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

        private void button1_Click(object sender, EventArgs e)
        {
            byte[] sendbuf = new byte[100];

            Msg_set_velocity msg_set_velocity = new Msg_set_velocity();
            msg_set_velocity.ref_angular_velocity = 1500;

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


    }
}
