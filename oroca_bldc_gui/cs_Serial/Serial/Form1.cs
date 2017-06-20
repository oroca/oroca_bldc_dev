using System;
using System.Drawing;
using System.Windows.Forms;
using System.IO.Ports;
using System.Threading;

namespace Serial
{
    public partial class Form1 : Form
    {

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
            SP.PortName = "COM1";
            SP.BaudRate = (int)19200;
            SP.DataBits = (int)8;
            SP.Parity = Parity.None;
            SP.StopBits = StopBits.One;
            //SP.ReadTimeout = (int)100;
            //SP.WriteTimeout = (int)100;
        }

        private void SP_DataReceived(object sender, SerialDataReceivedEventArgs e)
        {
            if (SP.IsOpen)
            {
                string str = SP.ReadLine();

                str = str.Trim().Replace("\r\n", "");
                lbResult.Text = str;
                rbText.Text = string.Format("{0}{1}{2}", rbText.Text, "[Received]", str+"\r\n");
                rbText.SelectionStart = rbText.Text.Length;
                rbText.ScrollToCaret();
                //rbText.Text += "[전송된 Data] " + str;
                Thread.Sleep(1000);
            }
        }

        private void btnOpen_Click(object sender, EventArgs e)
        {
                SP.Open();
                if (SP.IsOpen)
                {
                    //rbText.Text = string.Format("{0}{1}", rbText.Text, "\r\n[Succed] Port Open!!");
                    rbText.Text = "["+SP.PortName.ToString() +"] Port Open Connect!!";
                    lbStatus.Text = "Connect!!";
                    btnOpen.Visible = false;
                    btnPortClose.Visible = true;
                }
                else
                {
                    //rbText.Text = string.Format("{0}{1}", rbText.Text, "\r\n[Fail] Port Open!!");
                    rbText.Text = "[" + SP.PortName.ToString() + "] Port Open Failed!";
                    lbStatus.Text = "[Fail] Port Open!";
                    lbStatus.ForeColor = Color.Red;
                }
        }

        private void cmbPort_SelectedIndexChanged(object sender, EventArgs e)
        {
            SP.PortName = cmbPort.SelectedItem.ToString();
        }

        private void cmbBRate_SelectedIndexChanged(object sender, EventArgs e)
        {
            switch (cmbBRate.SelectedIndex)
            {
                case 0 :
                    SP.BaudRate = (int)9600;
                    break;
                case 1:
                    SP.BaudRate = (int)14400;
                    break;
                case 2:
                    SP.BaudRate = (int)19200;
                    break;
                case 3:
                    SP.BaudRate = (int)38400;
                    break;
                case 4:
                    SP.BaudRate = (int)57600;
                    break;
                case 5:
                    SP.BaudRate = (int)115200;
                    break;
                default:
                    SP.BaudRate = (int)19200;
                    break;
            }
        }

        private void cmbDataBits_SelectedIndexChanged(object sender, EventArgs e)
        {
            switch (cmbDataBits.SelectedIndex)
            {
                case 0:
                    SP.DataBits = 8;
                    break;
                case 1:
                    SP.DataBits = 7;
                    break;
                default :
                    SP.DataBits = 8;
                    break;
            }
        }

        private void cmbParity_SelectedIndexChanged(object sender, EventArgs e)
        {
            switch (cmbParity.SelectedIndex)
            {
                case 0:
                    SP.Parity = Parity.Even;
                    break;
                case 1:
                    SP.Parity = Parity.Mark;
                    break;
                case 2:
                    SP.Parity = Parity.None;
                    break;
                case 3:
                    SP.Parity = Parity.Odd;
                    break;
                case 4:
                    SP.Parity = Parity.Space;
                    break;
                default:
                    SP.Parity = Parity.None;
                    break;
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
                case 1:
                    SP.StopBits = StopBits.One;
                    break;
                case 2:
                    SP.StopBits = StopBits.OnePointFive;
                    break;
                case 3:
                    SP.StopBits = StopBits.Two;
                    break;
                default:
                    SP.StopBits = StopBits.One;
                    break;
            }
        }

        private void Form1_Load(object sender, EventArgs e)
        {
            CheckForIllegalCrossThreadCalls = false;
        }

        private void btnPortClose_Click(object sender, EventArgs e)
        {
            SP.Close();
            rbText.Text += "\r\n" + "[" + SP.PortName.ToString() + "] Port Close!!";
            lbStatus.Text = "Not Connect!!";
            btnOpen.Visible = true;
            btnPortClose.Visible = false;
        }

        private void Form1_FormClosed(object sender, FormClosedEventArgs e)
        {
            if (SP.IsOpen)
            {
                SP.Close();
            }
        }
    }
}
