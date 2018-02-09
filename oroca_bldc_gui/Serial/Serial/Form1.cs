using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Windows.Forms;

using System.Linq;
using System.Drawing;
using System.Text;

using System.Threading;
using System.Runtime.InteropServices;

using System.IO;
using System.IO.Ports;

using System.Data;
using System.Data.OleDb;

using MavLink;

namespace Serial
{
    public partial class Form1 : Form
    {
        //====================
        // usb auto detection
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
        
        #region form_func

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
        private void Form1_Load(object sender, EventArgs e)
        {
            CheckForIllegalCrossThreadCalls = false;
            //openFileDialog1.ShowDialog();
            DataView_Load();
        }
        private void Form1_FormClosed(object sender, FormClosedEventArgs e)
        {
            if (SerialPort.IsOpen)
            {
                SerialPort.Close();
            }
        }


      //  private void DataView_Load(object sender, EventArgs e)
        private void DataView_Load()
        {
            DataGridView dataGridView = dataGridView1;
            DataTable dataTable = new DataTable();

            this.Size = new Size(750, 450);
            dataGridView.Size = new Size(600, 400);
            dataGridView.Location = new Point(5, 5);
 
            //int month = DateTime.Now.Month;
            //int day = DateTime.Now.Day;
            //string filePath = @"E:\ResultData\Model\" + month.ToString() + @"\" + day.ToString() + @"\Data.csv"; // 현재 날짜에 맞는 디렉토리 경로

            string filePath = "Sample.csv";
            string[] raw_text = File.ReadAllLines(filePath); // Data.csv 파일의 모든 라인을 읽는다.(배열 하나당 한 줄씩 들어간다.)
            string[] data_col = null;
            int x = 0;
            foreach (string text_line in raw_text)
            {
                data_col = text_line.Split(',');
                if (x == 0)
                {
                    for (int i = 0; i <= data_col.Count() - 1; i++)
                    {
                        dataTable.Columns.Add(data_col[i]); // data_col[i]의 데이터 크기 와 data_col 배열 크기 를 고려하여 table의 항목을 만든다.
                    }
                    x++;
                }
                else
                {
                    dataTable.Rows.Add(data_col); // 그안에 data_col의 값을 입력한다.
                }
            }
 
            dataGridView.DataSource = dataTable; // 테이블을 그리드 뷰에 올린다.
            //this.Controls.Add(dataGridView); // 그림으로 표시
        }
        public void saveData() // 호출 할 때 마다 날짜에 맞는 폴더 안에 csv 파일 생성 이미 생성 된경우 이어쓰기
        {
            int month, day;
 
            month = DateTime.Now.Month;
            day = DateTime.Now.Day;
            
            string filePath = @"E:\ResultData\Model\" + month.ToString() + @"\" + day.ToString() + @"\Data.csv"; // 현재 날짜에 맞        는 디렉토리 경로
            FileInfo fileInfo = new FileInfo(filePath);
            if (fileInfo.Exists) // 파일이 존재 한다면
            {
                using (FileStream streamWriter = new FileStream(filePath, FileMode.Append, FileAccess.Write)) // 파일 이어쓰기
                {
                    using (StreamWriter sWriter = new StreamWriter(streamWriter))
                    {
                        sWriter.WriteLine(DateTime.Now.ToLongTimeString()); // 덧 붙일 내용
                    }
                }
            }
            else
            {
                StreamWriter streamWriter = new StreamWriter(filePath, false, Encoding.Unicode); // 파일 생성
            }
        }


        private string Excel03ConString = "Provider=Microsoft.Jet.OLEDB.4.0;Data Source={0};Extended Properties='Excel 8.0;HDR={1}'";
        private string Excel07ConString = "Provider=Microsoft.ACE.OLEDB.12.0;Data Source={0};Extended Properties='Excel 8.0;HDR={1}'";

        private void openFileDialog1_FileOk(object sender, CancelEventArgs e)
        {
            string filePath = openFileDialog1.FileName;
            string fileExtension = Path.GetExtension(filePath);
            //string header = rbHeaderYes.Checked ? "Yes" : "No";
            string header = "No";
            string connectionString = string.Empty;
            string sheetName = string.Empty;
 
            // 확장자로 구분하여 커넥션 스트링을 가져옮
            switch(fileExtension)
            {
                case ".xls":    //Excel 97-03
                    connectionString = string.Format(Excel03ConString, filePath, header);
                    break;
                case ".xlsx":  //Excel 07
                    connectionString = string.Format(Excel07ConString, filePath, header);
                    break;
            }
 
            // 첫 번째 시트의 이름을 가져옮
            using (OleDbConnection con = new OleDbConnection(connectionString))
            {
                using (OleDbCommand cmd = new OleDbCommand())
                {
                    cmd.Connection = con;
                    con.Open();
                    DataTable dtExcelSchema = con.GetOleDbSchemaTable(OleDbSchemaGuid.Tables, null);
                    sheetName = dtExcelSchema.Rows[0]["TABLE_NAME"].ToString();
                    con.Close();
                }
            }
            Console.WriteLine("sheetName = " + sheetName);
 
            // 첫 번째 쉬트의 데이타를 읽어서 datagridview 에 보이게 함.
            using (OleDbConnection con = new OleDbConnection(connectionString))
            {
                using (OleDbCommand cmd = new OleDbCommand())
                {
                    using (OleDbDataAdapter oda = new OleDbDataAdapter())
                    {
                        DataTable dt = new DataTable();
                        cmd.CommandText = "SELECT * From [" + sheetName + "]";
                        cmd.Connection = con;
                        con.Open();
                        oda.SelectCommand = cmd;
                        oda.Fill(dt);
                        con.Close();
 
                        //Populate DataGridView.
                        //dataGridView1.DataSource = dt;
                    }
                }
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
        #endregion

        #region Mavlink_func

        // Mavlink message object
        Mavlink receivedMsg = new Mavlink();

        static class Constants
        {
            public const int OK = 0x0000;

            public const int ERR_TIMEOUT = 0xF020;
            public const int ERR_MISMATCH_ID = 0xF021;
            public const int ERR_SIZE_OVER = 0xF022;
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

        private MavlinkPacket MsgAckHandler;

        public void PrintRecievedPackets(object sender, MavlinkPacket e)
        {
            // Print Message Basic Info
            Console.WriteLine("System ID: {0}", e.SystemId);
            Console.WriteLine("Message: {0}", e.Message.ToString());
            Console.WriteLine("Time Stamp: {0}", e.TimeStamp);

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
                rbText.Text += DateTime.Now + " " + Encoding.Default.GetString(temp);
                rbText.Text += "\r\n";

                rbText.SelectionStart = rbText.TextLength;
                rbText.ScrollToCaret();

            }
        }

        #endregion

        #region serial_func
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
        #endregion

       
    }
}
