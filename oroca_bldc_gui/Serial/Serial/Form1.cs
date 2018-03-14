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
        #region formBasefunction

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

            dataGridView1.DataSource = DataView_import("Sample.csv");
        }
        private void Form1_FormClosed(object sender, FormClosedEventArgs e)
        {
            if (SerialPort.IsOpen)
            {
                SerialPort.Close();
            }
        }

        private DataTable DataView_import(string filePath)
        {
            string[] ReadDataBuffer = new string[1000];

            int DataNum = 0;
            int i = 0;

            //string filePath = "Sample.csv";

            StreamReader sr = new StreamReader(filePath, Encoding.Default);
            ReadDataBuffer[0] = sr.ReadLine();//read header

            while (ReadDataBuffer[i] != null)
            {
                i = i + 1;
                ReadDataBuffer[i] = sr.ReadLine();
            }
            DataNum = i;
            sr.Close();

            DataRow row = null;
            DataTable dataTable = new DataTable();
            for (i = 0; i < DataNum; i++)
            {
                string[] split = ReadDataBuffer[i].Split(new Char[] { ',', '\t' });

                foreach (string split_data in split)
                {
                    if (i == 0)
                    {
                        dataTable.Columns.Add();
                    }
                }
                row = dataTable.NewRow();
                row.ItemArray = split;
                dataTable.Rows.Add(row);
            }
            return dataTable;
        }
        private void DataView_export(DataTable dt, string filePath) // 호출 할 때 마다 날짜에 맞는 폴더 안에 csv 파일 생성 이미 생성 된경우 이어쓰기
        {
            // Create the CSV file to which grid data will be exported.
            //string filePath = "Sample.csv";

            StreamWriter sw = new StreamWriter(filePath);
            // First we will write the headers.
            //DataTable dt = ((DataSet)grid1.DataSource).Tables[0];

            int iColCount = dt.Columns.Count;
        //    for (int i = 0; i < iColCount; i++)
        //    {
        //        sw.Write(dt.Columns[i]);
        //        if (i < iColCount - 1)
        //        {
        //            sw.Write(",");
        //        }
        //    }
            //sw.Write(sw.NewLine);
            // Now write all the rows.
            foreach (DataRow dr in dt.Rows)
            {
                for (int i = 0; i < iColCount; i++)
                {
                    if (!Convert.IsDBNull(dr[i]))
                    {
                        sw.Write(dr[i].ToString());
                    }

                    if (i < iColCount - 1)
                    {
                        sw.Write(System.Globalization.CultureInfo.CurrentCulture.TextInfo.ListSeparator);
                    }
                }
                sw.Write(sw.NewLine);
            }
            sw.Close();
        }

        private void setControlValueFromDataview()
        {
            Int32 val = Convert.ToInt32(dataGridView1[1, 0].Value);
            if (val == 0)//openloop:1 closedloop:0
                rdBtn_ClosedLoop.Checked = true;
            else
                rdBtn_OpenLoop.Checked = true;
            ControlModeTextBoxStatusUpdate(rdBtn_OpenLoop.Checked);

            textBox_openDValue.Text = Convert.ToString(dataGridView1[3, 1].Value);
            textBox_openQValue.Text = Convert.ToString(dataGridView1[4, 1].Value);
            textBox_DeltaAngle.Text = Convert.ToString(dataGridView1[5, 1].Value);

            cmbEncoderMode.SelectedIndex = Convert.ToInt32(dataGridView1[1, 1].Value);
        }
        private void dataGridView1_BindingCompleteEvent(object sender, DataGridViewBindingCompleteEventArgs e)
        {
            //MessageBox.Show("BindingCompleteEvent");
            setControlValueFromDataview();
        }

        #endregion

//엑셀시트와 연동하면서 테스트 했던 코드 
#if false
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
#endif

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

#if false
        private byte[] mavlink_packet_gen(int systemID, int ComponentId, ref packet)
        {
            MavlinkPacket mavlink_packet = new MavlinkPacket();
            mavlink_packet.ComponentId = 121;
            mavlink_packet.SystemId = 9;
            mavlink_packet.Message = packet;

            Mavlink mav_link = new Mavlink();
            byte[] sendbuf = mav_link.Send(mavlink_packet);

            return sendbuf;
        }
#endif
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
        private void cmd_set_openloop()
        {
            byte[] sendbuf = new byte[100];

            Msg_set_openloop msg_set_openloop = new Msg_set_openloop();
            msg_set_openloop.openLoopMode = (Byte)numericUpDown1.Value;

            //sendbuf = mavlink_packet_gen(9, 121,(MavlinkPacket)msg_set_openloop);
            MavlinkPacket mavlink_packet = new MavlinkPacket();
            mavlink_packet.ComponentId = 121;
            mavlink_packet.SystemId = 9;
            mavlink_packet.Message = msg_set_openloop;

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

        #region serial_func
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
        private void cmbPort_SelectedIndexChanged(object sender, EventArgs e)
        {
            SerialPort.PortName = cmbPort.SelectedItem.ToString();

        }
        private void cmbBRate_SelectedIndexChanged(object sender, EventArgs e)
        {
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

        #endregion

        #region GUI_event
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

        private void ControlModeTextBoxStatusUpdate(bool en)
        {
            textBox_openDValue.Enabled = en;
            textBox_openQValue.Enabled = en;
            textBox_DeltaAngle.Enabled = en;
        }

        private void rdBtn_OpenLoop_CheckedChanged(object sender, EventArgs e)
        {
            if (rdBtn_OpenLoop.Checked == true)
            {
                dataGridView1[1, 0].Value = "1";

            }
            else
            {
                dataGridView1[1, 0].Value = "0";
            }

            ControlModeTextBoxStatusUpdate(rdBtn_OpenLoop.Checked);
        }
        private void cmbEncoderMode_SelectedIndexChanged(object sender, EventArgs e)
        {
            dataGridView1[1, 1].Value = cmbEncoderMode.SelectedIndex;
        }
        private void btn_CSVExport_Click(object sender, EventArgs e)
        {
            SaveFileDialog saveFileDialog1 = new SaveFileDialog();

            //saveFileDialog1.InitialDirectory = @"C:\";
            saveFileDialog1.Title = "Save CSV Files";
            saveFileDialog1.CheckFileExists = true;
            saveFileDialog1.CheckPathExists = true;
            saveFileDialog1.DefaultExt = "txt";
            saveFileDialog1.Filter = "csv files (*.csv)|*.csv|All files (*.*)|*.*";
            saveFileDialog1.FilterIndex = 2;
            saveFileDialog1.RestoreDirectory = true;

            if (saveFileDialog1.ShowDialog() == DialogResult.OK)
            {
                DataView_export((DataTable)dataGridView1.DataSource, saveFileDialog1.FileName);
            }

        }
        private void btn_CSVImport_Click(object sender, EventArgs e)
        {
            OpenFileDialog openFileDialog1 = new OpenFileDialog();
            //openFileDialog1.InitialDirectory = @"C:\";
            openFileDialog1.Title = "Browse CSV Files";
            openFileDialog1.CheckFileExists = true;
            openFileDialog1.CheckPathExists = true;
            openFileDialog1.DefaultExt = "txt";
            openFileDialog1.Filter = "csv files (*.csv)|*.csv|All files (*.*)|*.*";
            openFileDialog1.FilterIndex = 2;
            openFileDialog1.RestoreDirectory = true;
            openFileDialog1.ReadOnlyChecked = true;
            openFileDialog1.ShowReadOnly = true;

            if (openFileDialog1.ShowDialog() == DialogResult.OK)
            {
                dataGridView1.DataSource = DataView_import(openFileDialog1.FileName);
            }


        }
        #endregion

    }
}
