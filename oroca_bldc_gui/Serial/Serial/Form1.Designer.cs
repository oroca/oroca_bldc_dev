namespace Serial
{
    partial class Form1
    {
        /// <summary>
        /// 필수 디자이너 변수입니다.
        /// </summary>
        private System.ComponentModel.IContainer components = null;

        /// <summary>
        /// 사용 중인 모든 리소스를 정리합니다.
        /// </summary>
        /// <param name="disposing">관리되는 리소스를 삭제해야 하면 true이고, 그렇지 않으면 false입니다.</param>
        protected override void Dispose(bool disposing)
        {
            if (disposing && (components != null))
            {
                components.Dispose();
            }
            base.Dispose(disposing);
        }

        #region Windows Form 디자이너에서 생성한 코드

        /// <summary>
        /// 디자이너 지원에 필요한 메서드입니다.
        /// 이 메서드의 내용을 코드 편집기로 수정하지 마십시오.
        /// </summary>
        private void InitializeComponent()
        {
            this.components = new System.ComponentModel.Container();
            System.Windows.Forms.DataGridViewCellStyle dataGridViewCellStyle3 = new System.Windows.Forms.DataGridViewCellStyle();
            System.Windows.Forms.DataGridViewCellStyle dataGridViewCellStyle4 = new System.Windows.Forms.DataGridViewCellStyle();
            this.rbText = new System.Windows.Forms.RichTextBox();
            this.btnPortClose = new System.Windows.Forms.Button();
            this.lbStatus = new System.Windows.Forms.Label();
            this.label2 = new System.Windows.Forms.Label();
            this.label1 = new System.Windows.Forms.Label();
            this.btnOpen = new System.Windows.Forms.Button();
            this.cmbBRate = new System.Windows.Forms.ComboBox();
            this.cmbPort = new System.Windows.Forms.ComboBox();
            this.SerialPort = new System.IO.Ports.SerialPort(this.components);
            this.numericUpDown1 = new System.Windows.Forms.NumericUpDown();
            this.tabControl1 = new System.Windows.Forms.TabControl();
            this.tabPage1 = new System.Windows.Forms.TabPage();
            this.groupBox_EncoderMode = new System.Windows.Forms.GroupBox();
            this.label3 = new System.Windows.Forms.Label();
            this.cmbEncoderMode = new System.Windows.Forms.ComboBox();
            this.groupBox_ControlMode = new System.Windows.Forms.GroupBox();
            this.rdBtn_OpenLoop = new System.Windows.Forms.RadioButton();
            this.rdBtn_ClosedLoop = new System.Windows.Forms.RadioButton();
            this.tabPage2 = new System.Windows.Forms.TabPage();
            this.btn_CSVImport = new System.Windows.Forms.Button();
            this.btn_CSVExport = new System.Windows.Forms.Button();
            this.dataGridView1 = new System.Windows.Forms.DataGridView();
            this.tabPage3 = new System.Windows.Forms.TabPage();
            this.groupBox1 = new System.Windows.Forms.GroupBox();
            this.btn_EEPROMWrite = new System.Windows.Forms.Button();
            this.btn_EEPROMRead = new System.Windows.Forms.Button();
            this.textBox_openDValue = new System.Windows.Forms.TextBox();
            this.label4 = new System.Windows.Forms.Label();
            this.label5 = new System.Windows.Forms.Label();
            this.textBox_openQValue = new System.Windows.Forms.TextBox();
            this.label6 = new System.Windows.Forms.Label();
            this.textBox_DeltaAngle = new System.Windows.Forms.TextBox();
            ((System.ComponentModel.ISupportInitialize)(this.numericUpDown1)).BeginInit();
            this.tabControl1.SuspendLayout();
            this.tabPage1.SuspendLayout();
            this.groupBox_EncoderMode.SuspendLayout();
            this.groupBox_ControlMode.SuspendLayout();
            this.tabPage2.SuspendLayout();
            ((System.ComponentModel.ISupportInitialize)(this.dataGridView1)).BeginInit();
            this.tabPage3.SuspendLayout();
            this.groupBox1.SuspendLayout();
            this.SuspendLayout();
            // 
            // rbText
            // 
            this.rbText.BackColor = System.Drawing.SystemColors.ActiveCaptionText;
            this.rbText.Font = new System.Drawing.Font("Verdana", 8.25F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.rbText.ForeColor = System.Drawing.SystemColors.Window;
            this.rbText.Location = new System.Drawing.Point(7, 6);
            this.rbText.Name = "rbText";
            this.rbText.ReadOnly = true;
            this.rbText.Size = new System.Drawing.Size(982, 482);
            this.rbText.TabIndex = 0;
            this.rbText.Text = "";
            // 
            // btnPortClose
            // 
            this.btnPortClose.BackColor = System.Drawing.SystemColors.ButtonFace;
            this.btnPortClose.Font = new System.Drawing.Font("Verdana", 8.25F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.btnPortClose.ForeColor = System.Drawing.Color.Black;
            this.btnPortClose.Location = new System.Drawing.Point(358, 7);
            this.btnPortClose.Name = "btnPortClose";
            this.btnPortClose.Size = new System.Drawing.Size(83, 28);
            this.btnPortClose.TabIndex = 13;
            this.btnPortClose.Text = "disconnect";
            this.btnPortClose.UseVisualStyleBackColor = false;
            this.btnPortClose.Visible = false;
            this.btnPortClose.Click += new System.EventHandler(this.btnPortClose_Click);
            // 
            // lbStatus
            // 
            this.lbStatus.AutoSize = true;
            this.lbStatus.BackColor = System.Drawing.Color.Transparent;
            this.lbStatus.Font = new System.Drawing.Font("맑은 고딕", 8.25F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(129)));
            this.lbStatus.ForeColor = System.Drawing.Color.DodgerBlue;
            this.lbStatus.Location = new System.Drawing.Point(467, 14);
            this.lbStatus.Name = "lbStatus";
            this.lbStatus.Size = new System.Drawing.Size(71, 13);
            this.lbStatus.TabIndex = 12;
            this.lbStatus.Text = "Not Connect";
            // 
            // label2
            // 
            this.label2.AutoSize = true;
            this.label2.Font = new System.Drawing.Font("Verdana", 8.25F, System.Drawing.FontStyle.Bold, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.label2.ForeColor = System.Drawing.Color.Gray;
            this.label2.Location = new System.Drawing.Point(166, 14);
            this.label2.Name = "label2";
            this.label2.Size = new System.Drawing.Size(72, 13);
            this.label2.TabIndex = 4;
            this.label2.Text = "Baud Rate";
            // 
            // label1
            // 
            this.label1.AutoSize = true;
            this.label1.Font = new System.Drawing.Font("Verdana", 8.25F, System.Drawing.FontStyle.Bold, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.label1.ForeColor = System.Drawing.Color.Gray;
            this.label1.Location = new System.Drawing.Point(6, 14);
            this.label1.Name = "label1";
            this.label1.Size = new System.Drawing.Size(65, 13);
            this.label1.TabIndex = 3;
            this.label1.Text = "COM Port";
            // 
            // btnOpen
            // 
            this.btnOpen.BackColor = System.Drawing.SystemColors.ButtonHighlight;
            this.btnOpen.Font = new System.Drawing.Font("Verdana", 8.25F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.btnOpen.ForeColor = System.Drawing.Color.Black;
            this.btnOpen.Location = new System.Drawing.Point(358, 7);
            this.btnOpen.Name = "btnOpen";
            this.btnOpen.Size = new System.Drawing.Size(83, 28);
            this.btnOpen.TabIndex = 2;
            this.btnOpen.Text = "connect";
            this.btnOpen.UseVisualStyleBackColor = false;
            this.btnOpen.Click += new System.EventHandler(this.btnOpen_Click);
            // 
            // cmbBRate
            // 
            this.cmbBRate.Font = new System.Drawing.Font("Verdana", 8.25F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.cmbBRate.FormattingEnabled = true;
            this.cmbBRate.Items.AddRange(new object[] {
            "9600 bps",
            "14400 bps",
            "19200 bps",
            "38400 bps",
            "57600 bps",
            "115200 bps"});
            this.cmbBRate.Location = new System.Drawing.Point(244, 11);
            this.cmbBRate.Name = "cmbBRate";
            this.cmbBRate.Size = new System.Drawing.Size(108, 21);
            this.cmbBRate.TabIndex = 1;
            this.cmbBRate.SelectedIndexChanged += new System.EventHandler(this.cmbBRate_SelectedIndexChanged);
            // 
            // cmbPort
            // 
            this.cmbPort.Font = new System.Drawing.Font("Verdana", 8.25F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.cmbPort.FormattingEnabled = true;
            this.cmbPort.Location = new System.Drawing.Point(77, 11);
            this.cmbPort.Name = "cmbPort";
            this.cmbPort.Size = new System.Drawing.Size(83, 21);
            this.cmbPort.TabIndex = 0;
            this.cmbPort.SelectedIndexChanged += new System.EventHandler(this.cmbPort_SelectedIndexChanged);
            // 
            // SerialPort
            // 
            this.SerialPort.DataReceived += new System.IO.Ports.SerialDataReceivedEventHandler(this.SP_DataReceived);
            // 
            // numericUpDown1
            // 
            this.numericUpDown1.Increment = new decimal(new int[] {
            10,
            0,
            0,
            0});
            this.numericUpDown1.Location = new System.Drawing.Point(12, 580);
            this.numericUpDown1.Maximum = new decimal(new int[] {
            2200,
            0,
            0,
            0});
            this.numericUpDown1.Minimum = new decimal(new int[] {
            800,
            0,
            0,
            0});
            this.numericUpDown1.Name = "numericUpDown1";
            this.numericUpDown1.Size = new System.Drawing.Size(120, 22);
            this.numericUpDown1.TabIndex = 17;
            this.numericUpDown1.Value = new decimal(new int[] {
            1500,
            0,
            0,
            0});
            this.numericUpDown1.ValueChanged += new System.EventHandler(this.numericUpDown1_ValueChanged);
            // 
            // tabControl1
            // 
            this.tabControl1.Controls.Add(this.tabPage1);
            this.tabControl1.Controls.Add(this.tabPage2);
            this.tabControl1.Controls.Add(this.tabPage3);
            this.tabControl1.Location = new System.Drawing.Point(1, 2);
            this.tabControl1.Name = "tabControl1";
            this.tabControl1.Padding = new System.Drawing.Point(6, 7);
            this.tabControl1.SelectedIndex = 0;
            this.tabControl1.Size = new System.Drawing.Size(1003, 572);
            this.tabControl1.TabIndex = 18;
            // 
            // tabPage1
            // 
            this.tabPage1.Controls.Add(this.groupBox_EncoderMode);
            this.tabPage1.Controls.Add(this.groupBox_ControlMode);
            this.tabPage1.Location = new System.Drawing.Point(4, 31);
            this.tabPage1.Name = "tabPage1";
            this.tabPage1.Padding = new System.Windows.Forms.Padding(3);
            this.tabPage1.Size = new System.Drawing.Size(995, 537);
            this.tabPage1.TabIndex = 2;
            this.tabPage1.Text = "Control";
            this.tabPage1.UseVisualStyleBackColor = true;
            // 
            // groupBox_EncoderMode
            // 
            this.groupBox_EncoderMode.Controls.Add(this.label3);
            this.groupBox_EncoderMode.Controls.Add(this.cmbEncoderMode);
            this.groupBox_EncoderMode.Location = new System.Drawing.Point(218, 6);
            this.groupBox_EncoderMode.Name = "groupBox_EncoderMode";
            this.groupBox_EncoderMode.Size = new System.Drawing.Size(304, 51);
            this.groupBox_EncoderMode.TabIndex = 26;
            this.groupBox_EncoderMode.TabStop = false;
            this.groupBox_EncoderMode.Text = "Encoder Mode";
            // 
            // label3
            // 
            this.label3.AutoSize = true;
            this.label3.BackColor = System.Drawing.Color.Transparent;
            this.label3.ForeColor = System.Drawing.Color.Black;
            this.label3.Location = new System.Drawing.Point(28, 21);
            this.label3.Name = "label3";
            this.label3.Size = new System.Drawing.Size(35, 14);
            this.label3.TabIndex = 27;
            this.label3.Text = "type";
            // 
            // cmbEncoderMode
            // 
            this.cmbEncoderMode.FormattingEnabled = true;
            this.cmbEncoderMode.Items.AddRange(new object[] {
            "ENC_NONE",
            "ENC_ABI",
            "ENC_AS504x_SPI",
            "ENC_AHALL,",
            "ENC_PWM"});
            this.cmbEncoderMode.Location = new System.Drawing.Point(86, 17);
            this.cmbEncoderMode.Name = "cmbEncoderMode";
            this.cmbEncoderMode.Size = new System.Drawing.Size(147, 22);
            this.cmbEncoderMode.TabIndex = 22;
            this.cmbEncoderMode.SelectedIndexChanged += new System.EventHandler(this.cmbEncoderMode_SelectedIndexChanged);
            // 
            // groupBox_ControlMode
            // 
            this.groupBox_ControlMode.Controls.Add(this.label6);
            this.groupBox_ControlMode.Controls.Add(this.rdBtn_OpenLoop);
            this.groupBox_ControlMode.Controls.Add(this.rdBtn_ClosedLoop);
            this.groupBox_ControlMode.Controls.Add(this.textBox_DeltaAngle);
            this.groupBox_ControlMode.Controls.Add(this.label4);
            this.groupBox_ControlMode.Controls.Add(this.textBox_openDValue);
            this.groupBox_ControlMode.Controls.Add(this.label5);
            this.groupBox_ControlMode.Controls.Add(this.textBox_openQValue);
            this.groupBox_ControlMode.Location = new System.Drawing.Point(7, 6);
            this.groupBox_ControlMode.Name = "groupBox_ControlMode";
            this.groupBox_ControlMode.Size = new System.Drawing.Size(205, 158);
            this.groupBox_ControlMode.TabIndex = 25;
            this.groupBox_ControlMode.TabStop = false;
            this.groupBox_ControlMode.Text = "Control Mode";
            // 
            // rdBtn_OpenLoop
            // 
            this.rdBtn_OpenLoop.AutoSize = true;
            this.rdBtn_OpenLoop.ForeColor = System.Drawing.Color.Black;
            this.rdBtn_OpenLoop.Location = new System.Drawing.Point(6, 21);
            this.rdBtn_OpenLoop.Name = "rdBtn_OpenLoop";
            this.rdBtn_OpenLoop.Size = new System.Drawing.Size(90, 18);
            this.rdBtn_OpenLoop.TabIndex = 20;
            this.rdBtn_OpenLoop.Text = "OpenLoop";
            this.rdBtn_OpenLoop.UseVisualStyleBackColor = true;
            this.rdBtn_OpenLoop.CheckedChanged += new System.EventHandler(this.rdBtn_OpenLoop_CheckedChanged);
            // 
            // rdBtn_ClosedLoop
            // 
            this.rdBtn_ClosedLoop.AutoSize = true;
            this.rdBtn_ClosedLoop.Checked = true;
            this.rdBtn_ClosedLoop.ForeColor = System.Drawing.Color.Black;
            this.rdBtn_ClosedLoop.Location = new System.Drawing.Point(102, 21);
            this.rdBtn_ClosedLoop.Name = "rdBtn_ClosedLoop";
            this.rdBtn_ClosedLoop.Size = new System.Drawing.Size(99, 18);
            this.rdBtn_ClosedLoop.TabIndex = 21;
            this.rdBtn_ClosedLoop.TabStop = true;
            this.rdBtn_ClosedLoop.Text = "ClosedLoop";
            this.rdBtn_ClosedLoop.UseVisualStyleBackColor = true;
            // 
            // tabPage2
            // 
            this.tabPage2.Controls.Add(this.btn_CSVImport);
            this.tabPage2.Controls.Add(this.btn_CSVExport);
            this.tabPage2.Controls.Add(this.dataGridView1);
            this.tabPage2.Location = new System.Drawing.Point(4, 31);
            this.tabPage2.Name = "tabPage2";
            this.tabPage2.Padding = new System.Windows.Forms.Padding(3);
            this.tabPage2.Size = new System.Drawing.Size(995, 537);
            this.tabPage2.TabIndex = 0;
            this.tabPage2.Text = "settingValue";
            this.tabPage2.UseVisualStyleBackColor = true;
            // 
            // btn_CSVImport
            // 
            this.btn_CSVImport.ForeColor = System.Drawing.Color.Black;
            this.btn_CSVImport.Location = new System.Drawing.Point(757, 503);
            this.btn_CSVImport.Name = "btn_CSVImport";
            this.btn_CSVImport.Size = new System.Drawing.Size(113, 28);
            this.btn_CSVImport.TabIndex = 20;
            this.btn_CSVImport.Text = "CSV Import";
            this.btn_CSVImport.UseVisualStyleBackColor = true;
            this.btn_CSVImport.Click += new System.EventHandler(this.btn_CSVImport_Click);
            // 
            // btn_CSVExport
            // 
            this.btn_CSVExport.ForeColor = System.Drawing.Color.Black;
            this.btn_CSVExport.Location = new System.Drawing.Point(876, 503);
            this.btn_CSVExport.Name = "btn_CSVExport";
            this.btn_CSVExport.Size = new System.Drawing.Size(113, 28);
            this.btn_CSVExport.TabIndex = 19;
            this.btn_CSVExport.Text = "CSV Export";
            this.btn_CSVExport.UseVisualStyleBackColor = true;
            this.btn_CSVExport.Click += new System.EventHandler(this.btn_CSVExport_Click);
            // 
            // dataGridView1
            // 
            dataGridViewCellStyle3.BackColor = System.Drawing.SystemColors.Window;
            dataGridViewCellStyle3.ForeColor = System.Drawing.SystemColors.WindowText;
            dataGridViewCellStyle3.SelectionBackColor = System.Drawing.SystemColors.Highlight;
            dataGridViewCellStyle3.SelectionForeColor = System.Drawing.SystemColors.HighlightText;
            this.dataGridView1.AlternatingRowsDefaultCellStyle = dataGridViewCellStyle3;
            this.dataGridView1.ColumnHeadersHeightSizeMode = System.Windows.Forms.DataGridViewColumnHeadersHeightSizeMode.AutoSize;
            this.dataGridView1.Location = new System.Drawing.Point(3, 3);
            this.dataGridView1.Name = "dataGridView1";
            dataGridViewCellStyle4.BackColor = System.Drawing.SystemColors.Window;
            dataGridViewCellStyle4.Font = new System.Drawing.Font("Verdana", 9F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            dataGridViewCellStyle4.ForeColor = System.Drawing.SystemColors.WindowText;
            dataGridViewCellStyle4.SelectionBackColor = System.Drawing.SystemColors.Highlight;
            dataGridViewCellStyle4.SelectionForeColor = System.Drawing.SystemColors.HighlightText;
            this.dataGridView1.RowsDefaultCellStyle = dataGridViewCellStyle4;
            this.dataGridView1.RowTemplate.Height = 23;
            this.dataGridView1.Size = new System.Drawing.Size(986, 445);
            this.dataGridView1.TabIndex = 0;
            this.dataGridView1.DataBindingComplete += new System.Windows.Forms.DataGridViewBindingCompleteEventHandler(this.dataGridView1_BindingCompleteEvent);
            // 
            // tabPage3
            // 
            this.tabPage3.Controls.Add(this.rbText);
            this.tabPage3.Location = new System.Drawing.Point(4, 31);
            this.tabPage3.Name = "tabPage3";
            this.tabPage3.Padding = new System.Windows.Forms.Padding(3);
            this.tabPage3.Size = new System.Drawing.Size(995, 537);
            this.tabPage3.TabIndex = 1;
            this.tabPage3.Text = "Terminal";
            this.tabPage3.UseVisualStyleBackColor = true;
            // 
            // groupBox1
            // 
            this.groupBox1.BackColor = System.Drawing.Color.Transparent;
            this.groupBox1.Controls.Add(this.btnPortClose);
            this.groupBox1.Controls.Add(this.lbStatus);
            this.groupBox1.Controls.Add(this.label2);
            this.groupBox1.Controls.Add(this.label1);
            this.groupBox1.Controls.Add(this.btnOpen);
            this.groupBox1.Controls.Add(this.cmbBRate);
            this.groupBox1.Controls.Add(this.cmbPort);
            this.groupBox1.Location = new System.Drawing.Point(433, -7);
            this.groupBox1.Name = "groupBox1";
            this.groupBox1.Size = new System.Drawing.Size(571, 37);
            this.groupBox1.TabIndex = 1;
            this.groupBox1.TabStop = false;
            // 
            // btn_EEPROMWrite
            // 
            this.btn_EEPROMWrite.ForeColor = System.Drawing.Color.Black;
            this.btn_EEPROMWrite.Location = new System.Drawing.Point(851, 695);
            this.btn_EEPROMWrite.Name = "btn_EEPROMWrite";
            this.btn_EEPROMWrite.Size = new System.Drawing.Size(145, 23);
            this.btn_EEPROMWrite.TabIndex = 19;
            this.btn_EEPROMWrite.Text = "EEPROM Write";
            this.btn_EEPROMWrite.UseVisualStyleBackColor = true;
            // 
            // btn_EEPROMRead
            // 
            this.btn_EEPROMRead.ForeColor = System.Drawing.Color.Black;
            this.btn_EEPROMRead.Location = new System.Drawing.Point(700, 695);
            this.btn_EEPROMRead.Name = "btn_EEPROMRead";
            this.btn_EEPROMRead.Size = new System.Drawing.Size(145, 23);
            this.btn_EEPROMRead.TabIndex = 20;
            this.btn_EEPROMRead.Text = "EEPROM Read";
            this.btn_EEPROMRead.UseVisualStyleBackColor = true;
            // 
            // textBox_openDValue
            // 
            this.textBox_openDValue.Location = new System.Drawing.Point(85, 60);
            this.textBox_openDValue.Name = "textBox_openDValue";
            this.textBox_openDValue.Size = new System.Drawing.Size(100, 22);
            this.textBox_openDValue.TabIndex = 28;
            // 
            // label4
            // 
            this.label4.AutoSize = true;
            this.label4.BackColor = System.Drawing.Color.Transparent;
            this.label4.ForeColor = System.Drawing.Color.Black;
            this.label4.Location = new System.Drawing.Point(36, 64);
            this.label4.Name = "label4";
            this.label4.Size = new System.Drawing.Size(16, 14);
            this.label4.TabIndex = 29;
            this.label4.Text = "D";
            // 
            // label5
            // 
            this.label5.AutoSize = true;
            this.label5.BackColor = System.Drawing.Color.Transparent;
            this.label5.ForeColor = System.Drawing.Color.Black;
            this.label5.Location = new System.Drawing.Point(36, 92);
            this.label5.Name = "label5";
            this.label5.Size = new System.Drawing.Size(17, 14);
            this.label5.TabIndex = 31;
            this.label5.Text = "Q";
            // 
            // textBox_openQValue
            // 
            this.textBox_openQValue.Location = new System.Drawing.Point(85, 88);
            this.textBox_openQValue.Name = "textBox_openQValue";
            this.textBox_openQValue.Size = new System.Drawing.Size(100, 22);
            this.textBox_openQValue.TabIndex = 30;
            // 
            // label6
            // 
            this.label6.AutoSize = true;
            this.label6.BackColor = System.Drawing.Color.Transparent;
            this.label6.ForeColor = System.Drawing.Color.Black;
            this.label6.Location = new System.Drawing.Point(7, 120);
            this.label6.Name = "label6";
            this.label6.Size = new System.Drawing.Size(74, 14);
            this.label6.TabIndex = 33;
            this.label6.Text = "deltaAngle";
            // 
            // textBox_DeltaAngle
            // 
            this.textBox_DeltaAngle.Location = new System.Drawing.Point(85, 116);
            this.textBox_DeltaAngle.Name = "textBox_DeltaAngle";
            this.textBox_DeltaAngle.Size = new System.Drawing.Size(100, 22);
            this.textBox_DeltaAngle.TabIndex = 32;
            // 
            // Form1
            // 
            this.AutoScaleDimensions = new System.Drawing.SizeF(8F, 14F);
            this.AutoScaleMode = System.Windows.Forms.AutoScaleMode.Font;
            this.AutoScroll = true;
            this.BackColor = System.Drawing.SystemColors.Menu;
            this.ClientSize = new System.Drawing.Size(1008, 730);
            this.Controls.Add(this.btn_EEPROMRead);
            this.Controls.Add(this.btn_EEPROMWrite);
            this.Controls.Add(this.groupBox1);
            this.Controls.Add(this.tabControl1);
            this.Controls.Add(this.numericUpDown1);
            this.Font = new System.Drawing.Font("Verdana", 9F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.ForeColor = System.Drawing.Color.White;
            this.MaximizeBox = false;
            this.MaximumSize = new System.Drawing.Size(1024, 768);
            this.MinimizeBox = false;
            this.MinimumSize = new System.Drawing.Size(1024, 768);
            this.Name = "Form1";
            this.StartPosition = System.Windows.Forms.FormStartPosition.CenterScreen;
            this.Text = "시리얼 통신 V1.0";
            this.FormClosed += new System.Windows.Forms.FormClosedEventHandler(this.Form1_FormClosed);
            this.Load += new System.EventHandler(this.Form1_Load);
            ((System.ComponentModel.ISupportInitialize)(this.numericUpDown1)).EndInit();
            this.tabControl1.ResumeLayout(false);
            this.tabPage1.ResumeLayout(false);
            this.groupBox_EncoderMode.ResumeLayout(false);
            this.groupBox_EncoderMode.PerformLayout();
            this.groupBox_ControlMode.ResumeLayout(false);
            this.groupBox_ControlMode.PerformLayout();
            this.tabPage2.ResumeLayout(false);
            ((System.ComponentModel.ISupportInitialize)(this.dataGridView1)).EndInit();
            this.tabPage3.ResumeLayout(false);
            this.groupBox1.ResumeLayout(false);
            this.groupBox1.PerformLayout();
            this.ResumeLayout(false);

        }

        #endregion

        private System.Windows.Forms.RichTextBox rbText;
        private System.Windows.Forms.ComboBox cmbPort;
        private System.Windows.Forms.ComboBox cmbBRate;
        private System.Windows.Forms.Button btnOpen;
        private System.Windows.Forms.Label label1;
        private System.Windows.Forms.Label label2;
        private System.Windows.Forms.Label lbStatus;
        private System.IO.Ports.SerialPort SerialPort;
        private System.Windows.Forms.Button btnPortClose;
        private System.Windows.Forms.NumericUpDown numericUpDown1;
        private System.Windows.Forms.TabControl tabControl1;
        private System.Windows.Forms.TabPage tabPage2;
        private System.Windows.Forms.TabPage tabPage3;
        private System.Windows.Forms.GroupBox groupBox1;
        private System.Windows.Forms.DataGridView dataGridView1;
        private System.Windows.Forms.Button btn_CSVExport;
        private System.Windows.Forms.TabPage tabPage1;
        private System.Windows.Forms.GroupBox groupBox_EncoderMode;
        private System.Windows.Forms.Label label3;
        private System.Windows.Forms.ComboBox cmbEncoderMode;
        private System.Windows.Forms.GroupBox groupBox_ControlMode;
        private System.Windows.Forms.RadioButton rdBtn_OpenLoop;
        private System.Windows.Forms.RadioButton rdBtn_ClosedLoop;
        private System.Windows.Forms.Button btn_EEPROMWrite;
        private System.Windows.Forms.Button btn_CSVImport;
        private System.Windows.Forms.Button btn_EEPROMRead;
        private System.Windows.Forms.TextBox textBox_openDValue;
        private System.Windows.Forms.Label label6;
        private System.Windows.Forms.TextBox textBox_DeltaAngle;
        private System.Windows.Forms.Label label5;
        private System.Windows.Forms.TextBox textBox_openQValue;
        private System.Windows.Forms.Label label4;
    }
}

