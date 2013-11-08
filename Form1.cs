using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Data;
using System.Drawing;
using System.Linq;
using System.Text;
using System.Windows.Forms;
using System.IO.Ports;
using System.IO;
using System.Globalization;
using System.Threading;
using System.Collections;
using MathNet.Numerics.LinearAlgebra.Double;



namespace imu_reader_sharp
{
    public partial class Form1 : Form
    {
        const int MAX_COM_NUM = 50;
        int validCOM = 0;
        string log_name;
        string selected_com = "COM0";   // СОМ0 означает что датчик не выбран
        SerialPort active_com = new SerialPort();
        bool[] flags = new bool[2];
        Queue files_q = new Queue();    // считанные файлы добавляются в очередь для дальнейшей обработки
        int block_index = 0;

        file[] files_st;    // после окончания чтения файлы из очереди сохраняются в этот массив

        public Form1()
        {
            InitializeComponent();
        }

        // инициализация выбранного СОМ порта
        private void Connect ()
        {
	        char[] button_name = new char[200];
	        char[] tempS = new char[200];

	        validCOM = 1;
            active_com = new SerialPort(selected_com, 3000000, 0, 8, StopBits.One);
            active_com.WriteBufferSize = 512;
            active_com.ReadBufferSize = 8192;
            active_com.Open();
            //active_com.ErrorReceived += new SerialErrorReceivedEventHandler(port_error);
            //active_com.Disposed += new EventHandler(port_error);
        }

        private void port_error(object sender, EventArgs e)
        {
            if (!active_com.IsOpen)
            {
                MessageBox.Show("Сбой при чтении данных (Этап 3).\nПроверьте подключение датчика.");
                Disconnect();
                return;
            }
        }

        // Отключение от выбранного СОМ порта
        private void Disconnect()
        {
	        active_com.Close();
	        validCOM = 0;
            infoLabel.Text = "";
            infoLabel.Update();
        }

        // Поиск доступных СОМ портов и отображение
        // списка в textBox
        private void com_search()
        {
            string[] ports = SerialPort.GetPortNames();

            foreach (string port in ports)
                comBox.Items.Add(port);
            if (ports.Length != 0)
            {
                comBox.SelectedIndex = 0;
                selected_com = comBox.SelectedItem.ToString();
            }
        }

        // Запуск поиска СОМ портов при запуске программы
        private void Form1_Load(object sender, EventArgs e)
        {
            com_search();
        }

        // Выбор файла для сохранения данных
        private void fileButton_Click(object sender, EventArgs e)
        {
            saveFileDialog.InitialDirectory = "";
            saveFileDialog.Filter = "Все файлы (*.*)|*.*";
            saveFileDialog.Title = "Выберите файл для сохранения данных";
            DialogResult result = saveFileDialog.ShowDialog();
            if (result == DialogResult.OK)
            {
                flags[0] = true;
                string temp_file;
                string[] file_name_parts = saveFileDialog.FileName.Split('\\');
                temp_file = file_name_parts[file_name_parts.Length - 1];
                file_name_parts = temp_file.Split('.');
                fileLabel.Text = file_name_parts[0] + ".imu/.gps";
                infoLabel.Text = "Готовность к сохранению";
                infoLabel.Update();
                fileLabel.Update();
                log_name = saveFileDialog.FileName;
            }
            else
            {
                flags[0] = false;
                infoLabel.Text = "Неудалось выбрать файл";
                fileLabel.Text = "Недоступно";
            }
            check_save_available(sender, e);
        }

        // Проверка на уловие доступности кнопки чтения
        private void check_read_available(object sender, EventArgs e)
        {
            if ((comBox.Items.Count != 0) && !flags[1])
                readButton.Enabled = true;
            else
                readButton.Enabled = false;
        }

        // Очистка карты памяти модуля
        private void clearButton_Click(object sender, EventArgs e)
        {
            System.Diagnostics.Stopwatch timer = new System.Diagnostics.Stopwatch();
            char[] buffer = new char[8192];
	        int numb = 0;
            try
            {
                Connect();
            }
            catch (Exception)
            {
                MessageBox.Show("Неудалось подключиться к выбранному СОМ порту.\n"+
                    "Пожалуйста, проверьте правильно ли выбран СОМ порт\n" + 
                    "и включен ли датчик.");
                return;
            }
            infoLabel.Text = "Идет очистка";
            this.Update();
            active_com.DiscardInBuffer();
            active_com.Write("v");    // запрос версии и номера датчика
            timer.Start();
            while (active_com.BytesToRead < 4)
            {
                if (timer.ElapsedMilliseconds > 1000)
                {
                    MessageBox.Show("Не получен ответ от датчика. Проверьте подключение.");
                    Disconnect();
                    readButton.Enabled = true;
                    return;
                }
            }
            timer.Stop();
            numb = active_com.Read(buffer, 0, 4);
            active_com.Write("f");
            
            
            MessageBox.Show("Очистка закончена. Выключите модуль");
        }

        // При закрытии программы закрывается СОМ порт
        private void Form1_FormClosing(object sender, FormClosingEventArgs e)
        {
            if (active_com.IsOpen)
                active_com.Close();
            Application.Exit();
        }
        
        // Функция для сохранения данных в файлы .imu/.gps
        private void write_data(file file2write)
        {
            byte[] full_file = new byte[file2write.get_size()];
            for (int i = 0; i < full_file.Length / 4096 - 1; i++)
            {
                for (int j = 0; j < 4096; j++)
                {
                    full_file[i * 4096 + j] = file2write.get_packet(i).get_data(j);
                }
            }
            uint[] ticks = new uint[full_file.Length / 35 + 1];   // примерное количество IMU пакетов
            uint[] ticks2 = new uint[ticks.Length];   // примерное количество GPS пакетов неизвестно,                             
            byte[] type = new byte[ticks.Length];   // но оно не может превышать число IMU пакетов
            
            byte[] pack = new byte[32];
            byte[] pack2 = new byte[26];
            int crc;
            int k = 0;
            int k2 = 0;
            int tt = 0;
            int[] counter = new int[ticks.Length];
            double[,] a = new double[ticks.Length, 3];
            double[,] w = new double[ticks.Length, 3];
            double[,] m = new double[ticks.Length, 3];
            double[,] q = new double[ticks.Length, 4];
            double[] anglex = new double[ticks.Length];
            double[] angley = new double[ticks.Length];
            double[] anglez = new double[ticks.Length];
            double[] lat = new double[ticks2.Length];
            double[] lon = new double[ticks2.Length];
            double[] speed = new double[ticks2.Length];
            double[] course = new double[ticks2.Length];
            double[] time = new double[ticks2.Length];
            double[] stat = new double[ticks2.Length];
            double[] date = new double[ticks2.Length];
            byte[] buffer = new byte[2];
            byte[] buffer2 = new byte[4];
            double[] corr_mag = { 2.4912, 2.4907, 2.4998, 2.5912, 3.4920, 2.5428, 2.4593, 2.5500, 2.5196, 2.5275, 1, 2.5017, 3.7391, 2.9516, 2.4844 };
            double[] corr_gyr = { 1.1111, 0.9333, 0.9333, 1.0185, 1.5122, 0.8355, 0.9148, 1.1154, 1.1453, 1.1129, 1.1028, 1.1160, 1.5063, 0.9, 0.9111 };
            double[,] zero_mag = { {0.1780, 0.1092, -0.0961}, {0.1259,  0.0621, -0.2346}, { 0.4120, 0.0386, 0.2578}, {0.2565, -0.0926, 0.0760}, { 0.1221, -0.0659, 0.2637},
                                   {0.4300, 0.0348,  0.2616}, {0.3156, -0.1437, -0.2155}, {-0.0928, 0.0603, 0.1485}, {0.0405,  0.3184, 0.1702}, {-0.0189,  0.0232, 0.3462},
                                   {0,      0,       0     }, {0.4010,  0.0459, -0.9670}, { 0.2514, 0.0564, 0.2268}, {0.2202, 0.0130, 0.3759}, { 0.2044,  0.0045, 0.2669} };
            infoLabel.Text = "Идет сохранение файла.";
            this.Update();
            for (int i = 0; i < full_file.Length - 30; i++)
            {
                if ((i < full_file.Length - 35)&&(full_file[i + 34] == 3) && (full_file[i + 33] == 16) &&
                    (full_file[i] == 16) && (full_file[i + 1] == 49))   // условие начала IMU пакета
                {
                    crc = 0;
                    for (int j = 0; j < 32; j++)
                    {
                        pack[j] = full_file[i + j + 1];
                        if (j < 31)
                            crc = crc ^ pack[j];
                    }
                    if (crc == pack[pack.Length - 1])
                    {
                        //ticks[k] = pack[4] + pack[3] * (int)Math.Pow(2, 8) + pack[2] * (int)Math.Pow(2, 16) + pack[1] * (int)Math.Pow(2, 24);
                        ticks[k] = BitConverter.ToUInt32(pack, 1);
                        type[k] = pack[0];
                        if (type[k] == 49)
                        {
                            buffer[0] = pack[7]; buffer[1] = pack[8];
                            a[k,0] = ((double)BitConverter.ToInt16(buffer,0))*0.0018;
                            buffer[0] = pack[5]; buffer[1] = pack[6];
                            a[k, 1] = (double)BitConverter.ToInt16(buffer, 0) * 0.0018;
                            buffer[0] = pack[9]; buffer[1] = pack[10];
                            a[k, 2] = -(double)BitConverter.ToInt16(buffer, 0) * 0.0018;

                            buffer[0] = pack[13]; buffer[1] = pack[14];
                            w[k, 0] = (double)BitConverter.ToInt16(buffer, 0) * 0.00053264 * corr_gyr[block_index - 1];
                            buffer[0] = pack[11]; buffer[1] = pack[12];
                            w[k, 1] = (double)BitConverter.ToInt16(buffer, 0) * 0.00053264 * corr_gyr[block_index - 1];
                            buffer[0] = pack[15]; buffer[1] = pack[16];
                            w[k, 2] = (double)BitConverter.ToInt16(buffer, 0) * 0.00053264 * corr_gyr[block_index - 1];

                            buffer[0] = pack[17]; buffer[1] = pack[18];
                            m[k, 0] = -((double)BitConverter.ToInt16(buffer, 0) * 0.00030518 - zero_mag[block_index - 1, 0]) * corr_mag[block_index - 1];
                            buffer[0] = pack[19]; buffer[1] = pack[20];
                            m[k, 1] = ((double)BitConverter.ToInt16(buffer, 0) * 0.00030518 - zero_mag[block_index - 1, 1]) * corr_mag[block_index - 1];
                            buffer[0] = pack[21]; buffer[1] = pack[22];
                            m[k, 2] = -((double)BitConverter.ToInt16(buffer, 0) * 0.00030518 - zero_mag[block_index - 1, 2]) * corr_mag[block_index - 1];

                            buffer[0] = pack[23]; buffer[1] = pack[24];
                            q[k, 0] = (double)BitConverter.ToInt16(buffer, 0);
                            buffer[0] = pack[25]; buffer[1] = pack[26];
                            q[k, 1] = (double)BitConverter.ToInt16(buffer, 0) * 0.00003125;
                            buffer[0] = pack[27]; buffer[1] = pack[28];
                            q[k, 2] = (double)BitConverter.ToInt16(buffer, 0) * 0.00003125;
                            buffer[0] = pack[29]; buffer[1] = pack[30];
                            q[k, 3] = (double)BitConverter.ToInt16(buffer, 0) * 0.00003125;
                        }
                        counter[k] = k2;
                        k++;
                    }
                    else
                        tt++;   
                }
                if ((full_file[i + 29] == 3) && (full_file[i + 28] == 16) && (full_file[i] == 16) &&
                    (full_file[i + 1] == 50))   // условие начала GPS пакета
                {
                    crc = 50;
                    for (int j = 0; j < 26; j++)
                    {
                        pack2[j] = full_file[i + j + 2];
                        if (j < 25)
                            crc = crc ^ pack2[j];
                    }
                    if (crc == pack2[pack2.Length - 1])
                    {
                        //ticks2[k2] = pack2[3] + pack2[2] * (int)Math.Pow(2, 8) +
                        //    pack2[1] * (int)Math.Pow(2, 16) + pack2[0] * (int)Math.Pow(2, 24);
                        ticks2[k2] = BitConverter.ToUInt32(pack2, 0);
                        buffer2[0] = pack2[4]; buffer2[1] = pack2[5]; buffer2[2] = pack2[6]; buffer2[3] = pack2[7];
                        lat[k2] = ((double)BitConverter.ToInt32(buffer2, 0))/600000;
                        buffer2[0] = pack2[8]; buffer2[1] = pack2[9]; buffer2[2] = pack2[10]; buffer2[3] = pack2[11];
                        lon[k2] = ((double)BitConverter.ToInt32(buffer2, 0)) / 600000;
                        buffer[0] = pack2[12]; buffer[1] = pack2[13];
                        speed[k2] = (double)BitConverter.ToInt16(buffer, 0)/100;
                        buffer[0] = pack2[14]; buffer[1] = pack2[15];
                        course[k2] = (double)BitConverter.ToInt16(buffer, 0)/160;
                        buffer2[0] = pack2[16]; buffer2[1] = pack2[17]; buffer2[2] = pack2[18]; buffer2[3] = pack2[19];
                        time[k2] = ((double)BitConverter.ToInt32(buffer2, 0)) / 10;
                        stat[k2] = pack2[20];
                        buffer2[0] = pack2[21]; buffer2[1] = pack2[22]; buffer2[2] = pack2[23]; buffer2[3] = pack2[24];
                        date[k2] = ((double)BitConverter.ToInt32(buffer2, 0));
                        k2++;
                        
                    }
                }
            }
            // -------------- Сохранение в IMU/GPS
            infoLabel.Text = "Сохранение файла.";
            infoLabel.Update();
            //fileLabel.Text += " - " + block_index;
            //fileLabel.Update();
            
            DenseVector Magn_coefs = new DenseVector(get_magn_coefs(block_index));
            DenseVector Accl_coefs = new DenseVector(get_accl_coefs(block_index));
            DenseVector Gyro_coefs = new DenseVector(12);
            Kalman_class.Parameters Parameters = new Kalman_class.Parameters(Accl_coefs, Magn_coefs, Gyro_coefs);
            Kalman_class.Sensors Sensors = new Kalman_class.Sensors(new DenseMatrix(1, 3, 0), new DenseMatrix(1, 3, 0), new DenseMatrix(1, 3, 0));
            Matrix Initia_quat = new DenseMatrix(1, 4, 0);
            Initia_quat.At(0, 0, 1);
            Kalman_class.State State = new Kalman_class.State(Kalman_class.ACCLERATION_NOISE, Kalman_class.MAGNETIC_FIELD_NOISE, Kalman_class.ANGULAR_VELOCITY_NOISE,
                Math.Pow(10, -6), Math.Pow(10, -15), Math.Pow(10, -15), Initia_quat);
            double[] angles = new double[3];
            double[] mw, ma, mm;
            ma = new double[3];
            mw = new double[3];
            mm = new double[3];
            Tuple<Vector, Kalman_class.Sensors, Kalman_class.State> AHRS_result;

            string[] name_mass = log_name.Split('.');
            string file2save = "";
            for (int i = 0; i < name_mass.Length - 1; i++)
                file2save += name_mass[i] + ".";
            if (name_mass.Length == 1)
                file2save = log_name + ".";
            FileStream fs_imu = File.Create(file2save + "imu", 2048, FileOptions.None);
            BinaryWriter str_imu = new BinaryWriter(fs_imu);
            FileStream fs_gps = File.Create(file2save + "gps", 2048, FileOptions.None);
            BinaryWriter str_gps = new BinaryWriter(fs_gps);
            Int16 buf16; Byte buf8; Int32 buf32;
            Double bufD; Single bufS; UInt32 bufU32;
            progressBar.Invoke(new Action(() => progressBar.Maximum = k + k2));
            progressBar.Invoke(new Action(() => progressBar.Value = 0));

            double[] magn_c = get_magn_coefs(block_index);
            double[] accl_c = get_accl_coefs(block_index);
            double[] gyro_c = new double[12];

            double[] w_helper = new double [ticks.Length];

            for (int i=0; i<w_helper.Length; i++) w_helper[i] = w[i,0];
            anglex = Signal_processing.Zero_average_corr(w_helper, w_helper.Length);
            for (int i=0; i<w_helper.Length; i++) w_helper[i] = w[i,1];
            angley = Signal_processing.Zero_average_corr(w_helper, w_helper.Length);
            for (int i=0; i<w_helper.Length; i++) w_helper[i] = w[i,2];
            anglez = Signal_processing.Zero_average_corr(w_helper, w_helper.Length);

            for (int i = 0; i < k; i++)
            {
                progressBar.Invoke(new Action(() => progressBar.Value++));

               /* Sensors.a.At(0, 0, a[i, 0]);
                Sensors.a.At(0, 1, a[i, 1]);
                Sensors.a.At(0, 2, a[i, 2]);

                Sensors.w.At(0, 0, w[i, 0]);
                Sensors.w.At(0, 1, w[i, 1]);
                Sensors.w.At(0, 2, w[i, 2]);

                Sensors.m.At(0, 0, m[i, 0]);
                Sensors.m.At(0, 1, m[i, 1]);
                Sensors.m.At(0, 2, m[i, 2]);

                AHRS_result = Kalman_class.AHRS_LKF_EULER(Sensors, State, Parameters);

                State = AHRS_result.Item3;*/
                //------------------------------------------------------------------------
                mm = single_correction(magn_c, m[i, 0], m[i, 1], m[i, 2]);
                ma = single_correction(accl_c, a[i, 0], a[i, 1], a[i, 2]);
                mw = single_correction(gyro_c, w[i, 0], w[i, 1], w[i, 2]);
                //----------------------------------------------------------------------
                //mw[0] = w[i, 0];
                //mw[1] = w[i, 1];
                //mw[2] = w[i, 2];
                //ma[0] = a[i, 0];
                //ma[1] = a[i, 1];
                //ma[2] = a[i, 2];
                //mm[0] = m[i, 0];
                //mm[1] = m[i, 1];
                //mm[2] = m[i, 2];
                //----------------------------------------------------------------------
                angles[0] = (anglez[i]);
                angles[1] = (angley[i]);
                angles[2] = (anglex[i]);

                // IMU
                buf16 = (Int16)(angles[0] * 10000);
                str_imu.Write(buf16);
                buf16 = (Int16)(angles[1] * 10000);
                str_imu.Write(buf16);
                buf16 = (Int16)(angles[2] * 10000);
                str_imu.Write(buf16);

                buf16 = (Int16)(mw[0] * 3000);
                str_imu.Write(buf16);
                buf16 = (Int16)(mw[1] * 3000);
                str_imu.Write(buf16);
                buf16 = (Int16)(mw[2] * 3000);
                str_imu.Write(buf16);

                buf16 = (Int16)(ma[0] * 3000);
                str_imu.Write(buf16);
                buf16 = (Int16)(ma[1] * 3000);
                str_imu.Write(buf16);
                buf16 = (Int16)(ma[2] * 3000);
                str_imu.Write(buf16);

                buf16 = (Int16)(mm[0] * 3000);
                str_imu.Write(buf16);
                buf16 = (Int16)(mm[1] * 3000);
                str_imu.Write(buf16);
                buf16 = (Int16)(mm[2] * 3000);
                str_imu.Write(buf16);

                buf16 = (Int16)(q[i, 0]);
                str_imu.Write(buf16);

                //buf32 = (Int32)(counter[i]);
                buf32 = (Int32)(ticks[i]);
                str_imu.Write(buf32);

                buf8 = (Byte)(0);
                str_imu.Write(buf8);
            }
            for (int i = 0; i < k2; i++)
            {
                progressBar.Invoke(new Action(() => progressBar.Value++));
                // GPS
                bufD = (Double)(lat[i]) / ((180 / Math.PI) * 16.66);
                str_gps.Write(bufD);
                bufD = (Double)(lon[i]) / ((180 / Math.PI) * 16.66);
                str_gps.Write(bufD);
                bufD = (Double)(0);
                str_gps.Write(bufD);

                bufS = (Single)(time[i]);
                str_gps.Write(bufS);
                bufS = (Single)(speed[i]);
                str_gps.Write(bufS);
                bufS = (Single)(0);
                str_gps.Write(bufS);
                str_gps.Write(bufS);

                //bufU32 = (UInt32)(i);
                bufU32 = (UInt32)(ticks2[i]);
                str_gps.Write(bufU32);
                buf8 = (Byte)(0);
                str_gps.Write(buf8);
                str_gps.Write(buf8);
                str_gps.Write(buf8);
            }
            str_imu.Flush();
            str_imu.Close();
            str_gps.Flush();
            str_gps.Close();
                        

            MessageBox.Show("Сохранение завершено");
        }
    

        // Функция для чтения данных напрямую с датчика
        private void readButton_Click(object sender, EventArgs e)
        {
            readButton.Enabled = false;
            try
            {
                fileBox.Items.Clear();
                fileBox.Update();
                byte[] buffer = new byte[8192]; // максимальный пакет 4096 байт, взято с запасом
                int numb = 0;
                char[] tempS = new char[200];
                long numfiles = 0;
                long fcur = 0;
                char[] log_name_c = new char[2048];
                System.Diagnostics.Stopwatch timer = new System.Diagnostics.Stopwatch();
                //System.Diagnostics.Stopwatch control = new System.Diagnostics.Stopwatch();

                if ((validCOM == 0) && (selected_com != "COM0"))
                {
                    Connect();
                    //FileStream fs = File.Create(log_name, 2048, FileOptions.None);
                    //StreamWriter str_wr = new StreamWriter(fs);
                    //control.Start();

                    infoLabel.Text = "Идет считывание";
                    infoLabel.Update();
                    active_com.DiscardInBuffer();
                    timer.Start();
                    active_com.Write("v");    // запрос версии и номера датчика

                    while (active_com.BytesToRead < 4)
                    {
                        if (timer.ElapsedMilliseconds > 1000)
                        {
                            MessageBox.Show("Не получен ответ от датчика. Проверьте подключение.");
                            Disconnect();
                            readButton.Enabled = true;
                            return;
                        }
                    }

                    numb = active_com.Read(buffer, 0, 4);
                    if (numb < 4)
                    {
                        MessageBox.Show("Не получен ответ от датчика. Этап 1.");
                        Disconnect();
                        readButton.Enabled = true;
                        return;
                    }
                    block_index = buffer[0] * (int)Math.Pow(2, 24) + buffer[1] * (int)Math.Pow(2, 16) + buffer[2] * (int)Math.Pow(2, 8) + buffer[3];

                    active_com.Write("?");  // запрос на количество записей в памяти
                    Thread.Sleep(50);

                    numb = active_com.Read(buffer, 0, 4);
                    numfiles = buffer[0] * (int)Math.Pow(2, 24) + buffer[1] * (int)Math.Pow(2, 16) + buffer[2] * (int)Math.Pow(2, 8) + buffer[3];

                    active_com.Write("c");
                    Thread.Sleep(50);   // Временная задержка, рабочий вариант
                    numb = active_com.Read(buffer, 0, 4);
                    fcur = buffer[0] * (int)Math.Pow(2, 24) + buffer[1] * (int)Math.Pow(2, 16) + buffer[2] * (int)Math.Pow(2, 8) + buffer[3];
                    if (numb < 4)
                    {
                        MessageBox.Show("Не получен ответ от датчика. Этап 2.");
                        Disconnect();
                        readButton.Enabled = true;
                        return;
                    }

                    while (fcur > 1)
                    {
                        active_com.Write("-");
                        active_com.Read(buffer, 0, 4);
                        Thread.Sleep(20);
                        active_com.Write("c");
                        Thread.Sleep(20);
                        numb = active_com.Read(buffer, 0, 4);
                        fcur = buffer[0] * (int)Math.Pow(2, 24) + buffer[1] * (int)Math.Pow(2, 16) + buffer[2] * (int)Math.Pow(2, 8) + buffer[3];
                    }

                    int fsum = 0;
                    progressBar.Maximum = (int)numfiles;
                    progressBar.Value = 0;
                    file file2add;
                    packet packet2add;
                    int file_count = 1;
                    int packet_count = 0;
                    packetBox.Text = "" + packet_count;
                    packetBox.Update();
                    for (int q = 0; q < numfiles; q++)
                    {
                        progressBar.Value++;
                        fsum = 0;
                        
                        active_com.Write("r");
                        Thread.Sleep(20);
                        numb = active_com.Read(buffer, 0, 4);
                        file2add = new file(file_count); // для каждой записи создается новый объект file
                        //str_wr.WriteLine("file " + q + " read bytes: " + numb);
                        while (fsum != 4096 * 255)
                        {
                            fsum = 0;
                            
                            active_com.Write("n");
                            Thread.Sleep(100);
                            
                            numb = active_com.Read(buffer, 0, 4096);
                            for (int w = 0; w < numb; w++)
                                fsum += buffer[w];
                            packet2add = new packet(buffer, numb); // для каждого пакета - новый объект packet
                            file2add.add(packet2add);
                            packet_count++;
                            if (packet_count % 20 == 0)
                            {
                                packetBox.Text = "" + packet_count;
                                packetBox.Update();
                            }

                            //str_wr.Write(fsum + " ");
                        }
                        if (file2add.get_size() > 4096)
                        {
                            files_q.Enqueue(file2add);  // объект file добавляется в очередь
                            file_count++;               // только если в нем больше 1 пакета
                        }
                        //str_wr.WriteLine();
                    }
                    packetBox.Text = "" + packet_count;
                    packetBox.Update();
                    //str_wr.Flush();
                    //str_wr.Close();
                    //control.Stop();
                }
                if (validCOM == 1)
                {
                    Disconnect();
                    //MessageBox.Show("Чтение завершено. " + control.ElapsedMilliseconds + " ms elapsed.");
                    fileButton.Enabled = true;
                    flags[1] = true;    // this flag is used to keep readButton locked (unavailble)
                    MessageBox.Show("Чтение завершено.");
                    files_st = new file[files_q.Count];
                    for (int q = 0; q < files_st.Length; q++)
                    {
                        // вывод спика ненулевых записей в textBox
                        files_st[q] = (file)files_q.Dequeue();
                        fileBox.Items.Add("Запись " + files_st[q].get_id() + " - " + (files_st[q].get_size())/194700 + " мин");
                    }
                }
            }
            catch (Exception crit_error)
            {
                MessageBox.Show("Произошла критическая ошибка. Возможные причины ошибки:\n" +
                "• выбранный СОМ порт не связан с датчиком СКВП\n" + 
                "• датчик выключен\n" +
                "\n Код ошибки: " + crit_error.Message);
                if (active_com.IsOpen)
                    active_com.Close();
                readButton.Enabled = true;
                return;
            }
            return;
        }


        private void comBox_Click(object sender, EventArgs e)
        {
            selected_com = comBox.SelectedItem.ToString();
        }

        private void saveButton_Click(object sender, EventArgs e)
        {
            try
            {
                if (files_st.Length != 0)
                {
                    int save_ind = fileBox.SelectedIndex;
                    write_data(files_st[save_ind]);
                }
                else
                    MessageBox.Show("Нет файлов. Сначала считайте данные");
            }
            catch (Exception crit_error)
            {
                MessageBox.Show("Произошла критическая ошибка. Программа будет закрыта.\n" +
                "Пожалуйста обратитесь к разработчику\n" +
                "Ошибка: " + crit_error.Message);
                if (active_com.IsOpen)
                    active_com.Close();
                Application.Exit();
            }
        }

        private void check_save_available(object sender, EventArgs e)
        {
            if ((fileBox.Items.Count != 0) && (flags[0]))
            {
                if (fileBox.SelectedItem == null)
                    fileBox.SelectedIndex = 0;
                saveButton.Enabled = true;
            }
            else
                saveButton.Enabled = false;

        }

        private double[] get_accl_coefs(int index)
        {
            double[] result = new double[0];
            switch (index)
            {
                case 1:
                    double[] temp1 = { -0.0081, 0.0401, -0.0089, 0.0301,
                                        -0.0111, 0.0323, 0.0093, 0.0104, 0.0085, -0.0921, -0.1201, -0.1454 };
                    result = temp1;
                    break;
                case 2:
                    double[] temp2 = {0.043713810744819,   0.032204099593533,   0.014469608704782,   0.067770825077312,
                       0.124497968267554,  -0.069373064241876,  -0.020960499722065,  -0.119156909250651,
                       0.044421201741530,  -0.179463499557114,  -0.234503197266493,  -0.036294503190156 };
                    result = temp2;
                    break;
                case 3:
                    double[] temp3 = {0.024167075678158,   0.017921002182728,   0.027208980951327,   0.047646758555060,
                      -0.052181643992751,  -0.056215499304436,  -0.011304687635132,   0.087617101877347,
                       0.051573793476529,  -0.045096522590058,  -0.034826544109968,  -0.215775421842763 };
                    result = temp3;
                    break;
                case 4:
                    double[] temp4 = {0.114432559973540,   0.106838802436507,   0.051899182252855,   0.042431602755402,
                      -0.208355094982050,  -0.058741377932612,  -0.017284017902782,   0.109414756776846,
                       0.041872917797213,  -0.207903904900104,   0.069870156128220,  -0.420879615848955 };
                    result = temp4;
                    break;
                case 5:
                    double[] temp5 = {-0.089083516880583,  -0.212101554759966,  -0.074106774400082,   0.140475177135465,
                       0.050748404792032,   0.035466506409495,   0.135105447115893,   0.005040120014690,
                       0.034685336520958,  -0.105165286994008,  -0.049062087663207,   0.090272091944523 };
                    result = temp5;
                    break;
                case 6:
                    double[] temp6 = {-0.067103542084971,   0.079590227298170,   0.070742925683350,  -0.041636392993342,
                       0.220644176913664,  -0.170330066249381,   0.073407186641070,  -0.024528044153757,
                      -0.047531378473926,   0.078357526154640,  -0.155837767265987,  -0.238145173236917 };
                    result = temp6;
                    break;
                case 7:
                    double[] temp7 = {0.045134621164588,   0.068462421832649,   0.017700087773030,  -0.225994306022918,
                       0.088191771774388,  -0.179589945451456,   0.025986312625294,  -0.040027912330171,
                      -0.046415300400838,  -0.158947267757270,  -0.067242068899347,  -0.124325353336784 };
                    result = temp7;
                    break;
                case 8:
                    double[] temp8 = {0.104118389703866,   0.084958239636849,   0.032826888550085,  -0.140404308281075,
                      -0.017921707825125,   0.061748807252006,  -0.006715952930192,   0.012639691500341,
                       0.046309410792156,  -0.218542898428085,  -0.003731519086623,  -0.319272823668150 };
                    result = temp8;
                    break;
                case 9:
                    double[] temp9 = {0.083867364580635,   0.080541880516270,   0.112368594320036,   0.122244712046047,
                      -0.047981420650493,  -0.184024814423127,  -0.097675623527326,  -0.054026830167556,
                       0.174687241854919,  -0.252086667315104,   0.043187133234385,  -0.178080825685559 };
                    result = temp9;
                    break;
                case 10:
                    double[] temp10 = {-0.076212684578814,   0.036874608262696,  -0.006395329952186,  -0.048533020632689,
                       0.031596659443431,   0.080190089955160,   0.065893547405957,  -0.066878035481739,
                      -0.029911464522049,  -0.031068180680396,  -0.065093789262652,  -0.172359413648919 };
                    result = temp10;
                    break;
                case 11:
                    double[] temp11 = {0.019329839277928,   0.049200059518738,   0.044353143292047,   0.058542188048022,
                        0.062731113213260,  -0.028231153395039,  -0.121639863668443,  -0.066633118358829,
                        0.133914377849888,  -0.142745425008951,  -0.028743819809420,  -0.321475954225317 };
                    result = temp11;
                    break;
                case 12:
                    double[] temp12 = {-0.036279717606262,   0.053297786065231,   0.004975622946153,   0.006037301787334,
                       0.076916729330439,  -0.039059863553166,   0.022282449734369,  -0.072682985769156,
                       0.021034870244154,  -0.031357531783622,   0.048738371353602,  -0.305991758949551 };
                    result = temp12;
                    break;
                case 13:
                    double[] temp13 = {0.022459687724866,  -0.062077105256798,   0.046795345083684,  -0.281967510046409,
                      -0.111189762502934,   0.153912645890540,   0.113497567735599,  -0.236258801745200,
                       0.107476182651660,  -0.460519774903838,   0.115898799892106,  -0.013940296751586 };
                    result = temp13;
                    break;
                case 14:
                    double[] temp14 = {0.046558441422710,   0.042940673413982,  -0.045419253970332,   0.111789858589594,
                      -0.100811493207000,  -0.062191636788101,  -0.070385059467037,   0.079127029426911,   
                       0.054730589641503,   -0.120689833789384,  -0.040779188055713,  -0.135332965285551 };
                    result = temp14;
                    break;
                case 15:
                    double[] temp15 = {-0.091419543228102,   0.013318237063326,   0.015108665155053,  -0.045927518891181,
                       0.138517055483565,   0.115478666476207,   0.028512863952903,   0.154290791103919,
                      -0.019524419888084,  -0.171930546561895,  -0.301610286853745,   0.050182040765164 };
                    result = temp15;
                    break;
                default:
                    result = new double[12];
                    break;
            }

            return result;
        }

        private double[] get_magn_coefs(int index)
        {
            double[] result = new double[0];
            switch (index)
            {
                case 1:
                    double[] temp1 = { 0.028299362728460,  0.007032267012805, -0.023200541745735, -0.045213515412935,
                                      -0.000578469883817, -0.028325323775602,  0.000715600536868, -0.001895278941303,
                                      -0.004646794130663,  0.039087537663713, -0.018300888706668,  0.052442950622684 };
                    result = temp1;
                    break;
                case 2:
                    double[] temp2 = { 0.039448233982626,  0.009951083569470, -0.021302598543680, -0.022381553342052,
                                      -0.007008469952631, -0.006473130307942, -0.008196198238645,  0.001519358039019,
                                       0.000151917993501,  0.089859757858035, -0.061597535550000,  0.002810815433460 };
                    result = temp2;
                    break;
                case 3:
                    double[] temp3 = { 0.031091540838056, 0.043044634483806, -0.064899668582821, -0.025218056440666,
                                      -0.004095734923580, 0.010905999083761,  0.002629550752880, -0.003586717217649,
                                      -0.002187269382124, 0.056199172306864, -0.093523132313623,  0.042641416844719 };
                    result = temp3;
                    break;
                case 4:
                    double[] temp4 = {0.096587419313157,  0.076158626153120,  0.042671812524576, -0.027238865346898,
                                      0.009471897786253, -0.002470564246736, -0.036976886649463, -0.037333436952108,
                                      0.037355216799116,  0.248047780047615, -0.017404865090071,  0.221100251365818 };
                    result = temp4;
                    break;
                case 5:
                    double[] temp5 = { 0.053171058781627, 0.074547952525461,  0.012846307520027, -0.046289891518561,
                                      -0.017171887090777, 0.018460346427567,  0.008824098235424,  0.021625467970276,
                                      -0.017092489330800, 0.229993384731662, -0.173780891871488,  0.060741641607996 };
                    result = temp5;
                    break;
                case 6:
                    double[] temp6 = { 0.058139581705881, 0.079931661061508, -0.002864001885243, -0.035195541982848,
                                       0.000314672880480, 0.019253549148174,  0.026179418967180,  0.004510178357884,
                                      -0.024468778350392, 0.211145783181474, -0.145806073707686,  0.145807917877855 };
                    result = temp6;
                    break;
                case 7:
                    double[] temp7 = { 0.030056994825200, 0.030113479286534, -0.008337871296277, -0.039990177958099,
                                      -0.021323535429032, 0.021866336686083, -0.002656273886760,  0.009527382843519,
                                       0.007232536994917, 0.133835312997446, -0.099791067083884, -0.038343116746448 };
                    result = temp7;
                    break;
                case 8:
                    double[] temp8 = { 0.021664932302478, 0.048239244525574,  0.037264484870748, -0.031091168684767,
                                      -0.010363645890006, 0.002651872990929,  0.005575575653085,  0.004104721974544,
                                      -0.005318707369513, 0.183681390337269, -0.160421986635428,  0.015297831101545 };
                    result = temp8;
                    break;
                case 9:
                    double[] temp9 = { 0.036881959737974,  0.040950468250586,  0.006106136188219, -0.013789878998704,
                                       0.027705037917098,  0.006342192673892,  0.017595696334327, -0.025194832605796,
                                      -0.014242144004621, -0.120647322999176, -0.170624740218506,  0.053142499785697 };
                    result = temp9;
                    break;
                case 10:
                    double[] temp10 = { 0.042644757145007, 0.053077035868749, -0.016591920442825, -0.022870747342348,
                                        0.008895250187963, 0.007423175633859,  0.008233916686941, -0.004610368026744,
                                       -0.003397546936180, 0.102372484153449, -0.120443801003895,  0.135367996386840 };
                    result = temp10;
                    break;
                case 11:
                    double[] temp11 = {0, 0, 0, 0,
                                      0, 0, 0, 0,
                                      0, 0, 0, 0 };
                    result = temp11;
                    break;
                case 12:
                    double[] temp12 = { 0.039242824065257,  0.046060776035909, -0.040262758950151, -0.011305553481448,
                                       -0.007144070096403, -0.001379051919508, -0.007099540929270, -0.008926738684894,
                                        0.008821478168873,  0.150244314844445, -0.050025665184217,  0.097498485747751 };
                    result = temp12;
                    break;
                case 13:
                    double[] temp13 = { 0.078704787476603, 0.107463236347946,  0.095697570683190, -0.031468825824600,
                                        0.007693875024001, 0.000788104396258,  0.020126318596917, -0.011221493248971,
                                       -0.019858467270579, 0.225462662381028, -0.344845301674416,  0.073814870127890 };
                    result = temp13;
                    break;
                case 14:
                    double[] temp14 = { 0.085025908590983, 0.087642146240619,  0.048141724082588, -0.014139587166513,
                                       -0.027563390470391, 0.002487646065251, -0.017535840260737,  0.012101298220282,
                                        0.017375390645445, 0.031189278975972,  0.108316757491003,  0.296557742600664 };
                    result = temp14;
                    break;
                case 15:
                    double[] temp15 = { 0.037255373560827,  0.050270760213167,  0.005075035607089, -0.017802710837813,
                                        0.010570947607177,  0.014144170831781,  0.016803465046713, -0.009149046464284,
                                       -0.012953068852120, -0.087432166657643, -0.209386545091650,  0.065467263881198};
                    result = temp15;
                    break;
                default:
                    result = new double[12];
                    break;
            }
            return result;
        }

        private double[] single_correction(double[] coefs, double xdata, double ydata, double zdata)
        {
            double[] result = new double[3];
            Matrix B = new DiagonalMatrix(3, 3, 1);
            Matrix A = new DenseMatrix(3, 3);
            A.At(0, 0, coefs[0]);
            A.At(0, 1, coefs[3]);
            A.At(0, 2, coefs[4]);
            A.At(1, 0, coefs[5]);
            A.At(1, 1, coefs[1]);
            A.At(1, 2, coefs[6]);
            A.At(2, 0, coefs[7]);
            A.At(2, 1, coefs[8]);
            A.At(2, 2, coefs[2]);
            Matrix B1 = Kalman_class.Matrix_Minus(B, A);
            Matrix C = new DenseMatrix(3, 1);
            C.At(0, 0, xdata);
            C.At(1, 0, ydata);
            C.At(2, 0, zdata);
            Matrix D = new DenseMatrix(3, 1);
            D.At(0, 0, coefs[9]);
            D.At(1, 0, coefs[10]);
            D.At(2, 0, coefs[11]);
            Matrix res = new DenseMatrix(3, 1);
            res = Kalman_class.Matrix_Mult(B1, Kalman_class.Matrix_Minus(C, D));
            result[0] = res.At(0, 0);
            result[1] = res.At(1, 0);
            result[2] = res.At(2, 0);
            return result;
        }

    }


    class packet
    {
        byte[] data = new byte[4096];
        public packet(byte[] input, int length)
        {
            for (int i = 0; i < length; i++)
                data[i] = input[i];
        }
        public byte get_data(int num)
        {
            return data[num];
        }
    }
    class file
    {
        packet[] data;
        long file_num;
        public file(long input)
        {
            data = new packet[0];
            file_num = input;
        }
        public bool add(packet source)
        {
            if (source != null)
            {
                packet[] temp = data;
                data = new packet[temp.Length + 1];
                for (int i=0; i < temp.Length; i++)
                    data[i] = temp[i];
                data[temp.Length] = source;
                return true;
            }
            else
                return false;
        }
        public packet get_packet(int num)
        {
            return data[num];
        }
        public long get_id()
        {
            return file_num;
        }
        public long get_size()
        {
            return data.Length * 4096;
        }
    }
}
