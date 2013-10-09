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
            if ((comBox.Items.Count != 0))
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
			Connect();
            infoLabel.Text = "Идет очистка";
            this.Update();
            active_com.DiscardInBuffer();
            active_com.Write("f");
            timer.Start();
			while (active_com.BytesToWrite < 6)
			{
                if (timer.ElapsedMilliseconds > 1000)
                    break;  // Временная задержка, релизный вариант
			}
            timer.Stop();
            numb = active_com.Read(buffer, 0, 6);
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
            double[] lat = new double[ticks2.Length];
            double[] lon = new double[ticks2.Length];
            double[] speed = new double[ticks2.Length];
            double[] course = new double[ticks2.Length];
            double[] time = new double[ticks2.Length];
            double[] stat = new double[ticks2.Length];
            double[] date = new double[ticks2.Length];
            byte[] buffer = new byte[2];
            byte[] buffer2 = new byte[4];
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
                            w[k, 0] = (double)BitConverter.ToInt16(buffer, 0) * 0.00053375;
                            buffer[0] = pack[11]; buffer[1] = pack[12];
                            w[k, 1] = (double)BitConverter.ToInt16(buffer, 0) * 0.00053375;
                            buffer[0] = pack[15]; buffer[1] = pack[16];
                            w[k, 2] = (double)BitConverter.ToInt16(buffer, 0) * 0.00053375;

                            buffer[0] = pack[17]; buffer[1] = pack[18];
                            m[k, 0] = -(double)BitConverter.ToInt16(buffer, 0) * 0.0000625;
                            buffer[0] = pack[19]; buffer[1] = pack[20];
                            m[k, 1] = (double)BitConverter.ToInt16(buffer, 0) * 0.0000625;//* 0.00030518;
                            buffer[0] = pack[21]; buffer[1] = pack[22];
                            m[k, 2] = -(double)BitConverter.ToInt16(buffer, 0) * 0.0000625;

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

            
            DenseVector Magn_coefs = new DenseVector(get_magn_coefs(block_index));
            DenseVector Accl_coefs = new DenseVector(get_accl_coefs(block_index));
            DenseVector Gyro_coefs = new DenseVector(12);
            Kalman_class.Parameters Parameters = new Kalman_class.Parameters(Accl_coefs, Magn_coefs, Gyro_coefs);
            Kalman_class.Sensors Sensors = new Kalman_class.Sensors(new DenseMatrix(1, 3, 0), new DenseMatrix(1, 3, 0), new DenseMatrix(1, 3, 0));
            Matrix Initia_quat = new DenseMatrix(1, 4, 0);
            Initia_quat.At(0, 0, 1);
            Kalman_class.State State = new Kalman_class.State(Math.Pow(10, 2), Math.Pow(10, 2), Math.Pow(10, -3),
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

            for (int i = 0; i < k; i++)
            {
                progressBar.Invoke(new Action(() => progressBar.Value++));

                Sensors.a.At(0, 0, a[i, 0]);
                Sensors.a.At(0, 1, a[i, 1]);
                Sensors.a.At(0, 2, a[i, 2]);

                Sensors.w.At(0, 0, w[i, 0]);
                Sensors.w.At(0, 1, w[i, 1]);
                Sensors.w.At(0, 2, w[i, 2]);

                Sensors.m.At(0, 0, m[i, 0]);
                Sensors.m.At(0, 1, m[i, 1]);
                Sensors.m.At(0, 2, m[i, 2]);

                AHRS_result = Kalman_class.AHRS_LKF_EULER(Sensors, State, Parameters);

                State = AHRS_result.Item3;
                //------------------------------------------------------------------------
                mm = single_correction(magn_c, m[i, 0], m[i, 1], m[i, 2]);
                ma = single_correction(accl_c, a[i, 0], a[i, 1], a[i, 2]);
                mw = single_correction(gyro_c, w[i, 0], w[i, 1], w[i, 2]);
                //----------------------------------------------------------------------
                mw[0] = w[i, 0];
                mw[1] = w[i, 1];
                mw[2] = w[i, 2];
                ma[0] = a[i, 0];
                ma[1] = a[i, 1];
                ma[2] = a[i, 2];
                mm[0] = m[i, 0];
                mm[1] = m[i, 1];
                mm[2] = m[i, 2];
                //----------------------------------------------------------------------
                angles[0] = (AHRS_result.Item1.At(0));
                angles[1] = (AHRS_result.Item1.At(1));
                angles[2] = (AHRS_result.Item1.At(2));

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
                            MessageBox.Show("Не получен ответ от датчика.");
                            Disconnect();
                            return;
                        }
                    }

                    numb = active_com.Read(buffer, 0, 4);
                    if (numb < 4)
                    {
                        MessageBox.Show("Не получен ответ от датчика. Этап 1.");
                        Disconnect();
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
                    MessageBox.Show("Чтение завершено.");
                    files_st = new file[files_q.Count];
                    for (int q = 0; q < files_st.Length; q++)
                    {
                        // вывод спика ненулевых записей в textBox
                        files_st[q] = (file)files_q.Dequeue();
                        fileBox.Items.Add("Запись " + files_st[q].get_id() + " - " + files_st[q].get_size() + " кб");
                    }
                }
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

        // Функция чтения данных, основанная на коде из СИ
        /*
        private void readButton_Click_c(object sender, EventArgs e)
        {
            byte[] buffer = new byte[8192];
            int numb = 0;
            char[] tempS = new char[200];
            long numfiles = 0;
            char[] log_name_c = new char[2048];
            int sumb = 0;
            long ifile = 0;
            System.Diagnostics.Stopwatch timer = new System.Diagnostics.Stopwatch();
            if ((validCOM == 0) && (selected_com != "COM0"))
            {
                Connect();
                FileStream fs = File.Create(log_name, 2048, FileOptions.None);
                StreamWriter str_wr = new StreamWriter(fs);

                infoLabel.Text = "Идет считывание";
                active_com.DiscardInBuffer();
                sumb = 0;
                active_com.Write("?");
                timer.Start();
                while (active_com.BytesToRead < 4)
                {
                    if (timer.ElapsedMilliseconds > 1000)
                    {
                        MessageBox.Show("Ошибка в передаче данных. Этап 1.");
                        break;
                    }
                }
                timer.Stop();
                numb = active_com.Read(buffer, 0, 4);
                if (numb == 4)
                    numfiles = buffer[0] * (int)Math.Pow(2, 24) + buffer[1] * (int)Math.Pow(2, 16) + buffer[2] * (int)Math.Pow(2, 8) + buffer[3];
                else
                    numfiles = 0;
                ifile = 1;
                progressBar.Maximum = (int)numfiles;
                progressBar.Value = 0;
                while (ifile < numfiles)
                {
                    progressBar.Value++;
                    active_com.DiscardInBuffer();
                    sumb = 0;
                    active_com.Write("r");
                    timer.Start();
                    while (active_com.BytesToRead < 4)
                    {
                        if (timer.ElapsedMilliseconds > 1000)
                        {
                            MessageBox.Show("Ошибка в передаче данных. Этап 2.");
                            break;
                        }
                    }
                    timer.Stop();
                    numb = active_com.Read(buffer, 0, 4);
                    if (numb == 4)
                        ifile = buffer[0] * (int)Math.Pow(2, 24) + buffer[1] * (int)Math.Pow(2, 16) + buffer[2] * (int)Math.Pow(2, 8) + buffer[3];
                    else
                        ifile = 0;
                    str_wr.WriteLine("ifile = " + ifile);
                    while (sumb != 4096 * 255)
                    {
                        active_com.DiscardInBuffer();
                        sumb = 0;
                        active_com.Write(" ");
                        timer.Start();
                        while (active_com.BytesToRead < 4096)
                        {
                            if (timer.ElapsedMilliseconds > 1000)
                            {
                                MessageBox.Show("Ошибка в передаче данных. Этап 3.");
                                break;
                            }
                        }
                        timer.Stop();
                        numb = active_com.Read(buffer, 0, 4096);
                        for (int q = 0; q < numb; q++)
                            sumb += buffer[q];
                        str_wr.Write(sumb + " ");
                        if (numb < 4096)
                        {
                            MessageBox.Show("Ошибка в передаче данных. Этап 4.");
                            break;
                        }
                    }
                    str_wr.WriteLine();
                }
 
                str_wr.Flush();
                str_wr.Close();
            }
            if (validCOM == 1)
            {
                Disconnect();
                MessageBox.Show("Чтение завершено");
            }
        }
*/

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
                    double[] temp14 = {0.030554866588497,   0.041807742536512,   0.014637539550826,   0.070444712524782,
                      -0.045520819149987,  -0.053792609796159,   0.067583771817011,   0.044201999575329,
                      -0.068058903538097,  -0.133268083186438,  -0.157601990534356,  -0.125072062660633 };
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
                    double[] temp1 = {-1.3000, -1.3499, -1.2180, -0.5191, 1.2598, 0.8968,
                                         1.1259, -0.8541, -1.4552, -0.1676, 0.0201, 0.1032 };
                    result = temp1;
                    break;
                case 2:
                    double[] temp2 = {-1.021584893111037,  -1.156822840186938,  -1.059527431706735,   0.405595881908856,
                       0.954589891425432,  -0.069541017965736,   0.991639731762399,  -1.012866646546024,
                      -0.886632983175849,  -0.065839849453693,   0.059178141113860,   0.229560149816208 };
                    result = temp2;
                    break;
                case 3:
                    double[] temp3 = {0.238481805936342,  -0.611518102386768,  -0.521946604623734,  -0.152499267950600,
                       0.380041463141051,  -0.089921812966795,  -0.029707070918285,   0.777735008725552,
                      -0.263038218323014,   1.102625052444242,  -0.174805428028501,   0.764500679478734 };
                    result = temp3;
                    break;
                case 4:
                    double[] temp4 = {-0.912310601920176,  -0.035122859804258,  -0.980824053980284,  -0.900924288276854,
                      -0.392141063030012,  -0.212147078216013,   0.370184540925128,  -0.138611784797257,
                       0.592181216595855,  -0.716249303213732,   0.743440070172089,   0.417833919696198 };
                    result = temp4;
                    break;
                case 5:
                    double[] temp5 = {-0.810295610308645,  -0.669847770840476,   0.029900885597606,  -0.314087876668212,
                        -0.489991754352551,  -0.196143690771996,   0.785352181879690,  -0.148859193029027,
                        0.475841206204676,  -0.674462559992835,   0.797605908162643,   1.039652155282594 };
                    result = temp5;
                    break;
                case 6:
                    double[] temp6 = {-0.170985488990754,  -0.564445824510332,  -0.156834833150726,  -0.308524525175852,
                       0.442976518336103,  -0.358382840398978,  -0.080578225278669,   0.617372103949350,
                      -0.541560455196209,   0.718141250811105,  -0.546677511918727,   1.013917065538093 };
                    result = temp6;
                    break;
                case 7:
                    double[] temp7 = {0.240929547813333,  -0.055331307635026,   0.080962491207260,   0.111673173134500,
                      -0.437371531722260,   0.086278316536676,  -0.156009284276460,  -0.450383773465459,
                      -0.232586543173110,   1.800622790457513,   0.479391322929313,  -1.645807763696289 };
                    result = temp7;
                    break;
                case 8:
                    double[] temp8 = {-0.657211729785750,  -0.751518105983042,  -0.502998987070861,   0.346979570434948,
                      -0.626698848495405,   0.854645387115907,  -0.722084569035857,  -0.328055065021069,
                      -0.229782299399703,  -0.357543764086528,  -0.658127285343566,   0.346576778962207 };
                    result = temp8;
                    break;
                case 9:
                    double[] temp9 = {-0.626836215836308,  -0.013759850146307,  -0.583068885503070,   0.859006753692338,
                      -0.291104097898408,   0.359496708414707,  -0.377515731153946,  -0.003959749268316,
                      -0.269023628503547,  -0.839470996073222,  -0.734894289704346,   0.173586855759935 };
                    result = temp9;
                    break;
                case 10:
                    double[] temp10 = {-0.369311287771329,  -0.619181805694075,   0.215860213252399,  -0.202174116935395,
                      -0.469592802232751,  -0.047691482924571,   0.275440905803689,  -0.360029056603724,
                       0.184994296407090,  -0.770865355323793,   0.374651486398312,   1.142034879854652 };
                    result = temp10;
                    break;
                case 11:
                    double[] temp11 = {0.608714881216204,   0.726988256983956,   0.548780040935389,   0.016242466383838,
                       0.008309177330682,  -0.358605622379460,  -0.406898271841329,  -0.176070107760827,
                       0.023260260319988,  -0.615605787372553,                   0,  -0.736861602878822 };
                    result = temp11;
                    break;
                case 12:
                    double[] temp12 = {0.174793461009216,  -0.040792970147182,   0.416590023305863,   0.023815283418015,
                      -0.400898824923381,  -0.056977093394991,  -0.137939234650323,  -0.275073547098045,
                      -0.226813306028076,   1.386816689877866,   0.503695987968843,  -1.311523063280292 };
                    result = temp12;
                    break;
                case 13:
                    double[] temp13 = {-0.363402668053064,  -1.094360523138415,  -0.337177750046436,  -0.150554901049520,
                       0.760814281494252,  -0.527695809751047,  -0.366683816407952,   0.694017973652759,
                      -0.257690691379724,   0.940590498718544,  -0.603082996186667,   0.915339225862064 };
                    result = temp13;
                    break;
                case 14:
                    double[] temp14 = {0.239432068058017,  -0.725844731558349,  -1.151204001242439,   0.549384999741005,
                       0.017479681993509,   0.446775656927678,  -0.105826078256205,  -0.233477063739925,
                      -0.014288927249738,   1.072218089476175,   0.355024708548398,  -0.194902780695747 };
                    result = temp14;
                    break;
                case 15:
                    double[] temp15 = {0.106974544824006,  -0.921067620149456,  -0.544783980541099,  -0.113463751891383,
                       0.575096563967160,  -0.215872226257520,  -0.106232440232434,   0.665018879611869,
                      -0.120415653314238,   0.975928019145445,  -0.232693130860269,   0.505738827768481 };
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
