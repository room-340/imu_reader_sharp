namespace imu_reader_sharp
{
    partial class Form1
    {
        /// <summary>
        /// Required designer variable.
        /// </summary>
        private System.ComponentModel.IContainer components = null;

        /// <summary>
        /// Clean up any resources being used.
        /// </summary>
        /// <param name="disposing">true if managed resources should be disposed; otherwise, false.</param>
        protected override void Dispose(bool disposing)
        {
            if (disposing && (components != null))
            {
                components.Dispose();
            }
            base.Dispose(disposing);
        }

        #region Windows Form Designer generated code

        /// <summary>
        /// Required method for Designer support - do not modify
        /// the contents of this method with the code editor.
        /// </summary>
        private void InitializeComponent()
        {
            this.fileButton = new System.Windows.Forms.Button();
            this.readButton = new System.Windows.Forms.Button();
            this.clearButton = new System.Windows.Forms.Button();
            this.comBox = new System.Windows.Forms.ListBox();
            this.progressBar = new System.Windows.Forms.ProgressBar();
            this.fileLabel = new System.Windows.Forms.Label();
            this.infoLabel = new System.Windows.Forms.Label();
            this.fileInfoLabel = new System.Windows.Forms.Label();
            this.comLabel = new System.Windows.Forms.Label();
            this.saveFileDialog = new System.Windows.Forms.SaveFileDialog();
            this.fileBox = new System.Windows.Forms.ListBox();
            this.openFileDialog = new System.Windows.Forms.OpenFileDialog();
            this.packetBox = new System.Windows.Forms.TextBox();
            this.label1 = new System.Windows.Forms.Label();
            this.saveButton = new System.Windows.Forms.Button();
            this.filePickLabel = new System.Windows.Forms.Label();
            this.SuspendLayout();
            // 
            // fileButton
            // 
            this.fileButton.Enabled = false;
            this.fileButton.Font = new System.Drawing.Font("Microsoft Sans Serif", 10F, System.Drawing.FontStyle.Bold, System.Drawing.GraphicsUnit.Point, ((byte)(204)));
            this.fileButton.Location = new System.Drawing.Point(12, 12);
            this.fileButton.Name = "fileButton";
            this.fileButton.Size = new System.Drawing.Size(156, 46);
            this.fileButton.TabIndex = 0;
            this.fileButton.Text = "Выбрать файл для сохранения";
            this.fileButton.UseVisualStyleBackColor = true;
            this.fileButton.Click += new System.EventHandler(this.fileButton_Click);
            // 
            // readButton
            // 
            this.readButton.Font = new System.Drawing.Font("Microsoft Sans Serif", 10F, System.Drawing.FontStyle.Bold, System.Drawing.GraphicsUnit.Point, ((byte)(204)));
            this.readButton.Location = new System.Drawing.Point(390, 106);
            this.readButton.Name = "readButton";
            this.readButton.Size = new System.Drawing.Size(130, 42);
            this.readButton.TabIndex = 1;
            this.readButton.Text = "Считать данные";
            this.readButton.UseVisualStyleBackColor = true;
            this.readButton.Click += new System.EventHandler(this.readButton_Click);
            // 
            // clearButton
            // 
            this.clearButton.Font = new System.Drawing.Font("Microsoft Sans Serif", 10F, System.Drawing.FontStyle.Bold, System.Drawing.GraphicsUnit.Point, ((byte)(204)));
            this.clearButton.Location = new System.Drawing.Point(390, 299);
            this.clearButton.Name = "clearButton";
            this.clearButton.Size = new System.Drawing.Size(130, 39);
            this.clearButton.TabIndex = 2;
            this.clearButton.Text = "Очистить";
            this.clearButton.UseVisualStyleBackColor = true;
            this.clearButton.Click += new System.EventHandler(this.clearButton_Click);
            // 
            // comBox
            // 
            this.comBox.Font = new System.Drawing.Font("Microsoft Sans Serif", 10F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(204)));
            this.comBox.FormattingEnabled = true;
            this.comBox.ItemHeight = 16;
            this.comBox.Location = new System.Drawing.Point(12, 107);
            this.comBox.Name = "comBox";
            this.comBox.Size = new System.Drawing.Size(156, 116);
            this.comBox.TabIndex = 3;
            this.comBox.Click += new System.EventHandler(this.comBox_Click);
            this.comBox.SelectedIndexChanged += new System.EventHandler(this.check_read_available);
            // 
            // progressBar
            // 
            this.progressBar.Location = new System.Drawing.Point(77, 279);
            this.progressBar.Name = "progressBar";
            this.progressBar.Size = new System.Drawing.Size(250, 33);
            this.progressBar.TabIndex = 4;
            // 
            // fileLabel
            // 
            this.fileLabel.BorderStyle = System.Windows.Forms.BorderStyle.FixedSingle;
            this.fileLabel.Font = new System.Drawing.Font("Microsoft Sans Serif", 10F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(204)));
            this.fileLabel.Location = new System.Drawing.Point(195, 35);
            this.fileLabel.Name = "fileLabel";
            this.fileLabel.Size = new System.Drawing.Size(286, 23);
            this.fileLabel.TabIndex = 5;
            this.fileLabel.Text = "Файл";
            this.fileLabel.TextAlign = System.Drawing.ContentAlignment.MiddleCenter;
            this.fileLabel.TextChanged += new System.EventHandler(this.check_read_available);
            // 
            // infoLabel
            // 
            this.infoLabel.Font = new System.Drawing.Font("Microsoft Sans Serif", 10F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(204)));
            this.infoLabel.Location = new System.Drawing.Point(53, 240);
            this.infoLabel.Name = "infoLabel";
            this.infoLabel.Size = new System.Drawing.Size(298, 36);
            this.infoLabel.TabIndex = 6;
            this.infoLabel.Text = "Текущее действие";
            this.infoLabel.TextAlign = System.Drawing.ContentAlignment.BottomCenter;
            // 
            // fileInfoLabel
            // 
            this.fileInfoLabel.Font = new System.Drawing.Font("Microsoft Sans Serif", 10F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(204)));
            this.fileInfoLabel.Location = new System.Drawing.Point(195, 12);
            this.fileInfoLabel.Name = "fileInfoLabel";
            this.fileInfoLabel.Size = new System.Drawing.Size(286, 23);
            this.fileInfoLabel.TabIndex = 7;
            this.fileInfoLabel.Text = "Запись будет сохранена как:";
            this.fileInfoLabel.TextAlign = System.Drawing.ContentAlignment.MiddleCenter;
            // 
            // comLabel
            // 
            this.comLabel.Font = new System.Drawing.Font("Microsoft Sans Serif", 10F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(204)));
            this.comLabel.Location = new System.Drawing.Point(12, 83);
            this.comLabel.Name = "comLabel";
            this.comLabel.Size = new System.Drawing.Size(156, 21);
            this.comLabel.TabIndex = 8;
            this.comLabel.Text = "Выберите СОМ порт";
            this.comLabel.TextAlign = System.Drawing.ContentAlignment.MiddleCenter;
            // 
            // fileBox
            // 
            this.fileBox.Font = new System.Drawing.Font("Microsoft Sans Serif", 10F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(204)));
            this.fileBox.FormattingEnabled = true;
            this.fileBox.ItemHeight = 16;
            this.fileBox.Location = new System.Drawing.Point(195, 107);
            this.fileBox.Name = "fileBox";
            this.fileBox.Size = new System.Drawing.Size(156, 116);
            this.fileBox.TabIndex = 9;
            this.fileBox.SelectedIndexChanged += new System.EventHandler(this.check_save_available);
            // 
            // openFileDialog
            // 
            this.openFileDialog.FileName = "openFileDialog";
            this.openFileDialog.RestoreDirectory = true;
            // 
            // packetBox
            // 
            this.packetBox.Font = new System.Drawing.Font("Microsoft Sans Serif", 10F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(204)));
            this.packetBox.Location = new System.Drawing.Point(236, 316);
            this.packetBox.Name = "packetBox";
            this.packetBox.Size = new System.Drawing.Size(60, 23);
            this.packetBox.TabIndex = 10;
            // 
            // label1
            // 
            this.label1.Font = new System.Drawing.Font("Microsoft Sans Serif", 10F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(204)));
            this.label1.Location = new System.Drawing.Point(101, 319);
            this.label1.Name = "label1";
            this.label1.Size = new System.Drawing.Size(129, 23);
            this.label1.TabIndex = 11;
            this.label1.Text = "Считано пакетов";
            // 
            // saveButton
            // 
            this.saveButton.Enabled = false;
            this.saveButton.Font = new System.Drawing.Font("Microsoft Sans Serif", 10F, System.Drawing.FontStyle.Bold, System.Drawing.GraphicsUnit.Point, ((byte)(204)));
            this.saveButton.Location = new System.Drawing.Point(390, 174);
            this.saveButton.Name = "saveButton";
            this.saveButton.Size = new System.Drawing.Size(130, 49);
            this.saveButton.TabIndex = 12;
            this.saveButton.Text = "Сохранить запись";
            this.saveButton.UseVisualStyleBackColor = true;
            this.saveButton.Click += new System.EventHandler(this.saveButton_Click);
            // 
            // filePickLabel
            // 
            this.filePickLabel.Font = new System.Drawing.Font("Microsoft Sans Serif", 10F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(204)));
            this.filePickLabel.Location = new System.Drawing.Point(195, 83);
            this.filePickLabel.Name = "filePickLabel";
            this.filePickLabel.Size = new System.Drawing.Size(156, 21);
            this.filePickLabel.TabIndex = 13;
            this.filePickLabel.Text = "Выберите запись";
            this.filePickLabel.TextAlign = System.Drawing.ContentAlignment.MiddleCenter;
            // 
            // Form1
            // 
            this.AutoScaleDimensions = new System.Drawing.SizeF(6F, 13F);
            this.AutoScaleMode = System.Windows.Forms.AutoScaleMode.Font;
            this.ClientSize = new System.Drawing.Size(532, 350);
            this.Controls.Add(this.filePickLabel);
            this.Controls.Add(this.saveButton);
            this.Controls.Add(this.label1);
            this.Controls.Add(this.packetBox);
            this.Controls.Add(this.fileBox);
            this.Controls.Add(this.comLabel);
            this.Controls.Add(this.fileInfoLabel);
            this.Controls.Add(this.infoLabel);
            this.Controls.Add(this.fileLabel);
            this.Controls.Add(this.progressBar);
            this.Controls.Add(this.comBox);
            this.Controls.Add(this.clearButton);
            this.Controls.Add(this.readButton);
            this.Controls.Add(this.fileButton);
            this.Name = "Form1";
            this.Text = "IMU Reader Sharp";
            this.FormClosing += new System.Windows.Forms.FormClosingEventHandler(this.Form1_FormClosing);
            this.Load += new System.EventHandler(this.Form1_Load);
            this.ResumeLayout(false);
            this.PerformLayout();

        }

        #endregion

        private System.Windows.Forms.Button fileButton;
        private System.Windows.Forms.Button readButton;
        private System.Windows.Forms.Button clearButton;
        private System.Windows.Forms.ListBox comBox;
        private System.Windows.Forms.ProgressBar progressBar;
        private System.Windows.Forms.Label fileLabel;
        private System.Windows.Forms.Label infoLabel;
        private System.Windows.Forms.Label fileInfoLabel;
        private System.Windows.Forms.Label comLabel;
        private System.Windows.Forms.SaveFileDialog saveFileDialog;
        private System.Windows.Forms.ListBox fileBox;
        private System.Windows.Forms.OpenFileDialog openFileDialog;
        private System.Windows.Forms.TextBox packetBox;
        private System.Windows.Forms.Label label1;
        private System.Windows.Forms.Button saveButton;
        private System.Windows.Forms.Label filePickLabel;
    }
}

