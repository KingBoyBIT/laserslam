using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Windows;
using System.Windows.Controls;
using System.Windows.Data;
using System.Windows.Documents;
using System.Windows.Input;
using System.Windows.Media;
using System.Windows.Media.Imaging;
using System.Windows.Navigation;
using System.Windows.Shapes;
using MahApps.Metro.Controls;
using MahApps.Metro.Controls.Dialogs;
using System.IO.Ports;
using RadarNavApp;
using System.Threading;
using System.ComponentModel;

namespace MetroWPF
{
	/// <summary> 
	/// MainWindow.xaml 的交互逻辑
	/// </summary>
	public partial class MainWindow : MetroWindow
	{
		SerialPort sp;
		bool startrecivedata = false;
		List<byte[]> recivecmdlst = new List<byte[]>();
		List<RadarData> RadataLst = new List<RadarData>();
		RadarData rd;

		AsyncOperation asyncOperation;

		public MainWindow()
		{
			InitializeComponent();
		}
		private void MetroWindow_Loaded(object sender, RoutedEventArgs e)
		{
			string[] portnames =SerialPort.GetPortNames();
			for (int i = 0; i < portnames.Length; i++)
			{
				ListBoxItem lbi = new ListBoxItem();
				lbi.Content = portnames[i];
				SerialPortNamelistBox.Items.Add(lbi);
			}

			
		}
		private void OpenCloseSerialBtn_Click(object sender, RoutedEventArgs e)
		{
			try
			{
				if (sp ==null)
				{
					ListBoxItem lbi = ((sender as ListBox).SelectedItem as ListBoxItem);
					string portname = lbi.Content.ToString();
					sp = new SerialPort(portname, Convert.ToInt32(BaudtextBox.Text), (Parity)Convert.ToInt32(ParitytextBox.Text), Convert.ToInt32(DataBittextBox.Text), (StopBits)Convert.ToInt32(StopBittextBox.Text));
					sp.Open();
					OpenCloseSerialBtn.Content = "串口已打开";
					return;
				}
				if (sp.IsOpen == false)
				{
					sp.Open();
					OpenCloseSerialBtn.Content = "串口已打开";
				}
				else
				{
					sp.Close();
					OpenCloseSerialBtn.Content = "串口已关闭";
				}
			}
			catch
			{
				this.ShowMessageAsync(null, "错误输入");
				return;
				throw;
			}
		}

		private void SerialPortNamelistBox_SelectionChanged(object sender, SelectionChangedEventArgs e)
		{
			ListBoxItem lbi = ((sender as ListBox).SelectedItem as ListBoxItem);
			string portname = lbi.Content.ToString();
			try
			{
				sp = new SerialPort(portname, Convert.ToInt32(BaudtextBox.Text),(Parity)Convert.ToInt32(ParitytextBox.Text),Convert.ToInt32(DataBittextBox.Text),(StopBits)Convert.ToInt32(StopBittextBox.Text));
				sp.Open();
				OpenCloseSerialBtn.Content = "串口已打开";
			}
			catch
			{
				this.ShowMessageAsync(null, "打开串口失败");
				return;
				throw;
			}
			
		}

		void ShowReciveDataText(object s)
		{
			this.data_rc.AppendText(s.ToString()+ Environment.NewLine);
			this.data_rc.ScrollToEnd();
			if (data_rc.Text.Length>100000)
			{
				data_rc.Text = "";
			}
		}

		public void ReciveData()
		{
			List<byte> recivedatalst = new List<byte>();
			
			bool cmd = false;
			while (startrecivedata)
			{
				byte s = (byte)sp.ReadByte();
				if (s == 0xaa)//帧头标记
				{
					cmd = true;
					recivedatalst.Clear();
					recivedatalst.Add(s);//帧头留下
					continue;
				}
				if (cmd == true)
				{
					int len = 0;
					if (recivedatalst.Count >= 3)
					{
						len = recivedatalst[1] * 256 + recivedatalst[2];
						recivedatalst.Add(s);
						for (int i = 0; i < len - 2; i++)
						{
							s = (byte)sp.ReadByte();
							recivedatalst.Add(s);
						}

						cmd = false;
						byte[] tmpdata = recivedatalst.ToArray();
						rd = new RadarData();
						rd.BuildRadarData(tmpdata);
										
						asyncOperation.Post(new SendOrPostCallback(ShowReciveDataText), Common.bytes2HexString(ref tmpdata, tmpdata.Length));

					}
					else
					{
						recivedatalst.Add(s);
					}
				}
			}
		}

		private void StartReciveDataBtn_Click(object sender, RoutedEventArgs e)
		{
			if (sp == null)
			{
				this.ShowMessageAsync(null, "未打开串口");
				return;
			}
			if (sp.IsOpen==false)
			{
				this.ShowMessageAsync(null, "未打开串口");
				return;
			}
			//byte[] recived_data = new byte[5000];

			//sp.Read(recived_data, 0, sp.ReceivedBytesThreshold);
			asyncOperation = AsyncOperationManager.CreateOperation(null);
			ThreadStart start = new ThreadStart(ReciveData);
			Thread t1 = new Thread(start);  //创建子线程，循环刷新label1
			

			if (startrecivedata == false)
			{
				startrecivedata = true;
				t1.Start(); //线程开始
			}

			var myPoint = new Ellipse();
			myPoint.Height = 1;
			myPoint.Width = 1;
			myPoint.Margin = new Thickness(5, 5, 0, 0);
			myPoint.Stroke = new SolidColorBrush(Colors.Red);
			PaintCanvas.Children.Add(myPoint);

		}

		private void StopReciveDataBtn_Click(object sender, RoutedEventArgs e)
		{
			startrecivedata = false;
		}

		private void ClearWindowBtn_Click(object sender, RoutedEventArgs e)
		{
			data_rc.Text = "";
		}

		private void button_Click(object sender, RoutedEventArgs e)
		{
			if (rd.getSuccesss()==CmdType.Measure)
			{
				double[] distance = rd.getdis();
				for (int i = 0; i < distance.Length; i++)
				{

				}
			}
			Ellipse ell = new Ellipse();
			ell.Fill = new SolidColorBrush(Colors.Green);
			ell.Width = 10;
			ell.Height = 10;
			Canvas.SetLeft(ell, 10);
			Canvas.SetLeft(ell, 20);

			//TextBlock textBlock = new TextBlock();

			//textBlock.Text = "sssasfa";

			//textBlock.Foreground = new SolidColorBrush(Color.FromRgb(200, 0, 0));

			//Canvas.SetLeft(textBlock, 10);

			//Canvas.SetTop(textBlock, 10);

			mainPanel.Children.Add(ell);
		}
	}
}
