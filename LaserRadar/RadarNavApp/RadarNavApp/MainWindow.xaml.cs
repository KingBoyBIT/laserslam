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

namespace MetroWPF
{
	/// <summary> 
	/// MainWindow.xaml 的交互逻辑
	/// </summary>
	public partial class MainWindow : MetroWindow
	{
		SerialPort sp;
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
			
			//this.ShowMessageAsync("ok！", "好的");
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
	}
}
