using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace RadarNavApp
{
	public class Common
	{
		public static string bytes2HexString(ref byte[]data,int len)
		{
			string str = "";
			if (data.Length==0||len>data.Length)
			{
				return "";
			}
			else
			{
				for (int i = 0; i < len; i++)
				{
					str += data[i].ToString("X02");
				}
				return str;
			}
		}

	}
}
