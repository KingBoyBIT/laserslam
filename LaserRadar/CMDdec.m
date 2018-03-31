function [data] = CMDdec(cmd)
errcode = 1;

if cmd(1)==hex2dec('aa')
	if cmd(4) == hex2dec('00')
		if cmd(5) == hex2dec('61')
			len = cmd(7)*256+cmd(8);
			if mod(len-5,3)~=0
				error('数据段长度错误')
			end
			ct = 9;
			switch cmd(6)
				case hex2dec('ad')%正常帧
					data.speed = cmd(ct)*0.05;ct = ct + 1;
					a = (cmd(ct))*256 + cmd(ct + 1);a(a>32768)=32768-a(a>32768);
					data.zerobias=a;ct = ct + 2;
					data.iniradangle = cmd(ct)*256 + cmd(ct + 1);ct = ct + 2;
					data.dis = zeros((len-5)/3,1);
					data.strength = zeros((len-5)/3,1);
					tt = 1;
					for i = 0:3:(len-5-3)
						data.strength(tt) = cmd(ct+i);
						data.dis(tt) = (cmd(ct+i+1)*256+cmd(ct+i+2))*0.25;
						tt = tt + 1;
					end
					
				case hex2dec('ae')%运转错误
			end
			errcode = 0;
		end
	end
end
if errcode == 1
	error('激光雷达收到的数据错误！！！')
end
end