clearvars -except s
clc,close all
fclose(instrfind)% 关闭正在占用的串口

baudrate = 115200;
portnames = seriallist;
for i = 1:length(portnames)
	if portnames(i) ~= "COM1"
		portname = portnames(i);
		break
	end
end
% 创建串口OBJ
sport=serial(portname,'BaudRate',baudrate);
% s.status %串口状态
fopen(sport)%打开串口
% fwrite(s,255)%发送，用不到
cmd = [];

% 接收200条
% for i = 1:1
while(1)
	while(1)
		ch = fread(sport,1,'uchar');% 接收帧头1字节
		if ch == 170%0xaa
			cmd = [cmd;ch];
			ch = fread(sport,2,'uchar');% 接收帧长
			cmd = [cmd;ch];
			ch = fread(sport,ch(1)*256+ch(2)-1,'uchar');%接收整个帧剩余部分
			cmd = [cmd;ch];
			% CRC校验暂时pass
			break;
		end
	end
	%解析帧
	if length(cmd)~=11
		data = CMDdec(cmd);
		fprintf("转速：%f\n",data.speed);
		fprintf("0偏：%f\n",data.zerobias)
		fprintf("初始角偏：%f\n",data.iniradangle)
		for i = 1:length(data.strength)
			fprintf("第%d个信号强度：%f\n",i,data.strength(i))
			fprintf("第%d个实际距离：%f mm\n",i,data.dis(i))
		end
	else
		cmd = [];
		continue;
	end
	cmd = [];
end
%% CLOSE
fclose(sport)%关闭串口2
fclose(instrfind)

