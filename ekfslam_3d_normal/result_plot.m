i=data.i;
x_erros=data.path(1,:)-data.true(1,:);
y_erros=data.path(2,:)-data.true(2,:);
z_erros=data.path(3,:)-data.true(3,:);
phiz_erros=pi_to_pi(data.path(6,:)-data.true(6,:));
phix_erros=pi_to_pi(data.path(4,:)-data.true(4,:));
phiy_erros=pi_to_pi(data.path(5,:)-data.true(5,:));
f_num=zeros(i,1);
x_cov=zeros(i,1);
y_cov=zeros(i,1);
z_cov=zeros(i,1);
phix_cov=zeros(i,1);
phiy_cov=zeros(i,1);
phiz_cov=zeros(i,1);
x_p_data = data.state;
for j=1:i
    P_state=sqrt(x_p_data(1,j).P);
    x_cov(j) = P_state(1);
    y_cov(j) = P_state(2);
    z_cov(j) = P_state(3);
    phix_cov(j) = P_state(4);
    phiy_cov(j) = P_state(5);
    phiz_cov(j) = P_state(6);
    f_num(j)=(length(P_state)-6);
end
t=0.25.*(1:i);

figure(3)
subplot(3,1,1)
plot(t,x_erros,'b',t,2.*x_cov,'r',t,-2.*x_cov,'r');
ylabel('XÎó²î(m)')
axis tight
grid on
subplot(3,1,2)
plot(t,y_erros,'b',t,2.*y_cov,'r',t,-2.*y_cov,'r');
ylabel('YÎó²î(m)')
axis tight
grid on
subplot(3,1,3)
plot(t,z_erros,'b',t,2.*z_cov,'r',t,-2.*z_cov,'r');
ylabel('ZÎó²î(m)')
xlabel('Ê±¼ät(s)')
axis tight
grid on

figure(4)
subplot(3,1,1)
plot(t,phix_erros*180/pi,'b',t,2.*phix_cov*180/pi,'r',t,-2.*phix_cov*180/pi,'r');
ylabel('phixÎó²î(deg)')
axis tight
grid on
subplot(3,1,2)
plot(t,phiy_erros*180/pi,'b',t,2.*phiy_cov*180/pi,'r',t,-2.*phiy_cov*180/pi,'r');
ylabel('phiyÎó²î(deg)')
axis tight
grid on
subplot(3,1,3)
plot(t,phiz_erros*180/pi,'b',t,2.*phiz_cov*180/pi,'r',t,-2.*phiz_cov*180/pi,'r');
ylabel('phizÎó²î(deg)')
axis tight
grid on
% % 
figure(5)
subplot(3,1,1)
plot(t,data.obs_noise(1,:))
ylabel('¾àÀë²Ð²î(m)')
axis([0 600 -4 4])
grid on
subplot(3,1,2)
plot(t,data.obs_noise(2,:).*180./pi)
ylabel('phi²Ð²î(rad)')
xlabel('Ê±¼ät(s)')
grid on
axis([0 600 -8 8])
subplot(3,1,3)
plot(t,data.obs_noise(3,:).*180./pi)
ylabel('theta²Ð²î(rad)')
xlabel('Ê±¼ät(s)')
grid on
axis([0 600 -8 8])
% 
figure(6)
plot(t,f_num)
ylabel('×´Ì¬Á¿ÖÐÌØÕ÷µãÊý')
xlabel('Ê±¼ät(s)')
axis tight
xx = data.state(end).x;
f_erros = [];
j = 0;
for i=1:length(da_table)
    if da_table(i)~=0
        j=j+1;
        erros = lm(:,i)-xx(6+3*da_table(i)-2:6+3*da_table(i));
        erros = norm(erros(1:2));
        if erros<=2
            f_erros = [f_erros erros];
        end
    end
end
figure(7)        
f_trace = [];
j = 0;
for i=1:length(t)
    xx = data.state(i).P;
    aa = sum(xx(7:end));
    f_trace = [f_trace aa];
end
plot(t,f_trace);
grid on
f_num=[0];

j=data.i;
for i=1:j
    xx = data.state(i).x;
    num = (length(xx)-6)/3;
    num = f_num(end)+num;
    f_num = [f_num num];
end
