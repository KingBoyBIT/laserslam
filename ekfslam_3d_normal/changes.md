# 改动部分

- update函数名称更换update_tmp，文件索引没有建立成功
- `function [zp,H]= observed_jac(X, idf)`中`fpos= Nxv + idf*3 - 2;`需要改动，根据你状态量维数调整
- ekfslam_sim中XE,PE的使用在新版本被禁止将全局变量输出，更改重新赋值输出
- 注意检查function关键字是否end对齐，2017版本需要，包括子函数
- 固化随机数种子

# 3.27
- `KF_cholesky_update`中的W1进行了缩放：`W1= 0.2*PHt * SCholInv;`
- 