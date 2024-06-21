# DJIMotorCtrlESP
A library for controlling DJI M3508, M2006, and GM6020 motors using ESP32
控制大疆电机从未如此简单
## Usage
    //引入头文件
    #include <DJIMotorCtrlESP.h>

    //初始化can总线,要使用电机，必须先调用此函数初始化can通信,并且开发板有can收发芯片
    //建议在setup函数中调用
    can_init(8, 18, 100); //第一个参数是TX引脚，第二个参数是RX引脚，第三个参数是电流更新频率

    //创建一个电机对象
    M3508_P19 moto(1); //构造函数中设置ID为1

    //执行你的操作
    M3508_P19.set_speed(-100);//比如让电机以-100RPM旋转

