# Notes

```Python
    def getCalcData(self):
        self.ser.write([0xA5, 0xA5, 0xA5, 0xA5, 0x00, 0x61, 0x00, 0x00, 0x61])
        datas = self.ser.read(18)
        packetHeaderNumber = hex(int.from_bytes(datas[0:4], "little"))
        addressNumber = hex(datas[4])
        commandTypeNumber = hex(datas[5])
        dataLengthNumber = int(datas[7] << 8 | datas[6])
        K0 = float(datas[9] << 8 | datas[8])
        B0 = float(datas[11] << 8 | datas[10])
        K1 = float(datas[13] << 8 | datas[12])
        B1 = float(datas[15] << 8 | datas[14])

        self.cl = calclidar(K0, B0, K1, B1, datas[16])

        checkCode = hex(datas[17])

        return checkCode, packetHeaderNumber, addressNumber, commandTypeNumber, dataLengthNumber
```

```Python
import serial
import matplotlib.pyplot as plt
import cmath
from YDLidar_GS2 import YDLidar_GS2 as ydlidar


# note: グラフ化して確認したい際にコメントアウト解除する。
fig = plt.figure(figsize=(8,8))
ax = fig.add_subplot(111, projection='polar')
ax.set_theta_direction(-1)
ax.set_theta_zero_location('N')
ax.set_title('lidar (exit: Key E)',fontsize=18)
plt.connect('key_press_event', lambda event: exit(1) if event.key == 'e' else None)



ydl = ydlidar(port='/dev/tty.SLAB_USBtoUART')

ydl.startlidar()

while True:
    rt = ydl.getData()
    ltheta = rt.thetas[:80]
    rtheta = rt.thetas[80:]
    
    # note: グラフ化して確認したい際にコメントアウト解除する。
    if('line' in locals()):
        line.remove()
    if('line2' in locals()):
        line2.remove()
    line = ax.scatter(ltheta, rt.distance[:80], c="red", s=5)
    line2 = ax.scatter(rtheta, rt.distance[80:], c="blue", s=5)
    ax.set_theta_offset(cmath.pi / 2)
    ax.set_theta_offset(cmath.pi / 2)
    plt.pause(0.00001)
    


ydl.ser.write([0xA5,0xA5,0xA5,0xA5,0x00,0x64,0x00,0x00,0x64])
ydl.ser.close()
```

```Python
    def dataCheck(self,datas):
        if(len(datas)!=331):
            return False

        if(int.from_bytes(datas[0:4],"little") != 0xa5a5a5a5):
            return False
        
        if(datas[330] != sum(datas[4:330]) & 0x00FF):
            return False
        
        return True
```