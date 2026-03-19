// ==================== 包含库 ====================
#include <Arduino.h>
#include <Wire.h>
#include <ICM42688.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

// ==================== OLED设置 ====================
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_ADDRESS 0x3C  // OLED地址
#define OLED_RESET -1

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// ==================== IMU设置 ====================
ICM42688 IMU(Wire, 0x68);  // ICM42688地址

// ==================== 四元数结构体 ====================
typedef struct {
    float w, x, y, z;
} Quaternion;

Quaternion q = {1.0f, 0.0f, 0.0f, 0.0f};  // 初始姿态
float beta = 0.05f;  // Madgwick滤波器参数

// ==================== 低通滤波器 ====================
typedef struct {
    float alpha;
    float last_value;
} LowPassFilter;

LowPassFilter accelFilter[3] = {{0.3f, 0}, {0.3f, 0}, {0.3f, 0}};
LowPassFilter gyroFilter[3] = {{0.3f, 0}, {0.3f, 0}, {0.3f, 0}};

// ==================== 3D可视化参数 ====================
#define CUBE_SIZE 10        // 立方体大小
#define CENTER_X 64         // 屏幕中心X
#define CENTER_Y 35         // 屏幕中心Y
#define SCALE_FACTOR 15.0f  // 3D缩放因子
#define AXIS_LENGTH 1.5f    // 轴长度（相对于立方体）

// 立方体顶点（标准化坐标）
float cubeVertices[8][3] = {
    {-1.0f, -1.0f, -1.0f},  // 0: 左前下
    { 1.0f, -1.0f, -1.0f},  // 1: 右前下
    { 1.0f,  1.0f, -1.0f},  // 2: 右后下
    {-1.0f,  1.0f, -1.0f},  // 3: 左后下
    {-1.0f, -1.0f,  1.0f},  // 4: 左前上
    { 1.0f, -1.0f,  1.0f},  // 5: 右前上
    { 1.0f,  1.0f,  1.0f},  // 6: 右后上
    {-1.0f,  1.0f,  1.0f}   // 7: 左后上
};

// 立方体边连接
int cubeEdges[12][2] = {
    {0, 1}, {1, 2}, {2, 3}, {3, 0},  // 底面
    {4, 5}, {5, 6}, {6, 7}, {7, 4},  // 顶面
    {0, 4}, {1, 5}, {2, 6}, {3, 7}   // 侧面
};

// 坐标轴定义（从立方体中心出发的向量）
float axisVectors[3][3] = {
    {2.0f, 0.0f, 0.0f},  // X轴：向右
    {0.0f, 2.0f, 0.0f},  // Y轴：向上
    {0.0f, 0.0f, 2.0f}   // Z轴：向前
};

// 坐标轴颜色（OLED只有白色，但可以用线型区分）
#define AXIS_X 0  // X轴索引
#define AXIS_Y 1  // Y轴索引
#define AXIS_Z 2  // Z轴索引

// ==================== 采样率控制 ====================
#define SAMPLE_RATE 100  // Hz
unsigned long lastIMUUpdate = 0;
unsigned long lastDisplayUpdate = 0;

// ==================== 函数声明 ====================
void setupOLED();
void setupIMU();
void readIMUData(float* accel, float* gyro);
float lowPassFilter(LowPassFilter* filter, float input);
void madgwickUpdate(float* accel, float* gyro);
void normalizeQuaternion(Quaternion* q);
Quaternion quaternionMultiply(Quaternion a, Quaternion b);
Quaternion quaternionConjugate(Quaternion q);
void rotatePoint(float* point);
void draw3DCubeWithAxes();
void drawAxisArrow(float* start, float* end, int axisType);
void displayAttitudeInfo();

// ==================== 初始化 ====================
void setup() {
    Serial.begin(115200);
    while (!Serial) delay(10);
    
    Serial.println("========================================");
    Serial.println("    IMU 3D姿态可视化系统");
    Serial.println("    带坐标轴旋转的立方体");
    Serial.println("========================================");
    
    // 初始化I2C
    Wire.begin();
    delay(100);
    
    // 初始化OLED
    setupOLED();
    
    // 初始化IMU
    setupIMU();
    
    Serial.println("系统初始化完成，开始3D可视化...");
}

// ==================== OLED初始化 ====================
void setupOLED() {
    Serial.print("初始化OLED...");
    
    if(!display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDRESS)) {
        Serial.println("失败!");
        Serial.println("请检查：");
        Serial.println("1. OLED是否正确连接");
        Serial.println("2. I2C地址是否正确");
        while(1);
    }
    
    Serial.println("成功!");
    
    // 清空屏幕
    display.clearDisplay();
    display.setTextColor(SSD1306_WHITE);
    
    // 显示启动画面
    display.setTextSize(2);
    display.setCursor(15, 10);
    display.println("IMU 3D");
    display.setTextSize(1);
    display.setCursor(25, 35);
    display.println("With Axes");
    display.display();
    delay(2000);
    
    // 清空屏幕准备主显示
    display.clearDisplay();
    display.display();
}

// ==================== IMU初始化 ====================
void setupIMU() {
    Serial.print("初始化ICM42688-P...");
    
    int status = IMU.begin();
    if (status < 0) {
        Serial.println("失败!");
        display.clearDisplay();
        display.setTextSize(1);
        display.setCursor(0, 0);
        display.println("IMU Init Fail");
        display.display();
        while(1);
    }
    
    Serial.println("成功!");
    
    // 配置传感器参数
    IMU.setAccelFS(ICM42688::gpm16);
    IMU.setGyroFS(ICM42688::dps2000);
    IMU.setAccelODR(ICM42688::odr100);
    IMU.setGyroODR(ICM42688::odr100);
    
    Serial.println("传感器配置完成");
    
    display.clearDisplay();
    display.setTextSize(1);
    display.setCursor(0, 0);
    display.println("IMU Ready");
    display.println("100Hz Sampling");
    display.display();
    delay(1000);
}

// ==================== 低通滤波函数 ====================
float lowPassFilter(LowPassFilter* filter, float input) {
    filter->last_value = filter->alpha * input + (1.0f - filter->alpha) * filter->last_value;
    return filter->last_value;
}

// ==================== 四元数运算函数 ====================
void normalizeQuaternion(Quaternion* q) {
    float norm = sqrt(q->w * q->w + q->x * q->x + q->y * q->y + q->z * q->z);
    if (norm > 0.0f) {
        q->w /= norm;
        q->x /= norm;
        q->y /= norm;
        q->z /= norm;
    }
}

Quaternion quaternionMultiply(Quaternion a, Quaternion b) {
    Quaternion result;
    result.w = a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z;
    result.x = a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y;
    result.y = a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x;
    result.z = a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w;
    return result;
}

Quaternion quaternionConjugate(Quaternion q) {
    Quaternion result = {q.w, -q.x, -q.y, -q.z};
    return result;
}

// ==================== Madgwick姿态融合算法 ====================
void madgwickUpdate(float* accel, float* gyro) {
    // 归一化加速度计数据
    float ax = accel[0], ay = accel[1], az = accel[2];
    float accelNorm = sqrt(ax*ax + ay*ay + az*az);
    if (accelNorm > 0.0f) {
        ax /= accelNorm;
        ay /= accelNorm;
        az /= accelNorm;
    }
    
    float qw = q.w, qx = q.x, qy = q.y, qz = q.z;
    
    // 计算加速度计误差梯度
    float gradient[4];
    gradient[0] = 2.0f * (qx * qz - qw * qy) - ax;
    gradient[1] = 2.0f * (qw * qx + qy * qz) - ay;
    gradient[2] = 2.0f * (0.5f - qx*qx - qy*qy) - az;
    
    // 计算雅可比矩阵
    float jacobian[12] = {
        -2.0f*qy, 2.0f*qz, -2.0f*qw, 2.0f*qx,
        2.0f*qx, 2.0f*qw, 2.0f*qz, 2.0f*qy,
        0.0f, -4.0f*qx, -4.0f*qy, 0.0f
    };
    
    // 计算梯度下降方向
    float nabla_f[4] = {0.0f};
    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 3; j++) {
            nabla_f[i] += jacobian[j*4 + i] * gradient[j];
        }
    }
    
    // 归一化梯度
    float gradientNorm = sqrt(nabla_f[0]*nabla_f[0] + nabla_f[1]*nabla_f[1] + 
                            nabla_f[2]*nabla_f[2] + nabla_f[3]*nabla_f[3]);
    if (gradientNorm > 0.0f) {
        nabla_f[0] /= gradientNorm;
        nabla_f[1] /= gradientNorm;
        nabla_f[2] /= gradientNorm;
        nabla_f[3] /= gradientNorm;
    }
    
    // 陀螺仪积分 + 梯度下降校正
    Quaternion q_dot;
    q_dot.w = 0.5f * (-qx * gyro[0] - qy * gyro[1] - qz * gyro[2]) - beta * nabla_f[0];
    q_dot.x = 0.5f * (qw * gyro[0] + qy * gyro[2] - qz * gyro[1]) - beta * nabla_f[1];
    q_dot.y = 0.5f * (qw * gyro[1] - qx * gyro[2] + qz * gyro[0]) - beta * nabla_f[2];
    q_dot.z = 0.5f * (qw * gyro[2] + qx * gyro[1] - qy * gyro[0]) - beta * nabla_f[3];
    
    // 四元数积分更新
    float dt = 1.0f / SAMPLE_RATE;
    q.w += q_dot.w * dt;
    q.x += q_dot.x * dt;
    q.y += q_dot.y * dt;
    q.z += q_dot.z * dt;
    
    // 归一化四元数
    normalizeQuaternion(&q);
}

// ==================== 读取IMU数据 ====================
void readIMUData(float* accel, float* gyro) {
    IMU.getAGT();
    
    accel[0] = IMU.accX();
    accel[1] = IMU.accY();
    accel[2] = IMU.accZ();
    
    gyro[0] = IMU.gyrX() * DEG_TO_RAD;
    gyro[1] = IMU.gyrY() * DEG_TO_RAD;
    gyro[2] = IMU.gyrZ() * DEG_TO_RAD;
    
    for (int i = 0; i < 3; i++) {
        accel[i] = lowPassFilter(&accelFilter[i], accel[i]);
        gyro[i] = lowPassFilter(&gyroFilter[i], gyro[i]);
    }
}

// ==================== 顶点旋转函数 ====================
void rotatePoint(float* point) {
    Quaternion p = {0.0f, point[0], point[1], point[2]};
    Quaternion q_conj = quaternionConjugate(q);
    Quaternion temp = quaternionMultiply(q, p);
    Quaternion p_rotated = quaternionMultiply(temp, q_conj);
    
    point[0] = p_rotated.x;
    point[1] = p_rotated.y;
    point[2] = p_rotated.z;
}

// ==================== 绘制坐标轴箭头 ====================
void drawAxisArrow(float* start, float* end, int axisType) {
    int x1 = CENTER_X + start[0] * SCALE_FACTOR;
    int y1 = CENTER_Y + start[1] * SCALE_FACTOR;
    int x2 = CENTER_X + end[0] * SCALE_FACTOR;
    int y2 = CENTER_Y + end[1] * SCALE_FACTOR;
    
    // 绘制轴线
    display.drawLine(x1, y1, x2, y2, SSD1306_WHITE);
    
    // 绘制箭头头部（小三角形）
    float dx = end[0] - start[0];
    float dy = end[1] - start[1];
    float length = sqrt(dx*dx + dy*dy);
    
    if (length > 0) {
        // 箭头方向
        float dirX = dx / length;
        float dirY = dy / length;
        
        // 箭头头部长度
        float arrowLength = 0.3f;
        
        // 计算箭头两个支点的位置
        float perpX = -dirY;  // 垂直方向
        float perpY = dirX;
        
        // 箭头支点1
        float arrowX1 = end[0] - dirX * arrowLength + perpX * 0.15f;
        float arrowY1 = end[1] - dirY * arrowLength + perpY * 0.15f;
        
        // 箭头支点2
        float arrowX2 = end[0] - dirX * arrowLength - perpX * 0.15f;
        float arrowY2 = end[1] - dirY * arrowLength - perpY * 0.15f;
        
        // 绘制箭头
        int ax1 = CENTER_X + arrowX1 * SCALE_FACTOR;
        int ay1 = CENTER_Y + arrowY1 * SCALE_FACTOR;
        int ax2 = CENTER_X + arrowX2 * SCALE_FACTOR;
        int ay2 = CENTER_Y + arrowY2 * SCALE_FACTOR;
        int ex = x2;
        int ey = y2;
        
        display.drawLine(ax1, ay1, ex, ey, SSD1306_WHITE);
        display.drawLine(ax2, ay2, ex, ey, SSD1306_WHITE);
        
        // 在箭头末端显示轴标签
        int labelX = x2 + (dirX > 0 ? 3 : -8);
        int labelY = y2 + (dirY > 0 ? 3 : -3);
        
        switch(axisType) {
            case AXIS_X:
                display.setCursor(labelX, labelY);
                display.print("X");
                break;
            case AXIS_Y:
                display.setCursor(labelX, labelY);
                display.print("Y");
                break;
            case AXIS_Z:
                display.setCursor(labelX, labelY);
                display.print("Z");
                break;
        }
    }
}

// ==================== 绘制带坐标轴的3D立方体 ====================
void draw3DCubeWithAxes() {
    display.clearDisplay();
    
    // 1. 绘制标题
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(0, 0);
    display.println("3D Cube with Axes");
    display.drawLine(0, 10, 128, 10, SSD1306_WHITE);
    
    // 2. 保存旋转后的顶点
    float rotatedVertices[8][2];
    float rotatedCubeVertices[8][3];  // 保存3D坐标用于轴计算
    
    for (int i = 0; i < 8; i++) {
        // 创建顶点副本
        float vertex[3] = {cubeVertices[i][0], cubeVertices[i][1], cubeVertices[i][2]};
        
        // 保存旋转后的3D坐标
        rotatedCubeVertices[i][0] = vertex[0];
        rotatedCubeVertices[i][1] = vertex[1];
        rotatedCubeVertices[i][2] = vertex[2];
        rotatePoint(rotatedCubeVertices[i]);
        
        // 投影到2D
        int screenX = CENTER_X + rotatedCubeVertices[i][0] * SCALE_FACTOR;
        int screenY = CENTER_Y + rotatedCubeVertices[i][1] * SCALE_FACTOR;
        
        // 确保坐标在屏幕范围内
        if (screenX < 0) screenX = 0;
        if (screenX >= SCREEN_WIDTH) screenX = SCREEN_WIDTH - 1;
        if (screenY < 12) screenY = 12;
        if (screenY >= SCREEN_HEIGHT) screenY = SCREEN_HEIGHT - 1;
        
        rotatedVertices[i][0] = screenX;
        rotatedVertices[i][1] = screenY;
    }
    
    // 3. 绘制立方体的所有边
    for (int i = 0; i < 12; i++) {
        int startIdx = cubeEdges[i][0];
        int endIdx = cubeEdges[i][1];
        
        display.drawLine(rotatedVertices[startIdx][0], rotatedVertices[startIdx][1],
                        rotatedVertices[endIdx][0], rotatedVertices[endIdx][1],
                        SSD1306_WHITE);
    }
    
    // 4. 计算立方体中心（3D空间）
    float cubeCenter[3] = {0, 0, 0};  // 立方体原点就是中心
    
    // 5. 绘制旋转后的坐标轴
    for (int axis = 0; axis < 3; axis++) {
        // 计算轴向量
        float axisVector[3] = {axisVectors[axis][0], axisVectors[axis][1], axisVectors[axis][2]};
        
        // 旋转轴向量
        rotatePoint(axisVector);
        
        // 计算轴终点
        float axisEnd[3] = {
            cubeCenter[0] + axisVector[0],
            cubeCenter[1] + axisVector[1],
            cubeCenter[2] + axisVector[2]
        };
        
        // 绘制轴箭头
        drawAxisArrow(cubeCenter, axisEnd, axis);
    }
    
    // 6. 绘制顶点（小圆点）
    for (int i = 0; i < 8; i++) {
        display.fillCircle(rotatedVertices[i][0], rotatedVertices[i][1], 1, SSD1306_WHITE);
    }
    
    // 7. 在底部显示欧拉角
    float qw = q.w, qx = q.x, qy = q.y, qz = q.z;
    
    // 计算欧拉角
    float sinr_cosp = 2.0f * (qw * qx + qy * qz);
    float cosr_cosp = 1.0f - 2.0f * (qx * qx + qy * qy);
    float roll = atan2(sinr_cosp, cosr_cosp);
    
    float sinp = 2.0f * (qw * qy - qz * qx);
    float pitch;
    if (fabs(sinp) >= 1.0f) {
        pitch = copysign(M_PI / 2.0f, sinp);
    } else {
        pitch = asin(sinp);
    }
    
    float siny_cosp = 2.0f * (qw * qz + qx * qy);
    float cosy_cosp = 1.0f - 2.0f * (qy * qy + qz * qz);
    float yaw = atan2(siny_cosp, cosy_cosp);
    
    // 显示角度信息
    display.setCursor(0, SCREEN_HEIGHT - 9);
    display.print("R:");
    display.print(roll * RAD_TO_DEG, 0);
    display.print(" P:");
    display.print(pitch * RAD_TO_DEG, 0);
    display.print(" Y:");
    display.print(yaw * RAD_TO_DEG, 0);
    
    // 8. 显示当前方向
    display.setCursor(80, SCREEN_HEIGHT - 9);
    if (fabs(roll) < 0.3 && fabs(pitch) < 0.3)
        display.print("Level");
    else if (roll > 0.5)
        display.print("Right");
    else if (roll < -0.5)
        display.print("Left");
    else if (pitch > 0.5)
        display.print("Up");
    else if (pitch < -0.5)
        display.print("Down");
    
    display.display();
}

// ==================== 显示姿态信息 ====================
void displayAttitudeInfo() {
    float qw = q.w, qx = q.x, qy = q.y, qz = q.z;
    
    float sinr_cosp = 2.0f * (qw * qx + qy * qz);
    float cosr_cosp = 1.0f - 2.0f * (qx * qx + qy * qy);
    float roll = atan2(sinr_cosp, cosr_cosp);
    
    float sinp = 2.0f * (qw * qy - qz * qx);
    float pitch;
    if (fabs(sinp) >= 1.0f) {
        pitch = copysign(M_PI / 2.0f, sinp);
    } else {
        pitch = asin(sinp);
    }
    
    float siny_cosp = 2.0f * (qw * qz + qx * qy);
    float cosy_cosp = 1.0f - 2.0f * (qy * qy + qz * qz);
    float yaw = atan2(siny_cosp, cosy_cosp);
    
    // 串口输出
    Serial.print("姿态: Roll=");
    Serial.print(roll * RAD_TO_DEG, 1);
    Serial.print("°, Pitch=");
    Serial.print(pitch * RAD_TO_DEG, 1);
    Serial.print("°, Yaw=");
    Serial.print(yaw * RAD_TO_DEG, 1);
    Serial.print("° | 四元数: w=");
    Serial.print(q.w, 3);
    Serial.print(", x=");
    Serial.print(q.x, 3);
    Serial.print(", y=");
    Serial.print(q.y, 3);
    Serial.print(", z=");
    Serial.println(q.z, 3);
}

// ==================== 主循环 ====================
void loop() {
    unsigned long currentTime = millis();
    
    // 1. IMU数据读取与姿态更新 (100Hz)
    if (currentTime - lastIMUUpdate >= (1000 / SAMPLE_RATE)) {
        float accel[3], gyro[3];
        
        readIMUData(accel, gyro);
        madgwickUpdate(accel, gyro);
        
        static int serialCounter = 0;
        if (serialCounter++ >= 50) {
            displayAttitudeInfo();
            serialCounter = 0;
        }
        
        lastIMUUpdate = currentTime;
    }
    
    // 2. OLED显示更新 (15Hz)
    if (currentTime - lastDisplayUpdate >= 66) {
        draw3DCubeWithAxes();  // 绘制带坐标轴的3D立方体
        lastDisplayUpdate = currentTime;
    }
}
