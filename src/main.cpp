#include "mbed.h"
#include "PID.hpp"

// グローバル変数
int16_t pwm[4] = {0, 0, 0, 0};

int count_hozon[4] = {};     // ロリコンのカウント合計値
int count_hozon_pre[4] = {}; // ロリコンの前回のカウント合計値

// PIDパラメーター
const float kp = 0.5;
const float ki = 0.001;
const float kd = 0.0;
const float sample_time = 0.01; // 10ms sample time

// PIDコントローラー
PID pid_controller(kp, ki, kd, sample_time);

// ロボットのパラメータ
const float wheel_diameter = 0.08;      // オムニホイールの直径
const int encoder_resolution = 256 * 4; // 256パルス * 4つ

// オドメトリ関数
void Odometry(float &x_mtere, float &y_mtere, float &theta)
{
    float dx = 0;
    float dy = 0;
    float dtheta_rad = 0;
    constexpr float M_4 = M_PI / 4; // エンコーダが4つあるので4で割る
    // x,y,θの変位を求める
    for (int i = 0; i < 4; ++i)
    {
        dx += (count_hozon[i] - count_hozon_pre[i]) * cos((2 * i) * M_4 + theta);
        dy += (count_hozon[i] - count_hozon_pre[i]) * sin((2 * i) * M_4 + theta);
        dtheta_rad += count_hozon[i] - count_hozon_pre[i];
    }
    // 現在の位置を更新
    x_mtere += dx * wheel_diameter / encoder_resolution;
    y_mtere += dy * wheel_diameter / encoder_resolution;
    theta += 2 * M_PI / -34552 * dtheta_rad;                              // 360度を一回転分のカウントで割り、ラジアンにする//34552
    printf("x= %4.2f,y= %4.2f,theta = %4.2f\n", x_mtere, y_mtere, theta); // 現在地

    // 現在の回転数を更新する
    for (int i = 0; i < 4; ++i)
    {
        count_hozon_pre[i] = count_hozon[i];
    }
}

// オムニホイールのPWM出力を計算する関数
void Omni(float Omni_x_PID, float Omni_y_PID, float Omni_theta_PID, int16_t pwm[4])
{
    float r = hypot(Omni_x_PID, Omni_y_PID);
    float theta_2 = atan2(Omni_y_PID, Omni_x_PID);
    //                                          |< + piteh(姿勢角)
    int pwm_power0 = r * 8000 * cos(theta_2 + (45 * M_PI / 180)) + Omni_theta_PID * 2000;
    int pwm_power3 = r * 8000 * cos(theta_2 - (45 * M_PI / 180)) + Omni_theta_PID * 2000;
    int pwm_power2 = r * 8000 * cos(theta_2 - (125 * M_PI / 180)) + Omni_theta_PID * 2000;
    int pwm_power1 = r * 8000 * cos(theta_2 - (225 * M_PI / 180)) + Omni_theta_PID * 2000;

    // 値をint16_tにキャストして範囲をクリップする
    pwm[0] = std::clamp(pwm_power0, -8000, 8000);
    pwm[3] = std::clamp(pwm_power3, -8000, 8000);
    pwm[2] = std::clamp(pwm_power2, -8000, 8000);
    pwm[1] = std::clamp(pwm_power1, -8000, 8000);
    // printf("pwm: pwm[0] = % 4.2f , pwm [1] = % 4.2f , pwm[2] = % 4.2f , pwm[3] = % 4.2f\n",
    //    static_cast<float>(pwm[0]), static_cast<float>(pwm[1]), static_cast<float>(pwm[2]), static_cast<float>(pwm[3]));
}

int main()
{
    // 初期化
    BufferedSerial pc(USBTX, USBRX, 115200);
    CAN can(PA_11, PA_12, (int)1e6);
    CANMessage msg;
    float x_mtere = 0.0;           // X座標(m)
    float y_mtere = 0.0;           // Y座標(m)
    float theta = 0;               // θ座標
    float target_x = 1.0;          // 目標のx座標(m)
    float target_y = 1.0;          // 目標のy座標(m)
    float target_theta = -M_PI / 2; // 目標のθ座標

    while (1)
    {
        // auto now = HighResClock::now();
        // static auto pre = now;

        if (can.read(msg) && msg.id == 48)
            for (int i = 0; i < 4; i += 1)
            {
                signed short data_value = (msg.data[2 * i + 1] << 8) | msg.data[2 * i];
                count_hozon[i] += data_value;
            }
        // printf("count%d\n", count_hozon[0]);
        // if (!can.read(msg))
        //     printf("can'tCAN\n");

        // オドメトリ計算
        Odometry(x_mtere, y_mtere, theta);

        // PID
        float Omni_x_PID = pid_controller.calculate(target_x, x_mtere); // オムニの速度を出す
        float Omni_y_PID = pid_controller.calculate(target_y, y_mtere);
        float Omni_theta_PID = pid_controller.calculate(target_theta, theta);
        // printf(" theta = %4.2f\n", Omni_theta_PID); // PID計算結果

        // オムニ
        Omni(Omni_x_PID, Omni_y_PID, Omni_theta_PID, pwm);

        CANMessage msg(1, (const uint8_t *)pwm, 8);
        can.write(msg);

        // if (now - pre > 50ms)
        //{
        //
        //  printf("output : x = % 4.2f, y = %4.2f , theta = %4.2f\n", Omni_x_PID, Omni_y_PID, Omni_theta_PID);                      // PID計算結果
        //  printf("pwm: pwm[0] = % 4.2d , pwm [1] = % 4.2d , pwm[2] = % 4.2d , pwm[3] = % 4.2d\n", pwm[0], pwm[1], pwm[2], pwm[3]); // pwmの値
        //  printf("pwm: pwm[0] = % 4.2f , pwm [1] = % 4.2f , pwm[2] = % 4.2f , pwm[3] = % 4.2f\n", static_cast<float>(pwm[0]), static_cast<float>(pwm[1]), static_cast<float>(pwm[2]), static_cast<float>(pwm[3]));
        // pre = now;
        // }
    }
}