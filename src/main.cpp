#include <Arduino.h>
#include <Arduino_JSON.h>
#include <AsyncTCP.h>
#include <BMP180I2C.h>
#include <ESPAsyncWebServer.h>
#include <PID_v1.h>
#include <WiFi.h>

#include "SPIFFS.h"
#include "Wire.h"
#include "soc/rtc_cntl_reg.h"
#include "soc/soc.h"

const char *ssid = "duron";
// const char *password = "12345678";

AsyncWebServer server(80);
AsyncWebSocket ws("/ws");

#define M_PI 3.14159265358979323846

#define ERROR_LED 4

#define LF_MOTOR_PIN 16
#define RF_MOTOR_PIN 12
#define LB_MOTOR_PIN 2
#define RB_MOTOR_PIN 13

#define LF_MOTOR 0
#define RF_MOTOR 1
#define LB_MOTOR 2
#define RB_MOTOR 3

#define PWM_FREQ 5000

#define MIN_THRUST 0
#define MAX_THRUST 255

#define MPU_addr 0x68
#define BMP_addr 0x77

#define gscale \
  ((250. / 32768.0) * (PI / 180.0))  // gyro default 250 LSB per d/s -> rad/s

BMP180I2C bmp180(BMP_addr);

void initFS() {
  if (!SPIFFS.begin()) {
    Serial.println("An error has occurred while mounting SPIFFS");
  } else {
    Serial.println("SPIFFS mounted successfully");
  }
}

class DroneState {
 public:
  int motor_thr = 0;

  int motor_speed[4];  // left front, right front, left back, right back

  PID pitch_pid;
  PID roll_pid;
  PID yaw_pid;

  struct {
    float A_cal[6] = {265.0, -80.0, -700.0,
                      0.994, 1.000, 1.014};  // 0..2 offset xyz, 3..5 scale xyz
    float G_off[3] = {-499.5, -17.7,
                      -82.0};  // raw offsets, determined for gyro at rest

    float q[4] = {1.0, 0.0, 0.0, 0.0};  // quaternion

    // Free parameters in the Mahony filter and fusion scheme,
    // Kp for proportional feedback, Ki for integral
    float Kp = 30.0;
    float Ki = 0.0;

    struct {
      int i = 0;
      float deltat = 0;                 // loop time in seconds
      unsigned long now = 0, last = 0;  // micros() timers
    } loop;

  } sensor_stuff;

  double desired_pitch;
  double desired_roll;
  double desired_yaw;

  double pitch_error;
  double roll_error;
  double yaw_error;

  double pitch_out;
  double roll_out;
  double yaw_out;

  double pitch;
  double roll;
  double yaw;

  DroneState()
      : motor_thr(0),
        motor_speed({0, 0, 0, 0}),
        pitch_pid(PID(&this->pitch, &this->pitch_out, &this->desired_pitch, 1,
                      0, 0, DIRECT)),
        roll_pid(PID(&this->roll, &this->roll_out, &this->desired_roll, 1, 0, 0,
                     DIRECT)),
        yaw_pid(PID(&this->yaw, &this->yaw_out, &this->desired_yaw, 1, 0, 0,
                    DIRECT)),
        desired_pitch(0),
        desired_roll(0),
        desired_yaw(0) {
    this->pitch_pid.SetMode(AUTOMATIC);
    this->roll_pid.SetMode(AUTOMATIC);
    this->yaw_pid.SetMode(AUTOMATIC);

    this->pitch_pid.SetOutputLimits(-255, 255);
    this->roll_pid.SetOutputLimits(-255, 255);
    this->yaw_pid.SetOutputLimits(-255, 255);
  }

  void Mahony_update(float ax, float ay, float az, float gx, float gy, float gz,
                     float deltat) {
    float recipNorm;
    float vx, vy, vz;
    float ex, ey, ez;  // error terms
    float qa, qb, qc;
    static float ix = 0.0, iy = 0.0, iz = 0.0;  // integral feedback terms
    float tmp;

    // Compute feedback only if accelerometer measurement valid (avoids NaN in
    // accelerometer normalisation)
    tmp = ax * ax + ay * ay + az * az;
    if (tmp > 0.0) {
      // Normalise accelerometer (assumed to measure the direction of gravity in
      // body frame)
      recipNorm = 1.0 / sqrt(tmp);
      ax *= recipNorm;
      ay *= recipNorm;
      az *= recipNorm;

      // Estimated direction of gravity in the body frame (factor of two divided
      // out)
      vx = sensor_stuff.q[1] * sensor_stuff.q[3] -
           sensor_stuff.q[0] * sensor_stuff.q[2];
      vy = sensor_stuff.q[0] * sensor_stuff.q[1] +
           sensor_stuff.q[2] * sensor_stuff.q[3];
      vz = sensor_stuff.q[0] * sensor_stuff.q[0] - 0.5f +
           sensor_stuff.q[3] * sensor_stuff.q[3];

      // Error is cross product between estimated and measured direction of
      // gravity in body frame (half the actual magnitude)
      ex = (ay * vz - az * vy);
      ey = (az * vx - ax * vz);
      ez = (ax * vy - ay * vx);

      // Compute and apply to gyro term the integral feedback, if enabled
      if (sensor_stuff.Ki > 0.0f) {
        ix += sensor_stuff.Ki * ex * deltat;  // integral error scaled by Ki
        iy += sensor_stuff.Ki * ey * deltat;
        iz += sensor_stuff.Ki * ez * deltat;
        gx += ix;  // apply integral feedback
        gy += iy;
        gz += iz;
      }

      // Apply proportional feedback to gyro term
      gx += sensor_stuff.Kp * ex;
      gy += sensor_stuff.Kp * ey;
      gz += sensor_stuff.Kp * ez;
    }

    // Integrate rate of change of quaternion, q cross gyro term
    deltat = 0.5 * deltat;
    gx *= deltat;  // pre-multiply common factors
    gy *= deltat;
    gz *= deltat;
    qa = sensor_stuff.q[0];
    qb = sensor_stuff.q[1];
    qc = sensor_stuff.q[2];
    sensor_stuff.q[0] += (-qb * gx - qc * gy - sensor_stuff.q[3] * gz);
    sensor_stuff.q[1] += (qa * gx + qc * gz - sensor_stuff.q[3] * gy);
    sensor_stuff.q[2] += (qa * gy - qb * gz + sensor_stuff.q[3] * gx);
    sensor_stuff.q[3] += (qa * gz + qb * gy - qc * gx);

    // renormalise quaternion
    recipNorm = 1.0 / sqrt(sensor_stuff.q[0] * sensor_stuff.q[0] +
                           sensor_stuff.q[1] * sensor_stuff.q[1] +
                           sensor_stuff.q[2] * sensor_stuff.q[2] +
                           sensor_stuff.q[3] * sensor_stuff.q[3]);
    sensor_stuff.q[0] = sensor_stuff.q[0] * recipNorm;
    sensor_stuff.q[1] = sensor_stuff.q[1] * recipNorm;
    sensor_stuff.q[2] = sensor_stuff.q[2] * recipNorm;
    sensor_stuff.q[3] = sensor_stuff.q[3] * recipNorm;
  }

  void get_sensor_data() {
    // raw data
    int16_t ax, ay, az;
    int16_t gx, gy, gz;
    int16_t Tmp;  // temperature

    // scaled data as vector
    float Axyz[3];
    float Gxyz[3];

    Wire.beginTransmission(MPU_addr);
    Wire.write(0x3B);  // starting with register 0x3B (ACCEL_XOUT_H)
    Wire.endTransmission(false);
    Wire.requestFrom(MPU_addr, 14, true);  // request a total of 14 registers
    int t = Wire.read() << 8;
    ax = t | Wire.read();  // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)
    t = Wire.read() << 8;
    ay = t | Wire.read();  // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
    t = Wire.read() << 8;
    az = t | Wire.read();  // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
    t = Wire.read() << 8;
    Tmp = t | Wire.read();  // 0x41 (TEMP_OUT_H) & 0x42 (TEMP_OUT_L)
    t = Wire.read() << 8;
    gx = t | Wire.read();  // 0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)
    t = Wire.read() << 8;
    gy = t | Wire.read();  // 0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L)
    t = Wire.read() << 8;
    gz = t | Wire.read();  // 0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)

    Axyz[0] = (float)ax;
    Axyz[1] = (float)ay;
    Axyz[2] = (float)az;

    // apply offsets and scale factors from Magneto
    for (int i = 0; i < 3; i++)
      Axyz[i] = (Axyz[i] - sensor_stuff.A_cal[i]) * sensor_stuff.A_cal[i + 3];

    Gxyz[0] = ((float)gx - sensor_stuff.G_off[0]) *
              gscale;  // 250 LSB(d/s) default to radians/s
    Gxyz[1] = ((float)gy - sensor_stuff.G_off[1]) * gscale;
    Gxyz[2] = ((float)gz - sensor_stuff.G_off[2]) * gscale;

    //  snprintf(s,sizeof(s),"mpu raw %d,%d,%d,%d,%d,%d",ax,ay,az,gx,gy,gz);
    //  Serial.println(s);

    sensor_stuff.loop.now = micros();
    sensor_stuff.loop.deltat =
        (sensor_stuff.loop.now - sensor_stuff.loop.last) *
        1.0e-6;  // seconds since last update
    sensor_stuff.loop.last = sensor_stuff.loop.now;

    this->Mahony_update(Axyz[0], Axyz[1], Axyz[2], Gxyz[0], Gxyz[1], Gxyz[2],
                        sensor_stuff.loop.deltat);

    pitch = atan2((sensor_stuff.q[0] * sensor_stuff.q[1] +
                   sensor_stuff.q[2] * sensor_stuff.q[3]),
                  0.5 - (sensor_stuff.q[1] * sensor_stuff.q[1] +
                         sensor_stuff.q[2] * sensor_stuff.q[2]));
    roll = asin(2.0 * (sensor_stuff.q[0] * sensor_stuff.q[2] -
                       sensor_stuff.q[1] * sensor_stuff.q[3]));
    // conventional yaw increases clockwise from North. Not that the MPU-6050
    // knows where North is.
    yaw = -atan2((sensor_stuff.q[1] * sensor_stuff.q[2] +
                  sensor_stuff.q[0] * sensor_stuff.q[3]),
                 0.5 - (sensor_stuff.q[2] * sensor_stuff.q[2] +
                        sensor_stuff.q[3] * sensor_stuff.q[3]));
    // to degrees
    yaw *= 180.0 / PI;
    roll *= 180.0 / PI;
    pitch *= 180.0 / PI;

    pitch -= 8.0;
    roll += 5.9 ;
  }

  void tick() {
    this->pitch_error = this->desired_pitch - this->pitch;
    this->roll_error = this->desired_roll - this->roll;
    this->yaw_error = this->desired_yaw - this->yaw;

    float total_error = abs(pitch_error) + abs(roll_error) + abs(yaw_error);

    this->pitch_pid.Compute();
    this->roll_pid.Compute();
    this->yaw_pid.Compute();
    // Serial.print(millis() / 1000.0);
    // Serial.print(",");
    // Serial.print(this->pitch);
    // Serial.print(",");
    // Serial.println(this->pitch_out);
    int max_correction =
        /* this->motor_thr; */ this->motor_thr == 0 ? 0 : 65535;

    motor_speed[LF_MOTOR] =
        motor_thr + constrain(-(abs(pitch_error) / total_error) * pitch_out -
                                  (abs(roll_error) / total_error) * roll_out -
                                  (abs(yaw_error) / total_error) * yaw_out,
                              -max_correction, max_correction);
    motor_speed[RF_MOTOR] =
        motor_thr + constrain(-(abs(pitch_error) / total_error) * pitch_out +
                                  (abs(roll_error) / total_error) * roll_out +
                                  (abs(yaw_error) / total_error) * yaw_out,
                              -max_correction, max_correction);
    motor_speed[LB_MOTOR] =
        motor_thr + constrain((abs(pitch_error) / total_error) * pitch_out -
                                  (abs(roll_error) / total_error) * roll_out +
                                  (abs(yaw_error) / total_error) * yaw_out,
                              -max_correction, max_correction);
    motor_speed[RB_MOTOR] =
        motor_thr + constrain((abs(pitch_error) / total_error) * pitch_out +
                                  (abs(roll_error) / total_error) * roll_out -
                                  (abs(yaw_error) / total_error) * yaw_out,
                              -max_correction, max_correction);

    for (int i = 0; i < 4; i++) {
      motor_speed[i] = constrain(motor_speed[i], MIN_THRUST, MAX_THRUST);
    }
  }
};

DroneState Drone;

enum class error_code {
  pressure_sensor = 0,
  gyro_sensor = 1,
};

int error_codes[][2] = {
    {1000, 500},
    {500, 1000},
};

void error_out(error_code e) {
  while (1) {
    digitalWrite(ERROR_LED, HIGH);
    delay(error_codes[static_cast<int>(e)][0]);
    digitalWrite(ERROR_LED, LOW);
    delay(error_codes[static_cast<int>(e)][1]);
  }
}

void handleWebSocketMessage(void *arg, uint8_t *data, size_t len) {
  AwsFrameInfo *info = (AwsFrameInfo *)arg;
  if (info->final && info->index == 0 && info->len == len &&
      info->opcode == WS_TEXT) {
    data[len] = 0;
    String message = (char *)data;
    if (message.indexOf("1s") >= 0) {
      String sliderValue1 = message.substring(2);
      Drone.motor_thr = sliderValue1.toInt();
    } else if (message.indexOf("p1") >= 0) {
      String val = message.substring(2);
      Drone.pitch_pid.SetTunings(val.toFloat(), Drone.pitch_pid.GetKi(),
                                 Drone.pitch_pid.GetKd(), P_ON_E);
    } else if (message.indexOf("i1") >= 0) {
      String val = message.substring(2);
      Drone.pitch_pid.SetTunings(Drone.pitch_pid.GetKp(), val.toFloat(),
                                 Drone.pitch_pid.GetKd(), P_ON_E);
    } else if (message.indexOf("d1") >= 0) {
      String val = message.substring(2);
      Drone.pitch_pid.SetTunings(Drone.pitch_pid.GetKp(),
                                 Drone.pitch_pid.GetKi(), val.toFloat(),
                                 P_ON_E);
    } else if (message.indexOf("p2") >= 0) {
      String val = message.substring(2);
      Drone.roll_pid.SetTunings(val.toFloat(), Drone.roll_pid.GetKi(),
                                Drone.roll_pid.GetKd(), P_ON_E);
    } else if (message.indexOf("i2") >= 0) {
      String val = message.substring(2);
      Drone.roll_pid.SetTunings(Drone.roll_pid.GetKp(), val.toFloat(),
                                Drone.roll_pid.GetKd(), P_ON_E);
    } else if (message.indexOf("d2") >= 0) {
      String val = message.substring(2);
      Drone.roll_pid.SetTunings(Drone.roll_pid.GetKp(), Drone.roll_pid.GetKi(),
                                val.toFloat(), P_ON_E);
    } else if (message.indexOf("p3") >= 0) {
      String val = message.substring(2);
      Drone.yaw_pid.SetTunings(val.toFloat(), Drone.yaw_pid.GetKi(),
                               Drone.yaw_pid.GetKd(), P_ON_E);
    } else if (message.indexOf("i3") >= 0) {
      String val = message.substring(2);
      Drone.yaw_pid.SetTunings(Drone.yaw_pid.GetKp(), val.toFloat(),
                               Drone.yaw_pid.GetKd(), P_ON_E);
    } else if (message.indexOf("d3") >= 0) {
      String val = message.substring(2);
      Drone.yaw_pid.SetTunings(Drone.yaw_pid.GetKp(), Drone.yaw_pid.GetKi(),
                               val.toFloat(), P_ON_E);
    }
    // Serial.print("Drone PIDs:");
    // Serial.print(Drone.pitch_pid.GetKp());
    // Serial.print(",");
    // Serial.print(Drone.pitch_pid.GetKi());
    // Serial.print(",");
    // Serial.print(Drone.pitch_pid.GetKd());
    // Serial.print("|");
    // Serial.print(Drone.roll_pid.GetKp());
    // Serial.print(",");
    // Serial.print(Drone.roll_pid.GetKi());
    // Serial.print(",");
    // Serial.print(Drone.roll_pid.GetKd());
    // Serial.print("|");
    // Serial.print(Drone.yaw_pid.GetKp());
    // Serial.print(",");
    // Serial.print(Drone.yaw_pid.GetKi());
    // Serial.print(",");
    // Serial.print(Drone.yaw_pid.GetKd());
    // Serial.println();
  }
}

void onEvent(AsyncWebSocket *server, AsyncWebSocketClient *client,
             AwsEventType type, void *arg, uint8_t *data, size_t len) {
  switch (type) {
    case WS_EVT_CONNECT:
      break;
    case WS_EVT_DISCONNECT:
      break;
    case WS_EVT_DATA:
      handleWebSocketMessage(arg, data, len);
      break;
    case WS_EVT_PONG:
    case WS_EVT_ERROR:
      break;
  }
}

String get_debug_state() {
  JSONVar sliderValues;
  sliderValues["lf"] = String(Drone.motor_speed[LF_MOTOR]);
  sliderValues["rf"] = String(Drone.motor_speed[RF_MOTOR]);
  sliderValues["lb"] = String(Drone.motor_speed[LB_MOTOR]);
  sliderValues["rb"] = String(Drone.motor_speed[RB_MOTOR]);
  sliderValues["pt"] = String(Drone.pitch);
  sliderValues["rl"] = String(Drone.roll);
  sliderValues["yw"] = String(Drone.yaw);

  String jsonString = JSON.stringify(sliderValues);
  return jsonString;
}

void initWebSocket() {
  ws.onEvent(onEvent);
  server.addHandler(&ws);
}

void setup() {
  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0);  // disable   detector

  ledcSetup(LF_MOTOR, PWM_FREQ, 8);
  ledcAttachPin(LF_MOTOR_PIN, LF_MOTOR);
  ledcSetup(RF_MOTOR, PWM_FREQ, 8);
  ledcAttachPin(RF_MOTOR_PIN, RF_MOTOR);
  ledcSetup(LB_MOTOR, PWM_FREQ, 8);
  ledcAttachPin(LB_MOTOR_PIN, LB_MOTOR);
  ledcSetup(RB_MOTOR, PWM_FREQ, 8);
  ledcAttachPin(RB_MOTOR_PIN, RB_MOTOR);
  pinMode(ERROR_LED, OUTPUT);
  digitalWrite(ERROR_LED, LOW);
  Wire.begin(14, 15);
  // if (!mpu.begin())
  //   error_out(error_code::gyro_sensor);

  Wire.beginTransmission(MPU_addr);
  Wire.write(0x6B);  // PWR_MGMT_1 register
  Wire.write(0);     // set to zero (wakes up the MPU-6050)
  Wire.endTransmission(true);

  Wire.beginTransmission(MPU_addr);
  Wire.write(0x1A);  // write to address 26 of the register
  Wire.write(0x02);  // options here are 0x00 which is off, and 0x01, 0x02,
                     // 0x03, 0x04, 0x05, 0x06
  Wire.endTransmission(true);  // 0x06 being the highest filter setting

  // if (!bmp180.begin()) error_out(error_code::pressure_sensor);
  // bmp180.resetToDefaults();
  // bmp180.setSamplingMode(BMP180MI::MODE_UHR);
  Serial.begin(115200);

  initFS();
  Serial.print("Setting AP (Access Point)…");
  // WiFi.softAP(ssid, password);
  WiFi.softAP(ssid);

  IPAddress IP = WiFi.softAPIP();
  Serial.print("AP IP address: ");
  Serial.println(IP);

  initWebSocket();
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request) {
    request->send(SPIFFS, "/index.html", "text/html");
  });

  server.serveStatic("/", SPIFFS, "/");

  server.begin();
  Drone.motor_thr = 0;

  Drone.motor_thr = 0;
  Drone.pitch_pid.SetTunings(1, 0, 0, P_ON_E);
  Drone.roll_pid.SetTunings(1, 0, 0, P_ON_E);
  Drone.yaw_pid.SetTunings(0, 0, 0, P_ON_E);
}

long last_debug_time = 0;

void loop() {
  Drone.get_sensor_data();
  Drone.tick();

  ledcWrite(LF_MOTOR, Drone.motor_speed[LF_MOTOR]);
  ledcWrite(RF_MOTOR, Drone.motor_speed[RF_MOTOR]);
  ledcWrite(LB_MOTOR, Drone.motor_speed[LB_MOTOR]);
  ledcWrite(RB_MOTOR, Drone.motor_speed[RB_MOTOR]);

  Serial.print("Motor speed: LF: ");
  Serial.print(Drone.motor_speed[LF_MOTOR]);
  Serial.print("\tRF: ");
  Serial.print(Drone.motor_speed[RF_MOTOR]);
  Serial.print("\tLB: ");
  Serial.print(Drone.motor_speed[LB_MOTOR]);
  Serial.print("\tRB: ");
  Serial.print(Drone.motor_speed[RB_MOTOR]);
  Serial.print("\t\tPitch: ");
  Serial.print(Drone.pitch);
  Serial.print("\tRoll: ");
  Serial.println(Drone.roll);

  if (millis() - last_debug_time >= 100) {
    ws.textAll(get_debug_state());
    last_debug_time = millis();
  }

  ws.cleanupClients();
}