#include <ros.h>

/* srv */
#include <jsk_battery/BatteryCapacity.h>
#include <jsk_battery/OutputCharacteristic.h>
#include <jsk_battery/ZeroPointAdjustment.h>
/* msg */
#include <jsk_battery/BatteryStatus.h>
#include <jsk_battery/BatteryArray.h>

#define PIN 0
#define VREF 5.0

ros::NodeHandle nh;

jsk_battery::BatteryStatus bs_msg;
ros::Publisher pub("battery_state", &bs_msg);

float max_capacity = 15000;
float used_capacity = 0;
float ratio = 23;
float zero_point = 1024 / 2;

void bc_cb(const jsk_battery::BatteryCapacity::Request &req, jsk_battery::BatteryCapacity::Response &res) {
  max_capacity = req.battery_capacity;
}
void rovic_cb(const jsk_battery::OutputCharacteristic::Request &req, jsk_battery::OutputCharacteristic::Response &res) {
  ratio = req.ratio;
}
void zpa_cb(const jsk_battery::ZeroPointAdjustment::Request &req, jsk_battery::ZeroPointAdjustment::Response &res) {
  zero_point = req.zero_point;
  /* zero_point = averageAnalog(0, 10); */
}

ros::ServiceServer<jsk_battery::BatteryCapacity::Request, jsk_battery::BatteryCapacity::Response> capacity_service("set_battery_capacity", &bc_cb);
ros::ServiceServer<jsk_battery::OutputCharacteristic::Request, jsk_battery::OutputCharacteristic::Response> ratio_service("set_ratio_of_voltage_in_current", &rovic_cb);
ros::ServiceServer<jsk_battery::ZeroPointAdjustment::Request, jsk_battery::ZeroPointAdjustment::Response> calib_service("set_zero_point_adjustment", &zpa_cb);


uint32_t prev_time = 0;
uint32_t cur_time = 0;
uint32_t cycle_time = 0;

float averageAnalog(int pin, int num){
  float v = 0;
  for(int i=0; i<num; i++) {
    v+= analogRead(pin);
  }
  return v / num;
}

void setup() {
  nh.initNode();
  /* srv */
  nh.advertiseService(capacity_service);
  nh.advertiseService(ratio_service);
  nh.advertiseService(calib_service);
  /* msg */
  nh.advertise(pub);
  /* initialize */
  zero_point = averageAnalog(PIN, 20);
  prev_time = micros();
}

void main_loop() {
  float value = averageAnalog(PIN, 4);    /* [bit] */
  char buf[15];
  /* 
   * dtostrf(value, 5, 2, buf);
   * nh.loginfo(buf);
   * dtostrf(zero_point, 5, 2, buf);
   * nh.loginfo(buf);
   */

  float current;
  float remaining_capacity;
  float remaining_percentage;

  current = (value - zero_point) * (VREF * 1e3 / 1024) / ratio; /* [A] = [bit] * [mV/bit] / [mV/A] */
  used_capacity += current * ((float)cycle_time * 1e-6); /* [As] = [A] * [s] */
  remaining_capacity = max_capacity - used_capacity * 1e3 / 60 / 60; /* [mAh] */
  remaining_percentage = remaining_capacity / max_capacity * 100;

  bs_msg.current = current;
  bs_msg.remaining_capacity = remaining_capacity;
  bs_msg.remaining_percentage = remaining_percentage;

  /* 
   * bs_msg.current = (value - zero_point) * (5.0 / 1024) / ratio; /\* current [mA] : [bit] * [V/bit] / [mV/A] *\/
   * used_capacity += bs_msg.current * (cycle_time * 1e-6) / 60 / 60;        /\* [mA] * [h] *\/
   * bs_msg.remaining_capacity = (max_capacity - used_capacity);
   * bs_msg.remaining_percentage = (max_capacity - used_capacity) / max_capacity * 100;
   */
  delay(50);

  cur_time = micros();
  cycle_time = cur_time - prev_time;
  prev_time = cur_time;
}

void loop() {
  for(int i=0; i<10; i++) {
    main_loop();
  }
  /* 
   * char buf[15];
   * dtostrf(cycle_time, 5, 2, buf);
   * nh.loginfo(buf);
   * dtostrf(used_capacity, 10, 2, buf);
   * nh.loginfo(buf);
   */
  pub.publish(&bs_msg);
  nh.spinOnce();
}

