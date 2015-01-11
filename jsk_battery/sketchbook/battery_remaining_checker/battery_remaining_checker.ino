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
float discharged_capacity = 0;
int ratio = 23;
int zero_point = (int)(1024 / 2);
unsigned long prev_time = 0;
unsigned long cycle_time = 0;

void bc_cb(const jsk_battery::BatteryCapacity::Request &req, jsk_battery::BatteryCapacity::Response &res) {
  max_capacity = req.battery_capacity;
}
void rovic_cb(const jsk_battery::OutputCharacteristic::Request &req, jsk_battery::OutputCharacteristic::Response &res) {
  ratio = req.ratio;
}
void zpa_cb(const jsk_battery::ZeroPointAdjustment::Request &req, jsk_battery::ZeroPointAdjustment::Response &res) {
  char buf[15];
  zero_point = averageAnalog(PIN, 10);
  dtostrf(zero_point, 5, 2, buf);
  nh.loginfo("set zero point:");
  nh.loginfo(buf);
}

ros::ServiceServer<jsk_battery::BatteryCapacity::Request, jsk_battery::BatteryCapacity::Response> capacity_service("set_battery_capacity", &bc_cb);
ros::ServiceServer<jsk_battery::OutputCharacteristic::Request, jsk_battery::OutputCharacteristic::Response> ratio_service("set_ratio_of_voltage_in_current", &rovic_cb);
ros::ServiceServer<jsk_battery::ZeroPointAdjustment::Request, jsk_battery::ZeroPointAdjustment::Response> calib_service("set_zero_point_adjustment", &zpa_cb);


int averageAnalog(int pin, int num){
  int v = 0;
  for(int i=0; i<num; i++) {
    v+= analogRead(pin);
  }
  return (int)(v / num);
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
  delay(50);
  prev_time = micros();
}

void main_loop() {
  int value = averageAnalog(PIN, 4); /* [bit] */
  float current = (value - zero_point) * (VREF * 1e3 / 1024) / ratio; /* [A] = [bit] * [mV/bit] / [mV/A] */
  unsigned long cur_time = micros();
  cycle_time = cur_time - prev_time;
  if (cycle_time < 0) {
    cycle_time += pow(2, sizeof(unsigned long)*8);
  }
  prev_time = cur_time;
  discharged_capacity += current * (float)cycle_time * 1e-6; /* [As] = [A] * [s] */
  float remaining_capacity = max_capacity - discharged_capacity * 1e3 / 60 / 60; /* [mAh] */
  float remaining_percentage = remaining_capacity / max_capacity * 100;

  bs_msg.current = current;
  bs_msg.remaining_capacity = remaining_capacity;
  bs_msg.remaining_percentage = remaining_percentage;
  bs_msg.analog_value = value;
  bs_msg.zero_point = zero_point;
  bs_msg.discharged_capacity = discharged_capacity;
  bs_msg.cycle_time = (float)cycle_time;
  bs_msg.diff = (current * ((float)cycle_time * 1e-6));
}

void loop() {
  for(int i=0; i<20; i++) {
    delay(50);                  /* need delay for analog read */
    main_loop();
  }

  /* nh.loginfo(String(sizeof(unsigned int))); */
  pub.publish(&bs_msg);
  nh.spinOnce();
}

