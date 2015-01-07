#include <ros.h>

#include <std_msgs/String.h>
#include <std_msgs/Float64.h>
/* srv */
#include <jsk_battery/BatteryCapacity.h>
#include <jsk_battery/OutputCharacteristic.h>
#include <jsk_battery/ZeroPointAdjustment.h>
/* msg */
#include <jsk_battery/BatteryStatus.h>
#include <jsk_battery/BatteryArray.h>

ros::NodeHandle nh;

jsk_battery::BatteryStatus bs_msg;
ros::Publisher pub("battery_state", &bs_msg);

uint64_t max_capacity = 15 * 1e3;
uint64_t used_capacity = 0;
uint16_t ratio = 23;
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

static uint32_t currentTime = 0;
static uint16_t previousTime = 0;
static uint16_t cycleTime = 0;
static float prev_uci = 0.0; //prevous current integration value

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
  zero_point = averageAnalog(0, 20);
  prev_time = micros();
}

void main_loop() {
  int value = averageAnalog(0, 4);    /* [bit] */
  bs_msg.current = (value - zero_point) * (5.0 / 1024) / (ratio * 1e-3); /* current [A] : [bit] * [V/bit] / [V/A] */
  used_capacity += (bs_msg.current * 1e3) * (cycle_time * 1e-6 / 60 / 60);
  bs_msg.remaining_capacity = max_capacity - used_capacity;
  bs_msg.remaining_percentage = (max_capacity - used_capacity) / max_capacity * 100;
  delay(50);

  cur_time = micros();
  cycle_time = cur_time - prev_time;
  prev_time = cur_time;
}

void loop() {
  for(int i=0; i<10; i++) {
    main_loop();
  }
  pub.publish(&bs_msg);
  nh.spinOnce();
}

