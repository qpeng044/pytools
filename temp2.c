TriggerItemBits WaterPumpMonitor::OperationSafetyCheck() {
  // when pwm level is low, do not process
  if (water_pump_control_->CheckIfWaterPumpUsePwm() &&
      !water_pump_control_->GetTimPwmCurrentLevel() &&
      (within_lower_limit_count_ == 0 ||
       (within_lower_limit_count_ > 0 && !water_pump_water_shortage_))) {
    return 0;
  }

  TriggerItemBits trigger_item = 0;
  // if the filtered reading is within the normal operation range
  if (filtered_reading_ >=
          kMinPumpWorkValueScale * operation_parameter_.lower_limit &&
      filtered_reading_ <= operation_parameter_.lower_limit) {
    if (within_lower_limit_count_++ >=
        operation_parameter_.trigger_window_length) {
      water_pump_water_shortage_ = true;
      trigger_item |= WATER_PUMP_WATER_SHORTAGE;
    }
  } else if (filtered_reading_ >= operation_parameter_.upper_limit) {
    // if the filtered reading is beyond the normal operation range
    if (beyond_upper_limit_count_++ >=
        operation_parameter_.trigger_window_length) {
      trigger_item |= WATER_PUMP_OVER_CURRENT_TRIGGER_BIT;
    }
  } else {
    if (within_lower_limit_count_ > 0) {
      within_lower_limit_count_--;
      if (water_pump_water_shortage_ && within_lower_limit_count_ == 0) {
        water_pump_water_shortage_ = false;
        trigger_item |= WATER_PUMP_WATER_SHORTAGE_RESUME;
      }
    }
    if (beyond_upper_limit_count_ > 0) {
      beyond_upper_limit_count_--;
    }
  }

  return trigger_item;
}