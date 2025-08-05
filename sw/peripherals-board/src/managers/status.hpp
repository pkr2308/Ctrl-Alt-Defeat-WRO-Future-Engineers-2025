/**
 * @brief Enum for representing status. Will be dreprecated when FailureManager is implemented
 * @author DIY Labs
 */

#pragma once

enum status_t{

  STATUS_HEALTHY = 0,
  STATUS_FAULT = 1,
  STATUS_CALIBRATING = 2,
  STATUS_NO_SIGNAL = 3

};