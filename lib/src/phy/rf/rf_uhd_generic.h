/*
 * Copyright 2013-2020 Software Radio Systems Limited
 *
 * This file is part of srsLTE.
 *
 * srsLTE is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Affero General Public License as
 * published by the Free Software Foundation, either version 3 of
 * the License, or (at your option) any later version.
 *
 * srsLTE is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Affero General Public License for more details.
 *
 * A copy of the GNU Affero General Public License can be found in
 * the LICENSE file in the top-level directory of this distribution
 * and at http://www.gnu.org/licenses/.
 *
 */

#ifndef SRSLTE_RF_UHD_GENERIC_H
#define SRSLTE_RF_UHD_GENERIC_H

#include "rf_uhd_safe.h"

class rf_uhd_generic : public rf_uhd_safe_interface
{
private:
  uhd::usrp::multi_usrp::sptr     usrp                         = nullptr;
  const uhd::fs_path              TREE_DBOARD_RX_FRONTEND_NAME = "/mboards/0/dboards/A/rx_frontends/A/name";
  const std::chrono::milliseconds FE_RX_RESET_SLEEP_TIME_MS    = std::chrono::milliseconds(2000UL);

  uhd_error usrp_make_internal(const uhd::device_addr_t& dev_addr) override
  {
    // Destroy any previous USRP instance
    usrp = nullptr;

    UHD_SAFE_C_SAVE_ERROR(this, usrp = uhd::usrp::multi_usrp::make(dev_addr);)
  }

public:
  uhd_error usrp_make(const uhd::device_addr_t& dev_addr) override
  { // Make USRP
    uhd_error err = usrp_multi_make(dev_addr);
    if (err != UHD_ERROR_NONE) {
      return err;
    }

    std::string dboard_name = usrp->get_device()->get_tree()->access<std::string>(TREE_DBOARD_RX_FRONTEND_NAME).get();

    // Detect if it a AD9361 based device
    if (dboard_name.find("FE-RX") != std::string::npos) {
      Info("The device is based on AD9361, get RX stream for checking LIBUSB_TRANSFER_ERROR");
      uint32_t ntrials = 10;
      do {
        err = set_rx_rate(1.92e6);
        if (err != UHD_ERROR_NONE) {
          return err;
        }

        uhd::stream_args_t stream_args("fc32", "sc16");
        stream_args.channels = {0};
        size_t max_samp      = 0;
        err                  = get_rx_stream(stream_args, max_samp);

        // If no error getting RX stream, return
        if (err == UHD_ERROR_NONE) {
          return err;
        }

        // Close USRP
        usrp = nullptr;

        Warning("Failed to open Rx stream '" << last_error << "', trying to open device again. " << ntrials
                                             << " trials left. Waiting for " << FE_RX_RESET_SLEEP_TIME_MS.count()
                                             << " ms");

        // Sleep
        std::this_thread::sleep_for(FE_RX_RESET_SLEEP_TIME_MS);

        // Try once more making the device
        err = usrp_multi_make(dev_addr);

      } while (err == UHD_ERROR_NONE and --ntrials != 0);
    }

    return err;
  }

  uhd_error set_tx_subdev(const std::string& string) override
  {
    UHD_SAFE_C_SAVE_ERROR(this, usrp->set_tx_subdev_spec(string);)
  }
  uhd_error set_rx_subdev(const std::string& string) override
  {
    UHD_SAFE_C_SAVE_ERROR(this, usrp->set_rx_subdev_spec(string);)
  }
  uhd_error get_mboard_name(std::string& mboard_name) override
  {
    UHD_SAFE_C_SAVE_ERROR(this, mboard_name = usrp->get_mboard_name();)
  }
  uhd_error get_mboard_sensor_names(std::vector<std::string>& sensors) override
  {
    UHD_SAFE_C_SAVE_ERROR(this, sensors = usrp->get_mboard_sensor_names();)
  }
  uhd_error get_rx_sensor_names(std::vector<std::string>& sensors) override
  {
    UHD_SAFE_C_SAVE_ERROR(this, sensors = usrp->get_rx_sensor_names();)
  }
  uhd_error get_sensor(const std::string& sensor_name, uhd::sensor_value_t& sensor_value) override
  {
    UHD_SAFE_C_SAVE_ERROR(this, sensor_value = usrp->get_mboard_sensor(sensor_name);)
  }
  uhd_error get_rx_sensor(const std::string& sensor_name, uhd::sensor_value_t& sensor_value) override
  {
    UHD_SAFE_C_SAVE_ERROR(this, sensor_value = usrp->get_rx_sensor(sensor_name);)
  }
  uhd_error set_time_unknown_pps(const uhd::time_spec_t& timespec) override
  {
    UHD_SAFE_C_SAVE_ERROR(this, usrp->set_time_unknown_pps(timespec);)
  }
  uhd_error get_time_now(uhd::time_spec_t& timespec) override
  {
    UHD_SAFE_C_SAVE_ERROR(this, timespec = usrp->get_time_now();)
  }
  uhd_error set_sync_source(const std::string& source) override
  {
#if UHD_VERSION < 3140099
    UHD_SAFE_C_SAVE_ERROR(this, usrp->set_clock_source(source); usrp->set_time_source(source);)
#else
    UHD_SAFE_C_SAVE_ERROR(this, usrp->set_sync_source(source, source);)
#endif
  }
  uhd_error get_gain_range(uhd::gain_range_t& tx_gain_range, uhd::gain_range_t& rx_gain_range) override
  {
    UHD_SAFE_C_SAVE_ERROR(this, tx_gain_range = usrp->get_tx_gain_range(); rx_gain_range = usrp->get_rx_gain_range();)
  }
  uhd_error set_master_clock_rate(double rate) override
  {
    UHD_SAFE_C_SAVE_ERROR(this, usrp->set_master_clock_rate(rate);)
  }
  uhd_error set_rx_rate(double rate) override { UHD_SAFE_C_SAVE_ERROR(this, usrp->set_rx_rate(rate);) }
  uhd_error set_tx_rate(double rate) override { UHD_SAFE_C_SAVE_ERROR(this, usrp->set_tx_rate(rate);) }
  uhd_error set_command_time(const uhd::time_spec_t& timespec) override
  {
    UHD_SAFE_C_SAVE_ERROR(this, usrp->set_command_time(timespec);)
  }
  uhd_error get_rx_stream(const uhd::stream_args_t& args, size_t& max_num_samps) override
  {
    UHD_SAFE_C_SAVE_ERROR(this, rx_stream = nullptr; rx_stream = usrp->get_rx_stream(args);
                          max_num_samps = rx_stream->get_max_num_samps();
                          if (max_num_samps == 0UL) {
                            last_error = "The maximum number of receive samples is zero.";
                            return UHD_ERROR_VALUE;
                          })
  }
  uhd_error get_tx_stream(const uhd::stream_args_t& args, size_t& max_num_samps) override
  {
    UHD_SAFE_C_SAVE_ERROR(this, tx_stream = nullptr; tx_stream = usrp->get_tx_stream(args);
                          max_num_samps = tx_stream->get_max_num_samps();
                          if (max_num_samps == 0UL) {
                            last_error = "The maximum number of transmit samples is zero.";
                            return UHD_ERROR_VALUE;
                          })
  }
  uhd_error set_tx_gain(size_t ch, double gain) override { UHD_SAFE_C_SAVE_ERROR(this, usrp->set_tx_gain(gain, ch);) }
  uhd_error set_rx_gain(size_t ch, double gain) override { UHD_SAFE_C_SAVE_ERROR(this, usrp->set_rx_gain(gain, ch);) }
  uhd_error get_rx_gain(double& gain) override { UHD_SAFE_C_SAVE_ERROR(this, gain = usrp->get_rx_gain();) }
  uhd_error get_tx_gain(double& gain) override { UHD_SAFE_C_SAVE_ERROR(this, gain = usrp->get_tx_gain();) }
  uhd_error set_tx_freq(uint32_t ch, double target_freq, double& actual_freq) override
  {
    UHD_SAFE_C_SAVE_ERROR(this, uhd::tune_request_t tune_request(target_freq);
                          uhd::tune_result_t tune_result = usrp->set_tx_freq(tune_request, ch);
                          actual_freq                    = tune_result.target_rf_freq;)
  }
  uhd_error set_rx_freq(uint32_t ch, double target_freq, double& actual_freq) override
  {
    UHD_SAFE_C_SAVE_ERROR(this, uhd::tune_request_t tune_request(target_freq);
                          uhd::tune_result_t tune_result = usrp->set_rx_freq(tune_request, ch);
                          actual_freq                    = tune_result.target_rf_freq;)
  }
};

#endif // SRSLTE_RF_UHD_GENERIC_H