/**
 *
 * \section COPYRIGHT
 *
 * Copyright 2013-2021 Software Radio Systems Limited
 *
 * By using this file, you agree to the terms and conditions set
 * forth in the LICENSE file which can be found at the top level of
 * the distribution.
 *
 */

#ifndef SRSENB_MAC_METRICS_H
#define SRSENB_MAC_METRICS_H

#include <cstdint>
#include <vector>

namespace srsenb {

/// MAC metrics per user
struct mac_ue_metrics_t {
  uint16_t rnti;
  uint32_t nof_tti;
  uint32_t cc_idx;
  int      tx_pkts;
  int      tx_errors;
  int      tx_brate;
  int      rx_pkts;
  int      rx_errors;
  int      rx_brate;
  int      ul_buffer;
  int      dl_buffer;
  float    dl_cqi;
  float    dl_ri;
  float    dl_pmi;
  float    phr;
};

/// MAC misc information for each cc.
struct mac_cc_info_t {
  /// PCI value.
  uint32_t pci;
  /// RACH preamble counter per cc.
  uint32_t cc_rach_counter;
};

/// Main MAC metrics.
struct mac_metrics_t {
  /// Per CC info.
  std::vector<mac_cc_info_t> cc_info;
  /// Per UE MAC metrics.
  std::vector<mac_ue_metrics_t> ues;
};

} // namespace srsenb

#endif // SRSENB_MAC_METRICS_H
