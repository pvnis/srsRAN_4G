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

#include <bitset>
#include <inttypes.h>
#include <iostream>
#include <string.h>

#include "srsgnb/hdr/stack/mac/ue_nr.h"
#include "srsran/common/buffer_pool.h"
#include "srsran/common/string_helpers.h"
#include "srsran/interfaces/gnb_interfaces.h"

namespace srsenb {

ue_nr::ue_nr(uint16_t                rnti_,
             uint32_t                enb_cc_idx,
             sched_nr_interface*     sched_,
             rrc_interface_mac_nr*   rrc_,
             rlc_interface_mac*      rlc_,
             phy_interface_stack_nr* phy_,
             srslog::basic_logger&   logger_) :
  rnti(rnti_),
  sched(sched_),
  rrc(rrc_),
  rlc(rlc_),
  phy(phy_),
  logger(logger_),
  ue_rlc_buffer(srsran::make_byte_buffer())
{}

ue_nr::~ue_nr() {}

void ue_nr::reset()
{
  {
    std::lock_guard<std::mutex> lock(metrics_mutex);
    ue_metrics = {};
  }
  nof_failures = 0;
}

void ue_nr::ue_cfg(const sched_interface::ue_cfg_t& ue_cfg)
{
  // nop
}

void ue_nr::set_tti(uint32_t tti)
{
  last_tti = tti;
}

uint32_t ue_nr::read_pdu(uint32_t lcid, uint8_t* payload, uint32_t requested_bytes)
{
  return rlc->read_pdu(rnti, lcid, payload, requested_bytes);
}

int ue_nr::generate_pdu(srsran::byte_buffer_t* pdu, uint32_t grant_size)
{
  std::lock_guard<std::mutex> lock(mutex);

  if (mac_pdu_dl.init_tx(pdu, grant_size) != SRSRAN_SUCCESS) {
    logger.error("Couldn't initialize MAC PDU buffer");
    return SRSRAN_ERROR;
  }

  bool drb_activity = false; // inform RRC about user activity if true
  int  lcid         = 4;     // only supporting single DRB right now

  int32_t remaining_len = mac_pdu_dl.get_remaing_len();

  logger.debug("Adding MAC PDU for RNTI=%d (max %d B)", rnti, remaining_len);
  while (remaining_len >= MIN_RLC_PDU_LEN) {
    // clear read buffer
    ue_rlc_buffer->clear();

    // Determine space for RLC
    remaining_len -= remaining_len >= srsran::mac_sch_subpdu_nr::MAC_SUBHEADER_LEN_THRESHOLD ? 3 : 2;

    // read RLC PDU
    int pdu_len = rlc->read_pdu(rnti, lcid, ue_rlc_buffer->msg, remaining_len);

    if (pdu_len > remaining_len) {
      logger.error("Can't add SDU of %d B. Available space %d B", pdu_len, remaining_len);
      break;
    } else {
      // Add SDU if RLC has something to tx
      if (pdu_len > 0) {
        ue_rlc_buffer->N_bytes = pdu_len;
        logger.debug(ue_rlc_buffer->msg, ue_rlc_buffer->N_bytes, "Read %d B from RLC", ue_rlc_buffer->N_bytes);

        // add to MAC PDU and pack
        if (mac_pdu_dl.add_sdu(lcid, ue_rlc_buffer->msg, ue_rlc_buffer->N_bytes) != SRSRAN_SUCCESS) {
          logger.error("Error packing MAC PDU");
          break;
        }

        // set DRB activity flag but only notify RRC once
        if (lcid > 3) {
          drb_activity = true;
        }
      } else {
        break;
      }

      remaining_len -= pdu_len;
      logger.debug("%d B remaining PDU", remaining_len);
    }
  }

  mac_pdu_dl.pack();

  if (drb_activity) {
    // Indicate DRB activity in DL to RRC
    rrc->set_activity_user(rnti);
    logger.debug("DL activity rnti=0x%x", rnti);
  }

  if (logger.info.enabled()) {
    fmt::memory_buffer str_buffer;
    mac_pdu_dl.to_string(str_buffer);
    logger.info("0x%x %s", rnti, srsran::to_c_str(str_buffer));
  }
  return SRSRAN_SUCCESS;
}

/******* METRICS interface ***************/
void ue_nr::metrics_read(mac_ue_metrics_t* metrics_)
{
  uint32_t ul_buffer = 0; // sched->get_ul_buffer(rnti);
  uint32_t dl_buffer = 0; // sched->get_dl_buffer(rnti);

  std::lock_guard<std::mutex> lock(metrics_mutex);
  ue_metrics.rnti      = rnti;
  ue_metrics.ul_buffer = ul_buffer;
  ue_metrics.dl_buffer = dl_buffer;

  // set PCell sector id
  // TODO: use ue_cfg when multiple NR carriers are supported
  ue_metrics.cc_idx = 0;

  *metrics_            = ue_metrics;
  phr_counter          = 0;
  dl_cqi_valid_counter = 0;
  pucch_sinr_counter   = 0;
  pusch_sinr_counter   = 0;
  ue_metrics           = {};
}

void ue_nr::metrics_dl_cqi(const srsran_uci_cfg_nr_t& cfg_, uint32_t dl_cqi)
{
  std::lock_guard<std::mutex> lock(metrics_mutex);

  // Process CQI
  for (uint32_t i = 0; i < cfg_.nof_csi; i++) {
    // Skip if invalid or not supported CSI report
    if (cfg_.csi[i].cfg.quantity != SRSRAN_CSI_REPORT_QUANTITY_CRI_RI_PMI_CQI or
        cfg_.csi[i].cfg.freq_cfg != SRSRAN_CSI_REPORT_FREQ_WIDEBAND) {
      continue;
    }

    // Add statistics
    ue_metrics.dl_cqi = SRSRAN_VEC_SAFE_CMA(dl_cqi, ue_metrics.dl_cqi, dl_cqi_valid_counter);
    dl_cqi_valid_counter++;
  }
}

void ue_nr::metrics_rx(bool crc, uint32_t tbs)
{
  std::lock_guard<std::mutex> lock(metrics_mutex);
  if (crc) {
    ue_metrics.rx_brate += tbs * 8;
  } else {
    ue_metrics.rx_errors++;
  }
  ue_metrics.rx_pkts++;
}

void ue_nr::metrics_tx(bool crc, uint32_t tbs)
{
  std::lock_guard<std::mutex> lock(metrics_mutex);
  if (crc) {
    ue_metrics.tx_brate += tbs * 8;
  } else {
    ue_metrics.tx_errors++;
  }
  ue_metrics.tx_pkts++;
}

void ue_nr::metrics_dl_mcs(uint32_t mcs)
{
  std::lock_guard<std::mutex> lock(metrics_mutex);
  ue_metrics.dl_mcs = SRSRAN_VEC_CMA((float)mcs, ue_metrics.dl_mcs, ue_metrics.dl_mcs_samples);
  ue_metrics.dl_mcs_samples++;
}

void ue_nr::metrics_ul_mcs(uint32_t mcs)
{
  std::lock_guard<std::mutex> lock(metrics_mutex);
  ue_metrics.ul_mcs = SRSRAN_VEC_CMA((float)mcs, ue_metrics.ul_mcs, ue_metrics.ul_mcs_samples);
  ue_metrics.ul_mcs_samples++;
}

void ue_nr::metrics_cnt()
{
  std::lock_guard<std::mutex> lock(metrics_mutex);
  ue_metrics.nof_tti++;
}

void ue_nr::metrics_pucch_sinr(float sinr)
{
  std::lock_guard<std::mutex> lock(metrics_mutex);
  // discard nan or inf values for average SINR
  if (!std::isinf(sinr) && !std::isnan(sinr)) {
    ue_metrics.pucch_sinr = SRSRAN_VEC_SAFE_CMA((float)sinr, ue_metrics.pucch_sinr, pucch_sinr_counter);
    pucch_sinr_counter++;
  }
}

void ue_nr::metrics_pusch_sinr(float sinr)
{
  std::lock_guard<std::mutex> lock(metrics_mutex);
  // discard nan or inf values for average SINR
  if (!std::isinf(sinr) && !std::isnan(sinr)) {
    ue_metrics.pusch_sinr = SRSRAN_VEC_SAFE_CMA((float)sinr, ue_metrics.pusch_sinr, pusch_sinr_counter);
    pusch_sinr_counter++;
  }
}

} // namespace srsenb
