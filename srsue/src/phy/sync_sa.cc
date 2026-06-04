/**
 * Copyright 2013-2023 Software Radio Systems Limited
 *
 * This file is part of srsRAN.
 *
 * srsRAN is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Affero General Public License as
 * published by the Free Software Foundation, either version 3 of
 * the License, or (at your option) any later version.
 *
 * srsRAN is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Affero General Public License for more details.
 *
 * A copy of the GNU Affero General Public License can be found in
 * the LICENSE file in the top-level directory of this distribution
 * and at http://www.gnu.org/licenses/.
 *
 */

#include "srsue/hdr/phy/nr/sync_sa.h"
#include "srsran/radio/rf_buffer.h"

namespace srsue {
namespace nr {
sync_sa::sync_sa(srslog::basic_logger& logger_, worker_pool& workers_) :
  logger(logger_), workers(workers_), slot_synchronizer(logger_), searcher(logger_), ta(logger_), srsran::thread("SYNC")
{}

sync_sa::~sync_sa()
{
  if (rx_buffer != nullptr) {
    free(rx_buffer);
  }
}

bool sync_sa::init(const args_t& args, stack_interface_phy_nr* stack_, srsran::radio_interface_phy* radio_)
{
  stack    = stack_;
  radio    = radio_;
  srate_hz = args.srate_hz;
  slot_sz  = (uint32_t)(args.srate_hz / 1000.0f);

  // Initialise cell search internal object
  if (not searcher.init(args.get_cell_search())) {
    logger.error("Error initialising cell searcher");
    return false;
  }

  // Initialise slot synchronizer object
  if (not slot_synchronizer.init(args.get_slot_sync(), stack, radio)) {
    logger.error("Error initialising slot synchronizer");
    return false;
  }

  // Cell bandwidth must be provided at init so set now sampling rate
  radio->set_rx_srate(args.srate_hz);
  radio->set_tx_srate(args.srate_hz);

  // Compute subframe size
  slot_sz = (uint32_t)(args.srate_hz / 1000.0f);

  // Default to one slot per subframe (SCS-15); refined per cell numerology in cell_select_run()
  slots_per_subframe = 1;
  slot_len           = slot_sz;

  // Allocate receive buffer
  rx_buffer = srsran_vec_cf_malloc(2 * slot_sz);
  if (rx_buffer == nullptr) {
    logger.error("Error allocating buffer");
    return false;
  }

  // Thread control
  running = true;
  start(args.thread_priority);

  // If reached here it was successful
  return true;
}

void sync_sa::stop()
{
  running = false;
  wait_thread_finish();
  radio->reset();
}

bool sync_sa::reset()
{
  // Wait worker pool to finish any processing
  tti_semaphore.wait_all();
  ta.set_base_sec(0);

  return true;
}

void sync_sa::add_ta_cmd_rar(uint32_t tti_, uint32_t ta_cmd)
{
  ta.add_ta_cmd_rar(tti_, ta_cmd);
}

void sync_sa::add_ta_cmd_new(uint32_t tti_, uint32_t ta_cmd)
{
  ta.add_ta_cmd_new(tti_, ta_cmd);
}

void sync_sa::add_ta_offset(uint32_t ta_offset)
{
  ta.add_ta_offset(ta_offset);
}

void sync_sa::cell_go_idle()
{
  std::unique_lock<std::mutex> ul(rrc_mutex);
  phy_state.go_idle();
}

bool sync_sa::wait_idle()
{
  // Wait for SYNC thread to transition to IDLE (max. 2000ms)
  if (!phy_state.wait_idle(100)) {
    return false;
  }

  // Reset UE sync. Attention: doing this reset when the FSM is NOT IDLE can cause PSS/SSS out-of-sync
  //...

  // Wait for workers to finish PHY processing
  tti_semaphore.wait_all();

  // As workers have finished, make sure the Tx burst is ended
  radio->tx_end();

  return phy_state.is_idle();
}

cell_search::ret_t sync_sa::cell_search_run(const cell_search::cfg_t& cfg)
{
  std::unique_lock<std::mutex> ul(rrc_mutex);

  cs_ret        = {};
  cs_ret.result = cell_search::ret_t::ERROR;

  // Wait the FSM to transition to IDLE
  if (!wait_idle()) {
    logger.error("Cell Search: SYNC thread didn't transition to IDLE after 100 ms\n");
    return cs_ret;
  }

  rrc_proc_state = PROC_SEARCH_RUNNING;

  // tune radio
  logger.info("Tuning Rx channel %d to %.2f MHz", 0, cfg.center_freq_hz / 1e6);
  radio->set_rx_freq(0, cfg.center_freq_hz);

  if (not searcher.start(cfg)) {
    logger.error("Sync: failed to start cell search");
    return cs_ret;
  }

  logger.info("Cell Search: Running Cell search state");
  cell_search_nof_trials = 0;
  phy_state.run_cell_search();

  rrc_proc_state = PROC_IDLE;

  return cs_ret;
}

rrc_interface_phy_nr::cell_select_result_t sync_sa::cell_select_run(const phy_interface_rrc_nr::cell_select_args_t& req)
{
  std::unique_lock<std::mutex> ul(rrc_mutex);

  // By default, the result is set to error
  rrc_interface_phy_nr::cell_select_result_t result = {};
  result.status                                     = rrc_interface_phy_nr::cell_select_result_t::ERROR;

  // Wait the FSM to transition to IDLE
  if (!wait_idle()) {
    logger.error("Cell Search: SYNC thread didn't transition to IDLE after 100 ms\n");

    // Return default result with error status
    return result;
  }

  rrc_proc_state = PROC_SELECT_RUNNING;

  // tune radio
  logger.info("Tuning Rx channel %d to %.2f MHz", 0, req.carrier.dl_center_frequency_hz / 1e6);
  radio->set_rx_freq(0, req.carrier.dl_center_frequency_hz);
  logger.info("Tuning Tx channel %d to %.2f MHz", 0, req.carrier.ul_center_frequency_hz / 1e6);
  radio->set_tx_freq(0, req.carrier.ul_center_frequency_hz);

  // Derive the cell numerology (slots per subframe). SCS-15 -> 1 slot/subframe (1ms),
  // SCS-30 -> 2 slots/subframe (0.5ms each). The SSB tracker still works per 1ms subframe,
  // but the data-plane workers must be dispatched per slot.
  slots_per_subframe = SRSRAN_NSLOTS_PER_SF_NR(req.carrier.scs);
  slot_len           = slot_sz / slots_per_subframe;
  ta.set_numerology(req.carrier.scs); // RAR/MAC TA commands scale as 1/2^mu
  // Absolute slot index space for this numerology: nslots_per_frame * 1024 SFN. For SCS-15 this is
  // 10240 (matching TTI_ADD); for SCS-30 it is 20480, so the SFN derived from the slot index is correct.
  nof_slots_hyperframe = SRSRAN_NSLOTS_PER_FRAME_NR(req.carrier.scs) * 1024;

  // Configure cell
  srsran_ue_sync_nr_cfg_t cfg = {};
  cfg.N_id                    = req.carrier.pci;
  cfg.ssb                     = req.ssb_cfg;
  cfg.ssb.srate_hz            = srate_hz;
  if (slot_synchronizer.set_sync_cfg(cfg)) {
    logger.error("Cell Search: Failed setting slot synchronizer configuration");

    // Return default result with error status
    return result;
  }

  // SFN synchronization
  phy_state.run_sfn_sync();

  // Determine if the procedure was successful if the current state is camping, otherwise it is unsuccessful
  if (phy_state.is_camping()) {
    logger.info("Cell Select: SFN synchronized. CAMPING...");
    result.status = rrc_interface_phy_nr::cell_select_result_t::SUCCESSFUL;
  } else {
    logger.info("Cell Select: Could not synchronize SFN");
    result.status = rrc_interface_phy_nr::cell_select_result_t::UNSUCCESSFUL;
  }

  rrc_proc_state = PROC_IDLE;
  return result;
}

sync_state::state_t sync_sa::get_state()
{
  return phy_state.get_state();
}

void sync_sa::run_state_idle()
{
  if (not radio->is_init()) {
    return;
  }

  logger.debug("Discarding samples and sending tx_end");
  srsran::rf_buffer_t rf_buffer = {};
  rf_buffer.set_nof_samples(slot_sz);
  rf_buffer.set(0, rx_buffer);
  if (not slot_synchronizer.recv_callback(rf_buffer, last_rx_time.get_ptr(0))) {
    logger.error("SYNC: receiving from radio\n");
  }
  radio->tx_end();
}

void sync_sa::run_state_cell_search()
{
  // Initialise buffer
  if (cell_search_nof_trials == 0) {
    srsran_vec_cf_zero(rx_buffer, slot_sz);
  }

  // Receive samples
  srsran::rf_buffer_t rf_buffer = {};
  rf_buffer.set_nof_samples(slot_sz);
  rf_buffer.set(0, rx_buffer + slot_sz);
  if (not slot_synchronizer.recv_callback(rf_buffer, last_rx_time.get_ptr(0))) {
    logger.error("SYNC: receiving from radio\n");
  }

  // Run Searcher
  cs_ret = searcher.run_slot(rx_buffer, slot_sz);
  if (cs_ret.result < 0) {
    logger.error("Failed to run searcher. Transitioning to IDLE...");
  }

  srsran_vec_cf_copy(rx_buffer, rx_buffer + slot_sz, slot_sz);

  cell_search_nof_trials++;

  // Leave CELL_SEARCH state if error or success and transition to IDLE
  if (cs_ret.result == cell_search::ret_t::CELL_FOUND || cell_search_nof_trials >= cell_search_max_trials) {
    phy_state.state_exit();
  }
}

void sync_sa::run_state_sfn_sync()
{
  // Run SFN synchronization
  if (slot_synchronizer.run_sfn_sync()) {
    tti = slot_synchronizer.get_slot_cfg().idx;

    logger.info("SYNC: SFN synchronised successfully (SFN=%d). Transitioning to IDLE...",
                tti / (slots_per_subframe * SRSRAN_NOF_SF_X_FRAME));

    phy_state.state_exit(true);
    return;
  }

  // If not synchonized, increment number of trials
  sfn_sync_nof_trials++;

  // Abort SFN synchronization if the maximum number of trials is reached
  if (sfn_sync_nof_trials >= sfn_sync_max_trials) {
    logger.info("SYNC: The SFN sync reached the maximum number of trials (%d). Transitioning to IDLE...",
                sfn_sync_nof_trials);
    phy_state.state_exit(false);
  }
}

void sync_sa::run_state_cell_camping()
{
  // The SSB tracker receives and tracks one full subframe (slot_sz = 1 ms) at a time. For SCS-15
  // a subframe is one slot; for SCS-30 it is two slots. We receive the whole subframe into the
  // first worker's buffer, then dispatch one worker per slot with the correct absolute slot index.
  nr::sf_worker* rx_worker = workers.wait_worker(tti);
  if (rx_worker == nullptr) {
    running = false;
    return;
  }

  // Receive a whole subframe and track the SSB (CFO-corrected in place into rx_worker's buffer)
  srsran::rf_buffer_t rf_buffer = {};
  rf_buffer.set_nof_samples(slot_sz);
  rf_buffer.set(0, rx_worker->get_buffer(0, 0));
  if (not slot_synchronizer.run_camping(rf_buffer, last_rx_time)) {
    logger.error("SYNC: detected out-of-sync... skipping slot ...");
    is_pending_tx_end = true;
    rx_worker->release();
    return;
  }

  // Dispatch one worker per slot contained in the received subframe
  for (uint32_t slot = 0; slot < slots_per_subframe; slot++) {
    nr::sf_worker* nr_worker = nullptr;
    if (slot == 0) {
      // First slot is already at offset 0 of the receive worker's buffer
      nr_worker = rx_worker;
    } else {
      nr_worker = workers.wait_worker(tti);
      if (nr_worker == nullptr) {
        running = false;
        return;
      }
      // Copy this slot's slice from the receive buffer into the new worker's buffer
      srsran_vec_cf_copy(
          nr_worker->get_buffer(0, 0), rx_worker->get_buffer(0, 0) + slot * slot_len, slot_len);
    }

    srsran::phy_common_interface::worker_context_t context;
    context.sf_idx     = tti;
    context.worker_ptr = nr_worker;
    context.last       = true; // Set last if standalone

    // Receive time of this slot and the corresponding transmit time
    srsran::rf_timestamp_t slot_tx_time = {};
    slot_tx_time.copy(last_rx_time);
    slot_tx_time.add((double)(slot * slot_len) / srate_hz);
    // The DL->UL TX advance is FDD_HARQ_DELAY_DL_MS *slots* (TTI_TX adds it in slot units), not milliseconds.
    // At SCS-15 one slot == 1 ms so this equals the legacy 4 ms; at SCS-30 it must be 4 slots = 2 ms, otherwise the
    // PRACH/UL lands FDD_HARQ_DELAY_DL_MS*(2^mu - 1) slots past the gNB's RX window (PRACH rssi=-inf).
    slot_tx_time.add((double)FDD_HARQ_DELAY_DL_MS * (double)slot_len / srate_hz);
    if (const char* a = getenv("SRSUE_UL_ADV_US")) {
      slot_tx_time.add(atof(a) * 1e-6); // diagnostic: extra UL advance in microseconds
    }
    // Apply current TA
    slot_tx_time.sub((double)ta.get_sec());
    context.tx_time.copy(slot_tx_time);

    if (getenv("SRSUE_TXT_DBG") && (tti % (slots_per_subframe * 10) == 18 || tti % (slots_per_subframe * 10) == 14)) {
      fprintf(stderr, "TXT tti=%u slot=%u rx=%.6f slot_len=%u srate=%.0f adv=%.6f ta=%.6f -> tx=%.6f\n",
              tti, slot, last_rx_time.get(0).full_secs + last_rx_time.get(0).frac_secs, slot_len, srate_hz,
              FDD_HARQ_DELAY_DL_MS * (double)slot_len / srate_hz, ta.get_sec(),
              slot_tx_time.get(0).full_secs + slot_tx_time.get(0).frac_secs);
    }

    nr_worker->set_context(context);

    // NR worker needs to be launched first, phy_common::worker_end expects first the NR worker and the LTE worker.
    tti_semaphore.push(nr_worker);
    workers.start_worker(nr_worker);

    // Advance the absolute slot index within the numerology-aware hyperframe (SCS-15: mod 10240,
    // SCS-30: mod 20480) so the SFN derived downstream (idx / nslots_per_frame) stays correct.
    tti = (tti + 1) % nof_slots_hyperframe;
  }
}

void sync_sa::run_thread()
{
  while (running.load(std::memory_order_relaxed)) {
    logger.set_context(tti);

    logger.debug("SYNC:  state=%s, tti=%d", phy_state.to_string(), tti);

    switch (phy_state.run_state()) {
      case sync_state::IDLE:
        run_state_idle();
        break;
      case sync_state::CELL_SEARCH:
        run_state_cell_search();
        break;
      case sync_state::SFN_SYNC:
        run_state_sfn_sync();
        break;
      case sync_state::CAMPING:
        run_state_cell_camping();
        break;
    }
  }
}
void sync_sa::worker_end(const srsran::phy_common_interface::worker_context_t& w_ctx,
                         const bool&                                           tx_enable,
                         srsran::rf_buffer_t&                                  tx_buffer)
{
  // Wait for the green light to transmit in the current TTI
  tti_semaphore.wait(w_ctx.worker_ptr);

  // Add current time alignment
  srsran::rf_timestamp_t tx_time = w_ctx.tx_time; // get transmit time from the last worker
  // todo: tx_time.sub((double)ta.get_sec());

  // Check if any worker had a transmission
  if (tx_enable) {
    // Actual baseband transmission
    radio->tx(tx_buffer, tx_time);
  } else {
    if (radio->is_continuous_tx()) {
      if (is_pending_tx_end) {
        radio->tx_end();
        is_pending_tx_end = false;
      } else {
        if (!radio->get_is_start_of_burst()) {
          srsran::rf_buffer_t zeros_multi;
          zeros_multi.set_nof_samples(tx_buffer.get_nof_samples());
          radio->tx(zeros_multi, tx_time);
        }
      }
    } else {
      radio->tx_end();
    }
  }

  // Allow next TTI to transmit
  tti_semaphore.release();
}

} // namespace nr
} // namespace srsue
