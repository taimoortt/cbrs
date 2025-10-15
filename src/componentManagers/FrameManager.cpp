/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2010,2011,2012,2013 TELEMATICS LAB, Politecnico di Bari
 *
 * This file is part of LTE-Sim
 *
 * LTE-Sim is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 3 as
 * published by the Free Software Foundation;
 *
 * LTE-Sim is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with LTE-Sim; if not, see <http://www.gnu.org/licenses/>.
 *
 * Author: Giuseppe Piro <g.piro@poliba.it>
 */

#include "FrameManager.h"
#include "../core/spectrum/bandwidth-manager.h"
#include "../device/ENodeB.h"
#include "../device/HeNodeB.h"
#include "../device/UserEquipment.h"
#include "../flows/application/Application.h"
#include "../flows/radio-bearer.h"
#include "../load-parameters.h"
#include "../phy/enb-lte-phy.h"
#include "../protocolStack/mac/AMCModule.h"
#include "../protocolStack/mac/packet-scheduler/downlink-packet-scheduler.h"
#include "../protocolStack/mac/packet-scheduler/radiosaber-downlink-scheduler.h"
#include "../utility/eesm-effective-sinr.h"
#include <algorithm>
#include <cassert>
#include <cmath>
#include <cstdio>
#include <random>
#include <sstream>
#include <stdexcept>
#include <unordered_set>
using SchedulerAlgo = SliceContext::SchedulerAlgo;
bool weighted_pf = false;

int min_number_users_per_cell = 5;

FrameManager *FrameManager::ptr = NULL;

FrameManager::FrameManager() {
  m_nbFrames = 0;
  m_nbSubframes = 0;
  m_TTICounter = 0;
  m_frameStructure = FrameManager::FRAME_STRUCTURE_FDD; // Default Value
  m_TDDFrameConfiguration = 1;                          // Default Value
  Simulator::Init()->Schedule(0.0, &FrameManager::Start, this);
}

FrameManager::~FrameManager() {}

void FrameManager::SetFrameStructure(
    FrameManager::FrameStructure frameStructure) {
  m_frameStructure = frameStructure;
}

FrameManager::FrameStructure FrameManager::GetFrameStructure(void) const {
  return m_frameStructure;
}

void FrameManager::SetTDDFrameConfiguration(int configuration) {
  if (configuration < 0 && configuration > 6) {
    m_TDDFrameConfiguration = 0; // Default Value
  } else {
    m_TDDFrameConfiguration = configuration;
  }
}

int FrameManager::GetTDDFrameConfiguration(void) const {
  return m_TDDFrameConfiguration;
}

int FrameManager::GetSubFrameType(int nbSubFrame) {
  return TDDConfiguration[GetTDDFrameConfiguration()][nbSubFrame - 1];
}

void FrameManager::UpdateNbFrames(void) { m_nbFrames++; }

int FrameManager::GetNbFrames(void) const { return m_nbFrames; }

void FrameManager::UpdateNbSubframes(void) { m_nbSubframes++; }

void FrameManager::ResetNbSubframes(void) { m_nbSubframes = 0; }

int FrameManager::GetNbSubframes(void) const { return m_nbSubframes; }

void FrameManager::UpdateTTIcounter(void) { m_TTICounter++; }

unsigned long FrameManager::GetTTICounter() const { return m_TTICounter; }

void FrameManager::Start(void) {
#ifdef FRAME_MANAGER_DEBUG
  std::cout << " LTE Simulation starts now! " << std::endl;
#endif

  Simulator::Init()->Schedule(0.0, &FrameManager::StartFrame, this);
}

void FrameManager::StartFrame(void) {
  UpdateNbFrames();

#ifdef FRAME_MANAGER_DEBUG
  std::cout << " +++++++++ Start Frame, time =  " << Simulator::Init()->Now()
            << " +++++++++" << std::endl;
#endif

  Simulator::Init()->Schedule(0.0, &FrameManager::StartSubframe, this);
}

void FrameManager::StopFrame(void) {
  Simulator::Init()->Schedule(0.0, &FrameManager::StartFrame, this);
}

void FrameManager::StartSubframe(void) {
  // #ifdef FRAME_MANAGER_DEBUG
  std::cout << " --------- Start SubFrame, time =  " << Simulator::Init()->Now()
            << " --------- "
            << "TTI: " << GetTTICounter() << std::endl;
  // #endif

  // Prints UE positioning logs after the handovers have occured.
  if (GetTTICounter() == 120) {
    std::vector<ENodeB *> *enodebs = GetNetworkManager()->GetENodeBContainer();
    cout << "ENB Size: " << enodebs->size() << endl;
    for (auto iter = enodebs->begin(); iter != enodebs->end(); iter++) {
      ENodeB *enb = *iter;
      ENodeB::UserEquipmentRecords *ue_records = enb->GetUserEquipmentRecords();
      cout << "UE Records Size: " << ue_records->size() << endl;
      for (size_t i = 0; i < ue_records->size(); i++) {
        UserEquipment *x = (*ue_records)[i]->GetUE();
        cout << GetTTICounter();
        x->Print();
      }
      // if (ue_records->size() < min_number_users_per_cell){
      //   cout << "UE Attachment too low for Cell: " << enb->GetIDNetworkNode()
      // << "\t No of UEs: " << ue_records->size() << endl;
      //   cout << "Central Downlink RBs Allocation!" << endl;
      //   std::exit(0);
      // }
    }
  }

  if (GetTTICounter() == 2990) {
    std::vector<ENodeB *> *enodebs = GetNetworkManager()->GetENodeBContainer();
    cout << "ENB Size: " << enodebs->size() << endl;
    for (auto iter = enodebs->begin(); iter != enodebs->end(); iter++) {
      ENodeB *enb = *iter;
      ENodeB::UserEquipmentRecords *ue_records = enb->GetUserEquipmentRecords();
      cout << "UE Records Size: " << ue_records->size() << endl;
      for (size_t i = 0; i < ue_records->size(); i++) {
        UserEquipment *x = (*ue_records)[i]->GetUE();
        cout << GetTTICounter();
        x->Print();
      }
      // if (ue_records->size() < min_number_users_per_cell){
      //   cout << "UE Attachment too low for Cell: " << enb->GetIDNetworkNode()
      // << "\t No of UEs: " << ue_records->size() << endl;
      //   cout << "Central Downlink RBs Allocation!" << endl;
      //   std::exit(0);
      // }
    }
  }

  UpdateTTIcounter();
  UpdateNbSubframes();

  /*
   * At the beginning of each sub-frame the simulation
   * update user position. This function could be
   * implemented also every TTI.
   */
  // UpdateUserPosition(); --> moved to each UE class

#ifdef PLOT_USER_POSITION
  NetworkManager::Init()->PrintUserPosition();
#endif

  /*
   * According to the Frame Structure, the DW/UL link scheduler
   * will be called for each sub-frame.
   * (RBs allocation)
   */
  Simulator::Init()->Schedule(0, &FrameManager::CentralResourceAllocation,
                              this);
  Simulator::Init()->Schedule(0.001, &FrameManager::StopSubframe, this);
}

void FrameManager::StopSubframe(void) {
  if (GetNbSubframes() == 10) {
    ResetNbSubframes();
    Simulator::Init()->Schedule(0.0, &FrameManager::StopFrame, this);
  } else {
    Simulator::Init()->Schedule(0.0, &FrameManager::StartSubframe, this);
  }
}

NetworkManager *FrameManager::GetNetworkManager(void) {
  return NetworkManager::Init();
}

void FrameManager::UpdateUserPosition(void) {
  GetNetworkManager()->UpdateUserPosition(GetTTICounter() / 1000.0);
}

void FrameManager::CentralResourceAllocation(void) {
  std::vector<ENodeB *> *enodebs = GetNetworkManager()->GetENodeBContainer();
  assert(GetFrameStructure() == FrameManager::FRAME_STRUCTURE_FDD);
#ifdef FRAME_MANAGER_DEBUG
  // std::cout << "Resource Allocation for eNB:";
  // for (auto iter = enodebs->begin (); iter != enodebs->end (); iter++) {
  // ENodeB* enb = *iter;
  // std::cout << " " << enb->GetIDNetworkNode();
  // }
  // std::cout << std::endl;
#endif

#ifdef SET_CENTRAL_SCHEDULER
  // std::cout << "Central Downlink RBs Allocation!" << std::endl;
  CentralDownlinkRBsAllocation();
#else
  std::cout << "Original Per-Cell Downlink RBs Allocation!" << std::endl;
  for (auto iter = enodebs->begin(); iter != enodebs->end(); iter++) {
    ENodeB *enb = *iter;
    enb->DownlinkResourceBlockAllocation();
  }
#endif
  for (auto iter = enodebs->begin(); iter != enodebs->end(); iter++) {
    ENodeB *enb = *iter;
    enb->UplinkResourceBlockAllocation();
  }
}

void FrameManager::CentralDownlinkRBsAllocation(void) {
  std::vector<ENodeB *> *enodebs = GetNetworkManager()->GetENodeBContainer();
  std::vector<DownlinkPacketScheduler *> schedulers;
  // initialization
  int counter = 0;
  for (auto it = enodebs->begin(); it != enodebs->end(); it++) {
    ENodeB *enb = *it;
    DownlinkPacketScheduler *scheduler =
        (DownlinkPacketScheduler *)enb->GetDLScheduler();
    assert(counter == enb->GetIDNetworkNode());
    schedulers.push_back(scheduler);
    counter += 1;
  }
  // set up schedulers
  bool available_flow = false;
  for (size_t j = 0; j < schedulers.size(); j++) {
    schedulers[j]->UpdateAverageTransmissionRate();
    schedulers[j]->SelectFlowsToSchedule();
    if (schedulers[j]->GetFlowsToSchedule()->size() > 0) {
      available_flow = true;
    }
    auto flows = schedulers[j]->GetFlowsToSchedule();
    if (GetTTICounter() < 102)
      std::cout << "cell: " << j << " n_flows: " << flows->size() << std::endl;
  }
  // if no flow is available in any slice, stop schedulers
  if (!available_flow) {
    for (size_t j = 0; j < schedulers.size(); j++)
      schedulers[j]->StopSchedule();
    return;
  }
  // Handle asymmetric RB allocation - each cell may have different number of
  // RBs
  for (size_t j = 0; j < schedulers.size(); j++) {
    int nb_of_rbs = schedulers[j]
                        ->GetMacEntity()
                        ->GetDevice()
                        ->GetPhy()
                        ->GetBandwidthManager()
                        ->GetDlSubChannels()
                        .size();
    // print DL Subchannels for each cell
    if (GetTTICounter() < 102) {
      std::cout << "Cell " << j << " DL Subchannels size: " << nb_of_rbs
                << "\t";
      auto dl_subchannels = schedulers[j]
                                ->GetMacEntity()
                                ->GetDevice()
                                ->GetPhy()
                                ->GetBandwidthManager()
                                ->GetDlSubChannels();
      std::cout << "Freqs: ";
      for (auto subchannel : dl_subchannels) {
        std::cout << subchannel << " ";
      }
      std::cout << std::endl;
    }
    int rb_id = 0;
    while (rb_id < nb_of_rbs) { // Allocate in the form of PRBG of size RBG_SIZE
      auto *bm = schedulers[j]
                     ->GetMacEntity()
                     ->GetDevice()
                     ->GetPhy()
                     ->GetBandwidthManager();
      const std::vector<double> &dl_subchannels = bm->GetDlSubChannels();
      std::vector<int> global_indices = bm->GetDlGlobalRbIndices();
      int group_size = std::min(RBG_SIZE, nb_of_rbs - rb_id);
      // std::ostringstream oss;
      // oss << "Scheduling RBG: local_start=" << rb_id << " size=" <<
      // group_size; for (int li = rb_id; li < rb_id + group_size; ++li) {
      //   double freq = dl_subchannels[li];
      //   int g = (li < (int)global_indices.size()) ? global_indices[li] : -1;
      //   oss << " [l=" << li << " g=" << g << " fMHz=" << freq << "]";
      // }
      // std::cout << oss.str() << std::endl;
      AllocateRBs(schedulers, rb_id, j);
      rb_id += RBG_SIZE;
    }
  }
  for (size_t j = 0; j < schedulers.size(); j++) {
    schedulers[j]->FinalizeScheduledFlows();
    schedulers[j]->StopSchedule();
  }
}

void FrameManager::AllocateRBs(
    std::vector<DownlinkPacketScheduler *> &schedulers, int rb_id,
    int cell_index) {
  // Only allocate for the specific cell
  if (cell_index >= (int)schedulers.size()) {
    return;
  }

  RadioSaberDownlinkScheduler *scheduler =
      (RadioSaberDownlinkScheduler *)schedulers[cell_index];
  FlowsToSchedule *flows = scheduler->GetFlowsToSchedule();

  if (flows->size() == 0) {
    std::cout << "ERROR: No flows to schedule for cell: " << cell_index
              << std::endl;
    return; // No flows to schedule for this cell
  }

  AMCModule *amc = scheduler->GetMacEntity()->GetAmcModule();
  double max_metric = 0;
  FlowToSchedule *max_flow = nullptr;

  for (auto it = flows->begin(); it != flows->end(); it++) {
    FlowToSchedule *flow = *it;
    double spectraleff_rbg = 0.0;
    for (int i = rb_id; i < RBG_SIZE + rb_id; i++) {
      int cqi = flow->GetCqiFeedbacks().at(i);
      double se = amc->GetEfficiencyFromCQI(cqi);
      spectraleff_rbg += se;
    }
    double metric = scheduler->ComputeSchedulingMetric(flow->GetBearer(),
                                                       spectraleff_rbg, rb_id);
    if (metric > max_metric) {
      max_metric = metric;
      max_flow = flow;
    }
  }

  // Skip allocation if no flow is available for this scheduler
  if (max_flow == nullptr) {
    cout << "WARNING: No flow selected for cell: " << cell_index
         << " at RB: " << rb_id << std::endl;
    return;
  }

  scheduler->slice_rbgs_quota_[max_flow->GetSliceID()] -= 1;
  for (int i = rb_id; i < RBG_SIZE + rb_id; i++) {
    max_flow->GetListOfAllocatedRBs()->push_back(i);
  }
}
