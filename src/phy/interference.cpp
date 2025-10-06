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

#include "interference.h"
#include "../componentManagers/NetworkManager.h"
#include "../core/eventScheduler/simulator.h"
#include "../core/spectrum/bandwidth-manager.h"
#include "../device/ENodeB.h"
#include "../device/HeNodeB.h"
#include "../device/UserEquipment.h"
#include "../utility/ComputePathLoss.h"
#include "lte-phy.h"

Interference::Interference() {}

Interference::~Interference() {}

std::map<int, double> Interference::ComputeInterference(UserEquipment *ue) {
  ENodeB *node;

  std::map<int, double> rsrp;
  double tot_interference = 0;

  std::vector<ENodeB *> *eNBs = NetworkManager::Init()->GetENodeBContainer();
  std::vector<ENodeB *>::iterator it;

  int tti = Simulator::Init()->Now() * 1000;
  for (it = eNBs->begin(); it != eNBs->end(); it++) {
    node = (*it);
    if (node->GetPhy()->GetBandwidthManager()->GetDlOffsetBw() ==
        ue->GetTargetNode()->GetPhy()->GetBandwidthManager()->GetDlOffsetBw()) {
      // cout << "eNB " << node->GetIDNetworkNode() << " and UE "
      //      << ue->GetIDNetworkNode() << " are in the same band - Cell: "
      //      << node->GetPhy()->GetBandwidthManager()->GetDlOffsetBw() << " UE:
      //      "
      //      << ue->GetTargetNode()
      //             ->GetPhy()
      //             ->GetBandwidthManager()
      //             ->GetDlOffsetBw()
      //      << endl;
      double powerTx = pow(10., (node->GetPhy()->GetTxPower() - 30) / 10);
      double powerTXForSubBandwidth =
          10 *
          log10(
              powerTx /
              node->GetPhy()->GetBandwidthManager()->GetDlSubChannels().size());
      double receivedPower_db =
          powerTXForSubBandwidth - 10 -
          ComputePathLossForInterference(node, ue); // in dB
      double nodeRSRP = pow(10, receivedPower_db / 10);

      rsrp[node->GetIDNetworkNode()] = nodeRSRP;
      if (node->GetIDNetworkNode() != ue->GetTargetNode()->GetIDNetworkNode()) {
        tot_interference += (nodeRSRP);
#ifdef INTERFERENCE_DEBUG
// std::cerr << tti << " UE(" << ue->GetIDNetworkNode() << ")"
//   << " interference from eNB " << node->GetIDNetworkNode()
//   << ": " << receivedPower_db << " interfere(watt): " << nodeRSRP
//   << std::endl;
#endif
      } else {
#ifdef INTERFERENCE_DEBUG
        std::cout
            << tti << " UE(" << ue->GetIDNetworkNode() << ")"
            << " RSRP from serving eNB " << node->GetIDNetworkNode() << ": "
            << receivedPower_db << " RSRP(watt): " << nodeRSRP
            << " tx_power(db) " << powerTXForSubBandwidth << " path_loss(db) "
            << ComputePathLossForInterference(node, ue)
            << " Total Interference: " << tot_interference << " distance(m) "
            << node->GetMobilityModel()->GetAbsolutePosition()->GetDistance(
                   ue->GetMobilityModel()->GetAbsolutePosition())
            << std::endl;
#endif
      }
    }
  }
  return rsrp;
}
