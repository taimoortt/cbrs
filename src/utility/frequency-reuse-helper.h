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

#ifndef FREQUENCY_REUSE_HELPER_H_
#define FREQUENCY_REUSE_HELPER_H_

#include "../componentManagers/FrameManager.h"
#include "../core/spectrum/bandwidth-manager.h"
#include "stdlib.h"
#include <math.h>
#include <stdint.h>

/*

Number of supported non-overlapping channels in each frequency band and
bandwidth.

operative band		bandwidth 			Channel bandwidth (MHz)
                                                                                1.4 3 	5 	10 	15 	20

1 						60
— 	— 	12 	6 	4 	3 2
60 				42 	20 	12 	6	[4] [3] 3
75 				53 	23 	15 	7 	[5] [3]
...

XXX: now is supported only the 1-th operative sub-band
*/

static std::vector<BandwidthManager *>
RunFrequencyReuseTechniques(int nodes, bool reuse, double bandwidth,
                            int default_offset = 0) {
  std::vector<BandwidthManager *> spectrum;

  if (FrameManager::Init()->GetFrameStructure() ==
      FrameManager::FRAME_STRUCTURE_FDD) {
    // Calculate total RBs available for the given bandwidth
    int totalRBs = 0;
    double cellBandwidth = 0;
    cout << "Bandwidth: " << bandwidth << " MHz" << std::endl;

    if (bandwidth == 1.4) {
      totalRBs = 6;
    } else if (bandwidth == 3) {
      totalRBs = 15;
    } else if (bandwidth == 5) {
      totalRBs = 25;
    } else if (bandwidth == 10) {
      totalRBs = 50;
    } else if (bandwidth == 15) {
      totalRBs = 75;
    } else if (bandwidth == 20) {
      totalRBs = 100;
    } else if (bandwidth == 50) {
      totalRBs = 248;
    } else if (bandwidth == 100) {
      totalRBs = 496;
    } else {
      // Default to 5MHz
      totalRBs = 25;
      bandwidth = 5;
    }

    // Calculate RBs per cell and handle remainder
    int rbsPerCell = 0;
    int remainderRBs = 0;
    if (reuse) {
      // Full reuse: every cell uses the entire RB set
      rbsPerCell = totalRBs;
      remainderRBs = 0;
    } else {
      // Static partitioning: divide equally and spread remainders
      rbsPerCell = totalRBs / nodes;
      remainderRBs = totalRBs % nodes;
    }

    std::cout << "Total RBs: " << totalRBs << ", Nodes: " << nodes
              << ", RBs per cell: " << rbsPerCell
              << ", Remainder: " << remainderRBs << std::endl;

    int currentOffset = default_offset;
    for (int i = 0; i < nodes; i++) {
      int cellRBs = rbsPerCell;

      int cellOffset = currentOffset;
      if (!reuse) {
        // Give remainder RBs to the last cell(s). You could also choose the
        // first ones.
        if (i >= nodes - remainderRBs) {
          cellRBs++;
        }
      } else {
        // Reuse: all cells share the same RBs and offset
        cellOffset = default_offset;
      }

      std::cout << "Cell " << i << ": RBs=" << cellRBs
                << ", Offset=" << cellOffset << (reuse ? " (reuse)" : "")
                << std::endl;

      // Use explicit RB-count constructor so the subchannel list length matches
      // cellRBs
      BandwidthManager *s = new BandwidthManager(
          bandwidth, bandwidth, cellOffset, cellOffset, cellRBs, cellRBs);
      spectrum.push_back(s);

      // Advance offset only when statically partitioning
      if (!reuse) {
        currentOffset += cellRBs;
      }
    }
  } else // case TDD
  {
    // Calculate total RBs available for the given bandwidth
    int totalRBs = 0;
    double cellBandwidth = 0;

    if (bandwidth == 1.4) {
      totalRBs = 6;
    } else if (bandwidth == 3) {
      totalRBs = 15;
    } else if (bandwidth == 5) {
      totalRBs = 25;
    } else if (bandwidth == 10) {
      totalRBs = 50;
    } else if (bandwidth == 15) {
      totalRBs = 75;
    } else if (bandwidth == 20) {
      totalRBs = 100;
    } else if (bandwidth == 50) {
      totalRBs = 248;
    } else if (bandwidth == 100) {
      totalRBs = 496;
    } else {
      // Default to 5MHz
      totalRBs = 25;
      bandwidth = 5;
    }

    // Calculate RBs per cell and handle remainder
    int rbsPerCell = 0;
    int remainderRBs = 0;
    if (reuse) {
      // Full reuse
      rbsPerCell = totalRBs;
      remainderRBs = 0;
    } else {
      rbsPerCell = totalRBs / nodes;
      remainderRBs = totalRBs % nodes;
    }

    std::cout << "TDD - Total RBs: " << totalRBs << ", Nodes: " << nodes
              << ", RBs per cell: " << rbsPerCell
              << ", Remainder: " << remainderRBs << std::endl;

    int currentOffset = default_offset;
    for (int i = 0; i < nodes; i++) {
      int cellRBs = rbsPerCell;

      int cellOffset = currentOffset;
      if (!reuse) {
        if (i >= nodes - remainderRBs) {
          cellRBs++;
        }
      } else {
        cellOffset = default_offset;
      }

      std::cout << "TDD Cell " << i << ": RBs=" << cellRBs
                << ", Offset=" << cellOffset << (reuse ? " (reuse)" : "")
                << std::endl;

      BandwidthManager *s = new BandwidthManager(
          bandwidth, bandwidth, cellOffset, cellOffset, cellRBs, cellRBs, true);
      spectrum.push_back(s);

      if (!reuse) {
        currentOffset += cellRBs;
      }
    }
  }

  return spectrum;
}

static std::vector<BandwidthManager *>
DivideResourcesFermi(int nodes, double bandwidth,
                     std::map<int, int> int_impacted_users) {
  std::vector<BandwidthManager *> spectrum;
  int totalRBs = 0;
  double cellBandwidth = 0;
  cout << "Bandwidth: " << bandwidth << " MHz" << std::endl;

  if (bandwidth == 1.4) {
    totalRBs = 6;
  } else if (bandwidth == 3) {
    totalRBs = 15;
  } else if (bandwidth == 5) {
    totalRBs = 25;
  } else if (bandwidth == 10) {
    totalRBs = 50;
  } else if (bandwidth == 15) {
    totalRBs = 75;
  } else if (bandwidth == 20) {
    totalRBs = 100;
  } else if (bandwidth == 50) {
    totalRBs = 248;
  } else if (bandwidth == 100) {
    totalRBs = 496;
  } else {
    // Default to 5MHz
    totalRBs = 25;
    bandwidth = 5;
  }

  int rbsPerCell = 0;
  int remainderRBs = 0;
  int currentOffset = 0;

  // get total number of interference impacted users
  int total_interference_limited_users = 0;
  for ( const auto &pair : int_impacted_users) {
    total_interference_limited_users += pair.second;
  }   

  cout << "Total Interference Impacted Users: " << total_interference_limited_users << endl;

  for (int i = 0; i < nodes; i++) {
    // distribute RBs in proportion to the number of interference impacted users
    rbsPerCell = totalRBs * int_impacted_users[i] / total_interference_limited_users;
    cout << "Cell " << i << ": RBs=" << rbsPerCell << endl;

    BandwidthManager *s = new BandwidthManager(
        bandwidth, bandwidth, currentOffset, currentOffset, rbsPerCell, rbsPerCell);
    spectrum.push_back(s);
    currentOffset += rbsPerCell;
  }

  return spectrum;
}

static std::vector<BandwidthManager *>
DivideResourcesFcbrs(int nodes, bool reuse, double bandwidth,
                     int int_impacted_users) {}

static std::vector<BandwidthManager *>
DivideResourcesIdeal(int nodes, bool reuse, double bandwidth,
                     int int_impacted_users) {}

#endif /* FREQUENCY_REUSE_HELPER_H_ */
