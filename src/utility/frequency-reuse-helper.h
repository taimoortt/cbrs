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
#include <iomanip>

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

// Unified function for proportional resource allocation
static std::vector<BandwidthManager *>
DivideResourcesProportional(int nodes, double bandwidth,
                            std::map<int, int> allocation_weights,
                            const std::string &allocation_type = "proportional",
                            bool ensure_minimum = false) {
  std::vector<BandwidthManager *> spectrum;
  int totalRBs = 0;
  cout << "Bandwidth: " << bandwidth << " MHz" << std::endl;

  // Calculate total RBs based on bandwidth
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

  // Calculate total weight across all cells
  int total_weight = 0;
  for (const auto &pair : allocation_weights) {
    total_weight += pair.second;
  }

  cout << "Total " << allocation_type << " Weight: " << total_weight << endl;
  cout << "Total RBs: " << totalRBs << endl;

  // Allocate resources proportionally
  int currentOffset = 0;
  for (int i = 0; i < nodes; i++) {
    int cellRBs = 0;

    if (total_weight > 0 &&
        allocation_weights.find(i) != allocation_weights.end()) {
      // Calculate RBs proportionally to the weight for this cell
      cellRBs = (totalRBs * allocation_weights[i]) / total_weight;
    }

    // // Ensure minimum of 1 RB per cell if requested and cell has weight
    if (ensure_minimum && cellRBs == 0 && allocation_weights[i] > 0) {
      cellRBs = 1;
    }

    cout << "Cell " << i << ": Weight=" << allocation_weights[i]
         << ", RBs=" << cellRBs << endl;

    // Create BandwidthManager with the calculated RBs
    BandwidthManager *s = new BandwidthManager(
        bandwidth, bandwidth, currentOffset, currentOffset, cellRBs, cellRBs);
    spectrum.push_back(s);

    // Update offset for next cell
    currentOffset += cellRBs;
  }

  return spectrum;
}

// Wrapper functions for backward compatibility
static std::vector<BandwidthManager *>
DivideResourcesFermi(int nodes, double bandwidth,
                     std::map<int, int> int_impacted_users) {
  return DivideResourcesProportional(nodes, bandwidth, int_impacted_users,
                                     "Fermi - Interference Impacted Users",
                                     false);
}

static std::vector<BandwidthManager *>
DivideResourcesFCBRS(int nodes, double bandwidth,
                     std::map<int, int> ues_per_cell) {
  return DivideResourcesProportional(nodes, bandwidth, ues_per_cell,
                                     "FCBRS - Total Users", false);
}

static std::vector<BandwidthManager *>
DivideResourcesIdeal(int nodes, double bandwidth,
                     std::map<int, int> int_impacted_users,
                     std::map<int, int> total_users_per_cell) {
  std::vector<BandwidthManager *> spectrum;
  int totalRBs = 0;
  cout << "Bandwidth: " << bandwidth << " MHz" << std::endl;

  // Calculate total RBs based on bandwidth
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

  cout << "Total RBs: " << totalRBs << endl;
  cout << "Ideal Resource Allocation - Interference-Aware Strategy" << endl;

  // Step 1: Calculate static partition share (equal division)
  int staticRBsPerCell = totalRBs / nodes;
  int remainderRBs = totalRBs % nodes;

  // Step 2: Calculate interference fractions and allocate resources
  std::vector<int> isolatedRBs(nodes, 0);  // RBs isolated for each cell
  std::vector<int> reuseRBs(nodes, 0);     // RBs available for reuse for each cell
  int totalIsolatedRBs = 0;

  for (int i = 0; i < nodes; i++) {
    int totalUsers = (total_users_per_cell.find(i) != total_users_per_cell.end()) 
                     ? total_users_per_cell[i] : 0;
    int interferenceUsers = (int_impacted_users.find(i) != int_impacted_users.end()) 
                            ? int_impacted_users[i] : 0;

    // Calculate interference fraction
    double interferenceFraction = 0.0;
    if (totalUsers > 0) {
      interferenceFraction = (double)interferenceUsers / totalUsers;
    }

    // Allocate isolated RBs based on interference fraction
    isolatedRBs[i] = (int)(staticRBsPerCell * interferenceFraction);
    
    // The remainder can be used for reuse
    reuseRBs[i] = staticRBsPerCell - isolatedRBs[i];
    
    totalIsolatedRBs += isolatedRBs[i];

    cout << "Cell " << i << ": Total Users=" << totalUsers 
         << ", Interference Users=" << interferenceUsers
         << ", Interference Fraction=" << std::fixed << std::setprecision(2) 
         << interferenceFraction * 100 << "%"
         << ", Isolated RBs=" << isolatedRBs[i]
         << ", Reuse RBs=" << reuseRBs[i] << endl;
  }

  // Handle remainder RBs - distribute them to cells with highest interference
  for (int i = 0; i < remainderRBs; i++) {
    // Find cell with highest interference fraction
    int maxInterferenceCell = 0;
    double maxFraction = 0.0;
    for (int j = 0; j < nodes; j++) {
      int totalUsers = (total_users_per_cell.find(j) != total_users_per_cell.end()) 
                       ? total_users_per_cell[j] : 0;
      int interferenceUsers = (int_impacted_users.find(j) != int_impacted_users.end()) 
                              ? int_impacted_users[j] : 0;
      double fraction = (totalUsers > 0) ? (double)interferenceUsers / totalUsers : 0.0;
      if (fraction > maxFraction) {
        maxFraction = fraction;
        maxInterferenceCell = j;
      }
    }
    isolatedRBs[maxInterferenceCell]++;
    totalIsolatedRBs++;
  }

  // Step 3: Calculate total reuse RBs available
  int totalReuseRBs = totalRBs - totalIsolatedRBs;
  cout << "Total Isolated RBs: " << totalIsolatedRBs << endl;
  cout << "Total Reuse RBs: " << totalReuseRBs << endl;

  // Step 4: Create BandwidthManager objects
  int currentOffset = 0;
  for (int i = 0; i < nodes; i++) {
    // Each cell gets its isolated RBs plus access to all reuse RBs
    int cellTotalRBs = isolatedRBs[i] + totalReuseRBs;
    
    cout << "Cell " << i << " Final Allocation: " 
         << isolatedRBs[i] << " isolated + " << totalReuseRBs 
         << " reuse = " << cellTotalRBs << " total RBs" << endl;

    // Create BandwidthManager with isolated RBs starting at currentOffset
    // and reuse RBs starting at the end of isolated allocations
    BandwidthManager *s = new BandwidthManager(
        bandwidth, bandwidth, currentOffset, currentOffset, 
        isolatedRBs[i], isolatedRBs[i]);
    spectrum.push_back(s);

    // Update offset for next cell's isolated resources
    currentOffset += isolatedRBs[i];
  }

  cout << "Ideal allocation complete - " << nodes << " cells configured" << endl;
  return spectrum;
}

#endif /* FREQUENCY_REUSE_HELPER_H_ */
