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
#include <algorithm>
#include <cassert>
#include <iomanip>
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

    // Randomly select which cells get the remainder RBs
    std::vector<int> extraRBs(nodes, 0);
    if (!reuse && remainderRBs > 0) {
      std::vector<int> indices(nodes);
      for (int i = 0; i < nodes; i++)
        indices[i] = i;
      // Shuffle indices and pick first remainderRBs cells
      for (int i = nodes - 1; i > 0; i--) {
        int j = rand() % (i + 1);
        std::swap(indices[i], indices[j]);
      }
      for (int i = 0; i < remainderRBs; i++) {
        extraRBs[indices[i]] = 1;
      }
    }

    int currentOffset = default_offset;
    int totalAllocated = 0;
    for (int i = 0; i < nodes; i++) {
      int cellRBs = rbsPerCell;

      int cellOffset = currentOffset;
      if (!reuse) {
        // Add randomly distributed remainder RBs
        cellRBs += extraRBs[i];
        totalAllocated += cellRBs;
      } else {
        // Reuse: all cells share the same RBs and offset
        cellOffset = default_offset;
      }

      std::cout << "Cell " << i << ": RBs=" << cellRBs
                << ", Offset=" << cellOffset << (reuse ? " (reuse)" : "")
                << std::endl;
      if (reuse) {
        cout << "Cell " << i << ": Total Users=" << 0 << ", Isolated RBs=" << 0
             << ", Reuse RBs=" << cellRBs << endl;
      } else {
        cout << "Cell " << i << ": Total Users=" << 0
             << ", Isolated RBs=" << cellRBs << ", Reuse RBs=" << 0 << endl;
      }

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

    // Assert that all RBs have been allocated (only for static partitioning)
    if (!reuse) {
      assert(totalAllocated == totalRBs &&
             "All RBs must be allocated to cells");
      std::cout << "FDD Assertion passed: All " << totalRBs << " RBs allocated"
                << std::endl;
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

    // Randomly select which cells get the remainder RBs
    std::vector<int> extraRBs(nodes, 0);
    if (!reuse && remainderRBs > 0) {
      std::vector<int> indices(nodes);
      for (int i = 0; i < nodes; i++)
        indices[i] = i;
      // Shuffle indices and pick first remainderRBs cells
      for (int i = nodes - 1; i > 0; i--) {
        int j = rand() % (i + 1);
        std::swap(indices[i], indices[j]);
      }
      for (int i = 0; i < remainderRBs; i++) {
        extraRBs[indices[i]] = 1;
      }
    }

    int currentOffset = default_offset;
    int totalAllocated = 0;
    for (int i = 0; i < nodes; i++) {
      int cellRBs = rbsPerCell;

      int cellOffset = currentOffset;
      if (!reuse) {
        // Add randomly distributed remainder RBs
        cellRBs += extraRBs[i];
        totalAllocated += cellRBs;
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

    // Assert that all RBs have been allocated (only for static partitioning)
    if (!reuse) {
      assert(totalAllocated == totalRBs &&
             "All RBs must be allocated to cells");
      std::cout << "TDD Assertion passed: All " << totalRBs << " RBs allocated"
                << std::endl;
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

  // Allocate resources proportionally and calculate remainder
  std::vector<int> cellRBs(nodes, 0);
  int allocatedRBs = 0;

  for (int i = 0; i < nodes; i++) {
    if (total_weight > 0 &&
        allocation_weights.find(i) != allocation_weights.end()) {
      // Calculate RBs proportionally to the weight for this cell
      cellRBs[i] = (totalRBs * allocation_weights[i]) / total_weight;
      allocatedRBs += cellRBs[i];
    }
  }

  // Calculate remainder RBs and distribute them randomly
  int remainderRBs = totalRBs - allocatedRBs;
  if (remainderRBs > 0) {
    std::vector<int> indices;
    for (int i = 0; i < nodes; i++) {
      if (allocation_weights.find(i) != allocation_weights.end() &&
          allocation_weights[i] > 0) {
        indices.push_back(i);
      }
    }

    // Shuffle indices
    for (int i = indices.size() - 1; i > 0; i--) {
      int j = rand() % (i + 1);
      std::swap(indices[i], indices[j]);
    }

    // Distribute remainder RBs
    for (int i = 0; i < remainderRBs && i < (int)indices.size(); i++) {
      cellRBs[indices[i]]++;
    }
  }

  cout << "Allocated RBs: " << allocatedRBs << ", Remainder: " << remainderRBs
       << endl;

  // Create BandwidthManager objects
  int currentOffset = 0;
  int totalFinalAllocated = 0;
  for (int i = 0; i < nodes; i++) {
    cout << "Cell " << i << ": Total Users=" << allocation_weights[i]
         << ", Isolated RBs=" << cellRBs[i] << ", Reuse RBs=" << 0 << endl;

    // Create BandwidthManager with the calculated RBs
    BandwidthManager *s =
        new BandwidthManager(bandwidth, bandwidth, currentOffset, currentOffset,
                             cellRBs[i], cellRBs[i]);
    spectrum.push_back(s);

    // Update offset for next cell
    currentOffset += cellRBs[i];
    totalFinalAllocated += cellRBs[i];
  }

  // Assert that all RBs have been allocated
  assert(totalFinalAllocated == totalRBs &&
         "All RBs must be allocated to cells");
  std::cout << "Proportional Assertion passed: All " << totalRBs
            << " RBs allocated" << std::endl;

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

  // --- replace from "Step 1: Calculate static partition share ..." down to
  // spectrum.push_back(s); ---

  // Step 1: base contiguous blocks per cell (handle remainder cleanly)
  std::vector<int> blockStart(nodes, 0), blockEnd(nodes, 0),
      blockSize(nodes, 0);
  int base = totalRBs / nodes; // 12
  int rem = totalRBs % nodes;  // 1

  // Randomly select which cells get the remainder RBs
  std::vector<int> extraRBs(nodes, 0);
  if (rem > 0) {
    std::vector<int> indices(nodes);
    for (int i = 0; i < nodes; i++)
      indices[i] = i;
    // Shuffle indices and pick first rem cells
    for (int i = nodes - 1; i > 0; i--) {
      int j = rand() % (i + 1);
      std::swap(indices[i], indices[j]);
    }
    for (int i = 0; i < rem; i++) {
      extraRBs[indices[i]] = 1;
    }
  }

  int cursor = 0;
  int totalBlockSize = 0;
  for (int i = 0; i < nodes; ++i) {
    blockSize[i] = base + extraRBs[i];
    blockStart[i] = cursor;
    blockEnd[i] = cursor + blockSize[i]; // exclusive
    cursor = blockEnd[i];
    totalBlockSize += blockSize[i];
  }

  // Assert that all RBs have been allocated to block sizes
  assert(totalBlockSize == totalRBs && "All RBs must be allocated to cells");
  std::cout << "Ideal Assertion passed: All " << totalRBs << " RBs allocated"
            << std::endl;

  // Step 2: per-cell isolated/reuse counts (bounded by each cell's block size)
  std::vector<int> isolatedRBs(nodes, 0), reuseRBs(nodes, 0);
  int totalIsolatedRBs = 0;

  for (int i = 0; i < nodes; ++i) {
    int totalUsers =
        (total_users_per_cell.count(i) ? total_users_per_cell[i] : 0);
    int interferenceUsers =
        (int_impacted_users.count(i) ? int_impacted_users[i] : 0);

    double frac =
        (totalUsers > 0) ? double(interferenceUsers) / totalUsers : 0.0;

    // cap isolated to the block size to be safe
    int iso = (int)std::round(frac * blockSize[i]);
    iso = std::max(0, std::min(iso, blockSize[i]));
    int rsz = blockSize[i] - iso;

    isolatedRBs[i] = iso;
    reuseRBs[i] = rsz;
    totalIsolatedRBs += iso;
  }

  int totalReuseRBs = totalRBs - totalIsolatedRBs;
  cout << "Total Isolated RBs: " << totalIsolatedRBs << endl;
  cout << "Total Reuse RBs: " << totalReuseRBs << endl;
  // After computing totals, print per-cell stats with Reuse RBs set to the
  // final totalIsolatedRBs (instead of per-cell rsz)
  for (int i = 0; i < nodes; ++i) {
    int totalUsers =
        (total_users_per_cell.count(i) ? total_users_per_cell[i] : 0);
    int interferenceUsers =
        (int_impacted_users.count(i) ? int_impacted_users[i] : 0);
    double frac =
        (totalUsers > 0) ? double(interferenceUsers) / totalUsers : 0.0;

    int iso = isolatedRBs[i];

    cout << "Cell " << i << ": Total Users=" << totalUsers
         << ", Isolated RBs=" << iso << ", Reuse RBs=" << totalReuseRBs
         << " , Allocated RBs=" << blockSize[i]
         << ", Interference Users=" << interferenceUsers
         << ", Interference Fraction=" << std::fixed << std::setprecision(2)
         << frac * 100 << "%"
         << ", Block=[" << blockStart[i] << "-" << (blockEnd[i] - 1) << "]"
         << endl;
  }

  // Step 3: place isolated & reuse ranges to maximize shared contiguity
  // Policy:
  // - If nodes == 2: cell0 isolated at low end; cell1 isolated at high end.
  //   => shared spans the boundary contiguously.
  // - Else (n>2): we still prioritize boundary sharing between neighbors by
  //   alternating sides (even: isolated low, odd: isolated high). This makes
  //   ~every other boundary shared and keeps big contiguous shared chunks.
  std::vector<int> isoStart(nodes, 0), isoEnd(nodes, 0);
  std::vector<std::pair<int, int>> reuseRanges(nodes); // [start, end) per cell

  for (int i = 0; i < nodes; ++i) {
    bool isoLow = false;
    if (nodes == 2) {
      isoLow = (i == 0); // cell0: isolated low, cell1: isolated high
    } else {
      isoLow = (i % 2 == 0); // alternate for n>2
    }

    if (isolatedRBs[i] == 0) {
      // no isolated, all reuse
      isoStart[i] = isoEnd[i] = blockStart[i];
      reuseRanges[i] = {blockStart[i], blockEnd[i]};
    } else if (reuseRBs[i] == 0) {
      // fully isolated (degenerate)
      if (isoLow) {
        isoStart[i] = blockStart[i];
        isoEnd[i] = blockEnd[i];
      } else {
        isoStart[i] = blockStart[i];
        isoEnd[i] = blockEnd[i];
      }
      reuseRanges[i] = {blockStart[i], blockStart[i]}; // empty
    } else if (isoLow) {
      // isolated at low end, reuse at high end
      isoStart[i] = blockStart[i];
      isoEnd[i] = blockStart[i] + isolatedRBs[i];
      reuseRanges[i] = {isoEnd[i], blockEnd[i]};
    } else {
      // isolated at high end, reuse at low end
      isoStart[i] = blockEnd[i] - isolatedRBs[i];
      isoEnd[i] = blockEnd[i];
      reuseRanges[i] = {blockStart[i], isoStart[i]};
    }

    if (isolatedRBs[i] > 0) {
      cout << "Cell " << i << " Layout: Isolated [" << isoStart[i] << "-"
           << (isoEnd[i] - 1) << "], Reuse [" << reuseRanges[i].first << "-"
           << (reuseRanges[i].second - 1) << "]\n";
    } else {
      cout << "Cell " << i << " Layout: No isolated, Reuse ["
           << reuseRanges[i].first << "-" << (reuseRanges[i].second - 1)
           << "]\n";
    }
  }

  // Step 4: build the GLOBAL shared set as the union of all per-cell reuse
  // ranges (for two cells this becomes one contiguous block across the
  // boundary)
  std::vector<std::pair<int, int>> sharedSegments =
      reuseRanges; // merge overlaps
  std::sort(sharedSegments.begin(), sharedSegments.end());
  std::vector<std::pair<int, int>> merged;
  for (auto seg : sharedSegments) {
    if (seg.first >= seg.second)
      continue;
    if (merged.empty() || seg.first > merged.back().second) {
      merged.push_back(seg);
    } else {
      merged.back().second = std::max(merged.back().second, seg.second);
    }
  }

  // Optional: print the merged shared regions
  int sharedTotal = 0;
  for (auto &m : merged)
    sharedTotal += (m.second - m.first);
  cout << "Merged Shared Regions: ";
  for (size_t k = 0; k < merged.size(); ++k) {
    cout << "[" << merged[k].first << "-" << (merged[k].second - 1) << "]";
    if (k + 1 < merged.size())
      cout << ", ";
  }
  cout << " (Total " << sharedTotal << " RBs)\n";

  // Step 5: materialize BandwidthManager objects
  for (int i = 0; i < nodes; ++i) {
    int isoCount = std::max(0, isoEnd[i] - isoStart[i]);

    // Create BM preloaded with isolated window
    BandwidthManager *s = new BandwidthManager(
        bandwidth, bandwidth, /*dlOffsetBw*/ isoStart[i],
        /*ulOffsetBw*/ isoStart[i], /*dlRBs*/ isoCount, /*ulRBs*/ isoCount);

    // Build isolated channels
    std::vector<double> dl = s->GetDlSubChannels();
    std::vector<double> ul = s->GetUlSubChannels();

    // Append GLOBAL shared channels (union of every cell’s reuse segments)
    for (auto seg : merged) {
      for (int rb = seg.first; rb < seg.second; ++rb) {
        dl.push_back(2110.0 + rb * 0.18);
        ul.push_back(1920.0 + rb * 0.18);
      }
    }
    // sort dl and ul
    std::sort(dl.begin(), dl.end());
    std::sort(ul.begin(), ul.end());

    s->SetDlSubChannels(dl);
    s->SetUlSubChannels(ul);
    spectrum.push_back(s);

    cout << "Cell " << i << " Final Allocation: " << isoCount << " isolated + "
         << sharedTotal << " shared = " << (isoCount + sharedTotal)
         << " total RBs\n";
  }
  // --- end replace ---
  return spectrum;
}

#endif /* FREQUENCY_REUSE_HELPER_H_ */
