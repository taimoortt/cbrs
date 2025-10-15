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

#include "bandwidth-manager.h"
#include <cmath>
#include <iostream>

#define UL_LOW_FREQUENCY_BAND_1 1920  // MHz
#define UL_HIGH_FREQUENCY_BAND_1 1980 // MHz
#define DL_LOW_FREQUENCY_BAND_1 2110  // MHz
#define DL_HIGH_FREQUENCY_BAND_1 2170 // MHz

#define RBs_FOR_1_4_MHz 6
#define RBs_FOR_3_MHz 15
#define RBs_FOR_5_MHz 25
#define RBs_FOR_10_MHz 50
#define RBs_FOR_15_MHz 75
#define RBs_FOR_20_MHz 100
#define RBs_FOR_50_MHz 248
#define RBs_FOR_100_MHz 496

BandwidthManager::BandwidthManager() {}

BandwidthManager::BandwidthManager(double ulBw, double dlBw, int ulOffset,
                                   int dlOffset) {
  m_ulBandwidth = ulBw;
  m_dlBandwidth = dlBw;
  m_ulOffsetBw = ulOffset;
  m_dlOffsetBw = dlOffset;

  m_operativeSubBand = 1;

  m_dlSubChannels.clear();
  m_ulSubChannels.clear();

  if (dlBw == 1.4) {
    for (int i = dlOffset; i < dlOffset + RBs_FOR_1_4_MHz; i++) {
      m_dlSubChannels.push_back(DL_LOW_FREQUENCY_BAND_1 + (i * 0.18));
      m_dlGlobalRbIndices.push_back(i);
    }
  } else if (dlBw == 3) {
    for (int i = dlOffset; i < dlOffset + RBs_FOR_3_MHz; i++) {
      m_dlSubChannels.push_back(DL_LOW_FREQUENCY_BAND_1 + (i * 0.18));
      m_dlGlobalRbIndices.push_back(i);
    }
  } else if (dlBw == 5) {
    for (int i = dlOffset; i < dlOffset + RBs_FOR_5_MHz; i++) {
      m_dlSubChannels.push_back(DL_LOW_FREQUENCY_BAND_1 + (i * 0.18));
      m_dlGlobalRbIndices.push_back(i);
    }
  } else if (dlBw == 10) {
    for (int i = dlOffset; i < dlOffset + RBs_FOR_10_MHz; i++) {
      m_dlSubChannels.push_back(DL_LOW_FREQUENCY_BAND_1 + (i * 0.18));
      m_dlGlobalRbIndices.push_back(i);
    }
  } else if (dlBw == 15) {
    for (int i = dlOffset; i < dlOffset + RBs_FOR_15_MHz; i++) {
      m_dlSubChannels.push_back(DL_LOW_FREQUENCY_BAND_1 + (i * 0.18));
      m_dlGlobalRbIndices.push_back(i);
    }
  } else if (dlBw == 20) {
    for (int i = dlOffset; i < dlOffset + RBs_FOR_20_MHz; i++) {
      m_dlSubChannels.push_back(DL_LOW_FREQUENCY_BAND_1 + (i * 0.18));
      m_dlGlobalRbIndices.push_back(i);
    }
  } else if (dlBw == 50) {
    for (int i = dlOffset; i < dlOffset + RBs_FOR_50_MHz; i++) {
      m_dlSubChannels.push_back(DL_LOW_FREQUENCY_BAND_1 + (i * 0.18));
      m_dlGlobalRbIndices.push_back(i);
    }
  } else if (dlBw == 100) {
    for (int i = dlOffset; i < dlOffset + RBs_FOR_100_MHz; i++) {
      m_dlSubChannels.push_back(DL_LOW_FREQUENCY_BAND_1 + (i * 0.18));
      m_dlGlobalRbIndices.push_back(i);
    }
  } else {
    for (int i = dlOffset; i < dlOffset + RBs_FOR_5_MHz; i++) {
      m_dlSubChannels.push_back(DL_LOW_FREQUENCY_BAND_1 + (i * 0.18));
      m_dlGlobalRbIndices.push_back(i);
    }
  }

  if (ulBw == 1.4) {
    for (int i = ulOffset; i < ulOffset + RBs_FOR_1_4_MHz; i++) {
      m_ulSubChannels.push_back(UL_LOW_FREQUENCY_BAND_1 + (i * 0.18));
      m_ulGlobalRbIndices.push_back(i);
    }
  } else if (ulBw == 3) {
    for (int i = ulOffset; i < ulOffset + RBs_FOR_3_MHz; i++) {
      m_ulSubChannels.push_back(UL_LOW_FREQUENCY_BAND_1 + (i * 0.18));
      m_ulGlobalRbIndices.push_back(i);
    }
  } else if (ulBw == 5) {
    for (int i = ulOffset; i < ulOffset + RBs_FOR_5_MHz; i++) {
      m_ulSubChannels.push_back(UL_LOW_FREQUENCY_BAND_1 + (i * 0.18));
      m_ulGlobalRbIndices.push_back(i);
    }
  } else if (ulBw == 10) {
    for (int i = ulOffset; i < ulOffset + RBs_FOR_10_MHz; i++) {
      m_ulSubChannels.push_back(UL_LOW_FREQUENCY_BAND_1 + (i * 0.18));
      m_ulGlobalRbIndices.push_back(i);
    }
  } else if (ulBw == 15) {
    for (int i = ulOffset; i < ulOffset + RBs_FOR_15_MHz; i++) {
      m_ulSubChannels.push_back(UL_LOW_FREQUENCY_BAND_1 + (i * 0.18));
      m_ulGlobalRbIndices.push_back(i);
    }
  } else if (ulBw == 20) {
    for (int i = ulOffset; i < ulOffset + RBs_FOR_20_MHz; i++) {
      m_ulSubChannels.push_back(UL_LOW_FREQUENCY_BAND_1 + (i * 0.18));
      m_ulGlobalRbIndices.push_back(i);
    }
  } else {
    for (int i = ulOffset; i < ulOffset + RBs_FOR_5_MHz; i++) {
      m_ulSubChannels.push_back(UL_LOW_FREQUENCY_BAND_1 + (i * 0.18));
      m_ulGlobalRbIndices.push_back(i);
    }
  }
}

BandwidthManager::BandwidthManager(double ulBw, double dlBw, int ulOffset,
                                   int dlOffset, bool tddTrue) {
  m_ulBandwidth = ulBw + dlBw;
  m_dlBandwidth = dlBw + ulBw;
  m_ulOffsetBw = ulOffset;
  m_dlOffsetBw = dlOffset;

  m_operativeSubBand = 1;

  m_dlSubChannels.clear();
  m_ulSubChannels.clear();

  if (dlBw == 1.4) {
    for (int i = dlOffset; i < dlOffset + RBs_FOR_1_4_MHz; i++) {
      m_dlSubChannels.push_back(DL_LOW_FREQUENCY_BAND_1 + (i * 0.18));
      m_ulSubChannels.push_back(DL_LOW_FREQUENCY_BAND_1 + (i * 0.18));
      m_dlGlobalRbIndices.push_back(i);
      m_ulGlobalRbIndices.push_back(i);
    }
  } else if (dlBw == 3) {
    for (int i = dlOffset; i < dlOffset + RBs_FOR_3_MHz; i++) {
      m_dlSubChannels.push_back(DL_LOW_FREQUENCY_BAND_1 + (i * 0.18));
      m_ulSubChannels.push_back(DL_LOW_FREQUENCY_BAND_1 + (i * 0.18));
      m_dlGlobalRbIndices.push_back(i);
      m_ulGlobalRbIndices.push_back(i);
    }
  } else if (dlBw == 5) {
    for (int i = dlOffset; i < dlOffset + RBs_FOR_5_MHz; i++) {
      m_dlSubChannels.push_back(DL_LOW_FREQUENCY_BAND_1 + (i * 0.18));
      m_ulSubChannels.push_back(DL_LOW_FREQUENCY_BAND_1 + (i * 0.18));
      m_dlGlobalRbIndices.push_back(i);
      m_ulGlobalRbIndices.push_back(i);
    }
  } else if (dlBw == 10) {
    for (int i = dlOffset; i < dlOffset + RBs_FOR_10_MHz; i++) {
      m_dlSubChannels.push_back(DL_LOW_FREQUENCY_BAND_1 + (i * 0.18));
      m_ulSubChannels.push_back(DL_LOW_FREQUENCY_BAND_1 + (i * 0.18));
      m_dlGlobalRbIndices.push_back(i);
      m_ulGlobalRbIndices.push_back(i);
    }
  } else if (dlBw == 15) {
    for (int i = dlOffset; i < dlOffset + RBs_FOR_15_MHz; i++) {
      m_dlSubChannels.push_back(DL_LOW_FREQUENCY_BAND_1 + (i * 0.18));
      m_ulSubChannels.push_back(DL_LOW_FREQUENCY_BAND_1 + (i * 0.18));
      m_dlGlobalRbIndices.push_back(i);
      m_ulGlobalRbIndices.push_back(i);
    }
  } else if (dlBw == 20) {
    for (int i = dlOffset; i < dlOffset + RBs_FOR_20_MHz; i++) {
      m_dlSubChannels.push_back(DL_LOW_FREQUENCY_BAND_1 + (i * 0.18));
      m_ulSubChannels.push_back(DL_LOW_FREQUENCY_BAND_1 + (i * 0.18));
      m_dlGlobalRbIndices.push_back(i);
      m_ulGlobalRbIndices.push_back(i);
    }
  } else {
    for (int i = dlOffset; i < dlOffset + RBs_FOR_5_MHz; i++) {
      m_dlSubChannels.push_back(DL_LOW_FREQUENCY_BAND_1 + (i * 0.18));
      m_ulSubChannels.push_back(DL_LOW_FREQUENCY_BAND_1 + (i * 0.18));
      m_dlGlobalRbIndices.push_back(i);
      m_ulGlobalRbIndices.push_back(i);
    }
  }

  if (ulBw == 1.4) {
    for (int i = ulOffset; i < ulOffset + RBs_FOR_1_4_MHz; i++) {
      m_ulSubChannels.push_back(UL_LOW_FREQUENCY_BAND_1 + (i * 0.18));
      m_dlSubChannels.push_back(UL_LOW_FREQUENCY_BAND_1 + (i * 0.18));
      m_ulGlobalRbIndices.push_back(i);
      m_dlGlobalRbIndices.push_back(i);
    }
  } else if (ulBw == 3) {
    for (int i = ulOffset; i < ulOffset + RBs_FOR_3_MHz; i++) {
      m_ulSubChannels.push_back(UL_LOW_FREQUENCY_BAND_1 + (i * 0.18));
      m_dlSubChannels.push_back(UL_LOW_FREQUENCY_BAND_1 + (i * 0.18));
      m_ulGlobalRbIndices.push_back(i);
      m_dlGlobalRbIndices.push_back(i);
    }
  } else if (ulBw == 5) {
    for (int i = ulOffset; i < ulOffset + RBs_FOR_5_MHz; i++) {
      m_ulSubChannels.push_back(UL_LOW_FREQUENCY_BAND_1 + (i * 0.18));
      m_dlSubChannels.push_back(UL_LOW_FREQUENCY_BAND_1 + (i * 0.18));
      m_ulGlobalRbIndices.push_back(i);
      m_dlGlobalRbIndices.push_back(i);
    }
  } else if (ulBw == 10) {
    for (int i = ulOffset; i < ulOffset + RBs_FOR_10_MHz; i++) {
      m_ulSubChannels.push_back(UL_LOW_FREQUENCY_BAND_1 + (i * 0.18));
      m_dlSubChannels.push_back(UL_LOW_FREQUENCY_BAND_1 + (i * 0.18));
      m_ulGlobalRbIndices.push_back(i);
      m_dlGlobalRbIndices.push_back(i);
    }
  } else if (ulBw == 15) {
    for (int i = ulOffset; i < ulOffset + RBs_FOR_15_MHz; i++) {
      m_ulSubChannels.push_back(UL_LOW_FREQUENCY_BAND_1 + (i * 0.18));
      m_dlSubChannels.push_back(UL_LOW_FREQUENCY_BAND_1 + (i * 0.18));
      m_ulGlobalRbIndices.push_back(i);
      m_dlGlobalRbIndices.push_back(i);
    }
  } else if (ulBw == 20) {
    for (int i = ulOffset; i < ulOffset + RBs_FOR_20_MHz; i++) {
      m_ulSubChannels.push_back(UL_LOW_FREQUENCY_BAND_1 + (i * 0.18));
      m_dlSubChannels.push_back(UL_LOW_FREQUENCY_BAND_1 + (i * 0.18));
      m_ulGlobalRbIndices.push_back(i);
      m_dlGlobalRbIndices.push_back(i);
    }
  } else {
    for (int i = ulOffset; i < ulOffset + RBs_FOR_5_MHz; i++) {
      m_ulSubChannels.push_back(UL_LOW_FREQUENCY_BAND_1 + (i * 0.18));
      m_dlSubChannels.push_back(UL_LOW_FREQUENCY_BAND_1 + (i * 0.18));
      m_ulGlobalRbIndices.push_back(i);
      m_dlGlobalRbIndices.push_back(i);
    }
  }
}

// New constructors allowing explicit RB counts irrespective of nominal
// bandwidth
BandwidthManager::BandwidthManager(double ulBw, double dlBw, int ulOffset,
                                   int dlOffset, int ulRbs, int dlRbs) {
  m_ulBandwidth = ulBw;
  m_dlBandwidth = dlBw;
  m_ulOffsetBw = ulOffset;
  m_dlOffsetBw = dlOffset;

  m_operativeSubBand = 1;

  m_dlSubChannels.clear();
  m_ulSubChannels.clear();

  // Fill exactly dlRbs starting from dlOffset
  for (int i = dlOffset; i < dlOffset + dlRbs; i++) {
    m_dlSubChannels.push_back(DL_LOW_FREQUENCY_BAND_1 + (i * 0.18));
    m_dlGlobalRbIndices.push_back(i);
  }

  // Fill exactly ulRbs starting from ulOffset
  for (int i = ulOffset; i < ulOffset + ulRbs; i++) {
    m_ulSubChannels.push_back(UL_LOW_FREQUENCY_BAND_1 + (i * 0.18));
    m_ulGlobalRbIndices.push_back(i);
  }
}

std::vector<double>
BandwidthManager::GetSharedDlChannels(int index, int num_shared_channels) {
  std::vector<double> shared;
  for (int i = index; i < num_shared_channels; i++) {
    shared.push_back(m_dlSubChannels.at(i));
  }
  return shared;
}

void BandwidthManager::AddSharedChannels(std::vector<double> shared_dlChannels,
                                         std::vector<double> shared_ulChannels,
                                         BandwidthManager &s) {
  // Add DL shared channels if not already present
  for (int i = 0; i < shared_dlChannels.size(); i++) {
    double f = shared_dlChannels.at(i);
    bool found = false;
    for (int j = 0; j < m_dlSubChannels.size(); j++) {
      if (m_dlSubChannels.at(j) == f) {
        found = true;
        break;
      }
    }
    if (!found) {
      m_dlSubChannels.push_back(f);
    }
  }
}

BandwidthManager::BandwidthManager(double ulBw, double dlBw, int ulOffset,
                                   int dlOffset, int ulRbs, int dlRbs,
                                   bool tddTrue) {
  // In TDD both directions share spectrum; still expose separate vectors
  // sized as requested
  m_ulBandwidth = ulBw + dlBw;
  m_dlBandwidth = dlBw + ulBw;
  m_ulOffsetBw = ulOffset;
  m_dlOffsetBw = dlOffset;

  m_operativeSubBand = 1;

  m_dlSubChannels.clear();
  m_ulSubChannels.clear();

  for (int i = dlOffset; i < dlOffset + dlRbs; i++) {
    double f = DL_LOW_FREQUENCY_BAND_1 + (i * 0.18);
    m_dlSubChannels.push_back(f);
    m_ulSubChannels.push_back(f);
    m_dlGlobalRbIndices.push_back(i);
    m_ulGlobalRbIndices.push_back(i);
  }

  for (int i = ulOffset; i < ulOffset + ulRbs; i++) {
    double f = UL_LOW_FREQUENCY_BAND_1 + (i * 0.18);
    m_ulSubChannels.push_back(f);
    m_dlSubChannels.push_back(f);
    m_ulGlobalRbIndices.push_back(i);
    m_dlGlobalRbIndices.push_back(i);
  }
}

BandwidthManager::~BandwidthManager() {}

void BandwidthManager::SetDlSubChannels(std::vector<double> s) {
  m_dlSubChannels = s;
  // Invalidate indices; recompute lazily
  m_dlGlobalRbIndices.clear();
}

std::vector<double> BandwidthManager::GetDlSubChannels(void) {
  return m_dlSubChannels;
}

void BandwidthManager::SetUlSubChannels(std::vector<double> s) {
  m_ulSubChannels = s;
  m_ulGlobalRbIndices.clear();
}

std::vector<double> BandwidthManager::GetUlSubChannels(void) {
  return m_ulSubChannels;
}

void BandwidthManager::SetOperativeSubBand(int s) { m_operativeSubBand = s; }

int BandwidthManager::GetOperativeSubBand(void) { return m_operativeSubBand; }

void BandwidthManager::SetUlBandwidth(double b) { m_ulBandwidth = b; }

void BandwidthManager::SetDlBandwidth(double b) { m_dlBandwidth = b; }

void BandwidthManager::SetUlOffsetBw(int o) { m_ulOffsetBw = o; }

void BandwidthManager::SetDlOffsetBw(int o) { m_dlOffsetBw = o; }

double BandwidthManager::GetUlBandwidth(void) { return m_ulBandwidth; }

double BandwidthManager::GetDlBandwidth(void) { return m_dlBandwidth; }

int BandwidthManager::GetUlOffsetBw(void) { return m_ulOffsetBw; }

int BandwidthManager::GetDlOffsetBw(void) { return m_dlOffsetBw; }

BandwidthManager *BandwidthManager::Copy() {
  BandwidthManager *s = new BandwidthManager();
  s->SetDlBandwidth(GetDlBandwidth());
  s->SetUlBandwidth(GetUlBandwidth());
  s->SetDlOffsetBw(GetDlOffsetBw());
  s->SetUlOffsetBw(GetUlOffsetBw());
  s->SetDlSubChannels(GetDlSubChannels());
  s->SetUlSubChannels(GetUlSubChannels());
  s->SetOperativeSubBand(GetOperativeSubBand());
  s->m_dlGlobalRbIndices = m_dlGlobalRbIndices;
  s->m_ulGlobalRbIndices = m_ulGlobalRbIndices;

  return s;
}

// ---- Global RB identity helpers implementation ----
std::vector<int> BandwidthManager::GetDlGlobalRbIndices() const {
  if (!m_dlGlobalRbIndices.empty())
    return m_dlGlobalRbIndices;
  // derive contiguous indices starting from first freq base
  std::vector<int> derived;
  derived.reserve(m_dlSubChannels.size());
  for (size_t i = 0; i < m_dlSubChannels.size(); ++i) {
    // Map frequency to index using base and spacing
    double delta = (m_dlSubChannels[i] - DL_LOW_FREQUENCY_BAND_1) / 0.18;
    int idx = (int)std::floor(delta + 0.5);
    derived.push_back(idx);
  }
  return derived;
}

void BandwidthManager::SetDlGlobalRbIndices(const std::vector<int> &idx) {
  m_dlGlobalRbIndices = idx;
}

std::vector<int> BandwidthManager::GetUlGlobalRbIndices() const {
  if (!m_ulGlobalRbIndices.empty())
    return m_ulGlobalRbIndices;
  std::vector<int> derived;
  derived.reserve(m_ulSubChannels.size());
  for (size_t i = 0; i < m_ulSubChannels.size(); ++i) {
    double delta = (m_ulSubChannels[i] - UL_LOW_FREQUENCY_BAND_1) / 0.18;
    int idx = (int)std::floor(delta + 0.5);
    derived.push_back(idx);
  }
  return derived;
}

void BandwidthManager::SetUlGlobalRbIndices(const std::vector<int> &idx) {
  m_ulGlobalRbIndices = idx;
}

void BandwidthManager::RecomputeGlobalIndicesFromFrequencies() {
  m_dlGlobalRbIndices.clear();
  m_ulGlobalRbIndices.clear();
  for (double f : m_dlSubChannels) {
    double delta = (f - DL_LOW_FREQUENCY_BAND_1) / 0.18;
    int idx = (int)std::floor(delta + 0.5);
    m_dlGlobalRbIndices.push_back(idx);
  }
  for (double f : m_ulSubChannels) {
    double delta = (f - UL_LOW_FREQUENCY_BAND_1) / 0.18;
    int idx = (int)std::floor(delta + 0.5);
    m_ulGlobalRbIndices.push_back(idx);
  }
}

void BandwidthManager::Print(void) {
  std::cout << "BandwidthManager: " << this << std::endl;

  std::cout << "\t operative sub band: " << m_operativeSubBand
            << "\n\t m_dlBandwidth " << m_dlBandwidth << "\n\t m_ulBandwidth "
            << m_ulBandwidth << "\n\t m_dlOffsetBw " << m_dlOffsetBw
            << "\n\t m_ulOffsetBw " << m_ulOffsetBw << std::endl;

  std::cout << "\t DL channels: ";
  for (int i = 0; i < m_dlSubChannels.size(); i++) {
    std::cout << m_dlSubChannels.at(i) << " ";
  }
  std::cout << std::endl;

  std::cout << "\t UL channels: ";
  for (int i = 0; i < m_ulSubChannels.size(); i++) {
    std::cout << m_ulSubChannels.at(i) << " ";
  }
  std::cout << std::endl;
}
