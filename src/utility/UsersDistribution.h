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

#ifndef USERSDISTRIBTION_H_
#define USERSDISTRIBTION_H_

#include "../componentManagers/NetworkManager.h"
#include "../core/cartesianCoodrdinates/CartesianCoordinates.h"
#include "CellPosition.h"

#include <iostream>
#include <random>
#include <time.h>
#include <vector>

static CartesianCoordinates *GetCartesianCoordinatesFromPolar(double r,
                                                              double angle) {
  double x = r * cos(angle);
  double y = r * sin(angle);

  CartesianCoordinates *coordinates = new CartesianCoordinates();
  coordinates->SetCoordinates(x, y);
  return coordinates;
}

static vector<CartesianCoordinates *> *GetUniformUsersDistribution(int idCell,
                                                                   int nbUE) {
  NetworkManager *networkManager = NetworkManager::Init();
  vector<CartesianCoordinates *> *vectorOfCoordinates =
      new vector<CartesianCoordinates *>;

  int n_cells = networkManager->GetCellContainer()->size();
  Cell *cell = networkManager->GetCellByID(idCell);
  double cell_x = cell->GetCellCenterPosition()->GetCoordinateX();
  double cell_y = cell->GetCellCenterPosition()->GetCoordinateY();
  Cell *cell_neighbor = networkManager->GetCellByID((idCell + 1) % n_cells);
  double cell_x_neighbor =
      cell_neighbor->GetCellCenterPosition()->GetCoordinateX();
  double cell_y_neighbor =
      cell_neighbor->GetCellCenterPosition()->GetCoordinateY();
  double midpoint_x = ((cell_x + cell_x_neighbor) / 2);
  double midpoint_y = ((cell_y + cell_y_neighbor) / 2);

  CartesianCoordinates *cellCoordinates = cell->GetCellCenterPosition();
  double r;
  double angle;
  int mod;

  // Generate Users Uniformly around a Cell
  if (cell->GetRadius() >= 1) {
    r = 1000;
  } else {
    r = 1000 / 5;
  }

  // *******  Generate Random Coordinates  *********
  for (int i = 0; i < nbUE; i++) {
    CartesianCoordinates *newCoordinates;
    double random_num_x = static_cast<double>(rand() % (int)r);
    double random_num_y = static_cast<double>(rand() % (int)r);
    cout << "Generating Position: " << random_num_x << "\t" << random_num_y
         << endl;
    int mod = rand() % 4;

    if (mod == 0) {
      newCoordinates = new CartesianCoordinates(cell_x + random_num_x,
                                                cell_y + random_num_y);
    } else if (mod == 1) {
      newCoordinates = new CartesianCoordinates(cell_x - random_num_x,
                                                cell_y + random_num_y);
    } else if (mod == 2) {
      newCoordinates = new CartesianCoordinates(cell_x + random_num_x,
                                                cell_y - random_num_y);
    } else if (mod == 3) {
      newCoordinates = new CartesianCoordinates(cell_x - random_num_x,
                                                cell_y - random_num_y);
    }
    vectorOfCoordinates->push_back(newCoordinates);
  }

  // for (int i = 0; i < nbUE; i++)
  // {
  //   r = (double) (rand() % (int)(cell->GetRadius()*1000) * 2);
  //   angle = (double)(rand() %360) * ((2*3.14)/360);
  //   CartesianCoordinates *newCoordinates = GetCartesianCoordinatesFromPolar
  //   (r, angle);
  //   //Compute absoluteCoordinates
  //   vectorOfCoordinates->push_back(newCoordinates);
  // }

  // for (int i = 0; i < nbUE; i++)
  // {
  //   double random_num = (double)(rand() % 60);
  //   CartesianCoordinates *newCoordinates = new CartesianCoordinates(
  //     midpoint_x + random_num, midpoint_y + random_num);
  //   vectorOfCoordinates->push_back(newCoordinates);
  // }

  return vectorOfCoordinates;
}

static vector<CartesianCoordinates *> *GetHotspotUsersDistribution(int nbUE) {
  NetworkManager *networkManager = NetworkManager::Init();
  vector<CartesianCoordinates *> *vectorOfCoordinates =
      new vector<CartesianCoordinates *>;
  int total_cells = networkManager->GetCellContainer()->size();
  int num_macro_cell = networkManager->GetNbMacroCell();
  int num_micro_cell = total_cells - num_macro_cell;
  networkManager->GetCellContainer();
  Cell *c;

  // Choose with  2/3 probability to select a cell from Micro cells
  // and with 1/3 probability to select a cell from Macro cells
  int cell_type;
  int cell_index = 0;
  CartesianCoordinates *cellCoordinates;
  double cell_x;
  double cell_y;
  double r;
  double angle;
  double random_num_x;
  double random_num_y;
  int mod;

  // *******  Generate Random Coordinates  *********
  for (int i = 0; i < nbUE; i++) {
    cell_type = rand() % 3;
    if (cell_type < 2 &&
        num_micro_cell > 0) { // 2/3 probability to select a Micro Cell
      // Select a Micro Cell
      // Generate a random number between num_macro cell and num_micro cell
      cell_index = num_macro_cell + (rand() % (total_cells - num_macro_cell));
    } else {
      // Select a Macro Cell
      cell_index = rand() % num_macro_cell;
    }
    c = networkManager->GetCellContainer()->at(cell_index);
    cellCoordinates = c->GetCellCenterPosition();
    if (c->GetRadius() >= 1) {
      r = 900;
    } else {
      r = 180;
    }
    random_num_x = static_cast<double>(rand() % (int)r + 50);
    random_num_y = static_cast<double>(rand() % (int)r + 50);
    // if (c->GetRadius() >= 1) {
    //   r = 750;
    //   random_num_x = static_cast<double>(rand() % (int)r + 200);
    //   random_num_y = static_cast<double>(rand() % (int)r + 200);
    // } else {
    //   r = 170;
    //   random_num_x = static_cast<double>(rand() % (int)r + 30);
    //   random_num_y = static_cast<double>(rand() % (int)r + 30);
    // }
    cell_x = cellCoordinates->GetCoordinateX();
    cell_y = cellCoordinates->GetCoordinateY();

    CartesianCoordinates *newCoordinates;
    mod = rand() % 4;

    if (mod == 0) {
      newCoordinates = new CartesianCoordinates(cell_x + random_num_x,
                                                cell_y + random_num_y);
    } else if (mod == 1) {
      newCoordinates = new CartesianCoordinates(cell_x - random_num_x,
                                                cell_y + random_num_y);
    } else if (mod == 2) {
      newCoordinates = new CartesianCoordinates(cell_x + random_num_x,
                                                cell_y - random_num_y);
    } else if (mod == 3) {
      newCoordinates = new CartesianCoordinates(cell_x - random_num_x,
                                                cell_y - random_num_y);
    }
    newCoordinates->SetCellID(cell_index);
    vectorOfCoordinates->push_back(newCoordinates);
  }
  return vectorOfCoordinates;
}

static inline double deg2rad(double d) { return d * M_PI / 180.0; }

static bool IsTooCloseToAnySite(double x, double y,
                                const std::vector<Cell *> *cells,
                                double minDistMeters) {
  for (const auto *cell : *cells) {
    const CartesianCoordinates *cc = cell->GetCellCenterPosition();
    auto *cc_nc = const_cast<CartesianCoordinates *>(cc);
    double dx = x - cc_nc->GetCoordinateX();
    double dy = y - cc_nc->GetCoordinateY();
    if (std::hypot(dx, dy) < minDistMeters)
      return true;
  }
  return false;
}

static std::vector<CartesianCoordinates *> *
GetInterferenceLimitedUsersDistribution(int nbUE) {
  NetworkManager *networkManager = NetworkManager::Init();
  vector<CartesianCoordinates *> *vectorOfCoordinates =
      new vector<CartesianCoordinates *>;
  for (int i = 0; i < nbUE / 3; i++) {
    Cell *c = networkManager->GetCellContainer()->at(0);
    CartesianCoordinates *cellCoordinates = c->GetCellCenterPosition();
    if (i == 0) {
      double x_cd = cellCoordinates->GetCoordinateX() + (100);
      double y_cd = cellCoordinates->GetCoordinateY() + (100);
      CartesianCoordinates *newCoordinates =
          new CartesianCoordinates(x_cd, y_cd);
      newCoordinates->SetCellID(0);
      vectorOfCoordinates->push_back(newCoordinates);
    } else {
      double x_cd = 0;
      double y_cd = 370;
      x_cd += (rand() % 70);
      CartesianCoordinates *newCoordinates =
          new CartesianCoordinates(x_cd, y_cd);
      newCoordinates->SetCellID(0);
      vectorOfCoordinates->push_back(newCoordinates);
    }
  }
  for (int i = nbUE / 3; i < nbUE; i++) {
    Cell *c = networkManager->GetCellContainer()->at(1);
    CartesianCoordinates *cellCoordinates = c->GetCellCenterPosition();
    if (i == nbUE / 3) {
      cout << "Here" << endl;
      double x_cd = 0;
      double y_cd = 390;
      x_cd += (rand() % 70);
      CartesianCoordinates *newCoordinates =
          new CartesianCoordinates(x_cd, y_cd);
      newCoordinates->SetCellID(1);
      vectorOfCoordinates->push_back(newCoordinates);
    } else {
      double x_cd = cellCoordinates->GetCoordinateX() + (rand() % 100);
      double y_cd = cellCoordinates->GetCoordinateY() + (rand() % 100);
      CartesianCoordinates *newCoordinates =
          new CartesianCoordinates(x_cd, y_cd);
      newCoordinates->SetCellID(1);
      vectorOfCoordinates->push_back(newCoordinates);
    }
  }
  return vectorOfCoordinates;
}

static vector<CartesianCoordinates *> *
GetUniformUsersDistributionInFemtoCell(int idCell, int nbUE) {
  NetworkManager *networkManager = NetworkManager::Init();
  vector<CartesianCoordinates *> *vectorOfCoordinates =
      new vector<CartesianCoordinates *>;

  Femtocell *cell = networkManager->GetFemtoCellByID(idCell);

  double side = cell->GetSide();

  CartesianCoordinates *cellCoordinates = cell->GetCellCenterPosition();
  double r;
  double angle;

  for (int i = 0; i < nbUE; i++) {
    r = (double)(rand() % (int)side);
    angle = (double)(rand() % 360) * ((2 * 3.14) / 360);

    CartesianCoordinates *newCoordinates =
        GetCartesianCoordinatesFromPolar(r, angle);

    // Compute absoluteCoordinates
    newCoordinates->SetCoordinateX(cellCoordinates->GetCoordinateX() +
                                   newCoordinates->GetCoordinateX());
    newCoordinates->SetCoordinateY(cellCoordinates->GetCoordinateY() +
                                   newCoordinates->GetCoordinateY());

    vectorOfCoordinates->push_back(newCoordinates);
  }

  return vectorOfCoordinates;
}

#endif /* USERSDISTRIBTION_H_ */
