// Copyright (C) 2017  I. Bogoslavskyi, C. Stachniss, University of Bonn

// This program is free software: you can redistribute it and/or modify it
// under the terms of the GNU General Public License as published by the Free
// Software Foundation, either version 3 of the License, or (at your option)
// any later version.

// This program is distributed in the hope that it will be useful, but WITHOUT
// ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
// FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
// more details.

// You should have received a copy of the GNU General Public License along
// with this program.  If not, see <http://www.gnu.org/licenses/>.

#include "projection_params.h"

#include <boost/algorithm/string.hpp>
#include <boost/algorithm/string/predicate.hpp>

#include <algorithm>
#include <fstream>
#include <string>
#include <vector>

namespace kamaz {

namespace hagen{

using std::vector;
using std::string;
using std::upper_bound;
using boost::algorithm::starts_with;

void ProjectionParams::SetSpan(const SpanParams& span_params,
                               const SpanParams::Direction& direction) {
  vector<SpanParams> params_vec = {{span_params}};
  this->SetSpan(params_vec, direction);
}

void ProjectionParams::SetSpan(const vector<SpanParams>& span_params,
                               const SpanParams::Direction& direction) {
  int num_beams = 0;
  for (const auto& span : span_params) {
    num_beams += span.num_beams();
  }
  switch (direction) {
    case SpanParams::Direction::HORIZONTAL:
      _h_span_params = SpanParams(span_params.front().start_angle(),
                                  span_params.back().end_angle(), num_beams);
      _col_angles = FillVector(span_params);
      break;
    case SpanParams::Direction::VERTICAL:
      _v_span_params = SpanParams(span_params.front().start_angle(),
                                  span_params.back().end_angle(), num_beams);
      _row_angles = FillVector(span_params);
      break;
  }
  FillCosSin();
}

vector<Radians> ProjectionParams::FillVector(const SpanParams& span_params) {
  vector<SpanParams> params_vec = {{span_params}};
  return this->FillVector(params_vec);
}

vector<Radians> ProjectionParams::FillVector(
    const vector<SpanParams>& span_params) {
  vector<Radians> res;
  for (const auto span_param : span_params) {
    Radians rad = span_param.start_angle();
    for (int i = 0; i < span_param.num_beams(); ++i) {
      res.push_back(rad);
      rad += span_param.step();
    }
  }
  return res;
}

double ProjectionParams::getMaxDistance(){
  return object_avoidance_zone;
}

bool ProjectionParams::valid() {
  bool all_params_valid = _v_span_params.valid() && _h_span_params.valid();
  bool arrays_empty = _row_angles.empty() && _col_angles.empty();
  bool cos_sin_empty = _row_angles_sines.empty() &&
                       _row_angles_cosines.empty() &&
                       _col_angles_sines.empty() && _col_angles_cosines.empty();
  if (!all_params_valid) {
    throw std::runtime_error("Projection parameters invalid.");
  }
  if (arrays_empty) {
    throw std::runtime_error("Projection parameters arrays not filled.");
  }
  if (cos_sin_empty) {
    throw std::runtime_error(
        "Projection parameters sin and cos arrays not filled.");
  }
  return true;
}

const Radians ProjectionParams::AngleFromRow(int row) const {
  if (row >= 0 && static_cast<size_t>(row) < _row_angles.size()) {
    return _row_angles[row];
  }
  fprintf(stderr, "ERROR: row %d is wrong\n", row);
  return 0.0_deg;
}

const Radians ProjectionParams::AngleFromCol(int col) const {
  int actual_col = col;
  if (col < 0) {
    actual_col = col + _col_angles.size();
  } else if (static_cast<size_t>(col) >= _col_angles.size()) {
    actual_col = col - _col_angles.size();
  }
  // everything is normal
  return _col_angles[actual_col];
}

size_t ProjectionParams::RowFromAngle(const Radians& angle) const {
  return FindClosest(_row_angles, angle);
}

size_t ProjectionParams::ColFromAngle(const Radians& angle) const {
  return FindClosest(_col_angles, angle);
}

size_t ProjectionParams::FindClosest(const vector<Radians>& vec,
                                     const Radians& val) {
  size_t found = 0;
  if (vec.front() < vec.back()) {
    found = upper_bound(vec.begin(), vec.end(), val) - vec.begin();
  } else {
    found = vec.rend() - upper_bound(vec.rbegin(), vec.rend(), val);
  }
  if (found == 0) {
    return found;
  }
  if (found == vec.size()) {
    return found - 1;
  }
  auto diff_next = Radians::Abs(vec[found] - val);
  auto diff_prev = Radians::Abs(val - vec[found - 1]);
  return diff_next < diff_prev ? found : found - 1;
}

std::unique_ptr<ProjectionParams> ProjectionParams::VLP_16(double object_avoidance_zone) {
  auto params = ProjectionParams();
  params.object_avoidance_zone = object_avoidance_zone;
  params.SetSpan(SpanParams(-180_deg, 180_deg, 870),
                 SpanParams::Direction::HORIZONTAL);
  params.SetSpan(SpanParams(15_deg, -15_deg, 16),
                 SpanParams::Direction::VERTICAL);
  params.FillCosSin();
  if (!params.valid()) {
    fprintf(stderr, "ERROR: params are not valid!\n");
    return nullptr;
  }
  return mem_utils::make_unique<ProjectionParams>(params);
}

std::unique_ptr<ProjectionParams> ProjectionParams::HDL_32(double object_avoidance_zone) {
  auto params = ProjectionParams();
  params.object_avoidance_zone = object_avoidance_zone;
  params.SetSpan(SpanParams(-180_deg, 180_deg, 870),
                 SpanParams::Direction::HORIZONTAL);
  params.SetSpan(SpanParams(10.0_deg, -30.0_deg, 32),
                 SpanParams::Direction::VERTICAL);
  params.FillCosSin();
  if (!params.valid()) {
    fprintf(stderr, "ERROR: params are not valid!\n");
    return nullptr;
  }
  return mem_utils::make_unique<ProjectionParams>(params);
}

std::unique_ptr<ProjectionParams> ProjectionParams::HDL_64(double object_avoidance_zone) {
  auto params = ProjectionParams();
  params.object_avoidance_zone = object_avoidance_zone;
  params.SetSpan(SpanParams(-180_deg, 180_deg, 870),
                 SpanParams::Direction::HORIZONTAL);
  SpanParams span_top(2.0_deg, -8.5_deg, 32);
  SpanParams span_bottom(-8.87_deg, -24.87_deg, 32);
  vector<SpanParams> spans = {{span_top, span_bottom}};
  params.SetSpan(spans, SpanParams::Direction::VERTICAL);
  params.FillCosSin();
  if (!params.valid()) {
    fprintf(stderr, "ERROR: params are not valid!\n");
    return nullptr;
  }
  return mem_utils::make_unique<ProjectionParams>(params);
}

const std::vector<float>& ProjectionParams::RowAngleCosines() const {
  return _row_angles_cosines;
}
const std::vector<float>& ProjectionParams::ColAngleCosines() const {
  return _col_angles_cosines;
}
const std::vector<float>& ProjectionParams::RowAngleSines() const {
  return _row_angles_sines;
}
const std::vector<float>& ProjectionParams::ColAngleSines() const {
  return _col_angles_sines;
}

void ProjectionParams::FillCosSin() {
  _row_angles_sines.clear();
  _row_angles_cosines.clear();
  for (const auto& angle : _row_angles) {
    _row_angles_sines.push_back(sin(angle.val()));
    _row_angles_cosines.push_back(cos(angle.val()));
  }
  _col_angles_sines.clear();
  _col_angles_cosines.clear();
  for (const auto& angle : _col_angles) {
    _col_angles_sines.push_back(sin(angle.val()));
    _col_angles_cosines.push_back(cos(angle.val()));
  }
}

}
}  // namespace kamaz