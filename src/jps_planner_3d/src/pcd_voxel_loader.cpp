#include "jps_planner_3d/pcd_voxel_loader.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <fstream>
#include <limits>
#include <sstream>
#include <string>
#include <unordered_map>
#include <vector>

namespace jps_planner_3d
{

namespace
{

struct VoxelKey
{
  int x;
  int y;
  int z;

  bool operator==(const VoxelKey & other) const
  {
    return x == other.x && y == other.y && z == other.z;
  }
};

struct VoxelKeyHash
{
  std::size_t operator()(const VoxelKey & key) const
  {
    std::size_t h1 = std::hash<int>{}(key.x);
    std::size_t h2 = std::hash<int>{}(key.y);
    std::size_t h3 = std::hash<int>{}(key.z);
    std::size_t seed = h1;
    seed ^= h2 + 0x9e3779b9 + (seed << 6) + (seed >> 2);
    seed ^= h3 + 0x9e3779b9 + (seed << 6) + (seed >> 2);
    return seed;
  }
};

bool splitInts(const std::string & s, std::vector<int> & out)
{
  out.clear();
  std::istringstream iss(s);
  int v = 0;
  while (iss >> v) {
    out.push_back(v);
  }
  return !out.empty();
}

bool splitStrings(const std::string & s, std::vector<std::string> & out)
{
  out.clear();
  std::istringstream iss(s);
  std::string token;
  while (iss >> token) {
    out.push_back(token);
  }
  return !out.empty();
}

bool readFieldValue(
  const char * ptr,
  int size,
  char type,
  double & value)
{
  value = 0.0;
  switch (type) {
    case 'F':
    case 'f':
      if (size == 4) {
        float v = 0.0f;
        std::memcpy(&v, ptr, sizeof(float));
        value = static_cast<double>(v);
        return true;
      }
      if (size == 8) {
        double v = 0.0;
        std::memcpy(&v, ptr, sizeof(double));
        value = v;
        return true;
      }
      return false;
    case 'I':
    case 'i':
      if (size == 1) {
        int8_t v = 0;
        std::memcpy(&v, ptr, sizeof(int8_t));
        value = static_cast<double>(v);
        return true;
      }
      if (size == 2) {
        int16_t v = 0;
        std::memcpy(&v, ptr, sizeof(int16_t));
        value = static_cast<double>(v);
        return true;
      }
      if (size == 4) {
        int32_t v = 0;
        std::memcpy(&v, ptr, sizeof(int32_t));
        value = static_cast<double>(v);
        return true;
      }
      if (size == 8) {
        int64_t v = 0;
        std::memcpy(&v, ptr, sizeof(int64_t));
        value = static_cast<double>(v);
        return true;
      }
      return false;
    case 'U':
    case 'u':
      if (size == 1) {
        uint8_t v = 0;
        std::memcpy(&v, ptr, sizeof(uint8_t));
        value = static_cast<double>(v);
        return true;
      }
      if (size == 2) {
        uint16_t v = 0;
        std::memcpy(&v, ptr, sizeof(uint16_t));
        value = static_cast<double>(v);
        return true;
      }
      if (size == 4) {
        uint32_t v = 0;
        std::memcpy(&v, ptr, sizeof(uint32_t));
        value = static_cast<double>(v);
        return true;
      }
      if (size == 8) {
        uint64_t v = 0;
        std::memcpy(&v, ptr, sizeof(uint64_t));
        value = static_cast<double>(v);
        return true;
      }
      return false;
    default:
      return false;
  }
}

void accumulateVoxel(
  double x,
  double y,
  double z,
  double resolution,
  double max_abs_coord,
  std::unordered_map<VoxelKey, uint32_t, VoxelKeyHash> & voxel_counts,
  int64_t & valid_point_count)
{
  if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) {
    return;
  }
  if (std::abs(x) > max_abs_coord || std::abs(y) > max_abs_coord || std::abs(z) > max_abs_coord) {
    return;
  }

  VoxelKey key;
  key.x = static_cast<int>(std::floor(x / resolution));
  key.y = static_cast<int>(std::floor(y / resolution));
  key.z = static_cast<int>(std::floor(z / resolution));

  ++voxel_counts[key];
  ++valid_point_count;
}

}  // namespace

bool loadAndVoxelizePcd(
  const std::string & pcd_path,
  double resolution,
  int min_points_per_voxel,
  double max_abs_coord,
  PcdVoxelizeResult & result,
  std::string & error_msg)
{
  result = PcdVoxelizeResult{};

  if (resolution <= 0.0) {
    error_msg = "resolution must be > 0";
    return false;
  }
  if (min_points_per_voxel < 1) {
    min_points_per_voxel = 1;
  }
  if (max_abs_coord <= 0.0 || !std::isfinite(max_abs_coord)) {
    max_abs_coord = 200.0;
  }

  std::ifstream in(pcd_path, std::ios::binary);
  if (!in.is_open()) {
    error_msg = "failed to open pcd file: " + pcd_path;
    return false;
  }

  std::unordered_map<std::string, std::string> header;
  std::string data_type;
  std::string line;
  while (std::getline(in, line)) {
    if (!line.empty() && line.back() == '\r') {
      line.pop_back();
    }
    if (line.empty() || line[0] == '#') {
      continue;
    }
    std::istringstream ls(line);
    std::string key;
    ls >> key;
    if (key.empty()) {
      continue;
    }
    if (key == "DATA") {
      ls >> data_type;
      break;
    }
    std::string value;
    std::getline(ls, value);
    if (!value.empty() && value[0] == ' ') {
      value.erase(0, 1);
    }
    header[key] = value;
  }

  if (data_type.empty()) {
    error_msg = "invalid pcd header: missing DATA section";
    return false;
  }

  std::vector<std::string> fields;
  if (!splitStrings(header.count("FIELDS") ? header.at("FIELDS") : "x y z", fields)) {
    error_msg = "invalid pcd header: FIELDS";
    return false;
  }

  std::vector<int> sizes;
  if (!splitInts(header.count("SIZE") ? header.at("SIZE") : "4 4 4", sizes)) {
    error_msg = "invalid pcd header: SIZE";
    return false;
  }

  std::vector<std::string> types;
  if (!splitStrings(header.count("TYPE") ? header.at("TYPE") : "F F F", types)) {
    error_msg = "invalid pcd header: TYPE";
    return false;
  }

  std::vector<int> counts;
  if (header.count("COUNT")) {
    if (!splitInts(header.at("COUNT"), counts)) {
      error_msg = "invalid pcd header: COUNT";
      return false;
    }
  } else {
    counts.assign(fields.size(), 1);
  }

  if (fields.size() != sizes.size() || fields.size() != types.size() || fields.size() != counts.size()) {
    error_msg = "invalid pcd header: inconsistent FIELDS/SIZE/TYPE/COUNT";
    return false;
  }

  auto findField = [&](const std::string & name) -> int {
    for (size_t i = 0; i < fields.size(); ++i) {
      if (fields[i] == name) {
        return static_cast<int>(i);
      }
    }
    return -1;
  };

  const int x_idx = findField("x");
  const int y_idx = findField("y");
  const int z_idx = findField("z");
  if (x_idx < 0 || y_idx < 0 || z_idx < 0) {
    error_msg = "pcd missing x/y/z fields";
    return false;
  }

  std::vector<int> field_widths(fields.size(), 0);
  std::vector<int> offsets(fields.size(), 0);
  int point_step = 0;
  for (size_t i = 0; i < fields.size(); ++i) {
    field_widths[i] = sizes[i] * counts[i];
    offsets[i] = point_step;
    point_step += field_widths[i];
  }

  int64_t point_count = 0;
  if (header.count("POINTS")) {
    try {
      point_count = std::stoll(header.at("POINTS"));
    } catch (...) {
      point_count = 0;
    }
  }
  if (point_count <= 0 && header.count("WIDTH") && header.count("HEIGHT")) {
    try {
      point_count = std::stoll(header.at("WIDTH")) * std::stoll(header.at("HEIGHT"));
    } catch (...) {
      point_count = 0;
    }
  }

  std::unordered_map<VoxelKey, uint32_t, VoxelKeyHash> voxel_counts;
  if (point_count > 0) {
    voxel_counts.reserve(static_cast<size_t>(point_count / 2));
  }

  int64_t valid_point_count = 0;

  if (data_type == "ascii") {
    while (std::getline(in, line)) {
      if (!line.empty() && line.back() == '\r') {
        line.pop_back();
      }
      if (line.empty()) {
        continue;
      }

      std::istringstream ss(line);
      std::vector<double> values;
      values.reserve(fields.size());
      double v = 0.0;
      while (ss >> v) {
        values.push_back(v);
      }
      if (static_cast<int>(values.size()) <= std::max({x_idx, y_idx, z_idx})) {
        continue;
      }

      accumulateVoxel(
        values[x_idx], values[y_idx], values[z_idx],
        resolution, max_abs_coord,
        voxel_counts, valid_point_count);
    }
  } else if (data_type == "binary") {
    std::vector<char> data((std::istreambuf_iterator<char>(in)), std::istreambuf_iterator<char>());
    if (data.empty()) {
      error_msg = "empty binary pcd payload";
      return false;
    }

    if (point_step <= 0) {
      error_msg = "invalid point_step in pcd header";
      return false;
    }

    int64_t max_points_by_payload = static_cast<int64_t>(data.size() / static_cast<size_t>(point_step));
    if (point_count <= 0 || point_count > max_points_by_payload) {
      point_count = max_points_by_payload;
    }

    const int x_offset = offsets[x_idx];
    const int y_offset = offsets[y_idx];
    const int z_offset = offsets[z_idx];

    for (int64_t i = 0; i < point_count; ++i) {
      const size_t base = static_cast<size_t>(i) * static_cast<size_t>(point_step);
      const char * point_ptr = data.data() + base;

      double x = 0.0, y = 0.0, z = 0.0;
      if (!readFieldValue(point_ptr + x_offset, sizes[x_idx], types[x_idx].empty() ? 'F' : types[x_idx][0], x)) {
        continue;
      }
      if (!readFieldValue(point_ptr + y_offset, sizes[y_idx], types[y_idx].empty() ? 'F' : types[y_idx][0], y)) {
        continue;
      }
      if (!readFieldValue(point_ptr + z_offset, sizes[z_idx], types[z_idx].empty() ? 'F' : types[z_idx][0], z)) {
        continue;
      }

      accumulateVoxel(x, y, z, resolution, max_abs_coord, voxel_counts, valid_point_count);
    }
  } else {
    error_msg = "unsupported pcd DATA type: " + data_type;
    return false;
  }

  result.raw_point_count = valid_point_count;
  result.occupied_voxels.clear();
  result.occupied_voxels.reserve(voxel_counts.size());

  for (const auto & kv : voxel_counts) {
    if (static_cast<int>(kv.second) < min_points_per_voxel) {
      continue;
    }
    const auto & key = kv.first;
    float cx = static_cast<float>((static_cast<double>(key.x) + 0.5) * resolution);
    float cy = static_cast<float>((static_cast<double>(key.y) + 0.5) * resolution);
    float cz = static_cast<float>((static_cast<double>(key.z) + 0.5) * resolution);
    result.occupied_voxels.emplace_back(cx, cy, cz);
  }

  if (result.occupied_voxels.empty()) {
    error_msg = "voxelization produced empty occupied set";
    return false;
  }

  return true;
}

}  // namespace jps_planner_3d
