// Converts Livox's scan-pattern CSV (Time, Azimuth, Zenith in degrees; UnitySensors' mid360.csv) to the plant's binary:
// "MMPSCAN1", uint32 count, count x float32 (azimuth, zenith). Every data row is kept, as UnitySensors' generator does.
#include <cstdint>
#include <cstdio>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

int main(int argc, char ** argv)
{
  if (argc != 3) {
    std::cerr << "usage: make_scan_pattern IN.csv OUT.bin\n";
    return 2;
  }
  std::ifstream in(argv[1]);
  std::string line;
  if (!in || !std::getline(in, line)) {
    std::cerr << "make_scan_pattern: cannot read " << argv[1] << "\n";
    return 1;
  }
  // header: the azimuth and zenith columns by name
  int azimuth = -1, zenith = -1, column = 0;
  std::stringstream header(line);
  for (std::string name; std::getline(header, name, ','); ++column) {
    for (auto & ch : name) ch = static_cast<char>(std::tolower(ch));
    if (name.find("zenith") != std::string::npos) zenith = column;
    else if (name.find("azimuth") != std::string::npos) azimuth = column;
  }
  if (azimuth < 0 || zenith < 0) {
    std::cerr << "make_scan_pattern: no azimuth/zenith columns\n";
    return 1;
  }
  std::vector<float> data;
  while (std::getline(in, line)) {
    if (line.empty() || line == "\r") continue;
    std::stringstream row(line);
    std::vector<std::string> cells;
    for (std::string cell; std::getline(row, cell, ',');) cells.push_back(cell);
    if (static_cast<int>(cells.size()) <= std::max(azimuth, zenith)) {
      std::cerr << "make_scan_pattern: short row " << line << "\n";
      return 1;
    }
    data.push_back(std::stof(cells[azimuth]));
    data.push_back(std::stof(cells[zenith]));
  }
  std::ofstream out(argv[2], std::ios::binary);
  const uint32_t count = static_cast<uint32_t>(data.size() / 2);
  out.write("MMPSCAN1", 8);
  out.write(reinterpret_cast<const char *>(&count), sizeof count);
  out.write(reinterpret_cast<const char *>(data.data()), static_cast<std::streamsize>(data.size() * sizeof(float)));
  std::cout << count << " directions -> " << argv[2] << "\n";
  return out ? 0 : 1;
}
