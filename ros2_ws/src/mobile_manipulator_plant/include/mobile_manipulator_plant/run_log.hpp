#pragma once
// The run log (MuJoCo plant spec Section 6.4): the frames of a run, zlib-compressed in chunks of about 1 MiB of whole
// frames, after a Hello frame carrying the run's header. File: "MMRUNLG1", then {[u32 raw_len][u32 zlib_len][zlib]}.
// The same frames give the same bytes (E2).
#include <nlohmann/json.hpp>

#include <fstream>
#include <string>

namespace mobile_manipulator_plant
{
class RunLogWriter
{
public:
  RunLogWriter(const std::string & path, const nlohmann::json & header);
  ~RunLogWriter();
  RunLogWriter(const RunLogWriter &) = delete;
  RunLogWriter & operator=(const RunLogWriter &) = delete;
  void write(const std::string & frame);  // one encoded frame
  void close();

private:
  void flush();
  std::ofstream out_;
  std::string raw_;
  bool open_ = false;
};

class RunLogReader
{
public:
  explicit RunLogReader(const std::string & path);  // std::runtime_error if it is not a run log
  const nlohmann::json & header() const { return header_; }
  bool next(std::string & frame);  // the next encoded frame; false at the end

private:
  bool load_chunk();
  std::ifstream in_;
  std::string raw_;
  size_t at_ = 0;
  nlohmann::json header_;
};
}  // namespace mobile_manipulator_plant
