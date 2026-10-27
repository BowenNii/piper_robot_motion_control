#include "experiment_io/fk_csv_writer.hpp"
#include <cerrno>
#include <cmath>
#include <cstring>
#include <fcntl.h>
#include <iomanip>
#include <locale>
#include <sstream>
#include <stdexcept>
#include <unistd.h>

namespace experiment_io {
namespace {
std::runtime_error io_error(const std::filesystem::path &path) {
  return std::runtime_error("CSV I/O error: " + path.string() + ": " +
                            std::strerror(errno));
}
} // namespace
FkCsvWriter::FkCsvWriter(const std::filesystem::path &path) : path_(path) {
  // 独占创建在操作系统中一次完成，避免 exists() 后又被别人创建的竞争。
  fd_ = ::open(path.c_str(), O_WRONLY | O_CREAT | O_EXCL | O_CLOEXEC, 0644);
  if (fd_ < 0)
    throw io_error(path_);
  try {
    std::ostringstream h;
    h << "t_s,timestamp_s,frame_span_s"
      << ",q1_rad,q2_rad,q3_rad,q4_rad,q5_rad,q6_rad"
      << ",feedback_x_m,feedback_y_m,feedback_z_m"
      << ",feedback_roll_rad,feedback_pitch_rad,feedback_yaw_rad"
      << ",poe_x_m,poe_y_m,poe_z_m";
    for (int row = 1; row <= 3; ++row)
      for (int col = 1; col <= 3; ++col)
        h << ",poe_r" << row << col;
    h << ",position_error_m,rotation_error_rad\n";
    write_text(h.str());
  } catch (...) {
    ::close(fd_);
    fd_ = -1;
    throw;
  }
}
FkCsvWriter::~FkCsvWriter() {
  if (fd_ >= 0)
    ::close(fd_);
}
void FkCsvWriter::write_text(const std::string &text) {
  if (fd_ < 0 || failed_)
    throw std::runtime_error("CSV is closed or failed: " + path_.string());
  std::size_t done = 0;
  // 一次 write 可能只写一部分，或被信号中断；必须继续写完。
  while (done < text.size()) {
    const auto n = ::write(fd_, text.data() + done, text.size() - done);
    if (n < 0 && errno == EINTR)
      continue;
    if (n <= 0) {
      if (n == 0)
        errno = EIO;
      failed_ = true;
      throw io_error(path_);
    }
    done += static_cast<std::size_t>(n);
  }
}
void FkCsvWriter::write(const FkValidationResult &r) {
  if (fd_ < 0 || failed_)
    throw std::runtime_error("CSV is closed or failed: " + path_.string());
  if (!std::isfinite(r.t_s) || r.t_s < 0 || !std::isfinite(r.timestamp_s) ||
      r.timestamp_s < 0 || !std::isfinite(r.frame_span_s) ||
      r.frame_span_s < 0 || !r.q_rad.allFinite() ||
      !piper_control::check_se3(r.T_feedback).ok ||
      !piper_control::check_se3(r.T_poe).ok ||
      !std::isfinite(r.error.position_m) || r.error.position_m < 0 ||
      !std::isfinite(r.error.rotation_rad) || r.error.rotation_rad < 0 ||
      r.error.rotation_rad > std::acos(-1.0) || !r.error.delta_p_m.allFinite())
    throw std::invalid_argument("Invalid FK CSV sample");
  for (const double value : r.feedback_rpy_rad)
    if (!std::isfinite(value))
      throw std::invalid_argument("Invalid CSV RPY");
  std::ostringstream s;
  s.imbue(std::locale::classic());
  s << std::setprecision(17) << r.t_s << ',' << r.timestamp_s << ','
    << r.frame_span_s;
  for (Eigen::Index i = 0; i < 6; ++i)
    s << ',' << r.q_rad(i);
  for (Eigen::Index i = 0; i < 3; ++i)
    s << ',' << r.T_feedback(i, 3);
  for (const double value : r.feedback_rpy_rad)
    s << ',' << value;
  for (Eigen::Index i = 0; i < 3; ++i)
    s << ',' << r.T_poe(i, 3);
  for (Eigen::Index row = 0; row < 3; ++row)
    for (Eigen::Index col = 0; col < 3; ++col)
      s << ',' << r.T_poe(row, col);
  s << ',' << r.error.position_m << ',' << r.error.rotation_rad << '\n';
  write_text(s.str());
  ++row_count_;
}
void FkCsvWriter::close() {
  if (fd_ < 0)
    return;
  const int fd = fd_;
  fd_ = -1;
  // fsync检查延迟写入失败；不能承诺多文件原子事务。
  if (::fsync(fd) != 0) {
    const int saved = errno;
    ::close(fd);
    errno = saved;
    throw io_error(path_);
  }
  if (::close(fd) != 0)
    throw io_error(path_);
  if (failed_)
    throw std::runtime_error("CSV output is incomplete: " + path_.string());
}
} // namespace experiment_io
