#pragma once
#include "experiment_io/fk_validation_result.hpp"
#include <cstddef>
#include <filesystem>
#include <string>

namespace experiment_io {
// 【IO-02】路径 -> 独占创建CSV；write(结果)写一行；close显式检查关闭错误。
// 保留project1的29列列名/顺序，classic
// locale、17位有效数字，角度rad/长度m/时间s。
// Ubuntu/Linux文件工具：O_EXCL拒绝覆盖（包括已有符号链接），不自动建目录。
// I/O错误抛runtime_error；格式错误抛invalid_argument。失败文件可能不完整，不自动删除。
// 析构不抛异常，正常结束必须调用close；关闭后不允许再write，不支持拷贝/移动。
class FkCsvWriter {
public:
  explicit FkCsvWriter(const std::filesystem::path &path);
  ~FkCsvWriter();
  FkCsvWriter(const FkCsvWriter &) = delete;
  FkCsvWriter &operator=(const FkCsvWriter &) = delete;
  void write(const FkValidationResult &result);
  void close();
  [[nodiscard]] std::size_t row_count() const { return row_count_; }

private:
  void write_text(const std::string &text);
  int fd_ = -1;
  std::filesystem::path path_;
  std::size_t row_count_ = 0;
  bool failed_ = false;
};
} // namespace experiment_io
