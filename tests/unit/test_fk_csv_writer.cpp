#include "experiment_io/fk_csv_writer.hpp"
#include "hal/can_log_parser.hpp"
#include "piper_control/kinematics/forward_kinematics.hpp"
#include "piper_control/robot_model/piper_model.hpp"
#include <algorithm>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <limits>
#include <locale>
#include <stdexcept>
#include <vector>

void require(bool ok, const char *message) {
  if (!ok)
    throw std::runtime_error(message);
}
template <class F> void rejected(F f) {
  bool threw = false;
  try {
    f();
  } catch (const std::exception &) {
    threw = true;
  }
  require(threw, "expected rejection");
}
struct TempDirectory {
  std::filesystem::path path;
  TempDirectory() {
    std::string name =
        (std::filesystem::temp_directory_path() / "piper-csv-test.XXXXXX")
            .string();
    std::vector<char> buffer(name.begin(), name.end());
    buffer.push_back('\0');
    auto p = ::mkdtemp(buffer.data());
    if (!p)
      throw std::runtime_error("mkdtemp");
    path = p;
  }
  ~TempDirectory() {
    std::error_code ec;
    std::filesystem::remove_all(path, ec);
  }
};
struct CommaLocale : std::numpunct<char> {
  char do_decimal_point() const override { return ','; }
};
int main() try {
  TempDirectory tmp;
  const auto output = tmp.path / "result.csv";
  experiment_io::FkValidationResult result;
  result.timestamp_s = 123.125;
  result.frame_span_s = 0.001;
  result.q_rad(0) = 0.12345678901234567;
  result.feedback_rpy_rad = {0.1, 0.2, 0.3};
  // Writer preserves supplied raw RPY; it does not independently rebuild FK.
  {
    const auto original = std::locale();
    std::locale::global(std::locale(original, new CommaLocale));
    experiment_io::FkCsvWriter writer(output);
    writer.write(result);
    require(writer.row_count() == 1, "row count");
    auto bad = result;
    bad.q_rad(0) = std::numeric_limits<double>::quiet_NaN();
    rejected([&] { writer.write(bad); });
    require(writer.row_count() == 1, "invalid row does not increment");
    writer.close();
    writer.close();
    rejected([&] { writer.write(result); });
    std::locale::global(original);
  }
  std::ifstream input(output);
  std::string header, line, extra;
  std::getline(input, header);
  std::getline(input, line);
  require(std::count(header.begin(), header.end(), ',') == 28, "29 columns");
  require(header.find("poe_r11,poe_r12,poe_r13,poe_r21") != std::string::npos,
          "row-major R");
  require(std::count(line.begin(), line.end(), ',') == 28,
          "classic decimal point");
  require(line.find("123.125") != std::string::npos, "precision/locale");
  require(!std::getline(input, extra), "one row only");
  rejected([&] { experiment_io::FkCsvWriter again(output); });
  std::ifstream unchanged(output);
  std::string still;
  std::getline(unchanged, still);
  require(still == header, "existing file not overwritten");
  rejected([&] {
    experiment_io::FkCsvWriter missing(tmp.path / "absent" / "x.csv");
  });
  std::filesystem::create_symlink(tmp.path / "absent.csv",
                                  tmp.path / "link.csv");
  rejected([&] { experiment_io::FkCsvWriter link(tmp.path / "link.csv"); });

  // Synthetic six-frame integration: parser -> assembler -> FK/error -> CSV.
  hal::PiperFeedbackAssembler assembler;
  const auto model = piper_control::make_piper_model();
  experiment_io::FkCsvWriter integrated(tmp.path / "pipeline.csv");
  for (unsigned id = 0x2A2; id <= 0x2A7; ++id) {
    hal::TimedCanFrame f;
    f.frame.id = id;
    f.frame.length = 8;
    f.timestamp_seconds = 10 + (id - 0x2A2) * 0.0001;
    f.interface_name = "can0";
    auto sample = assembler.push(f);
    if (sample) {
      result.q_rad.setZero();
      result.timestamp_s = sample->timestamp_seconds;
      result.frame_span_s = sample->span_seconds;
      result.feedback_rpy_rad = sample->end_pose.rpy_rad;
      result.T_poe =
          piper_control::forward_poe_space(model.S_list, result.q_rad, model.M);
      result.T_feedback.setIdentity();
      result.error = piper_control::pose_error(result.T_poe, result.T_feedback);
      integrated.write(result);
    }
  }
  integrated.close();
  require(integrated.row_count() == 1, "pipeline output");
  std::cout << "FK CSV writer PASS\n";
  return 0;
} catch (const std::exception &e) {
  std::cerr << e.what() << '\n';
  return 1;
}
