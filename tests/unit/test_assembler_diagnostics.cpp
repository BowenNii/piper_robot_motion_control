#include "hal/can_log_parser.hpp"
#include <iostream>
#include <stdexcept>

void require(bool ok) {
  if (!ok)
    throw std::runtime_error("diagnostics assertion");
}
hal::TimedCanFrame frame(unsigned id, double t) {
  hal::TimedCanFrame f;
  f.frame.id = id;
  f.frame.length = 8;
  f.timestamp_seconds = t;
  f.interface_name = "can0";
  return f;
}
int main() try {
  hal::PiperFeedbackAssembler a;
  auto push = [&](unsigned id, double t) { return a.push(frame(id, t)); };
  push(0x2A1, 1);
  push(0x2A2, 1);
  push(0x2A2, 1.001); // duplicate: discard one old frame
  push(0x2A3, 1.004); // over window
  push(0x2A4, 1.003); // reversal
  auto changed = frame(0x2A5, 1.0031);
  changed.interface_name = "can1";
  (void)a.push(changed);
  a.reset();
  a.reset();
  const auto d = a.diagnostics();
  require(d.received_frames == 6 && d.ignored_frames == 1);
  require(d.discarded_groups == 5 && d.discarded_frames == 5);
  require(d.duplicate_ids == 1 && d.over_window == 1 && d.time_reversals == 1 &&
          d.interface_changes == 1 && d.manual_resets == 1);
  a.clear_diagnostics();
  for (unsigned i = 0; i < 6; ++i) {
    auto s = push(0x2A2 + i, 2 + i * 0.0001);
    require(s.has_value() == (i == 5));
  }
  require(a.diagnostics().completed_samples == 1 &&
          a.diagnostics().discarded_groups == 0);
  auto bad = frame(0x2A2, 3);
  bad.frame.length = 7;
  bool thrown = false;
  try {
    (void)a.push(bad);
  } catch (const std::invalid_argument &) {
    thrown = true;
  }
  require(thrown && a.diagnostics().invalid_frames == 1);
  push(0x2A2, 3);
  a.clear_diagnostics();
  require(a.pending_frame_count() == 1 && a.diagnostics().received_frames == 0);
  // Simultaneous causes count separately but discard only once.
  push(0x2A2, 4);
  require(a.diagnostics().duplicate_ids == 1 &&
          a.diagnostics().over_window == 1 &&
          a.diagnostics().discarded_groups == 1);
  std::cout << "Assembler diagnostics PASS\n";
  return 0;
} catch (const std::exception &e) {
  std::cerr << e.what() << '\n';
  return 1;
}
