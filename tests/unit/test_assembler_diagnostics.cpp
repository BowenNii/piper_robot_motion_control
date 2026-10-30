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
  // 日志从2A3开始：丢弃半轮，后续所有完整轮都必须在同一周期内。
  hal::PiperFeedbackAssembler ordered(.05, true);
  for (unsigned i=1;i<6;++i)
    require(!ordered.push(frame(0x2A2+i,5+i*.0001)));
  for (unsigned cycle=0;cycle<3;++cycle)
    for (unsigned i=0;i<6;++i) {
      auto s=ordered.push(frame(0x2A2+i,5.005+cycle*.005+i*.0001));
      require(s.has_value()==(i==5));
      if(s)require(s->span_seconds<.001);
    }
  require(ordered.diagnostics().unaligned_frames==5);
  // 缺2A4时不能把不同周期补齐；下一次2A2恢复同步。
  (void)ordered.push(frame(0x2A2,6));
  (void)ordered.push(frame(0x2A3,6.0001));
  require(!ordered.push(frame(0x2A5,6.0003)));
  require(ordered.diagnostics().cycle_resets==1);
  for(unsigned i=0;i<6;++i) {
    auto s=ordered.push(frame(0x2A2+i,6.005+i*.0001));
    require(s.has_value()==(i==5));
  }
  // 超过50ms的组重启，非起帧不能开始新周期。
  (void)ordered.push(frame(0x2A2,7));
  require(!ordered.push(frame(0x2A3,7.06)));
  require(ordered.pending_frame_count()==0);
  std::cout << "Assembler diagnostics PASS\n";
  return 0;
} catch (const std::exception &e) {
  std::cerr << e.what() << '\n';
  return 1;
}
