# 正式真实记录

每个方案的每个方向单独prepare和execute。例如scheme2_out01、scheme2_return01。程序绝不自动往返。
参考轨迹放在上一级reference_complete，不在本目录伪造真机数据。

每轮保留trajectory.csv、manifest.json、generated、generator.log。
执行子目录execution_*保留raw_can.log、commands.csv、feedback.csv、status.csv、summary.json。
离线analyze生成decoded；MATLAB生成analysis_complete。summary.success=false的数据也必须保留并说明失败原因。
