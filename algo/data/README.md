# 数据目录

存放辨识与验证用的采集数据（不进 git，见 .gitignore）。

命名约定：

```
YYYYMMDD_HHMM_<类型>_<说明>.csv

20261110_1530_gravity_poses.csv      静态重力辨识的姿态-力矩表
20261112_0900_sysid_fourier.bin      Fourier 激励的动态数据
20261115_1400_validate_after.json    补偿后的验证结果
```
