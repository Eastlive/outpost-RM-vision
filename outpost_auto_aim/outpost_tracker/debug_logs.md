# 弹道延迟3s可以发射虚拟子弹，延迟5s无法发射
`5s` 后预测的装甲板不会经过 `yaw` 为 `0` 的点
同时计算三块装甲板即可
 
# permit_yaw_angle错误
计算角度 $atan(1.35 / 2.0 / outpost_radius)$
其中1.35单位是厘米，outpost_radius单位是米
还有 `atan` 算出来的是角度，还要转换成弧度