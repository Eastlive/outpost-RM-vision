# 弹道延迟3s可以发射虚拟子弹，延迟5s无法发射
`5s` 后预测的装甲板不会经过 `yaw` 为 `0` 的点
同时计算三块装甲板即可
 
# permit_yaw_angle错误
计算角度 $atan(1.35 / 2.0 / outpost_radius)$
其中1.35单位是厘米，outpost_radius单位是米
还有 `atan` 算出来的是角度，还要转换成弧度

# 面临的问题
帧率太低，允许发射的角度又太小，导致部分弹丸无法发射
在模拟器里大概只能发射70%的弹丸

# 优化方向
进行低通滤波

# 乱调参数的时候发现的问题
目前的阻力系数
    const double k_d = 0.5 * 0.22 * 0.0445 * 6.0 / M_PI / pow(0.042, 3) * M_PI * pow(0.042, 2) / 4.0;
大约是0.1748
如果把阻力系数调成0.1748 * 0.1 = 0.01748
结果就会接近修改前的弹道模型
如果把阻力系数调成0.1748 * 10 = 1.748
那么弹道就打不到装甲板了
这时is_fire_会一直是false

# 弹丸会打到装甲板偏后的位置是因为没有加fire_latency

# 装甲板permit计算错误
计算角度 $atan(1.35 / 2.0 / outpost_radius * 100) / 180 * M_PI$
其中1.35单位是厘米，outpost_radius单位是米
应该是 $atan(1.35 / 2.0 / outpost_radius / 100)$
算出来的结果是弧度
atan结果是弧度

# 想想就不对劲，谁家装甲板1.35厘米

# 2023.12.3
Debug
# 调试发现，tracker丢失装甲板信息
# 进一步调试发现，tracker丢失装甲板信息是因为进入了Armor Jump模式
# 进一步调试发现，Armor Jump模式是因为tracker的yaw角度预测量会超出pi
- 尝试调整方案
    - 1. 限制yaw角度预测量在-pi到pi之间
    - 2. 在卡尔曼滤波方程中将yaw角度限制在-pi到pi之间
    - 3. 调小temp_lost阈值，因为tracker进入Armor Jump模式的原因是temp_lost时间过长
    - 4. 修改完成Armor Jump之后的逻辑，因为Armor Jump似乎只改变了state值，但是没有改变预测值
- 哈哈哈我调通啦！！！
- 我调了啥？
- 我把max_match_distance_的值调小了
    - 之前的值是0.5，我调成了0.1
    - 因为Armor Jump模式中有一个判断条件是
        - if (armor->distance > max_match_distance_) {
            // 更新target_state
        - }
    因此，只要max_match_distance_设置合理，真正的装甲板跳变就会被修改