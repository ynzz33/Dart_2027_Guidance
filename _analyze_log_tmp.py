# -*- coding: utf-8 -*-
# 临时分析脚本：log_16.txt  LADRC roll 飞行日志
# 后12列实际含义(按 CallBack_Task.c val[0..11]):
#   z1 z2 z3 out_p out_r out_y cur_p cur_r cur_y tgt_p tgt_r tgt_y
import statistics as st

rows = []
with open('log_16.txt', encoding='utf-8', errors='ignore') as f:
    for ln, line in enumerate(f, 1):
        line = line.strip()
        if not line or line.startswith('#'):
            continue
        p = [x.strip() for x in line.split(',')]
        if len(p) < 19:
            continue
        try:
            v = [float(x) for x in p[7:19]]
        except ValueError:
            continue
        rows.append((ln, v))

NAMES = ['z1','z2','z3','out_p','out_r','out_y','cur_p','cur_r','cur_y','tgt_p','tgt_r','tgt_y']
def d(v): return dict(zip(NAMES, v))
R = [(ln, d(v)) for ln, v in rows]
print(f'解析数据行: {len(R)}  (源行号 {R[0][0]}..{R[-1][0]})')

# ---- 1) 去掉开头:z1==0 且 out_r==0 (LADRC 未启动) ----
start_i = 0
for i,(ln,r) in enumerate(R):
    if r['z1'] != 0.0 or r['out_r'] != 0.0:
        start_i = i; break
print(f'\n[起点] LADRC 启动于源行 {R[start_i][0]} (z1 首次非0)')

# ---- 2) 去掉结尾:连续完全重复的冻结段 ----
end_i = len(R)-1
last = R[-1][1]
froze_i = len(R)
for i in range(len(R)-1, start_i, -1):
    if R[i][1] == last:
        froze_i = i
    else:
        break
print(f'[结尾] 数据冻结(撞地后)始于源行 {R[froze_i][0]}, 冻结 {len(R)-froze_i} 行 → 丢弃')
end_i = froze_i - 1

# ---- 3) 失控检测: |cur_r|>90 第一次 ----
los = None
for i in range(start_i, end_i+1):
    if abs(R[i][1]['cur_r']) > 90:
        los = i; break
if los: print(f'[失控] |roll|>90° 首现于源行 {R[los][0]} (此后翻滚)')

# ---- 4) 发射点检测: cur_p 离开静态平台并持续下降 ----
# 静态平台 = 启动后一段 cur_p 的中位数
plat_seg = [R[i][1]['cur_p'] for i in range(start_i, min(start_i+80, end_i))]
plat = st.median(plat_seg)
launch = None
for i in range(start_i, end_i+1):
    if R[i][1]['cur_p'] < plat - 1.5:   # 俯冲超过1.5°
        launch = i; break
print(f'[发射] 静态 pitch≈{plat:.1f}°, cur_p 开始俯冲于源行 {R[launch][0] if launch else "?"}')

# ---- 分段统计 ----
def seg_stats(name, i0, i1):
    if i1 < i0:
        print(f'\n=== {name}: 空 ==='); return
    rr = [R[i][1] for i in range(i0, i1+1)]
    err = [x['tgt_r']-x['cur_r'] for x in rr]
    outr = [x['out_r'] for x in rr]
    z2 = [x['z2'] for x in rr]; z3 = [x['z3'] for x in rr]
    sat = sum(1 for o in outr if abs(o) >= 14.9)
    # 满幅符号翻转(相邻拍异号且至少一个接近饱和)
    flip = 0
    for a,b in zip(outr, outr[1:]):
        if a*b < 0 and (abs(a)>=10 or abs(b)>=10):
            flip += 1
    print(f'\n=== {name}  源行 {R[i0][0]}..{R[i1][0]}  ({len(rr)}拍) ===')
    print(f'  roll 实际 cur_r : [{min(x["cur_r"] for x in rr):7.2f}, {max(x["cur_r"] for x in rr):7.2f}]  均值{st.mean(x["cur_r"] for x in rr):6.2f}')
    print(f'  roll 目标 tgt_r : [{min(x["tgt_r"] for x in rr):7.2f}, {max(x["tgt_r"] for x in rr):7.2f}]  均值{st.mean(x["tgt_r"] for x in rr):6.2f}')
    print(f'  roll 误差        : 均值{st.mean(err):6.2f}  |max|{max(abs(e) for e in err):6.2f}  std{(st.pstdev(err)):5.2f}')
    print(f'  out_r 输出       : [{min(outr):7.2f}, {max(outr):7.2f}]  饱和(|.|>=14.9){sat}/{len(rr)}={100*sat/len(rr):.0f}%  满幅翻转{flip}次')
    print(f'  z2(角速度估计)   : [{min(z2):8.1f}, {max(z2):8.1f}]  std{st.pstdev(z2):7.1f}')
    print(f'  z3(扰动估计)     : [{min(z3):8.1f}, {max(z3):8.1f}]  均值{st.mean(z3):8.1f}  std{st.pstdev(z3):7.1f}')

# 有效段 = start..end；以 launch 切静态/飞行
if launch and launch > start_i:
    seg_stats('静态自稳段(发射前)', start_i, launch-1)
    seg_stats('飞行段(发射后→撞地前)', launch, end_i)
else:
    seg_stats('有效段', start_i, end_i)
