# -*- coding: utf-8 -*-
"""
离线验证脚本：读取回传 CSV（Time,CH0..CH13[CH14]），统计每份文件的运动状态占比。
用法：
    python offline_validate.py <CSV文件或目录> [<CSV2> ...]
示例：
    python offline_validate.py "E:/缓存/EricTool_v3.0.3/EricTool_v3.0.3/EricTool_v3.0.3"

通道定义（与固件一致）：
    CH0~CH5 左右髋角/角速度/力矩，CH6 pitch，CH7 roll，CH8 故障码，CH9 电量，
    CH10 状态编码(0INIT 1STATIC 2WALKING 3UPHILL 4DOWNHILL)，CH11 步态评分，
    CH12 俯仰坡度，CH13 高度变化率，CH14 原始高度。
"""
import csv
import glob
import os
import sys

STATE = {0: "INIT", 1: "STATIC", 2: "WALKING",
         3: "UPHILL", 4: "DOWNHILL", 5: "UNKNOWN"}


def analyze(path):
    rows = []
    for enc in ("utf-8-sig", "gb18030"):
        try:
            with open(path, newline="", encoding=enc) as fh:
                reader = csv.DictReader(fh)
                keys = reader.fieldnames or []
                rows = [r for r in reader]
            break
        except UnicodeDecodeError:
            continue
    if not rows:
        print("  [跳过] 无法读取:", os.path.basename(path))
        return
    if not all(("CH%d" % k) in keys for k in range(14)):
        print("  [跳过] 无 CH10 状态列:", os.path.basename(path))
        return
    n = len(rows)
    # 跳过开头 VOFA+ 占位全零行
    i0 = 0
    while i0 < n:
        vals = [rows[i0].get("CH%d" % k, "0") for k in range(10)]
        if any(abs(float(v)) > 1e-9 for v in vals):
            break
        i0 += 1
    d = rows[i0:]
    m = len(d)
    if m == 0:
        print("  [空] 无有效数据:", os.path.basename(path))
        return
    cnt = {}
    other = 0
    for r in d:
        s = int(round(float(r.get("CH10", 0))))
        if 0 <= s <= 5:
            cnt[s] = cnt.get(s, 0) + 1
        else:
            other += 1
    parts = ["%s %5.1f%%" % (STATE.get(s, "S%d" % s), 100.0 * v / m)
             for s, v in sorted(cnt.items())]
    if other:
        parts.append("OTHER %5.1f%%" % (100.0 * other / m))
    print("  %-16s 有效%5d/%d点(%.1fs)  " % (os.path.basename(path), m, n,
                                              0.02 * m) + "  ".join(parts))


def main():
    args = sys.argv[1:]
    if not args:
        print(__doc__)
        return
    files = []
    for a in args:
        if os.path.isdir(a):
            files += sorted(glob.glob(os.path.join(a, "*.csv")))
        else:
            files.append(a)
    for f in files:
        analyze(f)


if __name__ == "__main__":
    main()
