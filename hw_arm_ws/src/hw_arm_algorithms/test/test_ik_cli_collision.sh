#!/usr/bin/env bash
# ik_cli 的碰撞检查集成测试（审查报告 C01）
#
# 只运行算法 CLI，不连任何机器人 action。
# 验证三件事：
#   ① 正常目标：ok=true，且输出里有 collision_checked / 碰撞对数量 / 模型指纹
#   ② 已知穿桌面的目标：必须被拒，原因含"碰撞/穿桌面"
#   ③ 关掉桌面检查后同一目标通过 —— 证明是桌面拒的，不是别的过滤器
set -u

CLI="${1:?需要 ik_cli 路径}"
URDF="${2:?需要 URDF 路径}"
VMAX="10.47,27.23,5.86,20.94,20.94,20.94"
AMAX="84.8,4.0,80.8,785,852,1700"
fail=0

check() { # 名称 期望子串 输出
  if grep -qF -- "$2" <<< "$3"; then
    echo "  ✅ $1"
  else
    echo "  ❌ $1（输出里没有 '$2'）"
    echo "$3" | sed 's/^/      /'
    fail=1
  fi
}

# ⚠️ 2026-11-26：夹爪进 URDF 后，tool0 之外还多伸出 119.4 mm，
#    原来那个目标（TCP z=+0.104）会让**夹爪**穿到桌面以下 32.6 mm。
#    现在用的目标是从"关节限位内随机采样、筛掉夹爪最低点 < 80 mm"的位形里取的，
#    路径最低点 2.4 mm，自碰撞距离 3.5 mm。
echo "── ① 正常目标（夹爪最低点离桌面还有余量）──"
OUT=$("$CLI" --urdf "$URDF" \
  --seed "0 0.8 0.4 0 0.6 0" \
  --target 0.269901 0.190108 0.475476 -0.234811 0.912635 0.182157 0.280679 \
  --max-jump 3.0 --max-norm 6 --vmax "$VMAX" --amax "$AMAX" 2>&1)
check "ok=true"                 "ok: true"            "$OUT"
check "collision_checked=true"  "collision_checked: true" "$OUT"
check "碰撞对数 = 30"           "collision_pairs: 30" "$OUT"
check "有模型指纹"              "hulls=10"            "$OUT"

# ② 的目标 TCP 在 z=−0.127（桌面以下），必须被拒
echo "── ② 穿桌面目标（TCP z=−0.127）必须被拒 ──"
TGT="0.171803 0.098105 -0.127249 0.810132 -0.580701 0.019931 -0.077940"
OUT=$("$CLI" --urdf "$URDF" --seed "0 2.5 0.4 0 0.6 0" --target "$TGT" \
  --max-jump 5 --max-norm 10 --vmax "$VMAX" --amax "$AMAX" 2>&1)
check "ok=false"                "ok: false"           "$OUT"
check "原因是碰撞/穿桌面"       "碰撞/穿桌面"          "$OUT"

echo "── ③ 关掉桌面检查，同一目标应通过 ──"
OUT=$("$CLI" --urdf "$URDF" --seed "0 2.5 0.4 0 0.6 0" --target "$TGT" \
  --max-jump 5 --max-norm 10 --no-table --vmax "$VMAX" --amax "$AMAX" 2>&1)
check "ok=true（证明是桌面拒的）" "ok: true"           "$OUT"

echo "── ④ 碰撞模型建不起来时必须退出，不能静默降级 ──"
OUT=$(timeout 30 "$CLI" --urdf /nonexistent.urdf \
  --seed "0 0.8 0.4 0 0.6 0" --target "0 0 0 0 0 0 1" \
  --vmax "$VMAX" --amax "$AMAX" 2>&1)
RC=$?
if [ $RC -ne 0 ]; then
  echo "  ✅ 非零退出（rc=$RC）"
else
  echo "  ❌ 竟然返回 0"
  fail=1
fi

[ $fail -eq 0 ] && echo "全部通过" || echo "有失败项"
exit $fail
