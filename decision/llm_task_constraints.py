"""LLM 任务约束指令解析。

该模块只负责把自然语言指挥口令裁剪成设备2内部约束字典，不直接修改环境状态。
"""
import re
from typing import Any, Dict, Optional


_DIGITS = {
    "零": 0,
    "一": 1,
    "二": 2,
    "两": 2,
    "三": 3,
    "四": 4,
    "五": 5,
    "六": 6,
    "七": 7,
    "八": 8,
    "九": 9,
}


def parse_llm_number(token: Any) -> Optional[int]:
    """解析阿拉伯数字或常见中文数字。"""
    token = str(token or "").strip()
    if not token:
        return None
    if token.isdigit():
        return int(token)
    if token == "十":
        return 10
    if "十" in token:
        left, right = token.split("十", 1)
        tens = _DIGITS.get(left, 1 if left == "" else None)
        ones = _DIGITS.get(right, 0 if right == "" else None)
        if tens is not None and ones is not None:
            return tens * 10 + ones
    if len(token) == 1 and token in _DIGITS:
        return _DIGITS[token]
    return None


def parse_llm_task_constraints_command(text: Any, max_interceptors: int) -> Optional[Dict[str, Any]]:
    """把 LLM/指挥员文本解析成任务约束字典。"""
    text = str(text or "").strip()
    lower = text.lower()
    parsed: Dict[str, Any] = {}
    max_interceptors = max(0, int(max_interceptors or 0))

    if any(word in text for word in ("恢复默认约束", "清除任务约束", "取消LLM约束", "取消llm约束", "默认分配")):
        parsed["clear_all"] = True
        return parsed
    if any(word in text for word in ("取消区域优先", "全域优先", "不区分左右", "取消左翼优先", "取消右翼优先", "取消中路优先")):
        parsed["preferred_sector"] = None
    if any(word in text for word in ("取消出动上限", "不限出动", "取消兵力上限", "取消数量限制")):
        parsed["max_active_count"] = 0
    if any(word in text for word in ("取消高速优先", "取消突防优先", "恢复默认优先级")):
        parsed["target_priority"] = None
    if any(
        word in text
        for word in (
            "重新分配任务", "任务重新分配", "任务重分配", "重新分配", "重分配",
            "重新调度", "重调度", "任务重排", "重新规划任务", "重新部署任务",
            "对任务进行重新分配",
        )
    ):
        parsed["force_reassign"] = True
    if any(word in text for word in ("取消保留", "解除保留", "不保留", "清空保留")):
        parsed["reserve_count"] = 0

    reserve_match = re.search(
        r"(?:至少)?(?:保留|预留|留出|留下|留)\s*([0-9一二两三四五六七八九十]+)\s*(?:架|个)?(?:无人机|拦截机|飞机|机)?",
        text,
    )
    if reserve_match:
        count = parse_llm_number(reserve_match.group(1))
        if count is not None:
            parsed["reserve_count"] = max(0, min(max_interceptors, count))

    max_active_match = re.search(
        r"(?:最多|至多|上限|限制|只允许|只可|最多只)?(?:出动|派出|发射)\s*([0-9一二两三四五六七八九十]+)\s*(?:架|个)?(?:无人机|拦截机|飞机|机)?",
        text,
    )
    if max_active_match:
        count = parse_llm_number(max_active_match.group(1))
        if count is not None:
            parsed["max_active_count"] = max(0, min(max_interceptors, count))

    if (
        ("高速" in text or "速度快" in text or "速度最快" in text or "fast" in lower)
        and any(word in text for word in ("优先", "先拦", "先打", "先处理", "重点"))
    ):
        parsed["target_priority"] = "speed"
    if any(word in text for word in ("左翼", "左侧", "左边")) and any(
        word in text for word in ("优先", "先处理", "先拦", "先打", "重点")
    ):
        parsed["preferred_sector"] = "left"
    elif any(word in text for word in ("右翼", "右侧", "右边")) and any(
        word in text for word in ("优先", "先处理", "先拦", "先打", "重点")
    ):
        parsed["preferred_sector"] = "right"
    elif any(word in text for word in ("中路", "中间", "中央")) and any(
        word in text for word in ("优先", "先处理", "先拦", "先打", "重点")
    ):
        parsed["preferred_sector"] = "center"

    if any(word in text for word in ("突防线", "接近突防", "靠近突防", "临近突防")) and any(
        word in text for word in ("优先", "先处理", "先拦", "先打", "重点", "附近")
    ):
        parsed["target_priority"] = "breach"
    if "干扰" in text and any(
        word in text
        for word in ("避开", "绕开", "绕避", "不要进", "不要进入", "不要继续派机硬闯", "不要硬闯", "禁入", "避让")
    ):
        parsed["avoid_jam"] = True
    if "取消绕避干扰" in text or "取消干扰禁入" in text:
        parsed["avoid_jam"] = False

    return parsed if parsed else None


def _self_check() -> None:
    assert parse_llm_number("十") == 10
    assert parse_llm_number("二十三") == 23
    assert parse_llm_task_constraints_command("保留两架无人机，左翼优先，重新分配", 6) == {
        "force_reassign": True,
        "reserve_count": 2,
        "preferred_sector": "left",
    }
    assert parse_llm_task_constraints_command("最多出动十架", 6)["max_active_count"] == 6
    assert parse_llm_task_constraints_command("取消绕避干扰", 6)["avoid_jam"] is False
    print("llm_task_constraints自检通过")


if __name__ == "__main__":
    _self_check()
