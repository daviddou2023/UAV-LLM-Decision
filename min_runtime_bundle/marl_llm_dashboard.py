from pathlib import Path
import importlib.util


_ROOT_DASHBOARD = Path(__file__).resolve().parents[1] / "marl_llm_dashboard.py"
_SPEC = importlib.util.spec_from_file_location("_root_marl_llm_dashboard", _ROOT_DASHBOARD)
_MOD = importlib.util.module_from_spec(_SPEC)
_SPEC.loader.exec_module(_MOD)

LLMDashboardServer = _MOD.LLMDashboardServer
build_dashboard_snapshot = _MOD.build_dashboard_snapshot
