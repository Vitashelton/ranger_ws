#!/usr/bin/env python3
import json
import os
from pathlib import Path
import sys
from datetime import datetime

import plotly.graph_objects as go
import streamlit as st

WEB_UI_DIR = str(Path(__file__).resolve().parent)
if WEB_UI_DIR not in sys.path:
    sys.path.insert(0, WEB_UI_DIR)

from ros_bridge import RosBridge
from ui_state import EMPTY_STATUS, decode, person_label


st.set_page_config(page_title="EGA-OfficeNav", page_icon="🤖", layout="wide")
st.markdown("""
<style>
.stApp {background: #f7f8f4; color: #33413e}
.event {background:#fff; border-left:4px solid #8ba7d6; margin:5px 0; padding:7px 12px; border-radius:8px}
.truth-warning {background:#ffe8e8; color:#a40000; border:2px solid #d82929; padding:10px; border-radius:10px; font-weight:800}
</style>
""", unsafe_allow_html=True)


@st.cache_resource
def bridge_resource():
    return RosBridge()


bridge = bridge_resource()
st.title("EGA-OfficeNav")
st.caption("动态语义拓扑图 · 远程大语言模型规划 · 证据门控 · 安全闭环导航")

left, center, right = st.columns([1.05, 2.15, 1.15], gap="large")

PRESETS = {
    "去904": "去904", "去906": "去906", "去908": "去908",
    "找张老师": "找张老师",
    "找张老师并提醒十点去906开会": "找到张老师，提醒他十点去906开会",
    "找李同学，未找到则巡检908后重试": "找到李同学提醒交报告；如果第一次没找到，巡检908后再找一次。",
    "立即执行一次办公室巡检": "立即执行一次办公室巡检",
    "执行8点每日巡检仿真": "每天8点巡检 lobby、904门口、906门口，发现访客就记录并通知张老师，结束后生成每日简报。",
}

with left:
    st.subheader("任务控制")
    provider = st.selectbox("Provider", ["DeepSeek V4 Flash", "Offline"])
    api_ready = bool(os.environ.get("DEEPSEEK_API_KEY"))
    st.caption("DeepSeek API：" + ("已配置" if api_ready else "API未配置，将安全回退 Offline"))
    developer_truth = st.toggle("开发者真值模式", value=False,
                                help="只改变可视化，绝不改变任务规划。")
    bridge.set_developer_truth(developer_truth)
    if developer_truth:
        st.markdown('<div class="truth-warning">SIMULATION GROUND TRUTH — NOT AVAILABLE TO PLANNER</div>',
                    unsafe_allow_html=True)
    mission = st.text_area("自然语言任务", key="mission_text", placeholder="例如：找张老师")
    if st.button("发送任务", type="primary", width="stretch"):
        if mission.strip():
            bridge.send_mission(mission.strip(), provider)
            st.toast("MissionSpec 已发送")
        else:
            st.warning("请输入任务")
    stop_col, reset_col = st.columns(2)
    if stop_col.button("停止任务", width="stretch"):
        ok, message = bridge.stop()
        (st.toast if ok else st.error)(message)
    if reset_col.button("重置场景", width="stretch"):
        ok, message = bridge.reset()
        (st.toast if ok else st.error)(message)
    if st.button("注入幻觉测试（room_999）", width="stretch"):
        injected_task = mission.strip() or "找到张老师，提醒他十点去906开会，然后巡检908；如果办公室没找到，就根据当前环境继续搜索。"
        bridge.inject_hallucination(injected_task, provider)
        st.toast("已注入 room_999；Gate 将拒绝并回退 Offline")
    st.markdown("##### 预设任务")
    for label in PRESETS:
        if st.button(label, key=f"preset_{label}", width="stretch"):
            bridge.send_mission(PRESETS[label], provider)


OBSERVATION_MARKERS = {
    "lobby": (1.2, 2.3), "room_904": (8.0, 4.15),
    "room_906": (13.35, 4.15), "room_908": (18.55, 4.15),
    "corridor_junction": (11.0, 3.75),
}


def office_figure(status, graph, truth_payload, show_truth):
    fig = go.Figure()
    positions = {"lobby": (0, 1), "junction": (2, 1), "room_904": (4, 2),
                 "room_906": (4, 1), "room_908": (4, 0)}
    for edge in graph.get("edges", []):
        a, b = positions.get(edge.get("from")), positions.get(edge.get("to"))
        if not a or not b:
            continue
        blocked = edge.get("state") == "BLOCKED"
        fig.add_trace(go.Scatter(x=[a[0], b[0]], y=[a[1], b[1]], mode="lines",
            line=dict(color="#d53a3a" if blocked else "#6aa77b", width=5,
                      dash="dash" if blocked else "solid"),
            hovertext=f"{edge.get('state')} · cost={edge.get('cost')}",
            hoverinfo="text", showlegend=False))
    beliefs = next((item.get("candidates", {}) for item in graph.get("entity_beliefs", [])
                    if item.get("entity_id") == "teacher_zhang"), {})
    robot_region = (graph.get("robot") or {}).get("current_region", "lobby")
    active = status.get("target_region")
    for node in graph.get("nodes", []):
        node_id = node.get("id")
        if node_id not in positions:
            continue
        x, y = positions[node_id]
        searched = node.get("searched", False)
        color = "#2f78c4" if node_id == robot_region else "#93c89f" if searched else "#e4e9e6"
        symbol = "diamond" if node_id == active else "circle"
        label = f"{node_id}<br>belief {beliefs.get(node_id, 0):.0%}" + ("<br>SEARCHED" if searched else "")
        fig.add_trace(go.Scatter(x=[x], y=[y], mode="markers+text", text=[label],
            textposition="top center", marker=dict(size=38, color=color, symbol=symbol,
            line=dict(color="#1d4f7a" if node_id == active else "#66746e", width=3)),
            name=node_id, hovertext=f"reachable={node.get('reachable')}"))
    if robot_region in positions:
        x, y = positions[robot_region]
        fig.add_annotation(x=x, y=y-0.22, text="🤖 Ranger", showarrow=False)
    if show_truth:
        colors = {"teacher_zhang": "#8d63ad", "student_li": "#4e9a6b", "visitor": "#df8b45"}
        for npc in truth_payload.get("npcs", []):
            region = npc.get("schedule_region")
            if region not in positions:
                continue
            x, y = positions[region]
            fig.add_trace(go.Scatter(x=[x], y=[y-0.35], mode="markers+text",
                                     text=[f"{npc.get('display_name')} · TRUTH"], textposition="bottom center",
                                     marker=dict(size=17, color=colors.get(npc.get("identity_id"), "#999")),
                                     name=f"TRUTH {npc.get('display_name')}"))
    fig.update_xaxes(range=[-0.6, 4.8], visible=False)
    fig.update_yaxes(range=[-0.7, 2.7], visible=False)
    fig.update_layout(height=475, margin=dict(l=5, r=5, t=25, b=5),
                      paper_bgcolor="#f7f8f4", plot_bgcolor="#f7f8f4",
                      legend=dict(orientation="h", y=-0.05), hovermode="closest")
    return fig


@st.fragment(run_every=0.75)
def live_panel(show_truth):
    snapshot = bridge.snapshot()
    status = decode(snapshot["mission_status"], EMPTY_STATUS)
    npcs = decode(snapshot["npc_states"], {"npcs": []})
    truth = decode(snapshot["developer_truth"], {"npcs": []}) if show_truth else {"npcs": []}
    report = decode(snapshot["patrol_report"], {}) or status.get("patrol_report")
    graph = decode(snapshot["topology_graph"], {})
    with center:
        st.subheader("动态语义拓扑图")
        st.caption(f"graph_revision = {graph.get('graph_revision', 0)}")
        st.plotly_chart(office_figure(status, graph, truth, show_truth), width="stretch",
                        config={"displayModeBar": False})
        probabilities = status.get("candidate_probabilities") or {}
        if probabilities:
            st.caption("候选概率：" + " · ".join(
                f"{region} {probability:.0%}" for region, probability in probabilities.items()))
        queue = status.get("search_queue") or []
        if queue:
            st.info("搜索队列：" + " → ".join(queue))
        progress = status.get("search_progress") or {}
        if progress.get("total"):
            st.progress(min(1.0, progress.get("searched", 0) / progress["total"]),
                        text=progress.get("text"))
        st.markdown("##### 人员知识（不含未发现真值）")
        knowledge = status.get("person_knowledge") or {
            item.get("identity_id"): item for item in npcs.get("npcs", [])}
        cols = st.columns(3)
        for column, identity in zip(cols, ("teacher_zhang", "student_li", "visitor")):
            item = knowledge.get(identity, {})
            column.metric(person_label(identity), item.get("knowledge_state", "UNKNOWN"))
        if report:
            st.markdown("##### 结构化巡检报告")
            st.json(report, expanded=False)
    with right:
        st.subheader("当前任务")
        fields = [
            ("原始指令", status.get("input_text") or "—"),
            ("任务类型", status.get("mission_type") or "—"),
            ("目标人物", person_label(status.get("target_person"))),
            ("状态机阶段", status.get("state_machine_phase", "—")),
            ("当前区域", status.get("current_region", "—")),
            ("当前动作", status.get("current_action", "—")),
            ("搜索进度", (status.get("search_progress") or {}).get("text", "—")),
            ("重试", f"{status.get('retry_count',0)}/{status.get('retry_limit',0)}"),
            ("视觉触发", "是" if status.get("visual_triggered") else "否"),
            ("身份确认", "是" if status.get("identity_confirmed") else "否"),
            ("提醒内容", status.get("reminder_text") or "—"),
            ("LLM Provider", status.get("llm_provider", "offline")),
            ("最近错误", status.get("last_error") or "—"),
        ]
        for label, value in fields:
            st.markdown(f"**{label}**  \n{value}")
        safety = decode(snapshot["safety_status"], {})
        metrics = decode(snapshot["safety_metrics"], {})
        st.markdown("##### 当前安全状态")
        st.json({"sensor_gate": safety, "simulation_audit": metrics}, expanded=False)
    st.divider()
    st.subheader("事件时间线")
    events = []
    for raw in snapshot["events"][-20:]:
        try:
            events.append(json.loads(raw))
        except ValueError:
            pass
    if not events:
        st.caption("等待 ROS 事件…")
    for event in reversed(events):
        stamp_value = event.get("timestamp", event.get("stamp", 0))
        stamp = datetime.fromtimestamp(stamp_value).strftime("%H:%M:%S")
        st.markdown(f'<div class="event"><b>{stamp} · {event.get("event_type", "EVENT")}</b><br>{event.get("message", "")}</div>',
                    unsafe_allow_html=True)
    st.divider()
    st.subheader("Phase 2 语义规划调试")
    llm_debug = decode(snapshot["llm_planner_debug"], {})
    gate = decode(snapshot["plan_gate_result"], {})
    request = llm_debug.get("request")
    response = llm_debug.get("response")
    if not llm_debug:
        api_status = "等待任务"
        api_status_detail = "发送“找张老师”等搜索任务后，这里会显示语义规划请求。"
    elif request is None and status.get("mission_type") == "NAVIGATE_TO_REGION":
        api_status = "本任务未调用语义 API"
        api_status_detail = "直接导航到房间只走确定性导航链路，不需要 LLM 规划。"
    elif llm_debug.get("status") == "PENDING":
        api_status = "请求处理中"
        api_status_detail = "正在等待语义规划返回。"
    elif llm_debug.get("error"):
        api_status = "调用失败，已回退 Offline"
        api_status_detail = llm_debug["error"]
    else:
        api_status = "已完成"
        api_status_detail = "已收到语义规划结果。"
    graph_col, api_col, gate_col = st.columns(3)
    with graph_col:
        with st.expander("1. 当前动态拓扑图 JSON", expanded=False):
            st.json(graph or {"status": "等待图快照"})
    with api_col:
        with st.expander("2. API 输入与返回 JSON", expanded=False):
            st.caption(f"状态：{api_status} · {api_status_detail}")
            st.json({
                "api_called": llm_debug.get("api_called", False),
                "api_configured": llm_debug.get("api_configured", False),
                "provider": llm_debug.get("provider_used",
                                          llm_debug.get("provider_requested")),
                "model": llm_debug.get("model") or status.get("llm_model", ""),
                "latency_s": llm_debug.get("latency_s", ""),
                "request": request if request is not None else {},
                "response": response if response is not None else {},
                "fallback_response": llm_debug.get("fallback_response") or {},
                "error": llm_debug.get("error", ""),
            })
    with gate_col:
        with st.expander("3. plan_gate 与 graph_revision", expanded=False):
            st.json(gate or {
                "accepted": False,
                "graph_revision": status.get("graph_revision", 0),
                "rejected_reason": "等待规划",
            })


live_panel(developer_truth)
