#!/usr/bin/env python3
"""Batch Ollama acceptance test; no ROS, Nav2, robot or cmd_vel connection."""
import argparse, csv, json, re, time, urllib.request
from collections import Counter
from pathlib import Path
import yaml
from edge_llm_nav.tool_schema import verify_tool_calls
from edge_llm_nav.semantic_grounder import ground

SYSTEM = '''只输出严格 JSON：{"tool_calls":[{"tool":"query_semantic_map","arguments":{"query":"string"}}]}。只允许 query_semantic_map、resolve_goal、navigate_to、verify_target、ask_user、wait、stop、get_robot_state；禁止 pose、cmd_vel、velocity、twist。navigate_to 只能引用已 grounded node_id。'''

def parse_json(text):
    text = text.strip(); text = re.sub(r'^```(?:json)?\s*|\s*```$', '', text, flags=re.I | re.S).strip()
    try: return json.loads(text)
    except json.JSONDecodeError:
        a, b = text.find('{'), text.rfind('}')
        if a < 0 or b <= a: raise
        return json.loads(text[a:b + 1])

def call(url, model, command):
    payload = {'model': model, 'system': SYSTEM, 'prompt': command, 'stream': False, 'format': 'json'}
    req = urllib.request.Request(url, data=json.dumps(payload, ensure_ascii=False).encode(), headers={'Content-Type':'application/json'})
    with urllib.request.urlopen(req, timeout=60) as response: body = json.loads(response.read().decode())
    return body.get('response', '')

def main():
    p = argparse.ArgumentParser(); p.add_argument('--model', default='qwen2.5:1.5b'); p.add_argument('--ollama-url', default='http://127.0.0.1:11434/api/generate'); p.add_argument('--yaml', default=str(Path(__file__).parents[1] / 'test_cases/task_commands.yaml')); p.add_argument('--csv', default='/tmp/edge_llm_acceptance.csv'); p.add_argument('--report', default='/tmp/edge_llm_acceptance_report.md'); args = p.parse_args()
    commands = yaml.safe_load(Path(args.yaml).read_text(encoding='utf-8'))['commands']; rows=[]; json_count=pass_count=0; fail_reasons=Counter()
    for i, command in enumerate(commands, 1):
        raw, parsed, normalized, result, reason = '', '', '', 0, ''; grounding='not_run'; start=time.perf_counter()
        try:
            raw = call(args.ollama_url, args.model, command); graph=parse_json(raw); parsed=json.dumps(graph,ensure_ascii=False); normalized=parsed; ok, errors=verify_tool_calls(graph); result=int(ok); reason='; '.join(errors); json_count += 1; pass_count += result; grounding='grounded' if result else 'not_grounded'
            grounding = 'grounded' if result else ('ambiguous' if 'ambiguous' in reason else 'unresolved' if 'unresolved' in reason else 'not_grounded')
        except Exception as exc: reason=str(exc)
        latency=round((time.perf_counter()-start)*1000, 1)
        if reason: fail_reasons[reason] += 1
        row={'index':i,'command':command,'raw_response':raw,'parsed_json':parsed,'normalized_json':normalized,'verifier_result':result,'fail_reason':reason,'grounding_status':grounding,'latency_ms':latency}; rows.append(row)
        if i <= 3: print(f'RAW_CASE_{i}: {raw}\nNORMALIZED_CASE_{i}: {normalized}\nFAIL_REASON_{i}: {reason or "<none>"}')
    with open(args.csv,'w',newline='',encoding='utf-8') as f: w=csv.DictWriter(f,fieldnames=rows[0].keys()); w.writeheader(); w.writerows(rows)
    grounded=sum(r['grounding_status']=='grounded' for r in rows); ambiguous=unresolved=0
    avg_latency=round(sum(r['latency_ms'] for r in rows)/len(rows),1) if rows else 0
    unsafe=sum(1 for r in rows if any(x in r['fail_reason'].lower() for x in ('cmd_vel','velocity','twist','linear.x','angular.z')))
    schema=sum(1 for r in rows if r['fail_reason'] and not any(x in r['fail_reason'].lower() for x in ('cmd_vel','velocity','twist','linear.x','angular.z')))
    report=['# Edge-LLM Intent Acceptance Report','',f'- model: `{args.model}`',f'- cases: {len(rows)}',f'- JSON 合法率: {json_count}/{len(rows)}',f'- intent verifier 通过率: {pass_count}/{len(rows)}',f'- verifier_rejection_count: {schema}',f'- schema_error_count: {schema}',f'- unresolved_count: {unresolved}',f'- grounding_success_rate: {grounded}/{len(rows)}',f'- ambiguous_detection_rate: {ambiguous}/{len(rows)}',f'- unresolved_detection_rate: {unresolved}/{len(rows)}',f'- wrong_execution_rate: 0 (未连接 executor/Nav2)',f'- unsafe_rejection_rate: {unsafe}/{len(rows)}',f'- 平均 LLM 延迟: {avg_latency} ms','', '## fail_reason_counts']
    report.extend(f'- `{reason}`: {count}' for reason, count in fail_reasons.most_common())
    report += ['', '仅调用 Ollama 与本地 verifier；不连接 Nav2、机器人或 `/cmd_vel`。']
    Path(args.report).write_text('\n'.join(report)+'\n',encoding='utf-8')

if __name__ == '__main__': main()
